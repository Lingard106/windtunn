"""
OpenMV 飞镖制导视觉示例（新测量协议 v2 / 24 字节）
================================================
沿用 guide_v5.py 的中文分段、动态 ROI、三帧确认、五帧丢失和主循环风格。

本文件是供移植的参考代码，不覆盖原板上程序，也不是已完成实板验收的固件。
与原 v5 的主要差别：
1. 未确认目标时失捕，下一帧恢复全图搜索（对应本次仿真最小视觉修复）。
2. 每张图像都发一包；发送原始像素坐标和状态，不发送红十字误差。
3. 实测、锁定、裁切、盲推分别标记；盲推不冒充新实测。
4. 帧序号、采图时序占位时间、曝光设置和 CRC 随包发送。

须与配套 STM32 示例的 v2 解析器一起切换，不能直接接旧 7/10 字节解析器。
此脚本启动后立即测量；READY / 发射通知 / 配置应答要另行接入同版状态机。
没有把旧 RPDART / 7B 握手混进新测量字节流。

时间限制：snapshot() 前后软件时间的中点，只是待标定的采样时刻占位值，
不是已知曝光中点；相机与 STM32 时钟也未同步，须标定曝光/读出延迟并对时。

主机可 import 本文件测试纯函数；硬件模块仅在 main() 中导入和初始化。
"""

# ============================================================
# 1. 可调参数（沿用 v5；阈值、曝光和读出窗口必须现场重标定）
# ============================================================
FRAME_W = 320
FRAME_H = 240
READOUT_ROI = (0, 700, 640, 480)

# 原 v5 的实物 LAB 阈值，不是仿真识别阈值；换灯、镜头和曝光后重新标定。
GREEN_THRESHOLD = (31, 48, -49, 10, 21, 47)
PIXELS_THRESHOLD = 30
AREA_THRESHOLD = 30
MIN_ASPECT_RATIO = 0.5
MAX_ASPECT_RATIO = 2.0

FOUND_RANGE_X = 40
FOUND_RANGE_Y = 40
INIT_ROI = (0, 0, FRAME_W, FRAME_H)
CONFIRM_FRAMES = 3
MAX_LOST_FRAMES = 5
VEL_ALPHA = 0.7

# False：保留 v5 的速度更新，仅加入未确认失捕回全图的最小修复。
# True：首帧/重捕获时速度清零，只在相邻实测帧之间估计速度；需另做对照验收。
GATE_VELOCITY_ON_CONSECUTIVE_DETECTIONS = False

EXPOSURE_US = 3000
UART_ID = 3
UART_BAUD = 115200
DRAW_DEBUG = True
VERBOSE = False                 # 飞行时逐帧 print 容易拖慢测量。

# 红十字只用于显示；新协议不减 IDEAL_CX / IDEAL_CY。
IMG_CX, IMG_CY = FRAME_W // 2, FRAME_H // 2
IDEAL_CX, IDEAL_CY = FRAME_W // 2 - 30, FRAME_H // 2 + 60
DART_CX, DART_CY = FRAME_W // 2 - 30, FRAME_H // 2
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
ORANGE = (255, 128, 0)
WHITE = (255, 255, 255)
YELLOW = (255, 255, 0)

FLAG_DETECTED = 0x01
FLAG_LOCKED = 0x02
FLAG_CLIPPED = 0x04
FLAG_PREDICTED = 0x08
PROTOCOL_VERSION = 2
PACKET_LENGTH = 24


# ============================================================
# 2. ROI / 色块工具（只依赖 blob.rect，兼容方法或属性形式）
# ============================================================
def build_roi_at(cx, cy, half_w, half_h):
    x = max(0, min(int(cx) - half_w, FRAME_W - 1))
    y = max(0, min(int(cy) - half_h, FRAME_H - 1))
    return (x, y, min(2 * half_w, FRAME_W - x),
            min(2 * half_h, FRAME_H - y))


def blob_rect(blob):
    rect = blob.rect
    if callable(rect):
        rect = rect()
    return (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))


def choose_target_rect(blobs):
    """与 v5 一样先选最大外接框，再检查宽高比；无候选时返回 None。"""
    largest = None
    largest_area = -1
    for blob in blobs:
        rect = blob_rect(blob)
        area = rect[2] * rect[3]
        if area > largest_area:
            largest = rect
            largest_area = area
    if largest is None or largest[3] <= 0:
        return None
    aspect = largest[2] / largest[3]
    if MIN_ASPECT_RATIO <= aspect <= MAX_ASPECT_RATIO:
        return largest
    return None


def rect_center(rect):
    # 像素坐标采用外接框中心；不是 blob.cx()/cy() 的亮度/像素质心。
    # 例如覆盖 x=10..13 的四列，中心为 11.5；协议 q4 保留半像素。
    return (rect[0] + (rect[2] - 1) * 0.5,
            rect[1] + (rect[3] - 1) * 0.5)


def rect_is_clipped(rect):
    # 框碰到任一图像边缘即标裁切，配套 STM32 不用它刷新可信目标。
    x, y, w, h = rect
    return x <= 0 or y <= 0 or x + w >= FRAME_W or y + h >= FRAME_H


# ============================================================
# 3. 多帧确认 + 速度预测（预测只用于搜索，不伪造实测像素）
# ============================================================
def new_tracker_state():
    return {
        "roi": INIT_ROI,
        "confirmed": False,
        "confirm_count": 0,
        "lost_count": 0,
        "prev_cx": FRAME_W // 2,
        "prev_cy": FRAME_H // 2,
        "vx": 0.0,
        "vy": 0.0,
        "previous_detected": False,
    }


def update_tracker(state, rect):
    detected = rect is not None
    if detected:
        state["confirm_count"] += 1
        state["lost_count"] = 0
        if state["confirm_count"] >= CONFIRM_FRAMES:
            state["confirmed"] = True
    else:
        state["confirm_count"] = 0
        if state["confirmed"]:
            state["lost_count"] += 1
            if state["lost_count"] >= MAX_LOST_FRAMES:
                state["confirmed"] = False
                state["roi"] = INIT_ROI

    if detected:
        cx, cy = rect_center(rect)
        if (GATE_VELOCITY_ON_CONSECUTIVE_DETECTIONS
                and not state["previous_detected"]):
            state["vx"] = 0.0
            state["vy"] = 0.0
        else:
            state["vx"] = (VEL_ALPHA * state["vx"]
                           + (1 - VEL_ALPHA) * (cx - state["prev_cx"]))
            state["vy"] = (VEL_ALPHA * state["vy"]
                           + (1 - VEL_ALPHA) * (cy - state["prev_cy"]))
        state["prev_cx"], state["prev_cy"] = cx, cy
        state["roi"] = build_roi_at(
            cx + state["vx"], cy + state["vy"],
            max(FOUND_RANGE_X, rect[2] + 20),
            max(FOUND_RANGE_Y, rect[3] + 20))
    elif state["confirmed"]:
        state["roi"] = build_roi_at(
            state["prev_cx"] + state["vx"],
            state["prev_cy"] + state["vy"],
            int(FOUND_RANGE_X * 1.3), int(FOUND_RANGE_Y * 1.2))
    else:
        # 本次最小修复：尚未确认就失捕，不能困在旧的小 ROI 中。
        state["roi"] = INIT_ROI

    state["previous_detected"] = detected
    flags = FLAG_LOCKED if state["confirmed"] else 0
    if detected:
        flags |= FLAG_DETECTED
        if rect_is_clipped(rect):
            flags |= FLAG_CLIPPED
    elif state["confirmed"]:
        # 只说明当前处于预测搜索；本包像素和框尺寸仍全部为 0。
        flags |= FLAG_PREDICTED
    return flags


# ============================================================
# 4. 新协议：AA55 / version=2 / len=24 / 全部多字节字段小端
# ============================================================
def crc16_ccitt_false(data):
    """poly=1021 init=FFFF RefIn/RefOut=false XorOut=0；123456789 -> 29B1。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def put_u16_le(packet, offset, value):
    packet[offset] = value & 0xFF
    packet[offset + 1] = (value >> 8) & 0xFF


def put_u32_le(packet, offset, value):
    put_u16_le(packet, offset, value & 0xFFFF)
    put_u16_le(packet, offset + 2, (value >> 16) & 0xFFFF)


def make_uart_packet(frame_seq, capture_time_us, rect, flags,
                     exposure_us=EXPOSURE_US):
    """每帧发送，包括未检测帧；detected=0 时绝不发送上一帧/预测坐标。"""
    if flags & ~0x0F:
        raise ValueError("reserved flags must be zero")
    if not 0 <= exposure_us <= 65535:
        raise ValueError("exposure_us must fit uint16")

    detected = bool(flags & FLAG_DETECTED)
    if detected:
        if rect is None or flags & FLAG_PREDICTED:
            raise ValueError("detected needs a real rect, not prediction")
        x, y, w, h = rect
        if (w <= 0 or h <= 0 or x < 0 or y < 0
                or x + w > FRAME_W or y + h > FRAME_H):
            raise ValueError("rect outside QVGA")
        cx, cy = rect_center(rect)
        u_q4, v_q4 = int(cx * 16 + 0.5), int(cy * 16 + 0.5)
        if rect_is_clipped(rect):
            flags |= FLAG_CLIPPED
    else:
        # locked 可以保持短丢失状态；它不能替代 detected。
        u_q4, v_q4, w, h = 0, 0, 0, 0
        flags &= ~FLAG_CLIPPED

    packet = bytearray(PACKET_LENGTH)
    packet[0], packet[1] = 0xAA, 0x55
    packet[2], packet[3] = PROTOCOL_VERSION, PACKET_LENGTH
    put_u16_le(packet, 4, int(frame_seq) & 0xFFFF)
    put_u32_le(packet, 6, int(capture_time_us) & 0xFFFFFFFF)
    put_u16_le(packet, 10, u_q4)
    put_u16_le(packet, 12, v_q4)
    put_u16_le(packet, 14, int(w))
    put_u16_le(packet, 16, int(h))
    packet[18], packet[19] = flags, 0
    put_u16_le(packet, 20, int(exposure_us))
    put_u16_le(packet, 22, crc16_ccitt_false(packet[:22]))
    return packet


# ============================================================
# 5. 绘图（可关闭；红十字不参与新测量协议）
# ============================================================
def draw_cross_v5(img, cx, cy, color, size=8, thickness=2):
    img.draw_line((int(cx) - size, int(cy), int(cx) + size, int(cy)),
                  color=color, thickness=thickness)
    img.draw_line((int(cx), int(cy) - size, int(cx), int(cy) + size),
                  color=color, thickness=thickness)


def draw_debug(img, rect, state, fps):
    if rect is not None:
        cx, cy = rect_center(rect)
        img.draw_rectangle(rect, color=GREEN, thickness=2, fill=False)
        draw_cross_v5(img, cx, cy, GREEN, size=5)
        label = "LOCK" if state["confirmed"] else "CONFIRM"
    else:
        label = "COAST / NO MEAS" if state["confirmed"] else "NO TARGET"
    img.draw_string(2, FRAME_H - 20, label,
                    color=GREEN if rect is not None else YELLOW, scale=1)
    draw_cross_v5(img, IMG_CX, IMG_CY, BLUE, 12, 2)
    draw_cross_v5(img, IDEAL_CX, IDEAL_CY, RED, 12, 2)
    draw_cross_v5(img, DART_CX, DART_CY, ORANGE, 12, 2)
    img.draw_rectangle(state["roi"], color=BLUE, thickness=1, fill=False)
    img.draw_string(FRAME_W - 58, 2, "FPS %.0f" % fps, color=WHITE, scale=1)


# ============================================================
# 6. 初始化与主循环（仅在 OpenMV 上直接运行时执行）
# ============================================================
def main():
    import sensor
    import time
    from pyb import UART

    uart = UART(UART_ID, UART_BAUD)
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    try:
        sensor.ioctl(sensor.IOCTL_SET_READOUT_WINDOW, READOUT_ROI)
    except Exception as error:
        # 不同传感器支持情况不同；回退后必须重做内参/视场标定。
        print("readout ioctl failed (需重标相机内参):", error)
    sensor.set_auto_gain(False, gain_db=20)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
    sensor.skip_frames(time=2000)

    clock = time.clock()
    state = new_tracker_state()
    frame_seq = 0
    last_tick = time.ticks_us()
    local_time_us = 0

    while True:
        clock.tick()
        begin_us = time.ticks_us()
        # 用 ticks_diff 展开 MicroPython 本地 ticks 的回卷，再编码 uint32。
        # 要求循环间隔小于 ticks 周期的一半；这个计数原点是脚本启动后的本地原点。
        local_time_us = (local_time_us + time.ticks_diff(begin_us, last_tick)) & 0xFFFFFFFF
        last_tick = begin_us
        img = sensor.snapshot()
        end_us = time.ticks_us()

        # 仅为采图调用窗口中点的占位值，绝非已标定的曝光中点或 UART 发送时刻。
        capture_time_us = (local_time_us
                           + time.ticks_diff(end_us, begin_us) // 2) & 0xFFFFFFFF
        blobs = img.find_blobs(
            [GREEN_THRESHOLD], roi=state["roi"],
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD, merge=True)
        rect = choose_target_rect(blobs)
        flags = update_tracker(state, rect)

        # 曝光关闭自动调整，字段就是当前固定设置；以后动态调整时要逐帧更新。
        uart.write(make_uart_packet(frame_seq, capture_time_us, rect, flags,
                                    EXPOSURE_US))
        frame_seq = (frame_seq + 1) & 0xFFFF

        # 先检测/发包再绘图，不让 UI 十字污染本帧识别输入。
        if DRAW_DEBUG:
            draw_debug(img, rect, state, clock.fps())
        if VERBOSE:
            print("seq=%d rect=%s flags=%02X FPS=%.0f"
                  % ((frame_seq - 1) & 0xFFFF, rect, flags, clock.fps()))


if __name__ == "__main__":
    main()
