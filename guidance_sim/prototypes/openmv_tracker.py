"""OpenMV guide_v5.py 桌面画面预览；不调用相机或硬件 UART。"""
from __future__ import annotations

import cv2
import numpy as np

try:
    from .vision_settings import VisionSettings
except ImportError:
    from vision_settings import VisionSettings


class OpenMVTracker:
    """按 guide_v5.py 参数近似处理 RGB 图片，并生成可观察的串口数据包。"""
    # 源码使用 OpenMV LAB：L 为 0..100，a/b 以 0 为中心。
    OPENMV_LAB = (31, 48, -49, 10, 21, 47)
    # OpenCV 8 位 LAB：L 为 0..255，a/b 以 128 为中心。
    CV_LAB = (round(31*2.55), round(48*2.55), 79, 138, 149, 175)
    MIN_PIXELS = 30
    ASPECT_RANGE = (0.5, 2.0)
    ROI_HALF = (40, 40)
    CONFIRM_FRAMES = 3
    MAX_LOST = 5
    VEL_ALPHA = 0.7
    IDEAL = (160, 135)
    IMAGE_CENTER = (160, 120)
    DART_REFERENCE = (130, 120)
    EXPOSURE_US = 3000

    def __init__(self, settings=None):
        self.set_settings(VisionSettings() if settings is None else settings)

    @staticmethod
    def _cv_lab_for(lab):
        """将 OpenMV 的 L 0..100、中心化 a/b 转成 OpenCV 8 位 LAB。"""
        l_min, l_max, a_min, a_max, b_min, b_max = lab
        return (round(l_min * 2.55), round(l_max * 2.55),
                a_min + 128, a_max + 128, b_min + 128, b_max + 128)

    def set_settings(self, settings):
        """验证并应用一整套参数；有效更新会清空旧的跟踪状态。"""
        if not isinstance(settings, VisionSettings):
            raise TypeError("settings 必须是 VisionSettings")
        # 完成阈值转换后再改动实例，避免不完整设置破坏当前状态。
        cv_lab = self._cv_lab_for(settings.lab)
        self.settings = settings
        self.cv_lab = cv_lab
        self.reset()

    def prepare_image(self, rgb):
        """按当前曝光参数缩放显示用 RGB；process 不会重复应用曝光。"""
        return self.settings.prepare_image(rgb)

    def _threshold_mask(self, rgb):
        lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB)
        low = np.array([self.cv_lab[0], self.cv_lab[2], self.cv_lab[4]], np.uint8)
        high = np.array([self.cv_lab[1], self.cv_lab[3], self.cv_lab[5]], np.uint8)
        return cv2.inRange(lab, low, high)

    def reset(self):
        """清空上一帧速度/ROI 估计；新一轮开始先检查完整画面。"""
        self.previous = np.array(self.IMAGE_CENTER, dtype=np.float32)
        self.velocity = np.zeros(2, dtype=np.float32)
        self.lost_count = 0
        self.confirm_count = 0
        self.confirmed = False
        self.roi = (0, 0, 320, 240)

    def _packet(self, dx, dy, area, locked):
        """生成 guide_v5.py 所用 AA55 + 有符号误差 + 面积 + 状态 + XOR。"""
        def signed_bytes(value):
            number = int(value) & 0xffff
            return [(number >> 8) & 255, number & 255]
        area = min(int(area), 65535)
        payload = [0xAA, 0x55, *signed_bytes(dx), *signed_bytes(dy),
                   (area >> 8) & 255, area & 255, int(bool(locked))]
        check = 0
        for value in payload[2:]:
            check ^= value
        return bytes(payload + [check])

    def process(self, rgb):
        """识别最大的合格绿色块，返回叠加 RGB 图和状态字典。"""
        image = np.asarray(rgb, dtype=np.uint8)
        if image.shape != (240, 320, 3):
            raise ValueError("识别预览要求相机画面是 320×240 RGB 图像")
        mask = self._threshold_mask(image)
        x, y, w, h = self.roi
        roi_mask = np.zeros_like(mask)
        roi_mask[y:y+h, x:x+w] = mask[y:y+h, x:x+w]
        count, labels, stats, centers = cv2.connectedComponentsWithStats(roi_mask, 8)
        candidates = []
        for index in range(1, count):
            bx, by, bw, bh, pixels = stats[index]
            area = int(bw * bh)
            aspect = bw / max(1, bh)
            if pixels >= self.MIN_PIXELS and area >= self.MIN_PIXELS:
                candidates.append((area, int(pixels), int(bx), int(by), int(bw), int(bh), centers[index]))
        candidates.sort(reverse=True, key=lambda item: item[0])
        # guide_v5.py 先取最大色块，再检查宽高比；最大块不合格时不会改选次大块。
        found = bool(candidates)
        area, pixels, bbox, center = 0, 0, None, None
        if found:
            area, pixels, bx, by, bw, bh, raw_center = candidates[0]
            if self.ASPECT_RANGE[0] <= bw/max(1, bh) <= self.ASPECT_RANGE[1]:
                bbox, center = (bx, by, bw, bh), np.asarray(raw_center, dtype=np.float32)
            else:
                found = False
        if found:
            measured = center - self.previous
            self.velocity = self.VEL_ALPHA*self.velocity + (1-self.VEL_ALPHA)*measured
            self.previous = center.copy()
            self.lost_count = 0
            self.confirm_count += 1
            if self.confirm_count >= self.CONFIRM_FRAMES:
                self.confirmed = True
            self.roi = self._next_roi(center + self.velocity, max(40, bw+20), max(40, bh+20))
        else:
            self.confirm_count = 0
            if self.confirmed:
                self.lost_count += 1
                if self.lost_count >= self.MAX_LOST:
                    self.confirmed = False
                    self.roi = (0, 0, 320, 240)
                else:
                    # 锁定后短暂丢失时按预测点放大搜索区，连续丢满 5 帧才回全图。
                    self.roi = self._next_roi(self.previous + self.velocity, 52, 48)
        locked = self.confirmed
        dx = int(round((center[0] - self.settings.aim_point_px[0]))) if found else 0
        dy = int(round((center[1] - self.settings.aim_point_px[1]))) if found else 0
        packet = self._packet(dx, dy, area, locked)
        status = ("锁定" if found and locked else "确认中" if found else
                  "锁定中（短暂丢失）" if locked else "未检测到")
        details = {"detected": found, "locked": bool(locked), "status": status,
                   "lost_count": self.lost_count, "confirm_count": self.confirm_count,
                   "roi": list(self.roi), "bbox": list(bbox) if bbox else None,
                   "center_px": [float(value) for value in center] if found else None,
                   "error_px": [dx, dy], "area_px": int(area), "pixels_px": int(pixels),
                   "candidate_count": len(candidates),
                   "threshold_pixels": int(cv2.countNonZero(mask)),
                   "reason": ("色块有效" if found else "最大色块宽高比不合格" if candidates
                              else "搜索框内没有达到 30 像素的阈值色块"),
                   "packet_hex": packet.hex(" ").upper() if found else None,
                   "algorithm": "guide_v5.py 参数的桌面 OpenCV 近似；未连接 OV7725/串口",
                   "source_threshold_openmv_lab": list(self.OPENMV_LAB),
                   "lab": list(self.settings.lab),
                   "exposure_us": self.settings.exposure_us,
                   "aim_point_px": list(self.settings.aim_point_px),
                   "dart_reference_px": list(self.settings.dart_reference_px),
                   "image_center_px": list(self.IMAGE_CENTER),
                   "profile": self.settings.profile}
        return self.annotate(image, details), details

    @classmethod
    def annotate(cls, rgb, details):
        """只按已记录结果画标记；回放和重绘不能再次推进识别状态机。"""
        output = np.asarray(rgb, dtype=np.uint8).copy()
        roi = details.get("roi", [0, 0, 320, 240])
        bbox = details.get("bbox")
        center = details.get("center_px")
        found, locked = details.get("detected", False), details.get("locked", False)
        dx, dy = details.get("error_px", [0, 0])
        aim_point = tuple(details.get("aim_point_px", cls.IDEAL))
        dart_reference = tuple(details.get("dart_reference_px", cls.DART_REFERENCE))
        image_center = tuple(details.get("image_center_px", cls.IMAGE_CENTER))
        cv2.rectangle(output, (roi[0], roi[1]),
                      (roi[0]+roi[2]-1, roi[1]+roi[3]-1), (40, 130, 255), 1)
        for point, color, size in ((image_center, (80, 160, 255), 5),
                                   (dart_reference, (255, 170, 40), 4),
                                   (aim_point, (255, 70, 70), 6)):
            cv2.drawMarker(output, point, color, cv2.MARKER_CROSS, size, 1)
        if bbox:
            bx, by, bw, bh = bbox
            cv2.rectangle(output, (bx, by), (bx+bw, by+bh), (40, 255, 90), 2)
            cv2.circle(output, tuple(np.round(center).astype(int)), 3, (40, 255, 90), -1)
            cv2.line(output, aim_point, tuple(np.round(center).astype(int)), (255, 255, 30), 1)
        image_status = ("LOCKED" if found and locked else "TRACKING" if found else
                        "LOCKED-LOST" if locked else "NO TARGET")
        cv2.rectangle(output, (0, 0), (319, 19), (18, 33, 48), -1)
        cv2.putText(output, f"OpenMV approx: {image_status}  err=({dx},{dy}) px", (5, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, .36, (255, 255, 255), 1, cv2.LINE_AA)
        return output

    def mask_image(self, rgb):
        """显示当前设置命中的像素；输入与 process 一样应已完成曝光预处理。"""
        image = np.asarray(rgb, dtype=np.uint8)
        return cv2.cvtColor(self._threshold_mask(image), cv2.COLOR_GRAY2RGB)

    @staticmethod
    def _next_roi(predicted, half_width=40, half_height=40):
        """围绕预测中心构造不越出图像边缘的搜索区域。"""
        cx, cy = (int(round(v)) for v in predicted)
        width, height = half_width*2, half_height*2
        x = max(0, min(cx-half_width, 319))
        y = max(0, min(cy-half_height, 239))
        return (x, y, min(width, 320-x), min(height, 240-y))
