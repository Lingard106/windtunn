// =====================================================================
//  GuidanceTask接入示例：沿用中文分段与servo_cmd接口风格
// =====================================================================
//  可单独编译为对象，但链接到工程前必须实现下面Platform_*适配函数。
//  Platform_ReadControlInput必须来自真实导航/目标观察器/弹道预测器；
//  这里没有假造自身p/v，也没有用旧PNG(N=3)代替预测PN。
//  完整弹道C++内核仍需按MD移植，此文件是接线框架，不是完整飞控固件。

#include "stm32_control_example.h"

// IRQ中只发布队列/双缓冲，不在这里做预测或阻塞打印。
extern void Platform_PublishVisionFromIsr(const VisionFrameV2& frame);
// 取一致的IMU/请求/年龄；缺导航、未离架或输入不可用时返回false。
extern bool Platform_ReadControlInput(ControlInputExample& input);
// 主线程/周期任务写TIM3/4比较值；逐片反向/零位/比例在适配层标定。
extern void Platform_WriteServoPwm(const uint16_t pwm_us[4]);

static VisionPacketParserV2 g_vision_parser;
static PhysicalAutopilotExample g_autopilot;
static uint16_t g_dma_last_pos = 0;

// 1) 循环DMA回调适配
//    对应HAL_UARTEx_RxEventCallback的Size：写指针位置，不是新数据长度。
//    DMA仅在初始化启动一次；本函数不调用ReceiveToIdle重启。
void GuidanceExample_OnCircularDma(const uint8_t* buffer,
                                  uint16_t buffer_size, uint16_t write_pos) {
    if (!buffer || !buffer_size || write_pos > buffer_size || g_dma_last_pos > buffer_size)
        return;
    const auto feed = [buffer](uint16_t start, uint16_t end) {
        VisionFrameV2 frame;
        for (uint16_t i = start; i < end; ++i)
            if (g_vision_parser.push(buffer[i], frame)) Platform_PublishVisionFromIsr(frame);
    };
    if (write_pos > g_dma_last_pos) feed(g_dma_last_pos, write_pos);
    else if (write_pos < g_dma_last_pos) {
        feed(g_dma_last_pos, buffer_size);
        feed(0, write_pos);
    }
    g_dma_last_pos = write_pos;
    // 要及时服务HT/TC/IDLE；若整个缓冲区被覆盖多圈，单靠Size不能恢复丢失字节。
}

// 2) 相机重启/会话重建时调用。须在停止接收/保护共享状态后由工程调用。
void GuidanceExample_ResetCameraSession() {
    g_vision_parser.reset();
    g_dma_last_pos = 0;
}

// 3) 离架时清AP；图像/导航/预测状态在各自模块一并清零。
void GuidanceExample_OnLaunch() { g_autopilot.reset(); }

// 4) 周期任务每5ms调用，实际dt应由单调时钟提供。
//    预测任务独立约20ms更新，不能在200Hz任务/IRQ里阻塞运行大积分。
void GuidanceExample_ControlTick() {
    ControlInputExample input;
    const bool ready = Platform_ReadControlInput(input);
    if (!ready) input.flight_enabled = false;
    const ControlOutputExample output = g_autopilot.update(input);
    Platform_WriteServoPwm(output.pwm_us);
}
