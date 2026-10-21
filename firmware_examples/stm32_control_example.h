#ifndef STM32_CONTROL_EXAMPLE_H
#define STM32_CONTROL_EXAMPLE_H

#include <cstddef>
#include <cstdint>

// =====================================================================
//  移植示例：过载自驾仪 + OpenMV V2测量协议
// =====================================================================
//  与你原工程一样：构造时传增益，reset()清状态，update()每周期更新。
//  不依赖HAL/RTOS，便于先在PC测试，再接入GuidanceTask。
//  尚未包含：自身导航、世界目标观察器、弹道预测PN内核、实际PWM驱动。
//  本文件不是把旧PngGuidance改成N=3；预测导引需要单独实现。

class OverloadPid {
public:
    // Kp[deg/g], Ki[deg/(g*s)], Kd[deg*s/g], K_rate[s], sat[deg]
    OverloadPid(float Kp, float Ki, float Kd, float K_rate,
                float sat, float integral_limit = 10.0f);
    void reset();
    float update(float command_g, float omega_deg_s,
                 float measured_g, float dt_s);
    float integral() const { return integral_g_s_; }
private:
    float Kp_, Ki_, Kd_, K_rate_, sat_, integral_limit_;
    float integral_g_s_, previous_measured_g_;
    bool has_previous_;
};

// 两轴PID各有自己的积分、上一测量；滚转保留当前推荐的legacy P。
struct ControlInputExample {
    bool flight_enabled = false;         // 由实际离架状态机提供
    float raw_measurement_age_s = 1000;  // 距最近真实detected，不是任意收包年龄
    float trusted_target_age_s = 1000;   // 距最近locked且未裁切的实测
    float command_pitch_g = 0;           // 预测器已转为机体系的请求
    float command_yaw_g = 0;
    float measured_pitch_g = 0;          // -f_B.z / g
    float measured_yaw_g = 0;            // -f_B.y / g
    float omega_pitch_deg_s = 0;         // -omega_B.y * RAD2DEG
    float omega_yaw_deg_s = 0;           // +omega_B.z * RAD2DEG
    float roll_deg = 0;
    float dt_s = .005f;
};

struct ControlOutputExample {
    float pitch_deg = 0, yaw_deg = 0, roll_deg = 0;
    float servo_deg[4] = {0, 0, 0, 0};
    uint16_t pwm_us[4] = {1500, 1500, 1500, 1500};
    bool active = false;
};

class PhysicalAutopilotExample {
public:
    PhysicalAutopilotExample();
    void reset();
    ControlOutputExample update(const ControlInputExample& input);
private:
    OverloadPid pitch_, yaw_;
};

// =====================================================================
//  OpenMV -> STM32: AA55 / version2 / length24 / 小端 / CRC16
//  原7字节与10字节协议都不能直接交给这个解析器。
// =====================================================================
struct VisionFrameV2 {
    uint16_t sequence = 0;
    uint32_t capture_time_us = 0; // 相机时钟！需映射到STM32时钟后使用
    float u_px = 0, v_px = 0;
    uint16_t bbox_w = 0, bbox_h = 0, exposure_us = 0;
    uint8_t flags = 0;
    bool detected() const { return (flags & 0x01u) != 0; }
    bool locked() const { return (flags & 0x02u) != 0; }
    bool clipped() const { return (flags & 0x04u) != 0; }
    bool predicted() const { return (flags & 0x08u) != 0; }
    bool trusted() const { return detected() && locked() && !clipped() && !predicted(); }
};

uint16_t VisionCrc16(const uint8_t* bytes, std::size_t length);

class VisionPacketParserV2 {
public:
    VisionPacketParserV2();
    void reset();
    // 每个收到的字节调用一次。仅完整、校验正确且序号更新才返回true。
    // 无效检测状态包也会返回true，由flags决定能否更新可信目标。
    bool push(uint8_t byte, VisionFrameV2& frame);
private:
    void discardFirst();
    uint8_t buffer_[24];
    std::size_t used_;
    uint16_t last_sequence_;
    bool has_sequence_;
};

// 在stm32_task_integration_example.cpp中实现；Platform_*由你的工程提供。
void GuidanceExample_OnCircularDma(const uint8_t* buffer,
                                  uint16_t buffer_size, uint16_t write_pos);
void GuidanceExample_ResetCameraSession();
void GuidanceExample_OnLaunch();
void GuidanceExample_ControlTick();

#endif
