#include "stm32_control_example.h"
#include <cmath>
#include <cstring>

namespace {
float clampValue(float x, float low, float high) {
    return x < low ? low : (x > high ? high : x);
}
uint16_t readU16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}
uint32_t readU32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
}

// =====================================================================
//  1. 过载PID（测量微分 + 角速率反馈 + 条件积分）
// =====================================================================
OverloadPid::OverloadPid(float Kp, float Ki, float Kd, float K_rate,
                         float sat, float integral_limit)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd), K_rate_(K_rate), sat_(sat),
      integral_limit_(integral_limit) { reset(); }

void OverloadPid::reset() {
    integral_g_s_ = previous_measured_g_ = 0;
    has_previous_ = false;
}

float OverloadPid::update(float command_g, float omega_deg_s,
                          float measured_g, float dt_s) {
    if (!std::isfinite(command_g) || !std::isfinite(omega_deg_s) ||
        !std::isfinite(measured_g) || !std::isfinite(dt_s) || dt_s <= 0) {
        reset();
        return 0;
    }
    const float error = command_g - measured_g;
    const float derivative = has_previous_ ? (measured_g - previous_measured_g_) / dt_s : 0;
    float next_integral = clampValue(integral_g_s_ + error * dt_s,
                                    -integral_limit_, integral_limit_);
    const float feedback = Kp_ * error - Kd_ * derivative + K_rate_ * omega_deg_s;
    float delta = -(feedback + Ki_ * next_integral);
    // 输出前有负号，因此用“积分对舵角的实际增量”判断是否加重饱和。
    if (std::fabs(delta) > sat_ && delta * (-Ki_ * (next_integral - integral_g_s_)) > 0) {
        next_integral = integral_g_s_;
        delta = -(feedback + Ki_ * next_integral);
    }
    integral_g_s_ = next_integral;
    previous_measured_g_ = measured_g;
    has_previous_ = true;
    return clampValue(delta, -sat_, sat_);
}

// =====================================================================
//  2. 三通道 -> 四舵 -> 脉宽；具体通道反向/零位仍需实测标定
// =====================================================================
PhysicalAutopilotExample::PhysicalAutopilotExample()
    : pitch_(80, 100, 0, .1f, 30), yaw_(80, 100, 0, .1f, 30) {}
void PhysicalAutopilotExample::reset() { pitch_.reset(); yaw_.reset(); }

ControlOutputExample PhysicalAutopilotExample::update(const ControlInputExample& in) {
    ControlOutputExample out;
    const bool finite = std::isfinite(in.raw_measurement_age_s) &&
        std::isfinite(in.trusted_target_age_s) && std::isfinite(in.roll_deg) &&
        std::isfinite(in.dt_s) && in.dt_s > 0 &&
        std::isfinite(in.command_pitch_g) && std::isfinite(in.command_yaw_g) &&
        std::isfinite(in.measured_pitch_g) && std::isfinite(in.measured_yaw_g) &&
        std::isfinite(in.omega_pitch_deg_s) && std::isfinite(in.omega_yaw_deg_s);
    if (!in.flight_enabled || !finite || in.raw_measurement_age_s < 0 ||
        in.raw_measurement_age_s > .15f || in.trusted_target_age_s < 0) {
        reset();
        return out;
    }
    // 对齐仿真：可信目标过期 -> 请求0g，raw仍新鲜时AP仍可保留积分。
    const bool trusted = in.trusted_target_age_s <= .15f;
    const float pitch_g = trusted ? in.command_pitch_g : 0;
    const float yaw_g = trusted ? in.command_yaw_g : 0;
    out.pitch_deg = pitch_.update(pitch_g, in.omega_pitch_deg_s, in.measured_pitch_g, in.dt_s);
    out.yaw_deg = yaw_.update(yaw_g, in.omega_yaw_deg_s, in.measured_yaw_g, in.dt_s);
    out.roll_deg = -.1f * in.roll_deg; // 默认legacy P；不是三轴PID
    const float p = out.pitch_deg, y = out.yaw_deg, r = out.roll_deg;
    const float mixed[4] = {p+y+r, p-y-r, -p+y-r, -p-y+r};
    for (unsigned i = 0; i < 4; ++i) {
        out.servo_deg[i] = clampValue(mixed[i], -30, 30);
        out.pwm_us[i] = static_cast<uint16_t>(1500 + 10 * out.servo_deg[i]);
    }
    out.active = true;
    return out;
}

// =====================================================================
//  3. 按字节解析，支持半包/粘包/坏帧重同步；不依赖DMA回调边界
// =====================================================================
uint16_t VisionCrc16(const uint8_t* bytes, std::size_t length) {
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(bytes[i]) << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = static_cast<uint16_t>((crc & 0x8000u) ? (crc << 1) ^ 0x1021u : crc << 1);
    }
    return crc;
}

VisionPacketParserV2::VisionPacketParserV2() { reset(); }
void VisionPacketParserV2::reset() { used_ = 0; last_sequence_ = 0; has_sequence_ = false; }
void VisionPacketParserV2::discardFirst() {
    if (used_) { --used_; std::memmove(buffer_, buffer_ + 1, used_); }
}

bool VisionPacketParserV2::push(uint8_t byte, VisionFrameV2& frame) {
    buffer_[used_++] = byte;
    for (;;) {
        if (used_ && buffer_[0] != 0xAA) { discardFirst(); continue; }
        if (used_ < 2) return false;
        if (buffer_[1] != 0x55) { discardFirst(); continue; }
        if (used_ < 4) return false;
        if (buffer_[2] != 2 || buffer_[3] != 24) { discardFirst(); continue; }
        if (used_ < 24) return false;
        if (VisionCrc16(buffer_, 22) != readU16(buffer_ + 22)) { discardFirst(); continue; }
        VisionFrameV2 candidate;
        candidate.sequence = readU16(buffer_ + 4);
        candidate.capture_time_us = readU32(buffer_ + 6);
        candidate.u_px = readU16(buffer_ + 10) / 16.0f;
        candidate.v_px = readU16(buffer_ + 12) / 16.0f;
        candidate.bbox_w = readU16(buffer_ + 14);
        candidate.bbox_h = readU16(buffer_ + 16);
        candidate.flags = buffer_[18];
        candidate.exposure_us = readU16(buffer_ + 20);
        const bool shape_ok = !(candidate.flags & 0xF0u) && buffer_[19] == 0 &&
            !(candidate.detected() && candidate.predicted()) &&
            (candidate.detected() || (!candidate.clipped() && candidate.u_px == 0 &&
             candidate.v_px == 0 && candidate.bbox_w == 0 && candidate.bbox_h == 0)) &&
            (!candidate.detected() || (candidate.u_px < 320 && candidate.v_px < 240 &&
             candidate.bbox_w > 0 && candidate.bbox_w <= 320 &&
             candidate.bbox_h > 0 && candidate.bbox_h <= 240));
        const uint16_t forward = static_cast<uint16_t>(candidate.sequence - last_sequence_);
        const bool fresh_sequence = !has_sequence_ || (forward > 0 && forward < 0x8000u);
        used_ = 0; // 完整校验帧已消费；相机重启/断连后需显式reset会话
        if (!shape_ok || !fresh_sequence) return false;
        last_sequence_ = candidate.sequence;
        has_sequence_ = true;
        frame = candidate;
        return true;
    }
}
