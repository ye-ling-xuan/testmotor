#pragma once

#include "motor_pos_controller.hpp"

namespace GripperConfig
{

// 夹爪开合目标角（deg，相对上电自动归零后的零点）
inline constexpr float kOpenAngle  = 110.0f;
inline constexpr float kCloseAngle = 0.0f;

// 状态机到位判据
inline constexpr float    kSettleTolerance = 2.0f;   // |当前角-目标| < 2° 视为到位
inline constexpr uint32_t kSettleTimeoutMs = 2000U;  // 单步超时保护

// 软件限位（deg）
inline constexpr float kAngleMin = -360.0f;
inline constexpr float kAngleMax = 360.0f;

// M3508 位置环：ExternalPID 串级。位置环 100Hz、速度环 1kHz。
// 注意 abs_output_max 是 Iq 原始计数，不是安培（M3508 用 12000）。
inline constexpr controllers::MotorPosController::Config kPosConfig{
    .position_pid = { .Kp = 2.0f, .Ki = 0.0f, .Kd = 0.2f, .abs_output_max = 120.0f },
    .velocity_pid = { .Kp = 450.0f, .Ki = 0.1f, .Kd = 0.0f, .abs_output_max = 12000.0f },
    .pos_vel_freq_ratio = 10U,
};

} // namespace GripperConfig
