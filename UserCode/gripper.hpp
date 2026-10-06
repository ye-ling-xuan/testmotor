#pragma once

namespace Gripper
{

// 调试观测变量：加进调试器 Watch 窗口看（对应 MCU2 app.cpp 里的 aaa）
extern volatile float dbg_angle;
extern volatile float dbg_target;
extern volatile float dbg_rpm;

void gripper_init();
void gripper_control_init();
void update_1kHz();

} // namespace Gripper
