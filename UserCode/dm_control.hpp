#pragma once

namespace DmMotor
{

extern volatile float dbg_angle;
extern volatile float dbg_target;
extern volatile float dbg_rpm;

void dm_motor_init();
void dm_control_init();
void update_1kHz();

} // namespace DmMotor
