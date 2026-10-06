#pragma once

#include "dji.hpp"
#include "dm.hpp"

namespace Device
{

namespace motor
{
inline motors::DJIMotor* gripper_motor = nullptr;
inline motors::DMMotor*  dm_motor     = nullptr;
}

void app_device_init();
void update_1kHz();

} // namespace Device
