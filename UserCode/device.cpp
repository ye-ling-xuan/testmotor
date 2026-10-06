#include "device.hpp"
#include "can.h"
#include "can_driver.hpp"
#include "cmsis_os2.h"

namespace Device
{

constexpr motors::DJIMotor::Config gripper_motor_config = {
    .hcan           = &hcan1,
    .type           = motors::DJIMotor::Type::M3508_C620,
    .id1            = 3,
    .auto_zero      = true,   // 上电第50帧自动归零（对应 MCU2 arm_rotate_motor_config）
    .reverse        = false,
    .reduction_rate = 1.0f,   // 无外接减速比
};

constexpr motors::DMMotor::Config dm_motor_config = {
    .hcan               = &hcan1,
    .id0                = 0x09,                             // 指令帧 = 0x100|0x09 = 0x109
    .type               = motors::DMMotor::Type::J4340_2EC,
    .mode               = motors::DMMotor::Mode::Pos,       // 内部位置模式
    .pos_max_rad        = 3.14159f,                         // PMAX
    .vel_max_rad        = 10.0f,                            // VMAX
    .tor_max            = 28.0f,                            // TMAX
    .default_angle_zero = 0.0f,
    .auto_zero          = false,     // ← 从 true 改成 false
    .reverse            = false,
    .reduction_rate     = 1.0f,
};


static void can_init()
{
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);                 // DJI 滤波器（0x200~0x20F）
    motors::DMMotor::CAN_FilterInit(&hcan1, 1, 0x00);            // DM 滤波器（master_id=0x00）
    CAN_RegisterCallback(&hcan1, motors::DJIMotor::CANBaseReceiveCallback);
    CAN_RegisterCallback(&hcan1, motors::DMMotor::CANBaseReceiveCallback);
    CAN_InitMainCallback(&hcan1);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}


static void motor_init()
{
    motor::gripper_motor = new motors::DJIMotor(gripper_motor_config);
    motor::dm_motor     = new motors::DMMotor(dm_motor_config);
}

void app_device_init()
{
    can_init();
    motor_init();
}

void update_1kHz()
{
    // 把缓存好的 Iq 打包成 0x200 帧发出（对应 MCU2 update_1kHz_1）
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
}

} // namespace Device
