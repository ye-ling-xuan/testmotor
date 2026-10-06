#include "cmsis_os2.h"

#include "device.hpp"
#include "gripper.hpp"
#include "dm_control.hpp"          // ← 必须加，否则 DmMotor 未声明
#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"
#include "watchdog.hpp"

// 1kHz 控制中断：喂狗 → 位置环计算 → 发 CAN 电流
extern "C" void TIM_Callback_1kHz(TIM_HandleTypeDef* htim)
{
    (void)htim;
    service::Watchdog::EatAll();
    Gripper::update_1kHz();
    DmMotor::update_1kHz();
    Device::update_1kHz();
}

extern "C" void Init(void* argument)
{
    (void)argument;

    Device::app_device_init();

    // 夹爪：等上线 + 上电归零
    while (!Device::motor::gripper_motor->isConnected())
        osDelay(1);
    osDelay(100);

    Gripper::gripper_init();
    Gripper::gripper_control_init();

    // DM：等上线 + 上电归零（第50帧，约50ms）
    while (!Device::motor::dm_motor->isConnected())
        osDelay(1);
    osDelay(100);

    DmMotor::dm_motor_init();
    DmMotor::dm_control_init();

    // 启动 1kHz 定时器（TIM5）
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz);
    HAL_TIM_Base_Start_IT(&htim5);

    osThreadExit();
}
