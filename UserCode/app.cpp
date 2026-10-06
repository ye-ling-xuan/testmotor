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

    constexpr uint32_t kConnectTimeoutMs = 1000U;   // 每个电机最多等 1s

    // ---- 夹爪：独立等待，超时跳过 ----
    uint32_t t = 0;
    while (!Device::motor::gripper_motor->isConnected() && t < kConnectTimeoutMs)
    { osDelay(1); ++t; }
    if (Device::motor::gripper_motor->isConnected())
    {
        osDelay(100);                       // 等 auto_zero 第 50 帧归零
        Gripper::gripper_init();
        Gripper::gripper_control_init();
    }

    // ---- DM：无条件初始化（DM 是应答式，必须先发使能帧才能收到反馈）----
    DmMotor::dm_motor_init();
    DmMotor::dm_control_init();


    // ---- 无条件启动 1kHz 中断（解耦关键）----
    HAL_TIM_RegisterCallback(&htim5, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback_1kHz);
    HAL_TIM_Base_Start_IT(&htim5);

    osThreadExit();
}

