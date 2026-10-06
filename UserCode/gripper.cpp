#include "gripper.hpp"
#include "config.hpp"
#include "device.hpp"
#include "cmsis_os2.h"
#include "main.h"                  // HAL_GetTick
#include "motor_pos_controller.hpp"

#include <cmath>

namespace Gripper
{

using MotorPosController = controllers::MotorPosController;
using namespace Device::motor;

volatile float dbg_angle  = 0.0f;
volatile float dbg_target = 0.0f;
volatile float dbg_rpm    = 0.0f;

namespace
{

MotorPosController* gripper_pos = nullptr;

// ============ 两个状态机 ============
enum class OpenFsmState  { Idle, Moving, Hold };
enum class CloseFsmState { Idle, Moving, Hold };

OpenFsmState  open_state     = OpenFsmState::Idle;
CloseFsmState close_state    = CloseFsmState::Idle;
uint32_t      state_start_ms = 0;

constexpr uint32_t kHoldDwellMs = 500U;   // 到位后驻留 0.5s

float current_angle()
{
    return gripper_motor->getAngle();
}

// ---- 开夹爪状态机 ----
void open_fsm_start()
{
    gripper_pos->setRef(GripperConfig::kOpenAngle);
    dbg_target     = GripperConfig::kOpenAngle;
    open_state     = OpenFsmState::Moving;
    state_start_ms = HAL_GetTick();
}

void open_fsm_tick()
{
    switch (open_state)
    {
    case OpenFsmState::Moving:
        if (std::fabs(current_angle() - GripperConfig::kOpenAngle) < GripperConfig::kSettleTolerance ||
            (HAL_GetTick() - state_start_ms) > GripperConfig::kSettleTimeoutMs)
        {
            open_state     = OpenFsmState::Hold;
            state_start_ms = HAL_GetTick();
        }
        break;
    case OpenFsmState::Hold:
        if ((HAL_GetTick() - state_start_ms) > kHoldDwellMs)
            open_state = OpenFsmState::Idle;
        break;
    default:
        break;
    }
}

bool open_fsm_busy()
{
    return open_state != OpenFsmState::Idle;
}

// ---- 合夹爪状态机 ----
void close_fsm_start()
{
    gripper_pos->setRef(GripperConfig::kCloseAngle);
    dbg_target     = GripperConfig::kCloseAngle;
    close_state    = CloseFsmState::Moving;
    state_start_ms = HAL_GetTick();
}

void close_fsm_tick()
{
    switch (close_state)
    {
    case CloseFsmState::Moving:
        if (std::fabs(current_angle() - GripperConfig::kCloseAngle) < GripperConfig::kSettleTolerance ||
            (HAL_GetTick() - state_start_ms) > GripperConfig::kSettleTimeoutMs)
        {
            close_state    = CloseFsmState::Hold;
            state_start_ms = HAL_GetTick();
        }
        break;
    case CloseFsmState::Hold:
        if ((HAL_GetTick() - state_start_ms) > kHoldDwellMs)
            close_state = CloseFsmState::Idle;
        break;
    default:
        break;
    }
}

bool close_fsm_busy()
{
    return close_state != CloseFsmState::Idle;
}

// ---- 按键消抖 + 下降沿检测（在 100Hz 线程里轮询）----
constexpr uint8_t kDebounceCnt = 3;   // 连续 3 次采样 ≈ 30ms 消抖

struct Button
{
    GPIO_TypeDef* port;
    uint16_t      pin;
    uint8_t       count   = 0;     // 0=完全松开, kDebounceCnt=完全按下
    bool          pressed = false; // 当前稳定状态

    // 每次 100Hz 调用一次；返回本次是否检测到「按下沿」
    bool update()
    {
        bool raw = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET); // 按下=低电平
        if (raw)      { if (count < kDebounceCnt) ++count; }
        else          { if (count > 0) --count; }
        bool now  = pressed ? (count > 0) : (count >= kDebounceCnt);
        bool edge = now && !pressed;   // 只在“松开→按下”那一下触发
        pressed   = now;
        return edge;
    }
};

Button open_btn { KEY_OPEN_GPIO_Port,  KEY_OPEN_Pin  };
Button close_btn{ KEY_CLOSE_GPIO_Port, KEY_CLOSE_Pin };

} // namespace

// ---- 100Hz 控制线程：只做状态机 / setRef，不做 PID 计算 ----
// 对应 MCU2 的 ClampControl 线程（clamp.cpp:545）与 Arm_softTIM
extern "C" void GripperControl(void* argument)
{
    (void)argument;
    for (;;)
    {
        bool open_req  = open_btn.update();
        bool close_req = close_btn.update();

        if (open_fsm_busy())        open_fsm_tick();
        else if (close_fsm_busy())  close_fsm_tick();
        else
        {
            if (open_req)       open_fsm_start();   // PE10 按下 → 开
            else if (close_req) close_fsm_start();  // PE11 按下 → 合
        }
        osDelay(10);   // 100Hz
    }
}


void gripper_init()
{
    gripper_pos = new MotorPosController(gripper_motor, GripperConfig::kPosConfig);
    gripper_pos->enable();
    gripper_pos->setRef(current_angle());   // 锁定当前位置，防止上电跳动
}

void gripper_control_init()
{
    static const osThreadAttr_t attr = {
        .name       = "gripper",
        .stack_size = 128 * 4,
        .priority   = (osPriority_t)osPriorityNormal1,
    };
    osThreadNew(GripperControl, nullptr, &attr);
}

// ---- 1kHz ISR：只做 PID update()，不做 enable/setRef/状态切换 ----
// 对应 MCU2 clamp.cpp:509 的约定：“ISR 上下文只 update()”
void update_1kHz()
{
    if (gripper_pos != nullptr)
    {
        gripper_pos->update();
    }
    dbg_angle = current_angle();
    dbg_rpm   = gripper_motor->getVelocity();
}

} // namespace Gripper
