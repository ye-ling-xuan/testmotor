#include "dm_control.hpp"
#include "device.hpp"
#include "cmsis_os2.h"
#include "main.h"
#include "motor_pos_controller.hpp"

#include <cmath>

namespace DmMotor
{

using MotorPosController = controllers::MotorPosController;
using namespace Device::motor;

volatile float dbg_angle  = 0.0f;
volatile float dbg_target = 0.0f;
volatile float dbg_rpm    = 0.0f;

namespace
{

// DM 目标角与到位判据
constexpr float    kDeg0             = 0.0f;
constexpr float    kDeg90            = 90.0f;
constexpr float    kSettleTolerance  = 2.0f;
constexpr uint32_t kSettleTimeoutMs  = 2000U;
constexpr uint32_t kHoldDwellMs      = 500U;

// InternalPos：无 PID，只降内部位置指令的发送频率（1kHz update / 10 = 100Hz 发）
constexpr controllers::MotorPosController::Config kPosConfig{
    .internal_set_ratio = 10,
};

MotorPosController* dm_pos = nullptr;

enum class Deg0FsmState  { Idle, Moving, Hold };
enum class Deg90FsmState { Idle, Moving, Hold };

Deg0FsmState  deg0_state     = Deg0FsmState::Idle;
Deg90FsmState deg90_state    = Deg90FsmState::Idle;
uint32_t      state_start_ms = 0;

float current_angle()
{
    return dm_motor->getAngle();
}

void deg0_fsm_start()
{
    dm_pos->setRef(kDeg0);
    dbg_target     = kDeg0;
    deg0_state     = Deg0FsmState::Moving;
    state_start_ms = HAL_GetTick();
}

void deg0_fsm_tick()
{
    switch (deg0_state)
    {
    case Deg0FsmState::Moving:
        if (std::fabs(current_angle() - kDeg0) < kSettleTolerance ||
            (HAL_GetTick() - state_start_ms) > kSettleTimeoutMs)
        {
            deg0_state     = Deg0FsmState::Hold;
            state_start_ms = HAL_GetTick();
        }
        break;
    case Deg0FsmState::Hold:
        if ((HAL_GetTick() - state_start_ms) > kHoldDwellMs)
            deg0_state = Deg0FsmState::Idle;
        break;
    default:
        break;
    }
}

bool deg0_fsm_busy() { return deg0_state != Deg0FsmState::Idle; }

void deg90_fsm_start()
{
    dm_pos->setRef(kDeg90);
    dbg_target     = kDeg90;
    deg90_state    = Deg90FsmState::Moving;
    state_start_ms = HAL_GetTick();
}

void deg90_fsm_tick()
{
    switch (deg90_state)
    {
    case Deg90FsmState::Moving:
        if (std::fabs(current_angle() - kDeg90) < kSettleTolerance ||
            (HAL_GetTick() - state_start_ms) > kSettleTimeoutMs)
        {
            deg90_state    = Deg90FsmState::Hold;
            state_start_ms = HAL_GetTick();
        }
        break;
    case Deg90FsmState::Hold:
        if ((HAL_GetTick() - state_start_ms) > kHoldDwellMs)
            deg90_state = Deg90FsmState::Idle;
        break;
    default:
        break;
    }
}

bool deg90_fsm_busy() { return deg90_state != Deg90FsmState::Idle; }

// 消抖按键（与 gripper.cpp 完全相同的结构，含迟滞）
constexpr uint8_t kDebounceCnt = 3;

struct Button
{
    GPIO_TypeDef* port;
    uint16_t      pin;
    uint8_t       count   = 0;
    bool          pressed = false;

    bool update()
    {
        bool raw = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);
        if (raw)      { if (count < kDebounceCnt) ++count; }
        else          { if (count > 0) --count; }
        bool now  = pressed ? (count > 0) : (count >= kDebounceCnt);
        bool edge = now && !pressed;
        pressed   = now;
        return edge;
    }
};

Button key0  { KEY_DEG0_GPIO_Port,  KEY_DEG0_Pin  };
Button key90 { KEY_DEG90_GPIO_Port, KEY_DEG90_Pin };

// 发送 DM 清除故障帧（FF FF FF FF FF FF FF FB → StdId 0x109），
// 不依赖库里的 clearFault()，只在 UserCode 里拼原始 CAN 帧。
void dm_clear_fault()
{
    constexpr uint8_t kClearErr[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB };

    CAN_TxHeaderTypeDef hdr{};
    hdr.StdId = 0x109;            // Pos(0x100) | id0(0x09)，与 device.cpp 里 dm_motor_config 一致
    hdr.IDE   = CAN_ID_STD;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.DLC   = 8;

    CAN_SendMessage(&hcan1, &hdr, kClearErr);
}

} // namespace

extern "C" void DmMotorControl(void* argument)
{
    (void)argument;
    for (;;)
    {
        dm_motor->ping();   // ← 新增：未使能/失能时维持一收一回，并补发使能帧
        
        // 上电"编码器未识别"等锁存故障：检测到就清故障 + 重新使能
        const auto st = dm_motor->state();
        if (st != motors::DMMotor::State::Enabled &&
            st != motors::DMMotor::State::Disabled)
        {
            dm_clear_fault();
            dm_motor->enable();
        }

        bool req0  = key0.update();
        bool req90 = key90.update();

        if (deg0_fsm_busy())        deg0_fsm_tick();
        else if (deg90_fsm_busy())  deg90_fsm_tick();
        else
        {
            if (req0)       deg0_fsm_start();   // PE8 → 0°
            else if (req90) deg90_fsm_start();  // PE9 → 90°
        }
        osDelay(10);   // 100Hz
    }
}

void dm_motor_init()
{
    dm_pos = new MotorPosController(dm_motor, kPosConfig);
    dm_pos->enable();                       // ① 先发使能帧，DM 收到后开始回反馈

    // ② 等第一帧反馈（使能后通常几 ms 内），此时 getAngle() 才有真实值
    for (uint32_t t = 0; t < 500 && !dm_motor->isConnected(); ++t)
        osDelay(1);

    dm_pos->setRef(current_angle());        // ③ 锁当前角，防止上电跳动
}


void dm_control_init()
{
    static const osThreadAttr_t attr = {
        .name       = "dm_motor",
        .stack_size = 128 * 4,
        .priority   = (osPriority_t)osPriorityNormal1,
    };
    osThreadNew(DmMotorControl, nullptr, &attr);
}

void update_1kHz()
{
    if (dm_pos != nullptr)
        dm_pos->update();
    dbg_angle = current_angle();
    dbg_rpm   = dm_motor->getVelocity();
}

} // namespace DmMotor
