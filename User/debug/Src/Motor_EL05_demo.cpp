#include "Motor_EL05_demo.h"

#include <cstring>

#include "BSP_fdCAN_Driver.h"
#include "BSP_TimeStamp.h"
#include "fdcan.h"

namespace {

EL05_MotorDemo el05_motor_demo;
bool el05_demo_registered = false;
bool el05_demo_bus_inited = false;
bool el05_demo_zero_latched = false;

fdCANbus *get_el05_bus()
{
    static fdCANbus *bus = fdCANbus::getInstance(&hfdcan1);
    return bus;
}

EL05_Motor &get_el05_motor()
{
    static EL05_Motor el05_motor_1(0x01U, 0xFDU, get_el05_bus());
    return el05_motor_1;
}

} // namespace

volatile uint8_t EL05_demo_StartSignal = 0U;
volatile float EL05_demo_TargetAngle_deg = 0.0f;
volatile float EL05_demo_DeltaTime_us = 0.0f;
volatile uint64_t EL05_demo_LastTime_us = 0U;

void EL05_MotorDemo::init()
{
    EL05_Motor &motor = get_el05_motor();
    fdCANbus *bus = get_el05_bus();

    if (!el05_demo_registered) {
        bus->registerMotor(&motor);
        el05_demo_registered = true;
    }

    if (!el05_demo_bus_inited) {
        bus->init();
        el05_demo_bus_inited = true;
    }

    motor.reset_controlFrequency(1000);
    start(osPriorityNormal, 256);

    const char *msg = "Hello EL05_MotorDemo on UART8 JustFloat!\r\n";
    HAL_UART_Transmit(&huart8, reinterpret_cast<const uint8_t *>(msg), std::strlen(msg), HAL_MAX_DELAY);
}

void EL05_MotorDemo::loop()
{
    EL05_Motor &motor = get_el05_motor();
    const uint64_t time_now = TimeStamp::getInstance().getMicroseconds();
    if (EL05_demo_LastTime_us > 0U) {
        EL05_demo_DeltaTime_us = static_cast<float>(time_now - EL05_demo_LastTime_us);
    }
    EL05_demo_LastTime_us = time_now;

    const float payload[] = {
        EL05_demo_TargetAngle_deg,
        motor.getAngle(),
        motor.getRPM(),
        motor.getTorque()
    };

    if (HAL_UART_GetState(&huart8) == HAL_UART_STATE_READY) {
        debug_uart.printf_DMA_JustFloat(payload, sizeof(payload) / sizeof(payload[0]));
    }

    if (EL05_demo_StartSignal == 0U) {
        motor.motorDisable();
        el05_demo_zero_latched = false;
    } else if (EL05_demo_StartSignal == 1U) {
        motor.motorEnable();
        motor.setTargetAngle(EL05_demo_TargetAngle_deg);
        el05_demo_zero_latched = false;
    } else if (EL05_demo_StartSignal == 2U) {
        if (!el05_demo_zero_latched) {
            motor.motorSetZero();
            el05_demo_zero_latched = true;
        }
    } else {
        el05_demo_zero_latched = false;
    }
}

void EL05_debug_demo_init()
{
    el05_motor_demo.init();
}
