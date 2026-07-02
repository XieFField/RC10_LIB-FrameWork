#ifndef __MOTOR_EL05_DEMO_H__
#define __MOTOR_EL05_DEMO_H__

#pragma once

#if defined(__cplusplus) && __cplusplus < 201103L
#error "Motor_EL05_demo.h requires C++11 or later."
#elif !defined(__cplusplus)
#error "Motor_EL05_demo.h requires a C++ compiler."
#endif

#include <cstdint>

#include "BSP_RTOS.h"
#include "APP_debugTool.h"
#include "frame_demo.h"
#include "Motor_EL05.h"

class EL05_MotorDemo : public RtosTask {
public:
    EL05_MotorDemo() : RtosTask("EL05_MotorDemo", 10), debug_uart(&huart8) {}

    void init();
    void loop() override;

private:
    Debug_Printf debug_uart;
};

extern volatile uint8_t EL05_demo_StartSignal;
extern volatile float EL05_demo_TargetAngle_deg;
extern volatile float EL05_demo_DeltaTime_us;
extern volatile uint64_t EL05_demo_LastTime_us;

void EL05_debug_demo_init();

#endif // __MOTOR_EL05_DEMO_H__
