/**
 * @file speedplanner_demo.h
 * @author naoganlin
 * @brief 速度控制器demo,用宏定义调用开关
 * @version 1.0
 * @date 2025-10-28
 */

#ifndef __SPEEDPLANNER_DEMO_H
#define __SPEEDPLANNER_DEMO_H

#pragma once

#ifdef __cplusplus

extern "C"
{
#endif
#include "cmsis_os.h"
#include "usart.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "BSP_RTOS.h"
#include "APP_debugTool.h"
#include "APP_CoordConvert.h"
#include "BSP_TimeStamp.h"
#include "APP_Speedplanner.h"
#include "debug_setup.h"

#define trapezoid_Velocitytype 0
#define Positionaltype_1D 0
#define Positionaltype_2D 0

class SpeedPlanner_Demo : public RtosTask
{
public:
    SpeedPlanner_Demo() : RtosTask("SpeedPlanner_Demo", 1), debug_uart(&huart1) {}
    void init();
    void loop() override;
    Debug_Printf debug_uart;

private:
};

#endif // __cplusplus

#endif