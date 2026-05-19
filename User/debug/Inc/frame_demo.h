/**
 * @file frame_demo.h
 * @author XieFField
 * @brief 架构测试
 */

#ifndef __FRAME_DEMO_H
#define __FRAME_DEMO_H

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
#include "BSP_fdCAN_Driver.h"
#include "Motor_DJI.h"
#include "APP_PID.h"
#include "APP_debugTool.h"
#include "APP_CoordConvert.h"
#include "Motor_VESC.h"
#include "BSP_TimeStamp.h"
#include "Module_ChassisBase.h"
#include "Motor_DM.h"
#include "Module_GPIO.h"
#include "Module_Air_joy.h"

class FrameDemo : public RtosTask
{
public:
    FrameDemo() : RtosTask("FrameDemo", 1) {}
    void init();
    void loop() override;
    
};

//class CanTest : public fdCANbus
//{
//public:
//    CanTest(FDCAN_HandleTypeDef* hfdcan, uint32_t bus_id) : fdCANbus(hfdcan, bus_id) {}
//    ~CanTest(){}
//    
//};


class DJI_MotorDemo: public RtosTask{
public:
    DJI_MotorDemo() : RtosTask("DJI_MotorDemo", 10), debug_uart(&huart1) {}
    void init(DJI_Motor *motor);
    void loop() override;
    Debug_Printf debug_uart;
private:
    DJI_Motor *motor_;
};

class DM_MotorDemo:public RtosTask{
public:
	DM_MotorDemo() : RtosTask("DM_MotorDemo", 1), debug_uart(&huart1) {}
    void init();
    void loop() override;
    Debug_Printf debug_uart;
private:
};

class GPIODemo:public RtosTask{
public:
	GPIODemo() :RtosTask("GPIODemo", 1){}
	void init();
    void loop() override;
};

#endif // __cplusplus

#endif