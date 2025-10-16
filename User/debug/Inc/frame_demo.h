/**
 * @file frame_demo.h
 * @author XieFField
 * @brief �ܹ�����
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
#include "Module_Air_joy.h"




class DJI_MotorDemo: public RtosTask{
public:
    DJI_MotorDemo() : RtosTask("DJI_MotorDemo", 1), debug_uart(&huart1) {}
    void init();
    void loop() override;
    Debug_Printf debug_uart;

private:
    
};


class DM_MotorDemo: public RtosTask{
public:
    DM_MotorDemo() : RtosTask("DM_MotorDemo", 1), debug_uart(&huart1) {}
    void init();
    void loop() override;
    Debug_Printf debug_uart;
private:    
};


#endif // __cplusplus

#endif