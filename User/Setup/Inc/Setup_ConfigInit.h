/**
 * @file Setup_ConfigInit.h
 * @brief ∆Ù∂Ø≈‰÷√≥ı ºªØ
 */

#ifndef SETUP_CONFIGINIT_H
#define SETUP_CONFIGINIT_H

#ifdef __cplusplus
#pragma once

extern "C" {
    #include "stm32h7xx_hal.h"
    #include "cmsis_os.h"
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"
    #include "semphr.h"
    
    void ALL_Setup_ConfigInit(void);
}

#ifdef __cplusplus

#include "debug_setup.h"


#if DEBUG_DJI_Motor
#include "frame_demo.h"
#endif


#include <cstdint>
#include "BSP_CANFrame.h"
#include "BSP_RTOS.h"
#include "BSP_fdCAN_Driver.h"
#include "Motor_DJI.h"
#include "APP_tool.h"
#include "BSP_TimeStamp.h"
#include "APP_PID.h"
#include "debug_setup.h"
#include "Module_Air_joy.h"
#include "Module_Position.h"
#include "Locate_Setup.h"
/*==============Controller===============*/
#include "FSM_Controller.h"
#include "Arm_Setup.h"
#include "omni_chassisSetup.h"

#if SPEEDPLANNER_DEMO_DEBUG
    #include "speedplanner_demo.h"
#endif


#include "Module_Position.h"
#include "Module_LaserPosition.h"
#if ARM_DEMO_DEBUG

        #include "arm_demo.h"
#endif










#endif // __cplusplus

#endif


#endif

