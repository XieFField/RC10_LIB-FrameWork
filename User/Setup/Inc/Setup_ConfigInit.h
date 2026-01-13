/**
 * @file Setup_ConfigInit.h
 * @brief 启动文件，包含所有控制器和电机的实例化声明
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
#include "Motor_DM.h"
#include "APP_tool.h"
#include "BSP_TimeStamp.h"
#include "APP_PID.h"
#include "debug_setup.h"
#include "Module_Air_joy.h"
#include "Module_Position.h"
#include "Locate_Setup.h"
#include "Module_HWT.h"
/*==============Controller===============*/
#include "FSM_Controller.h"
#include "Arm_Setup.h"
#include "omni_chassisSetup.h"
#include "Module_CrsfReceiver.h"
#include "WeaponSage_Setup.h"
#if SPEEDPLANNER_DEMO_DEBUG
    #include "speedplanner_demo.h"
#endif



#include "Module_Position.h"
#include "Module_LaserPosition.h"
#if ARM_DEMO_DEBUG

        #include "arm_demo.h"
#endif



class test:public RtosTask {
public:
    test():RtosTask("test", 1) {}

    void init(DJI_Motor* motor) 
		{
				motor_t_ = motor;
        this->start(osPriorityNormal, 256);
    }

    void loop() override
    {
        a++;
			switch(mod)
			{
				case 0:
				{
					motor_t_->setTargetCurrent(0.0f);
					break;
				}
				
				case 1:
				{
					motor_t_->setTargetRPM(rpm);
					break;
				}
				
				case 2:
				{
					motor_t_->setTargetAngle(angle);
					break;
				}
				
				case 3:
				{
					motor_t_->setTargetTotalAngle(totalangle);
					break;
				}
				default:
						break;
			}
    }

int a = 0;
		DJI_Motor* motor_t_  = nullptr;
		int mod = 0;
		float totalangle=0, rpm=0, angle=0, current=0;
};






#endif // __cplusplus

#endif


#endif

