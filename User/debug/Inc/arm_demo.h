/**
 * @file arm_demo.h
 * @author XieFField
 * @brief 机械吸盘测试Demo
 */

#ifndef __ARM_DEMO_H
#define __ARM_DEMO_H

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "BSP_RTOS.h"
#include "BSP_fdCAN_Driver.h"
#include "Motor_DJI.h"
#include "APP_PID.h"
#include "APP_debugTool.h"
#include "Robot_Arm.h"  
#include "BSP_TimeStamp.h"

#include "Module_Air_joy.h"



// extern AirJoy air_joy;

class Robot_ArmDemo: public Robot_Arm, public RtosTask{
public:
    Robot_ArmDemo(Arm_InitData_S init_Data);

    void armInit(DJI_Motor *motor_ArmLaunch, DJI_Motor *motor_ArmStretch, 
        DJI_Motor *motor_ArmRotate, DJI_Motor *motor_ArmPitch);

protected:
    void loop() override;

    static inline float step_pm(uint16_t us, uint16_t mid=1500, uint16_t dead=50, float rate=1.0f)
    {
        // 返回单位步进速率系数（-rate..+rate）
        if(us > mid + dead) return +rate;
        if(us < mid - dead) return -rate;
        return 0.0f;
    }

private:
    Debug_Printf debug_uart = Debug_Printf(&huart1);

        bool init_flag = false;
};



#endif // __cplusplus

#endif
