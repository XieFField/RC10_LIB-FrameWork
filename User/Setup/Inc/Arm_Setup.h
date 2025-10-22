/**
 * @file Arm_setup.h
 * @author Your Name
 * @brief ARM Cortex-M7 Setup Header
 * @version 1.0
 * @date 2025-09-17
 */

#ifndef __ARM_SETUP_H
#define __ARM_SETUP_H

#ifdef __cplusplus
extern "C" {
#include "stdint.h"
}
#endif
#include "BSP_RTOS.h"
#include "Robot_Arm.h"
#include "APP_Tool.h"
#include "Module_Air_joy.h"
#include "Motor_DJI.h"
#include "BSP_TimeStamp.h"
#include "APP_debugTool.h"


#ifdef __cplusplus

class ArmSetup: public RtosTask ,public Robot_Arm {
public:
    ArmSetup(Arm_InitData_S init_Data)
        : Robot_Arm(init_Data), RtosTask("ArmSetup", 1) 
    {
    }

    void init(DJI_Motor *motor_ArmLaunch, DJI_Motor *motor_ArmStretch, 
        DJI_Motor *motor_ArmRotate, DJI_Motor *motor_ArmPitch)
    {
        this->registerMotor_Launch(motor_ArmLaunch);
        this->registerMotor_Stretch(motor_ArmStretch);
        this->registerMotor_Rotate(motor_ArmRotate);
        this->registerMotor_Pitch(motor_ArmPitch);

        start(osPriorityNormal, 256);

        init_flag = true;
    }

    
    
private:
    bool init_flag = false;

    Debug_Printf debug_uart = Debug_Printf(&huart1);
protected:
    void loop() override;

    static inline float step_pm(uint16_t us, uint16_t mid=1500, uint16_t dead=60, float rate=0.25f)
    {
        // 返回单位步进速率系数（-rate..+rate）
        if(us > mid + dead) return +rate;
        if(us < mid - dead) return -rate;
        return 0.0f;
    }
};

#endif //__cplusplus


#endif // __ARM_SETUP_H
