/**
 * @file omni_chassisSetup.h
 * @brief ?????????
 */
#ifndef __OMNI_CHASSISSETUP_H
#define __OMNI_CHASSISSETUP_H

#pragma once



#ifdef __cplusplus
#include "BSP_RTOS.h"   
#include "Module_ChassisOmni.h"
#include "Motor_Base.h"
#include "FSMstauts_enum.h"
#include "Module_Air_joy.h"
#include "APP_PID.h"
#include "Module_Position.h"
#include "APP_debugTool.h"

class OmniChassis_Setup:public RtosTask, public Chassis_Omni<4>{
public:
    OmniChassis_Setup(float wheel_radius, float max_wheel_rpm, float chassis_radius)
        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<4>(wheel_radius, max_wheel_rpm, chassis_radius),debug_uart(&huart2)
    {}
    Debug_Printf debug_uart;
    void setChassisStatus(CHASSIS_Status_E status)
    {
        chassis_status_ = status;
    }

    void init();
    
    PID_Position yaw_pid;
    float yaw_adjust(float now_angle, float target_angle_);


private:
        void loop() override;
        bool init_flag = false;
        float yaw_lock_angle = 0.0f;
        float yaw_real_angle = 0.0f;
        float target_angle = 0.0f, real_angle = 0.0f;
        float yaw_correction = 0.0f;
        
        

        void chassis_control_manualA();
        void chassis_control_manualB();
        void chassis_control_auto();
        void chassis_control_stop();

        //Robot_Twist last_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        
        CHASSIS_Status_E chassis_status_ = CHASSIS_STOP;
};
#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H