/**
 * @file omni_chassisSetup.h
 * @brief ȫ����̿���
 */
#ifndef __OMNI_CHASSISSETUP_H
#define __OMNI_CHASSISSETUP_H

#pragma once



#ifdef __cplusplus
#include "BSP_RTOS.h"   
#include "Module_ChassisOmni.h"
#include "Motor_Base.h"
#include "FSMstauts_enum.h"
#include "Module_CrsfReceiver.h"
#include "APP_debugTool.h"
#include "usart.h"
#include "Module_Position.h"
#include "APP_PID.h"

class OmniChassis_Setup:public RtosTask, public Chassis_Omni<3>{
public:
    OmniChassis_Setup(float wheel_radius, float max_wheel_rpm, float base_length, float side_length, bool three_wheel)
        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<3>(wheel_radius, max_wheel_rpm, base_length, side_length, three_wheel)
        ,debug_uart(&huart8)
    {}

    void setChassisStatus(CHASSIS_Status_E status)
    {
        chassis_status_ = status;
    }

    void init() 
    {
        if(this->wheels_[0] == nullptr ||this->wheels_[1] == nullptr ||
           this->wheels_[2] == nullptr ||this->wheels_[3] == nullptr)
            init_flag = false;
        
        yaw_pid_.set_params(lock_angle_pid_params, 10000.0f); 
        

        this->start(osPriorityHigh, 256);
        init_flag = true;
    }


private:
        void loop() override;
        bool init_flag = false;
          
        Robot_Twist last_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        
        // Robot_Twist chassis_maxSpeed_ = {0};
        float target_yaw_ = 0.0f;
        uint8_t yaw_pid_period_ = 3;
        uint8_t yaw_pid_period_count_ = 0;
        PID_Position yaw_pid_;

        const float LINESPEED_LIMIT = 10/500.f; // ����ģң��������ֵӳ��Ϊ���ٶȵı���
        const float YAWSPEED_LIMIT = 1/500.f; // ����ģң��������ֵӳ��Ϊ���ٶȵı���


        CHASSIS_Status_E chassis_status_ = CHASSIS_STOP;
        RmPocketData_t airjoy_data_; //ҡ��ֵΪ -1 ~ 1

        Debug_Printf debug_uart; // ���Դ�ӡʵ��
};
#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H