/**
 * @file omni_chassisSetup.h
 * @brief 全向底盘控制
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
class OmniChassis_Setup:public RtosTask, public Chassis_Omni<3>{
public:
    OmniChassis_Setup(float wheel_radius, float max_wheel_rpm, float base_length, float side_length, bool three_wheel)
        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<3>(wheel_radius, max_wheel_rpm, base_length, side_length, three_wheel)
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
        
        

        this->start(osPriorityHigh, 256);
        init_flag = true;
    }

    void regesiterCrsfReceiver(CrsfReceiver* receiver)
    {
        airjoy_ = receiver;
    }

private:
        void loop() override;
        bool init_flag = false;
       void manualControl_A();
          
        Robot_Twist last_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        
        // Robot_Twist chassis_maxSpeed_ = {0};
       
        CHASSIS_Status_E chassis_status_ = CHASSIS_STOP;
        RmPocketData_t airjoy_data_; //摇杆值为 -1 ~ 1
        CrsfReceiver *airjoy_;
};
#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H