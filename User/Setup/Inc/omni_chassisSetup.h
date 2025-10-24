/**
 * @file omni_chassisSetup.h
 * @brief È«Ïòµ×ÅÌ¿ØÖÆ
 */
#ifndef __OMNI_CHASSISSETUP_H
#define __OMNI_CHASSISSETUP_H

#pragma once




#ifdef __cplusplus
#include "BSP_RTOS.h"   
#include "Module_ChassisOmni.h"
#include "Motor_Base.h"
#include "FSMstauts_enum.h"

class OmniChassis_Setup:public RtosTask, public Chassis_Omni<4>{
public:
    OmniChassis_Setup(float wheel_radius, float max_wheel_rpm, float chassis_radius)
        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<4>(wheel_radius, max_wheel_rpm, chassis_radius)
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

    

private:
        void loop() override;
        bool init_flag = false;
       
        CHASSIS_Status_E chassis_status_ = CHASSIS_STOP;
};
#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H