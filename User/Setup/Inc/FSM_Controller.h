/**
 * @file FSM_Controller.h
 * @version 1.0
 * @author XieFField
 * @brief 机器人总状态机控制器
 */


#ifndef __FSM_CONTROLLER_H
#define __FSM_CONTROLLER_H

#pragma once

#ifdef __cplusplus
extern "C" {

}
#endif  


#ifdef __cplusplus

#include "BSP_RTOS.h"
#include "Module_Air_joy.h"
#include "APP_tool.h"
#include "BSP_TimeStamp.h"
#include "FSMstauts_enum.h"
#include "Arm_Setup.h"
#include "omni_chassisSetup.h"
#include "Module_CrsfReceiver.h"
#include "WeaponSage_Setup.h"

class FSM_Controller:public RtosTask {
public:
    FSM_Controller() : RtosTask("FSM_Controller", 1) {}

    void registerArmSetup(ArmSetup *arm_setup)
    {
        arm_setup_ = arm_setup;
        arm_setup_registered_ = true;
    }

    void registerChassisSetup(OmniChassis_Setup *chassis_setup)
    {
        chassis_setup_ = chassis_setup;
        chassis_setup_registered_ = true;
    }

    void registerWeaponSageSetup(Robot_WeaponSage_Setup *weaponSage_setup)
    {
        weaponSage_setup_ = weaponSage_setup;
        weaponSage_setup_registered_ = true;
    }

    void init()
    {
        if(!arm_setup_registered_ || !chassis_setup_registered_ || !weaponSage_setup_registered_)
            init_flag_ = false;
        
        this->arm_setup_->set_TargetKFS(3,0); //设置目标梅花桩编号

        this->start(osPriorityHigh, 256);
        init_flag_ = true;
    }

    void reset_airjoy_deadzone(float deadzone)
    {
        airjoy_deadzone_ = deadzone;
    }
private:
    void loop() override;

    //全部停下
    void all_stop();

    void manual_ctrl();

    void auto_ctrl();

    void debug();
    

    FSM_Status_E robot_status_ = ALL_STOP; FSM_Status_E last_robot_status_;

    float airjoy_deadzone_ = 50.0f; bool airjoy_connected_ = false;

    Robot_WeaponSage_Setup *weaponSage_setup_ = nullptr;
    bool weaponSage_setup_registered_ = false;
    
    ArmSetup *arm_setup_ = nullptr;  
    bool arm_setup_registered_ = false; 
    RmPocketData_t airjoy_data_; //摇杆值为 -1 ~ 1

    OmniChassis_Setup *chassis_setup_ = nullptr; 
    bool chassis_setup_registered_ = false; 
    bool init_flag_ = false; //所有需要注册的机构都已经注册完成
    uint8_t debug_flag_ = 0;
};

#endif




#endif

