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
#include "Setup_ConfigInit.h"
#include "Locate_Setup.h"
#include "Module_lora.h"
#include "Module_Serial1Protocol.h"

typedef enum{
    RELOCATE,
    SET_KFS,
    SET_SPEAR,
    NONE,
}set_e;


typedef struct KSTarget_t {
    uint8_t KFS[2];
    int8_t Spear;

    bool operator!=(const KSTarget_t& other) const {
         return KFS[0] != other.KFS[0] || KFS[1] != other.KFS[1] || Spear != other.Spear;
    }
};

class FSM_Controller:public RtosTask {
public:
    FSM_Controller() : RtosTask("FSM_Controller\0", 1) {}

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
        
        this->arm_setup_->set_TargetKFS(0,0); //设置目标梅花桩编号
        this->start(osPriorityHigh+1, 256);
        init_flag_ = true;
    }

private:
    void loop() override;

    //全部停下
    void all_stop();

    void manual_ctrl();

    void auto_ctrl();

    void debug();
#if !USE_RC10_AIRJOY
    void stop_modeswitch();
#endif

    FSM_Status_E robot_status_ = ALL_STOP; FSM_Status_E last_robot_status_;


    Robot_WeaponSage_Setup *weaponSage_setup_ = nullptr;
    bool weaponSage_setup_registered_ = false;
    
    ArmSetup *arm_setup_ = nullptr;  
    bool arm_setup_registered_ = false; 

    

    OmniChassis_Setup *chassis_setup_ = nullptr; 
    bool chassis_setup_registered_ = false; 
    bool init_flag_ = false; //所有需要注册的机构都已经注册完成
    uint8_t debug_flag_ = 0;

#if !USE_RC10_AIRJOY

    RmPocketData_t airjoy_data_; //摇杆值为 -1 ~ 1
    struct{
        
        TargetSet_t rsf_send_data={0};
        uint16_t count = 0;
        uint8_t now_setKFSindex = 0;
        uint8_t sroll_wheel_last = 0;
        bool isread_srollWheelKFS = false;
        bool kfs_setDone = false;
        bool spear_setDone = false;
        // bool issetFirstKFS = false;

        bool isread_srollWheelSpear = false;
    }crsf_send_s;

    set_e Stop_set_stauts = NONE;
#else

    communication::RC10_AirJoy_Data_S airjoy_data_; //摇杆值为 -1 ~ 1
    void set_cmd_to_R2();

#endif

    KSTarget_t KStarget = {0};
    KSTarget_t last_KStarget = {0};
};

#endif

/*
STOP 模式下的状态机
有三种状态
*/



#endif

