/**
 * @file WeaponSage_Setup.h
 * @author XieFField
 * @brief 锟斤拷锟斤拷锟斤拷师应锟矫诧拷
 */


#ifndef WEAPONSAGE_SETUP_H
#define WEAPONSAGE_SETUP_H

#pragma once

#ifdef __cplusplus
extern "C" {
    #include "arm_math.h"
}
#endif


#ifdef __cplusplus

#include "WeaponSage.h"
#include "FSMstauts_enum.h"
#include "BSP_RTOS.h"
#include "APP_debugTool.h"
#include "Module_CrsfReceiver.h"

namespace WeaponSage_Setup
{
    typedef struct{
        bool init_flag = false;

        float debug_start = 1; //锟斤拷锟皆匡拷始锟斤拷志 == 1 锟斤拷始锟斤拷锟斤拷

        float calibrate_startTime = 0;
        bool calibrate_start = false;
        bool is_calibrating = false;
    }ctrl_status_S;

    typedef enum{
        //将自动过程的每个状态枚举
        STATE_AIM_POSITION, //对准位置
        STATE_LOWER_CLAW,  //下降爪子
        STATE_GRAB_CLAW,   //抓取爪子
        STATE_LIFT_POSITION, //提升位置
    }auot_GRABstate_S;


    typedef struct{

        struct{
            // bool start

        }auto_state_bool_S; //局部状态结构体
    }auto_ctrl_S;

    float weapon_pos[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //武器位置数组

}





class Robot_WeaponSage_Setup : public RtosTask, public Robot_WeaponSage {
public:
    Robot_WeaponSage_Setup(WeaponSage_InitData_S init_data);
    
    void init(M3508* launch_Motor, M2006* claw_Motor,
        M2006* traverse_Motor, DM_Motor* wrist_Motor)
    {
        this->register_launch_Motor(launch_Motor);
        this->register_claw_Motor(claw_Motor);
        this->register_traverse_Motor(traverse_Motor);
        this->register_wrist_Motor(wrist_Motor);

        if(this->launch_Motor_ == nullptr ||
           this->claw_Motor_ == nullptr ||
           this->traverse_Motor_ == nullptr ||
           this->wrist_Motor_ == nullptr
        )
        {
            ctrl_status_.init_flag = false;
            return;
        }

        start(osPriorityNormal, 256);

        ctrl_status_.init_flag = true;
    }

    void setLowerClawStart(bool start)
    {
        
    }

    Point2D getClawPos()
    {   
        Point2D pos = {0.0f, 0.0f, 0.0f};
        return pos;
    }

    void setWeaponSageControlStatus(WeaponSage_Status_E status)
    {
        weaponSage_status_ = status;
    }

    Point2D getClawPos()
    {   
		
		this->current_pos_= get_CurrentPos();
		
        Point2D pos = {0.0f, 0.0f, 0.0f};
		pos.x=current_pos_.traverse_pos_;
		pos.y=current_pos_.launch_pos_;
		pos.theta=current_pos_.claw_pos_;
		
        return pos;
    }
    void setWeaponSageStatus(WeaponSage_Status_E status)
    {
        weaponSage_status_ = status;
    }

protected:
    void loop() override;

private:
    WeaponSage_Setup::ctrl_status_S ctrl_status_;
    Debug_Printf debug_uart = Debug_Printf(&huart1);

    void manualControl();
    void idle();
    void stop();
    void debug();
    void autoControl();

    void calibrate();


    bool State_AimPosition(int pole_num);
    void State_LowerClaw();
    bool State_GrabClaw();
    void State_Lift();
	WeaponSage_Setup::auto_ctrl_S auto_ctrl_;

    
    WeaponSage_Status_E weaponSage_status_ = WEAPONSAGE_IDLE;
    RmPocketData_t airjoy_data_; 
};

extern WeaponSage_InitData_S initData_;

#endif

#endif // WEAPONSAGE_SETUP_H