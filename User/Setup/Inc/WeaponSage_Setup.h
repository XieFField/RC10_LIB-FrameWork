/**
 * @file WeaponSage_Setup.h
 * @author XieFField 70er66
 * @brief 武器架控制实现
 * @version 1.0
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
#include "APP_PID.h"
#include "Module_CrsfReceiver.h"
#include "Locate_Setup.h"
#include "Module_OIDEncoder.h"

namespace WeaponSage_Setup
{
    typedef struct{
        bool init_flag = false;

        float debug_start = 1; //debug_start = 1表示开始调试
		float now_times=0.0f;
        float calibrate_startTime = 0.0f;
        bool calibrate_start = false;
        bool is_calibrating = false;

        int target_poleIndex = 0; //0~3号索引的矛杆
        
        int8_t last_manual_claw_state = 0; // 0: open, 1: close
        int8_t claw_switch_offset = 0;
        int8_t last_scroll_state = 0;
        int8_t scroll_offset = 0;

        int8_t isClaw_tight = 1; // 0 : open, 1: tight
        int8_t last_isClaw_tight = 1;

        int8_t arm_switch_offset = 0;
        int8_t last_arm_switch_state = 0;

        int8_t isArm_Vertical;   //竖直1 水平0
        int8_t last_isArm_Vertical;

        bool wrist_rotate_enable = false; //手腕转动能标志位

        bool is_claw_1_closed = false; // 夹爪1是否闭合的状态
        bool is_claw_2_closed = false; // 夹爪2是否闭合的状态 
        bool is_claw_3_closed = false; // 夹爪3是否闭合的状态
    }ctrl_status_S;

    typedef enum{
        //将自动过程的每个状态枚举
        STATE_AIM_POSITION, //对准位置
        STATE_LOWER_CLAW,  //下降爪子
        STATE_GRAB_CLAW,   //抓取爪子
        STATE_LIFT_POSITION, //提升位置
        STATE_DONE 
    }auto_GRABstate_S;


    typedef struct{
        float last_right_stick_x = 0.0f;
        float last_right_stick_y = 0.0f;

        bool changeTarget_state = false; //变更目标状态标志位
   
    }manual_ctrlForgrip_S;

    typedef struct{

        struct{
			bool is_matching = false;  
            bool grab_start = false;
            float grab_startTime = 0.0f;
            bool is_moving = false;  
			bool arm_enable=false;
        }auto_state_bool_S; //局部状态结构体
		float claw_close_pos = 32.36f;
        float claw_open_pos = 49.58f;
        float safe_height = 0.0f; 
;
        struct{
            bool is_clawed=false;
            bool is_catched=false;  //当夹爪到达安全高度视为已经完成抓取
        }flag;
        bool auto_ctrl1 = true;
        int pole_num = 1;
    }auto_ctrl_S;

     extern float weapon_pos[4];//武器位置数组


    typedef enum WeaponDock_E
    {
        LOW,
        MID,
        HIGH
    };
}





class Robot_WeaponSage_Setup : public RtosTask, public Robot_WeaponSage {
public:
    Robot_WeaponSage_Setup(WeaponSage_InitData_S init_data);
    
    /**
     * @brief 必须在注册完所有电机后调用一次 init() 来启动任务和完成必要的初始化，否则武器架将无法正常工作
     */
    void init(OIDEncoder * wrist_encoder)
    {
        if( this->launch_Motor_ == nullptr ||
            this->claw_1_Motor_ == nullptr ||
            this->claw_2_Motor_ == nullptr ||
			this->claw_3_Motor_ == nullptr ||
            this->wrist_Motor_ == nullptr  ||
			this->arm_Motor_==nullptr      
			)
        {
            ctrl_status_.init_flag = false;
            return;
        }

        start(osPriorityNormal, 512);

        ctrl_status_.init_flag = true;
    }

    void setLowerClawStart(bool start)
    {
        
    }

    bool isWeaponSageCalibrated() const
    {
        if(ctrl_status_.is_calibrating)
            return true;
        else
            return false;
    }

    void setTargetIndex(int8_t index)
    {
        ctrl_status_.target_poleIndex = index;
    }

    void setWeaponSageControlStatus(WeaponSage_Status_E status)
    {
        weaponSage_status_ = status;
        if(status != WEAPONSAGE_DEBUG)
        {
            debug_launch_target_valid_ = false;
        }
    }

   

    Point2D getClawPos()
    {   
		Point2D pos;
		this->current_pos_= get_CurrentPos();
		

        return pos;
    }
    void setWeaponSageStatus(WeaponSage_Status_E status)
    {
        weaponSage_status_ = status;
    }
	
	void Set_End_Flag(bool flag)
    {
        omni_flag = flag;
    }

    void setCBauto(bool flag)
    {
        auto_ctrl_.auto_ctrl1 = flag;
    }

    void set_dock(WeaponSage_Setup::WeaponDock_E dock)
    {
        target_dock_ = dock;
    }

    void set_claw_flag(bool claw_1, bool claw_2, bool claw_3)
    {
        ctrl_status_.is_claw_1_closed = claw_1;
        ctrl_status_.is_claw_2_closed = claw_2;
        ctrl_status_.is_claw_3_closed = claw_3;
    }

    bool Close_TargetClaw();

protected:
    void loop() override;

private:
	
	bool omni_flag = false;

    WeaponSage_Setup::ctrl_status_S ctrl_status_;
    Debug_Printf debug_uart = Debug_Printf(&huart1);

    void manualControl();
    void idle();
    void stop();
    void debug();
    void autoControl_catch();
    void autoControl_dock();

    void calibrate();


    bool State_AimPosition(int pole_num);
    void State_LowerClaw();
    bool State_GrabClaw();
    bool State_Lift();
    
	WeaponSage_Setup::auto_ctrl_S auto_ctrl_;

    
    WeaponSage_Status_E weaponSage_status_ = WEAPONSAGE_IDLE;
	WeaponSage_Status_E last_weaponSage_status_ = WEAPONSAGE_IDLE;
	WeaponSage_Setup::auto_GRABstate_S now_state_=WeaponSage_Setup::STATE_DONE;

    WeaponSage_Setup::WeaponDock_E target_dock_ = WeaponSage_Setup::MID; // for auto_dock

    bool weapon_CameraStart = false; // 主状态机触发相机流程的标志位。
    bool debug_launch_target_valid_ = false;
    float debug_launch_target_ = 0.0f;

	
    RmPocketData_t airjoy_data_; 

    WeaponSage_Setup::manual_ctrlForgrip_S manual_ctrlForgrip_;

    OIDEncoder *wrist_encoder_ = nullptr;
};

extern WeaponSage_InitData_S initData_;

#endif

#endif // WEAPONSAGE_SETUP_H    