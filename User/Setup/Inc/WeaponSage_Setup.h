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
			bool wrist_enable=false;
        }auto_state_bool_S; //局部状态结构体
		  float claw_close_pos = 32.36f;
        float claw_open_pos = 49.58f;
        float tarch_height = 0.0f; 
        float up_height = 0.0f;
        struct{
            bool aimposition_done = false;
            bool lowerclaw_done = false;
            bool grabclaw_done = false;
            bool lift_done = false;
        }flag;
        bool auto_ctrl1 = true;
        int pole_num = 1;
    }auto_ctrl_S;

     extern float weapon_pos[4];//武器位置数组

}





class Robot_WeaponSage_Setup : public RtosTask, public Robot_WeaponSage {
public:
    Robot_WeaponSage_Setup(WeaponSage_InitData_S init_data);
    
    /**
     * @brief 必须在注册完所有电机后调用一次 init() 来启动任务和完成必要的初始化，否则武器架将无法正常工作
     */
    void init()
    {
        if( this->launch_Motor_1_master == nullptr ||
            this->launch_Motor_1_slave == nullptr ||
            this->launch_Motor_2_master == nullptr ||
            this->launch_Motor_2_slave == nullptr ||
            this->claw_Motor_ == nullptr ||
            this->traverse_Motor_ == nullptr ||
            this->wrist_Motor_ == nullptr
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

    void set_camera_req(bool weapon_req, bool z_req, float z_ref)
    {
        camera_weapon_req_ = weapon_req; // 底盘下发：武器预对接动作请求位。
        camera_z_req_ = z_req; // 底盘下发：z 调整请求位。
        camera_z_ref_ = z_ref; // 底盘下发：z 调整参考值。
    }

    bool get_weapon_done() const
    {
        return camera_weapon_done_; // 武器回传：预对接动作完成位。
    }

    bool get_z_done() const
    {
        return camera_z_done_; // 武器回传：z 调整完成位。
    }

    void setDebugLaunchTarget(float launch_target)
    {
        debug_launch_target_ = launch_target;
        debug_launch_target_valid_ = true;
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
	
	void Set_End_Flag(bool flag)
    {
        omni_flag = flag;
    }

    void setCBauto(bool flag)
    {
        auto_ctrl_.auto_ctrl1 = flag;
    }

    void setWeapon_CameraStart(bool start)
    {
        weapon_CameraStart = start;
    }
	
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
    void autoControl();

    void camera_mode(); // 相机协同模式主流程。
    bool is_new_z(float z_now); // detect new z sample

    void calibrate();


    bool State_AimPosition(int pole_num);
    void State_LowerClaw();
    bool State_GrabClaw();
    bool State_Lift();
    
	WeaponSage_Setup::auto_ctrl_S auto_ctrl_;

    
    WeaponSage_Status_E weaponSage_status_ = WEAPONSAGE_IDLE;
	WeaponSage_Status_E last_weaponSage_status_ = WEAPONSAGE_IDLE;
	WeaponSage_Setup::auto_GRABstate_S now_state_=WeaponSage_Setup::STATE_DONE;


    bool weapon_CameraStart = false; // 主状态机触发相机流程的标志位。
    bool debug_launch_target_valid_ = false;
    float debug_launch_target_ = 0.0f;

    bool camera_weapon_req_ = false; // 底盘到武器：预对接动作请求位。

    bool camera_z_req_ = false; // 底盘到武器：z 调整请求位。

    float camera_z_ref_ = 0.0f; // 底盘到武器：z 调整参考值。

    bool camera_weapon_done_ = false; // 武器到底盘：预对接动作完成位。

    bool camera_z_done_ = false; // 武器到底盘：z 调整完成位。

    CamZ_Ctrl cam_z_ctrl_; // 相机 z 控制器。
    bool cam_z_run_ = false; // z 过程运行位。
    bool cam_z_req_last_ = false; // z 请求上升沿检测位。
    float cam_z_hold_ = 0.0f; // z 过程目标缓存。
    float cam_z_last_ = 0.0f; // 最近一次 z 样本。
    float cam_z_rpm_ = 0.0f; // 相机 z 速度指令缓存。
	
    RmPocketData_t airjoy_data_; 

    WeaponSage_Setup::manual_ctrlForgrip_S manual_ctrlForgrip_;
};

extern WeaponSage_InitData_S initData_;

#endif

#endif // WEAPONSAGE_SETUP_H    