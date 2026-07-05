/**
 * @file WeaponSage_Setup.h
 * @author XieFField 70er66
 * @brief 武器架控制实�?
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
#include "tim.h"
#include "Module_lora.h"


#define USE_NEW_AUTO 1
namespace WeaponSage_Setup
{
    typedef struct{
        bool init_flag = false;

        float debug_start = 1; //debug_start = 1表示开始调�?
		float now_times=0.0f;
        float calibrate_startTime = 0.0f;
        bool calibrate_start = false;
        bool is_calibrating = false;

        float closeclaw_startTime = 0.0f;
        float untight_startTime = 0.0f;

        bool is_closeclaw_start = false;
        bool is_untight_start = false;

        bool is_wrist_start=false;
        float wrist_startTime=0.0f;

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

        bool wrist_rotate_enable = false; //手腕�?动能标志�?

        bool is_claw_1_closed = false; // 夹爪1�?否闭合的状�?
        bool is_claw_2_closed = false; // 夹爪2�?否闭合的状�? 
        bool is_claw_3_closed = false; // 夹爪3�?否闭合的状�?
    }ctrl_status_S;

    typedef enum{
        //将自动过程的每个状态枚�?
        STATE_START,
        STATE_ARM_MOVE,
        STATE_CLAW_ADJUST,
        STATE_SAGE_ADJUST,
        STATE_DONE,
    }auto_GRABstate_S;

    typedef enum{
        STATE_START_1,
        STATE_SAGE_ADJUST_1,
        STATE_ARM_MOVE_1,
        STATE_SAGE_MOVE,
        STATE_DONE_1,
    }auto_NEW_GRABstate_S;

    typedef enum{
        BCAK_STATE_START,
        BACK_SAGE_ARM_MOVE,
        BACK_WRIST_ADJUST,
        BACK_CLAW_DOWN,
        BACK_DONE,
    }weapon_BACKstate_S;
    typedef struct{
        float last_right_stick_x = 0.0f;
        float last_right_stick_y = 0.0f;

        bool changeTarget_state = false; //变更�?标状态标志位
    

    }manual_ctrlForgrip_S;


    typedef struct{
        bool is_Dpad_left_enabled = false;       //�Ƿ�����D-pad����
        bool is_Dpad_right_enabled = false;
        bool is_Dpad_up_enabled = false;
        bool is_Dpad_down_enabled = false;
        bool is_Right_stick_enabled = false;//�Ƿ�ʹ����ҡ�˿���
        float last_right_stick_x = 0.0f;
        float last_right_stick_y = 0.0f;
        bool changeTarget_state = false; //�Ƿ����л�Ŀ��״̬
        float weapon_launch_fastrate = 0.0002; 
        float weapon_launch_slowrate = 0.0001;
        float weapon_wrist_rate=0.05f;
        bool is_Dpad_Done = false; //�Ƿ�����ʹ��D-pad����
        uint8_t last_Dpad_acting_state=0;
        uint8_t Dpad_acting_state=0;
    }manual_RC10_ctrlForgrip_S;

    typedef struct{

        struct{
			bool is_matching = false;  
            bool dock_start = false;
			bool arm_enable=false;
            bool launch_enable=false;
            bool wrist_enable=false;
            bool el05_wrist_enable=false;
        }auto_state_bool_S; //局部状态结构体
        struct{
            bool is_reach_sagelowest = false; //当升降达到最低点视为已经完成下降
            bool is_reach_closedclaw = false; //当夹�?到达�?标位�?视为已经完成抓取
            bool is_reach_start = false;
            bool is_clawed=false;
            bool is_untight=false;
            bool is_catched=false;  //当夹�?到达安全高度视为已经完成抓取
            bool is_moved=false;
            bool is_prepared=false; //准�?�就�?，满足抓取条�?
			bool is_arm_reset=false;
            bool is_over = false;
            bool is_catch = false; //碰到杆了
            bool is_reach_armrotate=false;
        }flag;
        bool auto_ctrl1 = false;
        int pole_num = 1;
        bool claw_flag[3]={false,false,false};
        struct{
            float launch_start = 0.82f;
            float launch_catch =0.0f;
            float launch_untight =0.45f;
            float launch_clawclosed = 0.8f;
            float launch_rotate =1.0f;
            float launch_lastcatch=0.20f;
            float launch_dockprepare=0.6f;
        //----------------------new_auto---------------------------
            float launch_sage_untight=0.9; //用于夹爪对其杆子后升降到合适位置以夹住杆子的中间部分
        }launch_kp;
		bool is_ServoStart= false;
		float dock_launch_time = 0.0f;      // autoControl_dock 定时器（替代 static）
		bool dock_is_launching = false;
        float sage_adjust_time = 0.0f;
        bool is_sage_adjust_start=false;

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
     * @brief 必须在注册完所有电机后调用一�? init() 来启动任务和完成必�?�的初�?�化，否则�?�器架将无法正常工作
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

        start(osPriorityNormal, 1024);
		wrist_encoder_ = wrist_encoder;
		
        ctrl_status_.init_flag = true;
    }

    void setLowerClawStart(bool start)
    {
        
    };

    bool isWeaponSageCalibrated() const
    {
        if(ctrl_status_.is_calibrating)
            return true;
        else
            return false;
    }

    bool is_auto_ctrl_over(void)
    {
        return auto_ctrl_.flag.is_over;
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
        auto_control_state_ = 0;    
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

    bool is_catch(){ return auto_ctrl_.flag.is_catch;}
    

    bool Close_TargetClaw();
    bool Close_TargetClaw_Untight();
    bool Close_TargetClaw_Untight1();
    void Judge_launch_status();
    void Judge_wrist_status();
    
    bool Get_Catch_flag(void)
    {
        return auto_ctrl_.flag.is_catched;
    }
    void Get_OMNI_IM_flag(bool a)
    {
        // 只在 catch 阶段接受对位信号，防止初始化阶段被旧信号污染
        if (auto_ctrl_.auto_ctrl1 && auto_control_state_ == 1)
            auto_ctrl_.auto_state_bool_S.is_matching = a;
    }
    void Get_OMNI_DS_flag(bool a)
    {
        // 只在 dock 阶段接受对接开始信号，防止初始化/catch 阶段被旧信号污染
        if (auto_ctrl_.auto_ctrl1 && auto_control_state_ == 2)
            auto_ctrl_.auto_state_bool_S.dock_start = a;
    }
	
	void Set_AutoStart(bool flag)
	{
		auto_ctrl_.auto_ctrl1=flag;      //手动置位：是否可以自动
        auto_control_state_ = 0;              //状态机回到初始状态
	}

    bool is_auto_ctrl1()
    {
        return auto_ctrl_.auto_ctrl1;
    }
	
    
	void SetServo_Angle(uint16_t angle);
    void set_kfs_idle_arm_90(bool is_arm_90)
    {
        this->is_arm_90 = is_arm_90;
    }

    void set_kfs_idle_active(bool active)
    {
        this->kfs_idle_active_ = active;
    }
protected:
    void loop() override;

private:
	bool is_arm_90 = false;
	bool kfs_idle_active_ = false;
	bool omni_flag = false;
    WeaponSage_Setup::ctrl_status_S ctrl_status_;
    Debug_Printf debug_uart = Debug_Printf(&huart1);

    void kfs_idle();
    void test_point();

    void manualControl();
    void idle();
    void stop();
    void debug();
    bool autoControl_catch();
    void autoControl_dock();
    void autoControl();
    void calibrate();
    void semi_autoControl();
    void semi_low_level();

    bool D_pad_acting();

    float WristToClosest_poistive(float current_angle);
    float WristToClosest_negative(float current_angle);
    bool Sage_to_high();
    bool Sage_to_low();
    bool Sage_back();
    
	WeaponSage_Setup::auto_ctrl_S auto_ctrl_;

    uint8_t auto_control_state_ = 0;
    WeaponSage_Status_E weaponSage_status_ = WEAPONSAGE_STOP;
	WeaponSage_Status_E last_weaponSage_status_ = WEAPONSAGE_STOP;
	WeaponSage_Setup::auto_GRABstate_S now_state_=WeaponSage_Setup::STATE_START;
    WeaponSage_Setup::auto_NEW_GRABstate_S now_new_state_= WeaponSage_Setup::STATE_START_1;
    WeaponSage_Setup::WeaponDock_E target_dock_ = WeaponSage_Setup::MID; // for auto_dock

    bool weapon_CameraStart = false; // 主状态机触发相机流程的标志位�?
    bool debug_launch_target_valid_ = false;
    float debug_launch_target_ = 0.0f;

	#if !USE_RC10_AIRJOY
    RmPocketData_t airjoy_data_; 
	#else
    communication::RC10_AirJoy_Data_S airjoy_data_; // -1 ~ 1
	#endif
    WeaponSage_Setup::manual_ctrlForgrip_S manual_ctrlForgrip_;

    OIDEncoder *wrist_encoder_ = nullptr;
	
	volatile float wrist_encoder_angle_ = 0.0f;
    WeaponSage_Setup::manual_RC10_ctrlForgrip_S manual_RC10_ctrlForgrip_;
    WeaponSage_Setup::weapon_BACKstate_S now_back_state_ =WeaponSage_Setup::BCAK_STATE_START;
};

extern WeaponSage_InitData_S initData_;

#endif

#endif // WEAPONSAGE_SETUP_H    