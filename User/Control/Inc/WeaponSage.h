/**
 * @file WeaponSage.h
 * @author XieFField
 * @brief 武器大师控制驱动�??
 * @version 1.0
 */
#ifndef WEAPONSAGE_H
#define WEAPONSAGE_H

#pragma once

#ifdef __cplusplus
extern "C" {
    #include "arm_math.h"
}

#endif // __cplusplus

#ifdef __cplusplus

#include <iostream> 
#include <cstdint>
#include "Motor_DJI.h"
#include "Motor_DM.h"
#include "APP_tool.h"
#include "Motor_GO.h"
#include "BSP_TimeStamp.h"
#include "FSMstauts_enum.h"
//初�?�化数据结构�??
#define USE_ARM_MIT  1
typedef struct 
{
    /* data */
    float max_launchHeight_; // 最大抬升高�??
    float max_clawAngle_; // 最大夹�??角度
    float max_arm_angle_; // 最大机械臂角度
    float max_wrist_angle_;
    float max_arm_rate_;
    float dock_height_;
    float wrist_protect_;
	bool is_Arm_fast;
	float min_arm_rate_;
    float claw_untight ;
	float arm_kp;
	float arm_kd;
	float arm_tff;
    float wrist_gearRatio_; //手腕减速比，手腕电机转一圈，�??�??关节�??多少�?? 360度，直驱
    float launch_Ratio_; // �??升减速比，抬升电机转一圈，�??�??关节移动多少�??
    float claw_gearRatio_; // 夹爪减速比，夹�??电机�??一圈，�??�??关节移动多少�??
    float arm_gearRatio_; // 机�?�臂减速比，机械臂电机�??一圈，�??�??关节移动多少�??
}WeaponSage_InitData_S;



namespace WeaponSage
{
    enum Motor_Type_E
    {
        Launch_Motor, // �??升电�??
        Arm_Motor, // 机�?�臂电机
        Claw_1_Motor,// 夹爪电机1
        Claw_2_Motor,// 夹爪电机2
        Claw_3_Motor,// 夹爪电机3
        Wrist_Motor// 手腕电机
    };

	typedef struct{
		float ramp__maxspeed_ = 0.0f;
		float max_accel_ = 0.0f; // 最大加速度 (Motor Angle deg/s^2)
		float current_velocity_ = 0.0f; // 当前速度 (Motor Angle deg/s)
		float ramp_target_ = 0.0f; 
		float filter_k_ = 0.0f; 
	}Fliter_Ramp_S;
	
    typedef struct 
    {
        float claw_1_reversed_ = -1.0f;
        float claw_2_reversed_ = -1.0f;
        float claw_3_reversed_ = -1.0f;
        float wrist_reversed_ = 1.0f;
        float launch_reversed_ = -1.0f; 
        float arm_reversed_ = 1.0f;
    }MotorReversed_S;

    enum WeaponSage_CtrlMode_S 
    {
        /* data */
        CURRENT_CONTROL, // 电流控制模式，直接控制电流输�??
        Join_POSITION_CONTROL, // 位置控制模式，控制关节位�??
        TOTAL_ANGLE_CONTROL,   // 总�?�度控制模式，控制关节总�?�度
    };
    
    typedef struct
    {
        float launch_pos_; //主�?�供调试时候使�??，实际控制以launch_TotalAngle_为准，单位米
        float claw_1_pos_;
        float claw_2_pos_;
        float claw_3_pos_;
        float wrist_pos_;
        float arm_pos_;

        float launch_TotalAngle_;
        float claw_1_TotalAngle_;
        float claw_2_TotalAngle_;
        float claw_3_TotalAngle_;
        float arm_TotalAngle_;
        float wrist_TotalAngle_;
    }WeaponSage_Pos_S;

};

class Robot_WeaponSage {

public:
    Robot_WeaponSage(WeaponSage_InitData_S init_data);
    ~Robot_WeaponSage(){}

    bool register_launch_Motor(M3508* motor)
    { 
        launch_Motor_ = motor; 
        if(launch_Motor_ != nullptr)
            return true; 
        else
            return false;
    }


    bool register_claw_1_Motor(M2006* motor)
    { 
        claw_1_Motor_ = motor; 
        if(claw_1_Motor_ != nullptr)
            return true; 
        else
            return false;
    }

    bool register_claw_2_Motor(M2006* motor)
    { 
        claw_2_Motor_ = motor; 
        if(claw_2_Motor_ != nullptr)
            return true; 
        else
            return false;
    }

    bool register_claw_3_Motor(M2006* motor)
    { 
        claw_3_Motor_ = motor; 
        if(claw_3_Motor_ != nullptr)
            return true; 
        else
            return false;
    }

    
    bool register_wrist_Motor(M2006* motor)
    { 
        wrist_Motor_ = motor; 
        if(wrist_Motor_ != nullptr)
            return true; 
        else
            return false;
    }

    bool register_arm_Motor(DM_Motor* motor)
    { 
        arm_Motor_ = motor; 
        if(arm_Motor_ != nullptr)
            return true; 
        else
            return false;
    }

    void update();
    
    /**
     * @brief 设置电机反转
     * @param reversed 需要反�??时传�?? true，否则传�?? false
     * @param motor_type 电机类型
     */

    bool setMotorReversed(bool reversed, WeaponSage::Motor_Type_E motor_type);


    bool setClaw_1_angle(float angle);
    bool setClaw_2_angle(float angle);
    bool setClaw_3_angle(float angle);
    bool setWrist_angle(float angle);
    bool setArm_angle(float angle);
    bool setLaunch_angle(float angle);
	bool setArm_angle_slow(float angle);



    void setCtrlMode(WeaponSage::WeaponSage_CtrlMode_S mode)
    {
        ctrl_mode_ = mode;
    }
	
    bool is_claw_close(int8_t index)
    {
        switch(index)
        {
            case 1:
                return claw_1_Motor_ != nullptr && current_pos_.claw_1_pos_ > 30.0f; // 夹爪角度大于30度认为是闭合状态
            case 2:
                return claw_2_Motor_ != nullptr && current_pos_.claw_2_pos_ > 30.0f;
            case 3:
                return claw_3_Motor_ != nullptr && current_pos_.claw_3_pos_ > 30.0f;
            default:
                return false;
        }
    }

	WeaponSage::WeaponSage_Pos_S get_CurrentPos()
	{
		WeaponSage::WeaponSage_Pos_S current_pos;
		current_pos.launch_pos_ = MotorTotalAngle_to_Realpos(launch_Motor_->getTotalAngle(), WeaponSage::Launch_Motor);
		current_pos.claw_1_pos_ = MotorTotalAngle_to_Realpos(claw_1_Motor_->getTotalAngle(), WeaponSage::Claw_1_Motor);
		current_pos.claw_2_pos_ = MotorTotalAngle_to_Realpos(claw_2_Motor_->getTotalAngle(), WeaponSage::Claw_2_Motor);
		current_pos.claw_3_pos_ = MotorTotalAngle_to_Realpos(claw_3_Motor_->getTotalAngle(), WeaponSage::Claw_3_Motor);
		current_pos.wrist_pos_ = MotorTotalAngle_to_Realpos(wrist_Motor_->getTotalAngle(), WeaponSage::Wrist_Motor);
        current_pos.arm_pos_ = MotorTotalAngle_to_Realpos(arm_Motor_->getTotalAngle(), WeaponSage::Arm_Motor);
		return current_pos;
	}

    void Weapon_arm_enable()
    {
        if(arm_Motor_ != nullptr)
            arm_Motor_->motorEnable();
    }
    void Weapon_arm_setZero()
    {
        if(arm_Motor_ != nullptr)
            arm_Motor_->motorSetZero();
    }

    float NormalizeAngle(float* angle)
    {
        float normalized = fmodf(*angle, 360.0f);
        if (normalized < 0) 
            normalized += 360.0f;
        return normalized;
    }

    
	void register_motors(M2006* claw_1_motor,M2006* claw_2_motor ,M2006* claw_3_motor, M3508* launch_motor ,M2006* wrist_motor, DM_Motor* arm_motor )
	{
		register_launch_Motor(launch_motor);
		register_claw_1_Motor(claw_1_motor);
		register_claw_2_Motor(claw_2_motor);
		register_claw_3_Motor(claw_3_motor);
		register_wrist_Motor(wrist_motor);
		register_arm_Motor(arm_motor);
	}
	
	WeaponSage_InitData_S getInitData() const { return initData_; }
	
private:

    WeaponSage::WeaponSage_CtrlMode_S ctrl_mode_ = WeaponSage::Join_POSITION_CONTROL;
    

    WeaponSage::MotorReversed_S motor_reversed_; 



protected:
	void set_claw1_angle(float angle)
	{
		target_pos_.claw_1_pos_ = angle;
	}

    float claw_speed_outlimit_ = 3000.0f;
    void setClawSpeedOutLimit(float limit) { claw_speed_outlimit_ = limit; }

    M3508 *launch_Motor_ = nullptr; // �??升电�??1，主电机
    M2006 *claw_1_Motor_ = nullptr; // 夹爪电机1，负责�?�器的夹取动�??
    M2006 *claw_2_Motor_ = nullptr; // 夹爪电机2，负责�?�器的夹取动�??
    M2006 *claw_3_Motor_ = nullptr; // 夹爪电机3，负责�?�器的夹取动�??
    M2006 *wrist_Motor_ = nullptr; // 手腕电机，负责�?�器的手腕动�??
    DM_Motor *arm_Motor_ = nullptr; // 机�?�臂电机，负责�?�器的机械臂动作

    WeaponSage::WeaponSage_Pos_S target_pos_;
    WeaponSage::WeaponSage_Pos_S current_pos_;
	WeaponSage::WeaponSage_Pos_S last_pos_;

    /**
     * @brief 将实际位�??�??�??为电机总�?�度
     * @param real_pos 实际位置
     * @param motor_type 电机类型
     */
    float Realpos_to_MotorTotalAngle(float real_pos, WeaponSage::Motor_Type_E motor_type);

    float MotorTotalAngle_to_Realpos(float motor_angle, WeaponSage::Motor_Type_E motor_type);

    bool setMotorTargetTotalAngle(float total_angle, WeaponSage::Motor_Type_E motor_type);

	WeaponSage_InitData_S initData_;
	
	

    #if CHALEENGE_12ZONE
    WeaponSage::Fliter_Ramp_S launch_fliter_ramp_ = {
        .ramp__maxspeed_ = 90000.0f,
        .max_accel_ = 4000000.0f, //  (Motor Angle deg/s^2)
        .current_velocity_ = 0.0f, // 记录当前 launch 速度
        .ramp_target_ = 0.0f, 
        .filter_k_ = 850.0f // launch 滤波�?(平滑)系数，值越大响应越�?，越小越平滑
    };
    #else
    WeaponSage::Fliter_Ramp_S launch_fliter_ramp_ = {
        .ramp__maxspeed_ = 110000.0f,
        .max_accel_ = 4000000.0f, //  (Motor Angle deg/s^2)
        .current_velocity_ = 0.0f, // 记录当前 launch 速度
        .ramp_target_ = 0.0f, 
        .filter_k_ = 850.0f // launch 滤波�?(平滑)系数，值越大响应越�?，越小越平滑
    };
    #endif
    
    
    float caculate_ramp_target(float current, float target, WeaponSage::Fliter_Ramp_S &ramp)
    {
        float diff = target - current;
        
        float target_vel = diff * ramp.filter_k_;

        // 限制�?标速度
        if (target_vel > ramp.ramp__maxspeed_) target_vel = ramp.ramp__maxspeed_;
        if (target_vel < -ramp.ramp__maxspeed_) target_vel = -ramp.ramp__maxspeed_;

        // 计算最大速度变化�?
        float max_dv = ramp.max_accel_ * 0.001;
        if (target_vel > ramp.current_velocity_ + max_dv) 
            ramp.current_velocity_ += max_dv;

        else if (target_vel < ramp.current_velocity_ - max_dv) 
            ramp.current_velocity_ -= max_dv;
            
        else 
            ramp.current_velocity_ = target_vel;

        // 计算步长
        float step = ramp.current_velocity_ * 0.001;

        // 检查是否到达目标位�?
        if(std::abs(diff) < 0.01f && std::abs(ramp.current_velocity_) < 0.1f) 
        {
            ramp.current_velocity_ = 0.0f; // 停�?�电�?
            return target;
        }

        return current + step;
    }
	float arm_speed_ =initData_.max_arm_rate_;
};


#endif // __cplusplus


#endif // WEAPONSAGE_H