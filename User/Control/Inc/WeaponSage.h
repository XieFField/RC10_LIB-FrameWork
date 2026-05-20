/**
 * @file WeaponSage.h
 * @author XieFField
 * @brief 武器大师控制驱动类
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

//初始化数据结构体
typedef struct 
{
    /* data */
    float max_launchHeight_; // 最大抬升高度
    float max_clawAngle_; // 最大夹爪角度
    float max_arm_angle_; // 最大机械臂角度
    float max_wrist_angle_;
    float max_arm_rate_;

    float wrist_gearRatio_; //手腕减速比，手腕电机转一圈，末端关节转多少度 360度，直驱
    float launch_Ratio_; // 抬升减速比，抬升电机转一圈，末端关节移动多少米
    float claw_gearRatio_; // 夹爪减速比，夹爪电机转一圈，末端关节移动多少米
    float arm_gearRatio_; // 机械臂减速比，机械臂电机转一圈，末端关节移动多少度
}WeaponSage_InitData_S;

namespace WeaponSage
{
    enum Motor_Type_E
    {
        Launch_Motor, // 抬升电机
        Arm_Motor, // 机械臂电机
        Claw_1_Motor,// 夹爪电机1
        Claw_2_Motor,// 夹爪电机2
        Claw_3_Motor,// 夹爪电机3
        Wrist_Motor// 手腕电机
    };

    typedef struct 
    {
        float claw_1_reversed_ = -1.0f;
        float claw_2_reversed_ = -1.0f;
        float claw_3_reversed_ = -1.0f;
        float wrist_reversed_ = -1.0f;
        float launch_reversed_ = -1.0f; 
        float arm_reversed_ = -1.0f;
    }MotorReversed_S;

    enum WeaponSage_CtrlMode_S 
    {
        /* data */
        CURRENT_CONTROL, // 电流控制模式，直接控制电流输出
        Join_POSITION_CONTROL, // 位置控制模式，控制关节位置
        TOTAL_ANGLE_CONTROL,   // 总角度控制模式，控制关节总角度
    };
    
    typedef struct
    {
        float launch_pos_; //主要供调试时候使用，实际控制以launch_TotalAngle_为准，单位米
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
     * @param reversed 需要反转时传入 true，否则传入 false
     * @param motor_type 电机类型
     */

    bool setMotorReversed(bool reversed, WeaponSage::Motor_Type_E motor_type);


    bool setTarget(float targetValue, WeaponSage::Motor_Type_E motor_type);

    void setCtrlMode(WeaponSage::WeaponSage_CtrlMode_S mode)
    {
        ctrl_mode_ = mode;
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
	
private:

    WeaponSage::WeaponSage_CtrlMode_S ctrl_mode_ = WeaponSage::Join_POSITION_CONTROL;
    

    WeaponSage::MotorReversed_S motor_reversed_; 



protected:

    M3508 *launch_Motor_ = nullptr; // 抬升电机1，主电机
    M2006 *claw_1_Motor_ = nullptr; // 夹爪电机1，负责武器的夹取动作
    M2006 *claw_2_Motor_ = nullptr; // 夹爪电机2，负责武器的夹取动作
    M2006 *claw_3_Motor_ = nullptr; // 夹爪电机3，负责武器的夹取动作
    M2006 *wrist_Motor_ = nullptr; // 手腕电机，负责武器的手腕动作
    DM_Motor *arm_Motor_ = nullptr; // 机械臂电机，负责武器的机械臂动作

    WeaponSage::WeaponSage_Pos_S target_pos_;
    WeaponSage::WeaponSage_Pos_S current_pos_;
	WeaponSage::WeaponSage_Pos_S last_pos_;

    /**
     * @brief 将实际位置转换为电机总角度
     * @param real_pos 实际位置
     * @param motor_type 电机类型
     */
    float Realpos_to_MotorTotalAngle(float real_pos, WeaponSage::Motor_Type_E motor_type);

    float MotorTotalAngle_to_Realpos(float motor_angle, WeaponSage::Motor_Type_E motor_type);

    bool setMotorTargetTotalAngle(float total_angle, WeaponSage::Motor_Type_E motor_type);

	WeaponSage_InitData_S initData_;
};


#endif // __cplusplus


#endif // WEAPONSAGE_H
