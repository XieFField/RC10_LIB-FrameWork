/**
 * @file WeaponSage.h
 * @author XieFField
 * @brief weaponsage控制驱动类
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
#include "BSP_TimeStamp.h"

//初始化数据结构体
typedef struct 
{
    /* data */
    float max_launchHeight_; //最大升降高度
    float max_clawAngle_; // 最大夹爪角度
    float max_traverseLength_; // 最大平移长度

    float wrist_gearRatio_; // 手腕关节减速比
    float launch_Ratio_; // 升降关节减速比
    float claw_gearRatio_; // 夹爪关节减速比
    float traverse_Ratio_; // 平移关节减速比

    float max_wristMotorRPM_; // 手腕电机最大转速

}WeaponSage_InitData_S;

namespace WeaponSage
{
    enum Motor_Type_E
    {
        Launch_Motor,
        Claw_Motor,
        Traverse_Motor,
        Wrist_Motor
    };

    typedef struct 
    {
        float launch_reversed_ = -1.0f;
        float claw_reversed_ = -1.0f;
        float traverse_reversed_ = 1.0f;
        float wrist_reversed_ = 1.0f;
    }MotorReversed_S;

    enum WeaponSage_CtrlMode_S 
    {
        /* data */
        CURRENT_CONTROL, // 电流控制模式
        Join_POSITION_CONTROL, // 关节角度控制模式
        TOTAL_ANGLE_CONTROL,   // 总角度控制模式
    };
    
    typedef struct
    {
        float launch_pos_;
        float claw_pos_;
        float traverse_pos_;
        float wrist_pos_;

        float launch_TotalAngle_;
        float claw_TotalAngle_;
        float traverse_TotalAngle_;
        float wrist_TotalAngle_;
    }WeaponSage_Pos_S;

};

class Robot_WeaponSage {

public:
    Robot_WeaponSage(WeaponSage_InitData_S init_data);
    ~Robot_WeaponSage(){}

    bool register_launch_Motor(M3508* motor)
    { launch_Motor_ = motor; if(launch_Motor_ != nullptr)return true; }
    
    bool register_claw_Motor(M2006* motor)
    { claw_Motor_ = motor; if(claw_Motor_ != nullptr)return true; }

    bool register_traverse_Motor(M2006* motor)
    { traverse_Motor_ = motor; if(traverse_Motor_ != nullptr)return true; }

    bool register_wrist_Motor(DM_Motor* motor)
    { wrist_Motor_ = motor; if(wrist_Motor_ != nullptr)return true; }
    
	
	void update();
	
	
    /**
     * @brief 设置电机反转，反转后控制命令会取反，正向转动时输出正值，反向转动时输出负值
     * @param reversed 需要反转时设置为true，不反转时设置为false
     * @param motor_type 需要设置反转的电机类型
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
		current_pos.launch_pos_=MotorTotalAngle_to_Realpos(launch_Motor_->getTotalAngle(), WeaponSage::Launch_Motor);
		current_pos.traverse_pos_=MotorTotalAngle_to_Realpos(traverse_Motor_->getTotalAngle(),WeaponSage::Traverse_Motor);
		current_pos.claw_pos_=MotorTotalAngle_to_Realpos(claw_Motor_->getTotalAngle(),WeaponSage::Claw_Motor);
		current_pos.wrist_pos_=MotorTotalAngle_to_Realpos(wrist_Motor_->getTotalAngle(),WeaponSage::Wrist_Motor);
		return current_pos;
	}
	
	void Weapon_wrist_setzero(){wrist_Motor_->motorSetZero();}
	void Weapon_wrist_enable(){wrist_Motor_->motorEnable();}
private:

    WeaponSage::WeaponSage_CtrlMode_S ctrl_mode_ = WeaponSage::Join_POSITION_CONTROL;
    

    WeaponSage::MotorReversed_S motor_reversed_; 



protected:

    M3508 *launch_Motor_ = nullptr; // 升降电机
    M2006 *claw_Motor_ = nullptr; // 夹爪电机
    M2006 *traverse_Motor_ = nullptr; // 平移电机
    DM_Motor *wrist_Motor_ = nullptr; // 手腕电机
    WeaponSage::WeaponSage_Pos_S target_pos_;
    WeaponSage::WeaponSage_Pos_S current_pos_;
	WeaponSage::WeaponSage_Pos_S last_pos_;

    /**
     * @brief 将实际位置转换为电机总角度
     * @param real_pos 实际位置值
     * @param motor_type 电机类型
     */
    float Realpos_to_MotorTotalAngle(float real_pos, WeaponSage::Motor_Type_E motor_type);

    float MotorTotalAngle_to_Realpos(float motor_angle, WeaponSage::Motor_Type_E motor_type);

    bool setMotorTargetTotalAngle(float total_angle, WeaponSage::Motor_Type_E motor_type);
	WeaponSage_InitData_S initData_;
};


#endif // __cplusplus


#endif // WEAPONSAGE_H
