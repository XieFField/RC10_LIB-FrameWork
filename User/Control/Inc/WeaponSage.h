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
#include "BSP_TimeStamp.h"

//初始化数据结构体
typedef struct 
{
    /* data */
    float max_launchHeight_; // 最大抬升高度
    float max_clawAngle_; // 最大夹爪角度
    float max_traverseLength_; // 最大 traverse 长度

    float wrist_gearRatio_; //手腕减速比，手腕电机转一圈，末端关节转多少度 360度，直驱
    float launch_Ratio_; // 抬升减速比，抬升电机转一圈，末端关节移动多少米
    float claw_gearRatio_; // 夹爪减速比，夹爪电机转一圈，末端关节移动多少米
    float traverse_Ratio_; // traverse减速比，traverse电机转一圈，末端关节移动多少米
    float max_wristMotorRPM_; // 最大手腕电机转速

}WeaponSage_InitData_S;

namespace WeaponSage
{
    enum Motor_Type_E
    {
        Launch_1_Motor, //仅在配置电机反相时候传入有用
        Launch_2_Motor, //仅在配置电机反相时候传入有用

        Launch_Motor, // 抬升电机

        Claw_Motor,

        Traverse_Motor,
        Wrist_Motor
    };

    typedef struct 
    {
        float launch_1_master_reversed_ = -1.0f;
        float launch_2_master_reversed_ = -1.0f;

        float launch_1_slave_reversed_ = -1.0f;
        float launch_2_slave_reversed_ = -1.0f;

        float claw_reversed_ = -1.0f;
        float traverse_reversed_ = 1.0f;
        float wrist_reversed_ = -1.0f;
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
        float launch_1_pos_; //主要供调试时候使用，实际控制以launch_TotalAngle_为准，单位米
        float launch_2_pos_;    
        float launch_pos_; // 抬升位置，单位米, 控制时候两边连轴电机共享，正解算以1_master电机为准
        float claw_pos_;
        float traverse_pos_;
        float wrist_pos_;

        float launch_TotalAngle_;
        float launch_1_TotalAngle_;
        float launch_2_TotalAngle_;

        float claw_TotalAngle_;
        float traverse_TotalAngle_;
        float wrist_TotalAngle_;
    }WeaponSage_Pos_S;

};

class Robot_WeaponSage {

public:
    Robot_WeaponSage(WeaponSage_InitData_S init_data);
    ~Robot_WeaponSage(){}

    bool register_launch_Motor_1(M3508* motor_master, M3508* motor_slave)
    { 
        launch_Motor_1_master = motor_master; 
        launch_Motor_1_slave = motor_slave;
        if(launch_Motor_1_master != nullptr && launch_Motor_1_slave != nullptr)
            return true; 
        else
            return false;
    }

    bool register_launch_Motor_2(M3508* motor_master, M3508* motor_slave)
    { 
        launch_Motor_2_master = motor_master; 
        launch_Motor_2_slave = motor_slave;
        if(launch_Motor_2_master != nullptr && launch_Motor_2_slave != nullptr)
            return true; 
        else
            return false;
    }

    bool register_claw_Motor(M2006* motor)
    { 
        claw_Motor_ = motor; 
        if(claw_Motor_ != nullptr)
            return true; 
        else
            return false;
    }

    bool register_traverse_Motor(M2006* motor)
    { 
        traverse_Motor_ = motor; 
        if(traverse_Motor_ != nullptr)
            return true; 
        else
            return false;
    }

    bool register_wrist_Motor(DM_Motor* motor)
    { 
        wrist_Motor_ = motor; 
        if(wrist_Motor_ != nullptr)
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
		current_pos.launch_1_pos_ = MotorTotalAngle_to_Realpos(launch_Motor_1_master->getTotalAngle(), WeaponSage::Launch_1_Motor);
		current_pos.launch_2_pos_ = MotorTotalAngle_to_Realpos(launch_Motor_2_master->getTotalAngle(), WeaponSage::Launch_2_Motor);
        current_pos.launch_pos_ = MotorTotalAngle_to_Realpos(launch_Motor_1_master->getTotalAngle(), WeaponSage::Launch_Motor); //以master电机为准
		current_pos.traverse_pos_ = MotorTotalAngle_to_Realpos(traverse_Motor_->getTotalAngle(),WeaponSage::Traverse_Motor);
		current_pos.claw_pos_ = MotorTotalAngle_to_Realpos(claw_Motor_->getTotalAngle(),WeaponSage::Claw_Motor);
		current_pos.wrist_pos_ = MotorTotalAngle_to_Realpos(wrist_Motor_->getTotalAngle(),WeaponSage::Wrist_Motor);
		return current_pos;
	}
	
	void Weapon_wrist_setzero(){wrist_Motor_->motorSetZero();}
	void Weapon_wrist_enable(){wrist_Motor_->motorEnable();}
    float get_launchVel()
    {

    }
private:

    WeaponSage::WeaponSage_CtrlMode_S ctrl_mode_ = WeaponSage::Join_POSITION_CONTROL;
    

    WeaponSage::MotorReversed_S motor_reversed_; 



protected:

    M3508 *launch_Motor_1_master = nullptr; // 抬升电机主电机1，负责武器的抬升动作
    M3508 *launch_Motor_1_slave = nullptr; // 抬升电机从电机2，负责武器的抬升动作

    M3508 *launch_Motor_2_master = nullptr; // 抬升电机主电机3，负责武器的抬升动作
    M3508 *launch_Motor_2_slave = nullptr; // 抬升电机从电机4，负责武器的抬升动作


    M2006 *claw_Motor_ = nullptr; // 夹爪电机，负责武器的夹取动作
    M2006 *traverse_Motor_ = nullptr; // traverse电机，负责武器的水平移动
    DM_Motor *wrist_Motor_ = nullptr; // 手腕电机，负责武器的手腕动作


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
    void set_launchMotorSpeed(float target)
    {
        launch_target_rpm_ = motor_reversed_.launch_1_master_reversed_ * target;
    }
    float launch_target_rpm_ = 0.0f;
	WeaponSage_InitData_S initData_;
};


#endif // __cplusplus


#endif // WEAPONSAGE_H
