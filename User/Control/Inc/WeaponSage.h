/**
 * @file WeaponSage.h
 * @author XieFField
 * @brief 武器大师驱动层
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

//一切转动都以逆时针为正方向
typedef struct 
{
    /* data */
    float max_launchHeight_; // 升降最大行程，单位米
    float max_clawAngle_; // 抓取最大角度，单位度
    float max_traverseLength_; // 横移最大行程，单位米

    float wrist_gearRatio_; // 手腕减速比，手腕电机转一圈，手腕转多少度(360度意味着直驱)
    float launch_Ratio_; // 升降比率，升降电机转一圈，升降多少米
    float claw_gearRatio_; // 抓取减速比，抓取电机转一圈，抓取多少度
    float traverse_Ratio_; // 横移比率，横移电机转一圈，横移多少米

    float max_wristMotorRPM_; // 手腕电机最大转速，单位RPM

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
        float launch_reversed_ = 1.0f;
        float claw_reversed_ = 1.0f;
        float traverse_reversed_ = 1.0f;
        float wrist_reversed_ = 1.0f;
    }MotorReversed_S;

    enum WeaponSage_CtrlMode_S 
    {
        /* data */
        CURRENT_CONTROL,
        Join_POSITION_CONTROL,
        TOTAL_ANGLE_CONTROL,
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
    
    /**
     * @brief 设置电机反相
     * @param reversed 是否反相 true反相，false不反相(默认不反相)
     * @param motor_type 电机类型
     */
    bool setMotorReversed(bool reversed, WeaponSage::Motor_Type_E motor_type);


    bool setTarget(float targetValue, WeaponSage::Motor_Type_E motor_type);

    void setCtrlMode(WeaponSage::WeaponSage_CtrlMode_S mode)
    {
        ctrl_mode_ = mode;
    }
private:

    WeaponSage::WeaponSage_CtrlMode_S ctrl_mode_ = WeaponSage::Join_POSITION_CONTROL;
    WeaponSage_InitData_S initData_;

    WeaponSage::MotorReversed_S motor_reversed_; 



protected:

    M3508 *launch_Motor_ = nullptr; // 升降电机
    M2006 *claw_Motor_ = nullptr; // 抓取电机
    M2006 *traverse_Motor_ = nullptr; // 横移电机
    DM_Motor *wrist_Motor_ = nullptr; // 手腕电机


    WeaponSage::WeaponSage_Pos_S target_pos_;
    WeaponSage::WeaponSage_Pos_S current_pos_;

    /**
     * @brief 真实位置转换为电机总角度
     * @param real_pos 真实位置，单位米或度，具体见initData_说明
     * @param motor_type 电机类型
     */
    float Realpos_to_MotorTotalAngle(float real_pos, WeaponSage::Motor_Type_E motor_type);

    float MotorTotalAngle_to_Realpos(float motor_angle, WeaponSage::Motor_Type_E motor_type);

    bool setMotorTargetTotalAngle(float total_angle, WeaponSage::Motor_Type_E motor_type);
};


#endif // __cplusplus


#endif // WEAPONSAGE_H
