/**
 * @file Robot_Arm.h
 * @author XieFField
 * @brief 串联刚体臂吸盘运动建模.
 *        目前是纯手动模型，后期再改进为半自动
 */

#ifndef __ROBOT_ARM_H
#define __ROBOT_ARM_H

#pragma once

#ifdef __cplusplus
extern "C" {
#endif
    #include "arm_math.h"
#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
#include <cstdint>
#include "Motor_Base.h"
#include "Motor_DJI.h"
#include "APP_tool.h"

/**
 * @brief 一切单位都是米和度
 */
typedef struct {
    float max_launchHeight_; // 升降最大行程，单位米
    float max_stretchLength_; // 伸展最大行程，单位米
    float arm_length_; // 机械臂长度

    float stretch_Ratio_; // 伸展比率，伸展电机转一圈，伸展多少米
    float launch_Ratio_; // 升降比率，升降电机转一圈，升降多少米
    float rotate_gearRatio_; // 旋转减速比，旋转电机转一圈，机械臂转多少度
    float pitch_gearRatio_; // 俯仰减速比，俯仰电机转一圈，末端关节转多少度
}Arm_InitData_S;

typedef enum {
    Flat_Status,//平放
    Place_Status//侧放
}SuckerJoint_E;

typedef enum {
    SUCK, //吸
    STOP  //停
}Sucker_Status_E;

typedef struct{
    float x; //末端关节坐标
    float y;
    float z;

    float suckerJoint_status_ ; // 末端关节状态
}Arm_Point_S;

typedef enum{
    TARGET_POSITION_MODE, // 目标位置模式
    MANUAL_MODE // 手动模式
}Arm_Control_mode_E;

/** 
 * @brief 又变成四自由度了，好，那么好。
 * @note 这里的坐标或者行程单位都是米，角度单位是度，角度制。
 */
class Robot_Arm {
public:
    

    Robot_Arm(Arm_InitData_S init_Data);
    ~Robot_Arm(){}

    /**
     * @brief 更新目标位置、速度到电机
     *        更新当前的launch_height_、stretch_length_、
     *        rotate_angle_，这个主要是用于调试
     * 
     */
    void update();

    void set_controlMode(Arm_Control_mode_E mode){ control_mode_ = mode; }

    void registerMotor_Launch(DJI_Motor* motor){ motor_launch_ = motor; }
    void registerMotor_Stretch(DJI_Motor* motor){ motor_stretch_ = motor; }
    void registerMotor_Rotate(DJI_Motor* motor){ motor_rotate_ = motor; }
    void registerMotor_Pitch(DJI_Motor* motor){ motor_pitch_ = motor; }

    // 设置目标位置
    void setArmTarget(Arm_Point_S target){ arm_target_ = target; }
    Arm_Point_S getArmTarget() const { return arm_target_; }

    void setSuckerStatus(Sucker_Status_E status){ sucker_status_ = status; }
    Sucker_Status_E getSuckerStatus() const { return sucker_status_; }

    
    // 正运动学：计算末端位姿（基于当前目标或电机角度）
    // 返回 true 表示计算成功，结果写入 out
    bool forwardKinematics(Arm_Point_S& out) const;
    // 设置末端连杆长度 Ls（单位 米），用于计算末端吸盘位置
    void setEndLinkLength(float Ls) { end_link_length_ = Ls; }


private:
    Arm_InitData_S init_data_;

    DJI_Motor* motor_launch_ = nullptr; // 升降电机
    DJI_Motor* motor_stretch_ = nullptr; // 伸展电机
    DJI_Motor* motor_rotate_ = nullptr; // 旋转电机

    DJI_Motor* motor_pitch_ = nullptr; // 末端关节俯仰电机

    float launchHeight_to_MotorTotalAngle(float height)
    {
        return height / init_data_.launch_Ratio_ * 360.0f;
    }

    float stretchLength_to_MotorTotalAngle(float length)
    {
        return length / init_data_.stretch_Ratio_ * 360.0f;
    }

    float rotateAngle_to_MotorTotalAngle(float angle)
    {
        return angle / init_data_.rotate_gearRatio_ * 360.0f;
    }

    float pitchAngle_to_MotorTotalAngle(float angle)
    {
        return angle / init_data_.pitch_gearRatio_ * 360.0f;
    }

/*=================================================================*/

    float MotorTotalAngle_to_launchHeight(float motor_angle)
    {
        return motor_angle * init_data_.launch_Ratio_ / 360.0f;
    }

    float MotorTotalAngle_to_stretchLength(float motor_angle)
    {
        return motor_angle * init_data_.stretch_Ratio_ / 360.0f;
    }

    float MotorTotalAngle_to_rotateAngle(float motor_angle)
    {
        return motor_angle * init_data_.rotate_gearRatio_ / 360.0f;
    }

    float MotorTotalAngle_to_pitchAngle(float motor_angle)
    {
        return motor_angle * init_data_.pitch_gearRatio_ / 360.0f;
    }


    float motorlaunch_height_ = 0.0f; // 当前升降高度
    float motorstretch_length_ = 0.0f; // 当前伸展长度
    float motorrotate_angle_ = 0.0f; // 当前旋转角度
    float motorpitch_angle_ = 0.0f; // 当前末端关节角度

    float target_launch_height_ = 0.0f; // 目标升降高度
    float target_stretch_length_ = 0.0f; // 目标伸展长度
    float target_rotate_angle_ = 0.0f; // 目标旋转角度
    float target_pitch_angle_ = 0.0f; // 目标末端关节角度
    float end_link_length_ = 0.0f;   // 吸盘刚体长臂长，单位米

    void inverseKinematics(Arm_Point_S arm_target_); // 运动学逆解
  

    const float minRotateAngle_ = 0.0f; // 旋转最小角度
    const float maxRotateAngle_ = 180.0f; // 旋转最大角度

    Arm_Point_S arm_target_ = {0.0f, 0.0f, 0.0f, SuckerJoint_E::Place_Status}; // 机械臂末端目标位置
    Arm_Point_S arm_ = {0.0f, 0.0f, 0.0f, SuckerJoint_E::Place_Status}; // 机械臂关节末端当前位置

    Sucker_Status_E sucker_status_ = Sucker_Status_E::STOP; // 吸盘状态

    Arm_Control_mode_E control_mode_ = TARGET_POSITION_MODE; // 机械臂控制模式
};


#endif // __cplusplus

#endif // __ROBOT_ARM_H