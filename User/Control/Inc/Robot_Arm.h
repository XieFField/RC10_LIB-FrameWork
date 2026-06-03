/**
 * @file Robot_Arm.h
 * @author XieFField
 * @brief 串联刚体臂吸盘运动建模.      
 * @version 1.0
 * 完成基本控制功能
 * @version 2.0
 * 增加雅可比矩阵计算，支持手动关节速度模式
 * @version 3.0
 * 增加旋转路径策略支持
 * 
 * @attention 云台旋转路径，在逼近目标值的时候，手动切换为最短路径策略，避免大幅度超调
 *            反正写和用的人都是我自己，怎么方便怎么来，懒得封装了
 * 
 * @attention 上方提到的已解决
 */

#ifndef __ROBOT_ARM_H
#define __ROBOT_ARM_H
/* 锟较憋拷路锟斤拷 */
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
#include "BSP_TimeStamp.h"
#include "Module_GPIO.h"
#include "Module_GPIO.h"
#include "Motor_DM.h"

/**
 * @brief
 */
typedef struct {
    float max_launchHeight_ = 0.0f; // 升降最大行程，单位米 0
    float max_stretchLength_ = 0.0f; // 伸展最大行程，单位米
    float arm_length_ = 0.0f; // 机械臂长度
    float end_link_length_ = 0.0f; // 末端连杆长度，吸盘到机械臂连接点的距离，单位米
    float max_pitchRPM_ = 50.0f; // 末端关节最大转速，单位RPM   
    float rotate_end = 265.0f;
    float rotate_start = 135.0f;
    float store_ext_length_ =0.0f; //存储时候伸展长度


    float stretch_Ratio_ = 0.0f; // 伸展比率，伸展电机转一圈，伸展多少米   0.0942米(94.2mm)
    float launch_Ratio_ = 0.0f; // 升降比率，升降电机转一圈，升降多少米    0.01099米(109.9mm)
    float rotate_gearRatio_ = 0.0f; // 旋转减速比，旋转电机转一圈，机械臂转多少度 115.179f
    float pitch_gearRatio_ = 0.0f; // 俯仰减速比，俯仰电机转一圈，末端关节转多少度 360度，直驱
    float min_rotate_angle_ = 0.0f; // 最小旋转角度
    float max_rotate_angle_ = 0.0f; // 最大旋转角度
    float safe_height_ = 0.0f; // 安全高度
    float store_height_ = 0.0f; // 储存高度
    float lock_height_ = 0.0f; // 云台锁定高度
    float pick_up_height_ =0.0f; //拾取高度
    float putdown_height_ = 0.0f; //放置高度


    GPIO_TypeDef * Sucker_GPIO_Port; // 吸盘GPIO控制端口
    uint16_t Sucker_GPIO_Pin;      // 吸盘GPIO控制引脚

    GPIO_TypeDef* Store_GPIO_Port; // 储存位GPIO控制端口
    uint16_t Store_GPIO_Pin;      // 储存位GPIO控制引脚

    GPIO_TypeDef* Sucker_Soleniod_GPIO_Port; // 吸盘电磁阀GPIO控制端口
    uint16_t Sucker_Soleniod_GPIO_Pin;      // 吸盘电磁阀GPIO控制引脚

    GPIO_TypeDef* Store_Soleniod_GPIO_Port; // 储存位电磁阀GPIO控制端口
    uint16_t Store_Soleniod_GPIO_Pin;      // 储存位电磁阀GPIO控制引脚
}Arm_InitData_S;

typedef enum {
    SUCK, // 吸附
    STOP  // 停止
}Sucker_Status_E;

typedef struct{
    float x; // 末端关节坐标
    float y;
    float z;

    float suckerJoint_status_ ; // 末锟剿关斤拷状态
}Arm_Point_S;

typedef struct{
    float launchJoint_Height_; // 升降高度 米
    float stretchJoint_Length_; // 伸展长度 米
    float rotateJoint_angle_; // 旋转角度 度
    float suckerJoint_angle_; // 吸盘角度 度
}Joint_Status_S;

typedef enum{
    TARGET_POSITION_MODE, // 目标位置模式
    MANUAL_MOTOR_POSITION_MODE, // 手动电机位置模式
    CURRENT_CONTROL_MODE // 电流控制模式 用于存储目标位置
}Arm_control_mode_E;

//rotate 旋转路径策略枚举
typedef enum {
    ROTATE_PATH_SHORTEST,   // 最短路径
    ROTATE_PATH_POSITIVE,   // 正向路径
    ROTATE_PATH_NEGATIVE    // 负向路径
}Rotate_Strategy_E;

typedef struct{
    float launch_current; // 升降电流
    float stretch_current; // 伸展电流
    float rotate_current; // 旋转电流
    float pitch_current; // 末锟剿关节电流
}ARMotor_Current_S;

typedef struct{
    // 雅可比矩阵参数
    float jac_lambda_ = 0.02f;

    // 速度限制
    float vmax_xy_ = 0.60f;  // 末锟斤拷 XY 平锟斤拷锟斤拷锟斤拷锟 m/s
    float vmax_z_  = 0.60f;  // 末锟斤拷 Z 锟斤拷锟斤拷锟斤拷锟斤拷锟 m/s
    float hdot_max_ = 0.80f;             // 锟斤拷锟斤拷锟截斤拷锟斤拷锟斤拷俣锟 m/s
    float ddot_max_ = 0.80f;             // 锟斤拷展锟截斤拷锟斤拷锟斤拷俣锟 m/s
    float thetadot_deg_max_ = 90.0f;     // 锟斤拷转锟截斤拷锟斤拷锟斤拷锟劫讹拷 deg/s
}Jacobian_InitData_S;

typedef struct{
    float sign_launch_ = 1.0f;
    float sign_stretch_ = 1.0f;
    float sign_rotate_ = 1.0f;
    float sign_pitch_ = 1.0f;
}MotorReversed_S;

typedef struct{
    float ramp__maxspeed_ = 0.0f;
    float max_accel_ = 0.0f; // 最大加速度 (Motor Angle deg/s^2)
    float current_velocity_ = 0.0f; // 当前速度 (Motor Angle deg/s)
    float ramp_target_ = 0.0f; 
    float filter_k_ = 0.0f; 
}Fliter_Ramp_S;

/** 
 * @brief Robot_Arm类，负责机械臂的运动控制和状态管理
 * @note 该类负责机械臂的运动控制和状态管理，包括位置控制、速度控制和力控制等功能
 */
class Robot_Arm {

protected:
    float now_time_s_ = 0.0f; 
    Arm_InitData_S init_data_;
    DJI_Motor* motor_launch_ = nullptr; // 升降电机
    DJI_Motor* motor_stretch_ = nullptr; // 伸展电机
    DJI_Motor* motor_rotate_ = nullptr; // 旋转电机

    DM_Motor* motor_pitch_ = nullptr; // 末锟剿关节电机
    bool is_pitchEnable_ = false; // 是否使能俯仰电机

public:
    


    Robot_Arm(Arm_InitData_S init_Data);
    ~Robot_Arm(){}

    /**
     * @brief 控制状态更新
     * 
     */
    void update();

    float get_dt(){return this->dt_;}

    /**
     * @brief 锟斤拷锟矫伙拷械锟桔匡拷锟斤拷模式
     */
    void set_controlMode(Arm_control_mode_E mode)
    {
        if(control_mode_ != mode)
        {
            last_rotate_cmd_ = joint_angle_.rotateJoint_angle_;
        }
        control_mode_ = mode;
    }

    void registerMotor_Launch(DJI_Motor* motor){ motor_launch_ = motor; }
    void registerMotor_Stretch(DJI_Motor* motor){ motor_stretch_ = motor; }
    void registerMotor_Rotate(DJI_Motor* motor){ motor_rotate_ = motor; }
    void registerMotor_Pitch(DM_Motor* motor){ motor_pitch_ = motor; }

    // 锟斤拷锟斤拷目锟斤拷位锟斤拷
    void setArmTarget(Arm_Point_S target){ arm_target_ = target; }
    Arm_Point_S getArmTarget() const { return arm_target_; }

    void setSuckerStatus(Sucker_Status_E status){ sucker_status_ = status; }

    void setStoreSuckerStatus(Sucker_Status_E status){ store_sucker_status_ = status; }

    Sucker_Status_E getSuckerStatus() const { return sucker_status_; }

    Sucker_Status_E getStoreSuckerStatus() const { return store_sucker_status_; }

    /**
     * @brief 旋转角度合法化，确保旋转角度在合理范围内，避免超出机械限制
      * @param current_0_360 当前旋转角度，范围0~360度
     */
    float calc_legal_rotate_target(float current_0_360, float target_0_360);

    /**
     * @brief 设置目标升降高度
     */

    void set_LaunchHeight(float height)
    {
        target_joint_angle_.launchJoint_Height_ = height;
    }

    /**
     * @brief 设置目标伸展长度
     */
    void set_StretchLength(float length)
    {
        target_joint_angle_.stretchJoint_Length_ = length;
    }

    /**
     * @brief 设置目标旋转角度
     */
    void set_RotateAngle(float angle)
    {
        target_joint_angle_.rotateJoint_angle_ = normalize_deg_0_360(angle);
    }

    /**
     * @brief 设置目标俯仰角度
     */

    void set_PitchAngle(float angle)
    {
        target_joint_angle_.suckerJoint_angle_ = angle;
    }


    //设置反相 true反相 false不反相
    void setLaunchReversed(bool reversed) {sign_reversed_.sign_launch_ = reversed ? -1.0f : 1.0f;}
    void setStretchReversed(bool reversed) {sign_reversed_.sign_stretch_ = reversed ? -1.0f : 1.0f;}
    void setRotateReversed(bool reversed) {sign_reversed_.sign_rotate_  = reversed ? -1.0f : 1.0f;}
    void setPitchReversed(bool reversed) {sign_reversed_.sign_pitch_  = reversed ? -1.0f : 1.0f;}

    Joint_Status_S get_currentJointStatus() const { return joint_angle_; }
    Joint_Status_S get_targetJointStatus() const { return target_joint_angle_; }

private:    
    
    Joint_Status_S joint_angle_ = {0.0f, 0.0f, 0.0f, 0.0f}; 

    Joint_Status_S target_joint_angle_ = {0.0f, 0.0f, 0.0f, 0.0f}; 
    
    void inverseKinematics(Arm_Point_S arm_target_); // 逆解算



    // 正解算
    bool forwardKinematics(Arm_Point_S& out) const;

    Arm_Point_S arm_target_ = {0.0f, 0.0f, 0.0f, 0.0f}; // 目标点
    Arm_Point_S arm_ = {0.0f, 0.0f, 0.0f, 0.0f}; // 当前点

    Sucker_Status_E sucker_status_ = Sucker_Status_E::STOP; // 吸附状态
    Sucker_Status_E store_sucker_status_ = Sucker_Status_E::STOP; // 储位吸附状态


    Arm_control_mode_E control_mode_ = TARGET_POSITION_MODE; // 机械臂控制模式

    Arm_Point_S arm_forward_ = {0.0f, 0.0f, 0.0f, 0.0f}; // 正解算得到的点




    float last_time_s_ = 0.0f;
    float dt_ = 0.0f;
    bool  time_initialized_ = false;

protected:
/*================================================================*/
    /*关节位置->电机角度*/
    constexpr inline float launchHeight_to_MotorTotalAngle(float height)
    {
        return sign_reversed_.sign_launch_ * height / init_data_.launch_Ratio_ * 360.0f;
    }

    constexpr inline float stretchLength_to_MotorTotalAngle(float length)
    {
        return sign_reversed_.sign_stretch_ * length / init_data_.stretch_Ratio_ * 360.0f;
    }

    constexpr inline float rotateAngle_to_MotorTotalAngle(float angle)
    {
        return sign_reversed_.sign_rotate_ * angle / init_data_.rotate_gearRatio_ * 360.0f;
    }

    constexpr inline float pitchAngle_to_MotorTotalAngle(float angle)
    {
        return sign_reversed_.sign_pitch_ * angle / init_data_.pitch_gearRatio_ * 360.0f;
    }
    
/*=================================================================*/
    /*电机角度->关节位置*/
    constexpr inline float MotorTotalAngle_to_launchHeight(float motor_angle)
    {
        return sign_reversed_.sign_launch_ * motor_angle * init_data_.launch_Ratio_ / 360.0f;
    }

    constexpr inline float MotorTotalAngle_to_stretchLength(float motor_angle)
    {
        return sign_reversed_.sign_stretch_ * motor_angle * init_data_.stretch_Ratio_ / 360.0f;
    }

    constexpr inline float MotorTotalAngle_to_rotateAngle(float motor_angle)
    {
        return sign_reversed_.sign_rotate_ * motor_angle * init_data_.rotate_gearRatio_ / 360.0f;
    }

    constexpr inline float MotorTotalAngle_to_pitchAngle(float motor_angle)
    {
        return sign_reversed_.sign_pitch_ * motor_angle * init_data_.pitch_gearRatio_ / 360.0f;
    }

    void setRotateFilterK(float k) { rotate_fliter_ramp_.filter_k_ = k; }
    void setRampRotateMaxSpeed(float maxspeed) { rotate_fliter_ramp_.ramp__maxspeed_ = maxspeed; }

    void setStrechFilterK(float k) { strech_fliter_ramp_.filter_k_ = k; }
    void setRampStrechMaxSpeed(float maxspeed) { strech_fliter_ramp_.ramp__maxspeed_ = maxspeed; }

private:
    MotorReversed_S sign_reversed_  = {1.0f, 1.0f, 1.0f, 1.0f};
    float last_rotate_cmd_ = 0.0f;
    Rotate_Strategy_E rotate_strategy_ = ROTATE_PATH_SHORTEST;
    float prev_rotate_target_ = 0.0f;
    float ramped_rotateMotorAngle_ = 0.0f;
    float prev_norm_target_ = 0.0f;
    
//bool  time_initialized_ = false;
    
    Fliter_Ramp_S rotate_fliter_ramp_ = {
        .ramp__maxspeed_ = 36000.0f,
        .max_accel_ = 80000.0f, //  (Motor Angle deg/s^2)
        .current_velocity_ = 0.0f, // 当前旋转电机速度 (Motor Angle deg/s)
        .ramp_target_ = 0.0f, 
        .filter_k_ = 200.0f // 越小越平滑 越大越快
    };
    
    Fliter_Ramp_S strech_fliter_ramp_ = {
        .ramp__maxspeed_ = 600000.0f,
        .max_accel_ = 1000000.0f, // 展开速度限制 (Motor Angle deg/s^2)
        .current_velocity_ = 0.0f, // 记录当前展开速度
        .ramp_target_ = 0.0f, 
        .filter_k_ = 450.0f // 展开滤波器(平滑)系数，值越大响应越快，越小越平滑
    };

    Fliter_Ramp_S launch_fliter_ramp_ = {
        .ramp__maxspeed_ = 1500000.0f,
        .max_accel_ = 3000000.0f, //  (Motor Angle deg/s^2)
        .current_velocity_ = 0.0f, // 记录当前 launch 速度
        .ramp_target_ = 0.0f, 
        .filter_k_ = 850.0f // launch 滤波器(平滑)系数，值越大响应越快，越小越平滑
    };

    
    constexpr inline float caculate_ramp_target(float current, float target, Fliter_Ramp_S &ramp)
    {
        float diff = target - current;
        
        float target_vel = diff * ramp.filter_k_;

        // 限制目标速度
        if (target_vel > ramp.ramp__maxspeed_) target_vel = ramp.ramp__maxspeed_;
        if (target_vel < -ramp.ramp__maxspeed_) target_vel = -ramp.ramp__maxspeed_;

        // 计算最大速度变化量
        float max_dv = ramp.max_accel_ * dt_;
        if (target_vel > ramp.current_velocity_ + max_dv) 
            ramp.current_velocity_ += max_dv;

        else if (target_vel < ramp.current_velocity_ - max_dv) 
            ramp.current_velocity_ -= max_dv;
            
        else 
            ramp.current_velocity_ = target_vel;

        // 计算步长
        float step = ramp.current_velocity_ * dt_;

        // 检查是否到达目标位置
        if(std::abs(diff) < 0.01f && std::abs(ramp.current_velocity_) < 0.1f) 
        {
            ramp.current_velocity_ = 0.0f; // 停止电机
            return target;
        }

        return current + step;
    }
};


#endif // __cplusplus

#endif // __ROBOT_ARM_H