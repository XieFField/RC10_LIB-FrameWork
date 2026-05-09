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
/* 南北路多 */
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
 * @brief 一切单位都是米和度
 */
typedef struct {
    float max_launchHeight_ = 0.0f; // 升降最大行程，单位米 0
    float max_stretchLength_ = 0.0f; // 伸展最大行程，单位米
    float arm_length_ = 0.0f; // 机械臂长度
    float end_link_length_ = 0.0f; // 末端连杆长度，吸盘到机械臂连接点的距离，单位米
    float max_pitchRPM_ = 50.0f; // 末端关节最大转速，单位RPM   
    

    float stretch_Ratio_ = 0.0f; // 伸展比率，伸展电机转一圈，伸展多少米   0.0942米(94.2mm)
    float launch_Ratio_ = 0.0f; // 升降比率，升降电机转一圈，升降多少米    0.01099米(109.9mm)
    float rotate_gearRatio_ = 0.0f; // 旋转减速比，旋转电机转一圈，机械臂转多少度 144.878度()   电机转222.289627度，机械臂转90度。  新矫正145.755789度
    float pitch_gearRatio_ = 0.0f; // 俯仰减速比，俯仰电机转一圈，末端关节转多少度 360度，直驱

    float min_rotate_angle_ = 0.0f; // 最小旋转角度
    float max_rotate_angle_ = 0.0f; // 最大旋转角度
    float safe_height_ = 0.0f; // 安全高度，单位米，低于这个高度，机械臂云台旋转受限
    float store_height_ = 0.0f; // 存储高度，单位米，机械臂在这个高度进行存储和取出操作
    GPIO_TypeDef * Sucker_GPIO_Port; // 吸盘控制GPIO端口
    uint16_t Sucker_GPIO_Pin;      // 吸盘控制GPIO引脚
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

typedef struct{
    float launchJoint_Height_; // 升降关节状态
    float stretchJoint_Length_; // 伸展关节状态
    float rotateJoint_angle_; // 旋转关节状态
    float suckerJoint_angle_; // 末端关节状态 朝下为0度
}Joint_Status_S;

typedef enum{
    TARGET_POSITION_MODE, // 目标位置模式
    MANUAL_MOTOR_POSITION_MODE, // 手动电机位置模式
    CURRENT_CONTROL_MODE // 电流控制模式 依旧使用Joint_Status_S存储目标电流值
}Arm_Control_mode_E;

//rotate 旋转路径枚举
typedef enum {
    ROTATE_PATH_SHORTEST,   // 最短路径
    ROTATE_PATH_POSITIVE,   // 正向路径
    ROTATE_PATH_NEGATIVE    // 负向路径
}Rotate_Strategy_E;

typedef struct{
    float launch_current; //升降电机电流
    float stretch_current; //伸展电机电流
    float rotate_current; //旋转电机电流
    float pitch_current; //末端关节电机电流
}ARMotor_Current_S;

typedef struct{
    // 阻尼系数 λ
    float jac_lambda_ = 0.02f;

    // 限幅
    float vmax_xy_ = 0.60f;  // 末端 XY 平面最大线速 m/s
    float vmax_z_  = 0.60f;  // 末端 Z 轴最大线速 m/s
    float hdot_max_ = 0.80f;             // 升降关节最大速度 m/s
    float ddot_max_ = 0.80f;             // 伸展关节最大速度 m/s
    float thetadot_deg_max_ = 90.0f;     // 旋转关节最大角速度 deg/s
}Jacobian_InitData_S;

typedef struct{
    float sign_launch_ = 1.0f;
    float sign_stretch_ = 1.0f;
    float sign_rotate_ = 1.0f;
    float sign_pitch_ = 1.0f;
}MotorReversed_S;

typedef struct{
    float ramp__maxspeed_ = 0.0f;
    float max_accel_ = 0.0f; // 加速度限制 (Motor Angle deg/s^2)，值越小起步/刹车越平滑
    float current_velocity_ = 0.0f; // 记录当前速度
    float ramp_target_ = 0.0f; 
    float filter_k_ = 0.0f; 
}Fliter_Ramp_S;

/** 
 * @brief 又变成四自由度了，好，那么好。
 * @note 这里的坐标或者行程单位都是米，角度单位是度，角度制。
 */
class Robot_Arm {

protected:
    float now_time_s_ = 0.0f; 
    Arm_InitData_S init_data_;
    DJI_Motor* motor_launch_ = nullptr; // 升降电机
    DJI_Motor* motor_stretch_ = nullptr; // 伸展电机
    DJI_Motor* motor_rotate_ = nullptr; // 旋转电机

    DM_Motor* motor_pitch_ = nullptr; // 末端关节俯仰电机
    bool is_pitchEnable_ = false; //是否使能电机

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

    float get_dt(){return this->dt_;}

    /**
     * @brief 设置机械臂控制模式
     */
    void set_controlMode(Arm_Control_mode_E mode)
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

    // 设置目标位置
    void setArmTarget(Arm_Point_S target){ arm_target_ = target; }
    Arm_Point_S getArmTarget() const { return arm_target_; }

    void setSuckerStatus(Sucker_Status_E status){ sucker_status_ = status; }


    Sucker_Status_E getSuckerStatus() const { return sucker_status_; }

    /**
     * @brief 设置旋转路径策略
     */
    void setRotateStrategy(Rotate_Strategy_E strategy){ rotate_strategy_ = strategy; }

    Rotate_Strategy_E getRotateStrategy() const { return rotate_strategy_; }

    float calc_rotate_targetByStrategy(float current_cont_angle, float target_raw_0_360);

    /**
     * @brief 手动设置每个自由度的目标位置，单位：m或度
     */

    void set_LaunchHeight(float height)
    {
        target_joint_angle_.launchJoint_Height_ = height;
    }

    /**
     * @brief 手动设置每个自由度的目标位置，单位：m或度
     */
    void set_StretchLength(float length)
    {
        target_joint_angle_.stretchJoint_Length_ = length;
    }

    /**
     * @brief 手动设置每个自由度的目标位置，单位：m或度
     */
    void set_RotateAngle(float angle)
    {
        target_joint_angle_.rotateJoint_angle_ = normalize_deg_0_360(angle);
    }

    /**
     * @brief 手动设置每个自由度的目标位置，单位：m或度
     */

    void set_PitchAngle(float angle)
    {
        target_joint_angle_.suckerJoint_angle_ = angle;
    }


    //设置电机是否反相 true取反，false不取反
    void setLaunchReversed(bool reversed) {sign_reversed_.sign_launch_ = reversed ? -1.0f : 1.0f;}
    void setStretchReversed(bool reversed) {sign_reversed_.sign_stretch_ = reversed ? -1.0f : 1.0f;}
    void setRotateReversed(bool reversed) {sign_reversed_.sign_rotate_  = reversed ? -1.0f : 1.0f;}
    void setPitchReversed(bool reversed) {sign_reversed_.sign_pitch_  = reversed ? -1.0f : 1.0f;}

    Joint_Status_S get_currentJointStatus() const { return joint_angle_; }
    Joint_Status_S get_targetJointStatus() const { return target_joint_angle_; }

private:    
    
    Rotate_Strategy_E rotate_strategy_ = ROTATE_PATH_SHORTEST; // 旋转路径策略，默认最短路径
    Joint_Status_S joint_angle_ = {0.0f, 0.0f, 0.0f, 0.0f}; 

    Joint_Status_S target_joint_angle_ = {0.0f, 0.0f, 0.0f, 0.0f}; 
    
    void inverseKinematics(Arm_Point_S arm_target_); // 运动学逆解



    // 正运动学：计算末端位姿（基于当前目标或电机角度）
    // 返回 true 表示计算成功，结果写入 out
    bool forwardKinematics(Arm_Point_S& out) const;

    Arm_Point_S arm_target_ = {0.0f, 0.0f, 0.0f, 0.0f}; // 机械臂末端目标位置
    Arm_Point_S arm_ = {0.0f, 0.0f, 0.0f, 0.0f}; // 机械臂关节末端当前位置

    Sucker_Status_E sucker_status_ = Sucker_Status_E::STOP; // 吸盘状态

    Arm_Control_mode_E control_mode_ = TARGET_POSITION_MODE; // 机械臂控制模式

    Arm_Point_S arm_forward_ = {0.0f, 0.0f, 0.0f, 0.0f}; // 机械臂正运动学计算位置





    // 时间戳（秒）
    float last_time_s_ = 0.0f;
    float dt_ = 0.0f;
    bool  time_initialized_ = false;

protected:
/*================================================================*/
    /*关节角度->电机总角度*/
    float launchHeight_to_MotorTotalAngle(float height)
    {
        return sign_reversed_.sign_launch_ * height / init_data_.launch_Ratio_ * 360.0f;
    }

    float stretchLength_to_MotorTotalAngle(float length)
    {
        return sign_reversed_.sign_stretch_ * length / init_data_.stretch_Ratio_ * 360.0f;
    }

    float rotateAngle_to_MotorTotalAngle(float angle)
    {
        return sign_reversed_.sign_rotate_ * angle / init_data_.rotate_gearRatio_ * 360.0f;
    }

    float pitchAngle_to_MotorTotalAngle(float angle)
    {
        return sign_reversed_.sign_pitch_ * angle / init_data_.pitch_gearRatio_ * 360.0f;
    }
    
/*=================================================================*/
    /*电机总角度->关节角度*/
    float MotorTotalAngle_to_launchHeight(float motor_angle)
    {
        return sign_reversed_.sign_launch_ * motor_angle * init_data_.launch_Ratio_ / 360.0f;
    }

    float MotorTotalAngle_to_stretchLength(float motor_angle)
    {
        return sign_reversed_.sign_stretch_ * motor_angle * init_data_.stretch_Ratio_ / 360.0f;
    }

    float MotorTotalAngle_to_rotateAngle(float motor_angle)
    {
        return sign_reversed_.sign_rotate_ * motor_angle * init_data_.rotate_gearRatio_ / 360.0f;
    }

    float MotorTotalAngle_to_pitchAngle(float motor_angle)
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
    float ramped_rotateMotorAngle_ = 0.0f;
    
//bool  time_initialized_ = false;
    
    Fliter_Ramp_S rotate_fliter_ramp_ = {
        .ramp__maxspeed_ = 36000.0f,
        .max_accel_ = 80000.0f, // 加速度限制 (Motor Angle deg/s^2)，值越小起步/刹车越平滑
        .current_velocity_ = 0.0f, // 记录当前速度
        .ramp_target_ = 0.0f, 
        .filter_k_ = 200.0f // 滤波(平滑)系数，值越大到达目标越快，越小刹车越平滑
    };
    
    Fliter_Ramp_S strech_fliter_ramp_ = {
        .ramp__maxspeed_ = 600000.0f,
        .max_accel_ = 1000000.0f, // 伸展加速度限制 (Motor Angle deg/s^2)
        .current_velocity_ = 0.0f, // 记录当前伸展速度
        .ramp_target_ = 0.0f, 
        .filter_k_ = 450.0f // 伸展滤波(平滑)系数，值越大到达目标越快，越小刹车越平滑
    };

    
    float caculate_ramp_target(float current, float target, Fliter_Ramp_S &ramp)
    {
        float diff = target - current;
        
        // 期望目标速度 (一次平滑 P控制)
        float target_vel = diff * ramp.filter_k_;

        // 速度限幅
        if (target_vel > ramp.ramp__maxspeed_) target_vel = ramp.ramp__maxspeed_;
        if (target_vel < -ramp.ramp__maxspeed_) target_vel = -ramp.ramp__maxspeed_;

        // 加速度限幅
        float max_dv = ramp.max_accel_ * dt_;
        if (target_vel > ramp.current_velocity_ + max_dv) {
            ramp.current_velocity_ += max_dv;
        } else if (target_vel < ramp.current_velocity_ - max_dv) {
            ramp.current_velocity_ -= max_dv;
        } else {
            ramp.current_velocity_ = target_vel;
        }

        // 计算步长
        float step = ramp.current_velocity_ * dt_;

        // 如果距离和速度都极小，直接赋值防止浮点数计算震荡
        if(std::abs(diff) < 0.01f && std::abs(ramp.current_velocity_) < 0.1f) 
        {
            ramp.current_velocity_ = 0.0f; // 完全停稳后清零速度
            return target;
        }

        return current + step;
    }
};


#endif // __cplusplus

#endif // __ROBOT_ARM_H