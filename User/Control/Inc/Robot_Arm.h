/**
 * @file Robot_Arm.h
 * @author XieFField
 * @brief 串联刚体臂吸盘运动建模.      
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


/**
 * @brief 一切单位都是米和度
 */
typedef struct {
    float max_launchHeight_; // 升降最大行程，单位米 0
    float max_stretchLength_; // 伸展最大行程，单位米
    float arm_length_; // 机械臂长度
    float end_link_length_; // 末端连杆长度，吸盘到机械臂连接点的距离，单位米

    

    float stretch_Ratio_; // 伸展比率，伸展电机转一圈，伸展多少米   0.0942米(94.2mm)
    float launch_Ratio_; // 升降比率，升降电机转一圈，升降多少米    0.01099米(109.9mm)
    float rotate_gearRatio_; // 旋转减速比，旋转电机转一圈，机械臂转多少度 144.878度()
    float pitch_gearRatio_; // 俯仰减速比，俯仰电机转一圈，末端关节转多少度 360度，直驱

    float min_rotate_angle_; // 最小旋转角度
    float max_rotate_angle_; // 最大旋转角度

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
    float suckerJoint_angle_; // 末端关节状态
}Joint_Status_S;

typedef enum{
    TARGET_POSITION_MODE, // 目标位置模式
    MANUAL_MOTOR_POSITION_MODE, // 手动电机位置模式
    MANUAL_JOINT_SPEED_MODE, // 手动关节速度模式
    CURRENT_CONTROL_MODE // 电流控制模式 依旧使用Joint_Status_S存储目标电流值
}Arm_Control_mode_E;

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

/** 
 * @brief 又变成四自由度了，好，那么好。
 * @note 这里的坐标或者行程单位都是米，角度单位是度，角度制。
 */
class Robot_Arm {

protected:
    float now_time_s_ = 0; 
    Arm_InitData_S init_data_;
    DJI_Motor* motor_launch_ = nullptr; // 升降电机
    DJI_Motor* motor_stretch_ = nullptr; // 伸展电机
    DJI_Motor* motor_rotate_ = nullptr; // 旋转电机

    DJI_Motor* motor_pitch_ = nullptr; // 末端关节俯仰电机

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

    /**
     * @brief 设置机械臂控制模式
     */
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

    /** 
     * @brief 设置手动末端速度速度
     */
    void setManualSpeed(float vx, float vy, float vz) 
    {
        manual_v_.vx = vx; manual_v_.vy = vy; manual_v_.vz = vz;
    }



    void setJacobianInitData(Jacobian_InitData_S jac_init_data)
    {
        jac_init_data_ = jac_init_data;
    }

    
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
        target_joint_angle_.rotateJoint_angle_ = angle;
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





    // Jacobian 速度控制用目标末端速度（m/s）
    Robot_Twist manual_v_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    void jacobianMatrix(); // 雅可比矩阵计算

    Jacobian_InitData_S jac_init_data_;


    // 时间戳（秒）
    float last_time_s_ = 0.0f;
    float dt_ = 0.0f;
    bool  time_initialized_ = false;

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

    MotorReversed_S sign_reversed_  = {1.0f, 1.0f, 1.0f, 1.0f};

};


#endif // __cplusplus

#endif // __ROBOT_ARM_H