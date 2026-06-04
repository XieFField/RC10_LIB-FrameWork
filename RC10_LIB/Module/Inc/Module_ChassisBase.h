/**
 * @file    Module_ChassisBase.h
 * @author  XieFField
 * @brief   底盘模块基类
 *          这是一个纯粹的运动学模型
 *          - 注册动力电机，应用速度到电机
 *          - 坐标系：遵循右手定则，yaw逆时针为正。
 * @version 1.0
 */

/*
   ______   __                                _            ____                         
  / ____/  / /_     ____ _   _____   _____   (_)  _____   / __ )   ____ _   _____   ___ 
 / /      / __ \   / __ `/  / ___/  / ___/  / /  / ___/  / __  |  / __ `/  / ___/  / _ \
/ /___   / / / /  / /_/ /  (__  )  (__  )  / /  (__  )  / /_/ /  / /_/ /  (__  )  /  __/
\____/  /_/ /_/   \__,_/  /____/  /____/  /_/  /____/  /_____/   \__,_/  /____/   \___/ 
                                                                                        
*/

#ifndef __MODULE_CHASSISBASE_H
#define __MODULE_CHASSISBASE_H

#ifdef __cplusplus
extern "C" {
#endif
#include "arm_math.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include <cstdint>
#include "APP_tool.h"
#include "APP_CoordConvert.h"
#include "Motor_Base.h"
#include "BSP_TimeStamp.h"
#endif// __cplusplus

#ifdef __cplusplus


typedef enum{
    CURRENT_ZERO_MODE, //电流置零
    SPEED_ZERO_MODE,   //速度置零 驻车模式
    ROBOT_SPEED_MODE,  //机器人速度模式
    WORLD_SPEED_MODE   //世界速度模式
}CHASSIS_CONTROL_MODE_E;

/**
 * @brief 底盘模块基类
 * @details
 *          后续任何底盘类都需继承此类。         
 */
template <std::size_t WheelCount>
class Chassis_Base{
public:
    Chassis_Base(float wheel_radius, float max_wheel_rpm);
    ~Chassis_Base();

    void set_ControlMode(CHASSIS_CONTROL_MODE_E mode) { ctrl_mode_ = mode; } // 设置底盘控制模式
    void set_Target(const Robot_Twist& target) // 设置底盘目标速度
    {
        switch (ctrl_mode_)
        {
            case CURRENT_ZERO_MODE:
                /* code */
                this->wheels_[0]->setTargetCurrent(0);
                this->wheels_[1]->setTargetCurrent(0);
                this->wheels_[2]->setTargetCurrent(0);
                if(WheelCount ==4)
                    this->wheels_[3]->setTargetCurrent(0);
                break;
            
            case SPEED_ZERO_MODE:
                this->wheels_[0]->setTargetRPM(0);
                this->wheels_[1]->setTargetRPM(0);
                this->wheels_[2]->setTargetRPM(0);
                if(WheelCount ==4)
                    this->wheels_[3]->setTargetRPM(0);
                break;

            case ROBOT_SPEED_MODE:
                setRobotSpeed(target);
                break;
            
            case WORLD_SPEED_MODE:
                setWorldSpeed(target);
                break;
            default:
                break;
        }
    }




    void update(); // 更新轮速应用到电机

    virtual void updateKinematics() = 0; // 更新运动学，调用逆解


    void updateAngleData(const Angle_Twist& angle_twist) { angle_twist_ = angle_twist; } // 更新角速度数据

    float getWheelTargetRPM(uint8_t wheel_index) const
    {
        if(wheel_index >= WheelCount)
            return 0.0f;
        return wheel_target_rpm_[wheel_index];
    }

    Robot_Twist getRobotSpeed_() const { return robot_twist_; } // 获取机器人速度（机器人坐标系）
    Robot_Twist getWorldSpeed_() const { return world_twist_; } // 获取机器人速度（世界坐标系）
    float getdt() const { return dt_; } // 获取时间差
 
    bool registerWheelMotor(uint8_t wheel_index, Motor_Base* motor) // 注册轮子电机
    {
        if(wheel_index >= WheelCount) 
            return false;
        wheels_[wheel_index] = motor;
        return true;
    }

    void reset_AccLimitStatus(bool reset) { accel_Limit_ = reset; } // 重置底盘线加速度限幅器
    void reset_AccValue(float reset) {accel_value_ = reset;}; // 重置底盘线加速度值
private:
    void setRobotSpeed(const Robot_Twist& twist); // 设置机器人速度（机器人坐标系）

    void setWorldSpeed(const Robot_Twist& twist); // 设置世界速度（世界坐标系）
protected:
    Robot_Twist robot_twist_ = {0}; // 机器人坐标系当前速度
    Robot_Twist world_twist_ = {0}; // 世界坐标系当前速度

    Robot_Twist robot_twist_forward = {0}; //正解算得到的机器人坐标系速度
    Robot_Twist world_twist_forward = {0}; //正解算得到的世界坐标系速度

    Robot_Twist robot_target_twist_ = {0}; // 机器人坐标系目标速度
    Robot_Twist world_target_twist_ = {0}; // 世界坐标系目标速度

    Angle_Twist angle_twist_ = {0}; // 从传感器得到的角速度、角度数据

    virtual void inverseKinematics(const Robot_Twist& twist) = 0; // 逆解，根据目标速度计算轮速

    virtual void forwardKinematics(){};

    bool accel_Limit_ = false; // 是否启用加速度限幅
    float accel_value_ = 0.0f; // 当前线加速度值

    const float wheel_radius_;    // 轮子半径 (m)
    const float max_wheel_rpm_;   // 轮子最大RPM
    const float max_wheel_speed_; // 轮子最大线速度 (m/s)

    // 时间戳，用于加速度斜坡
    float last_update_time_s_ = 0.0f; //单位：秒

    float wheel_target_rpm_[WheelCount] = {0}; // 存储逆解算出的各轮目标RPM

    Motor_Base* wheels_[WheelCount] = {nullptr}; // 轮子电机指针数组
    float dt_ = 0.0f; //更新时间差

    /**
     * @brief 把轮子线速度转换为电机转轴转速
     */

    float wheelSpeedToMotorRPM(float wheel_speed)
    {
        return (wheel_speed / (2 * PI * wheel_radius_)) * 60.0f;
    }

    CHASSIS_CONTROL_MODE_E ctrl_mode_ = CURRENT_ZERO_MODE; // 底盘控制模式
};

#endif // __cplusplus

#endif // __MODULE_CHASSISBASE_H