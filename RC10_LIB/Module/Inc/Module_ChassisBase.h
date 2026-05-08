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
  / ____/  / /_     ____ _   _____   _____   (_)  _____   / __ )
 / /      / __ \   / __ `/  / ___/  / ___/  / /  / ___/  / __  |
/ /___   / / / /  / /_/ /  (__  )  (__  )  / /  (__  )  / /_/ /
\____/  /_/ /_/   \__,_/  /____/  /____/  /_/  /____/  /_____/
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
#endif

#ifdef __cplusplus

typedef enum{
    CURRENT_ZERO_MODE,
    SPEED_ZERO_MODE,
    ROBOT_SPEED_MODE,
    WORLD_SPEED_MODE
} CHASSIS_CONTROL_MODE_E;

template <std::size_t WheelCount>
class Chassis_Base{
public:
    Chassis_Base(float wheel_radius, float max_wheel_rpm);
    ~Chassis_Base();

    void set_ControlMode(CHASSIS_CONTROL_MODE_E mode) { ctrl_mode_ = mode; }
    void set_Target(const Robot_Twist& target)
    {
        switch (ctrl_mode_)
        {
            case CURRENT_ZERO_MODE:
                for (std::size_t i = 0; i < WheelCount; ++i)
                {
                    if (wheels_[i] != nullptr)
                    {
                        wheels_[i]->setTargetCurrent(0.0f);
                    }
                }
                break;

            case SPEED_ZERO_MODE:
                for (std::size_t i = 0; i < WheelCount; ++i)
                {
                    if (wheels_[i] != nullptr)
                    {
                        wheels_[i]->setTargetRPM(0.0f);
                    }
                }
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

    void update();
    virtual void updateKinematics() = 0;

    void updateAngleData(const Angle_Twist& angle_twist) { angle_twist_ = angle_twist; }

    float getWheelTargetRPM(uint8_t wheel_index) const
    {
        if (wheel_index >= WheelCount)
        {
            return 0.0f;
        }
        return wheel_target_rpm_[wheel_index];
    }

    Robot_Twist getRobotSpeed() const { return robot_twist_; }
    Robot_Twist getWorldSpeed() const { return world_twist_; }
    float getdt() const { return dt_; }

    bool registerWheelMotor(uint8_t wheel_index, Motor_Base* motor)
    {
        if (wheel_index >= WheelCount)
        {
            return false;
        }
        wheels_[wheel_index] = motor;
        return true;
    }

    void reset_AccLimitStatus(bool reset) { accel_Limit_ = reset; }
    void reset_AccValue(float reset) { accel_value_ = reset; }

private:
    void setRobotSpeed(const Robot_Twist& twist);
    void setWorldSpeed(const Robot_Twist& twist);

protected:
    Robot_Twist robot_twist_ = {0};
    Robot_Twist world_twist_ = {0};

    Robot_Twist robot_twist_forward = {0};

    Robot_Twist robot_target_twist_ = {0};
    Robot_Twist world_target_twist_ = {0};

    Angle_Twist angle_twist_ = {0};

    virtual void inverseKinematics(const Robot_Twist& twist) = 0;
    virtual void forwardKinematics(){};

    bool accel_Limit_ = false;
    float accel_value_ = 0.0f;

    const float wheel_radius_;
    const float max_wheel_rpm_;

    float last_update_time_s_ = 0.0f;

    float wheel_target_rpm_[WheelCount] = {0};
    Motor_Base* wheels_[WheelCount] = {nullptr};
    float dt_ = 0.0f;

    float wheelSpeedToMotorRPM(float wheel_speed)
    {
        return (wheel_speed / (2 * PI * wheel_radius_)) * 60.0f;
    }

    CHASSIS_CONTROL_MODE_E ctrl_mode_ = CURRENT_ZERO_MODE;
};

#endif

#endif
