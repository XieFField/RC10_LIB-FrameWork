#include "Module_ChassisOmni.h"

//逆解算
template <std::size_t WheelCount>
void Chassis_Omni<WheelCount>::inverseKinematics(const Robot_Twist& twist)
{
    if constexpr (WheelCount == 3)
    {
        this->wheel_target_rpm_[0] = this->wheelSpeedToMotorRPM(twist.vx + twist.yaw_rate * chassis_radius_);
        this->wheel_target_rpm_[1] = this->wheelSpeedToMotorRPM(-twist.vx * SIN_30 - twist.vy * COS_30 + twist.yaw_rate * chassis_radius_);
        this->wheel_target_rpm_[2] = this->wheelSpeedToMotorRPM(-twist.vx * SIN_30 + twist.vy * COS_30 + twist.yaw_rate * chassis_radius_);
    }
    else if constexpr (WheelCount == 4)
    {
        this->wheel_target_rpm_[0] = this->wheelSpeedToMotorRPM(twist.vx * COS_45 - twist.vy * SIN_45 + twist.yaw_rate * chassis_radius_);
        this->wheel_target_rpm_[1] = this->wheelSpeedToMotorRPM(-twist.vx * COS_45 - twist.vy * SIN_45 + twist.yaw_rate * chassis_radius_);
        this->wheel_target_rpm_[2] = this->wheelSpeedToMotorRPM(-twist.vx * COS_45 + twist.vy * SIN_45 + twist.yaw_rate * chassis_radius_);
        this->wheel_target_rpm_[3] = this->wheelSpeedToMotorRPM(twist.vx * COS_45 + twist.vy * SIN_45 + twist.yaw_rate * chassis_radius_);
    }
    else
    {
        // 其他轮数的全向轮底盘暂不支持
        return;
    }
}


template <std::size_t WheelCount>
void Chassis_Omni<WheelCount>::updateKinematics()
{
    inverseKinematics(this->robot_twist_);
    forwardKinematics();
}

template <std::size_t WheelCount>
Chassis_Omni<WheelCount>::Chassis_Omni(float wheel_radius, float max_wheel_rpm, float chassis_radius)
    : Chassis_Base<WheelCount>(wheel_radius, max_wheel_rpm),
      chassis_radius_(chassis_radius)
{

}

template<std::size_t WheelCount>
void Chassis_Omni<WheelCount>::forwardKinematics()
{
    float wheel_speeds[WheelCount];
    for (uint8_t i = 0; i < WheelCount; ++i) 
        wheel_speeds[i] = this->getWheelTargetRPM(i)*2.0f*PI/60.0f*this->wheel_radius_; // 转换为线速度 (m/s)
    
    if constexpr (WheelCount == 3) 
    {
        // 三轮全向底盘的前向运动学计算
        this->robot_twist_forward.vx = (wheel_speeds[0] + wheel_speeds[1]*COS_30 - wheel_speeds[2]*COS_30) / 3.0f;
        this->robot_twist_forward.vy = (wheel_speeds[1]*SIN_30 + wheel_speeds[2]*SIN_30) / 3.0f;
        this->robot_twist_forward.yaw_rate = (wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2]) / (3.0f * chassis_radius_);
    } 
    else if constexpr (WheelCount == 4) 
    {
        // 四轮全向底盘的前向运动学计算
        this->robot_twist_forward.yaw_rate = (wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3]) / (4.0f * chassis_radius_);
        this->robot_twist_forward.vy = (-wheel_speeds[0] - wheel_speeds[1] + wheel_speeds[2]+ wheel_speeds[3]) / (2.0f*1.41421356f);
        this->robot_twist_forward.vx = (wheel_speeds[0] - wheel_speeds[1] - wheel_speeds[2] + wheel_speeds[3]) / (2.0f*1.41421356f);
    }
}

template class Chassis_Base<4>;
template class Chassis_Omni<4>;

