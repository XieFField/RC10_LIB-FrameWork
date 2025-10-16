#include "Module_ChassisOmni.h"

//�����
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
        // ����������ȫ���ֵ����ݲ�֧��
        return;
    }
}

template <std::size_t WheelCount>
void Chassis_Omni<WheelCount>::updateKinematics()
{
    inverseKinematics(this->robot_twist_);
    forwardKinematics(this->robot_twist_);
}
template <std::size_t WheelCount>
void Chassis_Omni<WheelCount>::forwardKinematics(const Robot_Twist& twist)
{
    
}
template <std::size_t WheelCount>
Chassis_Omni<WheelCount>::Chassis_Omni(float wheel_radius, float max_wheel_rpm, float chassis_radius_)
    : Chassis_Base<WheelCount>(wheel_radius, max_wheel_rpm),
      chassis_radius_(chassis_radius_)
{
}

