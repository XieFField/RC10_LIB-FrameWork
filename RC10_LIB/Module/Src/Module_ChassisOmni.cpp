#include "Module_ChassisOmni.h"

//逆解算
template <std::size_t WheelCount>
void Chassis_Omni<WheelCount>::inverseKinematics(const Robot_Twist& twist)
{
    if constexpr (WheelCount == 3)
    {
        this->wheel_target_rpm_[0] = this->wheelSpeedToMotorRPM(- twist.vx + twist.yaw_rate * chassis_radius_);
        this->wheel_target_rpm_[1] = this->wheelSpeedToMotorRPM(twist.vy * COS_31_87 + twist.vx * SIN_31_87 + twist.yaw_rate * chassis_radius_bottom_);
        this->wheel_target_rpm_[2] = this->wheelSpeedToMotorRPM(-twist.vy * COS_31_87 + twist.vx * SIN_31_87 + twist.yaw_rate * chassis_radius_bottom_);
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
    // 若任意轮超出最大转速，所有轮等比缩小（保持方向与比例）
    float max_abs = 0.0f;
    for (uint8_t i = 0; i < WheelCount; ++i) 
    {
        float a = fabsf(this->wheel_target_rpm_[i]);
        if (a > max_abs) max_abs = a;
    }
    if (max_abs > this->max_wheel_rpm_) 
    {
        float k = this->max_wheel_rpm_ / max_abs;
        for (uint8_t i = 0; i < WheelCount; ++i) this->wheel_target_rpm_[i] *= k;
    }
    forwardKinematics();
}

template <std::size_t WheelCount>
Chassis_Omni<WheelCount>::Chassis_Omni(float wheel_radius, float max_wheel_rpm, float chassis_radius)
    : Chassis_Base<WheelCount>(wheel_radius, max_wheel_rpm),
      chassis_radius_(chassis_radius)
{

}

// 新增：三轮等腰三角形构造，传入底边与腰长，自动计算两个旋转半径
template <>
Chassis_Omni<3>::Chassis_Omni(float wheel_radius, float max_wheel_rpm, float base_length, float side_length, bool three_wheel)
    : Chassis_Base<3>(wheel_radius, max_wheel_rpm)
{
    if(!three_wheel)
        return;
    float top_r = 0.f, bottom_r = 0.f;
    computeIsoscelesRadii(base_length, side_length, top_r, bottom_r);
    chassis_radius_ = top_r;
    chassis_radius_bottom_ = bottom_r;
}

template <std::size_t WheelCount>
void Chassis_Omni<WheelCount>::computeIsoscelesRadii(float base_length, float side_length, float& top_radius, float& bottom_radius)
{
    // 等腰三角形：底边 base，腰 side。高度 h = sqrt(side^2 - (base/2)^2)
    // 取旋转中心为三角形重心：顶点到重心距离 = 2/3 h，底边顶点到重心距离 = sqrt((base/2)^2 + (h/3)^2)
    if (base_length <= 0.f || side_length <= 0.f) { top_radius = 0.f; bottom_radius = 0.f; return; }
    float half_b = 0.5f * base_length;
    float h_sq = side_length * side_length - half_b * half_b;
    if (h_sq <= 0.f) { top_radius = 0.f; bottom_radius = half_b; return; }
    float h = sqrtf(h_sq);
    top_radius = (2.0f/3.0f) * h;
    float one_third_h = h / 3.0f;
    bottom_radius = sqrtf(half_b * half_b + one_third_h * one_third_h);
}

template<std::size_t WheelCount>
void Chassis_Omni<WheelCount>::forwardKinematics()
{
    float wheel_speeds[WheelCount];
    for (uint8_t i = 0; i < WheelCount; ++i) 
        wheel_speeds[i] = this->getWheelTargetRPM(i)*2.0f*PI/60.0f*this->wheel_radius_; // 转换为线速度 (m/s)
    
    if constexpr (WheelCount == 3) 
    {
        this->robot_twist_forward.vy = (wheel_speeds[1] - wheel_speeds[2]) / (2.0f*COS_31_87);
        this->robot_twist_forward.yaw_rate = (wheel_speeds[1]/2 + wheel_speeds[2]/2 + wheel_speeds[0]*SIN_31_87) / (SIN_31_87 * chassis_radius_+chassis_radius_bottom_);
        this->robot_twist_forward.vx = this->robot_twist_forward.yaw_rate * chassis_radius_ - wheel_speeds[0];
    } 
    else if constexpr (WheelCount == 4) 
    {
        // 四轮全向底盘的前向运动学计算
        this->robot_twist_forward.yaw_rate = (wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3]) / (4.0f * chassis_radius_);
        this->robot_twist_forward.vy = (-wheel_speeds[0] - wheel_speeds[1] + wheel_speeds[2]+ wheel_speeds[3]) / (2.0f*1.41421356f);
        this->robot_twist_forward.vx = (wheel_speeds[0] - wheel_speeds[1] - wheel_speeds[2] + wheel_speeds[3]) / (2.0f*1.41421356f);
    }
}

template class Chassis_Omni<4>;
template class Chassis_Omni<3>;
