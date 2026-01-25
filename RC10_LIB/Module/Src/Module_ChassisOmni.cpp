#include "Module_ChassisOmni.h"

//逆解算
template <std::size_t WheelCount>
void Chassis_Omni<WheelCount>::inverseKinematics(const Robot_Twist& twist)
{
    for (uint8_t i = 0; i < WheelCount; ++i)
    {
        this->wheel_target_rpm_[i] = this->wheelSpeedToMotorRPM(twist.vx * wheel_calculate_config_[i].cos_theta + twist.vy * wheel_calculate_config_[i].sin_theta + twist.yaw_rate * wheel_calculate_config_[i].radius);
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
Chassis_Omni<WheelCount>::Chassis_Omni(Chassis_Omni<WheelCount>::init_config& config)
    : Chassis_Base<WheelCount>(config.wheel_radius, config.max_wheel_rpm)
{
    for (uint8_t i = 0; i < WheelCount; ++i)
    {
        wheel_config_[i] = config.wheels[i];
        wheel_calculate_config_[i].cos_theta = cosf(config.wheels[i].theta * PI / 180.0f);
        wheel_calculate_config_[i].sin_theta = sinf(config.wheels[i].theta * PI / 180.0f);
        wheel_calculate_config_[i].radius = config.wheels[i].x * wheel_calculate_config_[i].sin_theta - config.wheels[i].y * wheel_calculate_config_[i].cos_theta;
    }
}

template <std::size_t WheelCount>
void Chassis_Omni<WheelCount>::computeIsoscelesRadii(float base_length, float side_length, float& top_radius, float& bottom_radius)
{
    // 等腰三角形：底边 base，腰 side。高度 h = sqrt(side^2 - (base/2)^2)
    // 旋转中心改为三角形内心（角平分线交点）。对于等腰三角形，内心坐标在 x=0，
    // y = (base * h) / (base + 2*side)。顶点到内心距离 = h - y = 2*side*h/(base + 2*side)
    if (base_length <= 0.f || side_length <= 0.f) { top_radius = 0.f; bottom_radius = 0.f; return; }
    float half_b = 0.5f * base_length;
    float h_sq = side_length * side_length - half_b * half_b;
    if (h_sq <= 0.f) { top_radius = 0.f; bottom_radius = half_b; return; }
    float h = sqrtf(h_sq);
    float denom = base_length + 2.0f * side_length;
    if (denom <= 0.f) { top_radius = 0.f; bottom_radius = half_b; return; }
    float incenter_y = (base_length * h) / denom; // 从基线到内心的垂直距离
    top_radius = h - incenter_y; // 顶点到内心距离
    bottom_radius = sqrtf(half_b * half_b + incenter_y * incenter_y); // 底边端点到内心距离
}

template<std::size_t WheelCount>
void Chassis_Omni<WheelCount>::forwardKinematics()
{
    float wheel_speeds[WheelCount];
    for (uint8_t i = 0; i < WheelCount; ++i) 
        wheel_speeds[i] = this->getWheelTargetRPM(i)*2.0f*PI/60.0f*this->wheel_radius_; // 转换为线速度 (m/s)
    
    if constexpr (WheelCount == 3) 
    {
        if(use_three_solver_==true)
        {
            // inverse uses: w1 = vy*COS - vx*SIN - y*Rb; w2 = -vy*COS - vx*SIN - y*Rb
            // therefore forward (invert): vy = (w1 - w2) / (2*COS)
            this->robot_twist_forward.vy = (wheel_speeds[1] - wheel_speeds[2]) / (2.0f * COS_31_87);

            // derive yaw_rate from combination:
            // 0.5*(w1+w2) = -vx*SIN - y*Rb ; w0 = vx - y*Rt
            // solving gives: y = -(w0*SIN + 0.5*(w1+w2)) / (Rt*SIN + Rb)
            float num = wheel_speeds[0] * SIN_31_87 + 0.5f * (wheel_speeds[1] + wheel_speeds[2]);
            float den = chassis_radius_ * SIN_31_87 + chassis_radius_bottom_;
            this->robot_twist_forward.yaw_rate = - num / den;

            // vx = w0 + omega * Rt
            this->robot_twist_forward.vx = wheel_speeds[0] + this->robot_twist_forward.yaw_rate * chassis_radius_;
        }
        else
        {
            this->robot_twist_forward.vy = (wheel_speeds[1] - wheel_speeds[2]) / (2.0f * COS_30);

            float num = wheel_speeds[0] * SIN_30 + 0.5f * (wheel_speeds[1] + wheel_speeds[2]);
            float den = chassis_radius_ * SIN_30 + chassis_radius_;
            this->robot_twist_forward.yaw_rate = - num / den;

            this->robot_twist_forward.vx = wheel_speeds[0] + this->robot_twist_forward.yaw_rate * chassis_radius_;
        }
    } 
    else if constexpr (WheelCount == 4) 
    {
        // 四轮全向底盘的前向运动学计算
        // Inverse uses vx/C, vy/C. Forward needs factor C/4. (Divide by 4/C = 4*sqrt(2) = 5.656)
        this->robot_twist_forward.yaw_rate = (wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3]) / (4.0f * chassis_radius_);
        
        // 4.0f * 1.414... = 5.6568
        this->robot_twist_forward.vy = (-wheel_speeds[0] - wheel_speeds[1] + wheel_speeds[2]+ wheel_speeds[3]) / (4.0f*1.41421356f);
        this->robot_twist_forward.vx = (wheel_speeds[0] - wheel_speeds[1] - wheel_speeds[2] + wheel_speeds[3]) / (4.0f*1.41421356f);
    }

    //this->world_twist_forward   
}

template class Chassis_Omni<4>;
template class Chassis_Omni<3>;
