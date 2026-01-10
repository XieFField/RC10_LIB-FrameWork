#include "Module_ChassisOmni.h"

//逆解算
template <std::size_t WheelCount>
void Chassis_Omni<WheelCount>::inverseKinematics(const Robot_Twist& twist)
{
    // 修正：由于电机或轮子安装方向导致平动反向，在此处对vx, vy取反。yaw_rate保持现状
    // float vx = -twist.vx;
    // float vy = -twist.vy;
    // float yaw_rate = twist.yaw_rate;

    if constexpr (WheelCount == 3)
    {
        if (use_three_solver_==true)
        {
           // 算法 A（默认）：原有实现，顶点使用 chassis_radius_，底边使用 chassis_radius_bottom_
            this->wheel_target_rpm_[0] = this->wheelSpeedToMotorRPM(twist.vx - twist.yaw_rate * chassis_radius_);
            this->wheel_target_rpm_[1] = this->wheelSpeedToMotorRPM(twist.vy * COS_31_87 - twist.vx * SIN_31_87 - twist.yaw_rate * chassis_radius_bottom_);
            this->wheel_target_rpm_[2] = this->wheelSpeedToMotorRPM(-twist.vy * COS_31_87 - twist.vx * SIN_31_87 - twist.yaw_rate * chassis_radius_bottom_);
        }
        else
        {
            this->wheel_target_rpm_[0] = this->wheelSpeedToMotorRPM(twist.vx - twist.yaw_rate * chassis_radius_);
            this->wheel_target_rpm_[1] = this->wheelSpeedToMotorRPM(twist.vy / COS_30 - twist.vx / SIN_30 - twist.yaw_rate * chassis_radius_);
            this->wheel_target_rpm_[2] = this->wheelSpeedToMotorRPM(-twist.vy / COS_30 - twist.vx / SIN_30 - twist.yaw_rate * chassis_radius_);
        }
    }
    else if constexpr (WheelCount == 4)
    {
        this->wheel_target_rpm_[0] = this->wheelSpeedToMotorRPM(twist.vx / COS_45 - twist.vy / SIN_45 + twist.yaw_rate * chassis_radius_);
        this->wheel_target_rpm_[1] = this->wheelSpeedToMotorRPM(-twist.vx / COS_45 - twist.vy / SIN_45 + twist.yaw_rate * chassis_radius_);
        this->wheel_target_rpm_[2] = this->wheelSpeedToMotorRPM(-twist.vx / COS_45 + twist.vy / SIN_45 + twist.yaw_rate * chassis_radius_);
        this->wheel_target_rpm_[3] = this->wheelSpeedToMotorRPM(twist.vx / COS_45 + twist.vy / SIN_45 + twist.yaw_rate * chassis_radius_);
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
        if(use_three_solver_==true)
        {
            // Inverse: vy/C, vx/S.  Forward: vy = (v1-v2)*C/2
            this->robot_twist_forward.vy = (wheel_speeds[1] - wheel_speeds[2]) * COS_31_87 / 2.0f;
            
            // Omega derived from New Inverse: -(v0 + 0.5(v1+v2)S) / (R1 + R2*S)
            float num = wheel_speeds[0] + (wheel_speeds[1] + wheel_speeds[2]) * 0.5f * SIN_31_87;
            float den = chassis_radius_ + chassis_radius_bottom_ * SIN_31_87;
            this->robot_twist_forward.yaw_rate = -num / den;

            // vx = v0 + omega * R1
            this->robot_twist_forward.vx = wheel_speeds[0] + this->robot_twist_forward.yaw_rate * chassis_radius_;
        }
        else
        {
            this->robot_twist_forward.vy = (wheel_speeds[1] - wheel_speeds[2]) * COS_30 / 2.0f;
            
            float num = wheel_speeds[0] + (wheel_speeds[1] + wheel_speeds[2]) * 0.5f * SIN_30;
            float den = chassis_radius_ + chassis_radius_ * SIN_30;
            this->robot_twist_forward.yaw_rate = -num / den;
            
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
