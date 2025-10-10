#include "Module_ChassisBase.h"

template<std::size_t WheelCount>
Chassis_Base<WheelCount>::Chassis_Base(float wheel_radius, float max_wheel_rpm)
             : wheel_radius_(wheel_radius),
               max_wheel_rpm_(max_wheel_rpm),
               max_wheel_speed_(wheel_radius * 2.0f * PI * (max_wheel_rpm / 60.0f))
{
    last_update_time_s_ = TimeStamp::getInstance().getSeconds();
}

template<std::size_t WheelCount>
void Chassis_Base<WheelCount>::setRobotSpeed(const Robot_Twist& twist)
{
    robot_target_twist_ = twist;
}

template<std::size_t WheelCount>
void Chassis_Base<WheelCount>::setWorldSpeed(const Robot_Twist& twist)
{
    world_target_twist_ = twist;

    // 使用yaw角将世界系速度转换到机器人坐标系
    float cos_yaw = arm_cos_f32(-deg_to_rad(angle_twist_.yaw_angle));
    float sin_yaw = arm_sin_f32(-deg_to_rad(angle_twist_.yaw_angle));

    robot_target_twist_.vx = twist.vx * cos_yaw - twist.vy * sin_yaw;
    robot_target_twist_.vy = twist.vx * sin_yaw + twist.vy * cos_yaw;
    robot_target_twist_.yaw_rate = twist.yaw_rate; // 角速度在2D平面上不变
}

template<std::size_t WheelCount>
void Chassis_Base<WheelCount>::update()
{
    //update time stamp
    float current_time_s = TimeStamp::getInstance().getSeconds();
    dt_ = current_time_s - last_update_time_s_;
    if(dt_ <= 0.0f || dt_ > 0.1f)
        dt_ = 0.001f; //异常值处理

    last_update_time_s_ = current_time_s;

    if(accel_Limit_)
    {
        ramp(robot_target_twist_.vx, robot_twist_.vx, accel_value_, dt_);
        ramp(robot_target_twist_.vy, robot_twist_.vy, accel_value_, dt_);

        robot_twist_.yaw_rate = robot_target_twist_.yaw_rate; // 角速度不做加速度限制
    }
    else
        robot_twist_ = robot_target_twist_;

    updateKinematics(); //调用逆解和正解，更新轮速和机器人速度

    // 更新世界坐标系速度
    float cos_yaw = arm_cos_f32(deg_to_rad(angle_twist_.yaw_angle));
    float sin_yaw = arm_sin_f32(deg_to_rad(angle_twist_.yaw_angle));
    world_twist_.vx = robot_twist_.vx * cos_yaw - robot_twist_.vy * sin_yaw;
    world_twist_.vy = robot_twist_.vx * sin_yaw + robot_twist_.vy * cos_yaw;
    world_twist_.yaw_rate = robot_twist_.yaw_rate;

    for(std::size_t i = 0; i < WheelCount; i++)
    {
        constrain(wheele_target_rpm_[i], -max_wheel_rpm_, max_wheel_rpm_); //限幅
        if(wheels_[i] != nullptr)
            wheels_[i]->setTargetRPM(wheele_target_rpm_[i]);
    }
}