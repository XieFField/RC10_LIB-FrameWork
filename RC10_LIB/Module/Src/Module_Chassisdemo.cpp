#include "Module_Chassisdemo.h"

template <std::size_t WheelCount>
void Chassis_Demo<WheelCount>::inverseKinematics(const Robot_Twist& twist)
{
    // 设置输入向量 [vx; vy; yaw_rate]
    input_vector_[0] = twist.vx;
    input_vector_[1] = twist.vy;
    input_vector_[2] = twist.yaw_rate;

    // 计算输出向量 = 变换矩阵 * 输入向量
    arm_mat_mult_f32(&kinematics_matrix, &input_mat_, &output_mat_);

    // 将输出向量转换为轮速 (RPM)
    for (uint8_t i = 0; i < WheelCount; ++i) {
        this->wheel_target_rpm_[i] = this->wheelSpeedToMotorRPM(output_vector_[i]);
    }
}

template <std::size_t WheelCount>
void Chassis_Demo<WheelCount>::forwardKinematics(Robot_Twist& twist)
{
    float wheel_speeds[WheelCount];
    for (uint8_t i = 0; i < WheelCount; ++i) {
                             
        wheel_speeds[i] = this->getWheelTargetRPM(i)*2.0f*PI/60.0f*this->wheel_radius_; // 转换为线速度 (m/s)
    }
    if constexpr (WheelCount == 3) {
        // 三轮全向底盘的前向运动学计算
        twist.vx = (wheel_speeds[0] + wheel_speeds[1]*COS_30 - wheel_speeds[2]*COS_30) / 3.0f;
        twist.vy = (wheel_speeds[1]*SIN_30 + wheel_speeds[2]*SIN_30) / 3.0f;
        twist.yaw_rate = (wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2]) / (3.0f * chassis_radius_);
    } else if constexpr (WheelCount == 4) {
        // 四轮全向底盘的前向运动学计算
        twist.yaw_rate = (wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3]) / (4.0f * chassis_radius_);
        twist.vy = (-wheel_speeds[0] - wheel_speeds[1] + wheel_speeds[2]+ wheel_speeds[3]) / (2.0f*1.4142f);
        twist.vx = (wheel_speeds[0] - wheel_speeds[1] - wheel_speeds[2] + wheel_speeds[3]) / (2.0f*1.4142f);
    }
}

template <std::size_t WheelCount>
void Chassis_Demo<WheelCount>::updateKinematics()
{
    inverseKinematics(this->robot_twist_);
    robot_twist_foward = this->robot_twist_;
    forwardKinematics(this->robot_twist_foward);  
}

template <std::size_t WheelCount>
Chassis_Demo<WheelCount>::Chassis_Demo(float wheel_radius, float max_wheel_rpm, float chassis_radius)
    : Chassis_Base<WheelCount>(wheel_radius, max_wheel_rpm),
      chassis_radius_(chassis_radius)
{
    if constexpr (WheelCount == 3) {
        // 三轮全向底盘的变换矩阵 (3x3)
        kinematics_matrix_data_[0] = 1.0f; kinematics_matrix_data_[1] = 0.0f; kinematics_matrix_data_[2] = chassis_radius_;
        kinematics_matrix_data_[3] = -SIN_30; kinematics_matrix_data_[4] = -COS_30; kinematics_matrix_data_[5] = chassis_radius_;
        kinematics_matrix_data_[6] = -SIN_30; kinematics_matrix_data_[7] = COS_30; kinematics_matrix_data_[8] = chassis_radius_;
        
        arm_mat_init_f32(&kinematics_matrix, 3, 3, kinematics_matrix_data_);
    }
    else if constexpr (WheelCount == 4) {
        // 四轮差速底盘的变换矩阵 (4x3)
        kinematics_matrix_data_[0] = COS_45; kinematics_matrix_data_[1] = -SIN_45; kinematics_matrix_data_[2] = chassis_radius_;
        kinematics_matrix_data_[3] = -COS_45; kinematics_matrix_data_[4] = -SIN_45; kinematics_matrix_data_[5] = chassis_radius_;
        kinematics_matrix_data_[6] = -COS_45; kinematics_matrix_data_[7] = SIN_45; kinematics_matrix_data_[8] = chassis_radius_;
        kinematics_matrix_data_[9] = COS_45; kinematics_matrix_data_[10] = SIN_45; kinematics_matrix_data_[11] = chassis_radius_;
        
        arm_mat_init_f32(&kinematics_matrix, 4, 3, kinematics_matrix_data_);
    }
     // 初始化输入和输出向量
    arm_mat_init_f32(&input_mat_, 3, 1, input_vector_);
    arm_mat_init_f32(&output_mat_, WheelCount, 1, output_vector_);
}

// 显式实例化Chassis_Base模板类
//template class Chassis_Base<3>;
//template class Chassis_Base<4>;

// 显式实例化模板类
template class Chassis_Demo<3>;
template class Chassis_Demo<4>;

