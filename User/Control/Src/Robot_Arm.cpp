#include "Robot_Arm.h"
#include <iostream>

Robot_Arm::Robot_Arm(Arm_InitData_S init_Data)
    : init_data_(init_Data)
{
}

void Robot_Arm::update()
{
    if(control_mode_ == TARGET_POSITION_MODE)
        inverseKinematics(arm_target_);
    
  // 机械臂位置更新
  
    float target_rotateMotorAngle = 0.0f;
    float target_stretchMotorAngle = 0.0f;
    float target_launchMotorAngle = 0.0f;
    float target_pitchMotorAngle = 0.0f;

    target_rotateMotorAngle = rotateAngle_to_MotorTotalAngle(target_rotate_angle_);
    target_stretchMotorAngle = stretchLength_to_MotorTotalAngle(target_stretch_length_);
    target_launchMotorAngle = launchHeight_to_MotorTotalAngle(target_launch_height_);
    target_pitchMotorAngle = pitchAngle_to_MotorTotalAngle(target_pitch_angle_);
    /*暂时不做斜坡处理*/

    motorlaunch_height_ = target_launchMotorAngle;
    motorstretch_length_ = target_stretchMotorAngle;
    motorrotate_angle_ = target_rotateMotorAngle;
    motorpitch_angle_ = target_pitchMotorAngle;

    if(motor_rotate_ != nullptr)
        motor_rotate_->setTargetTotalAngle(motorrotate_angle_);

    if(motor_stretch_ != nullptr)
        motor_stretch_->setTargetTotalAngle(motorstretch_length_);

    if(motor_launch_ != nullptr)
        motor_launch_->setTargetTotalAngle(motorlaunch_height_);

    if(motor_pitch_ != nullptr)
        motor_pitch_->setTargetTotalAngle(motorpitch_angle_);
}

void Robot_Arm::inverseKinematics(Arm_Point_S target_point)
{
    if (std::abs(target_point.x) < 1e-6 && std::abs(target_point.y) < 1e-6) 
        target_rotate_angle_ = 0.0f; // 处理奇异点
     
    else
        target_rotate_angle_ = atan2(target_point.y, target_point.x) * 180.0f / PI;
    
    target_launch_height_ = target_point.z;
    target_stretch_length_ = sqrt(target_point.x * target_point.x + target_point.y * target_point.y) - init_data_.arm_length_;

    /*角度制 */
    target_rotate_angle_ = atan2(target_point.y, target_point.x) * 180.0f / 3.1415926f;
    target_pitch_angle_ = target_point.suckerJoint_status_;
}


bool Robot_Arm::forwardKinematics(Arm_Point_S& out) const
{

    float L0 = init_data_.arm_length_;
    float theta = target_rotate_angle_ * 3.1415926f / 180.0f;
    float alpha = target_pitch_angle_ * 3.1415926f / 180.0f;

    float Ltot = L0 + target_stretch_length_;
    float pjx = Ltot * cosf(theta);
    float pjy = Ltot * sinf(theta);
    float pjz = target_launch_height_;

    // Rz(theta) * Ry(alpha) * [Ls,0,0]^T
    float v_x = end_link_length_ * cosf(alpha);
    float v_y = 0.0f;
    float v_z = -end_link_length_ * sinf(alpha);

    float vsx = cosf(theta) * v_x - sinf(theta) * v_y;
    float vsy = sinf(theta) * v_x + cosf(theta) * v_y;
    float vsz = v_z;

    out.x = pjx + vsx;
    out.y = pjy + vsy;
    out.z = pjz + vsz;
    out.suckerJoint_status_ = target_pitch_angle_;

    return true;
}


void test()
{
    // 1. 初始化机械臂参数
    Arm_InitData_S arm_params;
    arm_params.max_launchHeight_ = 1.0f;    // 最大升降高度 1米
    arm_params.max_stretchLength_ = 2.0f;   // 最大伸展长度 2米
    arm_params.arm_length_ = 0.5f;          // 机械臂基础长度 0.5米
    
    arm_params.stretch_Ratio_ = 0.01f;      // 伸展电机转一圈伸展0.01米
    arm_params.launch_Ratio_ = 0.005f;      // 升降电机转一圈升降0.005米  
    arm_params.rotate_gearRatio_ = 1.0f;    // 旋转电机转一圈机械臂转1度
    arm_params.pitch_gearRatio_ = 1.0f;     // 俯仰电机转一圈末端转1度

    // 2. 创建机械臂对象
    Robot_Arm my_arm(arm_params);
    
    // 3. 设置末端连杆长度（如果有的话）
    my_arm.setEndLinkLength(0.2f);  // 吸盘臂长0.2米

    // 4. 设置目标关节角度（方式一：通过设置目标位置，逆解会自动计算关节角度）
    Arm_Point_S target;
    target.x = 1.2f;                // 目标x坐标 1.2米
    target.y = 0.8f;                // 目标y坐标 0.8米  
    target.z = 0.5f;                // 目标z坐标 0.5米
    target.suckerJoint_status_ = 30.0f;  // 末端关节30度
    
    my_arm.setArmTarget(target);
    
    // 5. 更新机械臂状态（这会触发逆运动学计算）
    my_arm.update();

    // 6. 计算正运动学验证位置
    Arm_Point_S calculated_position;
    if (my_arm.forwardKinematics(calculated_position)) {
        std::cout << "正运动学计算结果:" << std::endl;
        std::cout << "X: " << calculated_position.x << " 米" << std::endl;
        std::cout << "Y: " << calculated_position.y << " 米" << std::endl; 
        std::cout << "Z: " << calculated_position.z << " 米" << std::endl;
        std::cout << "末端关节角度: " << calculated_position.suckerJoint_status_ << " 度" << std::endl;
}
}