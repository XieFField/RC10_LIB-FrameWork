#include "Robot_Arm.h"

Robot_Arm::Robot_Arm(Arm_InitData_S init_Data)
    : init_data_(init_Data)
{
}

void Robot_Arm::update()
{
  inverseKinematics(arm_target_);

  // 机械臂位置更新
  
  float target_rotateMotorAngle = 0.0f;
  float target_stretchMotorAngle = 0.0f;
  float target_launchMotorAngle = 0.0f;

  target_rotateMotorAngle = rotateAngle_to_MotorTotalAngle(target_rotate_angle_);
  target_stretchMotorAngle = stretchLength_to_MotorTotalAngle(target_stretch_length_);
  target_launchMotorAngle = launchHeight_to_MotorTotalAngle(target_launch_height_);

  /*暂时不做斜坡处理*/

  motorlaunch_height_ = target_launch_height_;
  motorstretch_length_ = target_stretch_length_;
  motorrotate_angle_ = target_rotate_angle_;

  if(motor_rotate_ != nullptr)
      motor_rotate_->setTargetTotalAngle(motorrotate_angle_);

  if(motor_stretch_ != nullptr)
      motor_stretch_->setTargetTotalAngle(motorstretch_length_);

  if(motor_launch_ != nullptr)
      motor_launch_->setTargetTotalAngle(motorlaunch_height_);
}

void Robot_Arm::inverseKinematics(Arm_Point_S target_point)
{
  target_launch_height_ = target_point.z;
  target_stretch_length_ = sqrt(target_point.x * target_point.x + target_point.y * target_point.y) - init_data_.arm_length_;

  /*角度制 */
  target_rotate_angle_ = atan2(target_point.y, target_point.x) * 180.0f / 3.1415926f;
}