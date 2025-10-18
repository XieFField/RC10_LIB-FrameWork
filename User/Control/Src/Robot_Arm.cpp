#include "Robot_Arm.h"

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
    target_launch_height_ = target_point.z;
    target_stretch_length_ = sqrt(target_point.x * target_point.x + target_point.y * target_point.y) - init_data_.arm_length_;

    /*角度制 */
    target_rotate_angle_ = atan2(target_point.y, target_point.x) * 180.0f / 3.1415926f;
    target_pitch_angle_ = target_point.suckerJoint_status_;
}

//虽然写了这么多,但是Ls=0，所以和上面的逆解等价
bool Robot_Arm::forwardKinematics(Arm_Point_S& out) const
{


    float d = target_stretch_length_;
    float h = target_launch_height_;
    float theta_deg = target_rotate_angle_;
    float alpha_deg = target_pitch_angle_;

    float L0 = init_data_.arm_length_;
    float Ls = 0.0f; //吸盘刚体长臂长,可以通过end_link_length_更改
#ifdef __cplusplus
    // 如果类中有 end_link_length_ 成员则使用，否则默认为0
#endif

    float theta = theta_deg * 3.1415926f / 180.0f;
    float alpha = alpha_deg * 3.1415926f / 180.0f;

    float Ltot = L0 + d;
    float pjx = Ltot * cosf(theta);
    float pjy = Ltot * sinf(theta);
    float pjz = h;

    // Rz(theta) * Ry(alpha) * [Ls,0,0]^T
    // Ry(alpha) * [Ls,0,0]^T = [Ls*cosα, 0, -Ls*sinα]^T
    float v_x = end_link_length_ * cosf(alpha);
    float v_y = 0.0f;
    float v_z = -end_link_length_ * sinf(alpha);

    // Rz(theta) * v = [ cosθ -sinθ 0; sinθ cosθ 0; 0 0 1 ] * v
    float vsx = cosf(theta) * v_x - sinf(theta) * v_y;
    float vsy = sinf(theta) * v_x + cosf(theta) * v_y;
    float vsz = v_z;

    out.x = pjx + vsx;
    out.y = pjy + vsy;
    out.z = pjz + vsz;
    out.suckerJoint_status_ = alpha_deg;

    return true;
}