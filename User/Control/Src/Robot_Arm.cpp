#include "Robot_Arm.h"
#include <iostream>

Robot_Arm::Robot_Arm(Arm_InitData_S init_Data)
    : init_data_(init_Data)
{
}

float ramp_rate = 18000;
void Robot_Arm::update()
{
    /*获取当前时间 */
    now_time_s_ = TimeStamp::getInstance().getSeconds();

    if(!time_initialized_)
    {
        last_time_s_ = now_time_s_;
        time_initialized_ = true;
        // 第一次更新时，直接将目标位置设置为当前机械臂位置，避免初始时的跳动
        target_joint_angle_ = joint_angle_;
        return;
    }

    dt_ = now_time_s_ - last_time_s_;
    last_time_s_ = now_time_s_;

    if (motor_rotate_ != nullptr)
    {
        ramped_rotateMotorAngle_ = motor_rotate_->getTotalAngle();
    }
    
    if(motor_rotate_ != nullptr)
    {
        // 直接获取机械臂旋转角度，范围为 0-360 度
        float raw_angle = MotorTotalAngle_to_rotateAngle(motor_rotate_->getTotalAngle());
        joint_angle_.rotateJoint_angle_ = normalize_deg_0_360(raw_angle);
    }
    if(motor_stretch_ != nullptr)
        joint_angle_.stretchJoint_Length_ = MotorTotalAngle_to_stretchLength(motor_stretch_->getTotalAngle());
    if(motor_launch_ != nullptr)
        joint_angle_.launchJoint_Height_ = MotorTotalAngle_to_launchHeight(motor_launch_->getTotalAngle());
    if(motor_pitch_ != nullptr)
        joint_angle_.suckerJoint_angle_ = MotorTotalAngle_to_pitchAngle(motor_pitch_->getTotalAngle());


    forwardKinematics(arm_forward_);

    if(control_mode_ == TARGET_POSITION_MODE)
        inverseKinematics(arm_target_);

    else if(control_mode_ == MANUAL_MOTOR_POSITION_MODE)
    {
        // 限幅
        target_joint_angle_.launchJoint_Height_  = constrain(target_joint_angle_.launchJoint_Height_,  0.0f, init_data_.max_launchHeight_);
        target_joint_angle_.stretchJoint_Length_ = constrain(target_joint_angle_.stretchJoint_Length_, 0.0f, init_data_.max_stretchLength_);
       
        target_joint_angle_.rotateJoint_angle_ = calc_rotate_targetByStrategy(
            joint_angle_.rotateJoint_angle_,
            target_joint_angle_.rotateJoint_angle_
        );
    }
    else if(control_mode_ == CURRENT_CONTROL_MODE)
        // 当前控制模式
        return; // 直接返回
    

  

    float target_rotateMotorAngle = 0.0f;
    float target_stretchMotorAngle = 0.0f;
    float target_launchMotorAngle = 0.0f;
    float target_pitchMotorAngle = 0.0f;

    // 旋转关节需要特殊处理，确保按照设定的路径策略旋转到目标位置
    if (motor_rotate_ != nullptr)
    {
        float current_arm_total = MotorTotalAngle_to_rotateAngle(motor_rotate_->getTotalAngle());
        
        // 目标旋转角度，范围为 0-360 度
        float target_arm = target_joint_angle_.rotateJoint_angle_;
        
        // 计算当前机械臂角度与目标角度的差值，考虑旋转路径
        float diff = current_arm_total - target_arm;
        float k;
        if (rotate_strategy_ == ROTATE_PATH_POSITIVE)
            k = ceilf(diff / 360.0f);
        else if (rotate_strategy_ == ROTATE_PATH_NEGATIVE)
            k = floorf(diff / 360.0f);
        else // SHORTEST
            k = roundf(diff / 360.0f);

        float target_arm_total = target_arm + k * 360.0f;
        target_rotateMotorAngle = rotateAngle_to_MotorTotalAngle(target_arm_total);

        rotate_fliter_ramp_.ramp_target_ = caculate_ramp_target(motor_rotate_->getTotalAngle(), 
            target_rotateMotorAngle, rotate_fliter_ramp_);
        motor_rotate_->setTargetTotalAngle(rotate_fliter_ramp_.ramp_target_);
    }

    target_stretchMotorAngle = stretchLength_to_MotorTotalAngle(target_joint_angle_.stretchJoint_Length_);
    target_launchMotorAngle = launchHeight_to_MotorTotalAngle(target_joint_angle_.launchJoint_Height_);
    target_pitchMotorAngle = pitchAngle_to_MotorTotalAngle(target_joint_angle_.suckerJoint_angle_);


    if(motor_stretch_ != nullptr)
    {
        strech_fliter_ramp_.ramp_target_ = caculate_ramp_target(motor_stretch_->getTotalAngle(), 
            target_stretchMotorAngle, strech_fliter_ramp_);
        motor_stretch_->setTargetTotalAngle(strech_fliter_ramp_.ramp_target_);
    }

    if(motor_launch_ != nullptr)
    {
        launch_fliter_ramp_.ramp_target_ = caculate_ramp_target(motor_launch_->getTotalAngle(), 
            target_launchMotorAngle, launch_fliter_ramp_);
        motor_launch_->setTargetTotalAngle(launch_fliter_ramp_.ramp_target_);
    }
    if(motor_pitch_ != nullptr)
        motor_pitch_->setTargetTotalAngle(init_data_.max_pitchRPM_, target_pitchMotorAngle);

    if(sucker_status_ == SUCK)
        HAL_GPIO_WritePin(init_data_.Sucker_GPIO_Port, init_data_.Sucker_GPIO_Pin, GPIO_PIN_SET);
    
    else
        HAL_GPIO_WritePin(init_data_.Sucker_GPIO_Port, init_data_.Sucker_GPIO_Pin, GPIO_PIN_RESET);
}

void Robot_Arm::inverseKinematics(Arm_Point_S target_point)
{
    // 计算旋转角度
    float raw_deg;
    if (std::abs(target_point.x) < 1e-6f && std::abs(target_point.y) < 1e-6f)
        raw_deg = joint_angle_.rotateJoint_angle_;  // 保持当前角度
    else
        raw_deg = atan2f(target_point.y, target_point.x) * 180.0f / PI;

    // 规范化目标旋转角度到 0-360 度
    target_joint_angle_.rotateJoint_angle_ = normalize_deg_0_360(raw_deg);

    target_joint_angle_.launchJoint_Height_ = target_point.z;
    target_joint_angle_.stretchJoint_Length_ = sqrt(target_point.x * target_point.x + target_point.y * target_point.y) - init_data_.arm_length_;



    /*规范化目标角度 */
   
    target_joint_angle_.suckerJoint_angle_ = target_point.suckerJoint_status_;

    target_joint_angle_.launchJoint_Height_ = constrain(target_joint_angle_.launchJoint_Height_,
                                                    0.0f,
                                                    init_data_.max_launchHeight_);
    target_joint_angle_.stretchJoint_Length_ = constrain(target_joint_angle_.stretchJoint_Length_,
                                                     0.0f,
                                                     init_data_.max_stretchLength_);
}


bool Robot_Arm::forwardKinematics(Arm_Point_S& out) const
{
    /*末锟剿关斤拷位锟斤拷*/
    float theta = joint_angle_.rotateJoint_angle_ * 3.1415926f / 180.0f;
    float Ltot  = init_data_.arm_length_ + joint_angle_.stretchJoint_Length_;

    out.x = Ltot * cosf(theta);
    out.y = Ltot * sinf(theta);
    out.z = joint_angle_.launchJoint_Height_;

    out.suckerJoint_status_ = joint_angle_.suckerJoint_angle_;
    return true;
}

float Robot_Arm::calc_rotate_targetByStrategy(float current_cont_angle, float target_raw_0_360)
{
    // 规范化当前角度到 0-360 度
    float current_mod = fmodf(current_cont_angle, 360.0f);
    if(current_mod < 0)
        current_mod += 360.0f;

    // 规范化目标角度到 0-360 度
    float target_mod = fmodf(target_raw_0_360, 360.0f);
    if(target_mod < 0)
        target_mod += 360.0f;

    float diff = target_mod - current_mod;

    // 计算最短旋转距离
    float shortest_diff = diff;
    if (shortest_diff > 180.0f)
        shortest_diff -= 360.0f;
    else if (shortest_diff < -180.0f)
        shortest_diff += 360.0f;

    if (_tool_Abs(shortest_diff) < 10.0f)
        return current_cont_angle + shortest_diff;

    switch(rotate_strategy_)
    {
        case ROTATE_PATH_SHORTEST:
            // 
            if (diff > 180.0f)       
                diff -= 360.0f;
            else if (diff < -180.0f) 
                diff += 360.0f;
            break;

        case ROTATE_PATH_POSITIVE:
            // 
            if (diff < 0.0f) 
                diff += 360.0f;
            break;

        case ROTATE_PATH_NEGATIVE:
            // 
            if (diff > 0.0f) 
                diff -= 360.0f;
            break;
    }

    return current_cont_angle + diff;
}



























