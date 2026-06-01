#include "Robot_Arm.h"
#include <iostream>

Robot_Arm::Robot_Arm(Arm_InitData_S init_Data)
    : init_data_(init_Data)
{
    if (init_data_.rotate_end < 250.0f || init_data_.rotate_end > 270.0f)
        init_data_.rotate_end = 265.0f;
}


void Robot_Arm::update()
{
    now_time_s_ = TimeStamp::getInstance().getSeconds();

    if(!time_initialized_)
    {
        last_time_s_ = now_time_s_;
        time_initialized_ = true;

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
        // 归一化到0~360度范围内
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
        // 归一化到0~360度范围内
        target_joint_angle_.launchJoint_Height_  = constrain(target_joint_angle_.launchJoint_Height_,  0.0f, init_data_.max_launchHeight_);
        target_joint_angle_.stretchJoint_Length_ = constrain(target_joint_angle_.stretchJoint_Length_, 0.0f, init_data_.max_stretchLength_);
       
        target_joint_angle_.rotateJoint_angle_ = calc_legal_rotate_target(
            joint_angle_.rotateJoint_angle_,
            target_joint_angle_.rotateJoint_angle_
        );
    }
    else if(control_mode_ == CURRENT_CONTROL_MODE)
        // 当前仅用于存储目标位置，不进行逆解和运动控制，直接
        return; // 直接返回，不进行任何操作
    
  
    // 计算目标电机角度，单位为电机转过的总角度，考虑减速比和机械臂结构
    float target_rotateMotorAngle = 0.0f;
    float target_stretchMotorAngle = 0.0f;
    float target_launchMotorAngle = 0.0f;
    float target_pitchMotorAngle = 0.0f;

    // 计算目标电机角度，单位为电机转过的总角度，考虑减速比和机械臂结构
    if (motor_rotate_ != nullptr)
    {
        float current_arm_total = MotorTotalAngle_to_rotateAngle(motor_rotate_->getTotalAngle());
        float target_arm = target_joint_angle_.rotateJoint_angle_;
        float diff = current_arm_total - target_arm;
        float k;
        if (rotate_strategy_ == ROTATE_PATH_POSITIVE)
            k = ceilf(diff / 360.0f);
        else if (rotate_strategy_ == ROTATE_PATH_NEGATIVE)
            k = floorf(diff / 360.0f);
        else
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
    /*计算时考虑斜率*/

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
    {
        HAL_GPIO_WritePin(init_data_.Sucker_GPIO_Port, init_data_.Sucker_GPIO_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(init_data_.Sucker_Soleniod_GPIO_Port, init_data_.Sucker_Soleniod_GPIO_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(init_data_.Sucker_GPIO_Port, init_data_.Sucker_GPIO_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(init_data_.Sucker_Soleniod_GPIO_Port, init_data_.Sucker_Soleniod_GPIO_Pin, GPIO_PIN_RESET);
    }


    if(store_sucker_status_ == SUCK)
    {
        HAL_GPIO_WritePin(init_data_.Store_GPIO_Port, init_data_.Store_GPIO_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(init_data_.Store_Soleniod_GPIO_Port, init_data_.Store_Soleniod_GPIO_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(init_data_.Store_GPIO_Port, init_data_.Store_GPIO_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(init_data_.Store_Soleniod_GPIO_Port, init_data_.Store_Soleniod_GPIO_Pin, GPIO_PIN_RESET);
    }
}

// 其实这段0个作用了
void Robot_Arm::inverseKinematics(Arm_Point_S target_point)
{
    // 解算
    float raw_deg;
    if (std::abs(target_point.x) < 1e-6f && std::abs(target_point.y) < 1e-6f)
        raw_deg = joint_angle_.rotateJoint_angle_;  // 
    else
        raw_deg = atan2f(target_point.y, target_point.x) * 180.0f / PI;
    target_joint_angle_.rotateJoint_angle_ = normalize_deg_0_360(raw_deg);

    target_joint_angle_.launchJoint_Height_ = target_point.z;
    target_joint_angle_.stretchJoint_Length_ = sqrt(target_point.x * target_point.x + target_point.y * target_point.y) - init_data_.arm_length_;

   
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
    /*解算*/
    float theta = joint_angle_.rotateJoint_angle_ * 3.1415926f / 180.0f;
    float Ltot  = init_data_.arm_length_ + joint_angle_.stretchJoint_Length_;

    out.x = Ltot * cosf(theta);
    out.y = Ltot * sinf(theta);
    out.z = joint_angle_.launchJoint_Height_;

    out.suckerJoint_status_ = joint_angle_.suckerJoint_angle_;
    return true;
}

float Robot_Arm::calc_legal_rotate_target(float current_0_360, float target_0_360)
{
    float re = init_data_.rotate_end;
    if (re < 250.0f || re > 270.0f)
        re = 265.0f;

    target_0_360 = fmodf(target_0_360, 360.0f);
    if (target_0_360 < 0.0f) target_0_360 += 360.0f;
    current_0_360 = fmodf(current_0_360, 360.0f);
    if (current_0_360 < 0.0f) current_0_360 += 360.0f;

    if (target_0_360 > init_data_.rotate_start && target_0_360 < re)
    {
        float dist_to_start = target_0_360 - init_data_.rotate_start;
        float dist_to_re = re - target_0_360;
        target_0_360 = (dist_to_start < dist_to_re) ? init_data_.rotate_start : re;
    }

    bool target_changed = (fabsf(target_0_360 - prev_norm_target_) > 0.01f);
    prev_norm_target_ = target_0_360;

    float diff = target_0_360 - current_0_360;

    float shortest_diff = diff;
    if (shortest_diff > 180.0f)
        shortest_diff -= 360.0f;
    else if (shortest_diff <= -180.0f)
        shortest_diff += 360.0f;

    if (_tool_Abs(shortest_diff) < 10.0f)
    {
        rotate_strategy_ = ROTATE_PATH_SHORTEST;
    }
    else
    {
        bool crosses = false;
        if (shortest_diff > 0.0f)
        {
            if (current_0_360 < re && (current_0_360 + shortest_diff) > init_data_.rotate_start)
                crosses = true;
        }
        else if (shortest_diff < 0.0f)
        {
            if (current_0_360 > init_data_.rotate_start && (current_0_360 + shortest_diff) < re)
                crosses = true;
        }

        if (crosses)
            rotate_strategy_ = (shortest_diff > 0.0f) ? ROTATE_PATH_NEGATIVE : ROTATE_PATH_POSITIVE;
        else
            rotate_strategy_ = ROTATE_PATH_SHORTEST;
    }

    if (diff > 180.0f)
        diff -= 360.0f;
    else if (diff <= -180.0f)
        diff += 360.0f;

    if (_tool_Abs(diff) >= 10.0f)
    {
        switch (rotate_strategy_)
        {
            case ROTATE_PATH_POSITIVE:
                if (diff < 0.0f) diff += 360.0f;
                break;
            case ROTATE_PATH_NEGATIVE:
                if (diff > 0.0f) diff -= 360.0f;
                break;
            default:
                break;
        }
    }

    float result = current_0_360 + diff;

    if (!target_changed)
    {
        float gap = result - prev_rotate_target_;
        float k = roundf(gap / 360.0f);
        result = result - k * 360.0f;
    }
    prev_rotate_target_ = result;

    result = fmodf(result, 360.0f);
    if (result < 0.0f) result += 360.0f;
    return result;
}