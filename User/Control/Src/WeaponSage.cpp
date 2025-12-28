#include "WeaponSage.h"

Robot_WeaponSage::Robot_WeaponSage(WeaponSage_InitData_S init_data) 
    : initData_(init_data)
{
}

bool Robot_WeaponSage::setMotorReversed(bool reversed, WeaponSage::Motor_Type_E motor_type)
{
    float sign = 0.0f;
    if(reversed)
        sign = -1.0f;
    else
        sign = 1.0f;
        
    switch(motor_type)
    {
        case WeaponSage::Launch_Motor:
            motor_reversed_.launch_reversed_ = sign;
            break;
        case WeaponSage::Claw_Motor:
            motor_reversed_.claw_reversed_ = sign;
            break;
        case WeaponSage::Traverse_Motor:
            motor_reversed_.traverse_reversed_ = sign;
            break;
        case WeaponSage::Wrist_Motor:
            motor_reversed_.wrist_reversed_ = sign;
            break;
        default:
            return false; 
    }
    return true;
}

void Robot_WeaponSage::update()
{
    current_pos_ = get_CurrentPos();
}



bool Robot_WeaponSage::setTarget(float targetValue, WeaponSage::Motor_Type_E motor_type)
{
    switch (ctrl_mode_)
    {
        case WeaponSage::CURRENT_CONTROL:
        {
            /* code */

            if(motor_type == WeaponSage::Launch_Motor)
            {
                if(launch_Motor_ != nullptr)
                    launch_Motor_->setTargetCurrent(targetValue);
                else
                    return false;
            }
            else if(motor_type == WeaponSage::Claw_Motor)
            {
                if(claw_Motor_ != nullptr)
                    claw_Motor_->setTargetCurrent(targetValue);
                else
                    return false;
            }
            else if(motor_type == WeaponSage::Traverse_Motor)
            {
                if(traverse_Motor_ != nullptr)
                    traverse_Motor_->setTargetCurrent(targetValue);
                else
                    return false;
            }
            /* 没有
            else if(motor_type == WeaponSage::Wrist_Motor)
            {
                if(wrist_Motor_ != nullptr)
                    wrist_Motor_->setTargetCurrent(targetValue);
                else
                    return false;
            }
            */
            else 
                return false;
            break;
        }

        case WeaponSage::Join_POSITION_CONTROL:
        {
            if(motor_type == WeaponSage::Launch_Motor)
            {
                if(launch_Motor_ != nullptr)
                {
                    target_pos_.launch_pos_ = constrain(targetValue, 0.0f, initData_.max_launchHeight_);
                    target_pos_.launch_TotalAngle_ = Realpos_to_MotorTotalAngle(target_pos_.launch_pos_, motor_type);
            
                    launch_Motor_->setTargetTotalAngle(target_pos_.launch_TotalAngle_);
                }
                else
                    return false;
            }
            else if (motor_type == WeaponSage::Claw_Motor)
            {
                if(claw_Motor_ != nullptr)
                {
                    target_pos_.claw_pos_ = constrain(targetValue, 0.0f, initData_.max_clawAngle_);
                    target_pos_.claw_TotalAngle_ = Realpos_to_MotorTotalAngle(target_pos_.claw_pos_, motor_type);
            
                    claw_Motor_->setTargetTotalAngle(target_pos_.claw_TotalAngle_);
                }
                else
                    return false;
            }
            else if (motor_type == WeaponSage::Traverse_Motor)
            {
                if(traverse_Motor_ != nullptr)
                {
                    target_pos_.traverse_pos_ = constrain(targetValue, 0.0f, initData_.max_traverseLength_);
                    target_pos_.traverse_TotalAngle_ = Realpos_to_MotorTotalAngle(target_pos_.traverse_pos_, motor_type);
            
                    traverse_Motor_->setTargetTotalAngle(target_pos_.traverse_TotalAngle_);
                }
                else
                    return false;
            }
            else if (motor_type == WeaponSage::Wrist_Motor)
            {
                if(wrist_Motor_ != nullptr)
                {
                    target_pos_.wrist_pos_ = targetValue; //手腕不限制位置
                    target_pos_.wrist_TotalAngle_ = Realpos_to_MotorTotalAngle(target_pos_.wrist_pos_, motor_type);
            
                    wrist_Motor_->setTargetTotalAngle(initData_.max_wristMotorRPM_, target_pos_.wrist_TotalAngle_);
                }
                else
                    return false;
            }
            else 
                return false;
            
            break;
        }

        case WeaponSage::TOTAL_ANGLE_CONTROL:
        {
            if(motor_type == WeaponSage::Launch_Motor)
            {
                if(launch_Motor_ != nullptr)
                {
                    float target_totoalAngle = constrain(targetValue,
                        Realpos_to_MotorTotalAngle(0.0f, motor_type),
                        Realpos_to_MotorTotalAngle(initData_.max_launchHeight_, motor_type)
                    );

                    launch_Motor_->setTargetTotalAngle(target_totoalAngle);
                }
                else
                    return false;
            } 
            else if(motor_type == WeaponSage::Claw_Motor)
            {
                if(claw_Motor_ != nullptr)
                {
                    float target_totoalAngle = constrain(targetValue,
                        Realpos_to_MotorTotalAngle(0.0f, motor_type),
                        Realpos_to_MotorTotalAngle(initData_.max_clawAngle_, motor_type)
                    );

                    claw_Motor_->setTargetTotalAngle(target_totoalAngle);
                }
                else
                    return false;
            } 
            else if(motor_type == WeaponSage::Traverse_Motor)
            {
                if(traverse_Motor_ != nullptr)
                {
                    float target_totoalAngle = constrain(targetValue,
                        Realpos_to_MotorTotalAngle(0.0f, motor_type),
                        Realpos_to_MotorTotalAngle(initData_.max_traverseLength_, motor_type)
                    );

                    traverse_Motor_->setTargetTotalAngle(target_totoalAngle);
                }
                else
                    return false;
            } 
            else if(motor_type == WeaponSage::Wrist_Motor)
            {
                if(wrist_Motor_ != nullptr)
                {
                    wrist_Motor_->setTargetTotalAngle(initData_.max_wristMotorRPM_, targetValue);
                }
                else
                    return false;
            } 
            else 
                return false;
            break;
        }

        default:
            break;
    }


    return true;
}


float Robot_WeaponSage::Realpos_to_MotorTotalAngle(float real_pos, WeaponSage::Motor_Type_E motor_type)
{
    switch(motor_type)
    {
        case WeaponSage::Launch_Motor:
            return motor_reversed_.launch_reversed_ * real_pos / initData_.launch_Ratio_ * 360.0f;

        case WeaponSage::Claw_Motor:
            return motor_reversed_.claw_reversed_ * real_pos / initData_.claw_gearRatio_ * 360.0f;

        case WeaponSage::Traverse_Motor:
            return motor_reversed_.traverse_reversed_ * real_pos / initData_.traverse_Ratio_ * 360.0f;

        case WeaponSage::Wrist_Motor:
            return motor_reversed_.wrist_reversed_ * real_pos / initData_.wrist_gearRatio_ * 360.0f;

        default:
            return 0.0f; 
    }
}

float Robot_WeaponSage::MotorTotalAngle_to_Realpos(float motor_angle, WeaponSage::Motor_Type_E motor_type)
{
    switch(motor_type)
    {
        case WeaponSage::Launch_Motor:
            return motor_reversed_.launch_reversed_ * motor_angle * initData_.launch_Ratio_ / 360.0f;

        case WeaponSage::Claw_Motor:
            return motor_reversed_.claw_reversed_ * motor_angle * initData_.claw_gearRatio_ / 360.0f;

        case WeaponSage::Traverse_Motor:
            return motor_reversed_.traverse_reversed_ * motor_angle * initData_.traverse_Ratio_ / 360.0f;

        case WeaponSage::Wrist_Motor:
            return motor_reversed_.wrist_reversed_ * motor_angle * initData_.wrist_gearRatio_ / 360.0f;

        default:
            return 0.0f; 
    }
}

bool Robot_WeaponSage::setMotorTargetTotalAngle(float total_angle, WeaponSage::Motor_Type_E motor_type)
{
    switch(motor_type)
    {
        case WeaponSage::Wrist_Motor :
        {
            if(wrist_Motor_ != nullptr)
            {
                wrist_Motor_->setTargetTotalAngle(initData_.max_wristMotorRPM_, total_angle);
                return true;
            }
            else
                return false;
        }

        case WeaponSage::Launch_Motor :
        {
            if(launch_Motor_ != nullptr)
            {
                launch_Motor_->setTargetTotalAngle(total_angle);
                return true;
            }
            else
                return false;
        }

        case WeaponSage::Claw_Motor :
        {
            if(claw_Motor_ != nullptr)
            {
                claw_Motor_->setTargetTotalAngle(total_angle);
                return true;
            }
            else
                return false;
        }

        case WeaponSage::Traverse_Motor :
        {
            if(traverse_Motor_ != nullptr)
            {
                traverse_Motor_->setTargetTotalAngle(total_angle);
                return true;
            }
            else
                return false;
        }
    }
}

