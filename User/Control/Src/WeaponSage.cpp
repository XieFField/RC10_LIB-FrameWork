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
        case WeaponSage::Claw_1_Motor:
            motor_reversed_.claw_1_reversed_ = sign;
            break;
        case WeaponSage::Claw_2_Motor:
            motor_reversed_.claw_2_reversed_ = sign;
            break;
        case WeaponSage::Claw_3_Motor:
            motor_reversed_.claw_3_reversed_ = sign;
            break;
        case WeaponSage::Wrist_Motor:
            motor_reversed_.wrist_reversed_ = sign;
            break;
        case WeaponSage::Arm_Motor:
            motor_reversed_.arm_reversed_ = sign;
            break;
        default:
            return false; 
    }
    return true;
}

void Robot_WeaponSage::update()
{
    current_pos_ = get_CurrentPos();
    switch(ctrl_mode_)
    {
        case WeaponSage::CURRENT_CONTROL:
            // do nothing
            break;
        case WeaponSage::Join_POSITION_CONTROL:
        {
            launch_Motor_->setTargetTotalAngle( target_pos_.launch_TotalAngle_);
            claw_1_Motor_->setTargetTotalAngle(target_pos_.claw_1_TotalAngle_);
            claw_2_Motor_->setTargetTotalAngle(target_pos_.claw_2_TotalAngle_);
            claw_3_Motor_->setTargetTotalAngle(target_pos_.claw_3_TotalAngle_);
            wrist_Motor_->setTargetTotalAngle( target_pos_.wrist_TotalAngle_);
            arm_Motor_->setTargetTotalAngle(initData_.max_arm_rate_,target_pos_.arm_TotalAngle_);
            break;
        }
            
        case WeaponSage::TOTAL_ANGLE_CONTROL:
        {
            launch_Motor_->setTargetTotalAngle( target_pos_.launch_TotalAngle_);
            claw_1_Motor_->setTargetTotalAngle(target_pos_.claw_1_TotalAngle_);
            claw_2_Motor_->setTargetTotalAngle(target_pos_.claw_2_TotalAngle_);
            claw_3_Motor_->setTargetTotalAngle(target_pos_.claw_3_TotalAngle_);
            wrist_Motor_->setTargetTotalAngle( target_pos_.wrist_TotalAngle_);
            arm_Motor_->setTargetTotalAngle( initData_.max_arm_rate_,target_pos_.arm_TotalAngle_);
            break;
        }
            
        default:
            break;
    }

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
                if(launch_Motor_ != nullptr )
                {
                    launch_Motor_->setTargetCurrent(targetValue);
                }
                else
                    return false;
            }
            else if(motor_type == WeaponSage::Claw_1_Motor)
            {
                if(claw_1_Motor_ != nullptr)
                    claw_1_Motor_->setTargetCurrent(targetValue);
                else
                    return false;
            }
            else if(motor_type == WeaponSage::Claw_2_Motor)
            {
                if(claw_2_Motor_ != nullptr)
                    claw_2_Motor_->setTargetCurrent(targetValue);
                else
                    return false;
            }
            else if(motor_type == WeaponSage::Claw_3_Motor)
            {
                if(claw_3_Motor_ != nullptr)
                claw_3_Motor_->setTargetCurrent(targetValue);
                else
                    return false;
            }

            else if(motor_type == WeaponSage::Wrist_Motor)
            {
                if(wrist_Motor_ != nullptr)
                    wrist_Motor_->setTargetCurrent(targetValue);
                else
                   return false;
            }
            else if(motor_type == WeaponSage::Arm_Motor)
            {
                // if(arm_Motor_ != nullptr)
                //     arm_Motor_->setTargetCurrent(targetValue);
                // else
                //     return false;

                //DM电机不支持电流控制，直接返回 false
                return false;
            }
            else 
                return false;
            break;
        }

        case WeaponSage::Join_POSITION_CONTROL:
        {
            if(motor_type == WeaponSage::Launch_Motor)
            {
                if(launch_Motor_ != nullptr )
                {
                    target_pos_.launch_pos_ = constrain(targetValue, 0.0f, initData_.max_launchHeight_);
                    target_pos_.launch_TotalAngle_ = Realpos_to_MotorTotalAngle(target_pos_.launch_pos_, WeaponSage::Launch_Motor);                
                }
                else
                    return false;
            }
            else if (motor_type == WeaponSage::Claw_1_Motor)
            {
                if(claw_1_Motor_ != nullptr)
                {
                    target_pos_.claw_1_pos_ = constrain(targetValue, 0.0f, initData_.max_clawAngle_);
                    target_pos_.claw_1_TotalAngle_ = Realpos_to_MotorTotalAngle(target_pos_.claw_1_pos_, WeaponSage::Claw_1_Motor);
                }
                else
                    return false;
            }
                else if (motor_type == WeaponSage::Claw_2_Motor)
                {
                    if(claw_2_Motor_ != nullptr)
                    {
                        target_pos_.claw_2_pos_ = constrain(targetValue, 0.0f, initData_.max_clawAngle_);
                        target_pos_.claw_2_TotalAngle_ = Realpos_to_MotorTotalAngle(target_pos_.claw_2_pos_, WeaponSage::Claw_2_Motor);
                    }
                    else
                        return false;
                }
                else if (motor_type == WeaponSage::Claw_3_Motor)
                {
                    if(claw_3_Motor_ != nullptr)
                    {
                        target_pos_.claw_3_pos_ = constrain(targetValue, 0.0f, initData_.max_clawAngle_);
                        target_pos_.claw_3_TotalAngle_ = Realpos_to_MotorTotalAngle(target_pos_.claw_3_pos_, WeaponSage::Claw_3_Motor);
                    }
                    else
                        return false;
                }
            else if (motor_type == WeaponSage::Wrist_Motor)
            {
                if(wrist_Motor_ != nullptr)
                {
                    target_pos_.wrist_pos_ = constrain(targetValue, 0.0f, initData_.max_wrist_angle_); //手腕不限制位置
                    target_pos_.wrist_TotalAngle_ = Realpos_to_MotorTotalAngle(target_pos_.wrist_pos_, WeaponSage::Wrist_Motor);
                }
                else
                    return false;
            }
            else if (motor_type == WeaponSage::Arm_Motor)
            {
                if(arm_Motor_ != nullptr)
                {
                    target_pos_.arm_pos_ = constrain(targetValue, 0.0f, initData_.max_arm_angle_); //机械臂不限制位置
                    target_pos_.arm_TotalAngle_ = Realpos_to_MotorTotalAngle(target_pos_.arm_pos_, WeaponSage::Arm_Motor);
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
                if(launch_Motor_ != nullptr )
                {
                    target_pos_.launch_TotalAngle_ = constrain(targetValue,
                        Realpos_to_MotorTotalAngle(0.0f, motor_type),
                        Realpos_to_MotorTotalAngle(initData_.max_launchHeight_, motor_type)
                    );
                    target_pos_.launch_pos_ = MotorTotalAngle_to_Realpos(target_pos_.launch_TotalAngle_, motor_type);
                }
                else
                    return false;
            } 
            else if(motor_type == WeaponSage::Claw_1_Motor)
            {
                if(claw_1_Motor_ != nullptr)
                {
                    target_pos_.claw_1_TotalAngle_ = constrain(targetValue,
                        Realpos_to_MotorTotalAngle(0.0f, motor_type),
                        Realpos_to_MotorTotalAngle(initData_.max_clawAngle_, motor_type)
                    );
                    target_pos_.claw_1_pos_ = MotorTotalAngle_to_Realpos(target_pos_.claw_1_TotalAngle_, motor_type);
                }
                else
                    return false;
            } 
            else if(motor_type == WeaponSage::Claw_2_Motor)
            {
                if(claw_2_Motor_ != nullptr)
                {
                    target_pos_.claw_2_TotalAngle_ = constrain(targetValue,
                        Realpos_to_MotorTotalAngle(0.0f, motor_type),
                        Realpos_to_MotorTotalAngle(initData_.max_clawAngle_, motor_type)
                    );
                        target_pos_.claw_2_pos_ = MotorTotalAngle_to_Realpos(target_pos_.claw_2_TotalAngle_, motor_type);
                }
                else
                    return false;
                } 
            else if(motor_type == WeaponSage::Claw_3_Motor)
            {
                if(claw_3_Motor_ != nullptr)
                {
                    target_pos_.claw_3_TotalAngle_ = constrain(targetValue,
                        Realpos_to_MotorTotalAngle(0.0f, motor_type),
                        Realpos_to_MotorTotalAngle(initData_.max_clawAngle_, motor_type)
                    );
                    target_pos_.claw_3_pos_ = MotorTotalAngle_to_Realpos(target_pos_.claw_3_TotalAngle_, motor_type);
                }
                else
                    return false;
                }
            else if(motor_type == WeaponSage::Wrist_Motor)
            {
                if(wrist_Motor_ != nullptr)
                {
                    target_pos_.wrist_TotalAngle_ = targetValue;
                    target_pos_.wrist_pos_ = MotorTotalAngle_to_Realpos(target_pos_.wrist_TotalAngle_, motor_type);
                }
                else
                    return false;
            } 
            else if(motor_type == WeaponSage::Arm_Motor)
            {
                if(arm_Motor_ != nullptr)
                {
                    target_pos_.arm_TotalAngle_ = targetValue;
                    target_pos_.arm_pos_ = MotorTotalAngle_to_Realpos(target_pos_.arm_TotalAngle_, motor_type);
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
}


float Robot_WeaponSage::Realpos_to_MotorTotalAngle(float real_pos, WeaponSage::Motor_Type_E motor_type)
{
    switch(motor_type)
    {
        case WeaponSage::Launch_Motor:
            return motor_reversed_.launch_reversed_ * real_pos / initData_.launch_Ratio_ * 360.0f; //以master电机为准
        case WeaponSage::Claw_1_Motor:
            return motor_reversed_.claw_1_reversed_ * real_pos / initData_.claw_gearRatio_ * 360.0f;
        case WeaponSage::Claw_2_Motor:
            return motor_reversed_.claw_2_reversed_ * real_pos / initData_.claw_gearRatio_ * 360.0f;
        case WeaponSage::Claw_3_Motor:
            return motor_reversed_.claw_3_reversed_ * real_pos / initData_.claw_gearRatio_ * 360.0f;
        case WeaponSage::Wrist_Motor:
            return motor_reversed_.wrist_reversed_ * real_pos / initData_.wrist_gearRatio_ * 360.0f;
        case WeaponSage::Arm_Motor:
            return motor_reversed_.arm_reversed_ * real_pos / initData_.arm_gearRatio_ * 360.0f;
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

        case WeaponSage::Claw_1_Motor:
            return motor_reversed_.claw_1_reversed_ * motor_angle * initData_.claw_gearRatio_ / 360.0f;

        case WeaponSage::Claw_2_Motor:
            return motor_reversed_.claw_2_reversed_ * motor_angle * initData_.claw_gearRatio_ / 360.0f;

        case WeaponSage::Claw_3_Motor:
            return motor_reversed_.claw_3_reversed_ * motor_angle * initData_.claw_gearRatio_ / 360.0f;

        case WeaponSage::Wrist_Motor:
            return motor_reversed_.wrist_reversed_ * motor_angle * initData_.wrist_gearRatio_ / 360.0f;

        case WeaponSage::Arm_Motor:
            return motor_reversed_.arm_reversed_ * motor_angle * initData_.arm_gearRatio_ / 360.0f;

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
                wrist_Motor_->setTargetTotalAngle(total_angle);
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

         case WeaponSage::Arm_Motor :
        {
            if(arm_Motor_ != nullptr)
            {
                arm_Motor_->setTargetTotalAngle(initData_.max_arm_rate_,total_angle);
                return true;
            }
            else
                return false;
        }

        case WeaponSage::Claw_1_Motor :
        {
            if(claw_1_Motor_ != nullptr)
            {
                claw_1_Motor_->setTargetTotalAngle(total_angle);
                return true;
            }
            else
                return false;
        }

         case WeaponSage::Claw_2_Motor :
        {
            if(claw_2_Motor_ != nullptr)
            {
                claw_2_Motor_->setTargetTotalAngle(total_angle);
                return true;
            }
            else
                return false;
        }

         case WeaponSage::Claw_3_Motor :
        {
            if(claw_3_Motor_ != nullptr)
            {
                claw_3_Motor_->setTargetTotalAngle(total_angle);
                return true;
            }
            else
                return false;
        }


 

        default:
        {
            return false;
            break;
        }
    }
}

