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
    if(ctrl_mode_ == WeaponSage::CURRENT_CONTROL)
	{
	         //电流控制模式 do nothing
	}
	if(ctrl_mode_ == WeaponSage::Join_POSITION_CONTROL)
	{
        
		float wrist_pos_ = normalize_deg_0_360(target_pos_.wrist_pos_);
        float diff= current_pos_.wrist_pos_ - wrist_pos_;
        float k =roundf(diff/360.0f);
        float target_wrist_total= wrist_pos_ + k*360.0f;
//		bool is_crossing_zero = false;
//        if(abs(wrist_pos_  -current_pos_.wrist_pos_)>180.0f)
//        {
//            is_crossing_zero = true;
//        }else
//        {
//            is_crossing_zero = false;
//        }
//        if(!is_crossing_zero)
//        {
//            if(current_pos_.wrist_pos_<wrist_pos_)
//            {
//                //do nothing
//            }else{
//                wrist_pos_ -= 360.0f;
//            }
//        }else{
//            if(current_pos_.wrist_pos_<wrist_pos_)
//            {
//                wrist_pos_ -= 360.0f;
//            }else{
//                //do nothing
//            }
//        }

		target_pos_.launch_TotalAngle_ = Realpos_to_MotorTotalAngle(target_pos_.launch_pos_, WeaponSage::Launch_Motor);
		target_pos_.claw_1_TotalAngle_ = Realpos_to_MotorTotalAngle(target_pos_.claw_1_pos_, WeaponSage::Claw_1_Motor);
		target_pos_.claw_2_TotalAngle_ = Realpos_to_MotorTotalAngle(target_pos_.claw_2_pos_, WeaponSage::Claw_2_Motor);
		target_pos_.claw_3_TotalAngle_ = Realpos_to_MotorTotalAngle(target_pos_.claw_3_pos_, WeaponSage::Claw_3_Motor);
		target_pos_.wrist_TotalAngle_ = Realpos_to_MotorTotalAngle(wrist_pos_, WeaponSage::Wrist_Motor);


		launch_Motor_->setTargetTotalAngle( target_pos_.launch_TotalAngle_);
		claw_1_Motor_->setTargetTotalAngle(target_pos_.claw_1_TotalAngle_);
		claw_2_Motor_->setTargetTotalAngle(target_pos_.claw_2_TotalAngle_);
		claw_3_Motor_->setTargetTotalAngle(target_pos_.claw_3_TotalAngle_);
		wrist_Motor_->setTargetTotalAngle( target_pos_.wrist_TotalAngle_);
		arm_Motor_->setTargetTotalAngle(initData_.max_arm_rate_,target_pos_.arm_pos_);
	}
}
bool Robot_WeaponSage::setClaw_1_angle(float angle)
{
    if(claw_1_Motor_ != nullptr)
    {
        target_pos_.claw_1_pos_ = constrain(angle, 0.0f, initData_.max_clawAngle_);
        return true;
    }
    else
        return false;
}

bool Robot_WeaponSage::setClaw_2_angle(float angle)
{
    if(claw_2_Motor_ != nullptr)
    {
        target_pos_.claw_2_pos_ = constrain(angle, 0.0f, initData_.max_clawAngle_);
        return true;
    }
    else
        return false;
}

bool Robot_WeaponSage::setClaw_3_angle(float angle)
{
    if(claw_3_Motor_ != nullptr)
    {
        target_pos_.claw_3_pos_ = constrain(angle, 0.0f, initData_.max_clawAngle_);
        return true;
    }
    else
        return false;
}

bool Robot_WeaponSage::setWrist_angle(float angle)
{
    if(wrist_Motor_ != nullptr)
    {
        target_pos_.wrist_pos_ = angle;


        return true;
    }
    else
        return false;
}

bool Robot_WeaponSage::setArm_angle(float angle)
{
    if(arm_Motor_ != nullptr)
    {
        target_pos_.arm_pos_ = constrain(angle, 0.0f, initData_.max_arm_angle_);
        return true;
    }
    else
        return false;
}

bool Robot_WeaponSage::setLaunch_angle(float angle)
{
    if(launch_Motor_ != nullptr)
    {
        target_pos_.launch_pos_ = constrain(angle, 0.0f, initData_.max_launchHeight_);
        return true;
    }
    else
        return false;
}


float Robot_WeaponSage::Realpos_to_MotorTotalAngle(float real_pos, WeaponSage::Motor_Type_E motor_type)
{
    switch(motor_type)
    {
        case WeaponSage::Launch_Motor:
            return motor_reversed_.launch_reversed_ * real_pos / initData_.launch_Ratio_ * 360.0f; 
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

