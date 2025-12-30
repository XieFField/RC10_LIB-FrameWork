#include "WeaponSage_Setup.h"

namespace WeaponSage_Setup {
    float weapon_pos[2] = {0.1217993396579f,0.3224356578564f};
}

Robot_WeaponSage_Setup::Robot_WeaponSage_Setup(WeaponSage_InitData_S init_data)
    : Robot_WeaponSage(init_data), RtosTask("WeaponSage_Setup", 1)
{
    
}

void Robot_WeaponSage_Setup::loop()
{	
	ctrl_status_.now_times=TimeStamp::getInstance().getSeconds();
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
	if(!ctrl_status_.is_calibrating)
	{
		calibrate();
		weaponSage_status_=WEAPONSAGE_CALIBRATE;
	}
    switch(weaponSage_status_)
    {
        case WEAPONSAGE_MANUAL_CONTROL:
            manualControl();
            break;
        case WEAPONSAGE_IDLE:
            idle();
            break;
        case WEAPONSAGE_STOP:
            stop();
            break;
        case WEAPONSAGE_DEBUG:
		{
            debug();
            break;
		}
        case WEAPONSAGE_AUTO_CONTROL:
		{
            break;
	    }
		case WEAPONSAGE_CALIBRATE:
		{
			//calibrate();
			
            break;
	    }
			
        default:
            idle();
            break;
    }

    this->update();
}
int CNT=0;
float traverse_rate=0.002;
float weapon_launch_rate=0.002;
float Kp_traverse=0.2;




void Robot_WeaponSage_Setup::calibrate()
{
    if(!ctrl_status_.is_calibrating)
    {
        if(!ctrl_status_.calibrate_start)
        {
            ctrl_status_.calibrate_startTime = TimeStamp::getInstance().getSeconds();
            ctrl_status_.calibrate_start = true;       
        }
	
        this->setCtrlMode(WeaponSage::CURRENT_CONTROL);
        this->setTarget(500.0f, WeaponSage::Claw_Motor);
        if(!auto_ctrl_.auto_state_bool_S.wrist_enable)
        {
            Weapon_wrist_enable();
        }
        if(ctrl_status_.now_times - ctrl_status_.calibrate_startTime > 1.5f)
        {
            //relocate
            this->claw_Motor_->relocate_totalAngle(0.0f);
            this->traverse_Motor_->relocate_totalAngle(0.0f);
            this->launch_Motor_->relocate_totalAngle(0.0f);
            
            this->setTarget(0.0f, WeaponSage::Claw_Motor);
            this->setTarget(0.0f, WeaponSage::Traverse_Motor);
            auto_ctrl_.auto_state_bool_S.wrist_enable=true;
            ctrl_status_.is_calibrating = true;
        }

    }
}

void Robot_WeaponSage_Setup::manualControl()
{
    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
    switch(airjoy_data_.SWA)
    {
        case 0x00:
        {
            //夹取武器
            if(airjoy_data_.SWD == 0x00)
                target_pos_.claw_pos_ = 0.0f; //开爪子
            else if(airjoy_data_.SWD == 0x01)
                target_pos_.claw_pos_ = initData_.max_clawAngle_; //紧爪子
            else
                target_pos_.claw_pos_ = 0.0f;

            //夹爪位置

            if(_tool_Abs(airjoy_data_.right_x) < 0.1)
                manual_ctrlForgrip_.changeTarget_state = false;

            else if(airjoy_data_.right_x > 0.5f)
            {
                manual_ctrlForgrip_.changeTarget_state = true;
                ctrl_status_.target_poleIndex++;
            }
            else if(airjoy_data_.right_x < -0.5f)
            {
                manual_ctrlForgrip_.changeTarget_state = true;
                ctrl_status_.target_poleIndex--;
            }

            if(ctrl_status_.target_poleIndex < 0)
                ctrl_status_.target_poleIndex = 0;
            else if(ctrl_status_.target_poleIndex > 3)
                ctrl_status_.target_poleIndex = 3;

            target_pos_.traverse_pos_ = WeaponSage_Setup::weapon_pos[ctrl_status_.target_poleIndex];

            if(airjoy_data_.right_y > 0.5f)
                target_pos_.launch_pos_ += weapon_launch_rate;
            else if(airjoy_data_.right_y < -0.5f)
                target_pos_.launch_pos_ -= weapon_launch_rate;
            else
                target_pos_.launch_pos_ = target_pos_.launch_pos_;
			if(auto_ctrl_.auto_state_bool_S.wrist_enable)
			{
            target_pos_.wrist_pos_ = 0.0f;
			}
			manual_ctrlForgrip_.last_right_stick_x = airjoy_data_.right_x;
            manual_ctrlForgrip_.last_right_stick_y = airjoy_data_.right_y;

            break;
        }

        case 0x01:
        {
            //进攻模式
			CNT++;
            if(CNT>10)
			{
				if(_tool_Abs(airjoy_data_.right_x) < 0.1)
                manual_ctrlForgrip_.changeTarget_state = false;

				else if(airjoy_data_.right_x > 0.5f)
				{
					manual_ctrlForgrip_.changeTarget_state = true;
					if(current_pos_.traverse_pos_>0.2*initData_.max_traverseLength_&&current_pos_.traverse_pos_<0.8*initData_.max_traverseLength_)
					{
						target_pos_.traverse_pos_+=traverse_rate;
					}
					if(current_pos_.traverse_pos_<=0.2*initData_.max_traverseLength_||current_pos_.traverse_pos_>=0.8*initData_.max_traverseLength_)
					{
						target_pos_.traverse_pos_+=traverse_rate*Kp_traverse;
					}
				}
				else if(airjoy_data_.right_x < -0.5f)
				{
					manual_ctrlForgrip_.changeTarget_state = true;
					if(current_pos_.traverse_pos_>0.2*initData_.max_traverseLength_&&current_pos_.traverse_pos_<0.8*initData_.max_traverseLength_)
					{
						target_pos_.traverse_pos_-=traverse_rate;
					}
					if(current_pos_.traverse_pos_<=0.2*initData_.max_traverseLength_||current_pos_.traverse_pos_>=0.8*initData_.max_traverseLength_)
					{
						target_pos_.traverse_pos_-=traverse_rate*Kp_traverse;
					}
				}


				if(airjoy_data_.right_y > 0.5f)
					target_pos_.launch_pos_ += weapon_launch_rate;
				else if(airjoy_data_.right_y < -0.5f)
					target_pos_.launch_pos_ -= weapon_launch_rate;
				else
					target_pos_.launch_pos_ = target_pos_.launch_pos_;
				if(auto_ctrl_.auto_state_bool_S.wrist_enable)
				{
				target_pos_.wrist_pos_ = 90.0f;
				}
				manual_ctrlForgrip_.last_right_stick_x = airjoy_data_.right_x;
				manual_ctrlForgrip_.last_right_stick_y = airjoy_data_.right_y;
				CNT=0;

			}
            break; 
        }

        default:
        {
            idle();
            break;
        }
    }

    this->setTarget(target_pos_.launch_pos_, WeaponSage::Launch_Motor);
    this->setTarget(target_pos_.claw_pos_, WeaponSage::Claw_Motor);
    this->setTarget(target_pos_.traverse_pos_, WeaponSage::Traverse_Motor);
    this->setTarget(target_pos_.wrist_pos_, WeaponSage::Wrist_Motor);
}

void Robot_WeaponSage_Setup::idle()
{
    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
	if(last_weaponSage_status_!=WEAPONSAGE_IDLE)
	{
		this->last_pos_ = this->get_CurrentPos();
		this->target_pos_=this->last_pos_;
		last_weaponSage_status_=WEAPONSAGE_IDLE;

	}
	this->setTarget(target_pos_.launch_pos_, WeaponSage::Launch_Motor);
    this->setTarget(target_pos_.claw_pos_, WeaponSage::Claw_Motor);
    this->setTarget(target_pos_.traverse_pos_, WeaponSage::Traverse_Motor);
    this->setTarget(target_pos_.wrist_pos_, WeaponSage::Wrist_Motor);
}

void Robot_WeaponSage_Setup::debug()
{
    //待实现
}

void Robot_WeaponSage_Setup::autoControl()
{
    //待实现
    /**
     * @brief 自动控制逻辑
     *  1.对于4个待取矛杆，硬编码四个位置
     *  2.当底盘靠位完成后，总状态机发来下降指令，执行下降
     *  3.当下降完成后，且底盘与武器架底部接触，则执行抓取
     *  4.当底盘后退到能将矛杆抬起的位置后，执行抬起
     */
	switch(now_state_)

    {
        case WeaponSage_Setup::STATE_DONE:
        {
            if(auto_ctrl_.auto_ctrl1)
            {
                auto_ctrl_.flag.aimposition_done = false;
                auto_ctrl_.flag.lowerclaw_done = false;
                auto_ctrl_.flag.grabclaw_done = false;
                auto_ctrl_.flag.lift_done = false;
                auto_ctrl_.auto_ctrl1 = false;
                now_state_ = WeaponSage_Setup::STATE_AIM_POSITION;
            }
            else
            {
                this->idle();
            }
            break;
        }
        case WeaponSage_Setup::STATE_AIM_POSITION:
        {
            auto_ctrl_.flag.aimposition_done = State_AimPosition(auto_ctrl_.pole_num);
            if(auto_ctrl_.flag.aimposition_done)
            {
                now_state_ = WeaponSage_Setup::STATE_LOWER_CLAW;
            }
            break;
        }

        case WeaponSage_Setup::STATE_LOWER_CLAW:
        {
            State_LowerClaw();
            Point2D now_claw_pos = this->getClawPos();
            if(abs(now_claw_pos.y - auto_ctrl_.tarch_height) < 0.001f)
            {
                auto_ctrl_.flag.lowerclaw_done = true;
            }
			if(auto_ctrl_.flag.lowerclaw_done)
			{
				now_state_=WeaponSage_Setup::STATE_GRAB_CLAW;
			}
            break;
        }

        case WeaponSage_Setup::STATE_GRAB_CLAW:
        {
            auto_ctrl_.flag.grabclaw_done = State_GrabClaw();
            if(auto_ctrl_.flag.grabclaw_done)
            {
                now_state_ = WeaponSage_Setup::STATE_LIFT_POSITION;
            }
            break;
        }

        case WeaponSage_Setup::STATE_LIFT_POSITION:
        {
            State_Lift();
            Point2D now_claw_pos = this->getClawPos();
            if(abs(now_claw_pos.y - auto_ctrl_.up_height) < 0.001f)
            {
                auto_ctrl_.flag.lift_done = true;    
            }
            if(auto_ctrl_.flag.lift_done)
            {
                now_state_ = WeaponSage_Setup::STATE_DONE;
                if(auto_ctrl_.pole_num < 3)
                {
                    auto_ctrl_.pole_num ++;
                }
            }
            break;
        }

        default:
            break;
    }
}

void Robot_WeaponSage_Setup::stop()
{
    //停止，电机不动
    this->setCtrlMode(WeaponSage::CURRENT_CONTROL);
    this->setTarget(0.0f, WeaponSage::Launch_Motor);
    this->setTarget(0.0f, WeaponSage::Claw_Motor);
    this->setTarget(0.0f, WeaponSage::Traverse_Motor);
    this->setTarget(0.0f, WeaponSage::Wrist_Motor);
}

bool Robot_WeaponSage_Setup::State_AimPosition(int pole_num)
{
    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
    this->setTarget( WeaponSage_Setup::weapon_pos[pole_num], WeaponSage::Traverse_Motor);
    Point2D claw_pos = this->getClawPos();
    if(fabs(claw_pos.y -  WeaponSage_Setup::weapon_pos[pole_num]) <0.001f)
    {
        return true;
    }
    else
    {
        return false;
    }
}
void Robot_WeaponSage_Setup::State_LowerClaw()
{
    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
	auto_ctrl_.tarch_height = initData_.max_launchHeight_;
    this->setTarget(auto_ctrl_.tarch_height, WeaponSage::Launch_Motor);
}

bool Robot_WeaponSage_Setup::State_GrabClaw()
{
    if(auto_ctrl_.auto_state_bool_S.is_matching)
    {
		
        this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
        
        this->setTarget(initData_.max_clawAngle_, WeaponSage::Claw_Motor);
       
    
        if(!auto_ctrl_.auto_state_bool_S.grab_start)
        {
            auto_ctrl_.auto_state_bool_S.grab_start = true;
            auto_ctrl_.auto_state_bool_S.grab_startTime = TimeStamp::getInstance().getSeconds();
        }

    
        if(ctrl_status_.now_times - auto_ctrl_.auto_state_bool_S.grab_startTime >= 2.0f)
        {
            return true;
        }
        else
        {
            return false;
        }
	}
	
}

bool Robot_WeaponSage_Setup::State_Lift()
{
    if(auto_ctrl_.auto_state_bool_S.is_moving == true)
    {
        this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
        auto_ctrl_.up_height = 0.7f * initData_.max_launchHeight_;
        this->setTarget(auto_ctrl_.up_height, WeaponSage::Launch_Motor);
    }

    Point2D current_pos = getClawPos();
    
    // 判断是否到达目标高度 (允许 1cm 的误差)
    if(fabs(current_pos.y - auto_ctrl_.up_height) < 0.01f)
    {
        return true;
    }
    else
    {
        return false;
    }
}

WeaponSage_InitData_S initData_=
{
    .max_launchHeight_ =0.420f,
    .max_clawAngle_ = 65.0f,
    .max_traverseLength_ = 0.450f,

    .wrist_gearRatio_ = 360.0f,
    .launch_Ratio_ = 0.1013056956038f,
    .claw_gearRatio_  =360.0f ,
    .traverse_Ratio_  = 0.0785210947199f,
    .max_wristMotorRPM_   =45.0f,
};
