#include "WeaponSage_Setup.h"

Robot_WeaponSage_Setup::Robot_WeaponSage_Setup(WeaponSage_InitData_S init_data)
    : Robot_WeaponSage(init_data), RtosTask("WeaponSage_Setup", 1)
{
    
}

void Robot_WeaponSage_Setup::loop()
{
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
            debug();
            break;
<<<<<<< Updated upstream
=======

        case WEAPONSAGE_AUTO_CONTROL:
            //��ʵ���Զ������߼�
            break;
>>>>>>> Stashed changes
        default:
            idle();
            break;
    }
}

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
    this->setTarget(-500.0f, WeaponSage::Traverse_Motor);
    float now_time_s = TimeStamp::getInstance().getSeconds();
    if(now_time_s - ctrl_status_.calibrate_startTime > 1.5f)
    {
        //relocate
        this->claw_Motor_->relocate_totalAngle(0.0f);
        this->traverse_Motor_->relocate_totalAngle(0.0f);
        this->launch_Motor_->relocate_totalAngle(0.0f);
        
        this->setTarget(0.0f, WeaponSage::Claw_Motor);
        this->setTarget(0.0f, WeaponSage::Traverse_Motor);

        ctrl_status_.is_calibrating = true;
    }
}
    //��ʵ��
}

void Robot_WeaponSage_Setup::manualControl()
{
    //��ʵ��
}

void Robot_WeaponSage_Setup::idle()
{
    //����״̬��ά�ֵ�ǰ״̬
}

<<<<<<< Updated upstream
void Robot_WeaponSage_Setup::stop()
{
    //ֹͣ���������
    this->setJointTarget(0.0f, WeaponSage::Launch_Motor);
    this->setJointTarget(0.0f, WeaponSage::Claw_Motor);
    this->setJointTarget(0.0f, WeaponSage::Traverse_Motor);
    this->setJointTarget(0.0f, WeaponSage::Wrist_Motor);
=======
void Robot_WeaponSage_Setup::debug()
{
    //��ʵ��
}

void Robot_WeaponSage_Setup::autoControl()
{
    //��ʵ��
    /**
     * @brief �Զ������߼�
     *  1.����4����ȡì�ˣ�Ӳ�����ĸ�λ��
     *  2.�����̿�λ��ɺ���״̬�������½�ָ�ִ���½�
     *  3.���½���ɺ��ҵ����������ܵײ��Ӵ�����ִ��ץȡ
     *  4.�����̺��˵��ܽ�ì��̧���λ�ú�ִ��̧��
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
                if(auto_ctrl_.pole_num < 4)
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
    //ֹͣ���������
    this->setTarget(0.0f, WeaponSage::Launch_Motor);
    this->setTarget(0.0f, WeaponSage::Claw_Motor);
    this->setTarget(0.0f, WeaponSage::Traverse_Motor);
    this->setTarget(0.0f, WeaponSage::Wrist_Motor);
>>>>>>> Stashed changes
}

bool Robot_WeaponSage_Setup::State_AimPosition(int pole_num)
{
    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
    this->setTarget(auto_ctrl_.Pole_pos[pole_num], WeaponSage::Traverse_Motor);
    Point2D claw_pos = this->getClawPos();
    if(fabs(claw_pos.y - auto_ctrl_.Pole_pos[pole_num]) <0.001f)
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
    this->setTarget(auto_ctrl_.claw_open_pos, WeaponSage::Launch_Motor);
  
    //����צ״̬
}

bool Robot_WeaponSage_Setup::State_GrabClaw()
{
      if(auto_ctrl_.auto_state_bool_S.is_matching)
    {
        this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
        auto_ctrl_.tarch_height = 0.5*initData_.max_launchHeight_;
        this->setTarget(auto_ctrl_.tarch_height, WeaponSage::Claw_Motor);
        auto_ctrl_.auto_state_bool_S.grab_start = true;
       
    }
    if(auto_ctrl_.auto_state_bool_S.grab_start==true)
    {
        auto_ctrl_.auto_state_bool_S.grab_start = false;
        auto_ctrl_.auto_state_bool_S.grab_startTime = TimeStamp::getInstance().getSeconds();
    }

    float currentTime = TimeStamp::getInstance().getSeconds();
    if(currentTime - auto_ctrl_.auto_state_bool_S.grab_startTime >= 0.2f)
    {
        return true;
    }
    else
    {
        return false;
    }
    //ץȡצ״̬
}

void Robot_WeaponSage_Setup::State_Lift()
{
    if(auto_ctrl_.auto_state_bool_S.is_moving == true)
    {
        this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
        auto_ctrl_.up_height = initData_.max_launchHeight_;
        this->setTarget(auto_ctrl_.up_height, WeaponSage::Launch_Motor);
    }
    //����״̬
}

WeaponSage_InitData_S initData_=
{
		.max_launchHeight_ = 1.0f,
		.max_clawAngle_ = 90.0f,
		.max_traverseLength_ = 1.0f,

		.wrist_gearRatio_ = 1.0f,
		.launch_Ratio_ = 1.0f,
		.claw_gearRatio_  = 1.0f,
		.traverse_Ratio_  = 1.0f,
        .max_wristMotorRPM_   = 100.0f,
};