#include "WeaponSage_Setup.h"

namespace WeaponSage_Setup {
  	float weapon_pos[4]={0.0074f,0.1322f,0.2822f,0.3411f}; //四个矛杆对应的爪子高度
}

Robot_WeaponSage_Setup::Robot_WeaponSage_Setup(WeaponSage_InitData_S init_data)
    : Robot_WeaponSage(init_data), RtosTask("WeaponSage_Setup", 1)
{
    cam_z_ctrl_.set_param(camera_z_ctrl_params);
}
uint32_t WeaponSagestackHighWaterMark = 0;
void Robot_WeaponSage_Setup::loop()
{	
	ctrl_status_.now_times=TimeStamp::getInstance().getSeconds();
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
	if(!ctrl_status_.is_calibrating)
	{
		calibrate();
		weaponSage_status_=WEAPONSAGE_CALIBRATE;
	}
	
//	WeaponSagestackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

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
			autoControl();
            break;
	    }
        case WEAPONSAGE_CAMERA:
        {
            camera_mode();
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
//	auto_ctrl_.auto_state_bool_S.is_matching=Locate_Setup::getInstance()->ifSwitch1On();
	auto_ctrl_.auto_state_bool_S.is_matching= omni_flag;
}
int CNT=0;
float traverse_rate=0.002f;
float weapon_launch_rate=0.0002f;
float Kp_traverse=0.2f;

bool Robot_WeaponSage_Setup::is_new_z(float z_now)
{
    // 变化超过 0.5mm 视为新样本。
    if(_tool_Abs(z_now - cam_z_last_) > 0.0005f)
    {
        cam_z_last_ = z_now;
        return true;
    }

    return false;
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
		this->setTarget(-900.0f, WeaponSage::Traverse_Motor);
        this->setTarget(500.0f, WeaponSage::Claw_Motor);
        if(!auto_ctrl_.auto_state_bool_S.wrist_enable)
        {
            Weapon_wrist_enable();
			auto_ctrl_.auto_state_bool_S.wrist_enable=true;
        }
        if(ctrl_status_.now_times - ctrl_status_.calibrate_startTime > 2.0f)
        {
            //relocate
            this->claw_Motor_->relocate_totalAngle(0.0f);
            this->traverse_Motor_->relocate_totalAngle(0.0f);
            this->launch_Motor_->relocate_totalAngle(0.0f);
            
			if(auto_ctrl_.auto_state_bool_S.wrist_enable)
			{	
				this->Weapon_wrist_setzero();
            }
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

    // 进入Manual模式时的状态绑定逻辑
    if(last_weaponSage_status_ != WEAPONSAGE_MANUAL_CONTROL)
    {
        // 获取当前爪子实际位置，判定逻辑状态
        float current_claw_theta = this->getClawPos().theta;
        int8_t current_claw_logical = (current_claw_theta > initData_.max_clawAngle_ * 0.5f) ? 1 : 0;
        
        // 记录状态
        ctrl_status_.last_manual_claw_state = current_claw_logical;
        
        // 计算偏移: offset = switch ^ state
        ctrl_status_.claw_switch_offset = (airjoy_data_.SWD & 0x01) ^ current_claw_logical;

        last_weaponSage_status_ = WEAPONSAGE_MANUAL_CONTROL;
    }

    switch(airjoy_data_.SWA)
    {
        case 0x00:
        {
            //夹取武器
            // 计算当前应当的逻辑状态 logic = switch ^ offset
            int8_t target_claw_logical = (airjoy_data_.SWD & 0x01) ^ ctrl_status_.claw_switch_offset;
            
            ctrl_status_.last_manual_claw_state = target_claw_logical;

            if(target_claw_logical == 0)
                target_pos_.claw_pos_ = 0.0f; //开爪子
            else
                target_pos_.claw_pos_ = initData_.max_clawAngle_; //紧爪子

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
            target_pos_.wrist_pos_ = -90.0f;
			}
			manual_ctrlForgrip_.last_right_stick_x = airjoy_data_.right_x;
            manual_ctrlForgrip_.last_right_stick_y = airjoy_data_.right_y;

            break;
        }

        case 0x01:
        {
            //进攻模式
            
			// CNT++;
            // if(CNT>10)
			// {
				if(_tool_Abs(airjoy_data_.right_x) < 0.1)
                manual_ctrlForgrip_.changeTarget_state = false;

				else if(airjoy_data_.right_x > 0.5f)
				{
					manual_ctrlForgrip_.changeTarget_state = true;
					if(current_pos_.traverse_pos_>0.2f*initData_.max_traverseLength_&&current_pos_.traverse_pos_<0.8*initData_.max_traverseLength_)
					{
						target_pos_.traverse_pos_+=traverse_rate;
					}
					if(current_pos_.traverse_pos_<=0.2f*initData_.max_traverseLength_||current_pos_.traverse_pos_>=0.8*initData_.max_traverseLength_)
					{
						target_pos_.traverse_pos_+=traverse_rate*Kp_traverse;
					}
				}
				else if(airjoy_data_.right_x < -0.5f)
				{
					manual_ctrlForgrip_.changeTarget_state = true;
					if(current_pos_.traverse_pos_>0.2f*initData_.max_traverseLength_&&current_pos_.traverse_pos_<0.8*initData_.max_traverseLength_)
					{
						target_pos_.traverse_pos_-=traverse_rate;
					}
					if(current_pos_.traverse_pos_<=0.2f*initData_.max_traverseLength_||current_pos_.traverse_pos_>=0.8*initData_.max_traverseLength_)
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
				target_pos_.wrist_pos_ = 0.0f;
				}
				manual_ctrlForgrip_.last_right_stick_x = airjoy_data_.right_x;
				manual_ctrlForgrip_.last_right_stick_y = airjoy_data_.right_y;
				CNT=0;

			// }
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
    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);

    // 首次进入DEBUG: 锁定当前姿态，后续只执行外部下发的launch目标。
    if(last_weaponSage_status_ != WEAPONSAGE_DEBUG)
    {
        this->last_pos_ = this->get_CurrentPos();
        this->target_pos_ = this->last_pos_;
        last_weaponSage_status_ = WEAPONSAGE_DEBUG;
    }

    if(debug_launch_target_valid_)
    {
        float launch_target = debug_launch_target_;
        if(launch_target < 0.0f)
            launch_target = 0.0f;
        else if(launch_target > initData_.max_launchHeight_)
            launch_target = initData_.max_launchHeight_;

        target_pos_.launch_pos_ = launch_target;
    }

    this->setTarget(target_pos_.launch_pos_, WeaponSage::Launch_Motor);
    this->setTarget(target_pos_.claw_pos_, WeaponSage::Claw_Motor);
    this->setTarget(target_pos_.traverse_pos_, WeaponSage::Traverse_Motor);
    this->setTarget(target_pos_.wrist_pos_, WeaponSage::Wrist_Motor);
}

void Robot_WeaponSage_Setup::camera_mode()
{
    if(!weapon_CameraStart) //主状态机没有给武器架相机模式触发信号，保持当前姿态不变，并锁定航向。
    {
        this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
        idle();
        return;
    }

    this->setCtrlMode(WeaponSage::CAMERA_MIX_CONTROL);// 相机模式下 launch 始终走 RPM 混合控制

    // 首次进入相机模式时锁定当前姿态基准。
    if(last_weaponSage_status_ != WEAPONSAGE_CAMERA)
    {
        this->last_pos_ = this->get_CurrentPos();// 读取当前关节位置。

        this->target_pos_ = this->last_pos_;// 用当前关节位置初始化目标位置。

        camera_weapon_done_ = false;// 清空相机流程完成标志。

        camera_z_done_ = false;// 清空相机流程完成标志。

        cam_z_run_ = false;// 清空 z 过程运行位。

        cam_z_hold_ = this->last_pos_.launch_pos_;// 对齐 z 目标缓存。

        cam_z_last_ = camera_z_ref_;// 对齐相机 z 样本。

        cam_z_rpm_ = 0.0f;// 清空相机 z 速度指令。

        cam_z_ctrl_.reset(this->last_pos_.launch_pos_);// 重置 z 控制器状态。

        last_weaponSage_status_ = WEAPONSAGE_CAMERA;// 更新上一次状态。
    }

    // 当底盘请求武器预对接姿态时执行机构动作。
    if(camera_weapon_req_)
    {
        target_pos_.claw_pos_ = initData_.max_clawAngle_;// 夹爪夹紧到上限角。

        target_pos_.traverse_pos_ = 0;// 导轨打到最边上。

        target_pos_.wrist_pos_ = 0.0f;// 腕部翻转到与其他流程一致的姿态。
        
        this->setTarget(target_pos_.claw_pos_, WeaponSage::Claw_Motor);// 下发夹爪目标。
        
        this->setTarget(target_pos_.traverse_pos_, WeaponSage::Traverse_Motor);// 下发导轨目标。

        this->setTarget(target_pos_.wrist_pos_, WeaponSage::Wrist_Motor);// 下发腕部目标。
        
        WeaponSage::WeaponSage_Pos_S now_pos = this->get_CurrentPos();// 读取当前姿态用于到位判定。

        float wrist_err = _tool_Abs(now_pos.wrist_pos_ - target_pos_.wrist_pos_);// 计算腕部误差。

        // 到位时返回 weapon_finished。
        camera_weapon_done_ = (wrist_err < 5.0f);
    }
    else
    {
        camera_weapon_done_ = false;// 未请求武器预对接时清除完成位。
    }

    // 新请求触发时锁存当前目标。
    if(camera_z_req_)
    {
        cam_z_hold_ = camera_z_ref_;
        cam_z_run_ = true;
    }

    // launch 在相机模式下始终按缓存目标做 RPM 闭环保持。
    float z_ref = constrain(cam_z_hold_, 0.0f, initData_.max_launchHeight_);
    target_pos_.launch_pos_ = z_ref;

    // 同步位置环目标角，确保目标缓存与当前参考一致。
    this->setTarget(target_pos_.launch_pos_, WeaponSage::Launch_Motor);

    // launch 线速度（已含正方向约定）。
    float z_vel = this->get_launchVel();

    // 仅在样本变化时执行一次融合。
    bool cam_new = is_new_z(camera_z_ref_);

    // 外环输出目标 rpm。
    float rpm_cmd = cam_z_ctrl_.run_step(z_ref, camera_z_ref_, cam_new, z_vel);
    cam_z_rpm_ = rpm_cmd;

    // 下发 launch 速度命令。
    this->set_launchMotorSpeed(rpm_cmd);

    // 仅在有 z 请求时回传完成位；无请求时清零。
    camera_z_done_ = camera_z_req_ ? cam_z_ctrl_.is_done() : false;

    // 持续保持当前四轴目标，避免状态切换时跳变。
    this->setTarget(target_pos_.claw_pos_, WeaponSage::Claw_Motor);

    // 持续保持当前四轴目标，避免状态切换时跳变。
    this->setTarget(target_pos_.traverse_pos_, WeaponSage::Traverse_Motor);

    // 持续保持当前四轴目标，避免状态切换时跳变。
    this->setTarget(target_pos_.wrist_pos_, WeaponSage::Wrist_Motor);

    if(camera_weapon_done_)
        weapon_CameraStart = false;// 预对接完成后复位主状态机触发位，准备下一次对接流程。
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
	
	this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
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
            
                now_state_ = WeaponSage_Setup::STATE_AIM_POSITION;
				
				this->setTarget(0.8f*initData_.max_launchHeight_, WeaponSage::Launch_Motor);
				this->setTarget(-90.0f, WeaponSage::Wrist_Motor);
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
				auto_ctrl_.auto_ctrl1 = false;
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
    if(fabs(claw_pos.x -  WeaponSage_Setup::weapon_pos[pole_num]) <0.001f)
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
	 if(auto_ctrl_.auto_state_bool_S.is_matching)
    {
		this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
		auto_ctrl_.tarch_height= 0.0193959419f;
		this->setTarget(auto_ctrl_.tarch_height, WeaponSage::Launch_Motor);
	}
}

bool Robot_WeaponSage_Setup::State_GrabClaw()
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
	


bool Robot_WeaponSage_Setup::State_Lift()
{
    
        this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
        auto_ctrl_.up_height =   initData_.max_launchHeight_;
        this->setTarget(auto_ctrl_.up_height, WeaponSage::Launch_Motor);
    

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
    .max_launchHeight_ =0.329f,
    .max_clawAngle_ = 65.0f,
    .max_traverseLength_ = 0.450f,

    .wrist_gearRatio_ = 360.0f,
    .launch_Ratio_ = 0.098482549317147f,
    .claw_gearRatio_  =360.0f ,
    .traverse_Ratio_  = 0.0785210947199f,
    .max_wristMotorRPM_   =45.0f,
};
