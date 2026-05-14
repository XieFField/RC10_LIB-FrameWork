#include "WeaponSage_Setup.h"

namespace WeaponSage_Setup {
  	float weapon_pos[4]={0.0074f,0.1322f,0.2822f,0.3411f}; //四个矛杆对应的爪子高度
}

Robot_WeaponSage_Setup::Robot_WeaponSage_Setup(WeaponSage_InitData_S init_data)
    : Robot_WeaponSage(init_data), RtosTask("WeaponSage_Setup", 1)
{
}
uint32_t WeaponSagestackHighWaterMark = 0;
void Robot_WeaponSage_Setup::loop()
{	
	ctrl_status_.now_times=TimeStamp::getInstance().getSeconds();
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
	
	
	
//	if((wrist_Motor_->getErrorNum()==0x00||!auto_ctrl_.auto_state_bool_S.wrist_enable))
//	{	               
//            Weapon_wrist_enable();
//			auto_ctrl_.auto_state_bool_S.wrist_enable=true;
//	}
    
	
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
float traverse_rate=0.0002f;
float weapon_launch_rate=0.0002f;
float Kp_traverse=0.5f;





void Robot_WeaponSage_Setup::calibrate()
{

	
}

float test_angle = 20.0f;

void Robot_WeaponSage_Setup::manualControl()
{
//    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);

//    // 进入Manual模式时的状态绑定逻辑
//    if(last_weaponSage_status_ != WEAPONSAGE_MANUAL_CONTROL)
//    {
//        // 获取当前爪子实际位置，判定逻辑状态
//        float current_claw_theta = this->getClawPos().theta;
//        int8_t current_claw_logical = (current_claw_theta > initData_.max_clawAngle_ * 0.5f) ? 1 : 0;
//        
//        // 记录状态
//        ctrl_status_.last_manual_claw_state = current_claw_logical;
//        
//        // 计算偏移: offset = switch ^ state
//        ctrl_status_.claw_switch_offset = (airjoy_data_.SWD & 0x01) ^ current_claw_logical;

//        last_weaponSage_status_ = WEAPONSAGE_MANUAL_CONTROL;

//        ctrl_status_.last_isClaw_tight = ctrl_status_.isClaw_tight;

//        ctrl_status_.last_scroll_state = airjoy_data_.scroll_wheel;

//        ctrl_status_.scroll_offset = (airjoy_data_.scroll_wheel & 0x01) ^ ctrl_status_.last_isClaw_tight; // 初始状态假设为0
//    }

//    switch(airjoy_data_.SWA)
//    {
//        case 0x00:
//        {
//            if(_tool_Abs(airjoy_data_.right_x) < 0.1)
//                manual_ctrlForgrip_.changeTarget_state = false;

//            else if(airjoy_data_.right_x > 0.5f)
//            {
//                manual_ctrlForgrip_.changeTarget_state = true;
//                if(current_pos_.traverse_pos_>0.2f*initData_.max_traverseLength_&&current_pos_.traverse_pos_<0.8*initData_.max_traverseLength_)
//                {
//                    target_pos_.traverse_pos_+=traverse_rate;
//                }
//                if(current_pos_.traverse_pos_<=0.2f*initData_.max_traverseLength_||current_pos_.traverse_pos_>=0.8*initData_.max_traverseLength_)
//                {
//                    target_pos_.traverse_pos_+=traverse_rate*Kp_traverse;
//                }
//            }
//            else if(airjoy_data_.right_x < -0.5f)
//            {
//                manual_ctrlForgrip_.changeTarget_state = true;
//                if(current_pos_.traverse_pos_>0.2f*initData_.max_traverseLength_&&current_pos_.traverse_pos_<0.8*initData_.max_traverseLength_)
//                {
//                    target_pos_.traverse_pos_-=traverse_rate;
//                }
//                if(current_pos_.traverse_pos_<=0.2f*initData_.max_traverseLength_||current_pos_.traverse_pos_>=0.8*initData_.max_traverseLength_)
//                {
//                    target_pos_.traverse_pos_-=traverse_rate*Kp_traverse;
//                }
//            }


//            if(airjoy_data_.right_y > 0.5f)
//                target_pos_.launch_pos_ += weapon_launch_rate;
//            else if(airjoy_data_.right_y < -0.5f)
//                target_pos_.launch_pos_ -= weapon_launch_rate;
//            else
//                target_pos_.launch_pos_ = target_pos_.launch_pos_;


//            if(auto_ctrl_.auto_state_bool_S.wrist_enable)
//            {
//                target_pos_.wrist_pos_ = 90.0f;  
//            }

//            int8_t target_claw_logical = (airjoy_data_.SWD & 0x01) ^ ctrl_status_.claw_switch_offset;
//            
//            ctrl_status_.last_manual_claw_state = target_claw_logical;

//            int8_t target_tight_logical = (airjoy_data_.scroll_wheel & 0x01) ^ ctrl_status_.scroll_offset;
//            ctrl_status_.isClaw_tight = target_tight_logical;

//            if(target_claw_logical == 0)
//            {
//                if(ctrl_status_.isClaw_tight)
//                    target_pos_.claw_pos_ = 0.0f; //开爪子
//                else
//                    target_pos_.claw_pos_ = test_angle; //不太紧爪子
//            }
//            else
//                target_pos_.claw_pos_ = initData_.max_clawAngle_; //紧爪子



//            manual_ctrlForgrip_.last_right_stick_x = airjoy_data_.right_x;
//            manual_ctrlForgrip_.last_right_stick_y = airjoy_data_.right_y;
//            break;
//        }

//        case 0x01:
//        {
//            //进攻模式

//				if(_tool_Abs(airjoy_data_.right_x) < 0.1)
//                manual_ctrlForgrip_.changeTarget_state = false;

//				else if(airjoy_data_.right_x > 0.5f)
//				{
//					manual_ctrlForgrip_.changeTarget_state = true;
//					if(current_pos_.traverse_pos_>0.2f*initData_.max_traverseLength_&&current_pos_.traverse_pos_<0.8*initData_.max_traverseLength_)
//					{
//						target_pos_.traverse_pos_+=traverse_rate;
//					}
//					if(current_pos_.traverse_pos_<=0.2f*initData_.max_traverseLength_||current_pos_.traverse_pos_>=0.8*initData_.max_traverseLength_)
//					{
//						target_pos_.traverse_pos_+=traverse_rate*Kp_traverse;
//					}
//				}
//				else if(airjoy_data_.right_x < -0.5f)
//				{
//					manual_ctrlForgrip_.changeTarget_state = true;
//					if(current_pos_.traverse_pos_>0.2f*initData_.max_traverseLength_&&current_pos_.traverse_pos_<0.8*initData_.max_traverseLength_)
//					{
//						target_pos_.traverse_pos_-=traverse_rate;
//					}
//					if(current_pos_.traverse_pos_<=0.2f*initData_.max_traverseLength_||current_pos_.traverse_pos_>=0.8*initData_.max_traverseLength_)
//					{
//						target_pos_.traverse_pos_-=traverse_rate*Kp_traverse;
//					}
//				}


//				if(airjoy_data_.right_y > 0.5f)
//					target_pos_.launch_pos_ += weapon_launch_rate;
//				else if(airjoy_data_.right_y < -0.5f)
//					target_pos_.launch_pos_ -= weapon_launch_rate;
//				else
//					target_pos_.launch_pos_ = target_pos_.launch_pos_;
//				if(auto_ctrl_.auto_state_bool_S.wrist_enable)
//				{
//				    target_pos_.wrist_pos_ = 0.0f;
//				}

//				manual_ctrlForgrip_.last_right_stick_x = airjoy_data_.right_x;
//				manual_ctrlForgrip_.last_right_stick_y = airjoy_data_.right_y;


//            break; 
//        }

//        default:
//        {
//            idle();
//            break;
//        }
//    }

//    this->setTarget(target_pos_.launch_pos_, WeaponSage::Launch_Motor);
//    this->setTarget(target_pos_.claw_1_pos_, WeaponSage::Claw_1_Motor);
//    this->setTarget(target_pos_.claw_2_pos_, WeaponSage::Claw_2_Motor);
//    this->setTarget(target_pos_.claw_3_pos_, WeaponSage::Claw_3_Motor);
//    this->setTarget(target_pos_.wrist_pos_, WeaponSage::Wrist_Motor);
//    this->setTarget(target_pos_.arm_pos_, WeaponSage::Arm_Motor);
}

void Robot_WeaponSage_Setup::idle()
{
	
}

void Robot_WeaponSage_Setup::debug()
{
//    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);

//    // 首次进入DEBUG: 锁定当前姿态，后续只执行外部下发的launch目标。
//    if(last_weaponSage_status_ != WEAPONSAGE_DEBUG)
//    {
//        this->last_pos_ = this->get_CurrentPos();
//        this->target_pos_ = this->last_pos_;
//        last_weaponSage_status_ = WEAPONSAGE_DEBUG;
//    }

//    if(debug_launch_target_valid_)
//    {
//        float launch_target = debug_launch_target_;
//        if(launch_target < 0.0f)
//            launch_target = 0.0f;
//        else if(launch_target > initData_.max_launchHeight_)
//            launch_target = initData_.max_launchHeight_;

//        target_pos_.launch_pos_ = launch_target;
//    }

//    this->setTarget(target_pos_.launch_pos_, WeaponSage::Launch_Motor);
//    this->setTarget(target_pos_.claw_pos_, WeaponSage::Claw_Motor);
//    this->setTarget(target_pos_.traverse_pos_, WeaponSage::Traverse_Motor);
//    this->setTarget(target_pos_.wrist_pos_, WeaponSage::Wrist_Motor);
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
	
	
}

void Robot_WeaponSage_Setup::stop()
{
    //停止，电机不动
    this->setCtrlMode(WeaponSage::CURRENT_CONTROL);
    this->setTarget(0.0f, WeaponSage::Launch_Motor);
    this->setTarget(0.0f, WeaponSage::Claw_1_Motor);
    this->setTarget(0.0f, WeaponSage::Claw_2_Motor);
    this->setTarget(0.0f, WeaponSage::Claw_3_Motor);
    this->setTarget(0.0f, WeaponSage::Wrist_Motor);
}

 /**
     * @brief 对准位置状态
     *  1.根据传入的杆号，设置对应的爪子位置
     *  2.判断当前爪子位置是否到达目标位置，返回
     */


WeaponSage_InitData_S initData_=
{
    .max_launchHeight_ =0.329f,
    .max_clawAngle_ = 65.0f,
    .max_arm_angle_ = 135.0f,
    .max_wrist_angle_ = 360.0f,
	.max_arm_rate_ =45.0f,
	
    .wrist_gearRatio_ = 360.0f,
    .launch_Ratio_ = 0.098482549317147f,
    .claw_gearRatio_  =360.0f ,
    .arm_gearRatio_ = 360.0f

};
