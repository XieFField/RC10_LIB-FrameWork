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
    
/*如果本次状态不是对接自动DOCK而上次状态是DOCK把ARM打到水平（高优先级）*/
/*
待实现
*/

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
        case WEAPONSAGE_AUTO_CONTROL_CATCH:
		{	
            break;
	    }
        case WEAPONSAGE_AUTO_CONTROL_DOCK:
        {
            autoControl_dock();
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




/**
 * @brief 武器架校准流程
 * 3个夹爪的校准： 和上一版一样，往外张开方向的电流顶住限位后计时，计时完成后重定位
 * arm电机的校准： 和先前一样，上电会有机械限位，以上电位置为0度就行。
 *                 arm电机有两个目标位置，一个水平一个垂直，上电位置的0度不是任何其中的一个位置
 *                 在离开校准模式后，如果首次进入非校准和STOP外的模式(即status第一次为非calibrate和stop时候)，抬到竖直位置。
 * wrist电机校准： 上电先读取OID_Encoder，如果encoder的get_encoder_raw()返回0，说明还没收到第一帧数据，继续等待
 *                 如果返回的不为0了，调用get_angle()，为重定位角度，可能不是0~360的范围，你自己进行归一化处理
 * launch电机校准： 施加小的反向电流，计时后完成重定位
 */
void Robot_WeaponSage_Setup::calibrate()
{
	if(!ctrl_status_.is_calibrating)
	{
        this->setCtrlMode(WeaponSage::CURRENT_CONTROL);
        this->setTarget(100.0f, WeaponSage::Claw_1_Motor);
        this->setTarget(100.0f, WeaponSage::Claw_2_Motor);
        this->setTarget(100.0f, WeaponSage::Claw_3_Motor);
        ctrl_status_.calibrate_startTime = TimeStamp::getInstance().getSeconds();
        if(!auto_ctrl_.auto_state_bool_S.arm_enable)
        {
            this->Weapon_arm_enable();
            auto_ctrl_.auto_state_bool_S.arm_enable=true;
        }
        if(this->wrist_encoder_->get_encoder_raw()!=0)
        {
            float wrist_angle = this->wrist_encoder_->get_angle();
            NormalizeAngle(&wrist_angle);
            this->wrist_Motor_->relocate_totalAngle(wrist_angle); 
        }
        if(ctrl_status_.now_times-ctrl_status_.calibrate_startTime >= 2.0f) // 2秒的校准时间
        {
            this->setTarget(0.0f, WeaponSage::Claw_1_Motor);
            this->setTarget(0.0f, WeaponSage::Claw_2_Motor);
            this->setTarget(0.0f, WeaponSage::Claw_3_Motor);
            this->claw_1_Motor_->relocate_totalAngle(0.0f);
            this->claw_2_Motor_->relocate_totalAngle(0.0f);
            this->claw_3_Motor_->relocate_totalAngle(0.0f);
            this->launch_Motor_->relocate_totalAngle(0.0f);
            if(auto_ctrl_.auto_state_bool_S.arm_enable)
            {
                this->Weapon_arm_setZero();
            }
            ctrl_status_.is_calibrating = true;
        }

		
	}
	
}

float test_angle = 20.0f;


//这版手操逻辑：（暂时）
/**
 *  大体和之前的差不多，一些地方有变化
 *  还是右摇杆的y控制升降
 *  SWD控制夹爪开合，手操先设定为三个夹爪都一起控制
 *  右摇杆x轴，往左往右打一次，代表手腕逆顺时针方向转90度，，每执行一次转90度需要遥控回中后才能执行下一次转动。
 *  SWA控制arm的竖直和水平。两档
 *  
 *  SWA和SWD的状态切换和之前一样，使用异或方式，防止状态的跳变
 */
void Robot_WeaponSage_Setup::manualControl()
{

    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);

//     进入Manual模式时的状态绑定逻辑
    if(last_weaponSage_status_ != WEAPONSAGE_MANUAL_CONTROL)
    {
        // 获取当前爪子实际位置，判定逻辑状态
        float current_claw_theta = this->get_CurrentPos().claw_1_pos_; // 这里以claw_1为代表，假设三个爪子位置一致
        float current_arm_pos= this->get_CurrentPos().arm_pos_;
        
        int8_t current_claw_logical = (current_claw_theta > initData_.max_clawAngle_ * 0.5f) ? 1 : 0;
        int8_t current_arm_logical = (current_arm_pos > initData_.max_arm_angle_ * 0.5f) ? 1 : 0;

        // 记录状态
        ctrl_status_.last_manual_claw_state = current_claw_logical;
        ctrl_status_.last_arm_switch_state = current_arm_logical;

        // 计算偏移: offset = switch ^ state
        ctrl_status_.claw_switch_offset = (airjoy_data_.SWD & 0x01) ^ current_claw_logical;
        ctrl_status_.arm_switch_offset = (airjoy_data_.SWA & 0x01) ^ current_arm_logical;

        last_weaponSage_status_ = WEAPONSAGE_MANUAL_CONTROL;

        ctrl_status_.last_isClaw_tight = ctrl_status_.isClaw_tight;
        ctrl_status_.last_scroll_state = airjoy_data_.scroll_wheel;

        ctrl_status_.last_isArm_Vertical = ctrl_status_.isArm_Vertical;

        ctrl_status_.scroll_offset = (airjoy_data_.scroll_wheel & 0x01) ^ ctrl_status_.last_isClaw_tight; // 初始状态假设为0
       
    }


        
/*----------------------------------------遥感X控制wrist_motor---------------------------------------------------- */
            if(_tool_Abs(airjoy_data_.right_x) < 0.1)
                {
                    manual_ctrlForgrip_.changeTarget_state = false;
                    ctrl_status_.wrist_rotate_enable=true;
                }
            else if(airjoy_data_.right_x > 0.5f&&ctrl_status_.wrist_rotate_enable)
            {
                manual_ctrlForgrip_.changeTarget_state = true;
                target_pos_.wrist_pos_+=90.0f;
                ctrl_status_.wrist_rotate_enable=false;
            }
            else if(airjoy_data_.right_x < -0.5f&&ctrl_status_.wrist_rotate_enable)
            {
                manual_ctrlForgrip_.changeTarget_state = true;
                target_pos_.wrist_pos_-=90.0f;
                ctrl_status_.wrist_rotate_enable=false;
            }
/*----------------------------------------遥感Y控制launch_motor---------------------------------------------------- */
            if(_tool_Abs(airjoy_data_.right_y) < 0.1)
                manual_ctrlForgrip_.changeTarget_state = false;
            if(airjoy_data_.right_y > 0.5f)
                target_pos_.launch_pos_ += weapon_launch_rate;
            else if(airjoy_data_.right_y < -0.5f)
                target_pos_.launch_pos_ -= weapon_launch_rate;
            else
                target_pos_.launch_pos_ = target_pos_.launch_pos_;

            int8_t target_claw_logical = (airjoy_data_.SWD & 0x01) ^ ctrl_status_.claw_switch_offset;
            
            ctrl_status_.last_manual_claw_state = target_claw_logical;

            int8_t target_tight_logical = (airjoy_data_.scroll_wheel & 0x01) ^ ctrl_status_.scroll_offset;
            ctrl_status_.isClaw_tight = target_tight_logical;

            if(target_claw_logical == 0)
            {
                if(ctrl_status_.isClaw_tight)
                    {
                    target_pos_.claw_1_pos_ = 0.0f; //开爪子
                    target_pos_.claw_2_pos_ = 0.0f; 
                    target_pos_.claw_3_pos_ = 0.0f;
                    }
                else
                   {
                    target_pos_.claw_1_pos_ = test_angle; //不太紧爪子
                    target_pos_.claw_2_pos_ = test_angle;
                    target_pos_.claw_3_pos_ = test_angle;
                   }
            }
            else
            {
                target_pos_.claw_1_pos_ = initData_.max_clawAngle_; //紧爪子
                target_pos_.claw_2_pos_ = initData_.max_clawAngle_;
                target_pos_.claw_3_pos_ = initData_.max_clawAngle_;
            }

            int8_t target_arm_logical = (airjoy_data_.SWA & 0x01) ^ ctrl_status_.arm_switch_offset;
            ctrl_status_.last_arm_switch_state = target_arm_logical;
            int8_t target_arm_vertical_logical = (airjoy_data_.SWA & 0x01) ^ ctrl_status_.arm_switch_offset;
            if(auto_ctrl_.auto_state_bool_S.arm_enable&&target_arm_logical==1)
            {
                target_pos_.arm_pos_ = 90.0f;   // 这里的角度待定
            }else if(auto_ctrl_.auto_state_bool_S.arm_enable&&target_arm_logical==0)
            {
                target_pos_.arm_pos_ = 0.0f;  //这里角度也待定
            }

            manual_ctrlForgrip_.last_right_stick_x = airjoy_data_.right_x;
            manual_ctrlForgrip_.last_right_stick_y = airjoy_data_.right_y;
    



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
    this->setTarget(target_pos_.claw_1_pos_, WeaponSage::Claw_1_Motor);
    this->setTarget(target_pos_.claw_2_pos_, WeaponSage::Claw_2_Motor);
    this->setTarget(target_pos_.claw_3_pos_, WeaponSage::Claw_3_Motor);
    this->setTarget(target_pos_.arm_pos_, WeaponSage::Arm_Motor);
    this->setTarget(target_pos_.wrist_pos_, WeaponSage::Wrist_Motor);
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




    /**
     * @brief 抓取流程
     *        新版抓取杆时候不需要抬高，需降到最低，贴近后等待底盘停稳信号
     *        底盘停稳信号到达后夹取目标杆
     *        夹取完成后抬高到安全高度，完流程
     */
void Robot_WeaponSage_Setup::autoControl_catch()
{
	this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
    this->setTarget(0.0f, WeaponSage::Launch_Motor);      //先把架杆放置到最低位置
    if(auto_ctrl_.auto_state_bool_S.is_matching) //如果已经在对位了
    {
		
        auto_ctrl_.flag.is_clawed=this->Close_TargetClaw(); //夹取目标杆
		if(auto_ctrl_.flag.is_clawed)
		{
			auto_ctrl_.safe_height = 0.5* initData_.max_launchHeight_;
			this->setTarget(auto_ctrl_.safe_height, WeaponSage::Launch_Motor);      //抬高到安全高度,高度待调整
		}
        if(this->get_CurrentPos().launch_pos_>=auto_ctrl_.safe_height*0.98f) //如果已经抬高到位了
           {
              auto_ctrl_.flag.is_catched=true; //完成抓取流程
           }
    }
}

/**
 * @brief 虽然放在auto里面，但其实这部分应该算是一串动作链，没有和其他机构的交互
 *        但这部分你还是需要设计一个小状态机。
 * 
 *        收到开始执行动作的信号时候
 *        1.先保证目前arm处于水平。
 *        2.半松爪子，不让杆子能掉出去，但也没有抓住杆的状态2.
 *        3.根据@paramtarget_dock_的值，调整爪子到对应的高度
 *        4.紧爪子抓住杆。
 *        5根据SwD是否被切换状态，按下决定是否将arm打到竖直
 *        6.旋转完成后，将arm抬到竖直位置。
 *        7.完成后升降到预定位置(先定为中间位置，用于对接)
 *        8.完成进入idle状态
 * 
 *        在进入idle状态后，如果SWD状态被切换了(同manual里的逻辑)
 *        则切换arm的状态，如果当前竖直就变水平，反之亦然。
 * 
 *        srollweel控制arm的竖直和水平，这部分逻辑和manual里的松紧类似
 *        
 */
void Robot_WeaponSage_Setup::autoControl_dock()
{
    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
    if(auto_ctrl_.auto_state_bool_S.dock_start)
    {
        switch (now_state_)
        {
            case WeaponSage_Setup::STATE_START:
            {
                now_state_ = WeaponSage_Setup::STATE_ARM_MOVE;
                break;
            }
            case WeaponSage_Setup::STATE_ARM_MOVE:
            {
                this->setTarget(0.0f, WeaponSage::Arm_Motor);      //先把arm放置到水平位置
                if(abs(this->get_CurrentPos().arm_pos_)<0.02f ) //如果已经在水平位置了，进入下一个状态
                {   
                    auto_ctrl_.flag.is_moved=true;
                    now_state_ = WeaponSage_Setup::STATE_CLAW_ADJUST;
                }
                break;
            }
            case WeaponSage_Setup::STATE_CLAW_ADJUST:
            {
                this->Close_TargetClaw_Untight();
                if(abs(current_pos_.claw_1_pos_-target_pos_.claw_1_pos_)<0.02f&&
                   abs(current_pos_.claw_2_pos_-target_pos_.claw_2_pos_)<0.02f&&
                   abs(current_pos_.claw_3_pos_-target_pos_.claw_3_pos_)<0.02f) //如果已经调整好爪子了，进入下一个状态
                {
                    this->setTarget(target_dock_, WeaponSage::Launch_Motor);      //根据target_dock_的值调整爪子高度
                    if(abs(this->get_CurrentPos().launch_pos_-target_dock_)<0.02f) //如果已经调整到位了，进入下一个状态
                    {
                        this->Close_TargetClaw();
                       if(auto_ctrl_.flag.is_clawed)
                       {
                            if(target_dock_==WeaponSage_Setup:: HIGH) //如果是低位对接，直接进入下一个状态
                            {
                                target_pos_.wrist_pos_=180.0f;
                                this->setTarget(target_pos_.wrist_pos_, WeaponSage::Wrist_Motor);      
                            }else
                            {
                                //do nothing,保持当前手腕角度不变   
                            }
                            if(abs(current_pos_.wrist_pos_-target_pos_.wrist_pos_)<0.02f) //如果手腕调整到位了，进入下一个状态
                           {
                            now_state_ = WeaponSage_Setup::STATE_LAUNCH_MOVE;
                           }
                       }
                    }
                }
                break;   
            }                           
            case WeaponSage_Setup::STATE_LAUNCH_MOVE:
            {   
                this->setTarget(target_dock_, WeaponSage::Launch_Motor);      //抬高到预定位置
                if(abs(this->get_CurrentPos().launch_pos_-target_dock_)<0.02f) //如果已经调整到位了，进入下一个状态
                {
                    now_state_ = WeaponSage_Setup::STATE_DONE;
                }
                break;
            }
            case WeaponSage_Setup::STATE_DONE:
            {
                break;
            }
            default:
            {
                break;
            }
        }
        auto_ctrl_.auto_state_bool_S.dock_start=false; //重置开始信号
    }
    else
    {
        this->idle(); //如果没有开始信号了，就进入idle状态
    }
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

 bool Robot_WeaponSage_Setup::Close_TargetClaw()
 {

    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
   
    auto_ctrl_.claw_flag[0]=ctrl_status_.is_claw_1_closed;
    auto_ctrl_.claw_flag[1]=ctrl_status_.is_claw_2_closed;
    auto_ctrl_.claw_flag[2]=ctrl_status_.is_claw_3_closed;
    float target_claw_pos[3]={0.0f,0.0f,0.0f};
    if(!auto_ctrl_.claw_flag[0]&&!auto_ctrl_.claw_flag[1]&&!auto_ctrl_.claw_flag[2])
    {
        auto_ctrl_.flag.is_clawed=false;
        return false;
    }

    if(auto_ctrl_.claw_flag[0])
    {
        target_claw_pos[0]=initData_.max_clawAngle_;
    }
    if(auto_ctrl_.claw_flag[1])
    {
       target_claw_pos[1]=initData_.max_clawAngle_;
    }
    if(auto_ctrl_.claw_flag[2])
    {
        target_claw_pos[2]=initData_.max_clawAngle_;
    }
    this->setTarget(target_claw_pos[0], WeaponSage::Claw_1_Motor);
    this->setTarget(target_claw_pos[1], WeaponSage::Claw_2_Motor);
    this->setTarget(target_claw_pos[2], WeaponSage::Claw_3_Motor);
    target_pos_.claw_1_pos_=target_claw_pos[0];
    target_pos_.claw_2_pos_=target_claw_pos[1];
    target_pos_.claw_3_pos_=target_claw_pos[2];
    if(this->get_CurrentPos().claw_1_pos_>=target_claw_pos[0]*0.98f&&
       this->get_CurrentPos().claw_2_pos_>=target_claw_pos[1]*0.98f&&
       this->get_CurrentPos().claw_3_pos_>=target_claw_pos[2]*0.98f)
    {
        auto_ctrl_.flag.is_clawed=true;
        return true;
    } 
    else{
        auto_ctrl_.flag.is_clawed=false;
        return true;
    }
 }

void Robot_WeaponSage_Setup::Close_TargetClaw_Untight()
{
    float target_claw_pos[3] = {0.0f,0.0f,0.0f};
    for (int i=0;i<3;i++)       
    {
        if(auto_ctrl_.claw_flag[i])
        {
            target_claw_pos[i]=0.5*initData_.max_clawAngle_;
        }
        else
        {
            target_claw_pos[i]=initData_.max_clawAngle_;
        }
    }
    target_pos_.claw_1_pos_=target_claw_pos[0];
    target_pos_.claw_2_pos_=target_claw_pos[1];
    target_pos_.claw_3_pos_=target_claw_pos[2];
    this->setTarget(target_claw_pos[0], WeaponSage::Claw_1_Motor); //半松爪子
    this->setTarget(target_claw_pos[1], WeaponSage::Claw_2_Motor);    
    this->setTarget(target_claw_pos[2], WeaponSage::Claw_3_Motor);
}

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
