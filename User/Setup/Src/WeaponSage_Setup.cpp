#include "WeaponSage_Setup.h"

namespace WeaponSage_Setup {
  	float weapon_pos[4]={0.0074f,0.1322f,0.2822f,0.3411f}; //四个矛杆对应的爪子高度
}

Robot_WeaponSage_Setup::Robot_WeaponSage_Setup(WeaponSage_InitData_S init_data)
    : Robot_WeaponSage(init_data), RtosTask("WeaponSage_Setup", 1)
{
}

float lift_pitch = -5.0f;
float stend_up = -92.0f;

uint32_t WeaponSagestackHighWaterMark = 0;
uint8_t relocate_mid= 0;
volatile  float wrist_encoder_angle_222 = 0.0f;
volatile uint8_t encoder_cnt = 0;
float test_target_arm = 0.0f;
uint8_t reverse___ = 1;
uint8_t index = 0;
uint8_t dm_zero_____ = 0;
void Robot_WeaponSage_Setup::loop()
{	
	ctrl_status_.now_times=TimeStamp::getInstance().getSeconds();
	#if !USE_RC10_AIRJOY
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
	#else
	communication::Lora_communication::GetInstance()->update_airjoy_data(&airjoy_data_);
	#endif
//	weaponSage_status_=WEAPONSAGE_STOP;
	encoder_cnt++;
	if(encoder_cnt > 20) 
	{
		wrist_encoder_->read_angle_cmd();
		encoder_cnt = 0;
	}
	if(relocate_mid < 10  && relocate_mid !=0)
	{
		relocate_mid++;
		wrist_encoder_->set_mid_angle();
	}
	else if(relocate_mid == 100)
	{
		relocate_mid = 0;
		wrist_encoder_->set_reverse(reverse___);
	}
	
	if(dm_zero_____)
	{
		this->Weapon_arm_setZero();
	}
    
	wrist_encoder_angle_ = this->wrist_encoder_->get_angle();
	wrist_encoder_angle_222 = this->wrist_encoder_->get_angle();

	
	if((arm_Motor_->getErrorNum()==0x00||!auto_ctrl_.auto_state_bool_S.arm_enable))
	{	               
		Weapon_arm_enable();
		auto_ctrl_.auto_state_bool_S.arm_enable=true;
	}

    if(this->wrist_Motor_->get_MotorState() !=0x02||!auto_ctrl_.auto_state_bool_S.wrist_enable)
    {
        this->Weapon_wrist_enable();
        auto_ctrl_.auto_state_bool_S.wrist_enable=true;
    }
    {
        this->Weapon_wrist_enable();
        auto_ctrl_.auto_state_bool_S.wrist_enable=true;
    }
	
    
/*如果本次状态不是对接自动DOCK而上次状态是DOCK把ARM打到水平（高优先级）*/
/*
待实现
*/
    Judge_launch_status();
    Judge_wrist_status();

	if(!ctrl_status_.is_calibrating)
	{
		calibrate();
		weaponSage_status_=WEAPONSAGE_CALIBRATE;
	}
//	
//	WeaponSagestackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
	if(index == 1)
		weaponSage_status_ = WEAPONSAGE_DEBUG;
    switch(weaponSage_status_)
    {
        case WEAPONSAGE_MANUAL_CONTROL:

            #if !USE_RC10_AIRJOY
            manualControl();
            #else
            if(last_weaponSage_status_!=WEAPONSAGE_MANUAL_CONTROL)
            {
                manual_RC10_ctrlForgrip_.is_Dpad_Done=false;
            }
            manual_RC10_ctrlForgrip_.is_Dpad_Done=D_pad_acting();
            if(manual_RC10_ctrlForgrip_.is_Dpad_Done)
            {
                manualControl();
                manual_RC10_ctrlForgrip_.last_Dpad_acting_state=0;
            }
            #endif
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
        case WEAPONSAGE_AUTOCONTROL:
        {
            autoControl();
            break;
        }
        case WEAPONSAGE_MANUAL_LOW_LEVEL:
        {
            semi_low_level();
            break;
        }
		case WEAPONSAGE_CALIBRATE:
		{
			//calibrate();
			
            break;
	    }

        case WEAPONSAGE_SEMI_AUTO_CONTROL:
        {
            semi_autoControl();
            break;
        }
			
        case WEAPONSAGE_KFS_IDLE:
        {
            kfs_idle();
            break;
        }
        
        case WEAPONSAGE_TEST_POINT:
        {
            test_point();
        }
        default:
            idle();
            break;
    }
	

//	this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
//	if(this->arm_Motor_->getErrorNum()==0x01){
//		this->setArm_angle(90.0f);
//	}
	if((arm_Motor_->getErrorNum()==0x00||!auto_ctrl_.auto_state_bool_S.arm_enable))
	{	               
		Weapon_arm_enable();
		auto_ctrl_.auto_state_bool_S.arm_enable=true;
	}
	this->update();

}
int CNT=0;
float traverse_rate=0.0002f;
float weapon_launch_rate=0.0001f;
float Kp_traverse=0.5f;

/**
 * @brief 夹爪位置获取姿态
 */
void Robot_WeaponSage_Setup::test_point()
{
    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
    this->setArm_angle_slow(lift_pitch);
    this->setLaunch_angle(0.0f);
}

/**
 * @brief 二区时候的空闲状态
 */
void Robot_WeaponSage_Setup::kfs_idle()
{
    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
    this->setLaunch_angle(initData_.max_launchHeight_);
    if(current_pos_.launch_pos_ > initData_.max_launchHeight_-0.1f 
        && is_arm_90)
    {
        this->setArm_angle_slow(lift_pitch);
    }
}


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

float read_encoder = 0.0f;
float angle_0_360 = 0.0f;
float relocate_wrist_totalAngle__ = 0.0f;
int8_t dm_zero_ = 1;
void Robot_WeaponSage_Setup::calibrate()
{
	if(!ctrl_status_.is_calibrating)
	{
        this->setCtrlMode(WeaponSage::CURRENT_CONTROL);
		this->claw_1_Motor_->setTargetCurrent(1000.0f);
		this->claw_2_Motor_->setTargetCurrent(1000.0f);
		this->claw_3_Motor_->setTargetCurrent(1000.0f);
		this->launch_Motor_->setTargetCurrent(500.0f);
		if(!ctrl_status_.calibrate_start)
		{
			ctrl_status_.calibrate_startTime = TimeStamp::getInstance().getSeconds();
			ctrl_status_.calibrate_start = true;
		}
        if(!auto_ctrl_.auto_state_bool_S.arm_enable)
        {
            this->Weapon_arm_enable();
            auto_ctrl_.auto_state_bool_S.arm_enable=true;
        }
        if(!auto_ctrl_.auto_state_bool_S.el05_wrist_enable)
        {
            this->Weapon_wrist_enable();
            auto_ctrl_.auto_state_bool_S.el05_wrist_enable=true;
        }
        if(this->wrist_encoder_->get_encoder_raw()!=0)
       {
			float relocate_wirst_angle = MotorTotalAngle_to_Realpos(wrist_encoder_angle_, WeaponSage::Motor_Type_E::Wrist_Motor);
			read_encoder = relocate_wirst_angle;
			float adjust=normalize_deg_0_360(relocate_wirst_angle);
			angle_0_360 = adjust;
		       float relocate_wrist_totalAngle =Realpos_to_MotorTotalAngle( adjust, WeaponSage::Motor_Type_E::Wrist_Motor);
			relocate_wrist_totalAngle__ = relocate_wrist_totalAngle;
//            this->wrist_Motor_->relocate_totalAngle(relocate_wrist_totalAngle); 
        }
        if(ctrl_status_.now_times-ctrl_status_.calibrate_startTime >= 2.0f) // 2秒的校准时间
        {
            this->claw_1_Motor_->relocate_totalAngle(2.0f);
            this->claw_2_Motor_->relocate_totalAngle(2.0f);
            this->claw_3_Motor_->relocate_totalAngle(2.0f);

            this->launch_Motor_->relocate_totalAngle(0.0f);

			if(auto_ctrl_.auto_state_bool_S.arm_enable)
			{
				this->setArm_angle(stend_up);
			}

            if(ctrl_status_.now_times-ctrl_status_.calibrate_startTime >= 2.2f)
            {
                this->claw_1_Motor_->setTargetCurrent(0.0f);
                this->claw_2_Motor_->setTargetCurrent(0.0f);
                this->claw_3_Motor_->setTargetCurrent(0.0f);
                this->launch_Motor_->setTargetCurrent(0.0f);
                ctrl_status_.is_calibrating = true;
            }
            
        }

		
	}
	
}

float test_angle = 27.0f;
uint8_t round_cnt =0;
float test_launch = 0.0625674725;

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
     #if !USE_RC10_AIRJOY
//     进入Manual模式时的状态绑定逻辑
	this->idle();
    if(last_weaponSage_status_ != WEAPONSAGE_MANUAL_CONTROL)
    {
        this->setClawSpeedOutLimit(3000.0f);
        // 获取当前爪子实际位置，判定逻辑状态
        float current_claw_theta = fmax(fmax(current_pos_.claw_1_pos_,current_pos_.claw_2_pos_),current_pos_.claw_3_pos_); // 这里以claw_1为代表，假设三个爪子位置一致
        float current_arm_pos= this->get_CurrentPos().arm_pos_;
        
        int8_t current_claw_logical = (current_claw_theta > initData_.max_clawAngle_ * 0.25f) ? 1 : 0;
        int8_t current_arm_logical = (current_arm_pos > initData_.max_arm_angle_ * 0.25f) ? 1 : 0;

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


        
/*-d..........---------------------------------------遥感X控制wrist_motor---------------------------------------------------- */
//e
    if(_tool_Abs(airjoy_data_.right_x) < 0.1)
    {
        manual_ctrlForgrip_.changeTarget_state = false;
        ctrl_status_.wrist_rotate_enable=true;
    }
    else if(airjoy_data_.right_x > 0.5f&&ctrl_status_.wrist_rotate_enable&&auto_ctrl_.auto_state_bool_S.wrist_enable)
    {
        manual_ctrlForgrip_.changeTarget_state = true;
        float temp =normalize_deg_0_360(current_pos_.wrist_pos_+90.0f+45.0f);
        float k = temp / 90.0f;
            round_cnt =floor(k);
        target_pos_.wrist_pos_=round_cnt*90.0f;
        ctrl_status_.wrist_rotate_enable=false;
    }
    else if(airjoy_data_.right_x < -0.5f&&ctrl_status_.wrist_rotate_enable&&auto_ctrl_.auto_state_bool_S.wrist_enable)
    {
        manual_ctrlForgrip_.changeTarget_state = true;
            float temp =normalize_deg_0_360(current_pos_.wrist_pos_-90.0f+45.0f);
        float k = temp / 90.0f;
        uint8_t round_cnt =floor(k);
        target_pos_.wrist_pos_=round_cnt*90.0f;
        ctrl_status_.wrist_rotate_enable=false;
    }else{
        target_pos_.wrist_pos_ = target_pos_.wrist_pos_;
    }
	
	
/*----------------------------------------遥感Y控制launch_motor---------------------------------------------------- */
    if(_tool_Abs(airjoy_data_.right_y) < 0.1)
        manual_ctrlForgrip_.changeTarget_state = false;
    if(airjoy_data_.right_y > 0.5f&&target_pos_.launch_pos_ <=initData_.max_launchHeight_)
    {
        
        target_pos_.launch_pos_ += weapon_launch_rate;			
    }
    else if(airjoy_data_.right_y < -0.5f)
    {

        target_pos_.launch_pos_ -= weapon_launch_rate;	
    }
    else
        target_pos_.launch_pos_ = target_pos_.launch_pos_;

    if(target_pos_.launch_pos_ > initData_.max_launchHeight_)
        target_pos_.launch_pos_ = initData_.max_launchHeight_;
    else if(target_pos_.launch_pos_ < 0.0f)
        target_pos_.launch_pos_ = 0.0f;

    int8_t target_claw_logical = (airjoy_data_.SWD & 0x01) ^ ctrl_status_.claw_switch_offset;
    
    ctrl_status_.last_manual_claw_state = target_claw_logical;

    int8_t target_tight_logical = (airjoy_data_.scroll_wheel & 0x01) ^ ctrl_status_.scroll_offset;
    ctrl_status_.isClaw_tight = target_tight_logical;

    if(target_claw_logical == 0)
    {
        
            target_pos_.claw_1_pos_ =0.0f; //开爪子
            target_pos_.claw_2_pos_ = 0.0f; 
            target_pos_.claw_3_pos_ = 0.0f;

        
    }
    else
    {
        if(ctrl_status_.isClaw_tight)
        {
        target_pos_.claw_1_pos_ = initData_.max_clawAngle_; //紧爪子
        target_pos_.claw_2_pos_ = initData_.max_clawAngle_;
        target_pos_.claw_3_pos_ = initData_.max_clawAngle_;
        }else
        {
            
            target_pos_.claw_1_pos_ = test_angle; //不太紧爪子
            target_pos_.claw_2_pos_ = test_angle;
            target_pos_.claw_3_pos_ = test_angle;
        }
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

    #else
    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
    if(last_weaponSage_status_ != WEAPONSAGE_MANUAL_CONTROL||manual_RC10_ctrlForgrip_.last_Dpad_acting_state!=0)
    {
        this->setClawSpeedOutLimit(3000.0f);
		//this->idle();
        // 获取当前爪子实际位置，判定逻辑状态
        float current_claw_theta = fmax(fmax(current_pos_.claw_1_pos_,current_pos_.claw_2_pos_),current_pos_.claw_3_pos_); // 以最大闭合角为为夹爪的角度进行计算；
        float current_arm_pos= this->get_CurrentPos().arm_pos_;
        
        int8_t current_claw_logical = (current_claw_theta > initData_.max_clawAngle_ * 0.5f) ? 1 : 0;
        int8_t current_arm_logical = (current_arm_pos > initData_.max_arm_angle_ * 0.5f) ? 1 : 0;

        // 记录状态
        ctrl_status_.last_manual_claw_state = current_claw_logical;
        ctrl_status_.last_arm_switch_state = current_arm_logical;

        // 计算偏移: offset = switch ^ state
        ctrl_status_.claw_switch_offset = (airjoy_data_.SWD & 0x01) ^ current_claw_logical;
        ctrl_status_.arm_switch_offset = (airjoy_data_.SWB & 0x01) ^ current_arm_logical;

        last_weaponSage_status_ = WEAPONSAGE_MANUAL_CONTROL;

        ctrl_status_.last_isClaw_tight = ctrl_status_.isClaw_tight;
        ctrl_status_.last_scroll_state = airjoy_data_.SWC;

        ctrl_status_.last_isArm_Vertical = ctrl_status_.isArm_Vertical;

        ctrl_status_.scroll_offset = (airjoy_data_.SWC & 0x01) ^ ctrl_status_.last_isClaw_tight; // 初始状态假设为0
		weaponSage_status_=WEAPONSAGE_MANUAL_CONTROL;
    }
    if(airjoy_data_.SWE==0)
    {
        if(_tool_Abs(airjoy_data_.right_y) < 0.1)
			manual_RC10_ctrlForgrip_.changeTarget_state = false;
        if(airjoy_data_.right_y > 0.5f&&target_pos_.launch_pos_ <=initData_.max_launchHeight_)
        {
            
            target_pos_.launch_pos_ += manual_RC10_ctrlForgrip_.weapon_launch_fastrate;			
        }
        else if(airjoy_data_.right_y < -0.5f)
        {

            target_pos_.launch_pos_ -= manual_RC10_ctrlForgrip_.weapon_launch_fastrate;	
        }
        else
            target_pos_.launch_pos_ = target_pos_.launch_pos_;

        if(target_pos_.launch_pos_ > initData_.max_launchHeight_)
            target_pos_.launch_pos_ = initData_.max_launchHeight_;
        else if(target_pos_.launch_pos_ < 0.0f)
            target_pos_.launch_pos_ = 0.0f;
    
            if(_tool_Abs(airjoy_data_.right_x) < 0.1)
            {
                manual_RC10_ctrlForgrip_.changeTarget_state = false;
            }
            else if(airjoy_data_.right_x > 0.5)
            {
                target_pos_.wrist_pos_ += manual_RC10_ctrlForgrip_.weapon_wrist_rate;
            }
            else if(airjoy_data_.right_x < -0.5)
            {
                target_pos_.wrist_pos_ -= manual_RC10_ctrlForgrip_.weapon_wrist_rate;
            }else{
                target_pos_.wrist_pos_ = target_pos_.wrist_pos_;
            }
        
        
        int8_t target_claw_logical = (airjoy_data_.SWD & 0x01) ^ ctrl_status_.claw_switch_offset;
        
        ctrl_status_.last_manual_claw_state = target_claw_logical;

        int8_t target_tight_logical = (airjoy_data_.SWC & 0x01) ^ ctrl_status_.scroll_offset;
        ctrl_status_.isClaw_tight = target_tight_logical;

        if(target_claw_logical == 0)
        {
            
            target_pos_.claw_1_pos_ =0.0f; //开爪子
            target_pos_.claw_2_pos_ = 0.0f; 
            target_pos_.claw_3_pos_ = 0.0f;

        }
        else
        {
            if(ctrl_status_.isClaw_tight)
            {
            target_pos_.claw_1_pos_ = initData_.max_clawAngle_; //紧爪子
            target_pos_.claw_2_pos_ = initData_.max_clawAngle_;
            target_pos_.claw_3_pos_ = initData_.max_clawAngle_;
            }else
            {
                
                target_pos_.claw_1_pos_ = test_angle; //不太紧爪子
                target_pos_.claw_2_pos_ = test_angle;
                target_pos_.claw_3_pos_ = test_angle;
            }
        }
        
        int8_t target_arm_logical = (airjoy_data_.SWB & 0x01) ^ ctrl_status_.arm_switch_offset;
        ctrl_status_.last_arm_switch_state = target_arm_logical;
        int8_t target_arm_vertical_logical = (airjoy_data_.SWB & 0x01) ^ ctrl_status_.arm_switch_offset;
        if(auto_ctrl_.auto_state_bool_S.arm_enable&&target_arm_logical==1)
        {
           this->setArm_angle_slow(stend_up);   // 这里的角度待定
        }else if(auto_ctrl_.auto_state_bool_S.arm_enable&&target_arm_logical==0)
        {
            this->setArm_angle_slow(lift_pitch);  //这里角度也待定
        }
    }
    
    manual_RC10_ctrlForgrip_.last_right_stick_x = airjoy_data_.right_x;
    manual_RC10_ctrlForgrip_.last_right_stick_y = airjoy_data_.right_y;
    #endif


}

void Robot_WeaponSage_Setup::idle()
{
	this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
	if(last_weaponSage_status_!=WEAPONSAGE_IDLE)
		this->setClawSpeedOutLimit(3000.0f);
	{
		this->last_pos_ = this->get_CurrentPos();
		this->target_pos_=this->last_pos_;
		last_weaponSage_status_=WEAPONSAGE_IDLE;

	}
	this->setLaunch_angle(target_pos_.launch_pos_);
    this->setClaw_1_angle(target_pos_.claw_1_pos_);
    this->setClaw_2_angle(target_pos_.claw_2_pos_);
    this->setClaw_3_angle(target_pos_.claw_3_pos_);
    this->setWrist_angle(target_pos_.wrist_pos_);
}


float target_wrist= 0.0f;
float target_arm = 0.0f;
float target_claw_=0.0f;
float kp=0.0f;
void Robot_WeaponSage_Setup::debug()
{
	this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
//	this->autoControl();
//	if(this->arm_Motor_->getErrorNum()==0x01)
//	{
//		this->setArm_angle(target_arm);
//	}
//	
	this->setWrist_angle(target_wrist);
//	this->setClaw_1_angle(target_claw_);
//	this->setClaw_2_angle(target_claw_);
//	this->setClaw_3_angle(target_claw_);
//	this->setLaunch_angle(kp*initData_.max_launchHeight_);
}




    /**
     * @brief 抓取流程
     *        新版抓取杆时候不需要抬高，需降到最低，贴近后等待底盘停稳信号
     *        底盘停稳信号到达后夹取目标杆
     *        夹取完成后抬高到安全高度，完流程
     */
bool Robot_WeaponSage_Setup::autoControl_catch()
{
    
	#if !USE_NEW_AUTO
    static float launch_time = 0.0f;
    static bool is_launching = false;
	this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
    if(!auto_ctrl_.flag.is_reach_start)
    {
		this->setClaw_1_angle(0.0f);
		this->setClaw_2_angle(0.0f);
		this->setClaw_3_angle(0.0f);
        this->setLaunch_angle(auto_ctrl_.launch_kp.launch_start*initData_.max_launchHeight_);
        if(abs(current_pos_.launch_pos_-auto_ctrl_.launch_kp.launch_start*initData_.max_launchHeight_)<0.02f)
        {
            auto_ctrl_.flag.is_reach_start=true;
        }
    }
//	if(!auto_ctrl_.flag.is_arm_reset)
//		{
//			this->setArm_angle(45.0f);
//			auto_ctrl_.flag.is_arm_reset=true;
//		}
//    if(auto_ctrl_.flag.is_reach_start&&abs(current_pos_.arm_pos_-45.0f)<0.02f) //如果已经到达预定位置了
//    {
//        this->setWrist_angle(180.0f); //进入对位成功的状态
//    }
	if(auto_ctrl_.flag.is_reach_start)
	{
		this->setWrist_angle(180.0f); 
	}
	float current_wrist_pos= normalize_deg_0_360(current_pos_.wrist_pos_);
    if(abs(current_wrist_pos-180.0f)<0.5f) //如果手腕也调整到位了，进入等待底盘停稳的状态
    {
		if(!auto_ctrl_.flag.is_prepared)
		{
			this->setArm_angle(-90.0f);
			this->setLaunch_angle(auto_ctrl_.launch_kp.launch_catch*initData_.max_launchHeight_);      //贴近目标杆
			if(abs(current_pos_.launch_pos_-auto_ctrl_.launch_kp.launch_catch*initData_.max_launchHeight_)<0.02f)
			{	
				auto_ctrl_.flag.is_prepared=true;
			}
		}
    }
/*--------------------------------------准备模式已就绪，等待底盘就位--------------------------------------*/

    if(auto_ctrl_.auto_state_bool_S.is_matching&&auto_ctrl_.flag.is_prepared) //如果已经在对位了
    {

		this->setArm_angle(-0.1f);
		auto_ctrl_.flag.is_arm_reset=true;
		if(abs(current_pos_.arm_pos_-0.1f)<1.0f)
		{
		    if (!auto_ctrl_.flag.is_clawed)
            {
		        this->Close_TargetClaw(); //夹取目标杆
            }
            
            
            if(auto_ctrl_.flag.is_clawed )
                 auto_ctrl_.flag.is_catched=true; //完成抓取流程
                 auto_ctrl_.flag.is_clawed = false; //重置夹爪状态，准备下次使用
                 return true;
            }
    }
    return false;
    #else
    if(!auto_ctrl_.flag.is_reach_start)
    {
		this->setClaw_1_angle(0.0f);
		this->setClaw_2_angle(0.0f);
		this->setClaw_3_angle(0.0f);
        this->setLaunch_angle(auto_ctrl_.launch_kp.launch_start*initData_.max_launchHeight_);
        if(abs(current_pos_.launch_pos_-auto_ctrl_.launch_kp.launch_start*initData_.max_launchHeight_)<0.02f)
        {
            auto_ctrl_.flag.is_reach_start=true;
        }
    }
    if(auto_ctrl_.flag.is_reach_start)
	{
		this->setWrist_angle(180.0f); 
	}
	float current_wrist_pos= normalize_deg_0_360(current_pos_.wrist_pos_);
    if(abs(current_wrist_pos-180.0f)<0.5f) //如果手腕也调整到位了，进入等待底盘停稳的状态
    {
		if(!auto_ctrl_.flag.is_prepared&&auto_ctrl_.flag.is_reach_start)
		{
			this->setArm_angle(stend_up);
			this->setLaunch_angle(auto_ctrl_.launch_kp.launch_catch*initData_.max_launchHeight_);      //贴近目标杆
			if(abs(current_pos_.launch_pos_-auto_ctrl_.launch_kp.launch_catch*initData_.max_launchHeight_)<0.02f)
			{	
				auto_ctrl_.flag.is_prepared=true;
			}
		}
    }
//-------------------------------------------------------------------------------------------------------------------------------------------------
    if(auto_ctrl_.auto_state_bool_S.is_matching&&auto_ctrl_.flag.is_prepared) //如果已经在对位了
    {

		this->setArm_angle(lift_pitch);
		if(abs(current_pos_.arm_pos_-lift_pitch)<1.0)
		{
            if(!auto_ctrl_.flag.is_untight)
            {
                this->Close_TargetClaw_Untight1(); //夹取目标杆
            }
            else
            {
                auto_ctrl_.flag.is_catch = true;
                if(!auto_ctrl_.flag.is_clawed)
                {
                    this->setLaunch_angle(auto_ctrl_.launch_kp.launch_sage_untight*initData_.max_launchHeight_);      //抬高到安全高度,高度待调整
                }
            }
         
            
            if(auto_ctrl_.flag.is_untight && abs(current_pos_.launch_pos_-auto_ctrl_.launch_kp.launch_sage_untight*initData_.max_launchHeight_)<0.02) //如果已经抬高到位了
            {    
                this->Close_TargetClaw(); //夹取目标杆   
            }
            if(auto_ctrl_.flag.is_clawed)
            {
                this->setLaunch_angle(initData_.max_launchHeight_);      //抬高到安全高度,高度待调整
                if(abs(current_pos_.launch_pos_-initData_.max_launchHeight_)<0.02f)
                {
                    auto_ctrl_.flag.is_catched=true; //完成抓取流程
                    auto_ctrl_.flag.is_clawed = false; //重置夹爪状态，准备下次使用
                    auto_ctrl_.flag.is_untight = false; //重置夹爪状态，准备下次使用
                    return true;
                }
            
            }
		}
    }
    return false;


    #endif
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
 
 float wrist_waitting_time=0.2f;
void Robot_WeaponSage_Setup::autoControl_dock()
{
    #if !USE_NEW_AUTO
    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
        switch (now_state_)

        {
            case WeaponSage_Setup::STATE_START:
            {
                if(!auto_ctrl_.auto_state_bool_S.dock_start)
                {
                    this->idle(); //进入idle状态，等待对接开始的信号
                }
                else
                {
					now_state_ = WeaponSage_Setup::STATE_CLAW_ADJUST;
                }
                break;
            }

            case WeaponSage_Setup::STATE_CLAW_ADJUST:
            {
                if (!auto_ctrl_.flag.is_reach_closedclaw)
                {
                    this->Close_TargetClaw_Untight1();
                    if(auto_ctrl_.flag.is_untight) //如果已经调整好爪子了，进入下一个状态
                    {
//                        if(!auto_ctrl_.is_sage_adjust_start)
//                        {
//                            auto_ctrl_.is_sage_adjust_start=true;
//                            auto_ctrl_.sage_adjust_time=TimeStamp::getInstance().getSeconds();
//                        }
//                        if(auto_ctrl_.is_sage_adjust_start&&(ctrl_status_.now_times-auto_ctrl_.sage_adjust_time)>0.4f) //等待0.4秒后再调整升降
//                        {
							this->setLaunch_angle(auto_ctrl_.launch_kp.launch_clawclosed*initData_.max_launchHeight_);   
                            if(abs(current_pos_.launch_pos_-auto_ctrl_.launch_kp.launch_clawclosed*initData_.max_launchHeight_)<0.02f) //如果已经调整到位了，进入下一个状态
                            {
                                auto_ctrl_.flag.is_reach_closedclaw=true;
                                auto_ctrl_.flag.is_untight=false;
                            }    
//                        }
					}
				} 					
                else //如果已经调整到位了，进入下一个状态
                {
					this->Close_TargetClaw();
                    if(auto_ctrl_.flag.is_clawed)
                    {
						this->setLaunch_angle(auto_ctrl_.launch_kp.launch_rotate*initData_.max_launchHeight_);
                        if(abs(current_pos_.launch_pos_-auto_ctrl_.launch_kp.launch_rotate*initData_.max_launchHeight_)<0.02f) //如果已经调整到位了，进入下一个状态
                        {							
							this->setWrist_angle(0.0f); 
                            if(!ctrl_status_.is_wrist_start)
                            {
                                ctrl_status_.is_wrist_start=true;
                                ctrl_status_.wrist_startTime=TimeStamp::getInstance().getSeconds();
                            }
							float current_wrist_pos=normalize_deg_0_360(current_pos_.wrist_pos_);
							if(abs(current_wrist_pos-target_pos_.wrist_pos_)<0.2f&&(ctrl_status_.now_times-ctrl_status_.wrist_startTime)>=wrist_waitting_time) //如果手腕调整到位了，进入下一个状态
							{
								now_state_ = WeaponSage_Setup::STATE_SAGE_ADJUST;
								auto_ctrl_.flag.is_clawed=false;
								auto_ctrl_.flag.is_untight=false;
							}
                        }
                     }
                 }
					
                break;
			}				
            case WeaponSage_Setup::STATE_SAGE_ADJUST:
            {
				if (!auto_ctrl_.flag.is_reach_sagelowest)
                {
                    this->Close_TargetClaw_Untight();
                    if(auto_ctrl_.flag.is_untight)
                    {
                        this->setLaunch_angle(auto_ctrl_.launch_kp.launch_lastcatch*initData_.max_launchHeight_);      //降到最低，贴近目标杆
                        if(abs(current_pos_.launch_pos_-auto_ctrl_.launch_kp.launch_lastcatch*initData_.max_launchHeight_)<0.02f)
                        {
                            auto_ctrl_.flag.is_reach_sagelowest=true;
                            auto_ctrl_.flag.is_untight=false;
                        }
                    }
                }
                else //如果已经调整好爪子和升降了，进入下一个状态
                {
                    this->Close_TargetClaw();
                    if(auto_ctrl_.flag.is_clawed)
                    {
                        this->setLaunch_angle(auto_ctrl_.launch_kp.launch_start*initData_.max_launchHeight_); //将arm打到竖直位置
						if(abs(current_pos_.launch_pos_-auto_ctrl_.launch_kp.launch_start*initData_.max_launchHeight_)<0.5f)
						{
							now_state_ = WeaponSage_Setup::STATE_ARM_MOVE;
						}
                    }
                }
                // 处理武器架调整逻辑
                break;
            }
            case WeaponSage_Setup::STATE_ARM_MOVE:
            {   

                this->setArm_angle(-90.0f); //将arm打到竖直位置
                if(!auto_ctrl_.flag.is_reach_armrotate)
                {
                    this->setLaunch_angle(initData_.max_launchHeight_*auto_ctrl_.launch_kp.launch_dockprepare);      //抬高到预定位置
                    auto_ctrl_.flag.is_reach_armrotate=true;
					auto_ctrl_.flag.is_over = true;
                }

                if(std::fabs(current_pos_.launch_pos_ - initData_.max_launchHeight_*auto_ctrl_.launch_kp.launch_dockprepare) < 0.02f && !auto_ctrl_.dock_is_launching)
                {
                    auto_ctrl_.dock_launch_time = TimeStamp::getInstance().getSeconds();
                    auto_ctrl_.dock_is_launching = true;
                }

                if(auto_ctrl_.dock_is_launching && TimeStamp::getInstance().getSeconds() - auto_ctrl_.dock_launch_time > 0.4f && auto_ctrl_.dock_launch_time > 0.1f) // 如果已经调整到位了，进入下一个状态
                {
                    auto_ctrl_.dock_is_launching = false;
                    this->setLaunch_angle(initData_.dock_height_);      //抬高到预定位置
                    // if(abs(current_pos_.launch_pos_-initData_.dock_height_)<0.02f)
                    // {
                    now_state_ = WeaponSage_Setup::STATE_DONE;
                    // }
                }

                break;
            }
            case WeaponSage_Setup::STATE_DONE:
            {
                auto_ctrl_.auto_state_bool_S.is_matching=false;
                auto_ctrl_.auto_state_bool_S.dock_start=false;
                auto_ctrl_.flag.is_prepared=false;
                auto_ctrl_.flag.is_clawed=false;
                auto_ctrl_.flag.is_catched=false;
                auto_ctrl_.flag.is_reach_start=false;
                auto_ctrl_.flag.is_reach_closedclaw=false;
                auto_ctrl_.flag.is_reach_sagelowest=false;
                auto_ctrl_.flag.is_arm_reset=false;
                auto_ctrl_.flag.is_untight=false;
                ctrl_status_.is_wrist_start=false;
                auto_ctrl_.flag.is_reach_armrotate=false;
                now_state_=WeaponSage_Setup::STATE_START;
                auto_ctrl_.auto_ctrl1=false;
                auto_control_state_=0;

                this->idle(); //进入idle状态
                break;
            }
            default:
            {
                this->idle(); //如果状态异常了，也进入idle状态
                break;
            }
        }
    #else
    
        this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
        switch (now_new_state_)
        {
            case WeaponSage_Setup::STATE_START_1:
            {
                if(auto_ctrl_.auto_state_bool_S.dock_start || Locate_Setup::getInstance()->get_RobotPos_inWorld().y > 0.9f)
                {
					now_new_state_ = WeaponSage_Setup::STATE_SAGE_ADJUST_1;
                }
                else
                {
					this->idle(); //进入idle状态，等待对接开始的信号
                }
                break;
            }
            case WeaponSage_Setup::STATE_SAGE_ADJUST_1:
            {
                this->setWrist_angle(0.0f);
                if(!ctrl_status_.is_wrist_start)
                {
                    ctrl_status_.is_wrist_start=true;
                    ctrl_status_.wrist_startTime=TimeStamp::getInstance().getSeconds();
                }
                float current_wrist_pos=normalize_deg_0_360(current_pos_.wrist_pos_);
                if(abs(current_wrist_pos-target_pos_.wrist_pos_)<0.5f 
					&& (ctrl_status_.now_times-ctrl_status_.wrist_startTime)>=wrist_waitting_time
					&& auto_ctrl_.auto_state_bool_S.dock_start) //如果手腕调整到位了，进入下一个状态
                {
                    this->Close_TargetClaw_Untight();
                }
                if(auto_ctrl_.flag.is_untight)
                {
                    this->setLaunch_angle(auto_ctrl_.launch_kp.launch_lastcatch*initData_.max_launchHeight_);      //降到最低，贴近目标杆
                    if(abs(current_pos_.launch_pos_-auto_ctrl_.launch_kp.launch_lastcatch*initData_.max_launchHeight_)<0.02f)
                    {
                        auto_ctrl_.flag.is_untight=false;
                        now_new_state_ = WeaponSage_Setup::STATE_ARM_MOVE_1;
                    }
                } 
                break; 
            }
            case WeaponSage_Setup::STATE_ARM_MOVE_1:
            {
                this->Close_TargetClaw();
                if(auto_ctrl_.flag.is_clawed)
                {
					if(current_pos_.launch_pos_ - auto_ctrl_.launch_kp.launch_lastcatch*initData_.max_launchHeight_ > 0.01f)
						auto_ctrl_.flag.is_over = true;
					
                    this->setLaunch_angle(auto_ctrl_.launch_kp.launch_dockprepare*initData_.max_launchHeight_);
                    if(abs(current_pos_.launch_pos_-auto_ctrl_.launch_kp.launch_dockprepare*initData_.max_launchHeight_)<0.02f) //如果已经调整到位了，进入下一个状态
                    {
                        this->setArm_angle(stend_up); //将arm打到竖直位置							
                        now_new_state_ = WeaponSage_Setup::STATE_SAGE_MOVE;
                    }
                }
                break;
            }
            case WeaponSage_Setup::STATE_SAGE_MOVE:
            {    
                this->setLaunch_angle(initData_.dock_height_);      //抬高到预定位置
                if(abs(current_pos_.launch_pos_-initData_.dock_height_)<0.02f) //如果已经调整到位了，进入下一个状态
                {
                    now_new_state_ = WeaponSage_Setup::STATE_DONE_1;
                }
                break;
            }
            case WeaponSage_Setup::STATE_DONE_1:
            {
              auto_ctrl_.auto_state_bool_S.is_matching=false;
                auto_ctrl_.auto_state_bool_S.dock_start=false;
                auto_ctrl_.flag.is_catch = false;
                auto_ctrl_.flag.is_prepared=false;
                auto_ctrl_.flag.is_clawed=false;
                auto_ctrl_.flag.is_catched=false;
                auto_ctrl_.flag.is_reach_start=false;
                auto_ctrl_.flag.is_reach_closedclaw=false;
                auto_ctrl_.flag.is_reach_sagelowest=false;
                auto_ctrl_.flag.is_arm_reset=false;
                auto_ctrl_.flag.is_untight=false;
                ctrl_status_.is_wrist_start=false;
                auto_ctrl_.flag.is_reach_armrotate=false;
                now_new_state_=WeaponSage_Setup::STATE_START_1;
                auto_ctrl_.auto_ctrl1=false;
                auto_control_state_=0;
                this->idle(); //进入idle状态
                break;
            }
            default:
            {
                this->idle(); //如果状态异常了，也进入idle状态
                break;
            }
		}

    #endif
}


void Robot_WeaponSage_Setup::autoControl()
{
    #if !USE_NEW_AUTO
    switch(auto_control_state_)
    {
        case 0:
		{
            if(auto_ctrl_.auto_ctrl1)
            {
                now_state_=WeaponSage_Setup::STATE_START;
                auto_control_state_=1;
                auto_ctrl_.flag.is_over = false;
                auto_ctrl_.auto_state_bool_S.is_matching=false;
                auto_ctrl_.auto_state_bool_S.dock_start=false;
                auto_ctrl_.flag.is_prepared=false;
                auto_ctrl_.flag.is_clawed=false;
                auto_ctrl_.flag.is_catched=false;
                auto_ctrl_.flag.is_reach_start=false;
                auto_ctrl_.flag.is_reach_closedclaw=false;
                auto_ctrl_.flag.is_reach_sagelowest=false;
                auto_ctrl_.flag.is_arm_reset=false;
                auto_ctrl_.auto_state_bool_S.is_matching=false;
                auto_ctrl_.auto_state_bool_S.dock_start=false;
                auto_ctrl_.flag.is_untight=false;
                ctrl_status_.is_wrist_start=false;
                auto_ctrl_.flag.is_moved=false;
                auto_ctrl_.flag.is_reach_armrotate=false;

                // 补充重置：上次中断可能留下的残留状态
                ctrl_status_.is_closeclaw_start = false;
                ctrl_status_.closeclaw_startTime = 0.0f;
                ctrl_status_.is_untight_start = false;
                ctrl_status_.untight_startTime = 0.0f;
                auto_ctrl_.claw_flag[0] = false;
                auto_ctrl_.claw_flag[1] = false;
                auto_ctrl_.claw_flag[2] = false;
                auto_ctrl_.dock_is_launching = false;
                auto_ctrl_.dock_launch_time = 0.0f;
                auto_ctrl_.is_sage_adjust_start = false;
                auto_ctrl_.sage_adjust_time = 0.0f;
            }
            else
			{
				this->idle();
				auto_control_state_=0;
			}
            break;
		}
        case 1:
		{
            bool catch_result = this->autoControl_catch();
            if(catch_result)
            {
                auto_control_state_=2;
            }
            break;
		}
        case 2:
//			auto_ctrl_.auto_state_bool_S.dock_start=true;
            autoControl_dock();
            break;
        default:
            break;
    }
    #else
    switch(auto_control_state_)
    {
        case 0:
        {
            if(auto_ctrl_.auto_ctrl1)
            {
                now_new_state_=WeaponSage_Setup::STATE_START_1;
                auto_ctrl_.flag.is_catch = false;
                auto_control_state_=1;
                auto_ctrl_.flag.is_over = false;
                auto_ctrl_.auto_state_bool_S.is_matching=false;
                auto_ctrl_.auto_state_bool_S.dock_start=false;
                auto_ctrl_.flag.is_prepared=false;
                auto_ctrl_.flag.is_clawed=false;
                auto_ctrl_.flag.is_catched=false;
                auto_ctrl_.flag.is_reach_start=false;
                auto_ctrl_.flag.is_reach_closedclaw=false;
                auto_ctrl_.flag.is_reach_sagelowest=false;
                auto_ctrl_.flag.is_arm_reset=false;
                auto_ctrl_.auto_state_bool_S.is_matching=false;
                auto_ctrl_.auto_state_bool_S.dock_start=false;
                auto_ctrl_.flag.is_untight=false;
                ctrl_status_.is_wrist_start=false;
                auto_ctrl_.flag.is_moved=false;
                auto_ctrl_.flag.is_reach_armrotate=false;

                // 补充重置：上次中断可能留下的残留状态
                ctrl_status_.is_closeclaw_start = false;
                ctrl_status_.closeclaw_startTime = 0.0f;
                ctrl_status_.is_untight_start = false;
                ctrl_status_.untight_startTime = 0.0f;
                auto_ctrl_.claw_flag[0] = false;
                auto_ctrl_.claw_flag[1] = false;
                auto_ctrl_.claw_flag[2] = false;
                auto_ctrl_.dock_is_launching = false;
                auto_ctrl_.dock_launch_time = 0.0f;
                auto_ctrl_.is_sage_adjust_start = false;
                auto_ctrl_.sage_adjust_time = 0.0f;
            }
            else
            {
                this->idle();
                auto_control_state_=0;
            }
            break;
        }
        case 1:
        {
            bool catch_result = this->autoControl_catch();
            if(catch_result)
            {
                auto_control_state_=2;
            }
            break;
        }
        case 2:
    //		auto_ctrl_.auto_state_bool_S.dock_start=true;
            autoControl_dock();
            break;
        default:
            break;
    }
#endif
 }  



void Robot_WeaponSage_Setup::stop()
{
    //停止，电机不动
    this->setCtrlMode(WeaponSage::CURRENT_CONTROL);
    this->claw_1_Motor_->setTargetCurrent(0.0f);
    this->claw_2_Motor_->setTargetCurrent(0.0f);
    this->claw_3_Motor_->setTargetCurrent(0.0f);
    this->launch_Motor_->setTargetCurrent(0.0f);
//    this->arm_Motor_->setTargetCurrent(0.0f);
//    this->wrist_Motor_->setTargetCurrent(0.0f);
}


void Robot_WeaponSage_Setup::semi_autoControl()
{
    #if USE_RC10_AIRJOY
    if(last_weaponSage_status_!=WEAPONSAGE_SEMI_AUTO_CONTROL)
    {
        manual_RC10_ctrlForgrip_.is_Dpad_Done=false;
    }
    if(airjoy_data_.SWD==1)
    {
        manual_RC10_ctrlForgrip_.is_Dpad_Done=D_pad_acting();
        if(manual_RC10_ctrlForgrip_.is_Dpad_Done)
        {
            if(airjoy_data_.right_y > 0.5f&&target_pos_.launch_pos_ <=initData_.max_launchHeight_)
            {
            target_pos_.launch_pos_ += manual_RC10_ctrlForgrip_.weapon_launch_fastrate;			
            }
            else if(airjoy_data_.right_y < -0.5f)
            {

            target_pos_.launch_pos_ -= manual_RC10_ctrlForgrip_.weapon_launch_fastrate;	
            }
        else
            target_pos_.launch_pos_ = target_pos_.launch_pos_;

        if(target_pos_.launch_pos_ > initData_.max_launchHeight_)
        target_pos_.launch_pos_ = initData_.max_launchHeight_;
        else if(target_pos_.launch_pos_ < 0.0f)
            target_pos_.launch_pos_ = 0.0f;
   
        if(_tool_Abs(airjoy_data_.right_x) < 0.1)
        {
            manual_RC10_ctrlForgrip_.changeTarget_state = false;
        }
        else if(airjoy_data_.right_x > 0.5)
        {
            target_pos_.wrist_pos_ += manual_RC10_ctrlForgrip_.weapon_wrist_rate;
        }
        else if(airjoy_data_.right_x < -0.5)
        {
            target_pos_.wrist_pos_ -= manual_RC10_ctrlForgrip_.weapon_wrist_rate;
        }else{
            target_pos_.wrist_pos_ = target_pos_.wrist_pos_;
        }
        }
    }


    #endif
}


//bool Robot_WeaponSage_Setup::D_pad_acting()
//{
//#if USE_RC10_AIRJOY
//    if(last_weaponSage_status_!=weaponSage_status_)
//    {
//        manual_RC10_ctrlForgrip_.is_Dpad_up_enabled=0;
//        manual_RC10_ctrlForgrip_.is_Dpad_down_enabled=0;
//        manual_RC10_ctrlForgrip_.is_Dpad_left_enabled=0;
//        manual_RC10_ctrlForgrip_.is_Dpad_right_enabled=0;
//        auto_ctrl_.flag.is_clawed=false;
//        auto_ctrl_.flag.is_untight=false;
//    }
//    bool is_Rceive_command=false;
//    if(airjoy_data_.d_pad_up==1&&manual_RC10_ctrlForgrip_.is_Dpad_up_enabled==0)
//    {
//        this->Sage_to_high();
//        manual_RC10_ctrlForgrip_.is_Dpad_up_enabled=1;
//        is_Rceive_command=true;
//    }else if(airjoy_data_.d_pad_up==0&&manual_RC10_ctrlForgrip_.is_Dpad_up_enabled==1)
//    {
//        manual_RC10_ctrlForgrip_.is_Dpad_up_enabled=0;
//    }
//    if(airjoy_data_.d_pad_down==1&&manual_RC10_ctrlForgrip_.is_Dpad_down_enabled==0)
//    {
//        this->Sage_to_low();
//        manual_RC10_ctrlForgrip_.is_Dpad_down_enabled=1;
//        is_Rceive_command=true;
//    }else if(airjoy_data_.d_pad_down==0&&manual_RC10_ctrlForgrip_.is_Dpad_down_enabled==1)
//    {
//        manual_RC10_ctrlForgrip_.is_Dpad_down_enabled=0;
//    }
//    if(airjoy_data_.d_pad_left==1&&manual_RC10_ctrlForgrip_.is_Dpad_left_enabled==0)
//    {
//        target_pos_.wrist_pos_=WristToClosest_negative(current_pos_.wrist_pos_);
//        manual_RC10_ctrlForgrip_.is_Dpad_left_enabled=1;
//        is_Rceive_command=true;
//    }else if(airjoy_data_.d_pad_left==0&&manual_RC10_ctrlForgrip_.is_Dpad_left_enabled==1)
//    {
//        manual_RC10_ctrlForgrip_.is_Dpad_left_enabled=0;
//    } 
//    if(airjoy_data_.d_pad_right==1&&manual_RC10_ctrlForgrip_.is_Dpad_right_enabled==0)
//    {
//        target_pos_.wrist_pos_=WristToClosest_poistive(current_pos_.wrist_pos_);
//        manual_RC10_ctrlForgrip_.is_Dpad_right_enabled=1; 
//        is_Rceive_command=true;
//    }else if(airjoy_data_.d_pad_right==0&&manual_RC10_ctrlForgrip_.is_Dpad_right_enabled==1)
//    {
//        manual_RC10_ctrlForgrip_.is_Dpad_right_enabled=0;
//    }
//    //------------------------------------------------------保证长按值只执行一次------------------------------------------------------
//    if(!is_Rceive_command)
//    {
//        return true;
//    }else{
//        if(abs(current_pos_.wrist_pos_-target_pos_.wrist_pos_)<0.1f||auto_ctrl_.flag.is_clawed)
//        {
//            return true;
//        }
//        else
//        {
//            return false;
//        }
//    }
//   #endif
//}



bool Robot_WeaponSage_Setup::D_pad_acting()
{
	#if USE_RC10_AIRJOY
    if(last_weaponSage_status_!=weaponSage_status_)
    {
        manual_RC10_ctrlForgrip_.is_Dpad_up_enabled=0;
        manual_RC10_ctrlForgrip_.is_Dpad_down_enabled=0;
        manual_RC10_ctrlForgrip_.is_Dpad_left_enabled=0;
        manual_RC10_ctrlForgrip_.is_Dpad_right_enabled=0;
        manual_RC10_ctrlForgrip_.Dpad_acting_state=0;
        auto_ctrl_.flag.is_clawed=false;
        auto_ctrl_.flag.is_untight=false;
    }
    bool is_Rceive_command=false;
    if(airjoy_data_.d_pad_up==1&&manual_RC10_ctrlForgrip_.is_Dpad_up_enabled==0)
    {
        manual_RC10_ctrlForgrip_.Dpad_acting_state=1;
        manual_RC10_ctrlForgrip_.is_Dpad_up_enabled=1;
        is_Rceive_command=true;
    }else if(airjoy_data_.d_pad_up==0&&manual_RC10_ctrlForgrip_.is_Dpad_up_enabled==1)
    {
        manual_RC10_ctrlForgrip_.is_Dpad_up_enabled=0;
    }
    if(airjoy_data_.d_pad_down==1&&manual_RC10_ctrlForgrip_.is_Dpad_down_enabled==0)
    {
        manual_RC10_ctrlForgrip_.Dpad_acting_state=2;
        manual_RC10_ctrlForgrip_.is_Dpad_down_enabled=1;
        is_Rceive_command=true;
    }else if(airjoy_data_.d_pad_down==0&&manual_RC10_ctrlForgrip_.is_Dpad_down_enabled==1)
    {
        manual_RC10_ctrlForgrip_.is_Dpad_down_enabled=0;
    }
    if(airjoy_data_.d_pad_left==1&&manual_RC10_ctrlForgrip_.is_Dpad_left_enabled==0)
    {
        manual_RC10_ctrlForgrip_.Dpad_acting_state=3;
        target_pos_.wrist_pos_=WristToClosest_negative(current_pos_.wrist_pos_);
        manual_RC10_ctrlForgrip_.is_Dpad_left_enabled=1;
        is_Rceive_command=true;
    }else if(airjoy_data_.d_pad_left==0&&manual_RC10_ctrlForgrip_.is_Dpad_left_enabled==1)
    {
        manual_RC10_ctrlForgrip_.is_Dpad_left_enabled=0;
    } 
    if(airjoy_data_.d_pad_right==1&&manual_RC10_ctrlForgrip_.is_Dpad_right_enabled==0)
    {
        manual_RC10_ctrlForgrip_.Dpad_acting_state=4;
        target_pos_.wrist_pos_=WristToClosest_poistive(current_pos_.wrist_pos_);
        manual_RC10_ctrlForgrip_.is_Dpad_right_enabled=1; 
        is_Rceive_command=true;
    }else if(airjoy_data_.d_pad_right==0&&manual_RC10_ctrlForgrip_.is_Dpad_right_enabled==1)
    {
        manual_RC10_ctrlForgrip_.is_Dpad_right_enabled=0;
    }
    //------------------------------------------------------保证长按值只执行一次------------------------------------------------------
    if( manual_RC10_ctrlForgrip_.Dpad_acting_state==0)
    {
    //   this->idle();
        auto_ctrl_.flag.is_clawed=false;
        auto_ctrl_.flag.is_untight=false;
    }else if(manual_RC10_ctrlForgrip_.Dpad_acting_state==1)
    {
        bool Sage_up_flag=Sage_back();
        
        if(Sage_up_flag)
        {
            manual_RC10_ctrlForgrip_.last_Dpad_acting_state=1;
            manual_RC10_ctrlForgrip_.Dpad_acting_state=0;
        }
    }else if(manual_RC10_ctrlForgrip_.Dpad_acting_state==2)
    {
        float sage_down_flag=Sage_to_low();
        
        if(sage_down_flag)
        {
            manual_RC10_ctrlForgrip_.last_Dpad_acting_state=2;
            manual_RC10_ctrlForgrip_.Dpad_acting_state=0;
        }
    }  
    else if(manual_RC10_ctrlForgrip_.Dpad_acting_state==3)
    {
        float currrent_wrist=normalize_deg_0_360(current_pos_.wrist_pos_);
    //    target_pos_.wrist_pos_=WristToClosest_poistive(current_pos_.wrist_pos_);
        if(abs(currrent_wrist-target_pos_.wrist_pos_)<0.1f)
        {
            manual_RC10_ctrlForgrip_.Dpad_acting_state=0;
            manual_RC10_ctrlForgrip_.last_Dpad_acting_state=3;
        }
    }else if(manual_RC10_ctrlForgrip_.Dpad_acting_state==4)
    {
    //    target_pos_.wrist_pos_=WristToClosest_negative(current_pos_.wrist_pos_);
        float currrent_wrist=normalize_deg_0_360(current_pos_.wrist_pos_);
        if(abs(currrent_wrist-target_pos_.wrist_pos_)<0.1f)
        {
            manual_RC10_ctrlForgrip_.Dpad_acting_state=0;
            manual_RC10_ctrlForgrip_.last_Dpad_acting_state=4;
        }
    }
    if(manual_RC10_ctrlForgrip_.Dpad_acting_state==0)
    {
        return true;
    }else
    {
        return false;
    }
   #endif
}


void Robot_WeaponSage_Setup::semi_low_level()
{
    #if USE_RC10_AIRJOY
    if(last_weaponSage_status_!=WEAPONSAGE_MANUAL_LOW_LEVEL)
    {
        manual_RC10_ctrlForgrip_.is_Dpad_Done=false;
    }
    manual_RC10_ctrlForgrip_.is_Dpad_Done=D_pad_acting();
    if(airjoy_data_.SWE==1&&manual_RC10_ctrlForgrip_.is_Dpad_Done)
    {
        manual_RC10_ctrlForgrip_.changeTarget_state = false;
        if(airjoy_data_.right_y > 0.5f&&target_pos_.launch_pos_ <=initData_.max_launchHeight_)
        {
            
            target_pos_.launch_pos_ += manual_RC10_ctrlForgrip_.weapon_launch_slowrate;			
        }
        else if(airjoy_data_.right_y < -0.5f)
        {

            target_pos_.launch_pos_ -= manual_RC10_ctrlForgrip_.weapon_launch_slowrate;	
        }
        else
            target_pos_.launch_pos_ = target_pos_.launch_pos_;

        if(target_pos_.launch_pos_ > initData_.max_launchHeight_)
            target_pos_.launch_pos_ = initData_.max_launchHeight_;
        else if(target_pos_.launch_pos_ < 0.0f)
            target_pos_.launch_pos_ = 0.0f;
        if(_tool_Abs(airjoy_data_.right_x) < 0.1)
        {
            manual_RC10_ctrlForgrip_.changeTarget_state = false;
        }
        else if(airjoy_data_.right_x > 0.5)
        {
            target_pos_.wrist_pos_ += manual_RC10_ctrlForgrip_.weapon_wrist_rate;
        }
        else if(airjoy_data_.right_x < -0.5)
        {
            target_pos_.wrist_pos_ -= manual_RC10_ctrlForgrip_.weapon_wrist_rate;
        }else{
            target_pos_.wrist_pos_ = target_pos_.wrist_pos_;
        }
    
    }
    #endif
}



float Robot_WeaponSage_Setup:: WristToClosest_poistive(float current_angle)
{
    // 先找到当前角度最接近的 90° 的倍数，然后再往正方向加 90°
    float nearest_90 = std::round(current_angle / 90.0f) * 90.0f;
    float target_angle = nearest_90 + 90.0f;
    
    // 处理 0-360° 零点跳变，超过 360° 归零
    while (target_angle >= 360.0f) target_angle -= 360.0f;
    while (target_angle < 0.0f) target_angle += 360.0f;
    
    return target_angle;
}

float Robot_WeaponSage_Setup::WristToClosest_negative(float current_angle)
{
    // 先找到当前角度最接近的 90° 的倍数，然后再往负方向减 90°
    float nearest_90 = std::round(current_angle / 90.0f) * 90.0f;
    float target_angle = nearest_90 - 90.0f;
    
    // 处理 0-360° 零点跳变，小于 0° 回滚
    while (target_angle >= 360.0f) target_angle -= 360.0f;
    while (target_angle < 0.0f) target_angle += 360.0f;
    
    return target_angle;
}
static bool launch_flag=false;
static bool is_launch_ok=false;
bool Robot_WeaponSage_Setup::Sage_to_high()
{
  
    if(!launch_flag)
    {
        this->setLaunch_angle(0.9*initData_.max_launchHeight_);
        

        if(abs(current_pos_.launch_pos_-0.9*initData_.max_launchHeight_)<0.02)
        {   
			float current_wrist=normalize_deg_0_360(current_pos_.wrist_pos_);
            if(abs(current_wrist-180.0f)<90.0f)
            {
                this->setWrist_angle(180.0f);
            }
            if(abs(current_wrist-180.0f)>90.0f)
            {
                this->setWrist_angle(0.0f);
            }
            launch_flag=true;
            is_launch_ok= true;
        }
            
    }
    if(is_launch_ok)
    {
        this->setArm_angle(lift_pitch);
        if(abs(current_pos_.arm_pos_+0.1f)<1.0f)
        {
            this->Close_TargetClaw_Untight();
            if(auto_ctrl_.flag.is_untight)
            {
                // bool flag_done=
                Close_TargetClaw();
                if(auto_ctrl_.flag.is_clawed)
                {
                    this->setLaunch_angle(initData_.max_launchHeight_);
                    if(abs(current_pos_.launch_pos_-initData_.max_launchHeight_)<0.02f)
                    {
                        // this->Close_TargetClaw();
                        // if(auto_ctrl_.flag.is_clawed)
                        // {
                        launch_flag=false;
                        is_launch_ok=false;
                        auto_ctrl_.flag.is_clawed=false;
                        auto_ctrl_.flag.is_untight=false;
                        return true;
                        // }         
                    }
                }   
            }
        }
    }
    return false;
}

bool Robot_WeaponSage_Setup::Sage_to_low()
{
    if(!launch_flag)
    {
        this->setLaunch_angle(0.9*initData_.max_launchHeight_);
        if(current_pos_.launch_pos_-0.9*initData_.max_launchHeight_)
        {
			float current_wrist=normalize_deg_0_360(current_pos_.wrist_pos_);
            if(abs(current_wrist-180.0f)<90.0f)
            {
                this->setWrist_angle(180.0f);
            }
            if(abs(current_wrist-180.0f)>90.0f)
            {
                this->setWrist_angle(0.0f);
            }
            launch_flag=true;
            is_launch_ok =true;
        }
    }
    if(is_launch_ok)
    {
        this->setArm_angle(lift_pitch);
        if(abs(current_pos_.arm_pos_-lift_pitch)<1.0f)
        {

            this->Close_TargetClaw_Untight();
            if(auto_ctrl_.flag.is_untight)
            {
                this->setLaunch_angle(0.1*initData_.max_launchHeight_);
                if(abs(current_pos_.launch_pos_-0.1*initData_.max_launchHeight_)<0.02f)
                {
                    //bool flag_done=
                    Close_TargetClaw();
                    if(auto_ctrl_.flag.is_clawed)
                    {
                        this->setLaunch_angle(0.2*initData_.max_launchHeight_);
                    }
                }
                else
                {
                    if(abs(current_pos_.launch_pos_-0.2f*initData_.max_launchHeight_)<0.02)
                    {
                        auto_ctrl_.flag.is_clawed=false;
                        auto_ctrl_.flag.is_untight=false;
                        launch_flag=false;
                        is_launch_ok=false;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool wrist_first=false;
bool is_wrist_ok=false;
bool Robot_WeaponSage_Setup::Sage_back()
{
    switch(now_back_state_)
    {
        case WeaponSage_Setup::BCAK_STATE_START:
        {
            now_back_state_=WeaponSage_Setup::BACK_SAGE_ARM_MOVE;
            break;
        }
        case WeaponSage_Setup::BACK_SAGE_ARM_MOVE:
        {
            this->setLaunch_angle(0.95*initData_.max_launchHeight_);
			if(abs(current_pos_.launch_pos_-0.95*initData_.max_launchHeight_)<0.02)
			{
				this->setArm_angle_slow(lift_pitch);
				if(abs(current_pos_.arm_pos_-lift_pitch)<1.0f)
                {
                    this->Close_TargetClaw_Untight();
                    if(auto_ctrl_.flag.is_untight)
                    {
                        now_back_state_=WeaponSage_Setup::BACK_WRIST_ADJUST;
                        auto_ctrl_.flag.is_untight=false;
                    }
                }
			}
            break;
        }
        case WeaponSage_Setup::BACK_WRIST_ADJUST:
        {
            this->Close_TargetClaw();
            if(auto_ctrl_.flag.is_clawed)
            {
                this->setLaunch_angle(initData_.max_launchHeight_);
                if(abs(current_pos_.launch_pos_-initData_.max_launchHeight_)<0.02f)
                {
					if(!wrist_first)
					{
						this->setWrist_angle(45.0f);
						float current_first_pos=normalize_deg_0_360(current_pos_.wrist_pos_);
						if(abs(current_first_pos-45.0f)<0.1f)
						is_wrist_ok=true;
					}
                   
                    if(is_wrist_ok)
                    {
						this->setWrist_angle(180.0f);
						 float current_wrist=normalize_deg_0_360(current_pos_.wrist_pos_);
						if(abs(current_wrist-180.0f)<0.1f)
						{
							now_back_state_=WeaponSage_Setup::BACK_CLAW_DOWN;
							auto_ctrl_.flag.is_clawed=false;
							wrist_first=false;
							is_wrist_ok=false;
						}
                    }
                }
            }
            break;
        }
        case WeaponSage_Setup::BACK_CLAW_DOWN:
        {
            this->Close_TargetClaw_Untight1();
            if(auto_ctrl_.flag.is_untight)
            {
                this->setLaunch_angle(0.1*initData_.max_launchHeight_);
                if(abs(current_pos_.launch_pos_-0.1*initData_.max_launchHeight_)<0.02f)
                {
                    now_back_state_=WeaponSage_Setup::BACK_DONE;
                    auto_ctrl_.flag.is_untight=false;
                }
            }
            break;
        }
        case WeaponSage_Setup::BACK_DONE:
        {
            this->Close_TargetClaw();   
            if(auto_ctrl_.flag.is_clawed)
            {
				this->setLaunch_angle(initData_.max_launchHeight_);
				if(abs(current_pos_.launch_pos_-initData_.max_launchHeight_)<0.02f)
				{
					auto_ctrl_.flag.is_clawed=false;
					now_back_state_=WeaponSage_Setup::BCAK_STATE_START;
					return true;
				}
                
            }
            break;
        }
		default:
			break;

    }
	return false;	
}

int8_t close_time_cnt = 0;
int8_t untight_time_cnt = 0;
int8_t close_over_cnt = 0;
int8_t untight_over_cnt = 0;


float claw_close_time=0.4f;
float claw_untight_time=0.4f;
bool Robot_WeaponSage_Setup::Close_TargetClaw()
{
    this->setClawSpeedOutLimit(3000.0f);
    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
   
    auto_ctrl_.claw_flag[0]=ctrl_status_.is_claw_1_closed;
    auto_ctrl_.claw_flag[1]=ctrl_status_.is_claw_2_closed;
    auto_ctrl_.claw_flag[2]=ctrl_status_.is_claw_3_closed;
    float target_claw_pos[3]={0.0f,0.0f,0.0f};
//    if(!auto_ctrl_.claw_flag[0]&&!auto_ctrl_.claw_flag[1]&&!auto_ctrl_.claw_flag[2])
//    {
//        auto_ctrl_.flag.is_clawed=false;
//    }

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

    if(!auto_ctrl_.auto_ctrl1)
    {
        for(int i=0;i<3;i++)
            target_claw_pos[i]=initData_.max_clawAngle_;
    }
    this->setClaw_1_angle(target_claw_pos[0]);
    this->setClaw_2_angle(target_claw_pos[1]);
    this->setClaw_3_angle(target_claw_pos[2]);
    target_pos_.claw_1_pos_=target_claw_pos[0];
    target_pos_.claw_2_pos_=target_claw_pos[1];
    target_pos_.claw_3_pos_=target_claw_pos[2];

    if(!ctrl_status_.is_closeclaw_start && !auto_ctrl_.flag.is_clawed) //如果已经调整好爪子了，进入下一个状态
     {
        ctrl_status_.closeclaw_startTime=TimeStamp::getInstance().getSeconds();
		close_time_cnt++;
        ctrl_status_.is_closeclaw_start=true;
     }
     if(ctrl_status_.now_times-ctrl_status_.closeclaw_startTime>= claw_close_time&&ctrl_status_.is_closeclaw_start) //保持夹紧状态0.5秒钟
     {
        auto_ctrl_.flag.is_clawed=true;
        ctrl_status_.is_closeclaw_start=false;
		 close_over_cnt++;
         return true;
     }
        return false;
}


bool Robot_WeaponSage_Setup::Close_TargetClaw_Untight()
{
    this->setClawSpeedOutLimit(4500.0f);
    auto_ctrl_.claw_flag[0]=ctrl_status_.is_claw_1_closed;
    auto_ctrl_.claw_flag[1]=ctrl_status_.is_claw_2_closed;
    auto_ctrl_.claw_flag[2]=ctrl_status_.is_claw_3_closed;
    float target_claw_pos[3] = {0.0f,0.0f,0.0f};
    for (int i=0;i<3;i++)       
    {
        if(auto_ctrl_.claw_flag[i])
        {
            target_claw_pos[i]=initData_.claw_untight;
        }
        else
        {
            target_claw_pos[i]=0;
        }
    }

    if(!auto_ctrl_.auto_ctrl1)
    {
        for (int i=0;i<3;i++)
            target_claw_pos[i] = initData_.claw_untight;
    }
    target_pos_.claw_1_pos_=target_claw_pos[0];
    target_pos_.claw_2_pos_=target_claw_pos[1];
    target_pos_.claw_3_pos_=target_claw_pos[2];
    this->setClaw_1_angle(target_claw_pos[0]); //半松爪子
    this->setClaw_2_angle(target_claw_pos[1]);
    this->setClaw_3_angle(target_claw_pos[2]);

    if(!ctrl_status_.is_untight_start && !auto_ctrl_.flag.is_untight) //如果已经调整好爪子了，进入下一个状态
    {
        ctrl_status_.untight_startTime=TimeStamp::getInstance().getSeconds();
        ctrl_status_.is_untight_start=true;
        untight_time_cnt++;
    }
    if(ctrl_status_.now_times-ctrl_status_.untight_startTime>=claw_untight_time&& ctrl_status_.is_untight_start) //保持半松状态1秒钟
    {
        auto_ctrl_.flag.is_untight=true;
        ctrl_status_.is_untight_start=false;
        untight_over_cnt++;
        return true;
    }else{
        return false;
    }
}

bool Robot_WeaponSage_Setup::Close_TargetClaw_Untight1()
    
{
    this->setClawSpeedOutLimit(4500.0f);
    auto_ctrl_.claw_flag[0]=ctrl_status_.is_claw_1_closed;
    auto_ctrl_.claw_flag[1]=ctrl_status_.is_claw_2_closed;
    auto_ctrl_.claw_flag[2]=ctrl_status_.is_claw_3_closed;
	float target_claw_pos[3] = {0.0f,0.0f,0.0f};
	for (int i=0;i<3;i++)       
	{
		if(auto_ctrl_.claw_flag[i])
		{
			target_claw_pos[i]=18.0f;
		}
		else
		{
			target_claw_pos[i]=0;
		}
	}

	if(!auto_ctrl_.auto_ctrl1)
	{
		for (int i=0;i<3;i++)
			target_claw_pos[i] = initData_.claw_untight;
	}
	target_pos_.claw_1_pos_=target_claw_pos[0];
	target_pos_.claw_2_pos_=target_claw_pos[1];
	target_pos_.claw_3_pos_=target_claw_pos[2];
	this->setClaw_1_angle(target_claw_pos[0]); //半松爪子
	this->setClaw_2_angle(target_claw_pos[1]);
	this->setClaw_3_angle(target_claw_pos[2]);

	if(!ctrl_status_.is_untight_start && !auto_ctrl_.flag.is_untight) //如果已经调整好爪子了，进入下一个状态
	{
		ctrl_status_.untight_startTime=TimeStamp::getInstance().getSeconds();
		ctrl_status_.is_untight_start=true;
		untight_time_cnt++;
	}
	if(ctrl_status_.now_times-ctrl_status_.untight_startTime>=claw_untight_time&& ctrl_status_.is_untight_start) //保持半松状态1秒钟
	{
		auto_ctrl_.flag.is_untight=true;
		ctrl_status_.is_untight_start=false;
		untight_over_cnt++;
		return true;
	}else{
		return false;
	}
}










void Robot_WeaponSage_Setup::Judge_launch_status()
{
  if(current_pos_.launch_pos_>=target_pos_.launch_pos_)
    {
        auto_ctrl_.auto_state_bool_S.launch_enable=true;
    }else
    {
        if(abs(current_pos_.arm_pos_)<0.1f)
        {
            auto_ctrl_.auto_state_bool_S.launch_enable=false;
        }else
        {
            auto_ctrl_.auto_state_bool_S.launch_enable=true;
        }
    }
}

void Robot_WeaponSage_Setup::Judge_wrist_status()
{
    if(current_pos_.launch_pos_>=initData_.wrist_protect_)
    {
        auto_ctrl_.auto_state_bool_S.wrist_enable=true;
    }else
    {
        auto_ctrl_.auto_state_bool_S.wrist_enable=false;
    }
}

void Robot_WeaponSage_Setup::SetServo_Angle(uint16_t angle)
{
	// if(!auto_ctrl_.is_ServoStart)
	// {
	// 	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	// 	auto_ctrl_.is_ServoStart=true;
	// }
	// uint16_t target_cnt = 500+(angle*2700)/2000; 
	// __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, target_cnt);
}

WeaponSage_InitData_S initData_=
{
    .max_launchHeight_ =0.358735231f,
    .max_clawAngle_ = 40.0f,
    .max_arm_angle_ = 135.0f,
    .max_wrist_angle_ = 360.0f,
	.max_arm_rate_ =1080.0f,
	.is_Arm_fast=false,
	.min_arm_rate_=200.0f,
	.dock_height_=0.065080245f,
    .wrist_protect_=0.174290136,
    .claw_untight = 27.0f,
    .wrist_gearRatio_ = 144.0f,
    .launch_Ratio_ = 0.139989366256f,
    .claw_gearRatio_  =360.0f ,
    .arm_gearRatio_ = 360.0f
};