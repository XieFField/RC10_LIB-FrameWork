#include "Arm_setup.h"

/**
 * @brief 控制循环
 */
// uint32_t ArmstackHighWaterMark = 0;
int8_t test_store = 0;
void ArmSetup::loop()
{
    if(!arm_ctrlStatus.init_flag)
        return;


	
//	ArmstackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

    // pitch 电机首次使能：校准前确保电机已使能
    if((motor_pitch_->getErrorNum() == 0x00 || !this->is_pitchEnable_))
    {
        motor_pitch_->motorEnable();
        this->is_pitchEnable_ = true;
    }

    if(!arm_ctrlStatus.is_calibrating)
    {
        calibrateMotor();
        arm_status_ = ARM_CALIBRATE;
        // 屏蔽其他状态的控制逻辑，优先进行校准
        this->update();
        last_arm_status_ = arm_status_;
        return;
    }
//    else if(!calibration_seen_)
//    {
//        calibration_seen_ = true;
//        arm_status_ = ARM_CALIBRATE;
//    }

    if(test_store == 1)
        this->setStoreSuckerStatus_OutSide(SUCK);
    else if(test_store == 2)
        this->setStoreSuckerStatus_OutSide(STOP);
    else if(test_store == 3)
        this->setStoreSuckerStatus_InSide(SUCK);
    else if(test_store == 4)
        this->setStoreSuckerStatus_InSide(STOP);

#if ARM_AUTO_DEBUG_NOCHASSIS
    //无底盘下的调试模式
    if(arm_status_ == ARM_AUTO_CONTROL&&arm_ctrlStatus.auto_start == 1)
    {
        auto_ctrl_.now_chassis_speed = get_nowChassisSpeed();
        auto_ctrl_.now_armPosition = get_nowArmPosition();
        auto_ctrl_.now_ChassisPosition = get_nowChassisPose();
    }
#else
    auto_ctrl_.now_chassis_speed = get_nowChassisSpeed();
    auto_ctrl_.now_armPosition = get_nowArmPosition();
    auto_ctrl_.now_ChassisPosition = get_nowChassisPose();
#endif

#if !USE_RC10_AIRJOY
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);

    arm_ctrlStatus.button_click_state =  button_detector_1.update(airjoy_data_.botton_click);
#else
    communication::Lora_communication::GetInstance()->update_airjoy_data(&airjoy_data_);
#endif
    if(arm_status_ == ARM_AUTO_CONTROL)
    {
        if(arm_ctrlStatus.auto_start == 1)
        {
            this->start_toAutoCtrl(true);
        }
        else
        {
            this->start_toAutoCtrl(false);
        }
    }

    // 捕获下降前云台角度：h>=lock_h时持续更新，h<lock_h时冻结
    {
        const float h = this->get_currentJointStatus().launchJoint_Height_;
        if (h >= init_data_.lock_height_)
            pre_descent_angle_ = this->get_currentJointStatus().rotateJoint_angle_;
    }
    switch(arm_status_)
    {
        case ARM_MANUAL_CONTROL:
        {
        #if !USE_RC10_AIRJOY
            if(last_arm_status_ != ARM_MANUAL_CONTROL)
            {
                arm_ctrlStatus.last_manual_store = 0; //切换到手操时候重置存储状态，避免跳变
                store_state_ = store_state::idle; //切换到手操时候重置存储状态，避免跳变
                arm_ctrlStatus.is_store_acting = 0; //切换到手操时候重置存储状态，避免跳变
            }

            if(arm_ctrlStatus.is_store_acting == 0) //非存储动作，正常手操
            {
                manualControl();

                if(arm_ctrlStatus.button_click_state == 2) //双击
                {
                    arm_ctrlStatus.is_store_acting = 2;
                }
                else if(arm_ctrlStatus.button_click_state == 3) //三击
                {
                    if(this->getSuckerStatus() == Sucker_Status_E::SUCK)
                    {
                        arm_ctrlStatus.is_store_acting = 2; //当前处于吸附状态，取出
                    }
                    else
                        arm_ctrlStatus.is_store_acting = 1;
                }
                arm_ctrlStatus.last_manual_store = 0;
                store_state_ = store_state::idle;
            }
            else if(arm_ctrlStatus.is_store_acting == 2) //存储
            {
                if(manual_store(0x00))
                { 
                    arm_ctrlStatus.is_store_acting = 0;
                }
                arm_ctrlStatus.last_manual_store = 2;
            }
            else if(arm_ctrlStatus.is_store_acting == 1) //取出
            {
                if(manual_takeout(0x00))
                {
                    arm_ctrlStatus.is_store_acting = 0;
                }
                arm_ctrlStatus.last_manual_store = 1;
            }
            else
            {
                arm_ctrlStatus.is_store_acting = 0;
            }
        #else
            if(last_arm_status_ != ARM_MANUAL_CONTROL)
            {
                arm_ctrlStatus.last_manual_store = 0;
                store_state_ = store_state::idle;
                arm_ctrlStatus.is_store_acting = 0;
            }
            else
            {
                arm_d_pad_ctrl();
            }
        #endif
            break;
        }
        case ARM_SEMI_AUTO_CONTROL:
        {
        #if USE_RC10_AIRJOY
            if(last_arm_status_ != ARM_SEMI_AUTO_CONTROL)
            {
                arm_ctrlStatus.last_manual_store = 0;
                store_state_ = store_state::idle;
                arm_ctrlStatus.is_store_acting = 0;
            }
            else
            {
                arm_d_pad_ctrl();
            }
        #endif
            break;
        }

        case ARM_COMP_SEMI_CONTROL:
        {
        #if USE_RC10_AIRJOY
            if(last_arm_status_ != ARM_COMP_SEMI_CONTROL)
            {
                arm_ctrlStatus.last_manual_store = 0;
                store_state_ = store_state::idle;
                arm_ctrlStatus.is_store_acting = 0;
            }
            else
            {
                arm_d_pad_ctrl();
            }
        #endif
            break;
        }

        case ARM_MANUAL_LOW_LEVEL:
        {
        #if USE_RC10_AIRJOY
            if(last_arm_status_ != ARM_MANUAL_LOW_LEVEL)
            {
                arm_ctrlStatus.last_manual_store = 0;
                store_state_ = store_state::idle;
                arm_ctrlStatus.is_store_acting = 0;
            }
            else
            {
                arm_d_pad_ctrl();
            }
        #endif
            break;
        }

        case ARM_AUTO_CONTROL:
        {
            autoControl();
            break;
        }   
        case ARM_STOP: 
        {
            // 停止状态
            stop();
            break;
        }
          
        case ARM_IDLE:
        {
            // 待机
            idle();
            break;
        }

        case ARM_DEBUG:
        {
            // 暂时无用了
            if(arm_ctrlStatus.debug_start == 1)
                debug();

            break;
        }
            
        case ARM_CALIBRATE:
        {
            //无事发生
            break;
        }
        default:
            break;
    }
    this->update(); //更新电机状态

    // pitch 电机使能检查：位于 update() 之后，确保 motorEnable() 后
    // 下一次循环的 update() 能恢复 dm_mode_ = MOTOR_POSVEL_MODE
    if((motor_pitch_->getErrorNum() == 0x00 || !this->is_pitchEnable_))
    {
        motor_pitch_->motorEnable();
        this->is_pitchEnable_ = true;
    }

    last_arm_status_ = arm_status_;
}

bool ArmSetup::manual_pickup()
{
    static bool is_pick = false;
    static float pickup_start_time = 0.0f;
    this->set_PitchAngle(0.0f);
    this->setSuckerStatus(Sucker_Status_E::SUCK);
    this->set_RotateAngle(90.0f);
    
    if(this->get_currentJointStatus().launchJoint_Height_ < init_data_.pick_up_height_ + 0.02f)
    {
        this->set_LaunchHeight(init_data_.pick_up_height_);
    }

    if(std::fabs(this->get_currentJointStatus().rotateJoint_angle_ - 90.0f) < 1.0f)
    {
        this->set_LaunchHeight(init_data_.pick_up_height_);

        if(std::fabs(this->get_currentJointStatus().launchJoint_Height_ - init_data_.pick_up_height_) < 0.008f)
        {
            is_pick = true;
            pickup_start_time = TimeStamp::getInstance().getSeconds();
        }
    }

    if(is_pick && TimeStamp::getInstance().getSeconds() - pickup_start_time > 0.5f && pickup_start_time > 0.5f)
    {
        this->set_LaunchHeight(init_data_.putdown_height_);
        if(std::fabs(this->get_currentJointStatus().launchJoint_Height_ - init_data_.putdown_height_) < 0.008f)
        {
            is_pick = false;
            pickup_start_time = 0.0f;
            return true;
        }
    }
    
    return false;
}


bool ArmSetup::manual_putdown()
{
    this->set_PitchAngle(init_data_.pitch_lift_angle_);
    this->set_LaunchHeight(init_data_.putdown_height_);
    this->set_RotateAngle(90.0f);

    if(std::fabs(this->get_currentJointStatus().launchJoint_Height_ - init_data_.putdown_height_) < 0.008f 
        && std::fabs(this->get_currentJointStatus().rotateJoint_angle_ - 90.0f) < 1.0f)
    {
        this->set_StretchLength(init_data_.max_stretchLength_);
        if(std::fabs(this->get_currentJointStatus().stretchJoint_Length_ - init_data_.max_stretchLength_) < 0.01f
            && arm_ctrlStatus.can_putdown)
        {
            this->setSuckerStatus(Sucker_Status_E::STOP);
            return true;
        }
    }

    return false;
}

void ArmSetup::manualControl_lowLevel()
{
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    if(last_arm_status_ != ARM_MANUAL_LOW_LEVEL || arm_ctrlStatus.last_manual_store != 0)
    {
        last_joint_status_ = this->get_currentJointStatus();
        target_joint_status_ = last_joint_status_;
        last_arm_status_ = ARM_MANUAL_LOW_LEVEL;
    }
#if USE_RC10_AIRJOY
    if(airjoy_data_.SWE == 0x00)
    {
        target_joint_status_.launchJoint_Height_ = this->get_currentJointStatus().launchJoint_Height_;
        target_joint_status_.rotateJoint_angle_ = this->get_currentJointStatus().rotateJoint_angle_;
    }
    else if(airjoy_data_.SWE == 0x01)
    {
        const float h = this->get_currentJointStatus().launchJoint_Height_;
        const float current_angle = this->get_currentJointStatus().rotateJoint_angle_;
        const bool pre_near_zero = (pre_descent_angle_ <= 3.0f || pre_descent_angle_ >= 357.0f);
        const bool angle_off_zero = !(current_angle <= 0.8f || current_angle >= 359.2f);
        const bool brake_active = pre_near_zero
                            && (h < init_data_.lock_height_ + 0.01f)
                            && angle_off_zero;
        // 升降==
        if(_tool_Abs(airjoy_data_.right_y) > 0.2f)
        {
            float next_height = this->get_currentJointStatus().launchJoint_Height_ ;
            if(airjoy_data_.right_y > 0.3f)
                next_height += manual_control.launch_rate;
            else if(airjoy_data_.right_y < -0.3f)
                next_height -= manual_control.launch_rate;
            else
                next_height = this->get_currentJointStatus().launchJoint_Height_ ;

            // 下降刹车：只有下降前云台在0.0±3.0度时才激活
            if (brake_active) {
                if (next_height > target_joint_status_.launchJoint_Height_)
                    next_height = target_joint_status_.launchJoint_Height_; // 禁止抬升

                if (next_height < init_data_.lock_height_)
                    next_height = init_data_.lock_height_; // 禁止降到lock_h以下
            }
            target_joint_status_.launchJoint_Height_ = next_height;
        }
        else
            target_joint_status_.launchJoint_Height_ = this->get_currentJointStatus().launchJoint_Height_; // 保持不变

        // 云台旋转控制 ==
        if(airjoy_data_.right_x > 0.5f)
            target_joint_status_.rotateJoint_angle_ -= manual_control.rotate_rate;
        else if(airjoy_data_.right_x < -0.5f)
            target_joint_status_.rotateJoint_angle_ += manual_control.rotate_rate;

        target_joint_status_.rotateJoint_angle_ = sanitizeRotateAngle(target_joint_status_.rotateJoint_angle_);
        target_joint_status_.rotateJoint_angle_ = normalize_deg_0_360(target_joint_status_.rotateJoint_angle_);

        float re = init_data_.rotate_end;
        if (re < 250.0f || re > 270.0f) re = 265.0f;
        float t = target_joint_status_.rotateJoint_angle_;
        if (t > init_data_.rotate_start && t < re)
        {
            float d135 = t - init_data_.rotate_start;
            float dre = re - t;
            target_joint_status_.rotateJoint_angle_ = (d135 < dre) ? init_data_.rotate_start : re;
        }

        // 刹车激活时强制自动旋转到0°
        if (brake_active)
            target_joint_status_.rotateJoint_angle_ = 0.0f;
    }

    this->set_LaunchHeight(target_joint_status_.launchJoint_Height_);
    this->set_RotateAngle(target_joint_status_.rotateJoint_angle_);
#endif
}


/**
 * @brief 手动控制
 */
void ArmSetup::manualControl()
{
    // 设置控制模式
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    
   
    if(last_arm_status_ != ARM_MANUAL_CONTROL || arm_ctrlStatus.last_manual_store != 0)//若首次进入此函数，或存储状态发生变化，则进行状态初始化，避免由于状态跳变导致的目标值突变
    {
        
        /*上一次状态*/
        last_joint_status_ = this->get_currentJointStatus();
        target_joint_status_ = last_joint_status_;
    #if !USE_RC10_AIRJOY
        // 绑定伸展状态
        // 判断当前是伸展状态
        // 假设阈值为 max_stretchLength / 2 或者 0.05m
        float current_stretch = this->get_currentJointStatus().stretchJoint_Length_;
        int8_t current_extend_logical = (current_stretch > 0.01f) ? 1 : 0;
        
        // 记录状态
        arm_ctrlStatus.last_manual_extend = current_extend_logical;
        
        // 标记偏移: offset = switch ^ state
        // 定 switch只有01状态
        arm_ctrlStatus.extend_switch_offset = (airjoy_data_.SWA & 0x01) ^ current_extend_logical;


        // 绑定吸盘状态
        int8_t current_sucker_logical = (this->getSuckerStatus() == Sucker_Status_E::SUCK) ? 1 : 0;
        arm_ctrlStatus.last_manual_sucker = current_sucker_logical;     
        arm_ctrlStatus.sucker_switch_offset = (airjoy_data_.SWD & 0x01) ^ current_sucker_logical;

        
        int8_t current_pitch_logical = (_tool_Abs(this->get_currentJointStatus().suckerJoint_angle_ - 90.0f) < 1.0f) ? 1: 0;
        // 判断当前是否在90度附近，假设为开启状态，反之为关闭状态
        arm_ctrlStatus.last_manual_pitch = current_pitch_logical;
        arm_ctrlStatus.pitch_switch_offset = (airjoy_data_.scroll_wheel & 0x01) ^ current_pitch_logical;
    #else
        //绑定伸展状态 由SWB控制
        float current_stretch = this->get_currentJointStatus().stretchJoint_Length_;
        int8_t current_extend_logical = (current_stretch > 0.02f) ? 1 : 0;
        //记录状态
        arm_ctrlStatus.last_manual_extend = current_extend_logical;
        //标记偏移: offset = switch ^ state
        //定SWB只有01状态
        arm_ctrlStatus.extend_switch_offset = (airjoy_data_.SWB & 0x01) ^ current_extend_logical;

        //绑定机械臂吸盘状态 由SWD控制
        int8_t current_sucker_logical = (this->getSuckerStatus() == Sucker_Status_E::SUCK) ? 1 : 0;
        arm_ctrlStatus.last_manual_sucker = current_sucker_logical;
        arm_ctrlStatus.sucker_switch_offset = (airjoy_data_.SWD & 0x01) ^ current_sucker_logical;

        //绑定pitch状态 由SWC控制
        int8_t current_pitch_logical = (_tool_Abs(this->get_currentJointStatus().suckerJoint_angle_ - 90.0f) < 1.0f) ? 1: 0;
        arm_ctrlStatus.last_manual_pitch = current_pitch_logical;
        arm_ctrlStatus.pitch_switch_offset = (airjoy_data_.SWC & 0x01) ^ current_pitch_logical;
    #endif
        last_arm_status_ = ARM_MANUAL_CONTROL;
    }

    // 下降刹车条件计算（作用域覆盖升降和旋转两个区域）
    const float h = this->get_currentJointStatus().launchJoint_Height_;
    const float current_angle = this->get_currentJointStatus().rotateJoint_angle_;
    const bool pre_near_zero = (pre_descent_angle_ <= 3.0f || pre_descent_angle_ >= 357.0f);
    const bool angle_off_zero = !(current_angle <= 0.8f || current_angle >= 359.2f);
    const bool brake_active = pre_near_zero
                           && (h < init_data_.lock_height_ + 0.01f)
                           && angle_off_zero;

    #if USE_RC10_AIRJOY
    if(airjoy_data_.SWE == 0x00)
    {
    #endif
        // 升降==
        if(_tool_Abs(airjoy_data_.right_y) > 0.2f)
        {
            float next_height = this->get_currentJointStatus().launchJoint_Height_ ;
            if(airjoy_data_.right_y > 0.3f)
                next_height += manual_control.launch_rate;
            else if(airjoy_data_.right_y < -0.3f)
                next_height -= manual_control.launch_rate;
            else
                next_height = this->get_currentJointStatus().launchJoint_Height_ ;

            // 下降刹车：只有下降前云台在0.0±3.0度时才激活
            if (brake_active) {
                if (next_height > target_joint_status_.launchJoint_Height_)
                    next_height = target_joint_status_.launchJoint_Height_; // 禁止抬升

                if (next_height < init_data_.lock_height_)
                    next_height = init_data_.lock_height_; // 禁止降到lock_h以下
            }
            target_joint_status_.launchJoint_Height_ = next_height;
        }
        else
            target_joint_status_.launchJoint_Height_ = this->get_currentJointStatus().launchJoint_Height_; // 保持不变

        // 云台旋转控制 ==
        if(airjoy_data_.right_x > 0.5f)
            target_joint_status_.rotateJoint_angle_ -= manual_control.rotate_rate;
        else if(airjoy_data_.right_x < -0.5f)
            target_joint_status_.rotateJoint_angle_ += manual_control.rotate_rate;

        target_joint_status_.rotateJoint_angle_ = sanitizeRotateAngle(target_joint_status_.rotateJoint_angle_);
        target_joint_status_.rotateJoint_angle_ = normalize_deg_0_360(target_joint_status_.rotateJoint_angle_);

        float re = init_data_.rotate_end;
        if (re < 250.0f || re > 270.0f) re = 265.0f;
        float t = target_joint_status_.rotateJoint_angle_;
        if (t > init_data_.rotate_start && t < re)
        {
            float d135 = t - init_data_.rotate_start;
            float dre = re - t;
            target_joint_status_.rotateJoint_angle_ = (d135 < dre) ? init_data_.rotate_start : re;
        }

        // 刹车激活时强制自动旋转到0°
        if (brake_active)
            target_joint_status_.rotateJoint_angle_ = 0.0f;
    #if USE_RC10_AIRJOY
    }
    else
    {
        target_joint_status_.launchJoint_Height_ = this->get_currentJointStatus().launchJoint_Height_;
        target_joint_status_.rotateJoint_angle_ = this->get_currentJointStatus().rotateJoint_angle_;
    }
    #endif
    //pitch 控制
#if !USE_RC10_AIRJOY
    int8_t target_pitch_logical = (airjoy_data_.scroll_wheel & 0x01) ^ arm_ctrlStatus.pitch_switch_offset;
#else
    int8_t target_pitch_logical = (airjoy_data_.SWC & 0x01) ^ arm_ctrlStatus.pitch_switch_offset;
#endif
    if(target_pitch_logical == 1)
    {
        target_joint_status_.suckerJoint_angle_ = init_data_.pitch_lift_angle_; // 吸盘打开到90度
    }
    else
        target_joint_status_.suckerJoint_angle_ = 0.0f; // 吸盘关闭到0度

    //stretch 控制
    // 记录应当的逻辑状态 logic = switch ^ offset
#if !USE_RC10_AIRJOY
    int8_t target_extend_logical = (airjoy_data_.SWA & 0x01) ^ arm_ctrlStatus.extend_switch_offset;
#else
    int8_t target_extend_logical = (airjoy_data_.SWB & 0x01) ^ arm_ctrlStatus.extend_switch_offset;
#endif
    // 更细记录状态
    arm_ctrlStatus.last_manual_extend = target_extend_logical;

    if(target_extend_logical == 0)
        target_joint_status_.stretchJoint_Length_ = 0.0f; // 收回
    else
        target_joint_status_.stretchJoint_Length_ = this->init_data_.max_stretchLength_; // 展开到最大位置

    //吸盘开启
#if !USE_RC10_AIRJOY
    int8_t target_sucker_logical = (airjoy_data_.SWD & 0x01) ^ arm_ctrlStatus.sucker_switch_offset;
#else
    int8_t target_sucker_logical = (airjoy_data_.SWD & 0x01) ^ arm_ctrlStatus.sucker_switch_offset;
#endif
    // 更新记录
    arm_ctrlStatus.last_manual_sucker = target_sucker_logical;

    if(target_sucker_logical == 1) 
        this->setSuckerStatus(Sucker_Status_E::SUCK);
    else
        this->setSuckerStatus(Sucker_Status_E::STOP);

    this->set_LaunchHeight(target_joint_status_.launchJoint_Height_);
    this->set_RotateAngle(target_joint_status_.rotateJoint_angle_);
    this->set_StretchLength(target_joint_status_.stretchJoint_Length_);
    this->set_PitchAngle(target_joint_status_.suckerJoint_angle_);
}

/**
 * @brief 十字键统一处理（AirJoy 路径）
 * @details 根据 arm_status_ 选择活跃函数和启用的十字键方向:
 *          COMP_SEMI: 左=存储  右=取出  上=放下 (无拾取)
 *          其他模式:  左=存储  右=取出  下=拾取 (无放下)
 */
void ArmSetup::arm_d_pad_ctrl()
{
#if USE_RC10_AIRJOY
    const bool enable_pickup  = (arm_status_ != ARM_COMP_SEMI_CONTROL);
    const bool enable_putdown = (arm_status_ == ARM_COMP_SEMI_CONTROL);

    if (arm_ctrlStatus.is_store_acting == 0)
    {
        // --- 活跃函数 ---
        if (arm_status_ == ARM_MANUAL_CONTROL)
            manualControl();
        else if (arm_status_ == ARM_SEMI_AUTO_CONTROL || arm_status_ == ARM_COMP_SEMI_CONTROL)
            idle();
        else if (arm_status_ == ARM_MANUAL_LOW_LEVEL)
            manualControl_lowLevel();

        // --- 十字键左：存储 ---
        if (airjoy_data_.d_pad_left == 1 && is_d_pad_left_clicked == 0)
        {
            arm_ctrlStatus.is_store_acting = 2;
            is_d_pad_left_clicked = 1;
        }
        else if (airjoy_data_.d_pad_left == 0 && is_d_pad_left_clicked == 1)
        {
            is_d_pad_left_clicked = 0;
        }

        // --- 十字键右：取出 ---
        if (airjoy_data_.d_pad_right == 1 && is_d_pad_right_clicked == 0)
        {
            arm_ctrlStatus.is_store_acting = 1;
            is_d_pad_right_clicked = 1;
        }
        else if (airjoy_data_.d_pad_right == 0 && is_d_pad_right_clicked == 1)
        {
            is_d_pad_right_clicked = 0;
        }

        // --- 十字键下：拾取 (COMP_SEMI 禁用) ---
        if (enable_pickup)
        {
            if (airjoy_data_.d_pad_down == 1 && is_d_pad_down_clicked == 0)
            {
                arm_ctrlStatus.is_store_acting = 3;
                is_d_pad_down_clicked = 1;
            }
            else if (airjoy_data_.d_pad_down == 0 && is_d_pad_down_clicked == 1)
            {
                is_d_pad_down_clicked = 0;
            }
        }

        // --- 十字键上：放下 (仅 COMP_SEMI 启用) ---
        if (enable_putdown)
        {
            if (airjoy_data_.d_pad_up == 1 && is_d_pad_up_clicked == 0)
            {
                arm_ctrlStatus.is_store_acting = 4;
                is_d_pad_up_clicked = 1;
            }
            else if (airjoy_data_.d_pad_up == 0 && is_d_pad_up_clicked == 1)
            {
                is_d_pad_up_clicked = 0;
            }
        }
    }
    else if (arm_ctrlStatus.is_store_acting == 1) // 取出
    {
        int8_t tar = (arm_ctrlStatus.store_manual_mode == OUTSIDE) ? 0x01 : 0x00;
        if (manual_takeout(tar))
            arm_ctrlStatus.is_store_acting = 0;
        arm_ctrlStatus.last_manual_store = 1;
    }
    else if (arm_ctrlStatus.is_store_acting == 2) // 存储
    {
        int8_t tar = (arm_ctrlStatus.store_manual_mode == OUTSIDE) ? 0x00 : 0x01;
        if (manual_store(tar))
            arm_ctrlStatus.is_store_acting = 0;
        arm_ctrlStatus.last_manual_store = 2;
    }
    else if (arm_ctrlStatus.is_store_acting == 3 && enable_pickup) // 拾取
    {
        if (manual_pickup())
            arm_ctrlStatus.is_store_acting = 0;
        arm_ctrlStatus.last_manual_store = 3;
    }
    else if (arm_ctrlStatus.is_store_acting == 4 && enable_putdown) // 放下
    {
        if (manual_putdown())
            arm_ctrlStatus.is_store_acting = 0;
        arm_ctrlStatus.last_manual_store = 4;
    }
#endif
}


bool ArmSetup::manual_store(uint8_t kfs_index)
{ 
    //0x00在外 0x01在内
    // 顶存的话，两个位置目标高度相同，流程差异只有伸不伸展
    static bool is_store = false;
    static float store_start_time = 0.0f;
    float target_store_height = (kfs_index == 0x01) ? 
            this->init_data_.store_height_inside_ : this->init_data_.store_height_outside_;
    float target_back_height = 0.0f;
    if(arm_status_ != ARM_AUTO_CONTROL)
    {
        target_back_height = (kfs_index == 0x01) ?
            this->init_data_.max_launchHeight_ - 0.02f : this->init_data_.max_launchHeight_ - 0.1f;
    }
    else
    {
        target_back_height = init_data_.max_launchHeight_;
    }

    switch(this->store_state_)
    {
        case store_state::idle:
        {
            if(arm_ctrlStatus.last_manual_store != 2 || this->auto_ctrl_.start_to_autoctrl)
            {
                is_store = false;
                store_start_time = 0.1f;
                this->store_state_ = store_state::laucnh_state;
                this->setSuckerStatus(Sucker_Status_E::SUCK); 
                if(kfs_index == 0x00)
                    this->setStoreSuckerStatus_OutSide(Sucker_Status_E::SUCK);
                else if(kfs_index == 0x01)
                    this->setStoreSuckerStatus_InSide(Sucker_Status_E::SUCK);

                this->set_StretchLength(0.0f); //收回

            }
            else
            {
                idle();
            }
            break;
        }

        case store_state::laucnh_state:
        {
            this->set_LaunchHeight(this->init_data_.max_launchHeight_);
            //全部顶存
            if(this->get_currentJointStatus().launchJoint_Height_ >= this->init_data_.max_launchHeight_ - 0.03f)
            {
                this->store_state_ = store_state::rotate_state;
            }
            break;
        }

        case store_state::rotate_state:
        {
            if(kfs_index == 0x00)
                this->set_RotateAngle(270.0f); //存储的目标旋转角度

            else if(kfs_index == 0x01)
                this->set_RotateAngle(310.0f);

            if(this->get_currentJointStatus().launchJoint_Height_ > init_data_.max_launchHeight_ - 0.005f)
            {
                if(this->get_currentJointStatus().rotateJoint_angle_ > 265.0f && this->get_currentJointStatus().rotateJoint_angle_ < 350.0f)
                {
                    if(kfs_index == 0x01)
                    this->set_PitchAngle(0.0f); //吸盘放下

                    if(kfs_index == 0x00)
                        this->set_StretchLength(init_data_.max_stretchLength_); // 伸展到存储位置需要的长度
                    this->store_state_ = store_state::lower_state;
                }
            }
            break;
        }
      
        case store_state::lower_state:
        {
            if(std::fabs(this->get_currentJointStatus().rotateJoint_angle_ - 270.0f) < 1.0f
                && kfs_index == 0x00)
            {
                this->set_PitchAngle(0.0f); //吸盘放下
            }

            if(std::fabs(this->get_currentJointStatus().rotateJoint_angle_ - 270.0f) < 0.2f 
                && std::fabs(this->get_currentJointStatus().suckerJoint_angle_ - 0.0f) < 0.2f && kfs_index == 0x00)
            {
                if(this->get_currentJointStatus().stretchJoint_Length_ > init_data_.max_stretchLength_ - 0.01f)
                {
                    this->set_LaunchHeight(init_data_.store_height_inside_); // 降低到存储高度
                }
            }
            else if(std::fabs(this->get_currentJointStatus().rotateJoint_angle_ - 310.0f) < 1.0f
                && kfs_index == 0x01 && this->get_currentJointStatus().suckerJoint_angle_ < 0.2f)
            {
                this->set_LaunchHeight(init_data_.store_height_inside_); // 降低到存储高度
            }

            if(std::fabs(this->get_currentJointStatus().launchJoint_Height_ - init_data_.store_height_inside_) < 0.01f)
            {
                is_store = true;
                store_start_time = TimeStamp::getInstance().getSeconds();
                this->store_state_ = store_state::outstate1;
            }
            break;
        }

        case store_state::outstate1:
        {
            if(is_store && TimeStamp::getInstance().getSeconds() - store_start_time > 0.6f && store_start_time > 0.1f )
            {
                this->setSuckerStatus(Sucker_Status_E::STOP); // 停止吸盘
                this->set_LaunchHeight(init_data_.max_launchHeight_); //提升到最高
                if(this->get_currentJointStatus().launchJoint_Height_ > init_data_.max_launchHeight_ - 0.01f)
                {

                    if(arm_status_ == ARM_AUTO_CONTROL)
                        this->set_RotateAngle(90.0f);
                    else
                        this->set_RotateAngle(0.0f); //旋转回0度

                    this->set_StretchLength(0.0f); //收回
                    this->store_state_ = store_state::outstate2;
                }
            }
            break;
        }

        case store_state::outstate2:
        {
            if(//std::fabs(this->get_currentJointStatus().launchJoint_Height_ - target_back_height) < 0.01f &&
                 (std::fabs(this->get_currentJointStatus().rotateJoint_angle_ - 0.0f) < 5.0f
                || std::fabs(this->get_currentJointStatus().rotateJoint_angle_ - 360.0f) < 5.0f))
            { 
                this->store_state_ = store_state::idle;
                return true;
            }
            break;
        }
    }

    return false;
}




bool ArmSetup::manual_takeout(uint8_t kfs_index)
{
    // 0x00在里 0x01在外
    // 采用顶存的方式，两个位置目标高度相同，流程差异只有伸不伸展
    static bool is_catch = false;
    static float catch_time = 0.0f; //记录碰到KFS的时间
    float target_store_height = (kfs_index == 0x00) ? 
            this->init_data_.store_height_inside_ : this->init_data_.store_height_outside_;

    float target_back_height = (kfs_index == 0x00) ?
            this->init_data_.max_launchHeight_ - 0.02f : this->init_data_.max_launchHeight_ - 0.1f;
    float target_rotate = 0.0f;
    switch(this->store_state_)
    {
        case store_state::idle:
        {
            if(arm_ctrlStatus.last_manual_store != 2)
            {
                this->store_state_ = store_state::laucnh_state;
                is_catch = false;
                catch_time = 0.0f;
            }
            else
            {
                idle();
            }
            break;
        }
        case store_state::laucnh_state:
        {
            this->set_LaunchHeight(this->init_data_.max_launchHeight_);
            this->set_PitchAngle(0.0f);
            this->set_StretchLength(0.0f);
            this->store_state_ = store_state::rotate_state;
            break;
        }

        case store_state::rotate_state:
        {
            
            if(kfs_index == 0x01)
                target_rotate = 270.0f; //存储的目标旋转角度
            else if(kfs_index == 0x00)
                target_rotate = 310.0f; //存储的目标旋转角度

            this->setSuckerStatus(Sucker_Status_E::SUCK);

            if(this->get_currentJointStatus().launchJoint_Height_ > init_data_.max_launchHeight_ - 0.02f)
            {
                this->set_RotateAngle(target_rotate);
                if(kfs_index == 0x01)
                    this->set_StretchLength(init_data_.max_stretchLength_); // 伸展到存储位置需要的长度
                this->store_state_ = store_state::lower_state;
            }
            
            break;
        }

        case store_state::lower_state:
        {
            if(this->get_currentJointStatus().rotateJoint_angle_ > 265.0f && this->get_currentJointStatus().rotateJoint_angle_ < 350.0f)
            {
                if(kfs_index == 0x01)
                    this->set_StretchLength(init_data_.max_stretchLength_); // 伸展到存储位置需要的长度
            }

            if(std::fabs(this->get_currentJointStatus().rotateJoint_angle_ - target_rotate) < 1.0f
                && std::fabs(this->get_currentJointStatus().suckerJoint_angle_ - 0.0f) < 0.2f)
            {
                this->set_LaunchHeight(init_data_.store_height_inside_); // 降低到存储高度
                this->store_state_ = store_state::outstate1;
            }
            break;
        }

        case store_state::outstate1:
        {
            if(!is_catch && std::fabs(this->get_currentJointStatus().launchJoint_Height_ - init_data_.store_height_inside_) < 0.01f)
            {
                if(kfs_index == 0x00)
                    this->setStoreSuckerStatus_InSide(Sucker_Status_E::STOP); // 停止存储吸盘
                else if(kfs_index == 0x01)
                    this->setStoreSuckerStatus_OutSide(Sucker_Status_E::STOP); // 停止存储吸盘

                catch_time = TimeStamp::getInstance().getSeconds(); //记录降低完成的时间
                is_catch = true;
            }

            if(is_catch && TimeStamp::getInstance().getSeconds() - catch_time > 0.6f && catch_time > 0.1f)
            {
                store_state_ = store_state::outstate2;
                this->set_LaunchHeight(init_data_.max_launchHeight_);//提升到最高
            }
            break;
        }

        case store_state::outstate2:
        {
            if(this->get_currentJointStatus().launchJoint_Height_ > init_data_.max_launchHeight_ - 0.01f)
            {
                if(kfs_index == 0x00)
                {
                    this->set_RotateAngle(0.0f);
                    this->store_state_ = store_state::idle;
                    return true;
                }
                else
                {
                    this->set_RotateAngle(0.0f);
                    this->set_StretchLength(0.0f); // 收回

                    if(std::fabs(this->get_currentJointStatus().stretchJoint_Length_ - 0.0f) < 0.02f)
                    {
                        this->store_state_ = store_state::idle;
                        return true;
                    }
                }
            }
            break;
        }
    }
    return false;
}


/*=======================================================*/

/**
 * @brief 如果有两个目标KFS，则第一个KFS拾取完后放到存储机构
 *        第二个KFS拾取完后留在吸盘上
 *        如果没有第二个，就吸在吸盘上，不必放到存储机构
 *  
 *        寻自动
 * 
 * 自动计算逻辑遵从串联臂自动逻辑末尾的数学公式
 */
void ArmSetup::autoControl()
{
    // 设置控制模式
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);


    if(last_arm_status_ != ARM_AUTO_CONTROL || auto_ctrl_.start_to_autoctrl != 1)//若首次进入此函数，或自动控制启动状态发生变化，则进行状态初始化，避免由于状态跳变导致的目标值突变
    {
        auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_DONE; //自动控制状态机回到初始状态
        
            auto_ctrl_.flag.isrecalcPath = false; //路径重计算标志
            auto_ctrl_.now_targetIndex = 0; //当前目标KFS索引初始化
            auto_ctrl_.flag.back_time = 0.0f;
            last_arm_status_ = ARM_AUTO_CONTROL;
    }
    
    if(auto_ctrl_.targetKFS[0] == 0)
        return; //没有目标KFS，直接返回

    //执行自动控制
    switch(auto_ctrl_.kfs_num)
    {
        case ONLY_ONE:
        {
            auto_stillnessOne();
            break;
        }
        
        case TWO_OR_THREE: //鍋?
        {
            auto_stillnessTwo();
            break;
        }
    }

}

/*
    新版机械臂的流程  和老版流程有不少不同，需要重写
    (1)若是顶吸： 
        执行state_to_waitStillness抬到最高，并将pitch设置为90度
        接着执行state_alignStillness对齐 接近之后执行state_extStillness伸长到目标KFS位置
        然后执行state_lowerStillness降低到目标KFS位置，并打开吸盘。(Lower阶段降到临界高度后停下，等待canExtend放行再下降到目标位置)
        之后执行state_launchStillness抬升到安全高度，最后执行state_backStillness返回初始位置。


    (2)若是侧吸：
        执行state_to_waitStillness抬到最高，并将pitch设置为0度
        接着执行state_alignStillness对齐 接近之后执行state_lowerStillness降低到目标KFS位置，并打开吸盘。
        之后执行state_extStillness伸长到安全位置，最后执行state_backStillness返回初始位置。
*/

// 流程函数 停下拾取==============
#if ARM_VERSION == 1

#else
// VERSION 0 的 纯侧吸版本
// 流程函数 停下拾取==============
void ArmSetup::auto_stillnessOne()
{
    switch(auto_ctrl_.now_state)
    {
        case ARM_AUTO_STILLNESS_E::STATE_DONE:
        {
            if(auto_ctrl_.start_to_autoctrl)
            {
                if(!auto_ctrl_.flag.isrecalcPath)
                {
                    this->set_TargetKFS(auto_ctrl_.targetKFS[0], 0, 0);
                    auto_ctrl_.now_targetIndex = 0;


                    auto_ctrl_.flag.isrecalcPath = true;//重新计算路径标志
                    auto_ctrl_.flag.canExtend = false; //重置伸展许可
                    auto_ctrl_.flag.canChassisStart = false; //重置底盘启动许可
                    auto_ctrl_.flag.isExtReach = false;
                    auto_ctrl_.flag.reach_finishTimeStore = 0.0f;
                }

                auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_TO_WAIT;
            }
            else
            {
                idle();
                auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_DONE; //保持在完成状态
            }
            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_TO_WAIT:
        {
            if(state_to_waitStillness(auto_ctrl_.targetKFS[0]))
            {
                auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_ALIGN;
            }
            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_ALIGN:
        {
            if(state_alignStillness(auto_ctrl_.targetKFS[0]))
            {
                auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_LOWER;
            }
            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_LOWER:
        {
            if(state_lowerStillness(auto_ctrl_.targetKFS[0]))
            {
                auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_EXT;
            }
            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_EXT:
        {
            if(auto_ctrl_.flag.canExtend)
            {
                if(state_extStillness(auto_ctrl_.targetKFS[0]))
                {
                    auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_LAUNCH;
                }
            }
            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_LAUNCH:
        {
            if(state_launchStillness(auto_ctrl_.targetKFS[0]))
            {
                auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_BACK;
            }
            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_BACK:
        {
            if(state_backStillness(auto_ctrl_.targetKFS[0]))
            {
                auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_DONE;
                arm_ctrlStatus.auto_start = 0;
                auto_ctrl_.start_to_autoctrl = false; //完车一次流程后重置启动状态
                auto_ctrl_.flag.isrecalcPath = false; //重置路径计算标志
            }
            break;
        }

        default:
            break;
    }
}

void ArmSetup::auto_stillnessTwo()
{
    //大体执行流程和stillnessOne一样,
    //但目前没有做存储机构，所以第一个KFS就在back阶段直接放下。
    switch(auto_ctrl_.now_state)
    {
        case ARM_AUTO_STILLNESS_E::STATE_DONE:
        {
            if(auto_ctrl_.start_to_autoctrl)
            {
                if(!auto_ctrl_.flag.isrecalcPath)
                {
                    this->set_TargetKFS(auto_ctrl_.targetKFS[0], auto_ctrl_.targetKFS[1], auto_ctrl_.targetKFS[2]);
                    auto_ctrl_.now_targetIndex = 0;

                    auto_ctrl_.flag.isrecalcPath = true;//重新计算路径标志，确保路径只在流程开始时计算一次
                    auto_ctrl_.flag.canExtend = false; //重置伸展许可，等待自定义流程伸展
                    auto_ctrl_.flag.canChassisStart = false; //重置底盘启动许可
                    auto_ctrl_.flag.isExtReach = false;
                    auto_ctrl_.flag.reach_finishTimeStore = 0.0f;
                }

                auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_TO_WAIT;
            }
            else
            {
                idle();
                auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_DONE; //保持在完成状态
            }
            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_TO_WAIT:
        {
            if(state_to_waitStillness(auto_ctrl_.targetKFS[auto_ctrl_.now_targetIndex]))
            {
                auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_ALIGN;
            }
            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_ALIGN:
        {
            if(state_alignStillness(auto_ctrl_.targetKFS[auto_ctrl_.now_targetIndex]))
            {
                auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_LOWER;
            }
            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_LOWER:
        {
            if(state_lowerStillness(auto_ctrl_.targetKFS[auto_ctrl_.now_targetIndex]))
            {
                auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_EXT;
            }
            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_EXT:
        {
            if(auto_ctrl_.flag.canExtend)
            {
                if(state_extStillness(auto_ctrl_.targetKFS[auto_ctrl_.now_targetIndex]))
                {
                    auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_LAUNCH;
                }
            }
            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_LAUNCH:
        {
            if(state_launchStillness(auto_ctrl_.targetKFS[auto_ctrl_.now_targetIndex]))
            {
                auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_BACK;
            }
            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_BACK:
        {
            auto_ctrl_.flag.canChassisStart = false;
            if(this->auto_ctrl_.now_targetIndex == 0)
            {
                if(!auto_ctrl_.flag.isbackdone)
                {
                    arm_ctrlStatus.last_manual_store = 0; // 强制激活 idle 入口
                    if(manual_store(0x00))
                    {
                        auto_ctrl_.flag.back_time = TimeStamp::getInstance().getSeconds();
                        auto_ctrl_.flag.isbackdone = true;
                    }
                }
                else if(TimeStamp::getInstance().getSeconds() - auto_ctrl_.flag.back_time >= 0.3f)
                {
                    this->setSuckerStatus(Sucker_Status_E::STOP);
                    auto_ctrl_.now_targetIndex ++;

                    auto_ctrl_.flag.canExtend = false;
                    auto_ctrl_.flag.canChassisStart = false;
                    auto_ctrl_.flag.isExtReach = false;
                    auto_ctrl_.flag.reach_finishTimeStore = 0.0f;
                    auto_ctrl_.flag.isbackdone = false;
                    auto_ctrl_.flag.back_time = 0.0f;

                    this->set_LaunchHeight(this->init_data_.max_launchCatch_Height_);
                    auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_TO_WAIT;
                }
            }
            else if(this->auto_ctrl_.now_targetIndex == 1 && auto_ctrl_.targetKFS[2] != 0)
            {
                if(!auto_ctrl_.flag.isbackdone)
                {
                    arm_ctrlStatus.last_manual_store = 0; // 强制激活 idle 入口
                    if(manual_store(0x01))
                    {
                        auto_ctrl_.flag.back_time = TimeStamp::getInstance().getSeconds();
                        auto_ctrl_.flag.isbackdone = true;
                    }
                }
                else if(TimeStamp::getInstance().getSeconds() - auto_ctrl_.flag.back_time >= 0.3f)
                {
                    this->setSuckerStatus(Sucker_Status_E::STOP);
                    auto_ctrl_.now_targetIndex ++;

                    auto_ctrl_.flag.canExtend = false;
                    auto_ctrl_.flag.canChassisStart = false;
                    auto_ctrl_.flag.isExtReach = false;
                    auto_ctrl_.flag.reach_finishTimeStore = 0.0f;
                    auto_ctrl_.flag.isbackdone = false;
                    auto_ctrl_.flag.back_time = 0.0f;

                    this->set_LaunchHeight(this->init_data_.max_launchCatch_Height_);
                    auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_TO_WAIT;
                }
            }
            else
            {
                if(state_backStillness(auto_ctrl_.targetKFS[auto_ctrl_.now_targetIndex]))
                {
                    arm_ctrlStatus.auto_start = 0;
                    auto_ctrl_.start_to_autoctrl = false; //完车一次流程后重置启动状态
                    auto_ctrl_.flag.isrecalcPath = false; //重置路径计算标志
                    auto_ctrl_.now_targetIndex = 1; //重置目标索引
                    auto_ctrl_.flag.back_time = 0.0f; //重置返回时间
                    auto_ctrl_.flag.isbackdone = false; //重置返回完成标志
                    auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_OVER;
                }
                
            }
            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_OVER:
        {
            //完成后待机
            idle();
            break;
        }

        default:
            break;
    }
}

//流程函数 行进间拾取==============
bool ArmSetup::state_to_waitStillness(int targetKFS)
{
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);

    float target_height = 0.0f;
    this->set_StretchLength(0.0f);
    target_height = this->init_data_.max_launchCatch_Height_; //直接伸展到最高，等待行进间拾取
    if(isRotateAllowed(this->get_currentJointStatus().rotateJoint_angle_) 
        || std::fabs(this->get_currentJointStatus().rotateJoint_angle_ - 360.0f) < 2.0f
        || std::fabs(this->get_currentJointStatus().rotateJoint_angle_ - 0.0f) < 2.0f)
        this->set_LaunchHeight(target_height); //伸展到目标高度
    else
    {
        this->set_LaunchHeight(this->get_currentJointStatus().launchJoint_Height_); //保持当前高度不变
        float sanitized_angle = sanitizeRotateAngle(this->get_currentJointStatus().rotateJoint_angle_);
        this->set_RotateAngle(sanitized_angle); //旋转到安全区域
    }                                                                                                                                                               

    if(_tool_Abs(this->get_currentJointStatus().launchJoint_Height_ - target_height) < 0.01f)
        return true;
    else
        return false;
}

bool ArmSetup::state_alignStillness(int targetKFS)
{
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    
    this->set_PitchAngle(this->init_data_.pitch_lift_angle_); //pitch抬平

    //对准kfs
    this->set_RotateAngle(90.0f);

    if(_tool_Abs(this->get_currentJointStatus().rotateJoint_angle_ - 90.0f) < 2.0f)
        return true;
    else
        return false;
}

bool ArmSetup::state_lowerStillness(int targetKFS)
{
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);

    float targetLowerHeight = 0.0f; //目标kfs高度
    float catch_offset = 0.0f;
    if(auto_ctrl_.now_targetIndex == 0)
        catch_offset = -0.07f;
    else if(auto_ctrl_.now_targetIndex == 1)
        catch_offset = 0.00f;
    else
        catch_offset = 0.0f;
    float kfs_h = GetKFSHeight(targetKFS);
    if(kfs_h == 0.2f)
        targetLowerHeight = init_data_.catch_20height;
    else if(kfs_h == 0.4f)
        targetLowerHeight = this->init_data_.catch_40height + catch_offset;
    else if(kfs_h == 0.6f)
        targetLowerHeight = this->init_data_.catch_60height + catch_offset;
    else
        targetLowerHeight = this->init_data_.max_launchCatch_Height_ + catch_offset;

    bool canLower = false;
    canLower = MF_AutoCtrler::isInTargetMap(auto_ctrl_.now_ChassisPosition,
                                            auto_ctrl_.pathInfo.MFroad[auto_ctrl_.now_targetIndex],
                                            0.45f); //判断是否可以开始下降
    if(canLower)
    {
        this->set_LaunchHeight(targetLowerHeight); //下降到目标高度
        this->setSuckerStatus(Sucker_Status_E::SUCK); //下降时打开吸盘
    }
    else
        return false;
        
    if(_tool_Abs(this->get_currentJointStatus().launchJoint_Height_ - targetLowerHeight) < 0.02f)
        return true;
    else
        return false;
}


bool ArmSetup::state_extStillness(int targetKFS)
{
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    
    this->set_StretchLength(this->init_data_.max_stretchLength_); //伸展到最大长度

    if(_tool_Abs(this->get_currentJointStatus().stretchJoint_Length_ - 
            this->init_data_.max_stretchLength_) < 0.02f)//伸展完成到目标
    {
        if(!auto_ctrl_.flag.isExtReach)
        {
            auto_ctrl_.flag.reach_finishTimeStore = TimeStamp::getInstance().getSeconds(); //记录伸展完成的时间
            auto_ctrl_.flag.isExtReach = true;
        }
    }
    
    const float now_s = TimeStamp::getInstance().getSeconds();
    if(auto_ctrl_.flag.isExtReach && (now_s - auto_ctrl_.flag.reach_finishTimeStore) >= 0.2f)
    {
        this->set_StretchLength(0.0f); //超过0.2秒后收回，准备行进间放置
        return true;
    }

    return false;
}

bool ArmSetup::state_launchStillness(int targetKFS)
{
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    float canMoveHeight = 0.0f;//是否可以移动的高度阈值，置
    float kfs_h = GetKFSHeight(targetKFS);
    if(kfs_h == 0.2f)
        canMoveHeight = this->init_data_.catch_40height - 0.1f;
    else if(kfs_h == 0.4f)
        canMoveHeight = this->init_data_.max_launchCatch_Height_;
    else if(kfs_h == 0.6f)
        canMoveHeight = this->init_data_.max_launchCatch_Height_;
    else
        canMoveHeight = this->init_data_.max_launchCatch_Height_;

    this->set_LaunchHeight(this->init_data_.max_launchCatch_Height_); //伸展到最大高度，准备移动

    if(this->get_currentJointStatus().launchJoint_Height_ > canMoveHeight - 0.02f)
    {
        auto_ctrl_.flag.canChassisStart = true; //机械臂已经伸展到可以移动的高度
        return true;
    }
    else
        return false;
    // return true;
}

bool ArmSetup::state_backStillness(int targetKFS)
{
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);

    this->set_RotateAngle(0.0f); //旋转到目标位置

    if(_tool_Abs(this->get_currentJointStatus().rotateJoint_angle_ - 0.0f) < 5.0f)
    {
        return true;
    }
    else
        return false;
}

#endif


/*=================================================================*/

/**
 * @brief 停止?
 */
void ArmSetup::stop()
{
    this->set_controlMode(CURRENT_CONTROL_MODE);
    this->motor_launch_->setTargetCurrent(0.0f);
    this->motor_stretch_->setTargetCurrent(0.0f);
    this->motor_rotate_->setTargetCurrent(0.0f);
    // this->motor_pitch_->setTargetCurrent(0.0f);
    // this->setSuckerStatus(Sucker_Status_E::STOP);
}

/**
 * @brief 寻校准
 * 
 * 
 * @brief 上电校准的重新设计
 *        1. 上电后，进入校准模式
 *        2. 伸展电机设计不变，依然是缩到最短
 *        3. pitch电机改为反向抬到180度进行校正
 *        4. 云台的话，后续机械会改成抵住铝管限位，限位重定位为180度。
 *        5. 抬升电机为在最低处，限位重定位为0米
 */

void ArmSetup::calibrateMotor()
{
    this->set_controlMode(CURRENT_CONTROL_MODE); 
    // 上电校准M2006电机位置
    // 给予M2006一个小电流顶住限位，然后计时1s，将当前位置重定位为0度
    if(!arm_ctrlStatus.calibrate_start)
    {
        arm_ctrlStatus.calibrate_startTime = TimeStamp::getInstance().getSeconds();
        arm_ctrlStatus.calibrate_start = true;
    }
    this->motor_stretch_->setTargetCurrent(700.0f); // 给予伸展电机一个小电流顶住限位
    this->motor_launch_->setTargetCurrent(700.0f); // 给予发射电机一个小电流顶住限位

    //this->motor_rotate_->setTargetCurrent(1000.0f);
    if(this->now_time_s_ - arm_ctrlStatus.calibrate_startTime > 1.5f)
    {
        //relocate
        this->motor_stretch_->relocate_totalAngle(0.0f);
        this->motor_rotate_->relocate_totalAngle(this->rotateAngle_to_MotorTotalAngle(0.001f));
        this->motor_launch_->relocate_totalAngle(0.0f);

        // if(this->is_pitchEnable_)
        // {
        //     this->motor_pitch_->motorSetZero();
        // }
        //set current to 0
        this->motor_stretch_->setTargetCurrent(0.0f);
        this->motor_rotate_->setTargetCurrent(0.0f);
        this->motor_launch_->setTargetCurrent(0.0f);


        arm_ctrlStatus.is_calibrating = true;
    }
}

/**
 * @brief 寻待机
 */
void ArmSetup::idle()
{
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);


    if(last_arm_status_ != ARM_IDLE)
    {
        last_joint_status_ = this->get_currentJointStatus();
        target_joint_status_ = last_joint_status_;

        last_arm_status_ = ARM_IDLE;
    }

    this->set_LaunchHeight(target_joint_status_.launchJoint_Height_);
    this->set_StretchLength(target_joint_status_.stretchJoint_Length_);
    this->set_RotateAngle(target_joint_status_.rotateJoint_angle_);
    this->set_PitchAngle(target_joint_status_.suckerJoint_angle_);

    // this->setSuckerStatus(Sucker_Status_E::STOP); // 
}

/**
 * @brief 瀵昏皟璇?
 */
void ArmSetup::debug()
{

    
}

Arm_InitData_S arm_initData = {
    .max_launchHeight_ = 0.39f,
    .max_launchCatch_Height_ = 0.32f,
    .max_stretchLength_ = 0.1358f,
    .arm_length_ = 0.6f,
    .end_link_length_ = 0.08f,

    .stretch_Ratio_ = 0.11421f,
    .launch_Ratio_ = 0.07221f,
    //    .rotate_gearRatio_ = 144.878f,  //
    // .rotate_gearRatio_ = 145.755789f,
    // .rotate_gearRatio_ = 115.179f,
    .rotate_gearRatio_ = 119.687040f,
    .pitch_gearRatio_ = 360.0f,

    .min_rotate_angle_ = 0.0f,
    .max_rotate_angle_ = 359.99999f,

    .rotate_end = 265.0f,
    .rotate_start = 135.0f,

    .safe_height_ = 0.118f,
    .store_height_outside_ = 0.15977098f,
    .store_height_inside_ = 0.359373629f,
    .lock_height_ = 0.055f, 
    .store_ext_length_ =0.0649999976f,

    .Sucker_GPIO_Port = SUCKER_1_GPIO_Port,
    .Sucker_GPIO_Pin = SUCKER_1_Pin,

    .Sucker_Soleniod_GPIO_Port = SUCKER_2_GPIO_Port,
    .Sucker_Soleniod_GPIO_Pin = SUCKER_2_Pin,

    .StoreOutside_GPIO_Port = SUCKER_3_GPIO_Port,
    .StoreOutside_GPIO_Pin = SUCKER_3_Pin,

    .StoreOutside_Soleniod_GPIO_Port = SUCKER_4_GPIO_Port,
    .StoreOutside_Soleniod_GPIO_Pin = SUCKER_4_Pin,

    .StoreInside_GPIO_Port = SUCKER_5_GPIO_Port,
    .StoreInside_GPIO_Pin = SUCKER_5_Pin,

    .StoreInside_Soleniod_GPIO_Port = SUCKER_6_GPIO_Port,
    .StoreInside_Soleniod_GPIO_Pin = SUCKER_6_Pin,

    .max_pitchRPM_ = 150.0f,
};