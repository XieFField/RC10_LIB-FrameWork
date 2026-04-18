#include "Arm_setup.h"

static bool s_has_recorded_strategy = false; //记录是否已经记录过策略

/**
 * @brief 寻主循环
 */
// uint32_t ArmstackHighWaterMark = 0;
void ArmSetup::loop()
{
    if(!arm_ctrlStatus.init_flag)
        return;

	
//	ArmstackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    if(!arm_ctrlStatus.is_calibrating)
    {
        calibrateMotor();
        arm_status_ = ARM_CALIBRATE;
        // 本拍仅执行校准，不进入其他状态分支，避免与上层状态机抢写 arm_status_
        this->update();
        last_arm_status_ = arm_status_;
        return;
    }

#if ARM_AUTO_DEBUG_NOCHASSIS
    //目前使用虚拟坐标进行自控逻辑验证
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



    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);

    if( arm_status_ == ARM_AUTO_CONTROL)
    {
        // static bool ifFirst = true;
        // if(ifFirst)
        // {
            if(arm_ctrlStatus.auto_start == 1)
            {
                this->start_toAutoCtrl(true);
                
            }
            else
            {
                this->start_toAutoCtrl(false);

                #if ARM_AUTOMOVE
                this->auto_ctrl_.now_state = ARM_AUTO_E::STATE_DONE;
                #endif
            }
        // }
    } 


    switch(arm_status_)
    {
        case ARM_MANUAL_CONTROL:
            {
                manualControl();
            }
            break;

        case ARM_AUTO_CONTROL:
            {
                    autoControl();
            }   
            break;

        case ARM_STOP: 
            {
                // 停止状态, 将各个关节回归初始位置后，将电流置零
                stop();
            }
            break;
        case ARM_IDLE:
            {
                // 空闲状态，维持当前状态
                idle();
                break;
            }

        case ARM_DEBUG:
            {
                // 调试状态
                if(arm_ctrlStatus.debug_start == 1)
                    debug();

                break;
            }
            

        case ARM_CALIBRATE:
            {
                // 校准状态
                // 上电校准M2006电机位置
                break;
            }
        default:
            break;
    }


    this->update(); //将控制信息发送给电机
    last_arm_status_ = arm_status_;
}

/**
 * @brief 寻手操
 */
void ArmSetup::manualControl()
{
    this->setRotateStrategy(ROTATE_PATH_SHORTEST);
    // 手动控制函数
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    
    if(last_arm_status_ != ARM_MANUAL_CONTROL)//若首次非此模式，需复制一下上次状态，免得跳变
    {
        /*串联臂*/
        last_joint_status_ = this->get_currentJointStatus();
        target_joint_status_ = last_joint_status_;

        // 绑定伸展状态
        // 判定当前是伸还是缩
        // 假设阈值为 max_stretchLength / 2 或者 0.05m
        float current_stretch = this->get_currentJointStatus().stretchJoint_Length_;
        int8_t current_extend_logical = (current_stretch > 0.01f) ? 1 : 0;
        
        // 记录状态
        arm_ctrlStatus.last_manual_extend = current_extend_logical;
        
        // 计算偏移: offset = switch ^ state
        // 假设 switch只有0和1
        arm_ctrlStatus.extend_switch_offset = (airjoy_data_.SWA & 0x01) ^ current_extend_logical;


        // 绑定吸盘状态
        int8_t current_sucker_logical = (this->getSuckerStatus() == Sucker_Status_E::SUCK) ? 1 : 0;
        arm_ctrlStatus.last_manual_sucker = current_sucker_logical;
        
        arm_ctrlStatus.sucker_switch_offset = (airjoy_data_.SWD & 0x01) ^ current_sucker_logical;

        last_arm_status_ = ARM_MANUAL_CONTROL;
    }


    //升降操控
    if(_tool_Abs(airjoy_data_.right_y) > 0.1f)
    {

        float next_height = this->get_currentJointStatus().launchJoint_Height_ ;
        if(airjoy_data_.right_y > 0.3f)
            next_height += manual_control.launch_rate;
        else if(airjoy_data_.right_y < -0.3f)
            next_height -= manual_control.launch_rate;
        else
            next_height = this->get_currentJointStatus().launchJoint_Height_ ;

        //抬升限制检查：如果不在30~135度的区间时候，云台禁止往上抬升 (从极低高度区进入干涉区)
        if(next_height > target_joint_status_.launchJoint_Height_) // 正在抬升
        {
             float current_angle = this->get_currentJointStatus().rotateJoint_angle_;
             float norm_angle = fmodf(current_angle, 360.0f);
             if(norm_angle < 0.0f) norm_angle += 360.0f;

             if(this->get_currentJointStatus().launchJoint_Height_ < 0.03f)
             {
                 // 目标区域限制是 30~135，所以必须在此范围内才能抬升
                 if(norm_angle < 60.0f || norm_angle > 185.0f)
                 {
                     next_height = target_joint_status_.launchJoint_Height_; // 保持不变
                 }
             }
        }
        target_joint_status_.launchJoint_Height_ = next_height;
    }
    else
        target_joint_status_.launchJoint_Height_ = this->get_currentJointStatus().launchJoint_Height_; // 保持不变

    manual_control.cnt++;
    if(manual_control.cnt > 10)
    {
        if(airjoy_data_.right_x > 0.5f)
            target_joint_status_.rotateJoint_angle_ += manual_control.rotate_rate;
        else if(airjoy_data_.right_x < -0.5f)
            target_joint_status_.rotateJoint_angle_ -= manual_control.rotate_rate;
        else
            target_joint_status_.rotateJoint_angle_ = this->get_currentJointStatus().rotateJoint_angle_; // 保持不变

        target_joint_status_.rotateJoint_angle_ = sanitizeRotateAngle(target_joint_status_.rotateJoint_angle_);
        manual_control.cnt = 0;
    }

    //pitch 开关
    if(airjoy_data_.scroll_wheel == 0x00)
        target_joint_status_.suckerJoint_angle_ = 0.0f; // 末端关节收
    else if(airjoy_data_.scroll_wheel == 0x01)
        target_joint_status_.suckerJoint_angle_ = 95.0f; // 末端关节开

    //stretch 开关
    // 计算当前应当的逻辑状态 logic = switch ^ offset
    int8_t target_extend_logical = (airjoy_data_.SWA & 0x01) ^ arm_ctrlStatus.extend_switch_offset;
    
    // 更新记忆
    arm_ctrlStatus.last_manual_extend = target_extend_logical;

    if(target_extend_logical == 0)
        target_joint_status_.stretchJoint_Length_ = 0.0f; // 伸展关节收回到最小位置
    else
        target_joint_status_.stretchJoint_Length_ = this->init_data_.max_stretchLength_; // 伸展关节伸出到最大位置

    //吸盘开关
    int8_t target_sucker_logical = (airjoy_data_.SWD & 0x01) ^ arm_ctrlStatus.sucker_switch_offset;

    // 更新记忆
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
    // 自动控制函数
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);

//    if(auto_ctrl_.start_to_autoctrl != 1)
//    {
//        idle();
//        return; //未进入自动控制流程，保持空闲状态
//    }


    if(last_arm_status_ != ARM_AUTO_CONTROL || auto_ctrl_.start_to_autoctrl != 1)//若首次非此模式，需初始化一些状态
    {
        auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_DONE; //自动流程状态机回到初始状态
        
            auto_ctrl_.flag.isrecalcPath = false; //路径重计算标志
        auto_ctrl_.now_targetIndex = 0; //当前目标KFS索引重置
        last_arm_status_ = ARM_AUTO_CONTROL;
    }

    /**
     * 整体流程：
     * 1. 升降到目标高度,吸盘pitch90度
     * 2. 锁住云台，等待移动到目标前一桩，云台开始预判旋转时机
     * 3. 旋转执行后，机械臂末端已经对住KFS侧法平面，预判并伸展到KFS位置
     * 4. 云台对准时候就打开吸盘，伸展到底后，停留延迟x(0.3)s，后缩回
     * 5. 缩回后，云台旋转回目标位置(初始or存储机构位置)
     */
    
    if(auto_ctrl_.targetKFS[0] == 0)
        return; //没有目标KFS，直接返回 


    //行进间拾取
    switch(auto_ctrl_.kfs_num)
    {
        case ONLY_ONE:
        {
            auto_stillnessOne();
            break;
        }
        
        case TWO: //做
        {
            auto_stillnessTwo();
            break;
        }
    }
}

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
                    this->set_TargetKFS(auto_ctrl_.targetKFS[0], 0);
                    auto_ctrl_.now_targetIndex = 0;


                    auto_ctrl_.flag.isrecalcPath = true;//重置路径重计算标志，确保路径只在流程开始时计算一次
                    auto_ctrl_.flag.canExtend = false; //重置伸展许可，等待自动控制流程放行
                    auto_ctrl_.flag.canChassisStart = false; //重置底盘移动许可
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
                #if ARM_AUTO_DEBUG_NOCHASSIS
                // auto_ctrl_.flag.canExtend = true; //放行进入伸展阶段
                #endif
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
                auto_ctrl_.start_to_autoctrl = false; //完成一次流程后，重置自动控制启动条件
                auto_ctrl_.flag.isrecalcPath = false; //重置路径重计算标志
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

                // if(auto_ctrl_.now_targetIndex > 1)
                // {
                //     auto_ctrl_.start_to_autoctrl = false; //防止越界访问
                //     idle();
                //     auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_DONE; //保持在完成状态
                //     return;
                // }

                if(!auto_ctrl_.flag.isrecalcPath)
                {
                    this->set_TargetKFS(auto_ctrl_.targetKFS[0], auto_ctrl_.targetKFS[1]);
                    auto_ctrl_.now_targetIndex = 0;

                    auto_ctrl_.flag.isrecalcPath = true;//重置路径重计算标志，确保路径只在流程开始时计算一次
                    auto_ctrl_.flag.canExtend = false; //重置伸展许可，等待自动控制流程放行
                    auto_ctrl_.flag.canChassisStart = false; //重置底盘移动许可
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
                #if ARM_AUTO_DEBUG_NOCHASSIS
                // auto_ctrl_.flag.canExtend = true; //放行进入伸展阶段
                #endif
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
            if(state_backStillness(auto_ctrl_.targetKFS[auto_ctrl_.now_targetIndex]))
            {
                
                
                if(auto_ctrl_.now_targetIndex == 0)
                {
                    if(!auto_ctrl_.flag.isbackdone)
                    {
                        auto_ctrl_.flag.back_time = TimeStamp::getInstance().getSeconds(); //记录返回开始的时间戳
                        auto_ctrl_.flag.isbackdone = true;
                    }

                    if(auto_ctrl_.flag.isbackdone && (TimeStamp::getInstance().getSeconds() - auto_ctrl_.flag.back_time) >= 1.2f)
                    {
                        this->setSuckerStatus(Sucker_Status_E::STOP); //放下第一个KFS后关闭吸盘
                        auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_TO_WAIT; //继续下一个KFS的流程  
                        auto_ctrl_.now_targetIndex ++; //切换到下一个目标KFS

                        auto_ctrl_.flag.canExtend = false; //重置伸展许可，等待自动控制流程放行
                        auto_ctrl_.flag.canChassisStart = false; //重置底盘移动许可
                        auto_ctrl_.flag.isExtReach = false;
                        auto_ctrl_.flag.reach_finishTimeStore = 0.0f;
                    }
                }
                else if(auto_ctrl_.now_targetIndex > 1)
                {
                    
                    arm_ctrlStatus.auto_start = 0;
                   
                    auto_ctrl_.start_to_autoctrl = false; //完成一次流程后，重置自动控制启动条件
                    auto_ctrl_.flag.isrecalcPath = false; //重置路径重计算标志
//                    auto_ctrl_.now_targetIndex = 1; //防止越界
                    auto_ctrl_.flag.back_time = 0.0f; //重置返回时间
                    auto_ctrl_.flag.isbackdone = false; //重置返回完成标志
                    auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_OVER;
                }
            }
            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_OVER:
        {
            //流程完成，保持在完成状态
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

    // float kfs_height = MF_high[targetKFS -1]; //获取目标KFS高度

    float target_height = 0.0f;

    target_height = this->init_data_.max_launchHeight_; //直接抬升到最高，等待行进间旋转对齐后再放低
    if(isRotateAllowed(this->get_currentJointStatus().rotateJoint_angle_))
        this->set_LaunchHeight(target_height); //抬升到目标高度
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
    
    //改为云台升到最高之后就旋转
    this->set_RotateAngle(90.0f); //对齐KFS侧面法向
    if(_tool_Abs(this->get_currentJointStatus().rotateJoint_angle_ - 90.0f) < 2.0f)
        return true;
    else
        return false;
}

bool ArmSetup::state_lowerStillness(int targetKFS)
{
    //判定到达目标的MF_road后，放低机械臂

    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);

    float targetLowerHeight = 0.0f; //KFS所在高度的云台下放高度，单位米
    if(MF_high[targetKFS - 1] == 0.2f)
        targetLowerHeight = 0.0f;
    else if(MF_high[targetKFS - 1] == 0.4f)
        targetLowerHeight = this->init_data_.safe_height; 
    else if(MF_high[targetKFS - 1] == 0.6f)
        targetLowerHeight = this->init_data_.max_launchHeight_;
    else
        targetLowerHeight = this->init_data_.max_launchHeight_;

    bool canLower = false;
    canLower = MF_AutoCtrler::isInTargetMap(auto_ctrl_.now_ChassisPosition,
                                            auto_ctrl_.pathInfo.MFroad[auto_ctrl_.now_targetIndex],
                                            0.45f); //进入目标KFS所在的MFroad中心且距离小于0.45m就放低
    if(canLower)
    {
        this->set_LaunchHeight(targetLowerHeight); //放低到目标高度
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
            this->init_data_.max_stretchLength_) < 0.01f)//伸展完成判定
    {
        if(!auto_ctrl_.flag.isExtReach)
        {
            auto_ctrl_.flag.reach_finishTimeStore = TimeStamp::getInstance().getSeconds(); //记录首次到达目标位置的时间戳
            auto_ctrl_.flag.isExtReach = true;
        }
    }
    
    const float now_s = TimeStamp::getInstance().getSeconds();
    if(auto_ctrl_.flag.isExtReach && (now_s - auto_ctrl_.flag.reach_finishTimeStore) >= 0.2f)
    {
        this->set_StretchLength(0.0f); //停留0.15s后缩回
        return true;
    }

    return false;

}

bool ArmSetup::state_launchStillness(int targetKFS)
{
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    float canMoveHeight = 0.0f;//云台升到此高度即可移动
    if(MF_high[targetKFS - 1] == 0.2f)
        canMoveHeight = this->init_data_.max_launchHeight_;
    else if(MF_high[targetKFS - 1] == 0.4f)
        canMoveHeight = this->init_data_.max_launchHeight_; 
    else if(MF_high[targetKFS - 1] == 0.6f)
        canMoveHeight = this->init_data_.max_launchHeight_;
    else
        canMoveHeight = this->init_data_.max_launchHeight_;

    this->set_LaunchHeight(this->init_data_.max_launchHeight_); //升到最高点，准备移动

    if(this->get_currentJointStatus().launchJoint_Height_ > canMoveHeight - 0.02f)
    {
        auto_ctrl_.flag.canChassisStart = true; //机械臂已经升到可以移动的高度了
        return true;
    }
    else
        return false;
    // return true;
}

bool ArmSetup::state_backStillness(int targetKFS)
{
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    float targetBackAngle = 0.0f; //根据车移动方向来，机械臂朝向底盘移动反方向

    const int8_t idx_mfroad = auto_ctrl_.pathInfo.Index_MFroad[auto_ctrl_.now_targetIndex];
    if(idx_mfroad < 0 || idx_mfroad >= 11)
        return false;

    float chassisDir = MF_AutoCtrler::chassisMoveDir(auto_ctrl_.pathInfo.mustPastMap[idx_mfroad]
                                        , auto_ctrl_.pathInfo.mustPastMap[idx_mfroad + 1]);
    int8_t c = 0,r = 0;
    MF_AutoCtrler::Map_ToCR(auto_ctrl_.pathInfo.MFroad[auto_ctrl_.now_targetIndex], c, r);

    if(_tool_Abs(chassisDir - 0.0f) < 5.0f) //底盘向右走
    {
        if(auto_ctrl_.kfs_num == ONLY_ONE 
                || (auto_ctrl_.kfs_num == TWO && auto_ctrl_.now_targetIndex == 1)) 
        {
            if(r == 1)//下侧
                targetBackAngle = 180.0f;
            
            else if(r == 6)//上侧
                targetBackAngle = 0.0f;
        }
        else
        {
            if(r == 1)//下侧
                targetBackAngle = 200.0f;
            
            else if(r == 6)//上侧
                targetBackAngle = 340.0f;
        }

    }
    else if(_tool_Abs(chassisDir - 180.0f) < 5.0f)//底盘向左
    {
        if(auto_ctrl_.kfs_num == ONLY_ONE 
            || (auto_ctrl_.kfs_num == TWO && auto_ctrl_.now_targetIndex == 1)) 
        {
             if(r == 1) // 下侧
                targetBackAngle = 0.0f;
            else if(r == 6) // 上侧
                targetBackAngle = 180.0f;
        }
        else
        {
            if (r == 1) // 下侧
                targetBackAngle = 340.0f;
            else if (r == 6) // 上侧
                targetBackAngle = 200.0f;
        }
    }
    else if(_tool_Abs(chassisDir - 90.0f) < 5.0f)//底盘向前
    {
        if(auto_ctrl_.kfs_num == ONLY_ONE 
            || (auto_ctrl_.kfs_num == TWO && auto_ctrl_.now_targetIndex == 1))
        {
             if(c == 1) // 左侧
                targetBackAngle = 0.0f;
            else if(c == 6) // 右侧
                targetBackAngle = 180.0f;
        }
        else
        {
            if (c == 1) // 左侧
                targetBackAngle = 340.0f;
            else if (c == 6) // 右侧
                targetBackAngle = 200.0f;
        }

    }
    else if(_tool_Abs(chassisDir - 270.0f) < 5.0f)//底盘向后
    {
        if(auto_ctrl_.kfs_num == ONLY_ONE
            || (auto_ctrl_.kfs_num == TWO && auto_ctrl_.now_targetIndex == 1))
        {
             if(c == 1) // 左侧
                targetBackAngle = 180.0f;
            else if(c == 6) // 右侧
                targetBackAngle = 0.0f;
        }
        else
        {
             if (c == 1) // 左侧
                targetBackAngle = 200.0f;
            else if (c == 6) // 右侧
                targetBackAngle = 340.0f;
        }
        
    }

    this->set_RotateAngle(targetBackAngle); //旋转到目标位置

    if(_tool_Abs(this->get_currentJointStatus().rotateJoint_angle_ - targetBackAngle) < 5.0f)
    {
        return true;
    }
    else
        return false;
}



/*=================================================================*/

/**
 * @brief 寻停止
 */
void ArmSetup::stop()
{
    this->set_controlMode(CURRENT_CONTROL_MODE);
    this->motor_launch_->setTargetCurrent(0.0f);
    this->motor_stretch_->setTargetCurrent(0.0f);
    this->motor_rotate_->setTargetCurrent(0.0f);
    this->motor_pitch_->setTargetCurrent(0.0f);
    this->setSuckerStatus(Sucker_Status_E::STOP);
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
    this->motor_stretch_->setTargetCurrent(-700.0f); // 给予一个小电流顶住限位
    this->motor_pitch_->setTargetCurrent(-1000.0f); // 给予一个小电流顶住限位
    //this->motor_rotate_->setTargetCurrent(1000.0f);
    if(this->now_time_s_ - arm_ctrlStatus.calibrate_startTime > 1.5f)
    {
        //relocate
        this->motor_stretch_->relocate_totalAngle(0.0f);
        this->motor_pitch_->relocate_totalAngle(179.9f); // 使用179.9f避免180度浮点临界值导致归一化为-180度
        this->motor_rotate_->relocate_totalAngle(this->rotateAngle_to_MotorTotalAngle(179.9f));
        this->motor_launch_->relocate_totalAngle(0.0f);

        //set current to 0
        this->motor_stretch_->setTargetCurrent(0.0f);
        this->motor_pitch_->setTargetCurrent(0.0f);
        this->motor_rotate_->setTargetCurrent(0.0f);
        // this->motor_launch_->setTargetCurrent(0.0f);

        arm_ctrlStatus.is_calibrating = true;
    }
}

/**
 * @brief 寻空闲
 */
void ArmSetup::idle()
{
    // 空闲控制函数，若上一时刻非此模式，则记忆上一时刻位置，并维持不变
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);

    this->setRotateStrategy(ROTATE_PATH_SHORTEST);

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

    // this->setSuckerStatus(Sucker_Status_E::STOP); // 保持上一刻状态，不强制关闭
}
// /*====================== 测试调试部分 ======================*/

// float test_rotate_angle = 0.2f;

// float test_launch_height = 0.01f;
// volatile float launch_see = 0.0f;
/**
 * @brief 寻调试
 */
void ArmSetup::debug()
{
    return; 
}

Arm_InitData_S arm_initData = {
   .max_launchHeight_ = 0.29f,
   .max_stretchLength_ = 0.105f,
   .arm_length_ = 0.6f,
   .end_link_length_ = 0.08f,

   .stretch_Ratio_ = 0.08417f,
   .launch_Ratio_ = 0.07221f,
//    .rotate_gearRatio_ = 144.878f,  //旧的
   .rotate_gearRatio_ = 145.755789f,
   .pitch_gearRatio_ = 360.0f,

   .min_rotate_angle_ = 0.0f,
   .max_rotate_angle_ = 359.99999f,
    .safe_height = 0.14f,
   .Sucker_GPIO_Port = SUCKER_error_GPIO_Port,
    .Sucker_GPIO_Pin =  SUCKER_error_Pin,

    
};