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
                // ifFirst = false;
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
                if(arm_ctrlStatus.auto_start == 1)
                    autoControl();
                else 
                    idle();
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
                 if(norm_angle < 30.0f || norm_angle > 135.0f)
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

#if ARM_AUTOMOVE 
    switch(auto_ctrl_.kfs_num)
    {
        case ONLY_ONE:
        {
            //单个KFS拾取流程
            auto_onlyOne();
            break;
        }
        
        case TWO: //没做
        {
            //两个KFS拾取流程
            auto_two();
            break;
        }
    }

#else //行进间拾取


    //行进间拾取
    switch(auto_ctrl_.kfs_num)
    {
        case ONLY_ONE:
        {
            auto_stillnessOne();
            break;
        }
        
        case TWO: //没做
        {
            idle();
            break;
        }
    }
#endif

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
                    auto_ctrl_.flag.isrecalcPath = true;
                }

                auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_TO_WAIT;
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
                auto_ctrl_.now_state = ARM_AUTO_STILLNESS_E::STATE_LAUNCH;
            }
            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_LOWER:
        {

            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_EXT:
        {

            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_LAUNCH:
        {

            break;
        }

        case ARM_AUTO_STILLNESS_E::STATE_BACK:
        {

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

    float kfs_height = MF_high[targetKFS -1]; //获取目标KFS高度

    float target_height = 0.0f;
    if(kfs_height - 0.2f < init_data_.safe_height)
        target_height = this->init_data_.safe_height; //安全高度
    else if(kfs_height == 0.4f)
        target_height = this->init_data_.safe_height; //目标高度-吸盘高度(0.2m)
    else if(kfs_height == 0.6f)
        target_height = this->init_data_.max_launchHeight_; //目标高度-吸盘高度(0.2m)
    else
        target_height = this->init_data_.max_launchHeight_; //默认安全高度
    
    //防止撞到60cm高的梅花桩
    if(targetKFS == 9 || targetKFS == 3)
    {
        if(auto_ctrl_.KFS_Movedirection[auto_ctrl_.now_targetIndex]
            == MF_AutoCtrler::Direction_E::Positive_Y && targetKFS == 9)
        {
            target_height = this->init_data_.max_launchHeight_;
        }
        else if(auto_ctrl_.KFS_Movedirection[auto_ctrl_.now_targetIndex]
            == MF_AutoCtrler::Direction_E::Negative_Y && targetKFS == 3)
        {
            target_height = this->init_data_.max_launchHeight_;
        }
    }


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
    //待底盘行驶至B1位置时候，云台开始旋转对齐KFS侧面法向
    bool can_rotate = false;

    int target_map = 0;
    if(auto_ctrl_.now_targetIndex == 0)
        target_map = auto_ctrl_.path.bestB1;
    else if(auto_ctrl_.now_targetIndex == 1)
        target_map = auto_ctrl_.path.bestB2;
    else
        target_map = 0;

    can_rotate = MF_AutoCtrler::isInTargetMap(auto_ctrl_.now_ChassisPosition, 
            target_map, 0.3f);

    if(!can_rotate)
        return false; //未到达目标位置，继续等待
    else
    {
        this->set_RotateAngle(90.0f); //对齐KFS侧面法向
        return true;
    }
    
}

bool ArmSetup::state_lowerStillness(int targetKFS)
{
    //判定越过PA点之后，放低机械臂

    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);

    float targetLowerHeight = 0.0f; //KFS所在高度的云台下放高度，单位米
    if(MF_high[targetKFS - 1] == 0.2f)
        targetLowerHeight = 0.0f;
    else if(MF_high[targetKFS - 1] == 0.4f)
        targetLowerHeight = this->init_data_.safe_height; 
    else if(MF_high[targetKFS - 1] == 0.6f)
        targetLowerHeight = this->init_data_.max_launchHeight_;
    else
        targetLowerHeight = this->init_data_.safe_height;

    int target_map = 0;
    if(auto_ctrl_.now_targetIndex == 0)
        target_map = auto_ctrl_.path.bestBMF1;
    else if(auto_ctrl_.now_targetIndex == 1)
        target_map = auto_ctrl_.path.bestBMF2;
    else
        target_map = 0;


    if(MF_AutoCtrler::isInTargetMap(auto_ctrl_.now_ChassisPosition,
            target_map, ))

    switch(auto_ctrl_.KFS_Movedirection[auto_ctrl_.now_targetIndex])
    {
        case MF_AutoCtrler::Positive_X:
        {

            break;
        }

        case MF_AutoCtrler::Negative_X:
        {
            break;
        }

        case MF_AutoCtrler::Positive_Y:
        {
            
            break;
        }

        case MF_AutoCtrler::Negative_Y:
        {
            break;
        }
    }
}

/**
 * @brief 自动高度
 */
void ArmSetup::state_toTargetHight(int targetKFS)
{
#if ARM_AUTOMOVE
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    /**
     * 1. 根据KFS编号，查询对应高度
     * 2. 升降到对应高度，吸盘pitch90度
     * 3. 锁住云台，等待移动到目标前一桩，云台开始预判旋转时机
     */

    //20cm台阶 升降0m, 40cm台阶升降 0.2m, 60cm台阶升降0.4m
    float kfs_height = MF_high[targetKFS -1]; //获取目标KFS高度

    if(kfs_height - 0.2f < auto_ctrl_.flag.safe_height)
        this->set_LaunchHeight(auto_ctrl_.flag.safe_height); //安全高度
    else if(kfs_height == 0.4f)
        this->set_LaunchHeight(auto_ctrl_.flag.safe_height); //目标高度-吸盘高度(0.2m)
    else if(kfs_height == 0.6f)
        this->set_LaunchHeight(init_data_.max_launchHeight_); //目标高度-吸盘高度(0.2m)

    
    this->set_PitchAngle(90.0f); //吸盘pitch90度
#endif
}


bool ArmSetup::check_Arm_collision(float px, float py, 
                            float pivot_x, float pivot_y, 
                            float arm_world_angle_deg, float L_arm, 
                            float W_arm)
{   //云台旋转碰撞判定
    float angle_rad = arm_world_angle_deg * (PI / 180.0f);

    float c = cosf(angle_rad);
    float s = sinf(angle_rad);

    Point2D d = {
        px - pivot_x,
        py - pivot_y,
        0.0f
    };

    // Local X: 沿机械臂轴向 (点乘方向向量 (c, s))
    // Local Y: 垂直机械臂轴向 (点乘法向量 (-s, c))
    Point2D local = {
         d.x * c + d.y * s, // Local X
        -d.x * s + d.y * c, // Local Y
         0.0f
    };
    
    // 矩形碰撞判断: x in [0, L], y in [-W/2, W/2]
    if(local.x >= 0.0f && local.x <= (L_arm ) && _tool_Abs(local.y) <= ((W_arm + 0.02f) / 2.0f))
        return true; //碰撞
    else
        return false; //未碰撞
}

/**
 * @brief 寻自动对齐
 */
Point2D toosee = {0};
void ArmSetup::state_signAlign(int targetKFS, bool &align_done)
{
#if ARM_AUTOMOVE
    float arm_width = 0.12f; //机械臂宽度，单位米
    float gimbal_calcHz = 100.0f; //云台预判计算频率
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    /**
     * 云台旋转时机预判，以及执行
     * 
     * @details 依旧屎山堆积
     */
    MF_AutoCtrler::Direction_E move_direction;
    Point2D target_pos = {0.0f, 0.0f ,0.0f};
    if(targetKFS == auto_ctrl_.targetKFS[0])
    {
        move_direction = auto_ctrl_.KFS_Movedirection[0];
        target_pos = auto_ctrl_.targetKFS_pos[0];
    }
    else if(targetKFS == auto_ctrl_.targetKFS[1])
    {
        move_direction = auto_ctrl_.KFS_Movedirection[1];
        target_pos = auto_ctrl_.targetKFS_pos[1];
    }
    else
        return;

    
    if(targetKFS == 1 || targetKFS ==2 || targetKFS == 3)
    {
        if(auto_ctrl_.now_armPosition.y < auto_ctrl_.pathPos.bestB1.y - 0.3f)
            return; //未到达目标位置，直接返回
    }
    else if(targetKFS == 10 || targetKFS ==11 || targetKFS ==12)
    {
        if(auto_ctrl_.now_armPosition.y < auto_ctrl_.pathPos.bestB1.y - 1.0f)
            return; //未到达目标位置，直接返回
    }

    //开始预判计算部分
    switch(move_direction) //还未到目标位置，不进入计算
    {
        case MF_AutoCtrler::Positive_X:
        {
            if(auto_ctrl_.now_armPosition.x < auto_ctrl_.pathPos.bestB1.x - 0.1f)
                return; //未到达目标位置，直接返回
            break;
        }
        case MF_AutoCtrler::Negative_X:
        {
            if(auto_ctrl_.now_armPosition.x > auto_ctrl_.pathPos.bestB1.x + 0.1f)
                return; //未到达目标位置，直接返回
            break;
        }

        case MF_AutoCtrler::Positive_Y:
        {
            if(auto_ctrl_.now_armPosition.y < auto_ctrl_.pathPos.bestB1.y - 0.1f)
                return; //未到达目标位置，直接返回
            break;
        }
        case MF_AutoCtrler::Negative_Y:
        {
            if(auto_ctrl_.now_armPosition.y > auto_ctrl_.pathPos.bestB1.y + 0.1f)
                return; //未到达目标位置，直接返回
            break;
        }
        default:
            break;
    }

    //达到目标位置，开始计算, 计算频率100Hz
    auto_ctrl_.gimbal_calcCount++;
    if(auto_ctrl_.gimbal_calcCount < 1000.0f/ 
                static_cast<float>(gimbal_calcHz))
        return; //未到计算时间，直接返回

    int index = 0;
    if(targetKFS == auto_ctrl_.targetKFS[0])
        index = 0;
    else if(targetKFS == auto_ctrl_.targetKFS[1])
        index = 1;

    //开始计算
    auto_ctrl_.gimbal_calcCount = 0;
    get_GimbalMF_PAPB(index, auto_ctrl_.PointPAB[index].PA, 
            auto_ctrl_.PointPAB[index].PB);


    Point2D PA = auto_ctrl_.PointPAB[index].PA;
    Point2D PB = auto_ctrl_.PointPAB[index].PB;

    float vx = 0.0f, vy = 0.0f;
    switch(move_direction)
    {
        case MF_AutoCtrler::Positive_X:
        {
            vx = auto_ctrl_.now_chassis_speed.x;
            vy = 0.0f;
            break;
        }
        case MF_AutoCtrler::Negative_X:
        {
            vx = -auto_ctrl_.now_chassis_speed.x;
            vy = 0.0f;
            break;
        }

        case MF_AutoCtrler::Positive_Y:
        {
            vx = 0.0f;
            vy = auto_ctrl_.now_chassis_speed.y;
            break;
        }
        case MF_AutoCtrler::Negative_Y:
        {
            vx = 0.0f;
            vy = -auto_ctrl_.now_chassis_speed.y;
            break;
        }
        default:
            break;
    }

        float current_deg = this->get_currentJointStatus().rotateJoint_angle_;
        float target_deg = 90.0f;

        // [修复] 先将当前连续角归一化到 0-360，再计算差值
        float current_mod = fmodf(current_deg, 360.0f);
        if(current_mod < 0.0f) current_mod += 360.0f;

        // 计算最短路径误差 (-180 ~ 180)
        float diff = target_deg - current_mod;
        
        // 归一化 diff 到 [-180, 180]
        if(diff > 180.0f)       diff -= 360.0f;
        else if(diff < -180.0f) diff += 360.0f;

        //步进预测循环
        float T_rot = _tool_Abs(diff) * (PI / 180.0f) / 
                    (auto_ctrl_.time_set.gimbal_max_rad * auto_ctrl_.time_set.rotateSpeedRate_); //云台旋转所需时间(s)

        bool safe = true;

    //碰撞检测 Lambda函数
    // 对齐法平面
        
    for(float t = 0.0f; t <= T_rot; t+= 0.05f)
    {
        Point2D pivot{ //pivot(t)
             .x = auto_ctrl_.now_armPosition.x + vx * t,
             .y = auto_ctrl_.now_armPosition.y + vy * t,
             .theta = 0.0f
        };

        float step_deg = 0.0f;

        if(T_rot > 0.001f) // 防止除零
            step_deg = (diff / T_rot) * t;
        else
            step_deg = diff;


        // if(diff > 0 )
        //     step_deg = 1.0f * (auto_ctrl_.time_set.gimbal_max_rad 
        //             * 0.3f * 180.0f / PI) * t; //每步旋转
        // else
        //     step_deg = -1.0f * (auto_ctrl_.time_set.gimbal_max_rad 
        //             * 0.3f * 180.0f / PI) * t; //每步旋转

        // theta(t)
        if(_tool_Abs(step_deg) > _tool_Abs(diff))
            step_deg = diff; //最后一步直接到达目标角度



        float gimbal_angle_t  = current_deg + step_deg;
        //phi(t) = yaw + theta(t)
        float world_angle_t = MF_AutoCtrler::Get_ArmWorldAngle
            (auto_ctrl_.now_ChassisPosition.theta, gimbal_angle_t);

        //碰撞检测
        //Edge_L Edge_R 与PA PB不重合
        if(check_Arm_collision(PA.x, PA.y, 
                            pivot.x, pivot.y, 
                            world_angle_t, 
                            arm_width, arm_width)
            ||
            check_Arm_collision(PB.x, PB.y, 
                            pivot.x, pivot.y, 
                            world_angle_t, 
                            arm_width, arm_width))
        {
            safe = false;
            break;
        }
    }

    //选择旋转策略
    if(_tool_Abs(diff) < 2.0f)
    {
        auto_ctrl_.current_strategy = ROTATE_PATH_SHORTEST;
    }

    else if(diff  > 0.0f)
        auto_ctrl_.current_strategy = ROTATE_PATH_POSITIVE;
    
    else if (diff < 0.0f)
        /* code */
        auto_ctrl_.current_strategy = ROTATE_PATH_NEGATIVE;
    
    else
        auto_ctrl_.current_strategy = ROTATE_PATH_SHORTEST;
    
    // [单圈模式] 记录Align阶段的旋转方向
    if(!rotate_multiTurn_)
    {
        // 仅在尚未记录明确策略时记录，避免后续微调或超调导致策略被覆盖为SHORTEST或反向
        if(recorded_align_strategy_ == ROTATE_PATH_SHORTEST)
        {
            if(auto_ctrl_.current_strategy == ROTATE_PATH_POSITIVE)
                recorded_align_strategy_ = ROTATE_PATH_POSITIVE;
            else if(auto_ctrl_.current_strategy == ROTATE_PATH_NEGATIVE)
                recorded_align_strategy_ = ROTATE_PATH_NEGATIVE;
        }
    }

    this->setRotateStrategy(auto_ctrl_.current_strategy);

    //执行
    if(safe)
    {
        // [Fix] 使用策略计算连续目标角度，防止多圈旋转问题
        float current_cont = this->get_currentJointStatus().rotateJoint_angle_;
        float target_cont = this->calc_rotate_targetByStrategy(current_cont, 90.0f);
        this->set_RotateAngle(target_cont); //对齐目标角度

        // [新增] 当云台不在禁区时候，再降低云台高度到零点

        float norm_angle = fmodf(current_deg, 360.0f);
        if(norm_angle < 0.0f) norm_angle += 360.0f;

            float kfs_height = MF_high[targetKFS -1];
        if(_tool_Abs(this->get_currentJointStatus().rotateJoint_angle_-90.0f) < 10.0f )
        {
           
           switch(move_direction)
           {
               case MF_AutoCtrler::Positive_X:
               {
                   if(auto_ctrl_.now_armPosition.x > PA.x)
                       auto_ctrl_.flag.issafetoLower = true;
                   else
                       auto_ctrl_.flag.issafetoLower = false;
                   break;
               }

               case MF_AutoCtrler::Negative_X:
               {
                   if(auto_ctrl_.now_armPosition.x < PA.x)
                       auto_ctrl_.flag.issafetoLower = true;
                   else
                       auto_ctrl_.flag.issafetoLower = false;
                   break;
               }

               case MF_AutoCtrler::Positive_Y:
               {
                   if (auto_ctrl_.now_armPosition.y > PA.y)
                       auto_ctrl_.flag.issafetoLower = true;
                   else
                       auto_ctrl_.flag.issafetoLower = false;
                   
                   break;
               }

               case MF_AutoCtrler::Negative_Y:
               {
                   if(auto_ctrl_.now_armPosition.y < PA.y)
                       auto_ctrl_.flag.issafetoLower = true;
                   else
                       auto_ctrl_.flag.issafetoLower = false;
                   break;
               }

           }

           if(auto_ctrl_.flag.issafetoLower)
           {
                if(kfs_height == 0.2f)
                    this->set_LaunchHeight(0.0f); //降到最低点
                else if(kfs_height == 0.4f)
                    this->set_LaunchHeight(auto_ctrl_.flag.safe_height); //降到safe
                else if(kfs_height == 0.6f)
                    this->set_LaunchHeight(init_data_.max_launchHeight_); //降到最高点
           }
        }

        if(_tool_Abs(diff) < 2.0f && auto_ctrl_.flag.issafetoLower)
        {
            //到达目标角度后，打开吸盘
            this->setSuckerStatus(Sucker_Status_E::SUCK);
            align_done = true; //对齐完成

            this->setRotateStrategy(ROTATE_PATH_SHORTEST);
        }

    }
    else
    {
         this->set_RotateAngle(current_deg); //保持不变
        return; //对齐未完成
    }
#endif
}

Point2D pos_tar_kfs = {0.0f, 0.0f, 0.0f};
Point2D pos_start_kfs = {0.0f, 0.0f, 0.0f};

/**
 * @brief 伸展到目标KFS位置 条件预判
 */

/**
 * @brief 寻自动伸展
 */
float t_needread = 0.0f;
bool ArmSetup::state_aimExt(int targetKFS)
{
#if ARM_AUTOMOVE
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    /**
     * 设置 伸展所需要的 时间 t_need 以及 底盘移动到目标位置的时间 t_tan
     * 判定是否可以伸展
     * 可以，则伸展 this->set_StretchLength(max_length)
     * 
     * this->get_currentJointStatus().stretchJoint_Length_ 获取当前伸展长度
     * 
     * 判断是否伸展完毕， 
     * 是， 则停留0.3s，后缩回 this->set_StretchLength(0.0f)
     */
		bool safe = false;
		float current_armLength;
		float t_need = 0.0f;
		float t_stretch = auto_ctrl_.time_set.stretch_time_s;
		//RawPos nowRaw=Position::GetInstance(&huart1)->getRawPosData();

        Point2D target_pos = {0.0f, 0.0f ,0.0f};

        MF_AutoCtrler::Direction_E move_direction;

        if(targetKFS == auto_ctrl_.targetKFS[0])
        {
            move_direction = auto_ctrl_.KFS_Movedirection[0];
            target_pos = auto_ctrl_.targetKFS_pos[0];
        }
        else if(targetKFS == auto_ctrl_.targetKFS[1])
        {
            move_direction = auto_ctrl_.KFS_Movedirection[1];
            target_pos = auto_ctrl_.targetKFS_pos[1];
        }
        else
            return false;
		
        //计算t_need
        if(!auto_ctrl_.flag.ext_started)
        {
            switch(move_direction)
            {
                case MF_AutoCtrler::Positive_X:
                {
//                    if(_tool_Abs(auto_ctrl_.now_chassis_speed.x) < 0.1f)
//                        return false; //速度为0，无法伸展
                    t_need = _tool_Abs((target_pos.x - auto_ctrl_.now_armPosition.x) 
                            / auto_ctrl_.now_chassis_speed.x);
                    break;
                }
                case MF_AutoCtrler::Negative_X:
                {
//                    if(_tool_Abs(auto_ctrl_.now_chassis_speed.x) < 0.1f)
//                        return false; //速度为0，无法伸展

                    t_need = _tool_Abs((auto_ctrl_.now_armPosition.x - target_pos.x) 
                            / auto_ctrl_.now_chassis_speed.x);
                    break;
                }

                case MF_AutoCtrler::Positive_Y:
                {
                    if(_tool_Abs(auto_ctrl_.now_chassis_speed.y) < 0.1f)
                        return false; //速度为0，无法伸展

                    t_need = _tool_Abs((target_pos.y - auto_ctrl_.now_armPosition.y) 
                            / auto_ctrl_.now_chassis_speed.y);
                    break;
                }
                case MF_AutoCtrler::Negative_Y:
                {
                    if(_tool_Abs(auto_ctrl_.now_chassis_speed.y) < 0.1f)
                        return false; //速度为0，无法伸展
                    t_needread = t_need;
                    t_need = _tool_Abs((auto_ctrl_.now_armPosition.y - target_pos.y) 
                            / auto_ctrl_.now_chassis_speed.y);
                    break;
                }
                default:
                    break;
            }
        }
        //或许delta_t < 0.02s会错过伸展窗口(计算频率)
        //给足提前量容忍，或许会更好，避免过严格的等式触发

        const float delta_t = 0.04f; //提前量容忍

        if(!auto_ctrl_.flag.ext_started)
        {
            if(_tool_Abs(t_need - t_stretch) < delta_t )//||
                        //(t_need < t_stretch - delta_t))//防止越窗未触发，暂时不启用
                // safe = true;
            {
                auto_ctrl_.flag.ext_started = true;
                pos_start_kfs = auto_ctrl_.now_armPosition; //记录伸展开始位置
            }
            
        }

        if(auto_ctrl_.flag.ext_started)
            this->set_StretchLength(arm_initData.max_stretchLength_);
        

		current_armLength = get_currentJointStatus().stretchJoint_Length_;

		if(_tool_Abs(current_armLength-arm_initData.max_stretchLength_) < 0.006f && !auto_ctrl_.flag.is_reachingTarget)
		{
			auto_ctrl_.flag.is_reachingTarget = true;
            pos_tar_kfs = auto_ctrl_.now_armPosition; //记录伸展到达位置
			auto_ctrl_.flag.reach_finishTime = TimeStamp::getInstance().getSeconds();
		}
		if(auto_ctrl_.flag.is_reachingTarget && (now_time_s_-auto_ctrl_.flag.reach_finishTime) >= 0.15f)
		{
			this->set_StretchLength(0.0f);
           // auto_ctrl_.flag.ext_started = false; //重置伸展开始标志

            if(this->get_currentJointStatus().stretchJoint_Length_ < 0.005f)
            { 
			    return true;
            }
            return false;
		}

        else
			return false;
#endif
}

/**
 * @brief 寻自动搬运
 */
// float diff_read=- 0.0f;
void ArmSetup::state_carrying(int targetKFS ,bool &carrying_done)
{
#if ARM_AUTOMOVE
    float arm_width = 0.12f; //机械臂宽度，单位米
    /**
     * 
     *  缩回后， 开始预判能否转回来；
     *  并将目标KFS放到存储机构位置
     * 
     * 具体流程：1. 判断可执行旋转，云台执行旋转
     *          2. 在旋转开始时候，判定当前云台高度是否比存储时候所需云台高度要高，否则，抬高
     *             是，则维持
     *          3. 旋转到目标位置后，降低云台放置KFS到存储机构位置，0.2s后吸盘关闭
     *          4. 抬高云台到安全高度
     * 
     *          将350mm的长度纳入机械臂长度考虑
     */
    float store_height = 0.14f;
    MF_AutoCtrler::Direction_E move_direction;
    if(targetKFS == auto_ctrl_.targetKFS[0])
    {
        move_direction = auto_ctrl_.KFS_Movedirection[0];
    }
    else if(targetKFS == auto_ctrl_.targetKFS[1])
    {
        move_direction = auto_ctrl_.KFS_Movedirection[1];
    }
    else
        return;

    int index = 0;
    if(targetKFS == auto_ctrl_.targetKFS[0])
        index = 0;
    else if(targetKFS == auto_ctrl_.targetKFS[1])
        index = 1;

    //获取障碍点 PA PB
    get_GimbalMF_PAPB(targetKFS, auto_ctrl_.PointPAB[index].PA, 
            auto_ctrl_.PointPAB[index].PB);
    Point2D PA = auto_ctrl_.PointPAB[index].PA;
    Point2D PB = auto_ctrl_.PointPAB[index].PB;

    // [新增] 判定当前云台高度是否比存储时候所需云台高度要高，否则，抬高
    if(auto_ctrl_.store[index].is_toPlace == false)
    {
         if(this->get_currentJointStatus().launchJoint_Height_ < auto_ctrl_.flag.safe_height - 0.01f)
        {
            this->set_LaunchHeight(auto_ctrl_.flag.safe_height);
            return;
        }
    }

    float vx = 0.0f, vy = 0.0f;

    /**
     * @details 我真受不了先前埋的这坨屎了
     */
    switch(move_direction)
    {
        case MF_AutoCtrler::Positive_X:
        {
            vx = auto_ctrl_.now_chassis_speed.x;
            vy = 0.0f;
            break;
        }
        case MF_AutoCtrler::Negative_X:
        {
            vx = -auto_ctrl_.now_chassis_speed.x;
            vy = 0.0f;
            break;
        }

        case MF_AutoCtrler::Positive_Y:
        {
            vx = 0.0f;
            vy = auto_ctrl_.now_chassis_speed.y;
            break;
        }
        case MF_AutoCtrler::Negative_Y:
        {
            vx = 0.0f;
            vy = -auto_ctrl_.now_chassis_speed.y;
            break;
        }
        default:
            break;
    }

    float current_deg = this->get_currentJointStatus().rotateJoint_angle_;
    float target_deg = 270.0f;

    // [单圈模式] 设定目标角度和继承策略
    if(!rotate_multiTurn_)
    {
        target_deg = 270.0f;
        auto_ctrl_.current_strategy = recorded_align_strategy_;
        recorded_carrying_strategy_ = recorded_align_strategy_;
        s_has_recorded_strategy = true; // [新增] 标记已记录
    }
    else
    {
        target_deg = 270.0f; //存储机构位置角度为270度
        auto_ctrl_.current_strategy = ROTATE_PATH_SHORTEST;
    }

    //Rotate_Strategy_E strategy = auto_ctrl_.current_strategy;

    //计算符合 的 diff
    float diff = 0.0f;
    float current_mod = fmodf(current_deg, 360.0f);
    if(current_mod <0)
        current_mod += 360.0f;
    
    float target_mod = fmodf(target_deg, 360.0f);
    if(target_mod < 0.0f) target_mod += 360.0f;

    float raw_diff = target_mod - current_mod;

    switch(auto_ctrl_.current_strategy)
    {
        case ROTATE_PATH_POSITIVE:
        {
            //必须正转
            if(raw_diff <= 0.0f)
            {
                // [Fix] 增加容差判断，防止微小超调导致判定为旋转一圈
                if(_tool_Abs(raw_diff) < 2.0f)
                    diff = raw_diff;
                else
                    diff = raw_diff + 360.0f;
            }
            else
                diff = raw_diff;
            break;
        }

        case ROTATE_PATH_NEGATIVE:
        {
            //必须负转
            if(raw_diff >= 0.0f)
            {
                // [Fix] 增加容差判断
                if(_tool_Abs(raw_diff) < 2.0f)
                    diff = raw_diff;
                else
                    diff = raw_diff - 360.0f;
            }
            else
                diff = raw_diff;
            break;
        }

        case ROTATE_PATH_SHORTEST:
        {
            diff = raw_diff;
            if(diff > 180.0f) diff -= 360.0f; 
            else if(diff < -180.0f) diff += 360.0f;
            break;
        }

        default:
            break;
    }

    //碰撞检测
    float kfs_size = 0.35f; //考虑350mm的KFS长度

    //Lenggth
    float check_L = init_data_.arm_length_ + kfs_size + init_data_.end_link_length_;

    //width
    float check_W = (arm_width > kfs_size) ? arm_width : kfs_size;

    bool safe = true;

    //time calc
    float T_rot = _tool_Abs(diff) * (PI / 180.0f) / 
                (auto_ctrl_.time_set.gimbal_max_rad * auto_ctrl_.time_set.rotateSpeedRate_); //云台旋转所需时间(s)

    // diff_read = diff;

    for(float t = 0.0f; t <= T_rot; t+= 0.05f)
    {
        Point2D pivot{
            .x = auto_ctrl_.now_armPosition.x + vx * t,
            .y = auto_ctrl_.now_armPosition.y + vy * t,
            .theta = 0.0f
        };

        float step_deg = 0.0f;
        if(diff > 0.0f )
            step_deg = 1.0f * (auto_ctrl_.time_set.gimbal_max_rad 
                    * auto_ctrl_.time_set.rotateSpeedRate_ * 180.0f / PI) * t; //每步旋转

        else
            step_deg = -1.0f * (auto_ctrl_.time_set.gimbal_max_rad 
                    * auto_ctrl_.time_set.rotateSpeedRate_ * 180.0f / PI) * t; //每步旋转
        //theta(t)
        if(_tool_Abs(step_deg) > _tool_Abs(diff))
            step_deg = diff; //最后一步直接到达目标角度

        // 旋转超过90度的时候，则视为安全——如果之后发现阈值不如，就进行调整
        if(_tool_Abs(step_deg) >= 90.0f)
            break;

        float gimbal_angle_t  = current_deg + step_deg;

        //phi(t) = yaw + theta(t)
        float world_angle_t = 
            MF_AutoCtrler::Get_ArmWorldAngle(
                auto_ctrl_.now_ChassisPosition.theta, 
                gimbal_angle_t);

        //碰撞检测
        // 当Edge_L Edge_R 与PA PB不重合
        if(check_Arm_collision(PA.x, PA.y, 
                            pivot.x, pivot.y, 
                            world_angle_t, 
                            check_L, check_W)
            ||
            check_Arm_collision(PB.x, PB.y, 
                            pivot.x, pivot.y, 
                            world_angle_t, 
                            check_L, check_W))
        {
            safe = false;
            break;
        }
    }

    if(auto_ctrl_.store[index].is_toPlace == false)
    {
        if(this->get_currentJointStatus().launchJoint_Height_
            < auto_ctrl_.flag.safe_height)
        {
            this->set_LaunchHeight(auto_ctrl_.flag.safe_height);
        }
        else
        {
            this->set_LaunchHeight(this->get_currentJointStatus().launchJoint_Height_);
        }
    }
    //执行
    if(safe)
    {
        this->set_RotateAngle(target_deg);

        //旋转到目标位置后
        static float wait_startTime = 0.0f;

        if(_tool_Abs(diff) > 5.0f)
        {
            auto_ctrl_.store[index].is_toPlace = false;

        }
        else if(_tool_Abs(diff) <= 2.0f)
        {
             this->setRotateStrategy(ROTATE_PATH_SHORTEST);
            if(auto_ctrl_.store[index].is_toPlace == false)
            {
                //降低云台放置KFS到存储机构位置
                this->set_LaunchHeight(store_height);

                wait_startTime = this->now_time_s_;
                auto_ctrl_.store[index].is_toPlace = true;
            }

            else if(auto_ctrl_.store[index].is_toPlace == true)
            {
                this->set_LaunchHeight(store_height); //维持不变

                //0.2s后吸盘关闭
                if(this->now_time_s_ - wait_startTime > 0.2f)     
                    this->setSuckerStatus(Sucker_Status_E::STOP);
                

                if(this->now_time_s_ - wait_startTime > 1.0f)
                {
                    //抬高云台到安全高度
                    this->set_LaunchHeight(auto_ctrl_.flag.safe_height);        
                    carrying_done = true; //放置完成
                }
            }
        
        }
    }

    //不安全，保持不变
    else    
    {
        this->setRotateStrategy(ROTATE_PATH_SHORTEST);
         this->set_RotateAngle(current_deg); //保持不变
        
    }
#endif
}

/**
 * @brief 寻自动返回
 * @param 这个不是KFS的编号，而是索引，0为第一个KFS，1为第二个KFS，其他值视为无下一个KFS
 */
bool ArmSetup::state_return(int next_targetKFS)
{
#if ARM_AUTOMOVE
    // [Fix] 修正逻辑：若低于安全高度，必须先旋转到安全区域（30~135度），才能抬升
    
    float current_angle = this->get_currentJointStatus().rotateJoint_angle_;
    float norm_angle = fmodf(current_angle, 360.0f);
    if(norm_angle < 0.0f) norm_angle += 360.0f;

    bool in_safe_zone = (norm_angle >= 60.0f && norm_angle <= 135.0f);
    float target_h  = 0.0f;
    if(next_targetKFS == 0)
    {
        if(MF_high[auto_ctrl_.targetKFS[next_targetKFS] - 1] == 0.2f)
            target_h = auto_ctrl_.flag.safe_height; // 20cm台阶，抬升到安全高度
        else if(MF_high[auto_ctrl_.targetKFS[next_targetKFS] - 1] == 0.4f)
            target_h = auto_ctrl_.flag.safe_height; // 40cm台阶，抬升到安全高度
        else if(MF_high[auto_ctrl_.targetKFS[next_targetKFS] - 1] == 0.6f)
            target_h = init_data_.max_launchHeight_; // 60cm台阶，抬升到最高高度
        else
            target_h = init_data_.max_launchHeight_; // 默认最高高度
    }
    else
    // 确定目标高度: 统一为最高高度，确保安全
        target_h  = init_data_.max_launchHeight_; 

    // 高度控制逻辑
    if(this->get_currentJointStatus().launchJoint_Height_ < auto_ctrl_.flag.safe_height - 0.01f)
    {
        // 当前低于安全高度
        if(in_safe_zone)
        {
            // [Fix] 在安全角度区域，允许抬升到目标高度，但必须暂停旋转等待抬升完成
            this->set_LaunchHeight(target_h);
            
            // 暂停旋转，保持当前角度 (使用最短路径策略原地保持)
            this->setRotateStrategy(ROTATE_PATH_SHORTEST);
            this->set_RotateAngle(current_angle);
            return false; // 等待抬升
        }
        else
        {
            // [Fix] 不在安全区域，禁止抬升，保持低位，并强制旋转向安全区(90度)
            this->set_LaunchHeight(0.0f); 
            
            this->setRotateStrategy(ROTATE_PATH_SHORTEST);
            this->set_RotateAngle(90.0f);
            return false; // 等待进入安全区
        }
    }
    else
    {
        // 当前已高于安全高度，直接前往目标高度
        this->set_LaunchHeight(target_h);
    }

    /**
     * @brief 
     *  1. 传入的下一个点
     *      a. 有下一个KFS，判断下一段拾取路径的车头朝向
     *      b. 将云台旋转至其方向
     * 
     * 2. 无下一个点， 云台转向0度
     * 
     *  3. 云台旋转时候，只能在车身投影内进行旋转
     *     (即，角度变化只能是在180度~ 359.999f)
     * 
     * 4. 传入非0和1的数，就默认没有下一个KFS，直接转回0度
     */		
    float angel = 0.0f;
    bool has_next = (next_targetKFS == 0 || next_targetKFS == 1);

    if (has_next)
    {
        int TargetMap;
        int Target_KFS = auto_ctrl_.targetKFS[next_targetKFS];
        if (next_targetKFS == 0)
            TargetMap = auto_ctrl_.path.bestB1;
        if (next_targetKFS == 1)
            TargetMap = auto_ctrl_.path.bestB2;

        angel = MF_AutoCtrler::Get_ArmBaseTargetAngle(TargetMap, auto_ctrl_.KFS_Movedirection[next_targetKFS]);
    }
    else
    {
        // [Version 8.0] auto_onlyOne return logic
        if (auto_ctrl_.kfs_num == ONLY_ONE)
        {
            // 获取起始点的地图节点编号
            int TargetMap = auto_ctrl_.path.bestB1;

            // 获取起始点的朝向 (0度 或 180度)
            float base_angle = MF_AutoCtrler::Get_ArmBaseTargetAngle(TargetMap, 
                                    auto_ctrl_.KFS_Movedirection[0]);
                                
            // 选择与base_angle相反角度
            if(_tool_Abs(base_angle - 180.0f) < 0.1f)
                angel = 0.0f;
            else if(_tool_Abs(base_angle - 0.0f) < 0.1f)
                angel = 180.0f;
            else
                angel = 0.0f; // Fallback

        }
        else
        {
            angel = 0.0f; // 默认返回0度
        }
    }

    //    float current_angle = this->get_currentJointStatus().rotateJoint_angle_;
    float diff = angel - fmodf(current_angle, 360.0f);

    // 简单的归一化处理，确保 diff 在 -180 ~ 180
    if (diff > 180.0f)
        diff -= 360.0f;
    else if (diff < -180.0f)
        diff += 360.0f;

    if (_tool_Abs(diff) < 2.0f)
    {
        auto_ctrl_.current_strategy = ROTATE_PATH_SHORTEST; // 误差极小时，锁定最短路径
    }
    else if (has_next)
    {
        if (angel == 0.0f)
            auto_ctrl_.current_strategy = ROTATE_PATH_POSITIVE;
        else if (angel == 180.0f)
            auto_ctrl_.current_strategy = ROTATE_PATH_NEGATIVE;
        else
            auto_ctrl_.current_strategy = ROTATE_PATH_SHORTEST;
    }
    else
    {
        if (auto_ctrl_.kfs_num == ONLY_ONE)
            auto_ctrl_.current_strategy = recorded_align_strategy_;
        else
            auto_ctrl_.current_strategy = ROTATE_PATH_SHORTEST;
    }

    // [单圈模式] 策略修正
    if (!rotate_multiTurn_)
    {
        if (auto_ctrl_.kfs_num == ONLY_ONE)
        {
            // [Version 8.0] 单圈模式下，OnlyOne模式强制跟随Align阶段的旋转方向
            if(s_has_recorded_strategy)
                auto_ctrl_.current_strategy = recorded_align_strategy_;
            else
            {
                // [Fix] 无记录（首次重定位后）：强制最短路径
                auto_ctrl_.current_strategy = ROTATE_PATH_SHORTEST;
            }
        }
        else
        {
            if (s_has_recorded_strategy)
            {
                // [Fix] 有记录：强制反向，严格执行单圈策略 (如 270->180->90->0)
                if (recorded_carrying_strategy_ == ROTATE_PATH_POSITIVE)
                    auto_ctrl_.current_strategy = ROTATE_PATH_NEGATIVE;
                else if (recorded_carrying_strategy_ == ROTATE_PATH_NEGATIVE)
                    auto_ctrl_.current_strategy = ROTATE_PATH_POSITIVE;
            }
            else
            {
                // [Fix] 无记录（首次重定位后）：强制最短路径
                auto_ctrl_.current_strategy = ROTATE_PATH_SHORTEST;
            }
        }
    }

    this->setRotateStrategy(auto_ctrl_.current_strategy);
    this->set_RotateAngle(angel);
    this->set_StretchLength(0.0f); //确保伸展缩回
    // [Fix] 只有当角度误差小于阈值时才返回 true，确保动作执行完成
    if(_tool_Abs(diff) < 2.0f)
        return true;
    else
        return false;
#endif
}

/**
 * @brief 寻自动单个
 */

 float test_target = 180.0f;
void ArmSetup::auto_onlyOne()
{
    /**
     * @brief 大致流程
     * 1. 升降到目标高度,吸盘pitch90度
     * 2. 云台旋转到起始位置，emmm，也可以调用return；
     * 3. 然后调用state_signAlign，预判旋转时机并执行对准KFS法平面，旋转完成后打开吸盘
     * 4. 伸展到目标KFS位置，吸附，停留0.3s后缩回
     * 5. 云台旋转回车头位置
     */
#if ARM_AUTOMOVE
    switch(auto_ctrl_.now_state)
    {
        case ARM_AUTO_E::STATE_DONE:
        {
            if(auto_ctrl_.start_to_autoctrl)
            {   
                if(!auto_ctrl_.flag.isrecalcPath)
                {
                    this->set_TargetKFS(auto_ctrl_.targetKFS[0], 0); //重置计算PathNode
                    auto_ctrl_.flag.isrecalcPath = true;
                }
                auto_ctrl_.flag.issafetoLower = false;
                auto_ctrl_.flag.align_done = false;
                auto_ctrl_.flag.ext_done = false;
                auto_ctrl_.flag.carry_done = false;
                auto_ctrl_.flag.return_done = false;
                auto_ctrl_.flag.ext_started = false;
                auto_ctrl_.flag.is_reachingTarget = false;
                auto_ctrl_.flag.reach_finishTime = 0.0f;
                
                s_has_recorded_strategy = false; // [新增] 重置策略记录标志
                recorded_align_strategy_ = ROTATE_PATH_SHORTEST; // [新增] 重置记录的Align策略

                bool return_done = false;

                return_done = state_return(0); //头一个KFS，传入0
                if(return_done)
                    auto_ctrl_.now_state = ARM_AUTO_E::STATE_TO_TARGET_HIGHT;
            }
            else
            {
                this->idle();
            }
            break;
        }

        case ARM_AUTO_E::STATE_TO_TARGET_HIGHT:
        {
            state_toTargetHight(auto_ctrl_.targetKFS[0]);
            //判断是否到达目标高度
            // [Fix] 如果目标高度低于安全高度，且当前已到达安全高度（因禁区限制无法继续下降），也允许进入下一状态
            float target_h = 0.0f;
            if(MF_high[auto_ctrl_.targetKFS[0]-1] ==0.2f)
                target_h = 0.0f;
            else if(MF_high[auto_ctrl_.targetKFS[0]-1] ==0.4f)
                target_h = auto_ctrl_.flag.safe_height;
            else if(MF_high[auto_ctrl_.targetKFS[0]-1] ==0.6f)
                target_h = init_data_.max_launchHeight_;
            float current_h = this->get_currentJointStatus().launchJoint_Height_;
            
            if(_tool_Abs(current_h - target_h) < 0.01f || 
               (target_h < auto_ctrl_.flag.safe_height 
                    && _tool_Abs(current_h - auto_ctrl_.flag.safe_height) < 0.01f))
            {
                auto_ctrl_.now_state = ARM_AUTO_E::STATE_SIGN_ALIGN;

            }
            break;
        }

        case ARM_AUTO_E::STATE_SIGN_ALIGN:
        {
            // static bool align_done = false;
            state_signAlign(auto_ctrl_.targetKFS[0], auto_ctrl_.flag.align_done);
            if(auto_ctrl_.flag.align_done)
            {
                auto_ctrl_.now_state = ARM_AUTO_E::STATE_AIM_EXT;
            }
            break;
        }

        case ARM_AUTO_E::STATE_AIM_EXT:
        {
            // static bool ext_done = false;
            auto_ctrl_.flag.ext_done = state_aimExt(auto_ctrl_.targetKFS[0]);
            if(auto_ctrl_.flag.ext_done)
            {
                auto_ctrl_.now_state = ARM_AUTO_E::STATE_RETURN;
            }
            break;
        }

        case ARM_AUTO_E::STATE_RETURN:
        {
            // static bool return_done = false;
            auto_ctrl_.flag.return_done = state_return(3); //无下一个KFS，传入0
            if(auto_ctrl_.flag.return_done)
            {
                auto_ctrl_.now_state = ARM_AUTO_E::STATE_DONE;
                //auto_ctrl_.start_to_autoctrl = false; //自动流程结束
                arm_ctrlStatus.auto_start = 0; //自动流程结束
                auto_ctrl_.start_to_autoctrl = false;
                auto_ctrl_.flag.isrecalcPath = false; //重置路径计算标志
            }
            break;
        }

        default:
            break;
    }
#endif
}

//下学期再根据实际需求修缮了
void ArmSetup::auto_two()
{
    /**
     * 就version 6.0的基础上，从原本的only_one模式，扩展到two模式
     *   在two模式下，机械臂会依次拾取两个KFS
     *   1. 执行和only_one模式一样的流程，拾取第一个KFS
     *   2. 在拾取第一个KFS的state_return阶段，机械臂会前往第二个KFS的初始位置(0/180度)，并且升高到安全高度(0.2m)
     *   3. 然后进入第二个KFS的拾取流程
     *   4. 第二个KFS的拾取流程和第一个类似，拾取完成后state_return到初始位置(0度)，结束。
     * 
     * 
     *   大致总体状态机策划
     *   1.刚进入auto_two模式，拾取第一个KFS的流程和only_one大致相同；
     *     都是先把标志位都先初始化完毕，然后进入state_done先调整到第一个KFS的初始位置
     *     然后进入state_to_target_height，升降到第一个KFS的目标高度
     *     然后进入state_sign_align，对齐第一个KFS 接着是state_aim_ext伸展吸附
     *     最后是state_carrying放置第一个KFS
     *     然后执行state_return，返回到第二个KFS的初始位置(0/180度)，并且升高到安全高度(0.2m)
     *     接着进入第二个KFS的拾取流程
     *     都是先进入state_to_target_height，升降到第二个KFS的目标高度
     *     然后进入state_sign_align，对齐第二个KFS
     *     接着是state_aim_ext伸展吸附
     *     然后是state_carrying放置第二个KFS
     *     最后执行state_return，返回到初始位置(0度)，结束。
     *     
     *     这里也有一点就是旋转策略的继承
     *     拾取第二个KFS的时候，旋转策略不继承第一个KFS的。
     *     而是在state_sign_align阶段重新记录旋转策略。
     *     这样做的目的是防止第一个KFS的旋转策略对第二个KFS造成影响。
     * 
     *     @details 没有人类了
     */
#if ARM_AUTOMOVE
    switch(auto_ctrl_.now_state)
    {
        case ARM_AUTO_E::STATE_DONE:
        {
            if(auto_ctrl_.start_to_autoctrl)
            {
                // 初始化标志位
                auto_ctrl_.flag.align_done = false;
                auto_ctrl_.flag.ext_done = false;
                auto_ctrl_.flag.carry_done = false;
                auto_ctrl_.flag.return_done = false;

                auto_ctrl_.flag.is_reachingTarget = false;
                auto_ctrl_.flag.reach_finishTime = 0.0f;
                
                s_has_recorded_strategy = false; 
                recorded_align_strategy_ = ROTATE_PATH_SHORTEST; 

                auto_ctrl_.now_targetIndex = 0; // 从第一个KFS开始

                // 前往第一个KFS的初始位置
                bool return_done = state_return(auto_ctrl_.now_targetIndex); 

                if(return_done)
                {
                    auto_ctrl_.now_state = ARM_AUTO_E::STATE_TO_TARGET_HIGHT; //影色舞
                }
            }
            else
            {
                this->idle();
            }
            break;
        }

        case ARM_AUTO_E::STATE_TO_TARGET_HIGHT:
        {
            int current_kfs_idx = auto_ctrl_.now_targetIndex;
            state_toTargetHight(auto_ctrl_.targetKFS[current_kfs_idx]);
            
            // 高度判断逻辑
            float target_h = 0.0f;
            float kfs_h = MF_high[auto_ctrl_.targetKFS[current_kfs_idx]-1];
            
            if(kfs_h == 0.2f)
                target_h = 0.0f;
            else if(kfs_h == 0.4f)
                target_h = auto_ctrl_.flag.safe_height;
            else if(kfs_h == 0.6f)
                target_h = init_data_.max_launchHeight_;

            float current_h = this->get_currentJointStatus().launchJoint_Height_;
            
            //判断是否到达目标高度
            // [Fix] 如果目标高度低于安全高度，且当前已到达安全高度（因禁区限制无法继续下降），
            // 也允许进入下一状态

            if(_tool_Abs(current_h - target_h) < 0.01f || 
               (target_h < auto_ctrl_.flag.safe_height && 
                    _tool_Abs(current_h - auto_ctrl_.flag.safe_height) < 0.01f))
            {
                auto_ctrl_.now_state = ARM_AUTO_E::STATE_SIGN_ALIGN;
            }
            break;
        }

        case ARM_AUTO_E::STATE_SIGN_ALIGN:
        {
            int current_kfs_idx = auto_ctrl_.now_targetIndex;
            state_signAlign(auto_ctrl_.targetKFS[current_kfs_idx], auto_ctrl_.flag.align_done);
            if(auto_ctrl_.flag.align_done)
            {
                auto_ctrl_.now_state = ARM_AUTO_E::STATE_AIM_EXT;
            }
            break;
        }

        case ARM_AUTO_E::STATE_AIM_EXT:
        {
            int current_kfs_idx = auto_ctrl_.now_targetIndex;
            auto_ctrl_.flag.ext_done = state_aimExt(auto_ctrl_.targetKFS[current_kfs_idx]);
            if(auto_ctrl_.flag.ext_done)
            {
                auto_ctrl_.now_state = ARM_AUTO_E::STATE_CARRYING;
            }
            break;
        }

        case ARM_AUTO_E::STATE_CARRYING:
        {
            int current_kfs_idx = auto_ctrl_.now_targetIndex;
            state_carrying(auto_ctrl_.targetKFS[current_kfs_idx], auto_ctrl_.flag.carry_done);
            if(auto_ctrl_.flag.carry_done)
            {
                auto_ctrl_.now_state = ARM_AUTO_E::STATE_RETURN;
            }
            break;
        }

        case ARM_AUTO_E::STATE_RETURN:
        {
            if(auto_ctrl_.now_targetIndex == 0)
            {
                // 第一个KFS完成，准备前往第二个KFS
                // 传入1，表示下一个目标是第二个KFS (index 1)
                auto_ctrl_.flag.return_done = state_return(1); 
                
                if(auto_ctrl_.flag.return_done)
                {
                    // 切换到第二个KFS
                    auto_ctrl_.now_targetIndex = 1;
                    
                    // 重置标志位
                    auto_ctrl_.flag.align_done = false;
                    auto_ctrl_.flag.ext_done = false;
                    auto_ctrl_.flag.carry_done = false;
                    auto_ctrl_.flag.return_done = false;
                    
                    auto_ctrl_.flag.is_reachingTarget = false;
                    auto_ctrl_.flag.reach_finishTime = 0.0f;

                    // 重置策略，防止继承
                    s_has_recorded_strategy = false;
                    recorded_align_strategy_ = ROTATE_PATH_SHORTEST;

                    // 回到高度调整状态
                    auto_ctrl_.now_state = ARM_AUTO_E::STATE_TO_TARGET_HIGHT;
                }
            }
            else
            {
                // 第二个KFS完成，结束
                // 传入3，表示无下一个目标
                auto_ctrl_.flag.return_done = state_return(3);
                
                if(auto_ctrl_.flag.return_done)
                {
                    auto_ctrl_.now_state = ARM_AUTO_E::STATE_DONE;
                    auto_ctrl_.start_to_autoctrl = false;
                }
            }
            break;
        }
        default:
            idle();
            break;
    }
#endif
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
    // //测试
    // if(test_signal == 0) //所有电机电流强制为0；检查电机转动方向是否需要置反
    // {
    //     this->set_controlMode(CURRENT_CONTROL_MODE);
    //     this->motor_launch_->setTargetCurrent(0.0f);
    //     this->motor_stretch_->setTargetCurrent(0.0f);
    //     this->motor_rotate_->setTargetCurrent(0.0f);
    //     this->motor_pitch_->setTargetCurrent(0.0f);
    // }

    // //1~4 signal test用于测试电机电流方向和电机转动方向是否同相
    // else if(test_signal == 1)
    //     this->motor_launch_->setTargetCurrent(test_current);

    // else if(test_signal == 2)
    //     this->motor_stretch_->setTargetCurrent(test_current);

    // else if(test_signal == 3)
    //     this->motor_rotate_->setTargetCurrent(test_current);
        
    // else if(test_signal == 4)
    //     this->motor_pitch_->setTargetCurrent(test_current);

    // //航模遥控操纵测试
    // else if(test_signal == 5)
    // {
    //     this->manualControl();
    // }

    // else if(test_signal == 6) //测试stop功能
    // {
    //     this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    //     this->stop();
    // }

    // else if(test_signal == 7) //测试idle功能
    // {
    //     this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    //     this->idle();
    // }
    // else if(test_signal == 8)
    // {
    //     this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    //     this->set_LaunchHeight(test_launch_height);
    // } 
    // else if(test_signal == 9)
    // {
    //     this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    //     this->set_RotateAngle(test_rotate_angle);
    // }
    // else //empty
    // {
    //     this->set_controlMode(CURRENT_CONTROL_MODE);
    //     this->set_LaunchHeight(0.0f);
    //     this->set_StretchLength(0.0f);
    //     this->set_RotateAngle(0.0f);
    //     this->set_PitchAngle(0.0f);
    // }
    
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

   .Sucker_GPIO_Port = SUCKER_GPIO_Port,
    .Sucker_GPIO_Pin = SUCKER_Pin,

    .safe_height = 0.14f,
};