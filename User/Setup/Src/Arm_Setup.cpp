#include "Arm_setup.h"

void ArmSetup::loop()
{
    if(!arm_ctrlStatus.init_flag)
        return;

    if(!arm_ctrlStatus.is_calibrating)
    {
        calibrateM2006();
        arm_status_ = ARM_CALIBRATE;
    }

    //目前使用虚拟坐标进行自控逻辑验证
   auto_ctrl_.now_chassis_speed = get_nowChassisSpeed();
   auto_ctrl_.now_armPosition = get_nowArmPosition();
   auto_ctrl_.now_ChassisPosition = get_nowChassisPose();
    
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);


   #if ARM_AUTO_DEBUG_NOCHASSIS
    if(arm_ctrlStatus.is_calibrating)
    {
        arm_status_ = ARM_AUTO_CONTROL;
        this->start_toAutoCtrl(true);
    }
   #endif

    switch(arm_status_)
    {
        case ARM_MANUAL_CONTROL:
            {
                manualControl();
            }
            break;

        case ARM_AUTO_CONTROL:
            {
                if(arm_ctrlStatus.auto_debug_start == 1)
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

uint8_t test_signal = 0;
float test_current = 0.0f;
float rotate_rate = 0.02f;
float launch_rate = 9.99999975e-05;

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

        last_arm_status_ = ARM_MANUAL_CONTROL;
    }

    if(AirJoy::getinstance().RIGHT_X < 1450)
            target_joint_status_.rotateJoint_angle_ -= rotate_rate; // 旋转关节逆时针
        else if(AirJoy::getinstance().RIGHT_X > 1550)
            target_joint_status_.rotateJoint_angle_ += rotate_rate; // 旋转关节顺时针
        else
            target_joint_status_ = target_joint_status_; // 保持不变



        if(AirJoy::getinstance().RIGHT_Y < 1450)
            target_joint_status_.launchJoint_Height_ -= launch_rate; // 伸展关节收回
        else if(AirJoy::getinstance().RIGHT_Y > 1550)
            target_joint_status_.launchJoint_Height_ += launch_rate; // 伸展关节伸出
        else
            target_joint_status_.launchJoint_Height_ = target_joint_status_.launchJoint_Height_; // 保持不变

        if(_tool_Abs(AirJoy::getinstance().SWA - 1000) < 50)
            target_joint_status_.stretchJoint_Length_ = 0.0f; // 伸展关节收回到最小位置
        else if(_tool_Abs(AirJoy::getinstance().SWA - 2000) < 50)
            target_joint_status_.stretchJoint_Length_ = this->init_data_.max_stretchLength_; // 伸展关节伸出到最大位置
        else 
            target_joint_status_.stretchJoint_Length_ = target_joint_status_.stretchJoint_Length_; // 保持不变



        if(_tool_Abs(AirJoy::getinstance().SWD - 1000) < 50)
            target_joint_status_.suckerJoint_angle_ = 0.0f; // 末端关节收
        else if(_tool_Abs(AirJoy::getinstance().SWD - 2000) < 50)
            target_joint_status_.suckerJoint_angle_ = 95.0f; // 末端关节开
        else 
            target_joint_status_.suckerJoint_angle_ = target_joint_status_.suckerJoint_angle_; // 保持不变

        this->set_LaunchHeight(target_joint_status_.launchJoint_Height_);
        this->set_StretchLength(target_joint_status_.stretchJoint_Length_);
        this->set_RotateAngle(target_joint_status_.rotateJoint_angle_);
        this->set_PitchAngle(target_joint_status_.suckerJoint_angle_);

        if(_tool_Abs(AirJoy::getinstance().SWC - 2000) < 50)
            this->setSuckerStatus(Sucker_Status_E::SUCK);
        else
            this->setSuckerStatus(Sucker_Status_E::STOP);
}
/*=======================================================*/

/**
 * @brief 如果有两个目标KFS，则第一个KFS拾取完后放到存储机构
 *        第二个KFS拾取完后留在吸盘上
 *        如果没有第二个，就吸在吸盘上，不必放到存储机构
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

    switch(auto_ctrl_.kfs_num)
    {
        case ONLY_ONE:
        {
            //单个KFS拾取流程
            auto_onlyOne();
            break;
        }
        
        case TWO:
        {
            //两个KFS拾取流程
            break;
        }
    }


}

//流程函数

void ArmSetup::state_toTargetHight(int targetKFS)
{
    /**
     * 1. 根据KFS编号，查询对应高度
     * 2. 升降到对应高度，吸盘pitch90度
     * 3. 锁住云台，等待移动到目标前一桩，云台开始预判旋转时机
     */

    //20cm台阶 升降0m, 40cm台阶升降 0.2m, 60cm台阶升降0.4m
    float kfs_height = MF_high[targetKFS -1]; //获取目标KFS高度

    this->set_LaunchHeight((kfs_height - 0.2f)); 
    this->set_PitchAngle(90.0f); //吸盘pitch90度
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

    // [修正] 正确的坐标变换：将世界坐标投影到机械臂局部坐标系
    // Local X: 沿机械臂轴向 (点乘方向向量 (c, s))
    // Local Y: 垂直机械臂轴向 (点乘法向量 (-s, c))
    Point2D local = {
         d.x * c + d.y * s, // Local X
        -d.x * s + d.y * c, // Local Y
         0.0f
    };
    
    // 矩形碰撞判断: x in [0, L], y in [-W/2, W/2]
    if(local.x >= 0.0f && local.x <= L_arm && _tool_Abs(local.y) <= (W_arm / 2.0f))
        return true; //碰撞
    else
        return false; //未碰撞
}

void ArmSetup::state_signAlign(int targetKFS, bool &align_done)
{
    /**
     * 云台旋转时机预判，以及执行
     * 
     * @details 依旧屎山堆积
     */
    MF_AutoCtrler::Direction_E move_direction;
    Point2D target_pos = {0, 0 ,0};
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
                static_cast<float>(auto_ctrl_.gimbal_calcHz))
        return; //未到计算时间，直接返回

    int index = 0;
    if(targetKFS == auto_ctrl_.targetKFS[0])
        index = 0;
    else if(targetKFS == auto_ctrl_.targetKFS[1])
        index = 1;

    //开始计算
    auto_ctrl_.gimbal_calcCount = 0;
    get_GimbalMF_PAPB(targetKFS, auto_ctrl_.PointPAB[index].PA, 
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
                    (auto_ctrl_.time_set.gimbal_max_rad * 0.3); //云台旋转所需时间(s)

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
                            auto_ctrl_.arm_width, auto_ctrl_.arm_width)
            ||
            check_Arm_collision(PB.x, PB.y, 
                            pivot.x, pivot.y, 
                            world_angle_t, 
                            auto_ctrl_.arm_width, auto_ctrl_.arm_width))
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

    else if(diff  > 0)
        auto_ctrl_.current_strategy = ROTATE_PATH_POSITIVE;
    
    else if (diff < 0)
        /* code */
        auto_ctrl_.current_strategy = ROTATE_PATH_NEGATIVE;
    
    else
        auto_ctrl_.current_strategy = ROTATE_PATH_SHORTEST;
    


    this->setRotateStrategy(auto_ctrl_.current_strategy);

    //执行
    if(safe)
    {
        this->set_RotateAngle(90.0f); //对齐目标角度
        if(_tool_Abs(diff) < 2.0f)
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
}

Point2D pos_tar_kfs = {0.0f, 0.0f, 0.0f};
Point2D pos_start_kfs = {0.0f, 0.0f, 0.0f};

/**
 * @brief 伸展到目标KFS位置 条件预判
 */

 bool back = false;
float angle = 0.0f;
bool ArmSetup::state_aimExt(int targetKFS)
{
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

        Point2D target_pos = {0, 0 ,0};

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
        switch(move_direction)
        {
            case MF_AutoCtrler::Positive_X:
            {
                if(_tool_Abs(auto_ctrl_.now_chassis_speed.x) < 0.1f)
                    return false; //速度为0，无法伸展
                t_need = _tool_Abs((target_pos.x - auto_ctrl_.now_armPosition.x) 
                        / auto_ctrl_.now_chassis_speed.x);
                break;
            }
            case MF_AutoCtrler::Negative_X:
            {
                if(_tool_Abs(auto_ctrl_.now_chassis_speed.x) < 0.1f)
                    return false; //速度为0，无法伸展

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

                t_need = _tool_Abs((auto_ctrl_.now_armPosition.y - target_pos.y) 
                        / auto_ctrl_.now_chassis_speed.y);
                break;
            }
            default:
                break;
        }

        //或许delta_t < 0.02s会错过伸展窗口(计算频率)
        //给足提前量容忍，或许会更好，避免过严格的等式触发

        const float delta_t = auto_ctrl_.time_set.stretch_time_s / 6; //提前量容忍

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
        



		// if(_tool_Abs(t_need-t_stretch) < 0.02)
		// {
		// 	safe = true;
		// }
		
		// if(safe)
		// {
		// 	this->set_StretchLength(arm_initData.max_stretchLength_);
		// }

		current_armLength = get_currentJointStatus().stretchJoint_Length_;

		if(_tool_Abs(current_armLength-arm_initData.max_stretchLength_) < 0.005f && !auto_ctrl_.flag.is_reachingTarget)
		{
			auto_ctrl_.flag.is_reachingTarget = true;
            pos_tar_kfs = auto_ctrl_.now_armPosition; //记录伸展到达位置
			auto_ctrl_.flag.reach_finishTime = TimeStamp::getInstance().getSeconds();
		}
		if(auto_ctrl_.flag.is_reachingTarget && (now_time_s_-auto_ctrl_.flag.reach_finishTime) >= 0.3f)
		{
			this->set_StretchLength(0.0f);
            auto_ctrl_.flag.ext_started = false; //重置伸展开始标志

            if(this->get_currentJointStatus().stretchJoint_Length_ < 0.005f)
            {
                
                   
			    return true;
            }
            return false;
		}

        else
			return false;
			
}

void ArmSetup::state_carrying(int targetKFS ,bool &carrying_done)
{
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
    float target_deg = 359.0f; //存储机构位置角度为0度

    //Rotate_Strategy_E strategy = auto_ctrl_.current_strategy;

    //计算符合 的 diff
    float diff = 0.0f;
    float current_mod = fmodf(current_deg, 360.0f);
    if(current_mod <0)
        current_mod += 360.0f;
    float target_mod = 0.0f;

    float raw_diff = target_mod - current_mod;

    switch(auto_ctrl_.current_strategy)
    {
        case ROTATE_PATH_POSITIVE:
        {
            //必须正转
            if(raw_diff <= 0.0f)
                diff = raw_diff + 360.0f;
            else
                diff = raw_diff;
            break;
        }

        case ROTATE_PATH_NEGATIVE:
        {
            //必须负转
            if(raw_diff >= 0.0f)
                diff = raw_diff - 360.0f;
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
    float check_W = (auto_ctrl_.arm_width > kfs_size) ? auto_ctrl_.arm_width : kfs_size;

    bool safe = true;

    //time calc
    float T_rot = _tool_Abs(diff) * (PI / 180.0f) / 
                (auto_ctrl_.time_set.gimbal_max_rad * 0.32f); //云台旋转所需时间(s)

    for(float t = 0.0f; t <= T_rot; t+= 0.05f)
    {
        Point2D pivot{
            .x = auto_ctrl_.now_armPosition.x + vx * t,
            .y = auto_ctrl_.now_armPosition.y + vy * t,
            .theta = 0.0f
        };

        float step_deg = 0.0f;
        if(diff > 0 )
            step_deg = 1.0f * (auto_ctrl_.time_set.gimbal_max_rad 
                    * 0.32f * 180.0f / PI) * t; //每步旋转

        else
            step_deg = -1.0f * (auto_ctrl_.time_set.gimbal_max_rad 
                    * 0.32f * 180.0f / PI) * t; //每步旋转
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

    if(auto_ctrl_.store[targetKFS].is_toPlace == false)
    {
        if(this->get_currentJointStatus().launchJoint_Height_
            < auto_ctrl_.store[targetKFS].safe_height)
        {
            this->set_LaunchHeight(auto_ctrl_.store[targetKFS].safe_height);
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
            auto_ctrl_.store[targetKFS].is_toPlace = false;

        }
        else if(_tool_Abs(diff) <= 2.0f)
        {
             this->setRotateStrategy(ROTATE_PATH_SHORTEST);
            if(auto_ctrl_.store[targetKFS].is_toPlace == false)
            {
                //降低云台放置KFS到存储机构位置
                this->set_LaunchHeight(auto_ctrl_.store[targetKFS].store_height);

                wait_startTime = this->now_time_s_;
                auto_ctrl_.store[targetKFS].is_toPlace = true;
            }

            else if(auto_ctrl_.store[targetKFS].is_toPlace == true)
            {
                this->set_LaunchHeight(auto_ctrl_.store[targetKFS].store_height); //维持不变

                //0.2s后吸盘关闭
                if(this->now_time_s_ - wait_startTime > 0.2f)     
                    this->setSuckerStatus(Sucker_Status_E::STOP);
                

                else if(this->now_time_s_ - wait_startTime > 0.5f)
                {
                    //抬高云台到安全高度
                    this->set_LaunchHeight(auto_ctrl_.store[targetKFS].safe_height);        
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
    
}

bool ArmSetup::state_return(int next_targetKFS)
{
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
    float angel;
	if(next_targetKFS==0||next_targetKFS==1)
	{
		
		int TargetMap;
		int Target_KFS=auto_ctrl_.targetKFS[next_targetKFS];
		if(next_targetKFS==0)
		    TargetMap=auto_ctrl_.path.bestB1;
		if(next_targetKFS==1)
		    TargetMap=auto_ctrl_.path.bestB2;
		
		angel=MF_AutoCtrler::Get_ArmBaseTargetAngle(TargetMap,auto_ctrl_.KFS_Movedirection[next_targetKFS]);

        float current_angle = this->get_currentJointStatus().rotateJoint_angle_;
        float diff = angel - fmodf(current_angle, 360.0f);

        // 简单的归一化处理，确保 diff 在 -180 ~ 180
        if(diff > 180.0f) diff -= 360.0f;
        else if(diff < -180.0f) diff += 360.0f;

        if(_tool_Abs(diff) < 2.0f)
        {
            auto_ctrl_.current_strategy = ROTATE_PATH_SHORTEST; // 误差极小时，锁定最短路径
        }
        else if(angel == 0)
        {
            auto_ctrl_.current_strategy=ROTATE_PATH_POSITIVE;
        }
        else if(angel==180)
        {
            auto_ctrl_.current_strategy=ROTATE_PATH_NEGATIVE;
        }


		this->setRotateStrategy(auto_ctrl_.current_strategy);
		this->set_RotateAngle(angel);
		return true;
	}
    else
    {
		// 同理修复 else 分支
        float current_angle = this->get_currentJointStatus().rotateJoint_angle_;
        float diff = angel - fmodf(current_angle, 360.0f);
        if(diff > 180.0f) diff -= 360.0f;
        else if(diff < -180.0f) diff += 360.0f;

        if(_tool_Abs(diff) < 2.0f)
        {
            auto_ctrl_.current_strategy = ROTATE_PATH_SHORTEST;
        }
        else
        {
            auto_ctrl_.current_strategy=ROTATE_PATH_POSITIVE;
        }

        this->setRotateStrategy(auto_ctrl_.current_strategy);
        this->set_RotateAngle(angel);
        return true;
	}
}

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

    switch(auto_ctrl_.now_state)
    {
        case STATE_DONE:
        {
            if(auto_ctrl_.start_to_autoctrl)
            {
                auto_ctrl_.flag.align_done = false;
                auto_ctrl_.flag.ext_done = false;
                auto_ctrl_.flag.carry_done = false;
                auto_ctrl_.flag.return_done = false;

                auto_ctrl_.flag.is_reachingTarget = false;
                auto_ctrl_.flag.reach_finishTime = 0.0f;

                bool return_done = false;

                return_done = state_return(0); //头一个KFS，传入0

                if(return_done)
                    auto_ctrl_.now_state = STATE_TO_TARGET_HIGHT;
            }
            else
            {
                this->idle();
            }
            break;
        }

        case STATE_TO_TARGET_HIGHT:
        {
            state_toTargetHight(auto_ctrl_.targetKFS[0]);
            //判断是否到达目标高度
            if(_tool_Abs(this->get_currentJointStatus().launchJoint_Height_ 
                - (MF_high[auto_ctrl_.targetKFS[0]-1] - 0.2f)) < 0.01f)
            {
                auto_ctrl_.now_state = STATE_SIGN_ALIGN;

            }
            break;
        }

        case STATE_SIGN_ALIGN:
        {
            // static bool align_done = false;
            state_signAlign(auto_ctrl_.targetKFS[0], auto_ctrl_.flag.align_done);
            if(auto_ctrl_.flag.align_done)
            {
                auto_ctrl_.now_state = STATE_AIM_EXT;
            }
            break;
        }

        case STATE_AIM_EXT:
        {
            // static bool ext_done = false;
            auto_ctrl_.flag.ext_done = state_aimExt(auto_ctrl_.targetKFS[0]);
            if(auto_ctrl_.flag.ext_done)
            {
                auto_ctrl_.now_state = STATE_CARRYING;
            }
            break;
        }

        case STATE_CARRYING:
        {
            // static bool carrying_done = false;
            state_carrying(auto_ctrl_.targetKFS[0], auto_ctrl_.flag.carry_done);
            //判断是否放置完毕
            if(auto_ctrl_.flag.carry_done)
            {
                auto_ctrl_.now_state = STATE_RETURN;
            }
            break;
        }

        case STATE_RETURN:
        {
            // static bool return_done = false;
            auto_ctrl_.flag.return_done = state_return(0); //无下一个KFS，传入0
            if(auto_ctrl_.flag.return_done)
            {
                auto_ctrl_.now_state = STATE_DONE;
                auto_ctrl_.start_to_autoctrl = false; //自动流程结束
            }
            break;
        }

        default:
            break;
    }
}



/*=================================================================*/

void ArmSetup::stop()
{
    // 停止控制函数
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    this->setSuckerStatus(Sucker_Status_E::STOP);

    this->set_LaunchHeight(0.0f);
    this->set_StretchLength(0.0f);
    this->set_RotateAngle(0.0f);
    this->set_PitchAngle(0.0f);

    if(_tool_Abs(this->motor_launch_->getTotalAngle() - 0.0f) < 0.1f)
        this->motor_launch_->setTargetCurrent(0.0f);
    
    if(_tool_Abs(this->motor_stretch_->getTotalAngle() - 0.0f) < 0.1f)
        this->motor_stretch_->setTargetCurrent(0.0f);

    if(_tool_Abs(this->motor_rotate_->getTotalAngle() - 0.0f) < 0.1f)
        this->motor_rotate_->setTargetCurrent(0.0f);

    if(_tool_Abs(this->motor_pitch_->getTotalAngle() - 0.0f) < 0.1f)
        this->motor_pitch_->setTargetCurrent(0.0f);
    if((_tool_Abs(this->motor_launch_->getTotalAngle() - 0.0f) < 0.1f) && 
       (_tool_Abs(this->motor_stretch_->getTotalAngle() - 0.0f) < 0.1f) &&
       (_tool_Abs(this->motor_rotate_->getTotalAngle() - 0.0f) < 0.1f) &&
       (_tool_Abs(this->motor_pitch_->getTotalAngle() - 0.0f) < 0.1f))
    {
        //全部到位后，切换到空闲模式
        this->set_controlMode(CURRENT_CONTROL_MODE);
        this->motor_launch_->setTargetCurrent(0.0f);
        this->motor_stretch_->setTargetCurrent(0.0f);
        this->motor_rotate_->setTargetCurrent(0.0f);
        this->motor_pitch_->setTargetCurrent(0.0f);
    }
}

void ArmSetup::calibrateM2006()
{
    this->set_controlMode(CURRENT_CONTROL_MODE); 
    // 上电校准M2006电机位置
    // 给予M2006一个小电流顶住限位，然后计时1s，将当前位置重定位为0度
    if(!arm_ctrlStatus.calibrate_start)
    {
        arm_ctrlStatus.calibrate_startTime = TimeStamp::getInstance().getSeconds();
        arm_ctrlStatus.calibrate_start = true;
    }
    this->motor_stretch_->setTargetCurrent(-1000.0f); // 给予一个小电流顶住限位
    this->motor_pitch_->setTargetCurrent(1000.0f); // 给予一个小电流顶住限位

    if(this->now_time_s_ - arm_ctrlStatus.calibrate_startTime > 1.5f)
    {
        this->motor_stretch_->relocate_totalAngle(0.0f);
        this->motor_pitch_->relocate_totalAngle(0.0f);
        this->motor_stretch_->setTargetCurrent(0.0f);
        this->motor_pitch_->setTargetCurrent(0.0f);

        arm_ctrlStatus.is_calibrating = true;
    }
}

void ArmSetup::idle()
{
    // 空闲控制函数，若上一时刻非此模式，则记忆上一时刻位置，并维持不变
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

    this->setSuckerStatus(Sucker_Status_E::STOP);
}

float stretch_starttime = 0;
bool stretch_flag = false;
float pitch_starttime = 0;
bool pitch_flag = false;
float stretch_usetime = 0;
float pitch_usetime = 0;


float test_rotate_angle = 0.2f;

float test_launch_height = 0.01f;
volatile float launch_see = 0.0f;
void ArmSetup::debug()
{
    //测试
    if(test_signal == 0) //所有电机电流强制为0；检查电机转动方向是否需要置反
    {
        this->set_controlMode(CURRENT_CONTROL_MODE);
        this->motor_launch_->setTargetCurrent(0.0f);
        this->motor_stretch_->setTargetCurrent(0.0f);
        this->motor_rotate_->setTargetCurrent(0.0f);
        this->motor_pitch_->setTargetCurrent(0.0f);
    }

    //1~4 signal test用于测试电机电流方向和电机转动方向是否同相
    else if(test_signal == 1)
        this->motor_launch_->setTargetCurrent(test_current);

    else if(test_signal == 2)
        this->motor_stretch_->setTargetCurrent(test_current);

    else if(test_signal == 3)
        this->motor_rotate_->setTargetCurrent(test_current);
        
    else if(test_signal == 4)
        this->motor_pitch_->setTargetCurrent(test_current);

    //航模遥控操纵测试
    else if(test_signal == 5)
    {
        this->manualControl();
    }

    else if(test_signal == 6) //测试stop功能
    {
        this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
        this->stop();
    }

    else if(test_signal == 7) //测试idle功能
    {
        this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
        this->idle();
    }
    else if(test_signal == 8)
    {
        this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
        this->set_LaunchHeight(test_launch_height);
    } 
    else if(test_signal == 9)
    {
        this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
        this->set_RotateAngle(test_rotate_angle);
    }
    else //empty
    {
        this->set_controlMode(CURRENT_CONTROL_MODE);
        this->set_LaunchHeight(0.0f);
        this->set_StretchLength(0.0f);
        this->set_RotateAngle(0.0f);
        this->set_PitchAngle(0.0f);
    }
    
}

Arm_InitData_S arm_initData = {
   .max_launchHeight_ = 0.4f,
   .max_stretchLength_ = 0.130f,
   .arm_length_ = 0.6f,
   .end_link_length_ = 0.08f,

   .stretch_Ratio_ = 0.08417f,
   .launch_Ratio_ = 0.07221f,
   .rotate_gearRatio_ = 144.878f,
   .pitch_gearRatio_ = 360.0f,

   .min_rotate_angle_ = 0.0f,
   .max_rotate_angle_ = 359.999f,
};



