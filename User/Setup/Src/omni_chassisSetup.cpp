#include "omni_chassisSetup.h"

// Path_line path_line_;
// Speedplanner_1D_Param_Config path_param({.maxAcc = 3.0f, .maxDec = 3.0f, .maxJerk = 4.0f, .maxSpeed = 0.5f, .initialSpeed = 0.05f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f});
// Path_line path_line_(path_param);

#if debug_ladar

int last_cout_ladar_data = -1;

#endif

uint32_t chassisstackHighWaterMark = 0;

void OmniChassis_Setup::ResetAutoControlStates(void)
{
    // 1) 阻尼项使用上一时刻 v_robot，退出自动流程后必须清零，避免“历史速度”带入下一次任务。
    v_robot_last_cmd_ = {0.0f, 0.0f};

    // 2) 前馈差分状态一并复位，避免参考点跳变时出现首帧尖峰。
    ff_diff_inited_ = false;
    ff_ref_point_last_ = {0.0f, 0.0f};
    ff_velocity_lpf_ = {0.0f, 0.0f};
    ff_last_tick_ms_ = 0;
}

Vector2D OmniChassis_Setup::ComputeLookaheadDiffFeedforward(bool near_end)
{
    // 使用 RTOS tick 估计离散 dt（单位秒）；该方法在嵌入式任务循环中稳定且开销小。
    //uint32_t now_tick_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    Vector2D v_ff_raw = {0.0f, 0.0f};

    // 首次进入或状态复位后，不做差分，先对齐历史参考点。
    if (!ff_diff_inited_)
    {
        ff_diff_inited_ = true;
        ff_ref_point_last_ = ff_ref_point_;
        //ff_last_tick_ms_ = now_tick_ms;
        ff_velocity_lpf_ = {0.0f, 0.0f};
        return ff_velocity_lpf_;
    }

    // 计算 dt，防止 0 或过小导致差分放大。
    float dt_s = getdt();
    if (dt_s <= 0.0f)
    {
        dt_s = control_period_s_;
    }
    if (dt_s < ff_dt_min_s_)
    {
        dt_s = ff_dt_min_s_;
    }
    if (dt_s > ff_dt_max_s_)
    {
        dt_s = ff_dt_max_s_;
    }

    // 参考点差分前馈：
    // p_ref 由 Path_correction 更新，常规阶段为 lookaheadPt，终点阶段为 endPt。
    // 这样可在不依赖 planspeed 的前提下，给 PID 额外“提前量”。
    v_ff_raw = (ff_ref_point_ - ff_ref_point_last_) * (kff_la_ / dt_s);

    // 一阶低通：抑制前视点参数 t 跳变引起的速度尖峰。
    ff_velocity_lpf_ = ff_velocity_lpf_ * (1.0f - ff_lpf_alpha_) + v_ff_raw * ff_lpf_alpha_;

    // 限幅：前馈只是加速辅助，不能反客为主压过 PID 闭环。
    if (ff_velocity_lpf_.magnitude() > max_ff_speed_)
    {
        ff_velocity_lpf_ = ff_velocity_lpf_.normalize() * max_ff_speed_;
    }

    // 终点段衰减：减少“冲终点”风险，把控制权更多交给 PID 位置吸附。
    Vector2D v_ff = ff_velocity_lpf_;
    if (near_end)
    {
        v_ff = v_ff * end_ff_scale_;
    }

    // 更新历史量，供下一周期差分。
    ff_ref_point_last_ = ff_ref_point_;
    //ff_last_tick_ms_ = now_tick_ms;

    return v_ff;
}

Vector2D OmniChassis_Setup::ComposeRobotVelocity(const Vector2D &v_pid, const Vector2D &v_ff_ref, bool near_end)
{
    float pid_scale = 1.0f;
    float max_speed = max_robot_speed_;

    if (near_end)
    {
        pid_scale = end_pid_scale_;
        max_speed = max_robot_speed_end_;
    }

    Vector2D v_pid_out = v_pid * pid_scale;
    // v_ff_ref 已经在 ComputeLookaheadDiffFeedforward 中完成了增益、滤波、限幅和终点衰减。
    Vector2D v_ff = v_ff_ref;

    Vector2D v_damp = v_robot_last_cmd_ * (-k_damp_);

    // 最终速度合成：闭环主导 + 前馈提速 + 历史速度阻尼。
    Vector2D v_robot = v_pid_out + v_ff + v_damp;
    if (v_robot.magnitude() > max_speed)
    {
        v_robot = v_robot.normalize() * max_speed;
    }

    v_robot_last_cmd_ = v_robot;
    return v_robot;
}

void OmniChassis_Setup::loop()
{
    if (!init_flag)
        return;

//	chassisstackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
	
    float dyaw = Locate_Setup::getInstance()->get_dyaw_from_position();
    yaw = Locate_Setup::getInstance()->get_yaw_from_position();
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
    ladar_data_ = Locate_Setup::getInstance()->get_RobotPos_inWorld();
    robot_pos_.x = ladar_data_.x;
    robot_pos_.y = ladar_data_.y;
    robot_pos_.x += original_point_.x;
    robot_pos_.y += original_point_.y;
    
    //Acc_target_yaw_ = Acc_yaw_.plan(target_yaw_);

    Angle_Twist angle_twist = {0};
    angle_twist.yaw_rate = dyaw;
    angle_twist.yaw_angle = yaw;
    this->updateAngleData(angle_twist);

    switch (chassis_status_)
    {
    case CHASSIS_MANUAL_CONTROL_A:
    {
        this->set_ControlMode(WORLD_SPEED_MODE);
        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_chassis_twist_.vx = airjoy_data_.left_x * 3 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vx = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_chassis_twist_.vy = airjoy_data_.left_y * 3 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vy = 0.0f;

        if (_tool_Abs(airjoy_data_.right_x) > 0.05f)
            target_chassis_twist_.yaw_rate = airjoy_data_.right_x * 6;
        else
            target_chassis_twist_.yaw_rate = 0.0f;

        target_yaw_ = yaw;

        this->set_Target(target_chassis_twist_);

        break;
    }

    case CHASSIS_MANUAL_CONTROL_B:
    {
        this->set_ControlMode(WORLD_SPEED_MODE);
        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_chassis_twist_.vx = airjoy_data_.left_x * 0.6 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vx = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_chassis_twist_.vy = airjoy_data_.left_y * 0.6 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vy = 0.0f;

        // 获取当前角度
        float yaw_real_angle = yaw;

        yaw_pid_period_count_++;
        if (yaw_pid_period_count_ >= yaw_pid_period_)
        {
            yaw_pid_period_count_ = 0;
            target_chassis_twist_.yaw_rate = yaw_pid_.pid_calc(target_yaw_, yaw_real_angle);
        }

        // this->setWorldSpeed(target_chassis_twist_);
        this->set_Target(target_chassis_twist_);
        // this->update();
        break;
    }

    case CHASSIS_LOCK_FORWEAPON:
    {
        this->set_ControlMode(WORLD_SPEED_MODE);
        float target_yaw_angle = 90.0f;

        float yaw_real_angle = yaw;

        yaw_pid_period_count_++;
        if (yaw_pid_period_count_ >= yaw_pid_period_)
        {
            yaw_pid_period_count_ = 0;
            target_chassis_twist_.yaw_rate = yaw_pid_.pid_calc(target_yaw_angle, yaw_real_angle);
        }

        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_chassis_twist_.vx = airjoy_data_.left_x * 3 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vx = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_chassis_twist_.vy = airjoy_data_.left_y * 3 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vy = 0.0f;

        // this->setWorldSpeed(target_chassis_twist_);
        this->set_Target(target_chassis_twist_);
        break;
    }

    case CHASSIS_AUTO_CONTROL_CB:
    {
        if (flag == 1)
        {
            flag = 0;
            flag_run = 1;
            Clamping_Bar_Selection_Planning();
			WeaponSage_Start=1;
        }
        if (flag_run == 1)
        {
            if (path_line_.Is_End() == true)
            {
                num++;
                // 5. 规划速度+叠加纠偏速度：计算路径规划的前进速度（切向速度）
                planspeed = path_line_.plan(robot_pos_);
                Path_correction();
                bool near_end = (tNearest > 0.95f);
                Vector2D v_ff = ComputeLookaheadDiffFeedforward(near_end);
                speed = ComposeRobotVelocity(corrVelocity, v_ff, near_end);
                target_chassis_twist_.vx = speed.x;
                target_chassis_twist_.vy = speed.y;
//                if(path_line_.get_pid_end_flag()==0)
//                {
//                    Path_correction();
//                    speed = planspeed + corrVelocity;// 最终速度 = 规划的前进速度 + 横向纠偏速度
//                }
//                else
//                    speed = planspeed; 
            }
            else
            {

                    flag = 0;
                    flag_run = 0;
                    speed.x = 0.0f;
                    speed.y = 0.0f;
                    path_line_.plan_reset();
                    path_line_.Reset();
                    planspeed.x = 0.0f;
                    planspeed.y = 0.0f;
                    ResetAutoControlStates();
                    chassis_status_ = CHASSIS_STOP;
                    target_chassis_twist_.vx = speed.x;
                    target_chassis_twist_.vy = speed.y;
					WeaponSage_END=1;

            }
        }
        else
        {
            target_yaw_ = yaw;
            target_chassis_twist_.vx = 0.0f;
            target_chassis_twist_.vy = 0.0f;
            speed.x = 0.0f;
            speed.y = 0.0f;
            ResetAutoControlStates();
        }
        if (path_line_.index_ == 1)
        {
                target_yaw_ = -90.0f;
//                if (abs(target_yaw_ - yaw) > 1.0f)
//                {
//                    target_chassis_twist_.vx = 0.0f;
//                    target_chassis_twist_.vy = 0.0f;
//                }

        }
        // 获取当前角度
        float yaw_real_angle = yaw;
        // float yaw_real_angle = ladar_data_.yaw;
        yaw_pid_period_count_++;
        if (yaw_pid_period_count_ >= yaw_pid_period_)
        {
            yaw_pid_period_count_ = 0;
            target_chassis_twist_.yaw_rate = yaw_pid_.pid_calc(target_yaw_, yaw_real_angle);
        }
        // this->set_ControlMode(CURRENT_ZERO_MODE);
        this->set_Target(target_chassis_twist_);
        break;
    }
    
    case CHASSIS_AUTO_CONTROL_KFS:
    {
        if (flag == 1)
        {
            flag = 0;
            flag_run = 1;
            KFS_Selection_Planning();
        }
        if (flag_run == 1)
        {
            if (path_line_.Is_End() == true)
            {
                num++;
                // 5. 规划速度+叠加纠偏速度：计算路径规划的前进速度（切向速度）
                planspeed = path_line_.plan(robot_pos_);
                Path_correction();
                bool near_end = (tNearest > 0.95f);
                Vector2D v_ff = ComputeLookaheadDiffFeedforward(near_end);
                speed = ComposeRobotVelocity(corrVelocity, v_ff, near_end);
                target_chassis_twist_.vx = speed.x;
                target_chassis_twist_.vy = speed.y;
//                if(path_line_.get_pid_end_flag()==0)
//                {
//                    Path_correction();
//                    speed = planspeed + corrVelocity;// 最终速度 = 规划的前进速度 + 横向纠偏速度
//                }
//                else
//                    speed = planspeed; 
            }
            else
            {

                    flag = 0;
                    flag_run = 0;
                    speed.x = 0.0f;
                    speed.y = 0.0f;
                    path_line_.plan_reset();
                    path_line_.Reset();
                    planspeed.x = 0.0f;
                    planspeed.y = 0.0f;
                    ResetAutoControlStates();
                    chassis_status_ = CHASSIS_STOP;
                    target_chassis_twist_.vx = speed.x;
                    target_chassis_twist_.vy = speed.y;
					WeaponSage_END=1;

            }
        }
        else
        {
            target_yaw_ = yaw;
            target_chassis_twist_.vx = 0.0f;
            target_chassis_twist_.vy = 0.0f;
            speed.x = 0.0f;
            speed.y = 0.0f;
            ResetAutoControlStates();
        }

        // 获取当前角度
        float yaw_real_angle = yaw;
        // float yaw_real_angle = ladar_data_.yaw;
        yaw_pid_period_count_++;
        if (yaw_pid_period_count_ >= yaw_pid_period_)
        {
            yaw_pid_period_count_ = 0;
            target_chassis_twist_.yaw_rate = yaw_pid_.pid_calc(target_yaw_, yaw_real_angle);
        }
        // this->set_ControlMode(CURRENT_ZERO_MODE);
        this->set_Target(target_chassis_twist_);
        break;
    }
    
    case CHASSIS_STOP:
    {

        // this->wheels_[0]->setTargetCurrent(0);
        // this->wheels_[1]->setTargetCurrent(0);
        // this->wheels_[2]->setTargetCurrent(0);
        this->set_ControlMode(CURRENT_ZERO_MODE);
        this->set_Target({0, 0, 0});
        ResetAutoControlStates();
        break;
    }

    case CHASSIS_MANUAL_CONTROL_C:
    {
        this->set_ControlMode(WORLD_SPEED_MODE);
        target_chassis_twist_.yaw_rate = 0.0f;

        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_chassis_twist_.vx = airjoy_data_.left_x * 6 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vx = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_chassis_twist_.vy = airjoy_data_.left_y * 6 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vy = 0.0f;

        this->set_Target(target_chassis_twist_);
        break;
    }
    default:
    {
        break;
    }
    }

    // 接收一次雷达数据打印一次

#if debug_ladar

    if (Lader_position::GetInstance(&hUsbDeviceHS)->return_coutlar_data() > last_cout_ladar_data)
    {
        debug_uart.Printf_Ladar(ladar_data_.x, ladar_data_.y);
        last_cout_ladar_data = Lader_position::GetInstance(&hUsbDeviceHS)->return_coutlar_data();
    }

#endif
    // debug_uart.Printf_Ladar(ladar_data_.x, ladar_data_.y);

    // debug_uart.printf_DMA("%f,%f,%f,%f\r\n",
    //                       target_yaw_,yaw,target_chassis_twist_.yaw_rate,dyaw);

    this->update();

    Point2D fk_speed;
    fk_speed.x = this->getWorldSpeed().vx;
    fk_speed.y = this->getWorldSpeed().vy;

    SpeedFK_Queue.send(fk_speed);
}


/////////////////////////////////    路径纠偏代码   //////////////////////////////////////////////

/**
 * @brief 整合已有接口，获取“最近点坐标”和“对应的t值”
 * @param robotPos 输入：机器人当前实际位置（闭环核心输入）
 * @param tNearest 输出：最近点对应的曲线参数t（0~1），给后续找前视点用
 * @return Vector2D 输出：最近点的坐标（给后续算横向偏差用）
 */
Vector2D OmniChassis_Setup::GetPathNearestPoint(BezierCurve &path_, const Vector2D &robotPos, float &tNearest)
{
    // 第一步：调用你的Get_Nearest_Distance，拿到tNearest（最近点对应的t值）
    // 重点：第二个参数传 &tNearest（tNearest的地址），因为你的函数是“输出参数”（通过指针赋值）
    path_.Get_Nearest_Distance(robotPos, &tNearest);

    // 第二步：用第一步拿到的tNearest，调用你的Get_Point，拿到最近点坐标
    Vector2D nearestPt = path_.Get_Point(tNearest);

    // 第三步：返回最近点坐标，给后续“算横向偏差”用
    return nearestPt;
}

// 函数作用：输入最近点的编号tNearest，输出前视点坐标和它的编号tLookahead
Vector2D OmniChassis_Setup::FindLookaheadPoint(BezierCurve &path_, float tNearest, float &tLookahead)
{
    // -------------- 对应第1步：初始化，从最近点开始 --------------
    tLookahead = tNearest;        // 前视点的编号，先从最近点的编号开始（比如t=0.3）
    float accumulatedDist = 0.0f; // 累计挪了多少距离（刚开始是0）
    float step = 0.01f;           // 每次挪的“小步子”

    // 拿到最近点的坐标（比如(5.2, 6.1)），作为“挪步”的起点
    Vector2D lastPt = path_.Get_Point(tLookahead);

    // -------------- 对应第2步：小步慢挪，直到累计距离够前视距离 --------------
    // 条件：1. 编号t没到终点（<1.0）；2. 累计距离还没到前视距离（<0.4m）
    while (tLookahead < 1.0f && accumulatedDist < m_lookaheadDist)
    {
        // 1. 往前挪一小步：t增加0.005（比如0.3→0.305）
        float nextT = tLookahead + step;
        // 防止挪超终点：如果nextT>1.0，就改成1.0（不能超出曲线）
        if (nextT > 1.0f)
        {
            nextT = 1.0f;
        }

        // 2. 拿到这一步挪到的点的坐标（比如t=0.305对应的曲线点(5.22, 6.11)）
        Vector2D nextPt = path_.Get_Point(nextT);

        // 3. 计算这一步走了多远（比如从(5.2,6.1)到(5.22,6.11)，距离≈0.022m）
        float distStep = (nextPt - lastPt).magnitude();

        // 4. 累计距离：把这一步的距离加进去（比如0+0.022=0.022m）
        accumulatedDist += distStep;

        // 5. 更新：准备下一步挪步（把当前点当起点，当前t当下一步的基础）
        tLookahead = nextT; // 编号更新
        lastPt = nextPt;    // 起点更新为(5.22,6.11)
    }

    // -------------- 对应第3步：如果到终点了，直接用终点当前视点 --------------
    if (tLookahead >= 1.0f)
    {
        lastPt = path_.Get_Point(1.0f); // 拿曲线终点坐标
    }

    // -------------- 返回前视点坐标 --------------
    return lastPt;
}

void OmniChassis_Setup::KFS_Selection_Planning(void)
{
    int cho = 0;

    Point2D temp;
    temp.x = robot_pos_.x;
    temp.y = robot_pos_.y;
    if (robot_pos_.x < -0.5f || robot_pos_.x > 5.2f || robot_pos_.y < -0.5f || robot_pos_.y > 8.3f)
    {
        flag_run = 0;
        cho = 0;
    }
    else if (robot_pos_.y < 2.0f)
    {
        cho = 1;
    }
    else
    {
        //cho = 2;
    }

    if (cho == 1)
    {
        KFS_result_ = MF_AutoCtrler::PathNodeResult_calc(temp, KFS, 0, 26);
        int point_sum = MF_AutoCtrler::BFS_GetPath(KFS_result_.entranceMap, KFS_result_.bestBMF1, path_point_, 20);
        int index = 0;
        for (int i = 0; i < point_sum; i++)
        {
            if (path_point_[i] == 1 || path_point_[i] == 5 || path_point_[i] == 30 || path_point_[i] == 26 )
            {
                path_key_point_[index] = path_point_[i];
                index++;
            }
        }
        int KFS_next_index=index;
        
        point_sum = MF_AutoCtrler::BFS_GetPath(KFS_result_.bestBMF1, KFS_result_.exitMap, path_point_, 20);

        for (int i = 0; i < point_sum; i++)
        {
            if (path_point_[i] == 1 || path_point_[i] == 5 || path_point_[i] == 30 || path_point_[i] == 26 || path_point_[i] == KFS_result_.exitMap)
            {
                path_key_point_[index] = path_point_[i];
                index++;
            }
        }

        // 修改车子朝向
        if (abs(path_key_point_[KFS_next_index] - KFS_result_.bestBMF1) < 5.0f)
        {
            if (KFS_result_.bestBMF1 < 10.0f)
            {
                target_yaw_ = 90.0f;
            }
            else
            {
                target_yaw_ = -90.0f;
            }
        }
        else
        {
            if (KFS_result_.bestBMF1 == 21 || KFS_result_.bestBMF1 == 16 || KFS_result_.bestBMF1 == 11 || KFS_result_.bestBMF1 == 6)
            {
                target_yaw_ = 180.0f;
            }
            else
            {
                target_yaw_ = 0.0f;
            }
        }

        path_line_.plan_reset();
        path_line_.Reset();
        path_line_.Add_Start_Point(Vector2D{robot_pos_.x, robot_pos_.y}, path_param_);
        for (int i = 0; i < index; i++)
        {
            if (i == (index - 1))
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(path_key_point_[i]);
                path_line_.Add_End_Point(Vector2D{temp_vector.x - 0.5f, temp_vector.y - 0.5f});
            }
            else
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(path_key_point_[i]);
                path_line_.Add_Point(Vector2D{temp_vector.x - 0.5f, temp_vector.y - 0.5f});
            }
        }
    }
    else if (cho == 2)
    {

        KFS_result_ = MF_AutoCtrler::PathNodeResult_calc(Point2D{temp.x, 1.2f, 0.0f}, KFS, 0, 26);
        temp.x += 0.5f;
        temp.y += 0.5f;
        point_map = MF_AutoCtrler::GetMapNumFromPos(temp);
        // 将车子开到空旷地带的路口
        int point_sum = MF_AutoCtrler::BFS_GetPath(point_map, KFS_result_.bestBMF1, path_point_, 20);
        int index = 0;
        for (int i = 0; i < point_sum; i++)
        {
            if (path_point_[i] == 1 || path_point_[i] == 5 || path_point_[i] == 30 || path_point_[i] == 26 || path_point_[i] == KFS_result_.bestBMF1)
            {
                path_key_point_[index] = path_point_[i];
                index++;
            }
        }
        // 将车子开到kfs前
        //        point_sum = MF_AutoCtrler::BFS_GetPath(point_map, KFS_result_.bestBMF1, path_point_, 20);
        //        for (int i = 0; i < point_sum; i++)
        //        {
        //            if (path_point_[i] == 1 || path_point_[i] == 5 || path_point_[i] == 30 || path_point_[i] == 26 || path_point_[i] == KFS_result_.bestBMF1)
        //            {
        //                path_key_point_[index] = path_point_[i];
        //                index++;
        //            }
        //        }
        // 将车子开到斜坡前
        point_sum = MF_AutoCtrler::BFS_GetPath(KFS_result_.bestBMF1, KFS_result_.exitMap, path_point_, 20);
        for (int i = 0; i < point_sum; i++)
        {
            if (path_point_[i] == 1 || path_point_[i] == 5 || path_point_[i] == 30 || path_point_[i] == 26 || path_point_[i] == KFS_result_.exitMap)
            {
                path_key_point_[index] = path_point_[i];
                index++;
            }
        }

        // 修改车子朝向
        if (abs(path_key_point_[0] - KFS_result_.bestBMF1) < 5.0f)
        {
            if (KFS_result_.bestBMF1 < 10.0f)
            {
                target_yaw_ = 90.0f;
            }
            else
            {
                target_yaw_ = -90.0f;
            }
        }
        else
        {
            if (KFS_result_.bestBMF1 == 21 || KFS_result_.bestBMF1 == 16 || KFS_result_.bestBMF1 == 11 || KFS_result_.bestBMF1 == 6)
            {
                target_yaw_ = 180.0f;
            }
            else
            {
                target_yaw_ = 0.0f;
            }
        }

        path_line_.plan_reset();
        path_line_.Reset();
        path_line_.Add_Start_Point(Vector2D{robot_pos_.x, robot_pos_.y}, path_param_);
        for (int i = 0; i < index; i++)
        {
            if (i == (index - 1))
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(path_key_point_[i]);
                path_line_.Add_End_Point(Vector2D{temp_vector.x - 0.5f, temp_vector.y - 0.5f});
            }
            else
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(path_key_point_[i]);
                path_line_.Add_Point(Vector2D{temp_vector.x - 0.5f, temp_vector.y - 0.5f});
            }
        }
    }
}

void OmniChassis_Setup::Path_correction(void)
{
    // 获取曲线（带保护）
    curve = path_line_.get_bezier_curve();

    //pathEnd = curve.Get_Point(1.0f);空语句，不知为什么有
    
    // 1. 找最近点+t值：获取路径上距离当前位置最近的点及其参数 tNearest
    nearestPt = GetPathNearestPoint(curve, robot_pos_, tNearest);

    // ======== 终点纠偏（新架构下平滑退化为终点位置吸附）========
    if (tNearest > 0.95f || path_line_.Is_End() == false)
    {
        Vector2D endPt = curve.Get_End_point();
        // 终点段把前馈参考点切换为终点坐标，差分会自然收敛到 0。
        ff_ref_point_ = endPt;
        // 如果曲线未初始化（例如空曲线），不进行操作
//        if (endPt.magnitude() < 0.0001f && curve.Get_Start_point().magnitude() < 0.0001f)
//        {
//            speed = planspeed; // 保持原有速度
//            return;
//        }
        if (curve.Get_len()<0.0001f)
        {
            corrVelocity = {0.0f, 0.0f};
            return;
        }

        corrVelocity.x = pid_pos_x.pid_calc(endPt.x, robot_pos_.x);
        corrVelocity.y = pid_pos_y.pid_calc(endPt.y, robot_pos_.y);

        // 限制最大纠偏速度，防止终点抖动
        float max_corr = 0.5f;
        if (corrVelocity.magnitude() > max_corr)
        {
            corrVelocity = corrVelocity.normalize() * max_corr;
        }
        return;
    }

    // ======== 动态兔子追踪 (2D Cartesian PID) ========
    // 2. 寻找前视点作为我们追踪的“虚拟兔子”
    lookaheadPt = FindLookaheadPoint(curve, tNearest, tLookahead);
    // 非终点阶段前馈参考点使用前视点。
    ff_ref_point_ = lookaheadPt;
    
    //lookaheadTangent = curve.Get_Tangent_Vector(tLookahead); // 留作状态观测前视点的切线方向

    // 3. 在绝对世界坐标系下，独立计算X轴和Y轴的纠偏向速度
    // 将不再计算切法向，直接基于XY差值PID
    corrVelocity.x = pid_pos_x.pid_calc(lookaheadPt.x, robot_pos_.x);
    corrVelocity.y = pid_pos_y.pid_calc(lookaheadPt.y, robot_pos_.y);

    // 4. (可选) 限制最大动态纠偏速度
    float dynamic_max_corr = 0.8f;
    if (corrVelocity.magnitude() > dynamic_max_corr)
    {
        corrVelocity = corrVelocity.normalize() * dynamic_max_corr;
    }
}

void OmniChassis_Setup::Clamping_Bar_Selection_Planning(void)
{
    target_yaw_ = 0.0f;
    path_line_.plan_reset();
    path_line_.Reset();
    path_line_.Add_Start_Point(Vector2D{robot_pos_.x, robot_pos_.y}, path_param_1);
//   path_line_.Add_Point(Vector2D{1.8f, 0.8f});
//   path_line_.Add_End_Point(Clamping_Bar_Selection_pos_);
     path_line_.Add_End_Point(Vector2D{3.92f, 1.38f});
}
