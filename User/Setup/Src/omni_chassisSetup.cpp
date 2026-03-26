#include "omni_chassisSetup.h"

// Path_line path_line_;
// Speedplanner_1D_Param_Config path_param({.maxAcc = 3.0f, .maxDec = 3.0f, .maxJerk = 4.0f, .maxSpeed = 0.5f, .initialSpeed = 0.05f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f});
// Path_line path_line_(path_param);

#if debug_ladar

int last_cout_ladar_data = -1;

#endif

uint32_t chassisstackHighWaterMark = 0;
extern Chassis chassis;

void OmniChassis_Setup::ResetAutoControlStates(void)
{
    // 1) 阻尼项使用上一时刻 v_robot，退出自动流程后必须清零，避免“历史速度”带入下一次任务。
    v_robot_last_cmd_ = {0.0f, 0.0f};

    // 2) 前馈差分状态一并复位，避免参考点跳变时出现首帧尖峰。
    ff_diff_inited_ = false;
    ff_ref_point_last_ = {0.0f, 0.0f};
    ff_velocity_lpf_ = {0.0f, 0.0f};
    // ff_last_tick_ms_ = 0;
}

Vector2D OmniChassis_Setup::ComputeLookaheadDiffFeedforward(bool near_end)
{
    // 使用 RTOS tick 估计离散 dt（单位秒）；该方法在嵌入式任务循环中稳定且开销小。
    // uint32_t now_tick_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    Vector2D v_ff_raw = {0.0f, 0.0f};

    // 首次进入或状态复位后，不做差分，先对齐历史参考点。
    if (!ff_diff_inited_)
    {
        ff_diff_inited_ = true;
        ff_ref_point_last_ = ff_ref_point_;
        // ff_last_tick_ms_ = now_tick_ms;
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
    // ff_last_tick_ms_ = now_tick_ms;

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
	
    yaw = Locate_Setup::getInstance()->get_yaw_from_position();
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
    ladar_data_ = Locate_Setup::getInstance()->get_RobotPos_inWorld();
    robot_pos_.x = ladar_data_.x;
    robot_pos_.y = ladar_data_.y;
    // robot_pos_.x += original_point_.x;
    // robot_pos_.y += original_point_.y;

    // Acc_target_yaw_ = Acc_yaw_.plan(target_yaw_);


    switch (chassis_status_)
    {
    case CHASSIS_MANUAL_CONTROL_A:
    {
        float target_vel_x = 0.0f;
        float target_vel_y = 0.0f;
        float target_omega_z = 0.0f;

        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_vel_x = airjoy_data_.left_x * 3 * this->is_chassis_reverse_;
        else
            target_vel_x = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_vel_y = airjoy_data_.left_y * 3 * this->is_chassis_reverse_;
        else
            target_vel_y = 0.0f;

        if (_tool_Abs(airjoy_data_.right_x) > 0.05f)
            target_omega_z = airjoy_data_.right_x * 6;
        else
            target_omega_z = 0.0f;

        chassis.setTargetWorldSpeedMode(target_vel_x, target_vel_y, target_omega_z);

        break;
    }

    case CHASSIS_MANUAL_CONTROL_B:
    {
        float target_vel_x = 0.0f;
        float target_vel_y = 0.0f;

        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_vel_x = airjoy_data_.left_x * 0.6 * this->is_chassis_reverse_;
        else
            target_vel_x = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_vel_y = airjoy_data_.left_y * 0.6 * this->is_chassis_reverse_;
        else
            target_vel_y = 0.0f;

        chassis.setTargetWorldSpeedLockNowRotZMode(target_vel_x, target_vel_y);

        break;
    }

    case CHASSIS_LOCK_FORWEAPON:
    {
        const float target_yaw_angle = 90.0f;
        const float target_yaw_rad = 90.0f * PI / 180.0f;

        float target_vel_x = 0.0f;
        float target_vel_y = 0.0f;
        float target_omega_z = 0.0f;

        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_vel_x = airjoy_data_.left_x * 3 * this->is_chassis_reverse_;
        else
            target_vel_x = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_vel_y = airjoy_data_.left_y * 3 * this->is_chassis_reverse_;
        else
            target_vel_y = 0.0f;

        target_omega_z = target_yaw_rad;

        chassis.setTargetWorldSpeedLockToRotZMode(target_vel_x, target_vel_y, target_omega_z);

        break;
    }

    case CHASSIS_AUTO_CONTROL_CB:
    {

        this->set_ControlMode(WORLD_SPEED_MODE);
        if (flag == 1)
        {
            flag_reset();
            flag = 0;
            flag_run = 1;
            Clamping_Bar_Selection_Planning();
            WeaponSage_END =0;
        }
        if (flag_run == 1)
        {
            if (path_line_.Is_End() == true)
            {
                // 获取曲线（带保护）
                curve = path_line_.get_bezier_curve();

                if (Clamping_Bar_Selection_pos_.x == curve.Get_End_point().x && Clamping_Bar_Selection_pos_.y == curve.Get_End_point().y)
                {
                    target_yaw_ = -90.0f;
                }
                // 5. 规划速度+叠加纠偏速度：计算路径规划的前进速度（切向速度）
                planspeed = path_line_.plan(robot_pos_);
                Path_correction();
                bool near_end = (tNearest > 0.95f);
                Vector2D v_ff = ComputeLookaheadDiffFeedforward(near_end);
                speed = ComposeRobotVelocity(corrVelocity, v_ff, near_end);
                target_chassis_twist_.vx = speed.x;
                target_chassis_twist_.vy = speed.y;
            }
            else
            {
                flag = 0;
                flag_run = 0;
                speed.x = 0.0f;
                speed.y = 0.0f;
                planspeed.x = 0.0f;
                planspeed.y = 0.0f;
                path_line_.plan_reset();
                path_line_.Reset();
                ResetAutoControlStates();
                target_chassis_twist_.vx = speed.x;
                target_chassis_twist_.vy = speed.y;
                WeaponSage_END = 1;
                flag_reset();
            }
        }
        else
        {
            target_yaw_ = yaw;
            target_chassis_twist_.vx = 0.0f;
            target_chassis_twist_.vy = 0.0f;
            ResetAutoControlStates();
        }

        float target_yaw_rad = target_yaw_ * PI / 180.0f;
        chassis.setTargetWorldSpeedLockToRotZMode(target_chassis_twist_.vx,target_chassis_twist_.vy,target_yaw_rad);

        break;
    }

    case CHASSIS_AUTO_CONTROL_KFS:
    {
        this->set_ControlMode(WORLD_SPEED_MODE);
        if (flag == 1)
        {
            flag_reset();
            flag = 0;
            flag_run = 1;
            KFS_Selection_Planning();
        }
        if (flag_run == 1)
        {
            // 获取曲线（带保护）
            curve = path_line_.get_bezier_curve();

            if (path_line_.Is_End() == true)
            {
                // 上方停止点旋转
                if (spin_up_flag == true)
                {
                    if (spin_point_.x == curve.Get_End_point().x && spin_point_.y == curve.Get_End_point().y)
                    {
                        get_spin_flag = true;
                    }
                    else if (get_spin_flag == true)
                    {
                        get_spin_flag = false;
                        // target_yaw_=MF2_target_yaw_;
                        Spin_Start = true;
                    }
                    else if (Spin_Start == true)
                    {
                        if (_tool_Abs(yaw - target_yaw_) < 2.0f)
                        {
                            Spin_Start = false;
                            spin_up_flag = false;
                        }
                    }
                }
                else if (spin_down_flag == true)
                {
                    if (MF1_finish == true)
                    {
                        if (target_yaw_ == 90.0f)
                        {
                            target_yaw_ = MF2_target_yaw_;
                            spin_down_flag = false;
                        }
                        else if (MF1_pos_.x == curve.Get_Start_point().x && MF1_pos_.y == curve.Get_Start_point().y)
                        {
                            get_spin_flag = true;
                        }
                        else if (get_spin_flag == true)
                        {
                            target_yaw_ = MF2_target_yaw_;
                            spin_down_flag = false;
                            get_spin_flag = false;
                        }
                    }
                }

                if (MF1_pos_.x == curve.Get_End_point().x && MF1_pos_.y == curve.Get_End_point().y)
                {
                    MF1_flag = true;
                }
                else if (MF2_pos_.x == curve.Get_End_point().x && MF2_pos_.y == curve.Get_End_point().y)
                {
                    MF2_flag = true;
                }
                else if (MF1_flag == true || MF2_flag == true)
                {
                    MF1_flag = false;
                    MF2_flag = false;
                    Arm_Start = true;
                    MF1_finish = true;
                }

                if (Arm_Start == false && Spin_Start == false)
                {
                    // 5. 规划速度+叠加纠偏速度：计算路径规划的前进速度（切向速度）
                    planspeed = path_line_.plan(robot_pos_);

                    Path_correction();

                    bool near_end = (tNearest > t_deadzone);
                    Vector2D v_ff = ComputeLookaheadDiffFeedforward(near_end);
                    speed = ComposeRobotVelocity(corrVelocity, v_ff, near_end);

                    target_chassis_twist_.vx = speed.x;
                    target_chassis_twist_.vy = speed.y;
                }
                else
                {
                    target_chassis_twist_.vx = 0.0f;
                    target_chassis_twist_.vy = 0.0f;
                }
            }
            else
            {
                flag = 0;
                flag_run = 0;
                speed.x = 0.0f;
                speed.y = 0.0f;
                planspeed.x = 0.0f;
                planspeed.y = 0.0f;
                path_line_.plan_reset();
                path_line_.Reset();
                ResetAutoControlStates();
                target_chassis_twist_.vx = speed.x;
                target_chassis_twist_.vy = speed.y;
                flag_reset();
            }
        }
        else
        {
            target_yaw_ = yaw;
            target_chassis_twist_.vx = 0.0f;
            target_chassis_twist_.vy = 0.0f;
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
        chassis.setWheelTorqueFreeMode();
        
        break;
    }

    case CHASSIS_MANUAL_CONTROL_C:
    {
        target_chassis_twist_.yaw_rate = 0.0f;

        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_chassis_twist_.vx = airjoy_data_.left_x * 6 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vx = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_chassis_twist_.vy = airjoy_data_.left_y * 6 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vy = 0.0f;

        chassis.setTargetWorldSpeedMode(target_chassis_twist_.vx,target_chassis_twist_.vy,target_chassis_twist_.yaw_rate);

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


    Point2D fk_speed;
    fk_speed.x = chassis.getTargetWorldVelX();
    fk_speed.y = chassis.getTargetWorldVelY();

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
    // 单位转换
    robot_point_.x = robot_pos_.x;
    robot_point_.y = robot_pos_.y;
    // 计算理想的KFS路径
    KFS_KeyPoint_ = MF_AutoCtrler::PathInformation_calc(robot_point_, MF1, MF2);
    // 判断MF1的车子朝向
    MF1_Point_ = KFS_KeyPoint_.mustPastMap[KFS_KeyPoint_.Index_MFroad[0]];
    MF2_Point_ = KFS_KeyPoint_.mustPastMap[KFS_KeyPoint_.Index_MFroad[1]];

    if (abs(KFS_KeyPoint_.mustPastMap[KFS_KeyPoint_.Index_MFroad[0] + 1] - MF1_Point_) < 5.0f)
    {
        if (MF1_Point_ < 10.0f)
        {
            target_yaw_ = -90.0f;
        }
        else
        {
            target_yaw_ = 90.0f;
        }
    }
    else
    {
        if (MF1_Point_ == 21 || MF1_Point_ == 16 || MF1_Point_ == 11 || MF1_Point_ == 6)
        {
            target_yaw_ = 180.0f;
        }
        else if (MF1_Point_ == 25 || MF1_Point_ == 20 || MF1_Point_ == 15 || MF1_Point_ == 10)
        {
            target_yaw_ = 0.0f;
        }
    }

    // 判断MF2的车子朝向
    if (abs(KFS_KeyPoint_.mustPastMap[KFS_KeyPoint_.Index_MFroad[1] + 1] - MF2_Point_) < 5.0f)
    {
        if (MF2_Point_ < 10.0f)
        {
            MF2_target_yaw_ = -90.0f;
        }
        else
        {
            MF2_target_yaw_ = 90.0f;
        }
    }
    else
    {
        if (MF2_Point_ == 21 || MF2_Point_ == 16 || MF2_Point_ == 11 || MF2_Point_ == 6)
        {
            MF2_target_yaw_ = 180.0f;
        }
        else if (MF2_Point_ == 25 || MF2_Point_ == 20 || MF2_Point_ == 15 || MF2_Point_ == 10)
        {
            MF2_target_yaw_ = 0.0f;
        }
    }

    // 判断是否需要转向
    if (target_yaw_ == MF2_target_yaw_ || MF2==0.0f)
    {
        spin_flag = false;
    }
    else
    {
        spin_flag = true;
    }

    // 计算出口索引
    index_exit = 0;
    while (index_exit < 12 && KFS_KeyPoint_.mustPastMap[index_exit] != 0)
    {
        index_exit++;
    }

    // // 在“经过 MF1 后的下一个路点”尝试插入转向过渡点。
    // int mf1_route_idx = -1;
    // for (int i = 0; i < index_exit; i++)
    // {
    //     int map_idx = KFS_KeyPoint_.Index_MFroad[i];
    //     if (KFS_KeyPoint_.mustPastMap[map_idx] == MF1_Point_)
    //     {
    //         mf1_route_idx = i;
    //         break;
    //     }
    // }
    // const int spin_insert_route_idx = (mf1_route_idx >= 0) ? (mf1_route_idx + 1) : -1;

    // 写入路径点坐标
    path_line_.plan_reset();
    path_line_.Reset();
    path_line_.Add_Start_Point(robot_pos_, path_param_KFS_);

    MF1_pos_ = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[KFS_KeyPoint_.Index_MFroad[0]]);
    MF2_pos_ = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[KFS_KeyPoint_.Index_MFroad[1]]);

    if (spin_flag == false)
    {
        for (int i = 0; i < index_exit; i++)
        {
            if (i == (index_exit - 1))
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                path_line_.Add_End_Point(temp_vector);
            }
            else
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                path_line_.Add_Point(temp_vector);
            }
        }
    }
    else if (spin_flag == true)
    {
        if (target_yaw_ == 90.0f) // 下
        {
            for (int i = 0; i < index_exit; i++)
            {
                if (i == (index_exit - 1))
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_End_Point(temp_vector);
                }
                else if (i == (KFS_KeyPoint_.Index_MFroad[0] + 1))
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    temp_vector.y = temp_vector.y + spin_skew_;
                    path_line_.Add_Point(temp_vector);
                    spin_down_flag = true;
                }
                else
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_Point(temp_vector);
                }
            }
        }
        else if (target_yaw_ == -90.0f) // 上
        {
            for (int i = 0; i < index_exit; i++)
            {
                if (i == (index_exit - 1))
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_End_Point(temp_vector);
                }
                else if (i == (KFS_KeyPoint_.Index_MFroad[0] + 1))
                {
                    path_line_.Add_Point(spin_point_);
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_Point(temp_vector);
                    spin_up_flag = true;
                }
                else
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_Point(temp_vector);
                }
            }
        }
        else // 两边
        {
            for (int i = 0; i < index_exit; i++)
            {
                if (i == (index_exit - 1))
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_End_Point(temp_vector);
                }
                else if (i == (KFS_KeyPoint_.Index_MFroad[0] + 1))
                {
                    if (KFS_KeyPoint_.mustPastMap[i] == 26 || KFS_KeyPoint_.mustPastMap[i] == 30) // 上
                    {
                        path_line_.Add_Point(spin_point_);
                        Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                        path_line_.Add_Point(temp_vector);
                        spin_up_flag = true;
                    }
                    else if (KFS_KeyPoint_.mustPastMap[i] == 1 || KFS_KeyPoint_.mustPastMap[i] == 5) // 下
                    {
                        Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                        temp_vector.y = temp_vector.y + spin_skew_;
                        path_line_.Add_Point(temp_vector);
                        spin_down_flag = true;
                    }
                }
                else
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_Point(temp_vector);
                }
            }
        }
    }
}

void OmniChassis_Setup::Path_correction(void)
{

    // pathEnd = curve.Get_Point(1.0f);空语句，不知为什么有

    // 1. 找最近点+t值：获取路径上距离当前位置最近的点及其参数 tNearest
    nearestPt = GetPathNearestPoint(curve, robot_pos_, tNearest);

    // ======== 终点纠偏（新架构下平滑退化为终点位置吸附）========
    if (tNearest > t_deadzone || path_line_.Is_End() == false)
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
        if (curve.Get_len() < 0.0001f)
        {
            corrVelocity = {0.0f, 0.0f};
            return;
        }

        corrVelocity.x = pid_pos_x.pid_calc(endPt.x, robot_pos_.x);
        corrVelocity.y = pid_pos_y.pid_calc(endPt.y, robot_pos_.y);

        // 限制最大纠偏速度，防止终点抖动

        if (corrVelocity.magnitude() > max_corr_end_)
        {
            corrVelocity = corrVelocity.normalize() * max_corr_end_;
        }
        return;
    }

    // ======== 动态兔子追踪 (2D Cartesian PID) ========
    // 2. 寻找前视点作为我们追踪的“虚拟兔子”
    lookaheadPt = FindLookaheadPoint(curve, tNearest, tLookahead);
    // 非终点阶段前馈参考点使用前视点。
    ff_ref_point_ = lookaheadPt;

    // lookaheadTangent = curve.Get_Tangent_Vector(tLookahead); // 留作状态观测前视点的切线方向

    // 3. 在绝对世界坐标系下，独立计算X轴和Y轴的纠偏向速度
    // 将不再计算切法向，直接基于XY差值PID
    corrVelocity.x = pid_pos_x.pid_calc(lookaheadPt.x, robot_pos_.x);
    corrVelocity.y = pid_pos_y.pid_calc(lookaheadPt.y, robot_pos_.y);

    // 4. (可选) 限制最大动态纠偏速度
    corrVelocity = corrVelocity.normalize();
}

void OmniChassis_Setup::Clamping_Bar_Selection_Planning(void)
{
    target_yaw_ = 0.0f;
    path_line_.plan_reset();
    path_line_.Reset();
    path_line_.Add_Start_Point(robot_pos_, path_param_CB_);
    //   path_line_.Add_Point(Vector2D{1.8f, 0.8f});
    //   path_line_.Add_End_Point(Clamping_Bar_Selection_pos_);
    path_line_.Add_End_Point(Vector2D{4.31f, 1.88f});
}
void OmniChassis_Setup::flag_reset(void)
{
    MF1_flag = false;
    MF2_flag = false;
    spin_flag = false;
    spin_up_flag = false;
    spin_down_flag = false;
    MF1_finish = false;
    get_spin_flag = false;
    Spin_Start = false;
}