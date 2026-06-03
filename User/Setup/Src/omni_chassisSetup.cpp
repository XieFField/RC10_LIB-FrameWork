#include "omni_chassisSetup.h"

void OmniChassis_Setup::Path_CB_check(void)
{
    if (CB_point.Clamping_Bar_Selection_pos_.x == curve.Get_End_point().x && CB_point.Clamping_Bar_Selection_pos_.y == curve.Get_End_point().y)
    {
        CB_flag.Selection_flag = true;
    }
    else if (CB_flag.Selection_flag == true)
    {
        CB_flag.Selection_flag = false;
        pid_dead_flag=false;
        WeaponSage_Start = true;
    }

    if (CB_point.Clamping_Bar_Retreat_pos_.x == curve.Get_End_point().x && CB_point.Clamping_Bar_Retreat_pos_.y == curve.Get_End_point().y)
    {
        CB_flag.Retreat_flag = true;
    }
    else if (CB_flag.Retreat_flag == true)
    {
        CB_flag.Retreat_flag = false;
        pid_dead_flag=false;
        WeaponSage_End = true;
    }
}
void OmniChassis_Setup::Path_spin_check(void)
{
    // KFS拾取判断MF1
    if (KFS_point.MF1_pos_.x == curve.Get_End_point().x && KFS_point.MF1_pos_.y == curve.Get_End_point().y)
    {
        KFS_flag.MF1_flag = true;
    }
    else if (KFS_flag.MF1_flag == true)
    {
        KFS_flag.MF1_flag = false;
        pid_dead_flag=false;
        Arm_Start = true;
        KFS_flag.MF1_finish = true;
    }

    // KFS拾取判断MF2
    if (KFS_point.MF2_pos_.x == curve.Get_End_point().x && KFS_point.MF2_pos_.y == curve.Get_End_point().y)
    {
        KFS_flag.MF2_flag = true;
    }
    else if (KFS_flag.MF2_flag == true)
    {
        KFS_flag.MF2_flag = false;
        pid_dead_flag=false;
        Arm_Start = true;
    }

    // 上方停止点旋转
    if (KFS_flag.spin_up_flag == true)
    {
        // 判断旋转条件
        if (KFS_point.spin_point_.x == curve.Get_End_point().x && KFS_point.spin_point_.y == curve.Get_End_point().y)
        {
            KFS_flag.get_spin_flag = true;
        }
        // 开始旋转
        else if (KFS_flag.get_spin_flag == true)
        {
            KFS_flag.get_spin_flag = false;
            // target_yaw_=MF2_target_yaw_;
            KFS_flag.Spin_Start = true;
        }
        // 判断退出
        else if (KFS_flag.Spin_Start == true)
        {
            if (_tool_Abs(yaw - target_yaw_) < 2.0f)
            {
                KFS_flag.Spin_Start = false;
                KFS_flag.spin_up_flag = false;
            }
        }
    }

    if (KFS_flag.spin_down_flag == true) // 下方偏移旋转
    {
        if (KFS_flag.MF1_finish == true)
        {
            // 第一排旋转
            if (target_yaw_ == 90.0f)
            {
                // 延迟旋转
                if (robot_pos_.y <= 2.55f)
                {
                    target_yaw_ = KFS_point.MF2_target_yaw_;
                    KFS_flag.spin_down_flag = false;
                }
            }
            // 两侧旋转判断
            else if (KFS_point.MF1_pos_.x == curve.Get_Start_point().x && KFS_point.MF1_pos_.y == curve.Get_Start_point().y)
            {
                KFS_flag.get_spin_flag = true;
            }
            // 两侧开始旋转
            else if (KFS_flag.get_spin_flag == true)
            {
                target_yaw_ = KFS_point.MF2_target_yaw_;
                KFS_flag.spin_down_flag = false;
                KFS_flag.get_spin_flag = false;
            }
        }
    }
}

void OmniChassis_Setup::Clamping_Bar_Selection_Planning(void)
{
    // 夹杆流程只规划起点到固定终点的简化路径。
    target_yaw_ = 0.0f;
    path_line_.plan_reset();
    path_line_.Reset();

    //    path_line_.Add_Start_Point(robot_pos_);
    //    path_line_.Add_End_Point(test_point, path_param_CB_);
    // 曲线的测试
    //   path_line_.Add_Start_Point(robot_pos_);
    //   path_line_.Add_Point(Vector2D{robot_pos_.x-0.5f, robot_pos_.y}, path_param_curve_);
    //   path_line_.Add_Point(Vector2D{robot_pos_.x-0.5f-0.63f, robot_pos_.y+0.63f}, Vector2D{robot_pos_.x-0.5f-0.85f, robot_pos_.y-0.22f}, path_param_curve_);
    //   path_line_.Add_End_Point(Vector2D{robot_pos_.x-0.5f-0.63f, robot_pos_.y+0.63f+0.2f}, path_param_end_);

    // 夹杆路径的测试
    path_line_.Add_Start_Point(robot_pos_);
    path_line_.Add_Point(CB_point.CB_Selection_start_point_, path_param.start);
    path_line_.Add_Point(CB_point.Clamping_Bar_Selection_pos_, CB_point.CB_Selection_control_point_, path_param.curve);
    path_line_.Add_Point(CB_point.Clamping_Bar_Retreat_pos_, path_param.end);
    path_line_.Add_End_Point(CB_point.Clamping_Bar_Retreat_pos_, path_param.end);

    Path_end_point = path_line_.Get_End_Point();
}

extern Chassis chassis;

void OmniChassis_Setup::loop()
{
    // 未初始化时不进入控制流程。
    if (!init_flag)
        return;

    float dyaw = Locate_Setup::getInstance()->get_dyaw_from_position();

    yaw = Locate_Setup::getInstance()->get_yaw_from_position();
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
    ladar_data_ = Locate_Setup::getInstance()->get_RobotPos_inWorld();
    robot_pos_.x = ladar_data_.x;
    robot_pos_.y = ladar_data_.y;

    switch (chassis_status_)
    {
    case CHASSIS_MANUAL_CONTROL_A:
    {
        // 模式 A：大速度手动平移 + 角速度控制。
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

        chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, target_vel_x, target_vel_y, target_omega_z);

        break;
    }
    case CHASSIS_MANUAL_CONTROL_B:
    {
        // 模式 B：低速手动平移，锁当前航向。
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

        chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, target_vel_x, target_vel_y);

        break;
    }
    case CHASSIS_AUTO_CONTROL_CB:
    {
        // 夹杆自动流程：触发后执行路径规划、纠偏和速度合成。
        if (flag == 1)
        {
            flag_reset();
            flag = 0;
            flag_run = 1;
            Clamping_Bar_Selection_Planning();
        }
        if (flag_run == 1)
        {
            if (path_line_.Is_End() == false)
            {
                // 获取曲线（带保护）
                curve = path_line_.get_bezier_curve();
                Path_CB_check();
                if (WeaponSage_Start == false && WeaponSage_End == false)
                {
                    planspeed = path_line_.plan(robot_pos_);
                    Path_correction();
                    corrVelocity = V.PID_coefficient * corrVelocity;
                    speed = v_limit();
                    target_chassis_twist_.vx = speed.x;
                    target_chassis_twist_.vy = speed.y;
                }
                else
                {
                    float lock_err = (robot_pos_ - CB_point.Clamping_Bar_Selection_pos_).magnitude();
                    speed = path_lock.pid_calc(0.0f, lock_err) * (robot_pos_ - CB_point.Clamping_Bar_Selection_pos_).normalize();
                    target_chassis_twist_.vx = speed.x;
                    target_chassis_twist_.vy = speed.y;
                    pid_dead_flag=path_lock.get_is_in_dead_zone();
                }
            }
            else
            {
                float lock_err = (robot_pos_ - Path_end_point).magnitude();
                speed = path_lock.pid_calc(0.0f, lock_err) * (robot_pos_ - Path_end_point).normalize();
                target_chassis_twist_.vx = speed.x;
                target_chassis_twist_.vy = speed.y;
            }
            float target_yaw_rad = target_yaw_ * PI / 180.0f;
            chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, target_chassis_twist_.vx, target_chassis_twist_.vy, target_yaw_rad);
        }
        else
        {
            target_chassis_twist_ = {0.0f, 0.0f};
            chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, target_chassis_twist_.vx, target_chassis_twist_.vy);
        }

        break;
    }

    case CHASSIS_AUTO_CONTROL_KFS:
    {
        // KFS 自动流程：路径跟踪 + 旋转点处理 + 机械臂联动。
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

            if (path_line_.Is_End() == false)
            {
                // 旋转点位判断以及KFS的拾取判断
                Path_spin_check();
                if (Arm_Start == false && KFS_flag.Spin_Start == false)
                {
                    // 5. 规划速度+叠加纠偏速度：计算路径规划的前进速度（切向速度）
                    planspeed = path_line_.plan(robot_pos_);
                    Path_correction();
                    corrVelocity = V.PID_coefficient * corrVelocity;
                    speed = v_limit();
                    target_chassis_twist_.vx = speed.x;
                    target_chassis_twist_.vy = speed.y;
                }
                else
                {
                    Vector2D lock_point = curve.Get_Start_point();
                    float lock_err = (robot_pos_ - lock_point).magnitude();
                    speed = path_lock.pid_calc(0.0f, lock_err) * (robot_pos_ - lock_point).normalize();
                    target_chassis_twist_.vx = speed.x;
                    target_chassis_twist_.vy = speed.y;
                    pid_dead_flag=path_lock.get_is_in_dead_zone();
                }
            }
            else
            {
                float lock_err = (robot_pos_ - Path_end_point).magnitude();
                speed = path_lock.pid_calc(0.0f, lock_err) * (robot_pos_ - Path_end_point).normalize();
                target_chassis_twist_.vx = speed.x;
                target_chassis_twist_.vy = speed.y;
            }
            
            
            float target_yaw_rad = target_yaw_ * PI / 180.0f;
            chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, target_chassis_twist_.vx, target_chassis_twist_.vy, target_yaw_rad);
        }
        else
        {
            target_chassis_twist_ = {0.0f, 0.0f};
            chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, target_chassis_twist_.vx, target_chassis_twist_.vy);
        }

        break;
    }

    case CHASSIS_STOP:
    {
#ifdef s_debug
        num++;

        tp_speed_now = TP_1d.plan(tp_pos_now);
        tp_pos_now += tp_speed_now * 0.001f;

        if (num > 2)
        {
            // debug_uart.printf_DMA("%f,%f\n", tp_speed_now, tp_pos_now);
            num = 0;
        }
        if (TP_1d.isFinished() == 1)
        {
            a++;
        }
        else
        {
            a = 0;
        }
        if (a > 1000)
        {
            a = 0;
            tp_pos_now = 0.0f;
            TP_1d.param_reset(Param_1d);
        }

#endif
        chassis.setZeroCurrent();
        break;
    }

    case CHASSIS_MANUAL_CONTROL_C:
    {
        // 模式 C：全向速度控制，角速度固定为 0。
        target_chassis_twist_.yaw_rate = 0.0f;

        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_chassis_twist_.vx = airjoy_data_.left_x * 1 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vx = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_chassis_twist_.vy = airjoy_data_.left_y * 1 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vy = 0.0f;

        chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, target_chassis_twist_.vx, target_chassis_twist_.vy);
//        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
//            target_chassis_twist_.vx = airjoy_data_.left_x * 1 * this->is_chassis_reverse_;
//        else
//            target_chassis_twist_.vx = 0.0f;
//        chassis.setSteerDegAndDriveSpeed(90.0f, target_chassis_twist_.vx);
        break;
    }

    default:
    {
        // 其他模式不下发新命令。
        break;
    }
    }
}

//////////////////////////////////////////       路径纠偏      //////////////////////////////////////////////////////
#define OPIT 0
void OmniChassis_Setup::Path_correction(void)
{
    float tNearest = 0.0f;   // 最近点在贝塞尔曲线上的参数t (0~1)
    float tLookahead = 0.0f; // 前视点在贝塞尔曲线上的参数t (0~1)

    curve.Get_Nearest_Distance(robot_pos_, &tNearest);

    Vector2D nearestPt = curve.Get_Point(tNearest);

    float err_curve = (nearestPt - robot_pos_).magnitude();

    float obj_dis = _tool_Abs((curve.Get_End_point() - robot_pos_).magnitude());

    // ======== 终点纠偏（新架构下平滑退化为终点位置吸附）========
    if (obj_dis < m_lookaheadDist || path_line_.Is_End() == true)
    {
        Vector2D endPt = curve.Get_End_point();

        if (curve.Get_len() < 0.0001f)
        {
            corrVelocity = {0.0f, 0.0f};
            return;
        }
        corrVelocity.x = pid_pos_x.pid_calc(endPt.x, robot_pos_.x);
        corrVelocity.y = pid_pos_y.pid_calc(endPt.y, robot_pos_.y);
        return;
    }

    // ======== 动态兔子追踪 (2D Cartesian PID) ========
    // 2. 寻找前视点作为我们追踪的“虚拟兔子”

    Vector2D lookaheadPt; // 路径上的前视点
    if (curve.Get_Bezier_Order() == FIRST_ORDER_BEZIER)
    {
        m_lookaheadDist = V.m_lookaheadDist_line;
    }
    else
    {
        m_lookaheadDist = V.m_lookaheadDist_curve;
    }

    tLookahead = tNearest; // 前视点的编号，先从最近点的编号开始（比如t=0.3）

#if OPTI
    {
        // 弧长表二分查找：利用 BezierCurve 已有的 distance_list[200]
        // Get_Current_Len 内部做 O(1) 查表+线性插值，二分替代逐点步进
        float current_len = curve.Get_Current_Len(tNearest);
        float target_len  = current_len + m_lookaheadDist;

        if (target_len >= curve.Get_len()) {
            // 前视距离超出曲线总长 → 直接用终点
            tLookahead = 1.0f;
            lookaheadPt = curve.Get_Point(1.0f);
        } else {
            float lo = tNearest, hi = 1.0f;
            for (int i = 0; i < 8; i++) {
                float mid = (lo + hi) * 0.5f;
                if (curve.Get_Current_Len(mid) < target_len)
                    lo = mid;
                else
                    hi = mid;
            }
            tLookahead = hi;
            lookaheadPt = curve.Get_Point(tLookahead);
        }
    }
#else
    {
        // 原步进法
        float accumulatedDist = 0.0f; // 累计挪了多少距离
        float step = 0.01f;           // 步进

        lookaheadPt = curve.Get_Point(tLookahead);
        while (tLookahead < 1.0f && accumulatedDist < m_lookaheadDist)
        {
            float nextT = tLookahead + step;
            if (nextT > 1.0f)
                nextT = 1.0f;

            Vector2D nextPt = curve.Get_Point(nextT);
            float distStep = (nextPt - lookaheadPt).magnitude();
            accumulatedDist += distStep;

            tLookahead = nextT;
            lookaheadPt = nextPt;
        }

        if (tLookahead >= 1.0f)
            lookaheadPt = curve.Get_Point(1.0f);
    }
#endif

    // 3. 在绝对世界坐标系下，独立计算X轴和Y轴的纠偏向速度
    // 将不再计算切法向，直接基于XY差值PID
    corrVelocity.x = pid_pos_x.pid_calc(lookaheadPt.x, robot_pos_.x);
    corrVelocity.y = pid_pos_y.pid_calc(lookaheadPt.y, robot_pos_.y);
}

///////////////////////////////////       KFS路径生成            ////////////////////////////////

void OmniChassis_Setup::KFS_Selection_Planning(void)
{
    int8_t MF1_Point_ = 0; // MF1 对应地图点编号。
    int8_t MF2_Point_ = 0; // MF2 对应地图点编号。

    // 基于当前位置和目标点编号计算整条必经路径。
    // 自动规划接口转换
    Point2D robot_point_ = {robot_pos_.x, robot_pos_.y};
    // 计算理想的KFS路径
    KFS_KeyPoint_ = MF_AutoCtrler::PathInformation_calc(robot_point_, KFS_point.MF1, KFS_point.MF2);
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
            KFS_point.MF2_target_yaw_ = -90.0f;
        }
        else
        {
            KFS_point.MF2_target_yaw_ = 90.0f;
        }
    }
    else
    {
        if (MF2_Point_ == 21 || MF2_Point_ == 16 || MF2_Point_ == 11 || MF2_Point_ == 6)
        {
            KFS_point.MF2_target_yaw_ = 180.0f;
        }
        else if (MF2_Point_ == 25 || MF2_Point_ == 20 || MF2_Point_ == 15 || MF2_Point_ == 10)
        {
            KFS_point.MF2_target_yaw_ = 0.0f;
        }
    }

    // 判断是否需要转向
    if (target_yaw_ == KFS_point.MF2_target_yaw_ || KFS_point.MF2 == 0.0f)
    {
        KFS_flag.spin_flag = false;
    }
    else
    {
        KFS_flag.spin_flag = true;
    }

    // 计算出口索引
    int index_exit = 0; // 当前路径出口索引（有效路径点长度）。
    while (index_exit < 12 && KFS_KeyPoint_.mustPastMap[index_exit] != 0)
    {
        index_exit++;
    }

    // 写入路径点坐标
    path_line_.plan_reset();
    path_line_.Reset();
    path_line_.Add_Start_Point(robot_pos_);

    KFS_point.MF1_pos_ = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[KFS_KeyPoint_.Index_MFroad[0]]);
    if (KFS_point.MF2 != 0)
    {
        KFS_point.MF2_pos_ = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[KFS_KeyPoint_.Index_MFroad[1]]);
    }
    else
    {
        KFS_point.MF2_pos_ = {0.0f, 0.0f};
    }

    if (KFS_flag.spin_flag == false)
    {
        for (int i = 0; i < index_exit; i++)
        {
            if (i == (index_exit - 1))
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                path_line_.Add_End_Point(temp_vector, path_param.KFS);
            }
            else
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                path_line_.Add_Point(temp_vector, path_param.KFS);
            }
        }
    }
    else if (KFS_flag.spin_flag == true)
    {
        if (target_yaw_ == 90.0f) // 下
        {
            for (int i = 0; i < index_exit; i++)
            {
                if (i == (index_exit - 1))
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_End_Point(temp_vector, path_param.KFS);
                }
                else if (i == (KFS_KeyPoint_.Index_MFroad[0] + 1))
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    temp_vector.y = temp_vector.y + KFS_point.spin_skew_;
                    path_line_.Add_Point(temp_vector, path_param.KFS);
                    KFS_flag.spin_down_flag = true;
                }
                else
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_Point(temp_vector, path_param.KFS);
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
                    path_line_.Add_End_Point(temp_vector, path_param.KFS);
                }
                else if (i == (KFS_KeyPoint_.Index_MFroad[0] + 1))
                {
                    path_line_.Add_Point(KFS_point.spin_point_, path_param.KFS);
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_Point(temp_vector, path_param.KFS);
                    KFS_flag.spin_up_flag = true;
                }
                else
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_Point(temp_vector, path_param.KFS);
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
                    path_line_.Add_End_Point(temp_vector, path_param.KFS);
                }
                else if (i == (KFS_KeyPoint_.Index_MFroad[0] + 1))
                {
                    if (KFS_KeyPoint_.mustPastMap[i] == 26 || KFS_KeyPoint_.mustPastMap[i] == 30) // 上
                    {
                        path_line_.Add_Point(KFS_point.spin_point_, path_param.KFS);
                        Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                        path_line_.Add_Point(temp_vector, path_param.KFS);
                        KFS_flag.spin_up_flag = true;
                    }
                    else if (KFS_KeyPoint_.mustPastMap[i] == 1 || KFS_KeyPoint_.mustPastMap[i] == 5) // 下
                    {
                        Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                        temp_vector.y = temp_vector.y + KFS_point.spin_skew_;
                        path_line_.Add_Point(temp_vector, path_param.KFS);
                        KFS_flag.spin_down_flag = true;
                    }
                }
                else
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_Point(temp_vector, path_param.KFS);
                }
            }
        }
    }
    Path_end_point = path_line_.Get_End_Point();
}

// Vector2D OmniChassis_Setup::FindLookaheadPoint(BezierCurve &path_, float tNearest, float &tLookahead)
//{
//     tLookahead = tNearest;        // 前视点的编号，先从最近点的编号开始（比如t=0.3）
//     float accumulatedDist = 0.0f; // 累计挪了多少距离（刚开始是0）
//     float step = 0.01f;           // 每次挪的“小步子”

//    Vector2D lastPt = path_.Get_Point(tLookahead);

//    while (tLookahead < 1.0f && accumulatedDist < m_lookaheadDist)
//    {
//        // 1. 往前挪一小步：t增加0.005（比如0.3→0.305）
//        float nextT = tLookahead + step;
//        // 防止挪超终点：如果nextT>1.0，就改成1.0（不能超出曲线）
//        if (nextT > 1.0f)
//        {
//            nextT = 1.0f;
//        }

//        // 2. 拿到这一步挪到的点的坐标（比如t=0.305对应的曲线点(5.22, 6.11)）
//        Vector2D nextPt = path_.Get_Point(nextT);

//        // 3. 计算这一步走了多远（比如从(5.2,6.1)到(5.22,6.11)，距离≈0.022m）
//        float distStep = (nextPt - lastPt).magnitude();

//        // 4. 累计距离：把这一步的距离加进去（比如0+0.022=0.022m）
//        accumulatedDist += distStep;

//        // 5. 更新：准备下一步挪步（把当前点当起点，当前t当下一步的基础）
//        tLookahead = nextT; // 编号更新
//        lastPt = nextPt;    // 起点更新为(5.22,6.11)
//    }

//    if (tLookahead >= 1.0f)
//    {
//        lastPt = path_.Get_Point(1.0f); // 拿曲线终点坐标
//    }

//    return lastPt;
//}

void OmniChassis_Setup::flag_reset(void)
{
    // 统一清空自动流程的阶段标志与旋转状态。
    WeaponSage_Start = false;
    WeaponSage_End = false;
    Arm_Start = false;
    KFS_flag.MF1_flag = false;
    KFS_flag.MF2_flag = false;
    KFS_flag.spin_flag = false;
    KFS_flag.spin_up_flag = false;
    KFS_flag.spin_down_flag = false;
    KFS_flag.MF1_finish = false;
    KFS_flag.get_spin_flag = false;
    KFS_flag.Spin_Start = false;
    CB_flag.Selection_flag = false;
    CB_flag.Retreat_flag = false;
}

Vector2D OmniChassis_Setup::v_limit(void)
{
    //    // 使用单位向量做正交分解，避免 |normal|² 缩放
    //    Vector2D normal = curve.Get_End_point() - curve.Get_Start_point();
    //    Vector2D tangent_dir = normal.normalize();
    //    Vector2D normal_dir = tangent_dir.perpendicular();

    //    // 前馈速度：仅受 planspeed 幅度限制
    //    Vector2D v_ff = planspeed * FF_coefficient;
    //    if (v_ff.magnitude() > planspeed.magnitude())
    //        v_ff = v_ff.normalize() * planspeed.magnitude();

    //    // 切向 = 前馈 + PID纠偏沿切向分量（PID不限幅，终点 planspeed=0 时保留切向纠偏）
    //    Vector2D v_tangent = v_ff + corrVelocity.project_onto(tangent_dir);

    //    // 法向 = PID纠偏沿法向分量（限幅防止侧向过冲）
    //    Vector2D v_normal = corrVelocity.project_onto(normal_dir);
    //    if (v_normal.magnitude() > v_normal_max)
    //        v_normal = v_normal.normalize() * v_normal_max;

    //    Vector2D v = v_tangent + v_normal;

    // 使用单位向量做正交分解，避免 |normal|² 缩放
    Vector2D tangent = path_line_.Get_Tangent_Vector();
    Vector2D tangent_dir = tangent.normalize();
    Vector2D normal_dir = tangent_dir.perpendicular();

    // 切向 = 前馈 + PID纠偏沿切向分量（PID不限幅，终点 planspeed=0 时保留切向纠偏）
    Vector2D v_tangent = planspeed * V.FF_coefficient + corrVelocity.project_onto(tangent_dir);

    if (v_tangent.magnitude() > planspeed.magnitude())
        v_tangent = v_tangent.normalize() * planspeed.magnitude();

    // 法向 = PID纠偏沿法向分量（限幅防止侧向过冲）
    Vector2D v_normal = corrVelocity.project_onto(normal_dir);
    if (v_normal.magnitude() > V.v_normal_max)
        v_normal = v_normal.normalize() * V.v_normal_max;

    Vector2D v_end = v_tangent + v_normal;

    num++;
    if (num > 5)
    {
        debug_uart.printf_DMA("%f,%f,%f\n", v_end.magnitude(), v_tangent.magnitude(), v_normal.magnitude());
        num = 0;
    }
    return v_end;
}
