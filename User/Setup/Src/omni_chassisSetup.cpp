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
        pid_dead_flag = false;
        WeaponSage_Start = true;
    }

    if (CB_point.Clamping_Bar_Retreat_pos_.x == curve.Get_End_point().x && CB_point.Clamping_Bar_Retreat_pos_.y == curve.Get_End_point().y)
    {
        CB_flag.Retreat_flag = true;
        target_yaw_ = 90.0f;
    }
    else if (CB_flag.Retreat_flag == true)
    {
        CB_flag.Retreat_flag = false;
        pid_dead_flag = false;
        WeaponSage_End = true;
    }
}
void OmniChassis_Setup::Path_KFS_check(void)
{
    // KFS拾取判断MF1
    if (KFS_point.MF1_pos_.x == curve.Get_End_point().x && KFS_point.MF1_pos_.y == curve.Get_End_point().y)
    {
        KFS_flag.MF1_flag = true;
    }
    else if (KFS_flag.MF1_flag == true)
    {
        KFS_flag.MF1_flag = false;
        pid_dead_flag = false;
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
        pid_dead_flag = false;
        Arm_Start = true;
        KFS_flag.MF2_finish = true;
    }

    // KFS拾取判断MF3
    if (KFS_point.MF3_pos_.x == curve.Get_End_point().x && KFS_point.MF3_pos_.y == curve.Get_End_point().y)
    {
        KFS_flag.MF3_flag = true;
    }
    else if (KFS_flag.MF3_flag == true)
    {
        KFS_flag.MF3_flag = false;
        pid_dead_flag = false;
        Arm_Start = true;
    }

    if (KFS_flag.spin_flag == true && KFS_flag.MF1_finish == true)
    {
        // 第一排和最后一排旋转
        if (target_yaw_ == -90.0f || target_yaw_ == 90.0f)
        {
            target_yaw_ = KFS_point.MF2_target_yaw_;
            KFS_flag.spin_flag = false;
        }
        // 两侧旋转判断
        else if (KFS_point.spin_pos.x == curve.Get_End_point().x && KFS_point.spin_pos.y == curve.Get_End_point().y)
        {
            KFS_flag.get_spin_flag = true;
        }
        // 两侧开始旋转
        else if (KFS_flag.get_spin_flag == true)
        {
            target_yaw_ = KFS_point.MF2_target_yaw_;
            KFS_flag.spin_flag = false;
            KFS_flag.get_spin_flag = false;
        }
    }

    if (KFS_flag.spin_flag_2 == true && KFS_flag.spin_flag == false && KFS_flag.MF2_finish == true)
    {
        // 第一排旋转
        if (target_yaw_ == -90.0f || target_yaw_ == 90.0f)
        {
            target_yaw_ = KFS_point.MF3_target_yaw_;
            KFS_flag.spin_flag_2 = false;
        }
        // 两侧旋转判断
        else if (KFS_point.spin_pos_2.x == curve.Get_End_point().x && KFS_point.spin_pos_2.y == curve.Get_End_point().y)
        {
            KFS_flag.get_spin_flag = true;
        }
        // 两侧开始旋转
        else if (KFS_flag.get_spin_flag == true)
        {
            target_yaw_ = KFS_point.MF3_target_yaw_;
            KFS_flag.spin_flag_2 = false;
            KFS_flag.get_spin_flag = false;
        }
    }

    /*
        //-------------------------------          MF1              -------------------------------//
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
                target_yaw_ = KFS_point.MF2_target_yaw_;
                Spin_Start = true;
            }
            // 判断退出
            else if (Spin_Start == true)
            {
                if (_tool_Abs(yaw - target_yaw_) < 2.0f)
                {
                    Spin_Start = false;
                    KFS_flag.spin_up_flag = false;
                }
            }
        }

        if (KFS_flag.spin_down_flag == true) // 下方偏移旋转
        {
            if (KFS_flag.MF1_finish == true)
            {
                // 第一排旋转
                if (target_yaw_ == -90.0f)
                {
                    target_yaw_ = KFS_point.MF2_target_yaw_;
                    KFS_flag.spin_down_flag = false;
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

        //-------------------------------          MF2              -------------------------------//
        // 上方停止点旋转
        if (KFS_flag.spin_up_flag_2 == true && KFS_flag.spin_down_flag == false && KFS_flag.spin_up_flag == false)
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
                target_yaw_ = KFS_point.MF3_target_yaw_;
                Spin_Start = true;
            }
            // 判断退出
            else if (Spin_Start == true)
            {
                if (_tool_Abs(yaw - target_yaw_) < 2.0f)
                {
                    Spin_Start = false;
                    KFS_flag.spin_up_flag_2 = false;
                }
            }
        }

        if (KFS_flag.spin_down_flag_2 == true && KFS_flag.spin_down_flag == false && KFS_flag.spin_up_flag == false) // 下方偏移旋转
        {
            if (KFS_flag.MF2_finish == true)
            {
                // 第一排旋转
                if (target_yaw_ == -90.0f)
                {
                    target_yaw_ = KFS_point.MF3_target_yaw_;
                    KFS_flag.spin_down_flag_2 = false;
                }
                // 两侧旋转判断
                else if (KFS_point.MF2_pos_.x == curve.Get_Start_point().x && KFS_point.MF2_pos_.y == curve.Get_Start_point().y)
                {
                    KFS_flag.get_spin_flag = true;
                }
                // 两侧开始旋转
                else if (KFS_flag.get_spin_flag == true)
                {
                    target_yaw_ = KFS_point.MF3_target_yaw_;
                    KFS_flag.spin_down_flag_2 = false;
                    KFS_flag.get_spin_flag = false;
                }
            }
        }

    */
}

void OmniChassis_Setup::Clamping_Bar_Selection_Planning(void)
{
    // 夹杆流程只规划起点到固定终点的简化路径。
    target_yaw_ = 0.0f;
    path_line_.plan_reset();
    path_line_.Reset();

     path_line_.Add_Start_Point(robot_pos_);
     path_line_.Add_End_Point(test_point, path_param.CB);

    // 夹杆路径的测试
    //    path_line_.Add_Start_Point(robot_pos_);
    //    path_line_.Add_Point(CB_point.CB_Selection_start_point_, path_param.start);
    //    path_line_.Add_Point(CB_point.Clamping_Bar_Selection_pos_, CB_point.CB_Selection_control_point_, path_param.curve);
    //    path_line_.Add_End_Point(CB_point.Clamping_Bar_Retreat_pos_, path_param.end);

//    // 顺滑过弯
//    path_line_.Add_Start_Point(robot_pos_);
//    path_line_.Add_Point(Vector2D{0.6f + KFS_point.coner_ahead, 2.6f}, path_param.start);
//    path_line_.Add_Point(Vector2D{0.6f, 2.6f + KFS_point.coner_ahead}, path_param.curve);
//    path_line_.Add_End_Point(Vector2D{0.6f, 5.0f}, path_param.end);

    // 夹杆路径
    //    path_line_.Add_Start_Point(robot_pos_);
    //    path_line_.Add_Point(CB_point.CB_Start_pos, path_param.line);
    //    path_line_.Add_Point(CB_point.CB_Selection_pos, path_param.end);
    //    path_line_.Add_End_Point(CB_point.CB_End_pos, path_param.end);

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
                    V.planspeed = path_line_.plan(robot_pos_);
                    Path_correction();
                    V.corrVelocity = V.PID_coefficient * V.corrVelocity;
                    speed = v_limit();
                    if (path_line_.Get_Index() == 1)
                    {
                        speed = speed.magnitude() * Vector2D{0.0f, 1.0f};
                    }
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
                    pid_dead_flag = path_lock.get_is_in_dead_zone();
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
            if (path_line_.Is_End() == false)
            {
                // 获取曲线（带保护）
                curve = path_line_.get_bezier_curve();
                // 旋转点位判断以及KFS的拾取判断
                Path_KFS_check();
                if (Arm_Start == false && Spin_Start == false)
                {
                    // 5. 规划速度+叠加纠偏速度：计算路径规划的前进速度（切向速度）
                    V.planspeed = path_line_.plan(robot_pos_);
                    Path_correction();
                    V.corrVelocity = V.PID_coefficient * V.corrVelocity;
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
                    pid_dead_flag = path_lock.get_is_in_dead_zone();
                }
            }
            else
            {
                float lock_err = (robot_pos_ - Path_end_point).magnitude();
                speed = path_lock.pid_calc(0.0f, lock_err) * (robot_pos_ - Path_end_point).normalize();
                target_chassis_twist_.vx = speed.x;
                target_chassis_twist_.vy = speed.y;
            }
            robot_pos_ = robot_pos_ + speed * 0.001f;
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
#if s_debug
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

    default:
    {
        // 其他模式不下发新命令。
        break;
    }
    }
}

//////////////////////////////////////////       路径纠偏      //////////////////////////////////////////////////////

void OmniChassis_Setup::Path_correction(void)
{
    float tNearest = 0.0f;   // 最近点在贝塞尔曲线上的参数t (0~1)
    float tLookahead = 0.0f; // 前视点在贝塞尔曲线上的参数t (0~1)

    curve.Get_Nearest_Distance(robot_pos_, &tNearest);

    Vector2D nearestPt = curve.Get_Point(tNearest);

    // float err_curve = (nearestPt - robot_pos_).magnitude();

    float obj_dis = _tool_Abs((curve.Get_End_point() - robot_pos_).magnitude());

    // ======== 终点纠偏（新架构下平滑退化为终点位置吸附）========
    if (obj_dis < V.m_lookaheadDist || path_line_.Is_End() == true)
    {
        Vector2D endPt = curve.Get_End_point();

        if (curve.Get_len() < 0.0001f)
        {
            V.corrVelocity = {0.0f, 0.0f};
            return;
        }
        V.corrVelocity.x = pid_pos_x.pid_calc(endPt.x, robot_pos_.x);
        V.corrVelocity.y = pid_pos_y.pid_calc(endPt.y, robot_pos_.y);
        return;
    }

    // ======== 动态兔子追踪 (2D Cartesian PID) ========
    // 2. 寻找前视点作为我们追踪的“虚拟兔子”

    Vector2D lookaheadPt; // 路径上的前视点

    tLookahead = tNearest; // 前视点的编号，先从最近点的编号开始（比如t=0.3）

#if opti
    {
        // 弧长表二分查找：利用 BezierCurve 已有的 distance_list[200]
        // Get_Current_Len 内部做 O(1) 查表+线性插值，二分替代逐点步进
        float current_len = curve.Get_Current_Len(tNearest);
        float target_len = current_len + V.m_lookaheadDist;

        if (target_len >= curve.Get_len())
        {
            // 前视距离超出曲线总长 → 直接用终点
            tLookahead = 1.0f;
            lookaheadPt = curve.Get_Point(1.0f);
        }
        else
        {
            float lo = tNearest, hi = 1.0f;
            for (int i = 0; i < 8; i++)
            {
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
        while (tLookahead < 1.0f && accumulatedDist < V.m_lookaheadDist)
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
    V.corrVelocity.x = pid_pos_x.pid_calc(lookaheadPt.x, robot_pos_.x);
    V.corrVelocity.y = pid_pos_y.pid_calc(lookaheadPt.y, robot_pos_.y);
}

///////////////////////////////////       KFS路径生成            ////////////////////////////////

bool OmniChassis_Setup::KFS_Selection_Planning(void)
{
    int KFS_num = 0;
    if (KFS_point.MF1 > 0 && KFS_point.MF2 == 0 && KFS_point.MF3 == 0)
    {
        KFS_num = 1;
    }
    else if (KFS_point.MF1 > 0 && KFS_point.MF2 > 0 && KFS_point.MF3 == 0)
    {
        KFS_num = 2;
    }
    else if (KFS_point.MF1 > 0 && KFS_point.MF2 > 0 && KFS_point.MF3 > 0)
    {
        KFS_num = 3;
    }
    else
    {
        return false;
    }

    // 自动规划接口转换
    Point2D robot_point_ = {robot_pos_.x, robot_pos_.y};

    // 计算理想的KFS路径
    KFS_KeyPoint_ = MF_AutoCtrler::PathInformation_calc(robot_point_, KFS_point.MF1, KFS_point.MF2, KFS_point.MF3);

    int8_t MF1_Index_ = KFS_KeyPoint_.Index_MFroad[0]; // MF1 对应索引
    int8_t MF2_Index_ = KFS_KeyPoint_.Index_MFroad[1]; // MF2 对应索引
    int8_t MF3_Index_ = KFS_KeyPoint_.Index_MFroad[2]; // MF3 对应索引

    // 寻找MF拾取车辆点位
    MF1_Point_ = KFS_KeyPoint_.mustPastMap[MF1_Index_]; // MF1 对应地图点编号。
    MF2_Point_ = KFS_KeyPoint_.mustPastMap[MF2_Index_]; // MF2 对应地图点编号。
    MF3_Point_ = KFS_KeyPoint_.mustPastMap[MF3_Index_]; // MF3 对应地图点编号。

    // 写入MF地图对应坐标
    KFS_point.MF1_pos_ = MF_AutoCtrler::MapCenterWorld_Vector2D(MF1_Point_);
    if (KFS_num > 1)
    {
        KFS_point.MF2_pos_ = MF_AutoCtrler::MapCenterWorld_Vector2D(MF2_Point_);
    }
    else
    {
        KFS_point.MF2_pos_ = {0.0f, 0.0f};
    }
    if (KFS_num > 2)
    {
        KFS_point.MF3_pos_ = MF_AutoCtrler::MapCenterWorld_Vector2D(MF3_Point_);
    }
    else
    {
        KFS_point.MF3_pos_ = {0.0f, 0.0f};
    }

    // 判断MF1的车子朝向
    target_yaw_ = rotation_path(MF1_Point_);
    // 判断MF2的车子朝向
    if (KFS_num > 1)
    {
        KFS_point.MF2_target_yaw_ = rotation_path(MF2_Point_);
    }
    // 判断MF3的车子朝向
    if (KFS_num > 2)
    {
        KFS_point.MF3_target_yaw_ = rotation_path(MF3_Point_);
    }

    // 判断第一次是否需要转向
    if (target_yaw_ == KFS_point.MF2_target_yaw_ || KFS_num < 2.0f)
    {
        KFS_flag.spin_flag = false;
    }
    else if (target_yaw_ != KFS_point.MF2_target_yaw_)
    {
        KFS_flag.spin_flag = true;
    }
    else
    {
        KFS_flag.spin_flag = false;
    }

    // 判断第二次是否需要转向
    if (KFS_point.MF2_target_yaw_ == KFS_point.MF3_target_yaw_ || KFS_num < 3.0f)
    {
        KFS_flag.spin_flag_2 = false;
    }
    else if (KFS_point.MF2_target_yaw_ != KFS_point.MF3_target_yaw_)
    {
        KFS_flag.spin_flag_2 = true;
    }
    else
    {
        KFS_flag.spin_flag_2 = false;
    }

    // 计算出口索引
    int index_exit = 0; // 当前路径出口索引（有效路径点长度）。
    while (index_exit < 15 && KFS_KeyPoint_.mustPastMap[index_exit] != 0)
    {
        index_exit++;
    }

    // 写入路径点的临时变量
    Vector2D last_vector = robot_pos_;
    Vector2D temp_vector = {0.0f, 0.0f};
    Vector2D spin_vector = {0.0f, 0.0f};
    int temp_point = 0;
    int i = 0;

    // 在梅林内的情况
    if (MF_AutoCtrler::GetMapNumFromPos(robot_point_))
    {
        i = 1;
    }

    // 重置路径规划器
    path_line_.plan_reset();
    path_line_.Reset();
    path_line_.Add_Start_Point(robot_pos_);

    // 写入起点到MF2路径点坐标（不包含MF2）
    if (KFS_flag.spin_flag == false)
    {
        for (; i < (KFS_num == 1 ? index_exit : MF2_Index_); i++)
        {
            if (KFS_num == 1)
            {
                if (i == (index_exit - 1)) // 终点
                {
                    temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_End_Point(temp_vector, path_param.end);
                    // 取末端点进行路径退出后的锁点pid
                    Path_end_point = path_line_.Get_End_Point();
                    return true;
                }
            }
            temp_point = KFS_KeyPoint_.mustPastMap[i];
            temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(temp_point);
            // 四个拐点的顺滑处理
            if (temp_point == 1 || temp_point == 5 || temp_point == 26 || temp_point == 30)
            {
                spin_vector = spinodal_path(last_vector, temp_vector, i);
                if (spin_vector.x == 0.0f && spin_vector.x == 0.0f)
                    return false;
            }
            else if (temp_point == MF1_Point_) // MF停止点
            {
                path_line_.Add_Point(temp_vector, path_param.end);
            }
            else // 衔接路径
            {
                path_line_.Add_Point(temp_vector, path_param.start);
            }
            last_vector = temp_vector;
        }
    }
    else if (KFS_flag.spin_flag == true)
    {
        if (target_yaw_ == -90.0f || target_yaw_ == 90.0f) // MF1在下
        {
            // KFS_flag.spin_down_flag = true;
            for (; i < (KFS_num == 1 ? index_exit : MF2_Index_); i++)
            {
                if (KFS_num == 1)
                {
                    if (i == (index_exit - 1)) // 终点
                    {
                        temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                        path_line_.Add_End_Point(temp_vector, path_param.end);
                        // 取末端点进行路径退出后的锁点pid
                        Path_end_point = path_line_.Get_End_Point();
                        return true;
                    }
                }
                temp_point = KFS_KeyPoint_.mustPastMap[i];
                temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(temp_point);
                // 四个拐点的顺滑处理
                if (temp_point == 1 || temp_point == 5 || temp_point == 26 || temp_point == 30)
                {
                    spin_vector = spinodal_path(last_vector, temp_vector, i);
                    if (spin_vector.x == 0.0f && spin_vector.x == 0.0f)
                        return false;
                }
                else if (temp_point == MF1_Point_) // MF停止点
                {
                    path_line_.Add_Point(temp_vector, path_param.end);
                }
                else // 衔接路径
                {
                    path_line_.Add_Point(temp_vector, path_param.start);
                }
                last_vector = temp_vector;
            }
        }
        else // MF1在两边
        {
            bool FINSH = false;
            for (; i < (KFS_num == 1 ? index_exit : MF2_Index_); i++)
            {
                if (KFS_num == 1)
                {
                    if (i == (index_exit - 1)) // 终点
                    {
                        temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                        path_line_.Add_End_Point(temp_vector, path_param.end);
                        // 取末端点进行路径退出后的锁点pid
                        Path_end_point = path_line_.Get_End_Point();
                        return true;
                    }
                }
                temp_point = KFS_KeyPoint_.mustPastMap[i];
                temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(temp_point);
                // 四个拐点的顺滑处理
                if (temp_point == 1 || temp_point == 5 || temp_point == 26 || temp_point == 30)
                {
                    spin_vector = spinodal_path(last_vector, temp_vector, i);
                    if (spin_vector.x == 0.0f && spin_vector.x == 0.0f)
                        return false;
                    if (FINSH == true)
                    {
                        KFS_point.spin_pos = spin_vector;
                        FINSH = false;
                    }
                }
                else if (temp_point == MF1_Point_) // MF停止点
                {
                    path_line_.Add_Point(temp_vector, path_param.end);
                    FINSH = true;
                }
                else // 衔接路径
                {
                    path_line_.Add_Point(temp_vector, path_param.start);
                }
                last_vector = temp_vector;
            }
        }
    }

    // 写入MF2到终点路径点坐标
    if (KFS_flag.spin_flag_2 == false)
    {
        for (i = MF2_Index_; i < index_exit; i++)
        {
            if (i == (index_exit - 1)) // 终点
            {
                temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                path_line_.Add_End_Point(temp_vector, path_param.end);
            }
            else // 中间的路径点
            {
                temp_point = KFS_KeyPoint_.mustPastMap[i];
                temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(temp_point);
                // 四个拐点的顺滑处理
                if (temp_point == 1 || temp_point == 5 || temp_point == 26 || temp_point == 30)
                {
                    spin_vector = spinodal_path(last_vector, temp_vector, i);
                    if (spin_vector.x == 0.0f && spin_vector.x == 0.0f)
                        return false;
                }
                else if (((temp_point == MF3_Point_) && KFS_num > 2) || ((temp_point == MF2_Point_) && KFS_num > 1)) // MF停止点
                {
                    path_line_.Add_Point(temp_vector, path_param.end);
                }
                else // 衔接路径
                {
                    path_line_.Add_Point(temp_vector, path_param.start);
                }
                last_vector = temp_vector;
            }
        }
    }
    else if (KFS_flag.spin_flag_2 == true)
    {
        if (KFS_point.MF2_target_yaw_ == -90.0f || KFS_point.MF2_target_yaw_ == 90.0f) // MF2在下
        {
            for (i = MF2_Index_; i < index_exit; i++)
            {

                if (i == (index_exit - 1)) // 终点
                {
                    temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_End_Point(temp_vector, path_param.end);
                }
                else
                {
                    temp_point = KFS_KeyPoint_.mustPastMap[i];
                    temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(temp_point);
                    // 四个拐点的顺滑处理
                    if (temp_point == 1 || temp_point == 5 || temp_point == 26 || temp_point == 30)
                    {
                        spin_vector = spinodal_path(last_vector, temp_vector, i);
                        if (spin_vector.x == 0.0f && spin_vector.x == 0.0f)
                            return false;
                    }
                    else if (((temp_point == MF3_Point_) && KFS_num > 2) || ((temp_point == MF2_Point_) && KFS_num > 1)) // MF停止点
                    {
                        path_line_.Add_Point(temp_vector, path_param.end);
                    }
                    else // 衔接路径
                    {
                        path_line_.Add_Point(temp_vector, path_param.start);
                    }
                    last_vector = temp_vector;
                }
            }
        }
        else // MF2在两边
        {
            bool FINSH = false;
            for (i = MF2_Index_; i < index_exit; i++)
            {
                if (i == (index_exit - 1))
                {
                    temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_End_Point(temp_vector, path_param.end);
                }
                else
                {
                    temp_point = KFS_KeyPoint_.mustPastMap[i];
                    temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(temp_point);
                    // 四个拐点的顺滑处理
                    if (temp_point == 1 || temp_point == 5 || temp_point == 26 || temp_point == 30)
                    {
                        spin_vector = spinodal_path(last_vector, temp_vector, i);
                        if (spin_vector.x == 0.0f && spin_vector.x == 0.0f)
                            return false;
                        if (FINSH == true)
                        {
                            KFS_point.spin_pos_2 = spin_vector;
                            FINSH = false;
                        }
                    }
                    else if (((temp_point == MF3_Point_) && KFS_num > 2) || ((temp_point == MF2_Point_) && KFS_num > 1)) // MF停止点
                    {
                        if (temp_point == MF2_Point_)
                            FINSH = true;
                        path_line_.Add_Point(temp_vector, path_param.end);
                    }
                    else // 衔接路径
                    {
                        path_line_.Add_Point(temp_vector, path_param.start);
                    }
                    last_vector = temp_vector;
                }
            }
        }
    }

    // 取末端点进行路径退出后的锁点pid
    Path_end_point = path_line_.Get_End_Point();
    return true;
}
float OmniChassis_Setup::rotation_path(float MF_Point)
{
    if (MF_Point == 21 || MF_Point == 16 || MF_Point == 11 || MF_Point == 6)
    {
        return (RB_Flag ? 180.0f : 0.0f);
    }
    else if (MF_Point == 25 || MF_Point == 20 || MF_Point == 15 || MF_Point == 10)
    {
        return (RB_Flag ? 0.0f : 180.0f);
    }
    else if (MF_Point == 27 || MF_Point == 28 || MF_Point == 29 || MF_Point == 30)
    {
        return 90.0f;
    }
    else if (MF_Point == 2 || MF_Point == 3 || MF_Point == 4 || MF_Point == 5)
    {
        return -90.0f;
    }
}
Vector2D OmniChassis_Setup::spinodal_path(Vector2D last_vector, Vector2D temp_vector, int i)
{
    Vector2D forward_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i + 1]);
    // 拐点前偏移点
    path_line_.Add_Point((temp_vector + ((last_vector - temp_vector).normalize() * KFS_point.coner_ahead)), path_param.start);
    /*
    1->(1,0)
    2->(-1,0)
    3->(0,1)
    4->(0,-1)
    */
    // 拐点方向判断
    Vector2D tangent_vector = (forward_vector - temp_vector).normalize();
    if (tangent_vector.x == 1.0f)
        path_param.curve.targetPos = 1.0f;
    else if (tangent_vector.x == -1.0f)
        path_param.curve.targetPos = 2.0f;
    else if (tangent_vector.y == 1.0f)
        path_param.curve.targetPos = 3.0f;
    else if (tangent_vector.y == -1.0f)
        path_param.curve.targetPos = 4.0f;
    else
        return Vector2D{0.0f, 0.0f};
    // 拐点后偏移点
    Vector2D result = (temp_vector + (tangent_vector * KFS_point.coner_ahead));
    path_line_.Add_Point(result, path_param.curve);
    path_param.curve.targetPos = 999.0f;
    return result;
}

void OmniChassis_Setup::flag_reset(void)
{
    // 统一清空自动流程的阶段标志与旋转状态。
    WeaponSage_Start = false;
    WeaponSage_End = false;
    Arm_Start = false;
    Spin_Start = false;
    pid_dead_flag = false;

    KFS_flag.MF1_flag = false;
    KFS_flag.MF2_flag = false;
    KFS_flag.MF3_flag = false;

    KFS_flag.spin_flag = false;
    KFS_flag.spin_flag_2 = false;

    //    KFS_flag.spin_up_flag = false;
    //    KFS_flag.spin_down_flag = false;

    //    KFS_flag.spin_up_flag_2 = false;
    //    KFS_flag.spin_down_flag_2 = false;

    KFS_flag.MF1_finish = false;
    KFS_flag.MF2_finish = false;

    KFS_flag.get_spin_flag = false;

    CB_flag.Selection_flag = false;
    CB_flag.Retreat_flag = false;
}

Vector2D OmniChassis_Setup::v_limit(void)
{
    // 使用单位向量做正交分解，避免 |normal|² 缩放
    Vector2D tangent = path_line_.Get_Tangent_Vector();
    Vector2D tangent_dir = tangent.normalize();
    Vector2D normal_dir = tangent_dir.perpendicular();

    // 切向 = 前馈 + PID纠偏沿切向分量（PID不限幅，终点 planspeed=0 时保留切向纠偏）
    Vector2D v_tangent = V.planspeed * V.FF_coefficient + V.corrVelocity.project_onto(tangent_dir);

    if (v_tangent.magnitude() > V.planspeed.magnitude())
        v_tangent = v_tangent.normalize() * V.planspeed.magnitude();

    // 法向 = PID纠偏沿法向分量（限幅防止侧向过冲）
    Vector2D v_normal = V.corrVelocity.project_onto(normal_dir);
    if (v_normal.magnitude() > V.v_normal_max)
        v_normal = v_normal.normalize() * V.v_normal_max;

    Vector2D v_end = v_tangent + v_normal;

    num++;
    if (num > 5)
    {
        debug_uart.printf_DMA("%f,%f,%f,%f\n", robot_pos_.x, robot_pos_.y, speed.magnitude(), v_tangent);
        num = 0;
    }

    return v_end;
}
