#include "omni_chassisSetup.h"
extern Chassis chassis;
float CB_yaw = 89.0f;
void OmniChassis_Setup::Path_CB_check(void)
{
    if (CB_point.CB_Selection_pos.x == curve.Get_End_point().x && CB_point.CB_Selection_pos.y == curve.Get_End_point().y)
    {
        CB_flag.Selection_flag = true;
    }
    else if (CB_flag.Selection_flag == true)
    {
        CB_flag.Selection_flag = false;
        pid_dead_flag = false;
        WeaponSage_Start = true;
    }
    if (airjoy_data_.SWA == 0x00)
    {
        if (CB_point.CB_End_pos.x == curve.Get_End_point().x && CB_point.CB_End_pos.y == curve.Get_End_point().y)
        {
            CB_flag.Retreat_flag = true;
            if (WeaponSage_Start == false)
            {
                target_yaw = 90.0f;
            }
        }
        else if (CB_flag.Retreat_flag == true)
        {
            CB_flag.Retreat_flag = false;
            pid_dead_flag = false;
            WeaponSage_End = true;
        }
    }
    else if (airjoy_data_.SWA == 0x01)
    {
        if (CB_point.CB_transition_pos.x == curve.Get_End_point().x && CB_point.CB_transition_pos.y == curve.Get_End_point().y)
        {
            if (WeaponSage_Start == false)
            {
                target_yaw = 90.0f;
            }
        }
        if (CB_point.CB_welt_pos.x == curve.Get_End_point().x && CB_point.CB_welt_pos.y == curve.Get_End_point().y)
        {
            CB_flag.Retreat_flag = true;
        }
        else if (CB_flag.Retreat_flag == true)
        {
            CB_flag.Retreat_flag = false;
            pid_dead_flag = false;
            WeaponSage_End = true;
        }
    }
}

void OmniChassis_Setup::Clamping_Bar_Selection_Planning(void)
{
    // 夹杆流程只规划起点到固定终点的简化路径。
    // target_yaw = CB_yaw;
    target_yaw = 0.0f;
    path_line_.Reset();
    path_line_.plan_reset();

    // 夹杆路径
    path_line_.Add_Start_Point(robot_pos_);
    if (robot_pos_.y < CB_point.CB_Selection_pos.y)
    {
        path_line_.Add_Point(CB_point.CB_Start_pos, path_param.line);
    }
    path_line_.Add_Point(CB_point.CB_Selection_pos, path_param.end);

    // 相机流程
    if (airjoy_data_.SWA == 0x00)
    {
        path_line_.Add_End_Point(CB_point.CB_End_pos, path_param.end);
    }
    else if (airjoy_data_.SWA == 0x01)
    {
        path_line_.Add_Point(CB_point.CB_transition_pos, path_param.end);
        path_line_.Add_End_Point(CB_point.CB_welt_pos, path_param.end);
    }
    Path_end_point = path_line_.Get_End_Point();
}

void OmniChassis_Setup::CZ_R1_Selection_Planning(void)
{
    // 夹杆流程只规划起点到固定终点的简化路径。
    target_yaw = CZ_point.R1_yaw;
    path_line_.Reset();
    path_line_.plan_reset();

    path_line_.Add_Start_Point(robot_pos_);
    if (CZ_flag.R1_FB_index == 1)
    {
        path_line_.Add_End_Point({CZ_point.R1_pos[CZ_flag.R1_RL_index][1].x, robot_pos_.y}, path_param.end);
    }
    else
    {
        path_line_.Add_End_Point(CZ_point.R1_pos[CZ_flag.R1_RL_index][0], path_param.end);
    }

    Path_end_point = path_line_.Get_End_Point();
}

void OmniChassis_Setup::CZ_R2_Selection_Planning(void)
{
    // 夹杆流程只规划起点到固定终点的简化路径。
    target_yaw = CZ_point.fit_yaw;
    path_line_.Reset();
    path_line_.plan_reset();

    // 合体地点和等待地点的切换
    if (airjoy_data_.SWA == 0x01)
    {
        CZ_flag.fit_pos_index = (CZ_flag.fit_pos_index + 1) % 2;
        path_line_.Add_Start_Point(robot_pos_);
        path_line_.Add_End_Point(CZ_point.fit_pos[CZ_flag.fit_pos_index], path_param.end);
    }
    else if (airjoy_data_.SWA == 0x00)
    {
        CZ_flag.R2_pos_index = (CZ_flag.R2_pos_index + 1) % 3;
        path_line_.Add_Start_Point(robot_pos_);
        path_line_.Add_End_Point(CZ_point.R2_pos[CZ_flag.R2_pos_index], path_param.R2);
    }

    Path_end_point = path_line_.Get_End_Point();
}

void OmniChassis_Setup::loop()
{
    // 未初始化时不进入控制流程。
    if (!init_flag)
        return;

    yaw = Locate_Setup::getInstance()->get_yaw_from_position();
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
    Point3D ladar_data_ = Locate_Setup::getInstance()->get_RobotPos_inWorld();
    robot_pos_.x = ladar_data_.x;
    robot_pos_.y = ladar_data_.y;

    switch (chassis_status_)
    {
    //////-----------------------------------            手操模式           ----------------------------------/////
    case CHASSIS_MANUAL_CONTROL_A:
    {
        // 模式 A：大速度手动平移 + 角速度控制。
        CHASSIS_MANUAL(2.0f, 2.0f);
        chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, Chassis_Target.yaw_rate);
        chassis_status_last_ = chassis_status_;
        break;
    }
    case CHASSIS_MANUAL_CONTROL_B:
    {
        // 模式 B：低速手动平移，锁当前航向。
        CHASSIS_MANUAL(0.6f);
        chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY);
        chassis_status_last_ = chassis_status_;
        break;
    }
    case CHASSIS_MANUAL_CONTROL_C:
    {
        // 模式 C：全向速度控制，锁当前航向。
        CHASSIS_MANUAL(1.0f);
        chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY);
        chassis_status_last_ = chassis_status_;
        break;
    }
    case CHASSIS_MANUAL_CONTROL_D:
    {
        CHASSIS_MANUAL(1.0f, 1.0f);
        if (airjoy_data_.SWD == 0x00)
            chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, CB_yaw * PI / 180.0f);
        else if (airjoy_data_.SWD == 0x01)
            chassis.setSteerDegAndDriveSpeed(90.0f, Chassis_Target.VX);
        chassis_status_last_ = chassis_status_;
        break;
    }
    /////-----------------------------               一区            -----------------------------------/////
    case CHASSIS_AUTO_CONTROL_CB:
    {
        if (chassis_status_last_ != chassis_status_)
        {
            flag_reset();
            path_line_.Reset();
            path_line_.plan_reset();
            Path_end_point = robot_pos_;
        }
        if (flag == 1)
        {
            flag = 0;
            flag_reset();
            Clamping_Bar_Selection_Planning();
        }
        if (path_line_.Is_End() == false)
        {
            curve = path_line_.get_bezier_curve();
            Path_CB_check();
            if (WeaponSage_Start == false && WeaponSage_End == false)
                v_plan();
            else
                Path_lock_point(curve.Get_Start_point());
        }
        else
        {
            if (airjoy_data_.SWA == 0x00 && WeaponSage_End == true)
            {

                CHASSIS_MANUAL(0.4f, 0.0f, false);
                chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
            }
            else if (airjoy_data_.SWA == 0x01 && WeaponSage_End == true)
            {

                CHASSIS_MANUAL(0.4f, 0.4f);
                chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, Chassis_Target.yaw_rate);
            }
            else
            {
                Path_lock_point(Path_end_point);
                chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
            }
        }
        chassis_status_last_ = chassis_status_;
        break;
    }

    /////-----------------------------               二区            -----------------------------------/////
    case CHASSIS_AUTO_CONTROL_KFS:
    {
        if (chassis_status_last_ != chassis_status_)
        {
            flag_reset();
            path_line_.Reset();
            path_line_.plan_reset();
            Path_end_point = robot_pos_;
        }
        if (flag == 1)
        {
            flag = 0;
            flag_reset();
            KFS_Selection_Planning();
        }
        if (path_line_.Is_End() == false)
        {
            curve = path_line_.get_bezier_curve();
            Path_KFS_check();
            if (Arm_Start == false)
                v_plan();
            else
                Path_lock_point(curve.Get_Start_point());
        }
        else
        {
            Path_lock_point(Path_end_point);
        }

        chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));

        chassis_status_last_ = chassis_status_;
        break;
    }

    //----------------------------------             CZ_R1              --------------------------------------//
    case CHASSIS_AUTO_CONTROL_CZ_R1:
    {
        static int far_flag = 0;
        static int near_flag = 0;
        if (chassis_status_last_ != chassis_status_)
        {
            flag_reset();
            path_line_.Reset();
            path_line_.plan_reset();
            Path_end_point = robot_pos_;
            CZ_flag.fit_pos_index = 1;
            CZ_flag.R1_FB_index = 1;
            CZ_flag.R1_RL_index = 1;
            CZ_flag.R2_pos_index = 0;
        }

        if (airjoy_data_.right_x < -0.80f)
            far_flag = 1;
        else if (far_flag > 0 &&airjoy_data_.right_x > -0.05f&& CZ_flag.R1_FB_index == 0)
        {
            if(far_flag > 5)
            {
                if (CZ_flag.R1_RL_index < 2)
                    CZ_flag.R1_RL_index++;
                CZ_R1_Selection_Planning();
                far_flag = 0;
            }
            else
            {
                far_flag ++;
            }
        }
        else if(CZ_flag.R1_FB_index == 1)
        {
            far_flag=0;
        }


        if (airjoy_data_.right_x > 0.80f)
            near_flag  = 1;
        else if (near_flag >0 &&airjoy_data_.right_x < 0.05f&&CZ_flag.R1_FB_index == 0)
        {
            if(near_flag >5)
            {
                if (CZ_flag.R1_RL_index > 0)
                    CZ_flag.R1_RL_index --;
                CZ_R1_Selection_Planning();
                near_flag = 0;
                
            }
            else
            {
                near_flag++;
            }
                
        }
        else if(CZ_flag.R1_FB_index == 1)
        {
            near_flag=0;
        }

        if (flag == 1)
        {
            flag_reset();
            CZ_flag.R1_FB_index = (CZ_flag.R1_FB_index+1)%2;
            CZ_R1_Selection_Planning();
            flag = 0;
        }

        if (path_line_.Is_End() == false)
        {
            curve = path_line_.get_bezier_curve();
            if (true)
                v_plan();
            else
                Path_lock_point(curve.Get_Start_point());
        }
        else
        {
            // 锁点后切换为半手操
            if (pid_dead_flag == false)
            {
                Path_lock_point(Path_end_point);
            }
            else
            {
                if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
                    Chassis_Target.VY = (-airjoy_data_.left_x) * 0.6f * this->is_chassis_reverse_;
                else
                    Chassis_Target.VY = 0.0f;
                Chassis_Target.VX = 0.0f;
            }
        }

        chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));

        chassis_status_last_ = chassis_status_;
        break;
    }

    //----------------------------------             CZ_R2              --------------------------------------//
    case CHASSIS_AUTO_CONTROL_CZ_R2:
    {
        if (chassis_status_last_ != chassis_status_)
        {
            flag_reset();
            path_line_.Reset();
            path_line_.plan_reset();
            Path_end_point = robot_pos_;
            CZ_flag.fit_pos_index = 1;
            CZ_flag.R1_FB_index = 0;
            CZ_flag.R1_RL_index = 1;
            CZ_flag.R2_pos_index = 0;
        }
        if (flag == 1)
        {
            flag = 0;
            CZ_R2_Selection_Planning();
        }
        if (path_line_.Is_End() == false)
        {
            curve = path_line_.get_bezier_curve();
            if (true)
                v_plan();
            else
                Path_lock_point(curve.Get_Start_point());
        }
        else
        {
            Path_lock_point(Path_end_point);
        }

        chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
        chassis_status_last_ = chassis_status_;
        break;
    }

    //----------------------------------             CZ_新状态机              -----------------------------------//
    case CHASSIS_AUTO_CONTROL_CZ:
    {
        if (chassis_status_last_ != chassis_status_)
        {
            flag_reset();
            path_line_.Reset();
            path_line_.plan_reset();
            Path_end_point = robot_pos_;
            CZ_flag.R1_RL_index = 1;
            CZ_flag.R1_FB_index = 0;
            CZ_flag.fit_pos_index = 1;
            CZ_flag.R2_pos_index = -1;
        }

        CZ_state_switch();
        switch (CZ_state)
        {
        case MANUAL:
        {
            CHASSIS_MANUAL(1.0f, 1.0f);
            chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, Chassis_Target.yaw_rate);
            CZ_state_last = CZ_state;
            break;
        }
        case SEMI_AUIO_FIT:
        {
            CZ_init();
            CZ_FIT_Path_Init();

            if (path_line_.Is_End() == false)
            {
                curve = path_line_.get_bezier_curve();
                if (true)
                    v_plan();
                else
                    Path_lock_point(curve.Get_Start_point());
            }
            else
            {
                Path_lock_point(Path_end_point);
            }

            chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
            chassis_status_last_ = chassis_status_;
            CZ_state_last = CZ_state;
            break;
        }
        case SEMI_AUIO_ARM:
        {
            CZ_init();
            CZ_ARM_Path_Init();

            if (path_line_.Is_End() == false)
            {
                curve = path_line_.get_bezier_curve();
                if (true)
                    v_plan();
                else
                    Path_lock_point(curve.Get_Start_point());
            }
            else
            {
                Path_lock_point(Path_end_point);
            }

            chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
            chassis_status_last_ = chassis_status_;
            CZ_state_last = CZ_state;
            break;
        }
        case SEMI_AUIO_WEAPON:
        {
            CZ_init();
            CZ_state_last = CZ_state;
            break;
        }
        case CZ_STOP:
        {
            target_yaw = yaw;
            chassis.setZeroCurrent();
            CZ_state_last = CZ_state;
            break;
        }
        default:
        {
            // 正常不会进入
            target_yaw = yaw;
            chassis.setZeroCurrent();
            break;
        }
        }

        chassis_status_last_ = chassis_status_;
        break;
    }

    case CHASSIS_STOP:
    {
        target_yaw = yaw;
        chassis_status_last_ = chassis_status_;
        chassis.setZeroCurrent();
        break;
    }

    default:
    {
        // 正常不会进入
        target_yaw = yaw;
        chassis_status_last_ = chassis_status_;
        chassis.setZeroCurrent();
        break;
    }
    }
}

//////////////////////////////////////////       路径纠偏      //////////////////////////////////////////////////////
void OmniChassis_Setup::Path_lock_point(Vector2D lock_point)
{
    float lock_err = (robot_pos_ - lock_point).magnitude();
    if (airjoy_data_.SWA == 0x00 && chassis_status_ == CHASSIS_AUTO_CONTROL_CZ_R2)
    {
        speed = path_lock_r2.pid_calc(0.0f, lock_err) * (robot_pos_ - lock_point).normalize();
    }
    else
    {
        speed = path_lock.pid_calc(0.0f, lock_err) * (robot_pos_ - lock_point).normalize();
    }
    pid_dead_flag = path_lock.get_is_in_dead_zone();

    Chassis_Target.VX = speed.x;
    Chassis_Target.VY = speed.y;
    if (pid_dead_flag == true)
    {
        Chassis_Target.VX = 0.0f;
        Chassis_Target.VY = 0.0f;
    }
}

void OmniChassis_Setup::Path_correction(void)
{
    float tNearest = 0.0f;   // 最近点在贝塞尔曲线上的参数t (0~1)
    float tLookahead = 0.0f; // 前视点在贝塞尔曲线上的参数t (0~1)

    curve.Get_Nearest_Distance(robot_pos_, &tNearest);

    Vector2D nearestPt = curve.Get_Point(tNearest);

    float obj_dis = _tool_Abs((curve.Get_End_point() - robot_pos_).magnitude());

    // ======== 终点纠偏（新架构下平滑退化为终点位置吸附）========
    if (obj_dis < V.m_lookaheadDist)
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

    Vector2D lookaheadPt; // 路径上的前视点

    tLookahead = tNearest; // 前视点的编号，先从最近点的编号开始（比如t=0.3）

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
    pid_dead_flag = path_lock.get_is_in_dead_zone();

    // 3. 在绝对世界坐标系下，独立计算X轴和Y轴的纠偏向速度
    // 将不再计算切法向，直接基于XY差值PID
    V.corrVelocity.x = pid_pos_x.pid_calc(lookaheadPt.x, robot_pos_.x);
    V.corrVelocity.y = pid_pos_y.pid_calc(lookaheadPt.y, robot_pos_.y);
}

///////////////////////////////////       KFS路径生成            ////////////////////////////////

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
        KFS_flag.MF3_finish = true;
    }

    if (KFS_flag.spin_flag_0 == true && Arm_Start == false)
    {
        // 两侧旋转判断
        if (KFS_point.spin_pos_0.x == curve.Get_End_point().x && KFS_point.spin_pos_0.y == curve.Get_End_point().y)
        {
            KFS_flag.get_spin_flag = true;
        }
        // 两侧开始旋转
        else if (KFS_flag.get_spin_flag == true)
        {
            target_yaw = KFS_point.MF1_target_yaw_;
            KFS_flag.spin_flag_0 = false;
            KFS_flag.get_spin_flag = false;
        }
    }

    if (KFS_flag.spin_flag == true && KFS_flag.MF1_finish == true && Arm_Start == false)
    {
        // 第一排和最后一排旋转
        if (target_yaw == -90.0f || target_yaw == 90.0f)
        {
            target_yaw = KFS_point.MF2_target_yaw_;
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
            target_yaw = KFS_point.MF2_target_yaw_;
            KFS_flag.spin_flag = false;
            KFS_flag.get_spin_flag = false;
        }
    }

    if (KFS_flag.spin_flag_2 == true && KFS_flag.spin_flag == false && KFS_flag.MF2_finish == true && Arm_Start == false)
    {
        // 第一排旋转
        if (target_yaw == -90.0f || target_yaw == 90.0f)
        {
            target_yaw = KFS_point.MF3_target_yaw_;
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
            target_yaw = KFS_point.MF3_target_yaw_;
            KFS_flag.spin_flag_2 = false;
            KFS_flag.get_spin_flag = false;
        }
    }

    if (KFS_flag.uphill_flag == true)
    {
        // 上坡后旋转判断
        if (CZ_point.uphill_pos.x == curve.Get_Start_point().x && CZ_point.uphill_pos.y == curve.Get_Start_point().y)
        {
            if (robot_pos_.x > CZ_point.skew_yaw)
                target_yaw = CZ_point.R1_yaw;
        }
    }
}

///////////////////////////////////////       新代码主要服务于新遥控       //////////////////////////////////

void OmniChassis_Setup::CZ_state_switch(void)
{
    if (airjoy_data_.SWB == 0x00)
    {
        CZ_state = MANUAL;
    }
    else if (airjoy_data_.SWB == 0x01)
    {
        if (airjoy_data_.SWC == 0x00)
        {
            CZ_state = SEMI_AUIO_FIT;
        }
        else if (airjoy_data_.SWC == 0x01 && airjoy_data_.SWD == 0x00)
        {

            CZ_state = SEMI_AUIO_ARM;
        }
        else if (airjoy_data_.SWC == 0x01 && airjoy_data_.SWD == 0x01)
        {
            CZ_state = SEMI_AUIO_WEAPON;
        }
        else
        {
            CZ_state = CZ_STOP;
        }
    }
    else
    {
        CZ_state = CZ_STOP;
    }
}
void OmniChassis_Setup::CZ_FIT_Path_Init(void)
{
    if (false) // 上键合体
    {
        CZ_flag.fit_pos_index = 1;
        CZ_R1_Selection_Planning();
    }
    else if (false) // 下键等待
    {
        CZ_flag.fit_pos_index = 0;
        CZ_R1_Selection_Planning();
    }
    else if ((false && RB_Flag == true) || (false && RB_Flag == false)) // 蓝场左，红场右
    {
        if (CZ_flag.R2_pos_index-- < 0)
            CZ_flag.R2_pos_index = 0;
        CZ_R1_Selection_Planning();
    }
    else if ((false && RB_Flag == true) || (false && RB_Flag == false)) // 蓝场右，红场左
    {
        if (CZ_flag.R2_pos_index++ > 2)
            CZ_flag.R2_pos_index = 2;
        CZ_R1_Selection_Planning();
    }
}

void OmniChassis_Setup::CZ_ARM_Path_Init(void)
{
    if (false) // 上键合体
    {
        CZ_flag.fit_pos_index = 1;
        CZ_R1_Selection_Planning();
    }
    else if (false) // 下键等待
    {
        CZ_flag.fit_pos_index = 0;
        CZ_R1_Selection_Planning();
    }
    else if ((false && RB_Flag == true) || (false && RB_Flag == false)) // 蓝场左，红场右
    {
        if (CZ_flag.R2_pos_index-- < 0)
            CZ_flag.R2_pos_index = 0;
        CZ_R1_Selection_Planning();
    }
    else if ((false && RB_Flag == true) || (false && RB_Flag == false)) // 蓝场右，红场左
    {
        if (CZ_flag.R2_pos_index++ > 2)
            CZ_flag.R2_pos_index = 2;
        CZ_R1_Selection_Planning();
    }
}