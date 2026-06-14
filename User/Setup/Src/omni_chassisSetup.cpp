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

void OmniChassis_Setup::Clamping_Bar_Selection_Planning(void)
{
    // 夹杆流程只规划起点到固定终点的简化路径。
    target_yaw = CB_yaw;
    path_line_.Reset();
    path_line_.plan_reset();

    //     path_line_.Add_Start_Point(robot_pos_);
    //     path_line_.Add_End_Point(test_point, path_param.CB);

    // 夹杆路径的测试
    //    path_line_.Add_Start_Point(robot_pos_);
    //    path_line_.Add_Point(CB_point.CB_Selection_start_point_, path_param.start);
    //    path_line_.Add_Point(CB_point.Clamping_Bar_Selection_pos_, CB_point.CB_Selection_control_point_, path_param.curve);
    //    path_line_.Add_End_Point(CB_point.Clamping_Bar_Retreat_pos_, path_param.end);

    // 夹杆路径
    path_line_.Add_Start_Point(robot_pos_);
    //    if (robot_pos_.y < CB_point.CB_Selection_pos.y)
    //    {
    //        path_line_.Add_Point(CB_point.CB_Start_pos, path_param.line);
    //    }
    //    path_line_.Add_Point(CB_point.CB_Selection_pos, path_param.end);
    path_line_.Add_End_Point(CB_point.CB_End_pos, path_param.end);

    Path_end_point = path_line_.Get_End_Point();
}

void OmniChassis_Setup::CZ_R1_Selection_Planning(void)
{
    // 夹杆流程只规划起点到固定终点的简化路径。
    target_yaw = CZ_point.R1_yaw;
    path_line_.Reset();
    path_line_.plan_reset();

    CZ_point.R1_pos_index = (CZ_point.R1_pos_index + 1) % 3;
    path_line_.Add_Start_Point(robot_pos_);
    path_line_.Add_End_Point(CZ_point.R1_pos[CZ_point.R1_pos_index], path_param.end);

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
        CZ_point.fit_pos_index = (CZ_point.fit_pos_index + 1) % 2;
        path_line_.Add_Start_Point(robot_pos_);
        path_line_.Add_End_Point(CZ_point.fit_pos[CZ_point.fit_pos_index], path_param.end);
    }
    else if (airjoy_data_.SWA == 0x00)
    {
        CZ_point.R2_pos_index = (CZ_point.R2_pos_index + 1) % 3;
        path_line_.Add_Start_Point(robot_pos_);
        path_line_.Add_End_Point(CZ_point.R2_pos[CZ_point.R2_pos_index], path_param.R2);
    }

    Path_end_point = path_line_.Get_End_Point();
}

void OmniChassis_Setup::loop()
{
    // 未初始化时不进入控制流程。
    if (!init_flag)
        return;

    float dyaw = Locate_Setup::getInstance()->get_dyaw_from_position();

    //yaw = Locate_Setup::getInstance()->get_yaw_from_position();
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
    ladar_data_ = Locate_Setup::getInstance()->get_RobotPos_inWorld();
    //robot_pos_.x = ladar_data_.x;
    //robot_pos_.y = ladar_data_.y;

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

        target_yaw = yaw;
        chassis_status_last_ = chassis_status_;
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

        target_yaw = yaw;
        chassis_status_last_ = chassis_status_;
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

        target_yaw = yaw;
        chassis_status_last_ = chassis_status_;
        break;
    }
    case CHASSIS_MANUAL_CONTROL_D:
    {
        float target_vel_x = 0.0f;
        float target_vel_y = 0.0f;
        float target_omega_z = 0.0f;

        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_vel_x = airjoy_data_.left_x * 1 * this->is_chassis_reverse_;
        else
            target_vel_x = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_vel_y = airjoy_data_.left_y * 1 * this->is_chassis_reverse_;
        else
            target_vel_y = 0.0f;

        if (_tool_Abs(airjoy_data_.right_x) > 0.05f)
            target_omega_z = airjoy_data_.right_x * 1;
        else
            target_omega_z = 0.0f;

        if (airjoy_data_.SWD == 0x00)
            chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, target_vel_x, target_vel_y, CB_yaw * PI / 180.0f);

        else if (airjoy_data_.SWD == 0x01)
            chassis.setSteerDegAndDriveSpeed(90.0f, target_vel_x);

        chassis_status_last_ = chassis_status_;
        break;
    }

    case CHASSIS_AUTO_CONTROL_CB:
    {

        if (chassis_status_last_ != chassis_status_)
        {
            flag_reset();
            path_line_.Reset();
            path_line_.plan_reset();

            Path_end_point = robot_pos_;
        }
        // 夹杆自动流程：触发后执行路径规划、纠偏和速度合成。
        if (flag == 1)
        {
            flag = 0;
            Clamping_Bar_Selection_Planning();
        }

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
                if (path_line_.Get_Curve_Flag() == true)
                {
                    speed = speed * V.spinodal_coefficient;
                }
                target_chassis_twist_.vx = speed.x;
                target_chassis_twist_.vy = speed.y;
            }
            else
            {
                Path_lock_point(curve.Get_Start_point());
            }
        }
        else
        {
            Path_lock_point(Path_end_point);
        }
        float target_yaw_rad = target_yaw * PI / 180.0f;
        chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, target_chassis_twist_.vx, target_chassis_twist_.vy, target_yaw_rad);

        chassis_status_last_ = chassis_status_;
        break;
    }

    case CHASSIS_STOP:
    {
        if (chassis_status_last_ != chassis_status_)
        {
            flag_reset();
            path_line_.Reset();
            path_line_.plan_reset();
            Path_end_point = robot_pos_;
        }
        // KFS 自动流程：路径跟踪 + 旋转点处理 + 机械臂联动。
        if (flag == 1)
        {
            robot_pos_=test_point;
            flag = 0;
            KFS_Selection_Planning();
        }
        if (path_line_.Is_End() == false)
        {
            // 获取曲线（带保护）
            curve = path_line_.get_bezier_curve();
            // 旋转点位判断以及KFS的拾取判断
            Path_KFS_check();
            if (Arm_Start == false)
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
                Path_lock_point(curve.Get_Start_point());
            }
        }
        else
        {
            Path_lock_point(Path_end_point);
        }
        robot_pos_=robot_pos_+(speed*0.001f);
        
        float target_yaw_rad = target_yaw * PI / 180.0f;
        chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, target_chassis_twist_.vx, target_chassis_twist_.vy, target_yaw_rad);

        chassis_status_last_ = chassis_status_;
        break;
    }

    case CHASSIS_AUTO_CONTROL_CZ_R1:
    {
        if (chassis_status_last_ != chassis_status_)
        {
            flag_reset();
            path_line_.Reset();
            path_line_.plan_reset();
            Path_end_point = robot_pos_;
            CZ_point.fit_pos_index = 1;
            CZ_point.R1_pos_index = 0;
            CZ_point.R2_pos_index = 0;
        }
        // KFS 自动流程：路径跟踪 + 旋转点处理 + 机械臂联动。
        if (flag == 1)
        {
            flag = 0;
            CZ_R1_Selection_Planning();
        }
        if (path_line_.Is_End() == false)
        {
            // 获取曲线（带保护）
            curve = path_line_.get_bezier_curve();
            // 旋转点位判断以及KFS的拾取判断
            Path_KFS_check();
            if (true)
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
                Path_lock_point(curve.Get_Start_point());
            }
        }
        else
        {
            Path_lock_point(Path_end_point);
        }
        

        float target_yaw_rad = target_yaw * PI / 180.0f;
        chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, target_chassis_twist_.vx, target_chassis_twist_.vy, target_yaw_rad);

        chassis_status_last_ = chassis_status_;
        break;
    }

    case CHASSIS_AUTO_CONTROL_CZ_R2:
    {
        if (chassis_status_last_ != chassis_status_)
        {
            flag_reset();
            path_line_.Reset();
            path_line_.plan_reset();
            Path_end_point = robot_pos_;
            CZ_point.fit_pos_index = 1;
            CZ_point.R1_pos_index = 0;
            CZ_point.R2_pos_index = 0;
        }
        // KFS 自动流程：路径跟踪 + 旋转点处理 + 机械臂联动。
        if (flag == 1)
        {
            flag = 0;
            CZ_R2_Selection_Planning();
        }
        if (path_line_.Is_End() == false)
        {
            // 获取曲线（带保护）
            curve = path_line_.get_bezier_curve();
            // 旋转点位判断以及KFS的拾取判断
            Path_KFS_check();
            if (true)
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
                Path_lock_point(curve.Get_Start_point());
            }
        }
        else
        {
            Path_lock_point(Path_end_point);
        }

        float target_yaw_rad = target_yaw * PI / 180.0f;
        chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, target_chassis_twist_.vx, target_chassis_twist_.vy, target_yaw_rad);

        chassis_status_last_ = chassis_status_;
        break;
    }

    case CHASSIS_AUTO_CONTROL_KFS:
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

    target_chassis_twist_.vx = speed.x;
    target_chassis_twist_.vy = speed.y;
    if (pid_dead_flag == true)
    {
        target_chassis_twist_.vx = 0.0f;
        target_chassis_twist_.vy = 0.0f;
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

    // 3. 在绝对世界坐标系下，独立计算X轴和Y轴的纠偏向速度
    // 将不再计算切法向，直接基于XY差值PID
    V.corrVelocity.x = pid_pos_x.pid_calc(lookaheadPt.x, robot_pos_.x);
    V.corrVelocity.y = pid_pos_y.pid_calc(lookaheadPt.y, robot_pos_.y);
}

///////////////////////////////////       KFS路径生成            ////////////////////////////////

bool OmniChassis_Setup::KFS_Selection_Planning(void)
{
    // 只能在一区和二区进行启动
    if (robot_pos_.x < 0.0f || robot_pos_.x > 6.0f || robot_pos_.y > 10.0f || robot_pos_.y < 0.0f)
    {
        return false;
    }

    // 对kfs夹取数量进行判断，并进行合法判断
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
    int8_t MF1_Point_ = KFS_KeyPoint_.mustPastMap[MF1_Index_]; // MF1 对应地图点编号。
    int8_t MF2_Point_ = KFS_KeyPoint_.mustPastMap[MF2_Index_]; // MF2 对应地图点编号。
    int8_t MF3_Point_ = KFS_KeyPoint_.mustPastMap[MF3_Index_]; // MF3 对应地图点编号。

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
    KFS_point.MF1_target_yaw_ = rotation_path(MF1_Point_);
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
    if (KFS_point.MF1_target_yaw_ == KFS_point.MF2_target_yaw_ || KFS_num < 2.0f)
    {
        KFS_flag.spin_flag = false;
    }
    else if (KFS_point.MF1_target_yaw_ != KFS_point.MF2_target_yaw_)
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
    
    // 重置路径规划器
    path_line_.Reset();
    path_line_.plan_reset();

    path_line_.Add_Start_Point(robot_pos_);

    // 在梅林内的情况处理，如果需要在外面旋转会先生成路径，如果需要拿同左右同列的会退出
    if (MF_AutoCtrler::GetMapNumFromPos(robot_point_))
    {
        i = 1;
        temp_point = KFS_KeyPoint_.mustPastMap[0];
        // 拐角无法处理防止撞车
        if (temp_point == 1 || temp_point == 5 || temp_point == 26 || temp_point == 30)
        {
            if (robot_pos_.y > 2.6f || robot_pos_.y < 8.5f || robot_pos_.x > 0.7f || robot_pos_.x < 5.3f)
                return false;
        }
        else if (temp_point == 27 || temp_point == 28 || temp_point == 29 || temp_point == 30 || temp_point == 2 || temp_point == 3 || temp_point == 4 || temp_point == 5)        
        {
            // 在上下两排就直接转
            target_yaw = KFS_point.MF1_target_yaw_;
        }
        else if (temp_point == 21 || temp_point == 16 || temp_point == 11 || temp_point == 6 || temp_point == 25 || temp_point == 20 || temp_point == 15 || temp_point == 10)
        {
            if (i != (index_exit - 1))
            {
                // 左右两排且接下来不为终点
                if (KFS_KeyPoint_.mustPastMap[1] == 1 || KFS_KeyPoint_.mustPastMap[1] == 5 || KFS_KeyPoint_.mustPastMap[1] == 26 || KFS_KeyPoint_.mustPastMap[1] == 30)
                {

                    if (_tool_Abs(yaw - KFS_point.MF1_target_yaw_) < 10.0f)
                    {
                        // 角度差距小直接转当做无事发生
                        target_yaw = KFS_point.MF1_target_yaw_;
                    }
                    else
                    {
                        // 需要旋转的则生成路径并将索引改到2
                        KFS_flag.spin_flag_0 = true;
                        float spin_delay = 1.0f;
                        if (KFS_KeyPoint_.mustPastMap[1] == 1 || KFS_KeyPoint_.mustPastMap[1] == 5)
                        {
                            spin_delay *= (-1.0f);
                        }
                        spin_vector = spinodal_path(last_vector, MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[1]), i, (KFS_point.spin_skew * spin_delay));
                        if (spin_vector.x == 0.0f && spin_vector.x == 0.0f)
                            return false;
                        KFS_point.spin_pos_0 = spin_vector;
                        i = 2;
                        last_vector=MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[1]);
                    }
                }
                else
                {
                    return false;
                }
            }
        }
    }
    else
    {
        target_yaw = KFS_point.MF1_target_yaw_;
    }

    // 写入起点到MF2路径点坐标（不包含MF2）
    bool FINSH = false;
    for (; i < (KFS_num == 1 ? index_exit : MF2_Index_); i++)
    {
        if (KFS_num == 1)
        {
            if (i == (index_exit - 1)) // 终点
            {
                temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                // 梅林自动规划后是否直接上三区
                if (KFS_flag.uphill_flag == false)
                {
                    path_line_.Add_End_Point(temp_vector, path_param.end);
                }
                else if (KFS_flag.uphill_flag == true)
                {
                    if (last_vector.x == 0.6f)
                    {
                        path_line_.Add_Point(temp_vector, path_param.start);
                        path_line_.Add_Point(CZ_point.uphill_pos, path_param.up);
                        path_line_.Add_End_Point(CZ_point.R1_pos[1], path_param.end);
                    }
                    else if (last_vector.y == 8.6f)
                    {
                        path_line_.Add_Point((temp_vector + ((last_vector - temp_vector).normalize() * KFS_point.coner_ahead)), path_param.start);
                        path_line_.Add_Point((temp_vector + (Vector2D{0.0f, 1.0f} * KFS_point.coner_ahead)), path_param.curve);
                        path_line_.Add_Point(CZ_point.uphill_pos, path_param.up);
                        path_line_.Add_End_Point(CZ_point.R1_pos[1], path_param.end);
                    }
                }
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
            float spin_delay = KFS_flag.spin_flag == true && (KFS_point.MF1_target_yaw_ == 180.0f || KFS_point.MF1_target_yaw_ == 0.0f);
            if (temp_point == 1 || temp_point == 5)
            {
                spin_delay *= (-1.0f);
            }
            spin_vector = spinodal_path(last_vector, temp_vector, i, (KFS_point.spin_skew * spin_delay));
            if (spin_vector.x == 0.0f && spin_vector.x == 0.0f)
                return false;
            if (spin_delay != 0)
            {
                if (FINSH == true)
                {
                    KFS_point.spin_pos = spin_vector;
                    FINSH = false;
                }
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

    // 写入MF2到终点路径点坐标
    FINSH = false;
    for (i = MF2_Index_; i < index_exit; i++)
    {
        if (i == (index_exit - 1))
        {
            temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);

            // 梅林自动规划后是否直接上三区
            if (KFS_flag.uphill_flag == false)
            {
                path_line_.Add_End_Point(temp_vector, path_param.end);
            }
            else if (KFS_flag.uphill_flag == true)
            {
                if (last_vector.x == 0.6f)
                {
                    path_line_.Add_Point(temp_vector, path_param.start);
                    path_line_.Add_Point(CZ_point.uphill_pos, path_param.up);
                    path_line_.Add_End_Point(CZ_point.R1_pos[1], path_param.end);
                }
                else if (last_vector.y == 8.6f)
                {
                    path_line_.Add_Point((temp_vector + ((last_vector - temp_vector).normalize() * KFS_point.coner_ahead)), path_param.start);
                    path_line_.Add_Point((temp_vector + (Vector2D{0.0f, 1.0f} * KFS_point.coner_ahead)), path_param.curve);
                    path_line_.Add_Point(CZ_point.uphill_pos, path_param.up);
                    path_line_.Add_End_Point(CZ_point.R1_pos[1], path_param.end);
                }
            }
        }
        else
        {
            temp_point = KFS_KeyPoint_.mustPastMap[i];
            temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(temp_point);
            // 四个拐点的顺滑处理
            if (temp_point == 1 || temp_point == 5 || temp_point == 26 || temp_point == 30)
            {
                float spin_delay = KFS_flag.spin_flag_2 == true && (KFS_point.MF2_target_yaw_ == 180.0f || KFS_point.MF2_target_yaw_ == 0.0f);
                if (temp_point == 1 || temp_point == 5)
                {
                    spin_delay *= (-1.0f);
                }
                spin_vector = spinodal_path(last_vector, temp_vector, i, (KFS_point.spin_skew * spin_delay));
                if (spin_vector.x == 0.0f && spin_vector.x == 0.0f)
                    return false;
                if (spin_delay != 0)
                {
                    if (FINSH == true)
                    {
                        KFS_point.spin_pos_2 = spin_vector;
                        FINSH = false;
                    }
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
Vector2D OmniChassis_Setup::spinodal_path(Vector2D last_vector, Vector2D temp_vector, int i, float spin_flag)
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

    // 拐点偏移
    temp_vector.y = temp_vector.y + spin_flag;

    // 拐点后偏移点
    Vector2D result = (temp_vector + (tangent_vector * KFS_point.coner_behind));
    path_line_.Add_Point(result, path_param.curve);
    path_param.curve.targetPos = 999.0f;
    return result;
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

    if (KFS_flag.uphill_flag == true)
    {
        // 上坡后旋转判断
        if (CZ_point.uphill_pos.x == curve.Get_Start_point().x && CZ_point.uphill_pos.y == curve.Get_Start_point().y)
        {
            if (robot_pos_.x > CZ_point.skew_yaw)
                target_yaw = CZ_point.R1_yaw;
        }
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
}

void OmniChassis_Setup::flag_reset(void)
{
    // 统一清空自动流程的阶段标志与旋转状态。
    WeaponSage_Start = false;
    WeaponSage_End = false;
    Arm_Start = false;

    pid_dead_flag = false;

    KFS_flag.MF1_flag = false;
    KFS_flag.MF2_flag = false;
    KFS_flag.MF3_flag = false;

    KFS_flag.spin_flag_0 = false;
    KFS_flag.spin_flag = false;
    KFS_flag.spin_flag_2 = false;

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
        debug_uart.printf_DMA("%f,%f,%f,%f\n", robot_pos_.x, robot_pos_.y, speed.magnitude(), target_yaw);
        num = 0;
    }

    return v_end;
}
