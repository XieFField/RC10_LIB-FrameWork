#include "omni_chassisSetup.h"

#ifndef CAMERA_FAKE
#define CAMERA_FAKE 0
#endif

#if debug_ladar

int last_cout_ladar_data = -1;

#endif

uint32_t chassisstackHighWaterMark = 0;
extern Chassis chassis;

void OmniChassis_Setup::loop()
{
    // 未初始化时不进入控制流程。
    if (!init_flag)
        return;

    float dyaw = Locate_Setup::getInstance()->get_dyaw_from_position();

    //	chassisstackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

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

    case CHASSIS_LOCK_FORWEAPON:
    {
        // 武器联动模式：平移可控，航向强制锁定到 90 度。
        const float target_yaw_angle = 90.0f;
        const float target_yaw_rad = 90.0f * PI / 180.0f;

        float target_vel_x = 0.0f;
        float target_vel_y = 0.0f;
        float target_rot_z = 0.0f;

        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_vel_x = airjoy_data_.left_x * 3 * this->is_chassis_reverse_;
        else
            target_vel_x = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_vel_y = airjoy_data_.left_y * 3 * this->is_chassis_reverse_;
        else
            target_vel_y = 0.0f;

        target_rot_z = target_yaw_rad;

        chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, target_vel_x, target_vel_y, target_rot_z);

        break;
    }

    case CHASSIS_AUTO_CONTROL_CB:
    {
        num++;
        if(num>3)
        {
            debug_uart.printf_DMA("%f,%f,%f,%f\n", robot_pos_.x, robot_pos_.y, speed.magnitude(),err_curve);
            num=0;
        }
        // 夹杆自动流程：触发后执行路径规划、纠偏和速度合成。
        if (flag == 1)
        {
            flag_reset();
            flag = 0;
            flag_run = 1;
            Clamping_Bar_Selection_Planning();
            WeaponSage_END = 0;
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
#if FF_V
                speed = ComposeRobotVelocity(corrVelocity);
#else
                speed = corrVelocity;
#endif
                speed = v_limit(speed);
                target_chassis_twist_.vx = speed.x;
                target_chassis_twist_.vy = speed.y;
            }
//            else
//            {
//                float lock_err = (robot_pos_ - Clamping_Bar_Selection_pos_).magnitude();
//                speed = path_lock.pid_calc(0.0f, lock_err) * (robot_pos_ - Clamping_Bar_Selection_pos_).normalize();
//                target_chassis_twist_.vx = speed.x;
//                target_chassis_twist_.vy = speed.y;
//                WeaponSage_END = true;
//            }
            else
            {
                // 路径结束：复位状态并清空速度命令。
                flag = 0;
                flag_run = 0;
                flag_reset();
                WeaponSage_END = true;

                speed = {0.0f, 0.0f};
                planspeed = {0.0f, 0.0f};
                target_chassis_twist_ = {0.0f, 0.0f};

                path_line_.plan_reset();
                path_line_.Reset();
#if FF_V
                ResetAutoControlStates();
#endif
            }
        }
        else
        {
            target_chassis_twist_ = {0.0f, 0.0f};
            chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, target_chassis_twist_.vx, target_chassis_twist_.vy);
#if FF_V
            ResetAutoControlStates();
#endif
        }

        float target_yaw_rad = target_yaw_ * PI / 180.0f;
        chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, target_chassis_twist_.vx, target_chassis_twist_.vy, target_yaw_rad);

        break;
    }

    case CHASSIS_AUTO_CONTROL_KFS:
    {
        // KFS 自动流程：路径跟踪 + 旋转点处理 + 机械臂联动。
        if (flag == 1)
        {
            path_line_.plan_reset();
            path_line_.Reset();
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
                // 旋转点位判断以及KFS的拾取判断
                Path_spin_check();
                if (Arm_Start == false && Spin_Start == false)
                {
                    // 5. 规划速度+叠加纠偏速度：计算路径规划的前进速度（切向速度）
                    planspeed = path_line_.plan(robot_pos_);

                    Path_correction();
#if FF_V
                    speed = ComposeRobotVelocity(corrVelocity);
#else
                    speed = corrVelocity;
#endif
                    speed = v_limit(speed);
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
                }
            }
            else
            {
                // 路径结束：复位状态并清空速度命令。
                flag = 0;
                flag_run = 0;
                flag_reset();

                speed = {0.0f, 0.0f};
                planspeed = {0.0f, 0.0f};
                target_chassis_twist_ = {0.0f, 0.0f};

                path_line_.plan_reset();
                path_line_.Reset();
#if FF_V
                ResetAutoControlStates();
#endif
            }
            float target_yaw_rad = target_yaw_ * PI / 180.0f;
            chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, target_chassis_twist_.vx, target_chassis_twist_.vy, target_yaw_rad);
        }
        else
        {
            // 未运行时保持原地锁角并清理自动控制历史量。
            //            target_yaw_ = yaw;
            target_chassis_twist_ = {0.0f, 0.0f};
            chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, target_chassis_twist_.vx, target_chassis_twist_.vy);

#if FF_V
            ResetAutoControlStates();
#endif
        }

        break;
    }

    case CHASSIS_STOP:
    {

        chassis.setZeroCurrent();

        break;
    }

    case CHASSIS_MANUAL_CONTROL_C:
    {
        // 模式 C：全向速度控制，角速度固定为 0。
        target_chassis_twist_.yaw_rate = 0.0f;

        if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
            target_chassis_twist_.vx = airjoy_data_.left_x * 6 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vx = 0.0f;

        if (_tool_Abs(airjoy_data_.left_y) > 0.05f)
            target_chassis_twist_.vy = airjoy_data_.left_y * 6 * this->is_chassis_reverse_;
        else
            target_chassis_twist_.vy = 0.0f;

        chassis.setSpeed(Chassis::Coordinate::kWorld, target_chassis_twist_.vx, target_chassis_twist_.vy, target_chassis_twist_.yaw_rate);

        // 结束相机模式处理。
        break;
    }

    default:
    {
        // 其他模式不下发新命令。
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
}




//////////////////////////////////////////       路径纠偏      //////////////////////////////////////////////////////

void OmniChassis_Setup::Path_correction(void)
{
    // 1. 找最近点+t值：获取路径上距离当前位置最近的点及其参数 tNearest

    // 第一步：调用你的Get_Nearest_Distance，拿到tNearest（最近点对应的t值）
    // 重点：第二个参数传 &tNearest（tNearest的地址），因为你的函数是“输出参数”（通过指针赋值）
    curve.Get_Nearest_Distance(robot_pos_, &tNearest);

    // 第二步：用第一步拿到的tNearest，调用你的Get_Point，拿到最近点坐标
    Vector2D nearestPt = curve.Get_Point(tNearest);
    
    err_curve=(nearestPt-robot_pos_).magnitude();

    float obj_dis = _tool_Abs((curve.Get_End_point() - robot_pos_).magnitude());
    
    
    /*
    // ======== 终点纠偏（新架构下平滑退化为终点位置吸附）========
    if (obj_dis < gradient_start_ || path_line_.Is_End() == false)
    {
        Vector2D endPt = curve.Get_End_point();
#if FF_V
        // 终点段把前馈参考点切换为终点坐标，差分会自然收敛到 0。
        ff_ref_point_ = endPt;
#endif

        if (curve.Get_len() < 0.0001f)
        {
            corrVelocity = {0.0f, 0.0f};
            return;
        }

        corrVelocity.x = pid_pos_x.pid_calc(endPt.x, robot_pos_.x);
        corrVelocity.y = pid_pos_y.pid_calc(endPt.y, robot_pos_.y);
        
        if (obj_dis <= gradient_end_)
        {
            corrVelocity = corrVelocity * min_gradient_;
        }
        else
        {
            float gradient = min_gradient_ - (1 - _tool_Abs(obj_dis - gradient_end_) / _tool_Abs(gradient_start_ - gradient_end_)) * (1 - min_gradient_);
            corrVelocity = corrVelocity * gradient;
        }
        
        return;
    }
    */
    // ======== 动态兔子追踪 (2D Cartesian PID) ========
    // 2. 寻找前视点作为我们追踪的“虚拟兔子”
    Vector2D lookaheadPt; // 路径上的前视点
    if (curve.Get_Bezier_Order() == FIRST_ORDER_BEZIER)
    {
        m_lookaheadDist = m_lookaheadDist_line;
    }
    else
    {
        m_lookaheadDist = m_lookaheadDist_curve;
    }
    lookaheadPt = FindLookaheadPoint(curve, tNearest, tLookahead);
#if FF_V
    // 非终点阶段前馈参考点使用前视点。
    ff_ref_point_ = lookaheadPt;
#endif
    // 3. 在绝对世界坐标系下，独立计算X轴和Y轴的纠偏向速度
    // 将不再计算切法向，直接基于XY差值PID
    corrVelocity.x = pid_pos_x.pid_calc(lookaheadPt.x, robot_pos_.x);
    corrVelocity.y = pid_pos_y.pid_calc(lookaheadPt.y, robot_pos_.y);
    
    //corrVelocity=path_line_.Get_Tangent_Vector()*corrVelocity.magnitude();
}

void OmniChassis_Setup::Path_spin_check(void)
{
    // KFS拾取判断
    if (MF1_pos_.x == curve.Get_End_point().x && MF1_pos_.y == curve.Get_End_point().y)
    {
        MF1_flag = true;
    }
    else if (MF1_flag == true)
    {
        MF1_flag = false;
        Arm_Start = true;
        MF1_finish = true;
    }
    if (MF2_pos_.x == curve.Get_End_point().x && MF2_pos_.y == curve.Get_End_point().y)
    {
        MF2_flag = true;
    }
    else if (MF2_flag == true)
    {
        MF2_flag = false;
        Arm_Start = true;
        MF1_finish = true;
    }
    // 根据路径节点关系，处理上/下两种旋转过渡逻辑。
    // 上方停止点旋转
    if (spin_up_flag == true)
    {
        // 判断旋转条件
        if (spin_point_.x == curve.Get_End_point().x && spin_point_.y == curve.Get_End_point().y)
        {
            get_spin_flag = true;
        }
        // 开始旋转
        else if (get_spin_flag == true)
        {
            get_spin_flag = false;
            // target_yaw_=MF2_target_yaw_;
            Spin_Start = true;
        }
        // 判断退出
        else if (Spin_Start == true)
        {
            if (_tool_Abs(yaw - target_yaw_) < 2.0f)
            {
                Spin_Start = false;
                spin_up_flag = false;
            }
        }
    }
    else if (spin_down_flag == true) // 下方偏移旋转
    {
        if (MF1_finish == true)
        {
            // 第一排旋转
            if (target_yaw_ == 90.0f)
            {
                // 延迟旋转
                if (robot_pos_.y <= 2.55f)
                {
                    target_yaw_ = MF2_target_yaw_;
                    spin_down_flag = false;
                }
            }
            // 两侧旋转判断
            else if (MF1_pos_.x == curve.Get_Start_point().x && MF1_pos_.y == curve.Get_Start_point().y)
            {
                get_spin_flag = true;
            }
            // 两侧开始旋转
            else if (get_spin_flag == true)
            {
                target_yaw_ = MF2_target_yaw_;
                spin_down_flag = false;
                get_spin_flag = false;
            }
        }
    }
}

/////////////////////////////////    路径初始化代码   //////////////////////////////////////////////

void OmniChassis_Setup::Clamping_Bar_Selection_Planning(void)
{
    // 夹杆流程只规划起点到固定终点的简化路径。
    target_yaw_ = 0.0f;
    path_line_.plan_reset();
    path_line_.Reset();
    path_line_.Add_Start_Point(robot_pos_);
    path_line_.Add_Point(Vector2D{robot_pos_.x-0.5f, robot_pos_.y}, path_param_curve_);
    path_line_.Add_Point(Vector2D{robot_pos_.x-0.5f-0.63f, robot_pos_.y+0.63f}, Vector2D{robot_pos_.x-0.5f-0.85f, robot_pos_.y-0.22f}, path_param_curve_);
    //   path_line_.Add_End_Point(Clamping_Bar_Selection_pos_);
    path_line_.Add_End_Point(Vector2D{robot_pos_.x-0.5f-0.63f, robot_pos_.y+0.63f+0.2f}, path_param_end_);
}

void OmniChassis_Setup::KFS_Selection_Planning(void)
{
    int8_t MF1_Point_ = 0; // MF1 对应地图点编号。
    int8_t MF2_Point_ = 0; // MF2 对应地图点编号。

    // 基于当前位置和目标点编号计算整条必经路径。
    // 自动规划接口转换
    Point2D robot_point_ = {robot_pos_.x, robot_pos_.y};
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
    if (target_yaw_ == MF2_target_yaw_ || MF2 == 0.0f)
    {
        spin_flag = false;
    }
    else
    {
        spin_flag = true;
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

    MF1_pos_ = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[KFS_KeyPoint_.Index_MFroad[0]]);
    if(MF2!=0)
    {
        MF2_pos_ = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[KFS_KeyPoint_.Index_MFroad[1]]);
    }
    else
    {
        MF2_pos_ = {0.0f,0.0f};
    }
    
    if (spin_flag == false)
    {
        for (int i = 0; i < index_exit; i++)
        {
            if (i == (index_exit - 1))
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                path_line_.Add_End_Point(temp_vector, path_param_KFS_);
            }
            else
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                path_line_.Add_Point(temp_vector, path_param_KFS_);
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
                    path_line_.Add_End_Point(temp_vector, path_param_KFS_);
                }
                else if (i == (KFS_KeyPoint_.Index_MFroad[0] + 1))
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    temp_vector.y = temp_vector.y + spin_skew_;
                    path_line_.Add_Point(temp_vector, path_param_KFS_);
                    spin_down_flag = true;
                }
                else
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_Point(temp_vector, path_param_KFS_);
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
                    path_line_.Add_End_Point(temp_vector, path_param_KFS_);
                }
                else if (i == (KFS_KeyPoint_.Index_MFroad[0] + 1))
                {
                    path_line_.Add_Point(spin_point_, path_param_KFS_);
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_Point(temp_vector, path_param_KFS_);
                    spin_up_flag = true;
                }
                else
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_Point(temp_vector, path_param_KFS_);
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
                    path_line_.Add_End_Point(temp_vector, path_param_KFS_);
                }
                else if (i == (KFS_KeyPoint_.Index_MFroad[0] + 1))
                {
                    if (KFS_KeyPoint_.mustPastMap[i] == 26 || KFS_KeyPoint_.mustPastMap[i] == 30) // 上
                    {
                        path_line_.Add_Point(spin_point_, path_param_KFS_);
                        Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                        path_line_.Add_Point(temp_vector, path_param_KFS_);
                        spin_up_flag = true;
                    }
                    else if (KFS_KeyPoint_.mustPastMap[i] == 1 || KFS_KeyPoint_.mustPastMap[i] == 5) // 下
                    {
                        Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                        temp_vector.y = temp_vector.y + spin_skew_;
                        path_line_.Add_Point(temp_vector, path_param_KFS_);
                        spin_down_flag = true;
                    }
                }
                else
                {
                    Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    path_line_.Add_Point(temp_vector, path_param_KFS_);
                }
            }
        }
    }
}

/////////////////////////////////    路径纠偏代码   //////////////////////////////////////////////

Vector2D OmniChassis_Setup::FindLookaheadPoint(BezierCurve &path_, float tNearest, float &tLookahead)
{
    tLookahead = tNearest;        // 前视点的编号，先从最近点的编号开始（比如t=0.3）
    float accumulatedDist = 0.0f; // 累计挪了多少距离（刚开始是0）
    float step = 0.01f;           // 每次挪的“小步子”

    Vector2D lastPt = path_.Get_Point(tLookahead);

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

    if (tLookahead >= 1.0f)
    {
        lastPt = path_.Get_Point(1.0f); // 拿曲线终点坐标
    }

    return lastPt;
}

Vector2D OmniChassis_Setup::v_limit(Vector2D &v)
{
    // 判定是否进入终点段，用于控制参数切换。
    /* bool near_end = (_tool_Abs((curve.Get_End_point() - robot_pos_).magnitude()) < deadzone_max_end_);
    if (near_end)
    {
        v = v.normalize() * robot_speed_end_;
        return v;
    }
    if (v.magnitude() > max_robot_speed_)
    {
        v = v.normalize() * max_robot_speed_;
    } */
    bool near_end = (_tool_Abs((curve.Get_End_point() - robot_pos_).magnitude()) < deadzone_max_end_);
    if(MF2_flag == true ||MF1_flag == true)
    {
        if(near_end)
        {
            v = v.normalize() * robot_speed_end_;
            return v;
        }
    }
    if (v.magnitude() >  planspeed.magnitude())
    {
        v = v.normalize() * planspeed.magnitude();
    }
    if (v.magnitude() < min_robot_speed_)
    {
        v = v.normalize() * min_robot_speed_;
    }
    return v;
}

void OmniChassis_Setup::flag_reset(void)
{
    // 统一清空自动流程的阶段标志与旋转状态。
    WeaponSage_END = false;
    Arm_Start = false;
    MF1_flag = false;
    MF2_flag = false;
    spin_flag = false;
    spin_up_flag = false;
    spin_down_flag = false;
    MF1_finish = false;
    get_spin_flag = false;
    Spin_Start = false;
}
#if FF_V
void OmniChassis_Setup::ResetAutoControlStates(void)
{
    // 1) 阻尼项使用上一时刻 v_robot，退出自动流程后必须清零，避免“历史速度”带入下一次任务。
    v_robot_last_cmd_ = {0.0f, 0.0f};

    // 2) 前馈差分状态一并复位，避免参考点跳变时出现首帧尖峰。
    ff_diff_inited_ = false;
    ff_ref_point_last_ = {0.0f, 0.0f};
    ff_velocity_lpf_ = {0.0f, 0.0f};
}

Vector2D OmniChassis_Setup::ComposeRobotVelocity(const Vector2D &v_pid)
{
    // 判定是否进入终点段，用于控制参数切换。
    bool near_end = (_tool_Abs((curve.Get_End_point() - robot_pos_).magnitude()) < deadzone_max_end_);
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
        ff_velocity_lpf_ = ff_velocity_lpf_ * max_ff_speed_;
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

    float pid_scale = 1.0f;

    Vector2D v_pid_out = v_pid * pid_scale;

    Vector2D v_damp = v_robot_last_cmd_ * (-k_damp_);

    // 最终速度合成：闭环主导 + 前馈提速 + 历史速度阻尼。
    Vector2D v_robot = v_pid_out + v_ff + v_damp;

    v_robot_last_cmd_ = v_robot;
    return v_robot;
}
#endif


//=======================================              相机接口函数         =====================================================//
