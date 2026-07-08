#include "omni_chassisSetup.h"
extern Chassis chassis;
void OmniChassis_Setup::CB_Path_Init(void)
{
    static bool pause_click = false;
    if (flag == 1)
    {
        flag = 0;
        flag_reset();
        CB_Selection_Planning();
    }

    // 暂停自动
    if (airjoy_data_.RT == 1 && pause_click == false)
    {
        pause_click = true;
        flag_reset();
        CB_Home_Selection_Planning();
    }
    if (airjoy_data_.RT == 0)
    {
        pause_click = false;
    }
}
void OmniChassis_Setup::CB_Home_Selection_Planning(void)
{
    // 只能在一区和进行启动
    if (robot_pos_.x < 0.0f || robot_pos_.x > 6.0f || robot_pos_.y > 2.8f || robot_pos_.y < 0.0f)
        return;
    // 夹杆流程只规划起点到固定终点的简化路径。
    path_line_.Reset();
    path_line_.plan_reset();

    path_line_.Add_Start_Point(robot_pos_, CB_point.cb_dead);

    if (robot_pos_.y < CB_point.home_transition_pos[RB_Flag].y && (RB_Flag ? robot_pos_.x > CB_point.home_transition_pos[RB_Flag].x : robot_pos_.x < CB_point.home_transition_pos[RB_Flag].x))
    {
        path_line_.Add_Point(CB_point.home_transition_pos[RB_Flag], path_param.speed);
    }
    path_line_.Add_End_Point(CB_point.home_pos[RB_Flag], path_param.speed);

    Path_end_point = path_line_.Get_End_Point();
}


void OmniChassis_Setup::CB_Selection_Planning(void)
{
    // 只能在一区和进行启动
    if (robot_pos_.x < 0.0f || robot_pos_.x > 6.0f || robot_pos_.y > 2.8f || robot_pos_.y < 0.0f)
        return;
    // 夹杆流程只规划起点到固定终点的简化路径。
    target_yaw = 0.0f;
    path_line_.Reset();
    path_line_.plan_reset();
#if CB_SINGLE
    // 每次进入都挪杆
    CB_point.pole_index = (CB_point.pole_index + 1) % 4;
    CB_point.CB_Selection_pos[RB_Flag].x = CB_point.CB_Selection_pos_0_x[RB_Flag] + (RB_Flag ? 1.0f : (-1.0f)) * CB_point.pole_index * 0.2f;
    CB_point.CB_Selection_pos[RB_Flag].y = CB_point.CB_Selection_pos_0_y;
#endif

    // 夹杆路径
    path_line_.Add_Start_Point(robot_pos_, CB_point.cb_dead);
    if (robot_pos_.y < CB_point.CB_Selection_pos[RB_Flag].y && (RB_Flag ? robot_pos_.x < CB_point.CB_Selection_pos[RB_Flag].x : robot_pos_.x > CB_point.CB_Selection_pos[RB_Flag].x))
    {
        path_line_.Add_Point(CB_point.CB_Start_pos[RB_Flag], path_param.line);
    }
    path_line_.Add_Point(CB_point.CB_Selection_pos[RB_Flag], path_param.cb);

    path_line_.Add_Point({CB_point.CB_Selection_pos[RB_Flag].x, CB_point.back_y}, path_param.cb);

    // 相机流程
    if (airjoy_data_.SWC == 0x00)
    {
        path_line_.Add_End_Point(CB_point.CB_End_pos[RB_Flag], path_param.cb);
    }
    else if (airjoy_data_.SWC == 0x01)
    {
        path_line_.Add_Point(CB_point.CB_transition_pos[RB_Flag], path_param.speed);
        path_line_.Add_End_Point(CB_point.CB_welt_pos[RB_Flag], path_param.speed);
    }
    Path_end_point = path_line_.Get_End_Point();
}


void OmniChassis_Setup::CB_Path_Check(void)
{
    static bool back_flag = false;
    static bool Selection_flag = false;
    static bool Retreat_flag = false;

    if (CB_point.CB_Selection_pos[RB_Flag].x == curve.Get_End_point().x && CB_point.CB_Selection_pos[RB_Flag].y == curve.Get_End_point().y)
    {
        Selection_flag = true;
    }
    else if (Selection_flag == true)
    {
        Selection_flag = false;
        pid_dead_flag = false;
        WeaponSage_Start = true;
    }

    if (CB_point.CB_Selection_pos[RB_Flag].x == curve.Get_End_point().x && CB_point.back_y == curve.Get_End_point().y)
    {
        back_flag = true;
    }
    else if (back_flag == true)
    {
        back_flag = false;
        pid_dead_flag = false;
        WeaponSage_Back = true;
    }

    if (airjoy_data_.SWC == 0x00)
    {
        if ( path_line_.Is_End() == false)
        {
            Retreat_flag = true;
            if (robot_pos_.y > CB_point.spin_y && WeaponSage_Start == false)
            {
                target_yaw=(RB_Flag?90.0f:-90.0f);
            }
        }
        else if (Retreat_flag == true)
        {

            Retreat_flag = false;
            WeaponSage_End = true;
        }
    }
    else if (airjoy_data_.SWC == 0x01)
    {
        if (CB_point.CB_transition_pos[RB_Flag].x == curve.Get_End_point().x && CB_point.CB_transition_pos[RB_Flag].y == curve.Get_End_point().y)
        {
            if (robot_pos_.y > CB_point.spin_y && WeaponSage_Start == false)
            {
                target_yaw=(RB_Flag?90.0f:-90.0f);
            }
        }
        if (path_line_.Is_End() == false)
        {
            Retreat_flag = true;
        }
        else if (Retreat_flag == true)
        {

            Retreat_flag = false;
            WeaponSage_End = true;
        }
    }

    if (CB_point.home_pos[RB_Flag].x == curve.Get_End_point().x && CB_point.home_pos[RB_Flag].y == curve.Get_End_point().y)
    {
        target_yaw = 0.0f;
    }
}


///////////////////////                    主循环                  /////////////////////////////

void OmniChassis_Setup::loop()
{
    // 未初始化时不进入控制流程。
    if (!init_flag)
        return;

    communication::Lora_communication::GetInstance()->update_airjoy_data(&airjoy_data_);
    yaw = Locate_Setup::getInstance()->get_RobotPos_inWorld().yaw;
    Point3D ladar_data_ = Locate_Setup::getInstance()->get_RobotPos_inWorld();
    RB_Flag = MF_AutoCtrler::get_color();
    robot_pos_.x = ladar_data_.x;
    robot_pos_.y = ladar_data_.y;

    switch (chassis_status_)
    {
    //////-----------------------------------            手操模式           ----------------------------------/////
    case CHASSIS_MANUAL_CONTROL_A:
    {
        // 模式 A：大速度手动平移 + 角速度控制。
        CHASSIS_MANUAL(1.6f, 1.6f, 3.0f);
        chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, Chassis_Target.yaw_rate);
        chassis_status_last_ = chassis_status_;
        break;
    }
    case CHASSIS_MANUAL_CONTROL_B:
    {
        // 模式 B：低速手动平移，锁当前航向。
        CHASSIS_MANUAL(0.6f, 0.6f);
        chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY);
        chassis_status_last_ = chassis_status_;
        break;
    }
    case CHASSIS_MANUAL_CONTROL_C:
    {
        // 模式 C：全向速度控制，锁当前航向。
        CHASSIS_MANUAL(1.0f, 1.0f);
        chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY);
        chassis_status_last_ = chassis_status_;
        break;
    }
    case CHASSIS_MANUAL_CONTROL_D:
    {
        // 模式 C：全向速度控制，锁当前航向。
        CHASSIS_MANUAL(0.8f, 0.8f);
        chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (0.0f * PI / 180.0f));
        chassis_status_last_ = chassis_status_;
        break;
    }
    /////-----------------------               一区            -----------------------------------/////
    case CHASSIS_AUTO_CONTROL_CB:
    {
        mode_init();
        CB_Path_Init();
        CB_Path_Check();
        if (path_line_.Is_End() == false)
        {
            curve = path_line_.get_bezier_curve();
            if (WeaponSage_Start == false && WeaponSage_End == false && WeaponSage_Back == false)
                v_plan();
            else
                Path_lock_point(curve.Get_Start_point());
            chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
        }
        else
        {
            if ((_tool_Abs(yaw - target_yaw) < 1.0f))
            {
                CHASSIS_MANUAL(0.8f, 0.8f, 1.2f);
                chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, Chassis_Target.yaw_rate);
            }
            else
            {
                Path_lock_point(Path_end_point);
                chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
            }
        }
        break;
    }

    /////---------------------               二区            -----------------------------------/////
    case CHASSIS_AUTO_CONTROL_KFS:
    {
        mode_init();
        KFS_Path_Init();
        if (KFS_flag.pause_flag == false)
        {
            if (path_line_.Is_End() == false)
            {
                curve = path_line_.get_bezier_curve();
                KFS_Path_Check();
                if (Arm_Start == false)
                    v_plan();
                else
                    Path_lock_point(curve.Get_Start_point());
                chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
            }
            else
            {
                CHASSIS_MANUAL(1.6f, 1.6f, 3.0f);
                chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, Chassis_Target.yaw_rate);
            }
        }
        else
        {
            CHASSIS_MANUAL(1.6f, 1.6f, 3.0f);
            chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, Chassis_Target.yaw_rate);
        }
        break;
    }

    //----------------------------------             CZ_新状态机              -----------------------------------//
    case CHASSIS_MANUAL_CONTROL_CZ:
    {
        CHASSIS_MANUAL(1.0f, 1.0f, 2.0f, true);
        chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, Chassis_Target.yaw_rate);
        chassis_status_last_ = chassis_status_;
        break;
    }
    case SEMI_AUIO_CZ_FIT:
    {
        static bool yaw_lock = false;
        static bool yaw_tra = false;
        mode_init();
        CZ_FIT_Path_Init();
        if (CZ_point.R1_pos[2][RB_Flag].x == curve.Get_End_point().x && CZ_point.fit_end_pos[RB_Flag].y == curve.Get_End_point().y)
        {
            yaw_tra = true;
        }
        else if (yaw_tra == true)
        {
            yaw_tra = false;
            target_yaw = (RB_Flag ? 180.0f : 0.0f);
        }

        if (CZ_flag.fit_yaw_flag == true)
        {
            if (_tool_Abs(yaw - (RB_Flag ? -90.0f : 90.0f)) < 20.0f)
            {
                target_yaw = (RB_Flag ? -90.0f : 90.0f);
                yaw_lock = true;
            }
        }

        if (path_line_.Is_End() == false)
        {
            curve = path_line_.get_bezier_curve();
            if (true)
                v_plan();
            else
                Path_lock_point(curve.Get_Start_point());
            chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
        }
        else
        {
            if ((Path_end_point.x == CZ_point.fit_end_pos[RB_Flag].x && Path_end_point.y == CZ_point.fit_end_pos[RB_Flag].y))
            {
                chassis_manual_transform();
                if (manual_transform_flag == true)
                {
                    CZ_flag.dead_cnt = 500;
                    CHASSIS_MANUAL(0.5f, 0.5f, 0.0f, true);
                    chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
                }
                else
                {
                    if (pid_dead_flag == true && CZ_flag.dead_cnt < 400)
                        CZ_flag.dead_cnt++;
                    if (CZ_flag.dead_cnt > 300 && pid_dead_flag == true)
                    {
                        Chassis_Target = {0.0f, 0.0f, 0.0f};
                        chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
                        chassis.setIdlePostureMode(jia::FourSteerChassis::Chassis::IdlePostureMode::kXPark);
                    }
                    else
                    {
                        Path_lock_point(Path_end_point);
                        chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
                    }
                }
            }
            else if (yaw_lock == true || manual_transform_flag == false)
            {

                chassis_manual_transform();
                if (CZ_flag.fit_yaw_flag == true)
                {
                    if (_tool_Abs(yaw - (RB_Flag ? -90.0f : 90.0f)) < 1.0f)
                    {
                        yaw_lock = false;
                        CZ_flag.fit_yaw_flag = false;
                    }
                }
                Path_lock_point(Path_end_point);
                chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
            }
            else
            {
                CZ_flag.dead_cnt = 0;
                CHASSIS_MANUAL(1.0f, 1.0f, 1.2f, true);
                chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, Chassis_Target.yaw_rate);
            }
        }
        break;
    }
    case SEMI_AUIO_CZ_ARM:
    {
        mode_init();
        CZ_ARM_Path_Init();
        if (path_line_.Is_End() == false)
        {
            curve = path_line_.get_bezier_curve();
            if (true)
                v_plan();
            else
                Path_lock_point(curve.Get_Start_point());
            chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
        }
        else
        {
            // 锁点后切换为半手操
            if (manual_transform_flag == false)
            {
                chassis_manual_transform();
                Path_lock_point(Path_end_point);
                chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));

                if (CZ_Arm == false && CZ_flag.R1_FB_index == 1)
                {
                    CZ_flag.R1_FB_index = 0;
                    CZ_R1_Selection_Planning();
                }
            }
            else
            {
                if (CZ_Arm == false && CZ_flag.R1_FB_index == 1)
                {
                    CZ_flag.R1_FB_index = 0;
                    CZ_R1_Selection_Planning();
                }
                CZ_flag.dead_cnt = 0;
                CHASSIS_MANUAL(1.0f, 1.0f, 0.6f, true);
                chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, Chassis_Target.yaw_rate);
            }
        }
        break;
    }
    case SEMI_AUIO_CZ_WEAPON:
    {
        mode_init();
        if (airjoy_data_.SWE == 0)
        {
            CHASSIS_MANUAL(1.0f, 1.0f, 2.0f, true);
            chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, Chassis_Target.yaw_rate);
        }
        else if (airjoy_data_.SWE == 1)
        {
            CHASSIS_MANUAL(1.0f, 1.0f, 0.0f, false);
            chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, target_yaw * PI / 180.0f);
        }
        break;
    }

    case SEMI_AUIO_CZ_ARM_Challenge:
    {
        mode_init();
        CZ_ARM_Challenge_Path_Init();

        static bool catch_flag = false;
        if (CZ_point.catch_pos[RB_Flag].x == curve.Get_End_point().x && CZ_point.catch_pos[RB_Flag].y == curve.Get_End_point().y)
        {
            catch_flag = true;
        }
        else if (catch_flag == true)
        {
            catch_flag = false;
            pid_dead_flag = false;
            CZ_Catch = true;
        }

        if (CZ_point.R1_pos[1][RB_Flag].x == curve.Get_End_point().x && CZ_point.R1_pos[1][RB_Flag].y == curve.Get_End_point().y && robot_pos_.y < 10.86f)
        {

            target_yaw = 180.0f;
        }

        if (path_line_.Is_End() == false)
        {
            curve = path_line_.get_bezier_curve();
            if (CZ_Catch == false)
                v_plan();
            else
                Path_lock_point(curve.Get_Start_point());
            chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
        }
        else
        {
            // 锁点后切换为半手操
            if (manual_transform_flag == false)
            {
                chassis_manual_transform();
                Path_lock_point(Path_end_point);
                chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
            }
            else
            {
                CZ_flag.dead_cnt = 0;
                if (CZ_Arm == false && CZ_flag.R1_FB_index == 1)
                {
                    CZ_flag.R1_FB_index = 0;
                    CZ_R1_Selection_Planning();
                }
                CHASSIS_MANUAL(1.0f, 1.0f, 1.0f, true);
                chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, Chassis_Target.yaw_rate);
            }
        }
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

// 三区地图索引复位
void OmniChassis_Setup::CZ_index_reset(void)
{
    manual_transform_flag = false;
    CZ_flag.dead_cnt = 0;
    CZ_flag.R1_FB_index = 0;
    CZ_flag.R1_RL_index = 1;
    CZ_flag.R2_pos_index = 2;
}

///////////////////                  三区条件判断             ///////////////////////

void OmniChassis_Setup::CZ_ARM_Path_Init(void)
{
    static int right_flag = 0;
    static int left_flag = 0;

    static bool up_click = false;
    static bool down_click = false;

    // 上键放置物块
    if (airjoy_data_.d_pad_up == 1 && up_click == false)
    {
        up_click = true;
        CZ_flag.R1_FB_index = 1;
        CZ_R1_Selection_Planning();
    }
    else if (airjoy_data_.d_pad_up == 0)
    {
        up_click = false;
    }

    // 下键退回准备
    if (airjoy_data_.d_pad_down == 1 && down_click == false)
    {
        down_click = true;
        CZ_flag.R1_FB_index = 0;
        CZ_R1_Selection_Planning();
    }
    else if (airjoy_data_.d_pad_down == 0)
    {
        down_click = false;
    }

    // 右摇杆往左拨,蓝场远，红场近
    if (airjoy_data_.right_x > -0.05f)
        right_flag = 1;
    else if (right_flag > 0 && airjoy_data_.right_x < -0.70f && CZ_flag.R1_FB_index == 0)
    {
        if (right_flag > 5)
        {
            if (RB_Flag == true)
            {
                if (CZ_flag.R1_RL_index > 0)
                    CZ_flag.R1_RL_index--;
            }
            else if (RB_Flag == false)
            {
                if (CZ_flag.R1_RL_index < 2)
                    CZ_flag.R1_RL_index++;
            }
            CZ_R1_Selection_Planning();
            right_flag = 0;
        }
        else
        {
            right_flag++;
        }
    }
    else if (CZ_flag.R1_FB_index == 1)
    {
        right_flag = 0;
    }

    // 右摇杆往右拨,蓝场近，红场远
    if (airjoy_data_.right_x < 0.05f)
        left_flag = 1;
    else if (left_flag > 0 && airjoy_data_.right_x > 0.70f && CZ_flag.R1_FB_index == 0)
    {
        if (left_flag > 5)
        {
            if (RB_Flag == true)
            {
                if (CZ_flag.R1_RL_index < 2)
                    CZ_flag.R1_RL_index++;
            }
            else if (RB_Flag == false)
            {
                if (CZ_flag.R1_RL_index > 0)
                    CZ_flag.R1_RL_index--;
            }
            CZ_R1_Selection_Planning();
            left_flag = 0;
        }
        else
        {
            left_flag++;
        }
    }
    else if (CZ_flag.R1_FB_index == 1)
    {
        left_flag = 0;
    }
}

void OmniChassis_Setup::CZ_FIT_Path_Init(void)
{
    static bool up_click = false;
    static bool down_click = false;
    static bool far_click = false;
    static bool near_click = false;

    // 上键合体
    if (airjoy_data_.d_pad_up == 1 && up_click == false)
    {
        up_click = true;
        CZ_FIT_WAIT_Selection_Planning();
        CZ_flag.R2_pos_index = 2;
    }
    else if (airjoy_data_.d_pad_up == 0)
    {
        up_click = false;
    }

    // 下键等待
    if (airjoy_data_.d_pad_down == 1 && down_click == false)
    {
        down_click = true;
    }
    else if (airjoy_data_.d_pad_down == 0)
    {
        down_click = false;
    }

    // 蓝场左，红场右，拿远的
    if ((RB_Flag ? (airjoy_data_.d_pad_left == 1) : (airjoy_data_.d_pad_right == 1)) && far_click == false)
    {
        far_click = true;
        if (CZ_flag.R2_pos_index > 0)
            CZ_flag.R2_pos_index--;
        CZ_FIT_R2_Selection_Planning();
    }
    else if ((RB_Flag ? (airjoy_data_.d_pad_left == 0) : (airjoy_data_.d_pad_right == 0)))
    {
        far_click = false;
    }

    // 蓝场右，红场左，拿近的
    if ((RB_Flag ? (airjoy_data_.d_pad_right == 1) : (airjoy_data_.d_pad_left == 1)) && near_click == false)
    {
        near_click = true;
        if (CZ_flag.R2_pos_index < 2)
            CZ_flag.R2_pos_index++;
        CZ_FIT_R2_Selection_Planning();
    }
    else if ((RB_Flag ? (airjoy_data_.d_pad_right == 0) : (airjoy_data_.d_pad_left == 0)))
    {
        near_click = false;
    }
}
///////////////////                  三区路径规划             ///////////////////////

void OmniChassis_Setup::CZ_R1_Selection_Planning(void)
{
    manual_transform_flag = false;
    if (robot_pos_.y < 10.02f || robot_pos_.y > 11.6f || robot_pos_.x < 0.0f || robot_pos_.x > 6.0f)
        return;
    // 夹杆流程只规划起点到固定终点的简化路径。
    target_yaw = RB_Flag ? 180.0f : 0.0f;
    path_line_.Reset();
    path_line_.plan_reset();

    pid_dead_flag = false;
    path_line_.Add_Start_Point(robot_pos_);
    if (CZ_flag.R1_FB_index == 1)
    {
        CZ_Arm = true;
        path_line_.Add_End_Point({CZ_point.R1_pos[CZ_flag.R1_RL_index][RB_Flag].x + CZ_point.set_skew * (RB_Flag ? 1.0f : (-1.0f)), robot_pos_.y}, path_param.end);
    }
    else
    {
        CZ_Arm = false;
        path_line_.Add_End_Point(CZ_point.R1_pos[CZ_flag.R1_RL_index][RB_Flag], path_param.end);
    }
    Path_end_point = path_line_.Get_End_Point();
}

void OmniChassis_Setup::CZ_FIT_WAIT_Selection_Planning(void)
{
    manual_transform_flag = false;
    if (robot_pos_.y < 10.02f || robot_pos_.y > 11.6f || robot_pos_.x < 0.0f || robot_pos_.x > 6.0f)
        return;
    path_line_.Reset();
    path_line_.plan_reset();
    // 合体地点
    path_line_.Add_Start_Point(robot_pos_);
    if (_tool_Abs(_tool_Abs(yaw) - (RB_Flag ? 180.0f : 0.0f)) > 20.0f && (RB_Flag ? (robot_pos_.x > 4.70f) : (robot_pos_.x < (6.0f - 4.70f))))
    {
        path_line_.Add_Point({CZ_point.R1_pos[2][RB_Flag].x, CZ_point.fit_end_pos[RB_Flag].y + 0.2f}, path_param.cz);
    }
    else
    {
        target_yaw = (RB_Flag ? 180.0f : 0.0f);
    }
    path_line_.Add_End_Point(CZ_point.fit_end_pos[RB_Flag], path_param.cz);
    Path_end_point = path_line_.Get_End_Point();
}
void OmniChassis_Setup::CZ_FIT_R2_Selection_Planning(void)
{
    manual_transform_flag = false;
    if (robot_pos_.y < 10.02f || robot_pos_.y > 11.6f || robot_pos_.x < 0.0f || robot_pos_.x > 6.0f)
        return;
    // 夹杆流程只规划起点到固定终点的简化路径。
    if (_tool_Abs(yaw - (RB_Flag ? -90.0f : 90.0f)) < 20.0f)
    {
        target_yaw = (RB_Flag ? -90.0f : 90.0f);
    }
    else
    {
        CZ_flag.fit_yaw_flag = true;
    }

    path_line_.Reset();
    path_line_.plan_reset();

    // R2放置物块
    path_line_.Add_Start_Point(robot_pos_);
    path_line_.Add_End_Point(CZ_point.R2_pos[CZ_flag.R2_pos_index][RB_Flag], path_param.R2);
    Path_end_point = path_line_.Get_End_Point();
}

///////////////////                       挑战赛三区相关              /////////////////////////////////////

void OmniChassis_Setup::CZ_ARM_Challenge_Path_Init(void)
{
    static int right_flag = 0;
    static int left_flag = 0;

    static bool up_click = false;
    static bool down_click = false;
    if (flag == 1)
    {
        if (robot_pos_.x < 0.0f || robot_pos_.x > 6.0f || robot_pos_.y > 10.0f || robot_pos_.y < 0.0f)
            return;
        else
        {
            manual_transform_flag = false;
            CZ_flag.R1_RL_index = 2;
            flag = 0;
            flag_reset();
            path_line_.Reset();
            path_line_.plan_reset();
            path_line_.Add_Start_Point(robot_pos_);
            path_line_.Add_Point(CZ_point.uphill_pos[RB_Flag], path_param.up);
            path_line_.Add_Point({CZ_point.uphill_transitiont_pos[RB_Flag].x + (RB_Flag ? (-0.2f) : (0.2f)), CZ_point.uphill_transitiont_pos[RB_Flag].y}, path_param.line);
            path_param.curve.targetPos = 4.0f;
            path_line_.Add_Point({CZ_point.uphill_transitiont_pos[RB_Flag].x + (RB_Flag ? (0.3f) : (-0.3f)), CZ_point.uphill_transitiont_pos[RB_Flag].y - 0.5f}, path_param.curve);
            path_line_.Add_Point({CZ_point.uphill_transitiont_pos_1[RB_Flag].x, CZ_point.uphill_transitiont_pos_1[RB_Flag].y + 0.2f}, path_param.line);
            if (RB_Flag == true)
            {
                path_param.curve.targetPos = 1.0f;
            }
            else if (RB_Flag == false)
            {
                path_param.curve.targetPos = 2.0f;
            }
            path_line_.Add_Point({CZ_point.uphill_transitiont_pos_1[RB_Flag].x + (RB_Flag ? (0.5f) : (-0.5f)), CZ_point.uphill_transitiont_pos_1[RB_Flag].y}, path_param.curve);
            path_line_.Add_End_Point(CZ_point.R1_pos[CZ_flag.R1_RL_index][RB_Flag], path_param.cz);
            path_param.curve.targetPos = 999.0f;
            Path_end_point = path_line_.Get_End_Point();
        }
    }

    // 上键放置物块
    if (airjoy_data_.d_pad_up == 1 && up_click == false)
    {
        up_click = true;
        CZ_flag.R1_FB_index = 1;
        CZ_R1_Selection_Planning();
    }
    else if (airjoy_data_.d_pad_up == 0)
    {
        up_click = false;
    }

    // 下键退回准备
    if (airjoy_data_.d_pad_down == 1 && down_click == false)
    {
        down_click = true;
        CZ_Catch_Selection_Planning();
    }
    else if (airjoy_data_.d_pad_down == 0)
    {
        down_click = false;
    }

    // 右摇杆往左拨,蓝场远，红场近
    if (airjoy_data_.right_x > -0.05f)
        right_flag = 1;
    else if (right_flag > 0 && airjoy_data_.right_x < -0.80f && CZ_flag.R1_FB_index == 0)
    {
        if (right_flag > 5)
        {
            if (RB_Flag == true)
            {
                if (CZ_flag.R1_RL_index > 0)
                    CZ_flag.R1_RL_index--;
            }
            else if (RB_Flag == false)
            {
                if (CZ_flag.R1_RL_index < 2)
                    CZ_flag.R1_RL_index++;
            }
            CZ_R1_Selection_Planning();
            right_flag = 0;
        }
        else
        {
            right_flag++;
        }
    }
    else if (CZ_flag.R1_FB_index == 1)
    {
        right_flag = 0;
    }

    // 右摇杆往右拨,蓝场近，红场远
    if (airjoy_data_.right_x < 0.05f)
        left_flag = 1;
    else if (left_flag > 0 && airjoy_data_.right_x > 0.80f && CZ_flag.R1_FB_index == 0)
    {
        if (left_flag > 5)
        {
            if (RB_Flag == true)
            {
                if (CZ_flag.R1_RL_index < 2)
                    CZ_flag.R1_RL_index++;
            }
            else if (RB_Flag == false)
            {
                if (CZ_flag.R1_RL_index > 0)
                    CZ_flag.R1_RL_index--;
            }
            CZ_R1_Selection_Planning();
            left_flag = 0;
        }
        else
        {
            left_flag++;
        }
    }
    else if (CZ_flag.R1_FB_index == 1)
    {
        left_flag = 0;
    }
}

void OmniChassis_Setup::CZ_Catch_Selection_Planning(void)
{
    manual_transform_flag = false;
    if (robot_pos_.y < 10.02f || robot_pos_.y > 11.6f || robot_pos_.x < 0.0f || robot_pos_.x > 6.0f)
        return;
    target_yaw = -90.0f;
    CZ_flag.R1_RL_index = 1;
    flag_reset();
    path_line_.Reset();
    path_line_.plan_reset();
    path_line_.Add_Start_Point(robot_pos_);
    path_line_.Add_Point(CZ_point.catch_pos[RB_Flag], path_param.cz);
    path_line_.Add_End_Point(CZ_point.R1_pos[CZ_flag.R1_RL_index][RB_Flag], path_param.cz);
    Path_end_point = path_line_.Get_End_Point();
}
