#include "omni_chassisSetup.h"
extern Chassis chassis;

void OmniChassis_Setup::CB_Path_Check(void)
{
    if (CB_point.CB_Selection_pos[RB_Flag].x == curve.Get_End_point().x && CB_point.CB_Selection_pos[RB_Flag].y == curve.Get_End_point().y)
    {
        CB_flag.Selection_flag = true;
    }
    else if (CB_flag.Selection_flag == true)
    {
        CB_flag.Selection_flag = false;
        pid_dead_flag = false;
        WeaponSage_Start = true;
    }

#if !USE_RC10_AIRJOY
    if (airjoy_data_.SWA == 0x00)
    {
        if (CB_point.CB_End_pos.x == curve.Get_End_point().x && CB_point.CB_End_pos.y == curve.Get_End_point().y && path_line_.Is_End() == false)
        {
            CB_flag.Retreat_flag = true;
        }
        else if (CB_flag.Retreat_flag == true)
        {
            target_yaw = 90.0f;
            CB_flag.Retreat_flag = false;
            pid_dead_flag = false;
            WeaponSage_End = true;
        }
    }
    else if (airjoy_data_.SWA == 0x01)
    {
        if (CB_point.CB_transition_pos.x == curve.Get_End_point().x && CB_point.CB_transition_pos.y == curve.Get_End_point().y)
        {
            CB_flag.Retreat_flag = true;
        }
        else if (CB_flag.Retreat_flag == true)
        {
            target_yaw = 90.0f;
            CB_flag.Retreat_flag = false;
            pid_dead_flag = false;
            WeaponSage_End = true;
        }
    }
#else
    if (airjoy_data_.SWC == 0x00)
    {
        if (CB_point.CB_End_pos[RB_Flag].x == curve.Get_End_point().x && CB_point.CB_End_pos[RB_Flag].y == curve.Get_End_point().y && path_line_.Is_End() == false)
        {
            CB_flag.Retreat_flag = true;
			if(robot_pos_.y>CB_point.spin_y&&WeaponSage_Start == false)
			{
				if (RB_Flag)
                target_yaw = 90.0f;
            else
                target_yaw = -90.0f;
				
			}
        }
        else if (CB_flag.Retreat_flag == true)
        {
            CB_flag.Retreat_flag = false;
            WeaponSage_End = true;
        }
    }
    else if (airjoy_data_.SWC == 0x01)
    {
        if (CB_point.CB_transition_pos[RB_Flag].x == curve.Get_End_point().x && CB_point.CB_transition_pos[RB_Flag].y == curve.Get_End_point().y)
        {
			if(robot_pos_.y>CB_point.spin_y&&WeaponSage_Start == false)
			{
				if (RB_Flag)
                target_yaw = 90.0f;
            else
                target_yaw = -90.0f;
				
			}
        }
        
        if (CB_point.CB_welt_pos[RB_Flag].x == curve.Get_End_point().x && CB_point.CB_welt_pos[RB_Flag].y == curve.Get_End_point().y&& path_line_.Is_End() == false)
        {
            CB_flag.Retreat_flag = true;
        }
        else if (CB_flag.Retreat_flag == true)
        {
            CB_flag.Retreat_flag = false;
            WeaponSage_End = true;
        }
    }
#endif
}

void OmniChassis_Setup::CB_Selection_Planning(void)
{
	// 只能在一区和进行启动
    if (robot_pos_.x < 0.0f || robot_pos_.x > 6.0f || robot_pos_.y > 2.8f || robot_pos_.y < 0.0f)
       return ;
    // 夹杆流程只规划起点到固定终点的简化路径。
    target_yaw = 0.0f;
    path_line_.Reset();
    path_line_.plan_reset();

    // 夹杆路径
    path_line_.Add_Start_Point(robot_pos_,0.02f);
    if (robot_pos_.y < CB_point.CB_Selection_pos[RB_Flag].y)
    {
        path_line_.Add_Point(CB_point.CB_Start_pos[RB_Flag], path_param.line);
    }
    path_line_.Add_Point(CB_point.CB_Selection_pos[RB_Flag], path_param.cb);

#if !USE_RC10_AIRJOY
    // 相机流程
    if (airjoy_data_.SWA == 0x00)
    {
        path_line_.Add_End_Point(CB_point.CB_End_pos, path_param.end);
    }
    else if (airjoy_data_.SWA == 0x01)
    {
        path_line_.Add_Point(CB_point.CB_transition_pos, path_param.R2);
        path_line_.Add_Point(CB_point.CB_transition_pos_1, path_param.line);
        path_line_.Add_End_Point(CB_point.CB_welt_pos, path_param.R2);
    }
#else
    // 相机流程
    if (airjoy_data_.SWC == 0x00)
    {
        path_line_.Add_End_Point(CB_point.CB_End_pos[RB_Flag], path_param.end);
    }
    else if (airjoy_data_.SWC == 0x01)
    {
        path_line_.Add_Point(CB_point.CB_transition_pos[RB_Flag], path_param.speed);
        path_line_.Add_End_Point(CB_point.CB_welt_pos[RB_Flag], path_param.speed);
    }
#endif
    Path_end_point = path_line_.Get_End_Point();
}

///////////////////////          主循环         ////////////////////////////////////////

void OmniChassis_Setup::loop()
{
    // 未初始化时不进入控制流程。
    if (!init_flag)
        return;

#if !USE_RC10_AIRJOY
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
#else
    communication::Lora_communication::GetInstance()->update_airjoy_data(&airjoy_data_);
#endif
    yaw = Locate_Setup::getInstance()->get_yaw_from_position();
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
    /////-----------------------------               一区            -----------------------------------/////
    case CHASSIS_AUTO_CONTROL_CB:
    {
        mode_init();
        if (flag == 1)
        {
            flag = 0;
            flag_reset();
            CB_Selection_Planning();
        }
        CB_Path_Check();
        if (path_line_.Is_End() == false)
        {
            curve = path_line_.get_bezier_curve();
            if (WeaponSage_Start == false && WeaponSage_End == false)
                v_plan();
            else
                Path_lock_point(curve.Get_Start_point());
            chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
        }
        else
        {
            if((_tool_Abs(yaw - target_yaw) < 1.0f))
            {
                static bool end = false;
                if (airjoy_data_.left_x > 0.9f)
                {
                    Chassis_Target.VX = 0.0f;
                    end = true;
                }
                else if (airjoy_data_.left_x > 0.8f && airjoy_data_.left_x < 0.9f && end == false)
                    Chassis_Target.VX = 2.0f;
                else if (_tool_Abs(airjoy_data_.left_x) > 0.1f && end == false)
                    Chassis_Target.VX = airjoy_data_.left_x * 1.5f * this->is_chassis_reverse_;
                else
                {
                    Chassis_Target.VX = 0.0f;
                    if (_tool_Abs(airjoy_data_.left_x) < 0.1f)
                        end = false;
                }
                if (_tool_Abs(airjoy_data_.left_y) > 0.1f && end == false)
                    Chassis_Target.VY = airjoy_data_.left_y * 1.5f * this->is_chassis_reverse_;
                else
                    Chassis_Target.VY = 0.0f;
                if (_tool_Abs(airjoy_data_.right_x) > 0.1f)
                    Chassis_Target.yaw_rate = airjoy_data_.right_x * 1.5f * (-1.0f);
                else
                    Chassis_Target.yaw_rate = 0.0f;

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

    /////-----------------------------               二区            -----------------------------------/////
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

#if USE_RC10_AIRJOY
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
        mode_init();
        CZ_FIT_Path_Init();
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
            if (pid_dead_flag == false)
            {
                Path_lock_point(Path_end_point);
                chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
            }
            else
            {
                CHASSIS_MANUAL(1.0f, 1.0f, 0.6f, true);
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
            if (pid_dead_flag == false)
            {
                Path_lock_point(Path_end_point);
                chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
            }
            else
            {
                if (CZ_Arm == false && CZ_flag.R1_FB_index == 1)
                {
                    CZ_flag.R1_FB_index = 0;
                    CZ_R1_Selection_Planning();
                }
                CHASSIS_MANUAL(1.0f, 1.0f, 0.6f);
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

#else
    //----------------------------------             CZ_R1              --------------------------------------//
    case CHASSIS_AUTO_CONTROL_CZ_R1:
    {
        static int far_flag = 0;
        static int near_flag = 0;
        mode_init();

        if (airjoy_data_.right_x > -0.05f)
            far_flag = 1;
        else if (far_flag > 0 && airjoy_data_.right_x < -0.80f && CZ_flag.R1_FB_index == 0)
        {
            if (far_flag > 5)
            {
                if (CZ_flag.R1_RL_index < 2)
                    CZ_flag.R1_RL_index++;
                CZ_R1_Selection_Planning();
                far_flag = 0;
            }
            else
            {
                far_flag++;
            }
        }
        else if (CZ_flag.R1_FB_index == 1)
        {
            far_flag = 0;
        }

        if (airjoy_data_.right_x < 0.05f)
            near_flag = 1;
        else if (near_flag > 0 && airjoy_data_.right_x > 0.80f && CZ_flag.R1_FB_index == 0)
        {
            if (near_flag > 5)
            {
                if (CZ_flag.R1_RL_index > 0)
                    CZ_flag.R1_RL_index--;
                CZ_R1_Selection_Planning();
                near_flag = 0;
            }
            else
            {
                near_flag++;
            }
        }
        else if (CZ_flag.R1_FB_index == 1)
        {
            near_flag = 0;
        }

        if (flag == 1)
        {
            flag_reset();
            CZ_flag.R1_FB_index = (CZ_flag.R1_FB_index + 1) % 2;
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
        break;
    }

    //----------------------------------             CZ_R2              --------------------------------------//
    case CHASSIS_AUTO_CONTROL_CZ_R2:
    {
        mode_init();
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
        break;
    }
#endif
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
    CZ_flag.fit_pos_index = 1;
    CZ_flag.R1_FB_index = 0;
    CZ_flag.R1_RL_index = 0;
    CZ_flag.R2_pos_index = -1;
}

void OmniChassis_Setup::CZ_R1_Selection_Planning(void)
{
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
        CZ_flag.fit_pos_index = 1;
        CZ_FIT_WAIT_Selection_Planning();
    }
    else if (airjoy_data_.d_pad_up == 0)
    {
        up_click = false;
    }

    // 下键等待
    if (airjoy_data_.d_pad_down == 1 && down_click == false)
    {
        down_click = true;
        CZ_flag.fit_pos_index = 0;
        CZ_FIT_WAIT_Selection_Planning();
    }
    else if (airjoy_data_.d_pad_down == 0)
    {
        down_click = false;
    }

    // 蓝场左，红场右，拿远的
    if (((airjoy_data_.d_pad_left == 1 && RB_Flag == true) || (airjoy_data_.d_pad_right == 1 && RB_Flag == false)) && far_click == false)
    {
        far_click = true;
        if (CZ_flag.R2_pos_index > 0)
            CZ_flag.R2_pos_index--;
        CZ_FIT_R2_Selection_Planning();
    }
    else if ((airjoy_data_.d_pad_left == 0 && RB_Flag == true) || (airjoy_data_.d_pad_right == 0 && RB_Flag == false))
    {
        far_click = false;
    }

    // 蓝场右，红场左，拿近的
    if (((airjoy_data_.d_pad_right == 1 && RB_Flag == true) || (airjoy_data_.d_pad_left == 1 && RB_Flag == false)) && near_click == false)
    {
        near_click = true;
        if (CZ_flag.R2_pos_index < 2)
            CZ_flag.R2_pos_index++;
        CZ_FIT_R2_Selection_Planning();
    }
    else if ((airjoy_data_.d_pad_right == 0 && RB_Flag == true) || (airjoy_data_.d_pad_left == 0 && RB_Flag == false))
    {
        near_click = false;
    }
}

void OmniChassis_Setup::CZ_ARM_Path_Init(void)
{
    static int right_flag = 0;
    static int left_flag = 0;
    static bool first_flag = true;

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
    else if (right_flag > 0 && airjoy_data_.right_x < -0.80f && CZ_flag.R1_FB_index == 0)
    {
        if (right_flag > 5)
        {
            if (first_flag)
            {
                first_flag = false;
                CZ_flag.R1_RL_index = 1;
            }
            else
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
            if (first_flag)
            {
                first_flag = false;
                CZ_flag.R1_RL_index = 1;
            }
            else
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
void OmniChassis_Setup::CZ_FIT_WAIT_Selection_Planning(void)
{
    if (robot_pos_.y < 10.02f || robot_pos_.y > 11.6f || robot_pos_.x < 0.0f || robot_pos_.x > 6.0f)
        return;
    // 夹杆流程只规划起点到固定终点的简化路径。
    target_yaw = -90.0f;
    path_line_.Reset();
    path_line_.plan_reset();

    // 合体地点和等待地点的切换
    path_line_.Add_Start_Point(robot_pos_);
    path_line_.Add_Point(CZ_point.fit_pos[2][RB_Flag], path_param.line);
    path_line_.Add_End_Point(CZ_point.fit_pos[CZ_flag.fit_pos_index][RB_Flag], path_param.end);
    Path_end_point = path_line_.Get_End_Point();
}
void OmniChassis_Setup::CZ_FIT_R2_Selection_Planning(void)
{
    if (robot_pos_.y < 10.02f || robot_pos_.y > 11.6f || robot_pos_.x < 0.0f || robot_pos_.x > 6.0f)
        return;
    // 夹杆流程只规划起点到固定终点的简化路径。
    target_yaw = -90.0f;
    path_line_.Reset();
    path_line_.plan_reset();

    // R2放置物块
    path_line_.Add_Start_Point(robot_pos_);
    path_line_.Add_End_Point(CZ_point.R2_pos[CZ_flag.R2_pos_index][RB_Flag], path_param.R2);
    Path_end_point = path_line_.Get_End_Point();
}




/*
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

*/