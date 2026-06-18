#include "omni_chassisSetup.h"
extern Chassis chassis;
float CB_yaw = 89.0f;
void OmniChassis_Setup::CB_Path_Check(void)
{
    static bool Selection_flag = false;
     static bool Retreat_flag = false;
    if (CB_point.CB_Selection_pos.x == curve.Get_End_point().x && CB_point.CB_Selection_pos.y == curve.Get_End_point().y)
    {
        Selection_flag = true;
    }
    else if (Selection_flag == true)
    {
        Selection_flag = false;
        pid_dead_flag = false;
        WeaponSage_Start = true;
    }
    if (airjoy_data_.SWA == 0x00)
    {
        if (CB_point.CB_End_pos.x == curve.Get_End_point().x && CB_point.CB_End_pos.y == curve.Get_End_point().y&&path_line_.Is_End() == false)
        {
            Retreat_flag = true;
        }
        else if (Retreat_flag == true&&path_line_.Is_End() == true)
        {
			target_yaw = 90.0f;
            Retreat_flag = false;
            pid_dead_flag = false;
            WeaponSage_End = true;
        }
    }
    else if (airjoy_data_.SWA == 0x01)
    {
        if (CB_point.CB_transition_pos.x == curve.Get_End_point().x && CB_point.CB_transition_pos.y == curve.Get_End_point().y)
        {
            Retreat_flag = true;
        }
        else if (Retreat_flag == true)
        {
			target_yaw = 90.0f;
            Retreat_flag = false;
            pid_dead_flag = false;
            WeaponSage_End = true;
        }
    }
}

void OmniChassis_Setup::CB_Selection_Planning(void)
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
    path_line_.Add_Point(CB_point.CB_Selection_pos, path_param.cb);

    // 相机流程
    if (airjoy_data_.SWA == 0x00)
    {
        path_line_.Add_End_Point(CB_point.CB_End_pos, path_param.cb);
    }
    else if (airjoy_data_.SWA == 0x01)
    {
        path_line_.Add_Point(CB_point.CB_transition_pos, path_param.cb);
        path_line_.Add_End_Point(CB_point.CB_welt_pos, path_param.cb);
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
        CHASSIS_MANUAL(1.6f, 3.0f);
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
            if (airjoy_data_.SWA == 0x00)
            {
				if(pid_dead_flag == true)
				{
					CHASSIS_MANUAL(1.5f, 0.0f, false);
					chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));

					
				}
				else if(pid_dead_flag == false)
				{
					Path_lock_point(Path_end_point);
					chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));

				}
                            }
            else if (airjoy_data_.SWA == 0x01)
            {
				if(pid_dead_flag == true)
				{
					CHASSIS_MANUAL(1.5f, 1.5f);
					chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, Chassis_Target.yaw_rate);

					
				}
				else if(pid_dead_flag == false)
				{
					Path_lock_point(Path_end_point);
					chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));

				}

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
            KFS_Path_Check();
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
            CZ_index_reset();
        }

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
            CZ_index_reset();
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
            CZ_index_reset();
        }

        CZ_state_switch();
        switch (CZ_state)
        {
        case MANUAL:
        {
            CZ_init();
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
                // 锁点后切换为半手操
                if (pid_dead_flag == false)
                {
                    Path_lock_point(Path_end_point);
                }
                else
                {
                    if (_tool_Abs(airjoy_data_.left_x) > 0.05f)
                        Chassis_Target.VY = airjoy_data_.left_x * 0.3f * this->is_chassis_reverse_ * RB_Flag ? (-1) : 1;
                    else
                        Chassis_Target.VY = 0.0f;
                    Chassis_Target.VX = 0.0f;
                }
            }
            chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, (target_yaw * PI / 180.0f));
            chassis_status_last_ = chassis_status_;
            CZ_state_last = CZ_state;
            break;
        }
        case SEMI_AUIO_WEAPON:
        {
            CZ_init();
            // control a
            if (false)
            {
                CHASSIS_MANUAL(2.0f, 2.0f);
                chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, Chassis_Target.yaw_rate);
            }
            else if (false) // control b
            {
                CHASSIS_MANUAL(1.0f, 1.0f);
                chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, Chassis_Target.VX, Chassis_Target.VY, CB_yaw * PI / 180.0f);
            }
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

// 三区地图索引复位
void OmniChassis_Setup::CZ_index_reset(void)
{
    CZ_flag.fit_pos_index = 0;
    CZ_flag.R1_FB_index = 0;
    CZ_flag.R1_RL_index = -1;
    CZ_flag.R2_pos_index = -1;
}
void OmniChassis_Setup::CZ_init(void)
{
    if (CZ_state_last != CZ_state)
    {
        flag_reset();
        path_line_.Reset();
        path_line_.plan_reset();
        Path_end_point = robot_pos_;
        CZ_index_reset();
    }
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////       新代码主要服务于新遥控       //////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
        CZ_R2_Selection_Planning();
    }
    else if (false) // 下键等待
    {
        CZ_flag.fit_pos_index = 0;
        CZ_R2_Selection_Planning();
    }
    else if ((false && RB_Flag == true) || (false && RB_Flag == false)) // 蓝场左，红场右
    {
        if (CZ_flag.R2_pos_index < 0)
            CZ_flag.R2_pos_index--;
        CZ_R2_Selection_Planning();
    }
    else if ((false && RB_Flag == true) || (false && RB_Flag == false)) // 蓝场右，红场左
    {
        if (CZ_flag.R2_pos_index > 2)
            CZ_flag.R2_pos_index++;
        CZ_R2_Selection_Planning();
    }
}

void OmniChassis_Setup::CZ_ARM_Path_Init(void)
{
    static int right_flag = 0;
    static int left_flag = 0;
    static bool first_flag = true;
    if (false) // 上键放置
    {
        CZ_flag.R1_FB_index = 1;
        CZ_R1_Selection_Planning();
    }
    else if (false) // 下键等待
    {
        CZ_flag.R1_FB_index = 0;
        CZ_R1_Selection_Planning();
    }
    // 右摇杆往右拨
    if (airjoy_data_.right_x > -0.05f)
        right_flag = 1;
    else if (right_flag > 0 && airjoy_data_.right_x < -0.80f && CZ_flag.R1_FB_index == 0)
    {
        if (right_flag > 5)
        {
            if (first_flag)
            {
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
    // 右摇杆往左拨
    if (airjoy_data_.right_x < 0.05f)
        left_flag = 1;
    else if (left_flag > 0 && airjoy_data_.right_x > 0.80f && CZ_flag.R1_FB_index == 0)
    {
        if (left_flag > 5)
        {
            if (first_flag)
            {
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
