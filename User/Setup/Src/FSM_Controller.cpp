#include "FSM_Controller.h"
#include "chassis_swerve_task_demo.h"

int8_t test_led = 0;
float x_, y_, yaw_;
bool claw1, claw2, claw3;
void FSM_Controller::loop()
{
    if (!init_flag_)
        return;

    if (!test_led && test_led < 100)
    {
        Serial1Protocol::getInstance()->sendStop();
        test_led = 101;
    }
    else if (test_led != 0 && test_led < 100)
    {
        Serial1Protocol::getInstance()->send_cmd_to_R2(test_led);
        test_led = 101;
    }

#if !USE_RC10_AIRJOY
    CrsfReceiver::GetInstance(&huart7)->send_kfsandSpear(crsf_send_s.rsf_send_data.kfs1, crsf_send_s.rsf_send_data.kfs2,
                                                         crsf_send_s.rsf_send_data.Spear);
    CrsfReceiver::GetInstance(&huart7)->process();
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
#else

    // 设置红蓝场：遥控器反馈数据 1为蓝 2为红， 
    MF_AutoCtrler::set_color(airjoy_data_.color == 1 ? 1 : 0);

    communication::Lora_communication::GetInstance()->Task_Process();
    communication::Lora_communication::GetInstance()->Tim_It_Process();

    communication::Lora_communication::GetInstance()->update_airjoy_data(&airjoy_data_);

    // communication::Lora_communication::GetInstance()->send_robot_pos(x_, y_, yaw_);
    communication::Lora_communication::GetInstance()->send_robot_pos(
        Locate_Setup::getInstance()->get_RobotPos_inWorld().x,
        Locate_Setup::getInstance()->get_RobotPos_inWorld().y,
        Locate_Setup::getInstance()->get_RobotPos_inWorld().yaw);
    // communication::Lora_communication::GetInstance()->send_claw_status(claw1, claw2, claw3);

    communication::Lora_communication::GetInstance()->send_claw_status(
        weaponSage_setup_->is_claw_close(1),
        weaponSage_setup_->is_claw_close(2),
        weaponSage_setup_->is_claw_close(3));

    communication::Lora_communication::GetInstance()->send_sucker_status(
        arm_setup_->getSuckerStatus() == Sucker_Status_E::SUCK ? true : false);

    communication::Lora_communication::GetInstance()->set_robot_KFS_want_place(
        KStarget.KFS[0], KStarget.KFS[1], KStarget.KFS[2]);

    communication::Lora_communication::GetInstance()->send_robot_spear(
        KStarget.Spear[0], KStarget.Spear[1], KStarget.Spear[2]);

    communication::Lora_communication::GetInstance()->send_robot_kfs_keepplace(
        arm_setup_->get_store_side() == Store_MANUAL_E::INSIDE ? 0 : 1);

    KStarget.KFS[0] = airjoy_data_.KFS1_1;
    KStarget.KFS[1] = airjoy_data_.KFS1_2;
    KStarget.KFS[2] = airjoy_data_.KFS1_3;

#endif

#if !USE_RC10_AIRJOY
    switch (airjoy_data_.SWB)
    {
    case 0x00:
        robot_status_ = ALL_STOP;
        if (airjoy_data_.SWA == 0x01)
        {
            switch (airjoy_data_.SWC)
            {
            case 0x00:
                Stop_set_stauts = RELOCATE;
                break;
            case 0x01:
                Stop_set_stauts = SET_KFS;
                break;

            case 0x02:
                Stop_set_stauts = SET_SPEAR;
                break;
            }
        }
        else
            Stop_set_stauts = NONE;

        break;

    case 0x01:
        robot_status_ = MANUAL_CONTROL;
        break;

    case 0x02:
        // if(airjoy_data_.SWA == 0x01)
        //     robot_status_ = DEBUG_MODE;
        // else
        robot_status_ = AUTO_CONTROL;
        break;
    }
#else
    switch (airjoy_data_.SWA)
    {
    case 0x00:
        robot_status_ = ALL_STOP;
        break;

    case 0x01:
        robot_status_ = MANUAL_CONTROL;
        break;

    case 0x02:
        robot_status_ = AUTO_CONTROL;
        break;
    }

#endif

    // if(arm_setup_->isArmcalibrated() == false )
    if (arm_setup_->isArmcalibrated() == false || weaponSage_setup_->isWeaponSageCalibrated() == false)
    {
        robot_status_ = ALL_STOP;
    }

#if DEBUG_SHIT
    robot_status_ = ALL_STOP;
#endif

    switch (robot_status_)
    {
    case ALL_STOP:
        all_stop();
        break;

    case MANUAL_CONTROL:
        // 手操模式
        manual_ctrl();
        break;

    case AUTO_CONTROL:
        // 自动模式
        auto_ctrl();
        break;
    case DEBUG_MODE:
        // 调试模式
        debug();
        break;

    default:
        break;
    }

#if USE_RC10_AIRJOY

    static bool is_RB_click = 0;
    if (airjoy_data_.RB == 1 && is_RB_click == 0 && airjoy_data_.page != 0x01)
    {
        is_RB_click = 1;
        arm_setup_->set_store_side(
            arm_setup_->get_store_side() == Store_MANUAL_E::INSIDE ? OUTSIDE : INSIDE);
    }
    else if (airjoy_data_.RB == 0x00)
    {
        is_RB_click = 0;
    }

    // relocate
    if (robot_status_ == ALL_STOP && airjoy_data_.SWB == 0x01 && airjoy_data_.SWE == 0x00 && airjoy_data_.page != 0x01)
    {
        static bool is_click = 0;
        if (airjoy_data_.LB == 1 && !is_click)
        {
            Locate_Setup::getInstance()->Relocte_ToLader();
            is_click = true;
        }
        else if (airjoy_data_.LB == 0)
        {
            is_click = false;
        }
        communication::Lora_communication::GetInstance()->send_robot_mode(SEND_RELOCATE_LIDAR);
    }
    else if (robot_status_ == ALL_STOP && airjoy_data_.SWB == 0x01 && airjoy_data_.SWE == 0x01)
    {
        // 设置target 矛杆
        communication::Lora_communication::GetInstance()->send_robot_mode(SEND_SET_SPEAR);

        static bool is_click = 0;
        if (airjoy_data_.d_pad_down == 0x01 && is_click == 0)
        {
            is_click = 1;
            for (int i = 0; i < 3; i++)
            {
                KStarget.Spear[i] = 0;
            }
        }
        else if (airjoy_data_.d_pad_left == 0x01 && is_click == 0)
        {
            is_click = 1;
            KStarget.Spear[0] = 1;
        }
        else if (airjoy_data_.d_pad_up == 0x01 && is_click == 0)
        {
            is_click = 1;
            KStarget.Spear[1] = 1;
        }
        else if (airjoy_data_.d_pad_right == 0x01 && is_click == 0)
        {
            is_click = 1;
            KStarget.Spear[2] = 1;
        }
        else if (airjoy_data_.d_pad_down == 0x00 && airjoy_data_.d_pad_left == 0x00 && airjoy_data_.d_pad_right == 0x00 && airjoy_data_.d_pad_up == 0x00)
        {
            is_click = 0;
        }
        else
        {
            is_click = 0;
        }

        static bool is_LB_click = 0;
        static float cb_sum_x = 0.0f;
        static float cb_sum_y = 0.0f;
        static uint16_t cb_sample_cnt = 0;
        static bool cb_collecting = false;

        if (airjoy_data_.LB == 1 && is_LB_click == 0) // LB按下边沿 → 开始采集
        {
            is_LB_click = 1;
            cb_collecting = true;
            cb_sum_x = 0.0f;
            cb_sum_y = 0.0f;
            cb_sample_cnt = 0;
        }
        else if (airjoy_data_.LB == 0)
        {
            is_LB_click = 0;
        }

        if (cb_collecting)
        {
            cb_sum_x += Locate_Setup::getInstance()->get_RobotPos_inWorld().x;
            cb_sum_y += Locate_Setup::getInstance()->get_RobotPos_inWorld().y;
            cb_sample_cnt++;

            if (cb_sample_cnt >= 300)
            {
                chassis_setup_->reset_CB_point(
                    cb_sum_x / 300.0f,
                    cb_sum_y / 300.0f);
                cb_collecting = false;
            }
        }
    }
    else if (robot_status_ == ALL_STOP && airjoy_data_.SWB == 0x00 && airjoy_data_.SWE == 0x00)
    {
        communication::Lora_communication::GetInstance()->send_robot_mode(SEND_ALL_STOP);
    }
#endif
    if (KStarget != last_KStarget)
    {
        chassis_setup_->set_KFS(KStarget.KFS[0], KStarget.KFS[1], KStarget.KFS[2]);
        arm_setup_->set_TargetKFS(KStarget.KFS[0], KStarget.KFS[1], KStarget.KFS[2]);
        weaponSage_setup_->set_claw_flag(
            KStarget.Spear[0], KStarget.Spear[1], KStarget.Spear[2]);
    }

    last_KStarget = KStarget;

#if USE_RC10_AIRJOY
    set_cmd_to_R2();
#endif
}

#if USE_RC10_AIRJOY

SEND_CMD_TO_R2 now_cmd;
SEND_CMD_TO_R2 send_cmd;
void FSM_Controller::set_cmd_to_R2()
{
    if (airjoy_data_.page != 0x01)
    {
        if(cmd_to_r2_cnt < airjoy_data_.recv_command_total_cnt)
        {
            if (airjoy_data_.recv_command_command != 0)
            {
                Serial1Protocol::getInstance()->send_cmd_to_R2(airjoy_data_.recv_command_command);

                if (airjoy_data_.recv_command_command < 0x08 && airjoy_data_.recv_command_command >= 0x00)
                    send_cmd = (SEND_CMD_TO_R2)airjoy_data_.recv_command_command;
                cmd_to_r2_cnt++;
            }
            else if (airjoy_data_.recv_command_command == 0)
            {
                Serial1Protocol::getInstance()->sendStop();
                send_cmd = SEND_NONE;
                cmd_to_r2_cnt++;
            }
        }
        if (airjoy_data_.recv_command_command < 0x08 && airjoy_data_.recv_command_command >= 0x00)
            now_cmd = (SEND_CMD_TO_R2)airjoy_data_.recv_command_command;
    }
    else
        return;
}
#endif

void FSM_Controller::all_stop()
{
    combine_triggered_ = false;
    // 停止模式+目标设置模式
    chassis_setup_->setPathAutoStart(0); // 路径自动开始标志清零
    arm_setup_->set_Arm_autoStart(0);    // 自动流程标志清零
    if (arm_setup_->isArmcalibrated() == true)
        arm_setup_->setArmStatus(ARM_STOP);
    else
        arm_setup_->setArmStatus(ARM_CALIBRATE);

    if (weaponSage_setup_->isWeaponSageCalibrated() == true)
        weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_STOP);
    else
        weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_CALIBRATE);

    chassis_setup_->setChassisStatus(CHASSIS_STOP);

#if !USE_RC10_AIRJOY
    stop_modeswitch();
#endif
}

#if !USE_RC10_AIRJOY
void FSM_Controller::stop_modeswitch()
{
    switch (Stop_set_stauts)
    {
    case NONE:
    {
        crsf_send_s.isread_srollWheelSpear = false;
        break;
    }

    case RELOCATE:
    {
        crsf_send_s.isread_srollWheelKFS = false;
        crsf_send_s.isread_srollWheelSpear = false;

        static uint8_t iiii = 0;

        if (airjoy_data_.botton_click == 1 && iiii == 0)
        {
            Locate_Setup::getInstance()->Relocte_ToLader();

            iiii++;
        }
        else if (airjoy_data_.botton_click == 0)
        {
            iiii = 0;
        }
        break;
    }
    }
}

#endif
int8_t indexcccc = 0;
void FSM_Controller::manual_ctrl()
{
    chassis_setup_->setPathAutoStart(0); // 路径自动开始标志清零
    arm_setup_->set_Arm_autoStart(0);    // 自动流程标志清零
    weaponSage_setup_->setCBauto(false);
    combine_triggered_ = false;
#if !USE_RC10_AIRJOY
    switch (airjoy_data_.SWC)
    {
    case 0x00:
    {
        //        if (airjoy_data_.SWA == 0x00)
        chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
        //        else if (airjoy_data_.SWA == 0x01)
        //            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_D);

        arm_setup_->setArmStatus(ARM_IDLE);
        weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);

        break;
    }
    case 0x01:
    {
        chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);
        arm_setup_->setArmStatus(ARM_MANUAL_CONTROL);
        weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
        break;
    }
    case 0x02:
    {
        // chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);
        chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_C);
        arm_setup_->setArmStatus(ARM_IDLE);
        weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_MANUAL_CONTROL);
        break;
    }
    }
#else
    switch (airjoy_data_.SWF)
    {
    case 0x00:
    {
        chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
        arm_setup_->setArmStatus(ARM_IDLE);
        weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
        communication::Lora_communication::GetInstance()->send_robot_mode(SEND_CHASSIS_MANUAL_CTRL);
        break;
    }

    case 0x01:
    {
        arm_setup_->setArmStatus(ARM_MANUAL_CONTROL);
        weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
        if (airjoy_data_.SWE == 0x00)
            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);
        else if (airjoy_data_.SWE == 0x01)
            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);

        communication::Lora_communication::GetInstance()->send_robot_mode(SEND_ARM_MANUAL_CTRL);
        break;
    }

    case 0x02:
    {
        weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_MANUAL_CONTROL);
        arm_setup_->setArmStatus(ARM_IDLE);
        if (airjoy_data_.SWE == 0x00)
            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);
        else if (airjoy_data_.SWE == 0x01)
            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);

        communication::Lora_communication::GetInstance()->send_robot_mode(SEND_WEAPON_MANUAL_CTRL);
        break;
    }

    default:
        break;
    }
#endif
}

void FSM_Controller::auto_ctrl()
{
    // 自动模式
    // arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
    // chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL);

#if !USE_RC10_AIRJOY
    switch (airjoy_data_.SWC)
    {
    // 三区自动模式以及手操
    case 0x00:
    {
        weaponSage_setup_->setCBauto(false);
        arm_setup_->set_Arm_autoStart(0); // 自动流程标志清零
        if (airjoy_data_.SWA == 0x01 && airjoy_data_.SWD == 0x00)
        {
            chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL_CZ_R1);
            arm_setup_->setArmStatus(ARM_IDLE);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);

            static uint8_t is_click = 0;
            if (airjoy_data_.botton_click == 1 && is_click == 0)
            {
                chassis_setup_->setPathAutoStart(1); // 路径自动开始标志
                weaponSage_setup_->setCBauto(true);
                is_click = 1;
            }
            else if (airjoy_data_.botton_click == 0)
            {
                is_click = 0;
            }
        }
        else if (airjoy_data_.SWA == 0x00 && airjoy_data_.SWD == 0x00)
        {
            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
            arm_setup_->setArmStatus(ARM_IDLE);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
            chassis_setup_->setPathAutoStart(0); // 路径自动开始标志清零
        }
        else if (airjoy_data_.SWD == 0x01)
        {
            chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL_CZ_R2);
            arm_setup_->setArmStatus(ARM_IDLE);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);

            static uint8_t is_click = 0;
            if (airjoy_data_.botton_click == 1 && is_click == 0)
            {
                chassis_setup_->setPathAutoStart(1); // 路径自动开始标志
                weaponSage_setup_->setCBauto(true);
                is_click = 1;
            }
            else if (airjoy_data_.botton_click == 0)
            {
                is_click = 0;
            }

            if (airjoy_data_.SWA == 0x01) // 合体前命令
            {
                uint8_t sw_now = airjoy_data_.scroll_wheel & 0x01;
                if (crsf_send_s.sw_cmd_section != 1)
                {
                    Serial1Protocol::getInstance()->sendStop();
                    crsf_send_s.sw_cmd_section = 1;
                    crsf_send_s.sw_cmd_last = sw_now;
                    crsf_send_s.sw_cmd_toggle = false;
                }
                else if (sw_now != crsf_send_s.sw_cmd_last)
                {
                    crsf_send_s.sw_cmd_last = sw_now;
                    if (!crsf_send_s.sw_cmd_toggle)
                        Serial1Protocol::getInstance()->send_cmd_to_R2(SEND_WAIT_COMBINE);
                    else
                        Serial1Protocol::getInstance()->send_cmd_to_R2(SEND_COMBINE_CMD);
                    crsf_send_s.sw_cmd_toggle = !crsf_send_s.sw_cmd_toggle;
                }
            }
            else if (airjoy_data_.SWA == 0x00) // 合体后命令
            {
                uint8_t sw_now = airjoy_data_.scroll_wheel & 0x01;
                if (crsf_send_s.sw_cmd_section != 2)
                {
                    Serial1Protocol::getInstance()->sendStop();
                    crsf_send_s.sw_cmd_section = 2;
                    crsf_send_s.sw_cmd_last = sw_now;
                }
                else if (sw_now != crsf_send_s.sw_cmd_last)
                {
                    crsf_send_s.sw_cmd_last = sw_now;
                    Serial1Protocol::getInstance()->send_cmd_to_R2(SEND_PUT_DOWN_HIGH);
                }
            }
        }
        break;
    }

    // arm自动模式
    case 0x01:
    {
// 暂时不把路径规划部分纳入
#if !ARM_AUTO_DEBUG_NOCHASSIS

        chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL_KFS);
#else
        chassis_setup_->setChassisStatus(CHASSIS_STOP);
#endif
        weaponSage_setup_->setCBauto(false);
        arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
        // arm_setup_->setArmStatus(ARM_IDLE);
        weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_KFS_IDLE);

        static uint8_t is_click = 0;
        if (airjoy_data_.botton_click == 1 && is_click == 0)
        {
            arm_setup_->set_Arm_autoStart(1); // 开始自动流程
#if !ARM_AUTO_DEBUG_NOCHASSIS
            chassis_setup_->setPathAutoStart(1); // 路径自动开始标志
#endif
            is_click = 1;
        }
        else if (airjoy_data_.botton_click == 0)
        {
            is_click = 0;
        }

#if !ARM_AUTO_DEBUG_NOCHASSIS
        if (arm_setup_->isArmAutoStart())
        {
            // 判断是否可以进入伸展阶段
            if (chassis_setup_->Get_Arm_Start_flag())
            {
                arm_setup_->setAutocanExtend(true);
            }

            if (arm_setup_->isAutoChassisCanStart())
            {
                chassis_setup_->Receive_Arm_End_flag(false); // 上层已经完成拾取，通知底盘可以开始移动了
                arm_setup_->clearChassisStartFlag();
            }
        }
#endif

        break;
    }

    // weaponSage自动模式
    case 0x02:
    {

        weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_AUTOCONTROL);

        // 滚轮拨动 → 发送对接成功指令
        {
            uint8_t sw_now = airjoy_data_.scroll_wheel & 0x01;
            if (crsf_send_s.sw_cmd_section != 3)
            {
                Serial1Protocol::getInstance()->sendStop();
                crsf_send_s.sw_cmd_section = 3;
                crsf_send_s.sw_cmd_last = sw_now;
            }
            else if (sw_now != crsf_send_s.sw_cmd_last)
            {
                crsf_send_s.sw_cmd_last = sw_now;
                Serial1Protocol::getInstance()->send_cmd_to_R2(SEND_DOCK_SUCCESS);
            }
        }

        // weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
        chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL_CB);
        arm_setup_->setArmStatus(ARM_IDLE);

        static uint8_t is_click = 0;
        if (airjoy_data_.botton_click == 1 && is_click == 0)
        {
            chassis_setup_->setPathAutoStart(1); // 路径自动开始标志
            weaponSage_setup_->setCBauto(true);
            is_click = 1;
        }
        else if (airjoy_data_.botton_click == 0)
        {
            is_click = 0;
        }

        // if(weaponSage_setup_->is_auto_ctrl1())
        // {
        if (weaponSage_setup_->is_auto_ctrl_over())
        {
            chassis_setup_->ReceiveEnd_flag(false);
        }

        // 判断是否可以进行互相通讯
        if (chassis_setup_->GetReach_flag() == true)
        {
            weaponSage_setup_->Get_OMNI_IM_flag(true);
        }
        if (weaponSage_setup_->Get_Catch_flag() == true)
        {
            chassis_setup_->ReceiveReach_flag(false);
        }

        if (chassis_setup_->GetEnd_flag() == true)
        {
            weaponSage_setup_->Get_OMNI_DS_flag(true);
        }
        // }
        break;
    }
    }
#endif

#if USE_RC10_AIRJOY
    switch (airjoy_data_.SWF)
    {
    case 0x00:
    {

        if (airjoy_data_.SWD == 0x00 && airjoy_data_.SWB == 0x00) // 手操模式-机械臂十字键
        {
            arm_setup_->setArmStatus(ARM_SEMI_AUTO_CONTROL);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
            communication::Lora_communication::GetInstance()->send_robot_mode(SEND_ARM_SEMI);

            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_CZ);

            chassis_setup_->setPathAutoStart(0); // 路径自动开始标志清零
            arm_setup_->set_Arm_autoStart(0);    // 自动流程标志清零
        }
        else if (airjoy_data_.SWD == 0x01 && airjoy_data_.SWB == 0x00) // 手操模式-武器十字键
        {
            arm_setup_->setArmStatus(ARM_IDLE);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_SEMI_AUTO_CONTROL);
            communication::Lora_communication::GetInstance()->send_robot_mode(SEND_WEAPON_SEMI);

            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_CZ);

            chassis_setup_->setPathAutoStart(0); // 路径自动开始标志清零
            arm_setup_->set_Arm_autoStart(0);    // 自动流程标志清零
        }
        else if (airjoy_data_.SWB == 0x01 && airjoy_data_.SWC == 0x00) // 合体模式 上层IDLE
        {
            if (!combine_triggered_)
            {
                arm_setup_->setArmStatus(ARM_IDLE);
                if (airjoy_data_.d_pad_left == 1 || airjoy_data_.d_pad_right == 1 ||
                    airjoy_data_.d_pad_up == 1 || airjoy_data_.d_pad_down == 1)
                    combine_triggered_ = true;
            }
            else
            {
                arm_setup_->setArmStatus(ARM_COMBINE_IDLE);
            }
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
            communication::Lora_communication::GetInstance()->send_robot_mode(SEND_COMBINE_MODE);
            chassis_setup_->setChassisStatus(SEMI_AUIO_CZ_FIT);
        }
        else if (airjoy_data_.SWB == 0x01 && airjoy_data_.SWC == 0x01 && airjoy_data_.SWD == 0x00) // 竞技场 机械臂模式
        {
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);

            communication::Lora_communication::GetInstance()->send_robot_mode(SEND_COMP_ARM);

            #if CHALLENGE_3ZONE //挑战赛3区专用代码
            arm_setup_->setArmStatus(ARM_CHALLENGE_3ZONE);
            #else
            arm_setup_->setArmStatus(ARM_COMP_SEMI_CONTROL);
            chassis_setup_->setChassisStatus(SEMI_AUIO_CZ_ARM);
            
            if (chassis_setup_->Get_CZ_Arm_flag())
                arm_setup_->can_putdown(1);

            if(arm_setup_->is_putdown_done())
                chassis_setup_->Receive_CZ_Arm_flag(false);
            #endif  
        }
        else if (airjoy_data_.SWB == 0x01 && airjoy_data_.SWC == 0x01 && airjoy_data_.SWD == 0x01) // 竞技场 武器模式
        {
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_COMP_MANUAL_CONTROL);
            arm_setup_->setArmStatus(ARM_IDLE);
            communication::Lora_communication::GetInstance()->send_robot_mode(SEND_COMP_WEAPON);

            chassis_setup_->setChassisStatus(SEMI_AUIO_CZ_WEAPON);
        }
        else
        {
            combine_triggered_ = false;
            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_CZ);

            chassis_setup_->setPathAutoStart(0); // 路径自动开始标志清零
            arm_setup_->set_Arm_autoStart(0);    // 自动流程标志清零
        }
        break;
    }

    case 0x01:
    {
        combine_triggered_ = false;
        if (airjoy_data_.SWB == 0x01) // 自动模式
        {
            communication::Lora_communication::GetInstance()->send_robot_mode(SEND_ARM_AUTO);
// 暂时不把路径规划部分纳入
#if !ARM_AUTO_DEBUG_NOCHASSIS
            chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL_KFS);
#else
            chassis_setup_->setChassisStatus(CHASSIS_STOP);
#endif
            arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_KFS_IDLE);
            //arm_setup_->setArmStatus(ARM_IDLE);
            static uint8_t is_click = 0;

            if (airjoy_data_.LB == 1 && is_click == 0 && airjoy_data_.page != 0x01)
            {
                arm_setup_->set_Arm_autoStart(1); // 开始自动流程
#if !ARM_AUTO_DEBUG_NOCHASSIS
                chassis_setup_->setPathAutoStart(1); // 路径自动开始标志
#endif
                is_click = 1;
            }
            else if (airjoy_data_.LB == 0)
            {
                is_click = 0;
            }

#if !ARM_AUTO_DEBUG_NOCHASSIS
            static uint8_t prev_swf = 0xFF, prev_swb = 0xFF;
            bool from_weapon_auto = (prev_swf == 0x02 && prev_swb == 0x01
                                     && airjoy_data_.SWF == 0x01 && airjoy_data_.SWB == 0x01);
            bool kfs_active = from_weapon_auto || arm_setup_->isArmAutoStart();
            weaponSage_setup_->set_kfs_idle_active(kfs_active);

            if (arm_setup_->isArmAutoStart())
            {
                weaponSage_setup_->set_kfs_idle_arm_90(true);
                // 判断是否可以进入伸展阶段
                if (chassis_setup_->Get_Arm_Start_flag())
                {
                    arm_setup_->setAutocanExtend(true);
                }

                if (arm_setup_->isAutoChassisCanStart())
                {
                    chassis_setup_->Receive_Arm_End_flag(false); // 上层已经完成拾取，通知底盘可以开始移动了
                    arm_setup_->clearChassisStartFlag();
                }
            }
            else
            {
                weaponSage_setup_->set_kfs_idle_arm_90(false);
            }

            prev_swf = airjoy_data_.SWF;
            prev_swb = airjoy_data_.SWB;
#endif
        }
        else if (airjoy_data_.SWB == 0x00) // 手操模式
        {
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
            communication::Lora_communication::GetInstance()->send_robot_mode(SEND_ARM_LOW_MANUAL_LEVEL);
            if (airjoy_data_.SWE == 0x00)
                chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
            else if (airjoy_data_.SWE == 0x01)
                chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);

            arm_setup_->setArmStatus(ARM_MANUAL_LOW_LEVEL); // 需要区分SWE 来分开底盘和机械臂的手操模式
        }
        break;
    }

    case 0x02:
    {
        combine_triggered_ = false;
        arm_setup_->setArmStatus(ARM_IDLE);
        if (airjoy_data_.SWB == 0x01) // 全自动
        {
            communication::Lora_communication::GetInstance()->send_robot_mode(SEND_WEAPON_AUTO);
            // 这里使用SWD == 0x00为摄像头路径
            //         SWD == 0x01为贴边路径

            weaponSage_setup_->Set_End_Flag(chassis_setup_->GetReach_flag());
            // weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_AUTOCONTROL);
            chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL_CB);
            static uint8_t is_click = 0;
            if (airjoy_data_.LB == 1 && is_click == 0 && airjoy_data_.page != 0x01)
            {
                chassis_setup_->setPathAutoStart(1); // 路径自动开始标志
                weaponSage_setup_->setCBauto(true);
                is_click = 1;
            }
            else if (airjoy_data_.LB == 0)
            {
                is_click = 0;
            }

            if (weaponSage_setup_->is_auto_ctrl_over())
            {
                chassis_setup_->ReceiveEnd_flag(false);
            }

            // 判断是否可以进行互相通讯
            if (chassis_setup_->GetReach_flag() == true)
            {
                weaponSage_setup_->Get_OMNI_IM_flag(true);
            }
            if (weaponSage_setup_->Get_Catch_flag() == true)
            {
                chassis_setup_->ReceiveReach_flag(false);
            }

            if (chassis_setup_->GetEnd_flag() == true)
            {
                weaponSage_setup_->Get_OMNI_DS_flag(true);
            }

            if(weaponSage_setup_->is_catch() == true)
            {
                chassis_setup_->ReceiveBack_flag(false);
            }
        }
        else if (airjoy_data_.SWB == 0x00) // 半自动
        {
            communication::Lora_communication::GetInstance()->send_robot_mode(SEND_WEAPON_LOW_MANUAL_LEVEL);
            

            if(airjoy_data_.SWD == 0x00)
            {
                weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_MANUAL_LOW_LEVEL);
                if (airjoy_data_.SWE == 0x00)
                    chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
                else if (airjoy_data_.SWE == 0x01)
                    chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);
            }
            else if(airjoy_data_.SWD == 0x01)
            {
                chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_D);
                weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_TEST_POINT);
            }
        }
        break;
    }

    default:
        break;
    }
#endif
}

void FSM_Controller::debug()
{
}
