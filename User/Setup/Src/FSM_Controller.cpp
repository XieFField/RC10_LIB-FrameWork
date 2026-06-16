#include "FSM_Controller.h"
#include "chassis_swerve_task_demo.h"

int8_t test_led = 0;
float x_ , y_ , yaw_;
bool claw1,claw2,claw3;
void FSM_Controller::loop()
{
    if (!init_flag_)
        return;

    if (!test_led)
        Serial1Protocol::getInstance()->sendStop();
    else if(test_led !=0 && test_led < 100)
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

    communication::Lora_communication::GetInstance()->Task_Process();
    communication::Lora_communication::GetInstance()->Tim_It_Process();

    communication::Lora_communication::GetInstance()->update_airjoy_data(&airjoy_data_);

    // communication::Lora_communication::GetInstance()->send_robot_pos(
    //     Locate_Setup::getInstance()->get_RobotPos_inWorld().x,
    //     Locate_Setup::getInstance()->get_RobotPos_inWorld().y,
    //     Locate_Setup::getInstance()->get_RobotPos_inWorld().yaw);
    communication::Lora_communication::GetInstance()->send_robot_pos(
        x_,
        y_,
        yaw_);
    bool suker_status = false;

    if (arm_setup_->getSuckerStatus() == SUCK)
        suker_status = true;
    else
        suker_status = false;


    communication::Lora_communication::GetInstance()->send_sucker_status(
        suker_status,
        0); 

    // communication::Lora_communication::GetInstance()->send_claw_status(
    //     weaponSage_setup_->get_CurrentPos().claw_1_pos_ > weaponSage_setup_->getInitData().max_arm_angle_ - 20 ? true : false,
    //     weaponSage_setup_->get_CurrentPos().claw_2_pos_ > weaponSage_setup_->getInitData().max_arm_angle_ - 20 ? true : false,
    //     weaponSage_setup_->get_CurrentPos().claw_3_pos_ > weaponSage_setup_->getInitData().max_arm_angle_ - 20 ? true : false);
    communication::Lora_communication::GetInstance()->send_claw_status(
        claw1,
        claw2,
        claw3);

    // communication::Lora_communication::GetInstance()->send_auto_status(arm_setup_->isArmAutoStart());
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
    // relocate
    if (robot_status_ == ALL_STOP && airjoy_data_.SWB == 0x01 && airjoy_data_.SWE == 0x00)
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
    }
    else if (robot_status_ == ALL_STOP && airjoy_data_.SWB == 0x01 && airjoy_data_.SWE == 0x01)
    {
        // 设置target 矛杆
    }
#endif
    if (KStarget != last_KStarget)
    {
        chassis_setup_->set_KFS(KStarget.KFS[0], KStarget.KFS[1], KStarget.KFS[2]);
        arm_setup_->set_TargetKFS(KStarget.KFS[0], KStarget.KFS[1], KStarget.KFS[2]);
        weaponSage_setup_->setTargetIndex(KStarget.Spear - 1);
    }

    last_KStarget = KStarget;

#if USE_RC10_AIRJOY
    set_cmd_to_R2();
#endif
}

#if USE_RC10_AIRJOY

void FSM_Controller::set_cmd_to_R2()
{
    if (airjoy_data_.page == 0x02 && robot_status_ != ALL_STOP)
    {
    }
    else
        return;
}
#endif

void FSM_Controller::all_stop()
{
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

#if !USE_RC10_AIRJOY
    switch (airjoy_data_.SWC)
    {
        case 0x00:
        {
            if (airjoy_data_.SWA == 0x00)
                chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
            else if (airjoy_data_.SWA == 0x01)
                chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_D);

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
            break;
        }

        case 0x02:
        {
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_MANUAL_CONTROL);
            arm_setup_->setArmStatus(ARM_IDLE);
            if(airjoy_data_.SWE == 0x00)
                chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);
            else if (airjoy_data_.SWE == 0x01)
                chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
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
                arm_setup_->set_Arm_autoStart(0);    // 自动流程标志清零
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

            arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
            //arm_setup_->setArmStatus(ARM_IDLE);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);

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
                }
            }
    #endif

            break;
        }

        // weaponSage自动模式
        case 0x02:
        {

            // weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_AUTOCONTROL);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
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
            break;
        }
    }
#endif

#if USE_RC10_AIRJOY
    switch (airjoy_data_.SWF)
    {
    case 0x00: 
    {
        chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);


        chassis_setup_->setPathAutoStart(0); // 路径自动开始标志清零
        arm_setup_->set_Arm_autoStart(0);    // 自动流程标志清零

        if(airjoy_data_.SWD == 0x00 && airjoy_data_.SWB == 0x00) //手操模式-机械臂十字键
        {
            arm_setup_->setArmStatus(ARM_MANUAL_CONTROL);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
        }
        else if(airjoy_data_.SWD == 0x01 && airjoy_data_.SWB == 0x00) //手操模式-武器十字键
        {
            arm_setup_->setArmStatus(ARM_IDLE);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_MANUAL_CONTROL);
        }
        else if(airjoy_data_.SWB == 0x01 && airjoy_data_.SWC == 0x00) //合体模式 上层IDLE
        {
            arm_setup_->setArmStatus(ARM_IDLE);
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
        }
        else if(airjoy_data_.SWB == 0x01 && airjoy_data_.SWC == 0x01 
                && airjoy_data_.SWD == 0x00) //竞技场 机械臂模式
        {
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
            arm_setup_->setArmStatus(ARM_SEMI_AUTO_CONTROL);
        }
        else if(airjoy_data_.SWB == 0x01 && airjoy_data_.SWC == 0x01
                && airjoy_data_.SWD == 0x01) //竞技场 武器模式
        {
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_SEMI_AUTO_CONTROL);
            arm_setup_->setArmStatus(ARM_IDLE);
        }
        break;
    }

    case 0x01:
    {
        weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_IDLE);
        if (airjoy_data_.SWB == 0x01) // 自动模式
        {
// 暂时不把路径规划部分纳入
#if !ARM_AUTO_DEBUG_NOCHASSIS
            chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL_KFS);
#else
            chassis_setup_->setChassisStatus(CHASSIS_STOP);
#endif
            // arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
            arm_setup_->setArmStatus(ARM_IDLE);
            static uint8_t is_click = 0;

            if (airjoy_data_.LB == 1 && is_click == 0)
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
                }
            }
#endif
        }
        else if (airjoy_data_.SWB == 0x00) // 手操模式
        {
            if (airjoy_data_.SWE == 0x00)
                chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
            else if (airjoy_data_.SWE == 0x01)
                chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);

            arm_setup_->setArmStatus(ARM_SEMI_LOW_LEVEL); //需要区分SWE 来分开底盘和机械臂的手操模式
        }
        break;
    }

    case 0x02:
    {
        arm_setup_->setArmStatus(ARM_IDLE);
        if (airjoy_data_.SWB == 0x01) // 全自动
        {
            //这里使用SWD == 0x00为摄像头路径
            //        SWD == 0x01为贴边路径


            weaponSage_setup_->Set_End_Flag(chassis_setup_->GetReach_flag());
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_AUTO_CONTROL);
            chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL_CB);
            static uint8_t is_click = 0;
            if (airjoy_data_.LB == 1 && is_click == 0)
            {
                chassis_setup_->setPathAutoStart(1); // 路径自动开始标志
                weaponSage_setup_->setCBauto(true);
                is_click = 1;
            }
            else if (airjoy_data_.LB == 0)
            {
                is_click = 0;
            }
        }
        else if (airjoy_data_.SWB == 0x00) // 半自动
        {
            weaponSage_setup_->setWeaponSageControlStatus(WEAPONSAGE_MANUAL_CONTROL);

            if (airjoy_data_.SWE == 0x00)
                chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
            else if (airjoy_data_.SWE == 0x01)
                chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);
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
