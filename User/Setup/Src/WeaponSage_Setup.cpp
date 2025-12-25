#include "WeaponSage_Setup.h"

namespace WeaponSage_Setup {
    float weapon_pos[4] = {0.0f, 0.0f, 0.0f, 0.0f};
}

Robot_WeaponSage_Setup::Robot_WeaponSage_Setup(WeaponSage_InitData_S init_data)
    : Robot_WeaponSage(init_data), RtosTask("WeaponSage_Setup", 1)
{
    
}

void Robot_WeaponSage_Setup::loop()
{

    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);

    switch(weaponSage_status_)
    {
        case WEAPONSAGE_MANUAL_CONTROL:
            manualControl();
            break;
        case WEAPONSAGE_IDLE:
            idle();
            break;
        case WEAPONSAGE_STOP:
            stop();
            break;
        case WEAPONSAGE_DEBUG:
            debug();
            break;

        case WEAPONSAGE_AUTO_CONTROL:
            //待实现自动控制逻辑
            break;
        default:
            idle();
            break;
    }
}

void Robot_WeaponSage_Setup::calibrate()
{
    //待实现
}

void Robot_WeaponSage_Setup::manualControl()
{
    this->setCtrlMode(WeaponSage::Join_POSITION_CONTROL);
    switch(airjoy_data_.SWA)
    {
        case 0x00:
        {
            //夹取武器
            if(airjoy_data_.scroll_wheel == 0x00)
                this->setTarget(0.0f, WeaponSage::Claw_Motor); //张开爪子
            else if(airjoy_data_.scroll_wheel == 0x01)
                this->setTarget(initData_.max_clawAngle_, WeaponSage::Claw_Motor); //夹紧爪子
            else
                this->setTarget(0.0f, WeaponSage::Claw_Motor); //张开爪子

            //夹爪位置

            if(ctrl_status_.target_poleIndex < 0)
                ctrl_status_.target_poleIndex = 0;
            else if(ctrl_status_.target_poleIndex > 3)
                ctrl_status_.target_poleIndex = 3;

            if(_tool_Abs(airjoy_data_.right_x) < 0.1)
                manual_ctrlForgrip_.changeTarget_state = false;

            else if(airjoy_data_.right_x > 0.3f)
            {
                manual_ctrlForgrip_.changeTarget_state = true;
                ctrl_status_.target_poleIndex++;
            }
            else if(airjoy_data_.right_x < -0.3f)
            {
                manual_ctrlForgrip_.changeTarget_state = true;
                ctrl_status_.target_poleIndex--;
            }

            this->setTarget(WeaponSage_Setup::weapon_pos[ctrl_status_.target_poleIndex], WeaponSage::Traverse_Motor);

            if(airjoy_data_.SWD == 0x00)
                this->setTarget(0.0f, WeaponSage::Launch_Motor); //下降
            else if(airjoy_data_.SWD == 0x01)
                this->setTarget(initData_.max_launchHeight_, WeaponSage::Launch_Motor); //上升
            else
                this->setTarget(0.0f, WeaponSage::Launch_Motor); //下降
                

            

            manual_ctrlForgrip_.last_right_stick_x = airjoy_data_.right_x;
            manual_ctrlForgrip_.last_right_stick_y = airjoy_data_.right_y;

            break;
        }

        case 0x01:
        {
            //进攻模式
            this->setTarget(90.0f, WeaponSage::Wrist_Motor); //手腕前倾90度
            this->setTarget(initData_.max_clawAngle_, WeaponSage::Claw_Motor); //夹紧爪子


            if(_tool_Abs(airjoy_data_.right_y) > 0.1f)
            {
                if(airjoy_data_.right_y > 0.1f)
                    target_pos_.launch_pos_ += 0.001f; //升高
                else if(airjoy_data_.right_y < -0.1f)
                    target_pos_.launch_pos_ -= 0.001f; //降低
            }

            this->setTarget(target_pos_.launch_pos_, WeaponSage::Launch_Motor);


            if(airjoy_data_.SWD == 0x00)
                this->setTarget(0.0f, WeaponSage::Traverse_Motor); //收住
            else if(airjoy_data_.SWD == 0x01)
                this->setTarget(initData_.max_traverseLength_, WeaponSage::Traverse_Motor); //进攻
            else
                this->setTarget(0.0f, WeaponSage::Traverse_Motor); //收住

            break;
        }

        default:
        {
            idle();
            break;
        }
    }



}

void Robot_WeaponSage_Setup::idle()
{
    //空闲状态，维持当前状态
}

void Robot_WeaponSage_Setup::debug()
{
    //待实现
}

void Robot_WeaponSage_Setup::autoControl()
{
    //待实现
    /**
     * @brief 自动控制逻辑
     *  1.对于4个待取矛杆，硬编码四个位置
     *  2.当底盘靠位完成后，总状态机发来下降指令，执行下降
     *  3.当下降完成后，且底盘与武器架底部接触，则执行抓取
     *  4.当底盘后退到能将矛杆抬起的位置后，执行抬起
     */
}

void Robot_WeaponSage_Setup::stop()
{
    //停止，电机不动
    this->setTarget(0.0f, WeaponSage::Launch_Motor);
    this->setTarget(0.0f, WeaponSage::Claw_Motor);
    this->setTarget(0.0f, WeaponSage::Traverse_Motor);
    this->setTarget(0.0f, WeaponSage::Wrist_Motor);
}

WeaponSage_InitData_S initData_=
{
    .max_launchHeight_ =0.350f,
    .max_clawAngle_ = 49.0f,
    .max_traverseLength_ = 0.470f,

    .wrist_gearRatio_ = 1.0f,
    .launch_Ratio_ = 0.08627f,
    .claw_gearRatio_  =1.0f ,
    .traverse_Ratio_  = 0.00218f,
    .max_wristMotorRPM_   = 100.0f,
};
