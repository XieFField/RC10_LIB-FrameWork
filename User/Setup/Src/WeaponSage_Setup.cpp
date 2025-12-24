#include "WeaponSage_Setup.h"

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
    switch(airjoy_data_.SWA)
    {
        case 0x00:
        {
            //夹取武器
            break;
        }

        case 0x01:
        {
            //进攻模式
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
