#include "WeaponSage_Setup.h"

Robot_WeaponSage_Setup::Robot_WeaponSage_Setup(WeaponSage_InitData_S init_data)
    : Robot_WeaponSage(init_data), RtosTask("WeaponSage_Setup", 1)
{
    
}

void Robot_WeaponSage_Setup::loop()
{
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
    //待实现
}

void Robot_WeaponSage_Setup::idle()
{
    //空闲状态，维持当前状态
}

void Robot_WeaponSage_Setup::stop()
{
    //停止，电机不动
    this->setJointTarget(0.0f, WeaponSage::Launch_Motor);
    this->setJointTarget(0.0f, WeaponSage::Claw_Motor);
    this->setJointTarget(0.0f, WeaponSage::Traverse_Motor);
    this->setJointTarget(0.0f, WeaponSage::Wrist_Motor);
}
