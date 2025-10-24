#include "Arm_setup.h"

void ArmSetup::loop()
{
    if(!init_flag)
        return;
    
    static uint64_t last_us = 0;
    uint64_t now_us = TimeStamp::getInstance().getMicroseconds();
    if(last_us == 0) 
    { 
        last_us = now_us; 
        return; 
    }
    uint64_t dt_us = (now_us >= last_us) ? (now_us - last_us) : 0;
    last_us = now_us;
    if(dt_us == 0) 
        return;
    if(dt_us > 200000) 
        dt_us = 200000; 
    float dt = dt_us * 1e-6f;

    if(!is_calibrating)
    {
        calibrateM2006();
        is_calibrating = true;
    }

    switch(arm_status_)
    {
    case ARM_MANUAL_CONTROL:
        {
            manualControl();
        }
        break;

    case ARM_AUTO_CONTROL:
        {
            autoControl();
        }
        break;

    case ARM_STOP: 
        {
            // 停止状态, 将各个关节回归初始位置后，将电流置零
            stop();
        }
        break;
    case ARM_IDLE:
        {
            // 空闲状态，维持当前状态
            idle();
        }
    default:
        break;
    }

    this->update(); //将控制信息发送给电机
}

void ArmSetup::manualControl()
{
    // 手动控制函数
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
}


void ArmSetup::autoControl()
{
    // 自动控制函数
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
}

void ArmSetup::stop()
{
    // 停止控制函数
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
}

void ArmSetup::calibrateM2006()
{
    // 上电校准M2006电机位置
    // 给予M2006一个小电流顶住限位，然后计时1s，将当前位置重定位为0度
}

void ArmSetup::idle()
{
    // 空闲控制函数
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
}



