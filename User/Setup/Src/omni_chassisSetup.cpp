#include "omni_chassisSetup.h"

void OmniChassis_Setup::loop()
{
    if (!init_flag)
        return;
    
//    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
    // airjoy_->process();
    
    // airjoy_->getControlData(&airjoy_data_);
    switch (chassis_status_)
    {
        case CHASSIS_MANUAL_CONTROL_A:
        {
            manualControl_A();
            break;
        }

        case CHASSIS_MANUAL_CONTROL_B:
        {
            break;
        }
        case CHASSIS_AUTO_CONTROL:
        {
            break;
        }

        case CHASSIS_STOP:
        {
            break;
        }
        default:
            break;
    }
    


    this->update();
}


void OmniChassis_Setup::manualControl_A()
{
    // target_chassis_twist_.vx = -airjoy_data_.left_x * 3;
    // target_chassis_twist_.vy = airjoy_data_.left_y * 3;
    // target_chassis_twist_.yaw_rate = airjoy_data_.right_x;

    //速度设置
    this->setRobotSpeed(target_chassis_twist_);

    //this->wheels_[0].getRPM(); 电机实例是protected的，可以通过这样访问电机内部成员
}
