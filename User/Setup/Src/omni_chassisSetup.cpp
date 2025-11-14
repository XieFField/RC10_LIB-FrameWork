#include "omni_chassisSetup.h"

void OmniChassis_Setup::loop()
{
    if (!init_flag)
        return;
    
    switch (chassis_status_)
    {
        case CHASSIS_MANUAL_CONTROL_A:
        {
            break;
        }

        case CHASSIS_MANUAL_CONTROL_B:
        {
            break;
        }
        case CHASSIS_AUTO_CONTROL:
        {
            Chassis_Control_Auto();
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

void OmniChassis_Setup::Chassis_Control_Auto()
{
    target_chassis_twist_.vx=final_speed.x;
    target_chassis_twist_.vy=final_speed.y;
    setWorldSpeed(target_chassis_twist_);
}