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