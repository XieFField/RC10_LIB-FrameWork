#include "M3508_Steer_Debug.h"

void M3508_Steer_Debug::loop()
{
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);

    if(airjoy_data_.SWB != 0x01)
        return;

    
    switch(test_index)
    {
        case 0:
        {
            for(int i = 0; i < 4; i++)
            {
                steer[i]->setTargetCurrent(0.0f);
            }
        }

        case 1:
        {
            for(int i = 0; i < 4; i++)
            {
                steer[i]->setTargetTotalAngle(test_target_angle[i]);
            }
            break;
        }

        case 2:
        {
            for(int i = 0; i < 4; i++)
            {
                if(std::fabs(airjoy_data_.left_x) > 0.15f) // 死区
                    target_rpm[i] = airjoy_data_.left_x * k;
            }

            for(int i = 0; i < 4; i++)
            {
                steer[i]->setTargetRPM(target_rpm[i]);
            }
        }

        case 3:
        {
            for(int i = 0; i < 4; i++)
            {
                if(std::fabs(airjoy_data_.left_x) > 0.15f && std::fabs(test_target_angle[i]) < 900.0f) // 死区
                    test_target_angle[i] += airjoy_data_.left_x * 5.0f; // 每次调整5度

                steer[i]->setTargetTotalAngle(test_target_angle[i]);
            }
            break;
        }

        default:
           break;
    }
}