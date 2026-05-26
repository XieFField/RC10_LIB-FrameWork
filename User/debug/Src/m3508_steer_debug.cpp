#include "M3508_Steer_Debug.h"

float target_angle = 0.0f;

void M3508_Steer_Debug::loop()
{
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);

    if(airjoy_data_.SWB != 0x01)
    {
        for(int i = 0; i < 4; i++)
        {
            steer[i]->setTargetCurrent(0.0f);
        }
        return;
    }
        

    
    switch(test_index)
    {
        case 0:
        {
            for(int i = 0; i < 4; i++)
            {
                steer[i]->setTargetCurrent(0.0f);
            }
            break;
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
            break;
        }

        case 3:
        {
            for(int i = 0; i < 4; i++)
            {
                test_target_angle[i] = target_angle;

                steer[i]->setTargetAngle(test_target_angle[i]);
            }
            break;
        }

        case 4: 
        {
            for (int i = 0; i < 4; i++)
            {
                if(std::fabs(airjoy_data_.left_x) > 0.15f) // 死区
                    test_target_angle[i] = (airjoy_data_.left_x) * 180.0f;
                else
                    test_target_angle[i] = 0.0f;

                steer[i]->setTargetAngle(test_target_angle[i]);
            }
            break;
        }

        case 5:
        {
            for(int i = 0; i < 4; i++)
            {
                test_target_angle[i] = target_angle;

                steer[i]->setTargetTotalAngle(test_target_angle[i]);
            }
            break;
        }

        case 6:
        {
            for (int i = 0; i < 4; i++)
            {
                if(std::fabs(airjoy_data_.left_x) > 0.15f) // 死区
                    test_target_angle[i] = (airjoy_data_.left_x) * 180.0f;
                else
                    test_target_angle[i] = 0.0f;

                steer[i]->setTargetTotalAngle(test_target_angle[i]);
            }
            break;
        }
        
        default:
           break;
    }
}