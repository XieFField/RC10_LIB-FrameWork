#include "omni_chassisSetup.h"

void OmniChassis_Setup::loop()
{
    if (!init_flag)
        return;

    RealPos ra = Position::GetInstance(&huart1)->getRealPosData();
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);

    if(airjoy_data_.SWB == 0)
    {
        chassis_status_ = CHASSIS_MANUAL_CONTROL_A;
    }
    else
    {
        chassis_status_ = CHASSIS_MANUAL_CONTROL_B;
    }

    switch (chassis_status_)
    {
        case CHASSIS_MANUAL_CONTROL_A:
        {
            target_chassis_twist_.vx = -airjoy_data_.left_x * 6;
            target_chassis_twist_.vy = airjoy_data_.left_y * 6;
            target_chassis_twist_.yaw_rate = -airjoy_data_.right_y * 6;
			
			target_yaw_ = ra.world_yaw;
            
            break;
        }

        case CHASSIS_MANUAL_CONTROL_B:
        {
            target_chassis_twist_.vx = -airjoy_data_.left_x * 6;
            target_chassis_twist_.vy = airjoy_data_.left_y * 6;

            // 获取当前角度
            float yaw_real_angle = ra.world_yaw;

            yaw_pid_period_count_++;
            if(yaw_pid_period_count_ >= yaw_pid_period_)
            {
                yaw_pid_period_count_ = 0;
                target_chassis_twist_.yaw_rate = yaw_pid_.pid_calc(target_yaw_, yaw_real_angle);
            }
			
            break;
        }
        case CHASSIS_AUTO_CONTROL:
        {
            break;
        }
        case CHASSIS_STOP:
        {
            target_chassis_twist_.vx = 0;
            target_chassis_twist_.vy = 0;
            target_chassis_twist_.yaw_rate = 0;

            break;
        }
        default:
        {
            break;
        }
    }

    debug_uart.printf_DMA("%.2f,%.2f,%.2f,%.2f\r\n",target_yaw_,ra.world_yaw,target_chassis_twist_.yaw_rate,ra.dyaw);

    this->setWorldSpeed(target_chassis_twist_);

    this->update();
}
