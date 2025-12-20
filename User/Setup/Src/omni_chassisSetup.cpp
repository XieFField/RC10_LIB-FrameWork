#include "omni_chassisSetup.h"

#if debug_ladar

int last_cout_ladar_data = -1;

#endif
void OmniChassis_Setup::loop()
{
    if (!init_flag)
        return;

    RealPos ra = Position::GetInstance(&huart1)->getRealPosData();
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
    ladar_data_ = Lader_position::GetInstance(&hUsbDeviceHS)->Get_Rader_Data();

    chassis_status_ = CHASSIS_STOP;
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

    //接收一次雷达数据打印一次
    
    this->setWorldSpeed(target_chassis_twist_);

    #if debug_ladar

    if(Lader_position::GetInstance(&hUsbDeviceHS)->return_coutlar_data() > last_cout_ladar_data)
    {
        debug_uart.Printf_Ladar(ladar_data_.x, ladar_data_.y);    
        last_cout_ladar_data = Lader_position::GetInstance(&hUsbDeviceHS)->return_coutlar_data();
    }

    #endif
    //debug_uart.Printf_Ladar(ladar_data_.x, ladar_data_.y);
    this->update();

    
}
