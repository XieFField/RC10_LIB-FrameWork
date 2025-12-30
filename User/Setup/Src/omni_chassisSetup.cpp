#include "omni_chassisSetup.h"

#if debug_ladar

int last_cout_ladar_data = -1;

#endif
void OmniChassis_Setup::loop()
{
    if (!init_flag)
        return;

    float dyaw = Locate_Setup::getInstance()->get_dyaw_from_position();
    float yaw = Locate_Setup::getInstance()->get_yaw_from_position();
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
    ladar_data_ = Locate_Setup::getInstance()->get_RobotPos_inWorld();

    Angle_Twist angle_twist = {0};
    angle_twist.yaw_rate = dyaw;
    angle_twist.yaw_angle = yaw;
    this->updateAngleData(angle_twist);

    switch (chassis_status_)
    {
        case CHASSIS_MANUAL_CONTROL_A:
        {
            if(_tool_Abs(airjoy_data_.left_x) > 0.05f)
                target_chassis_twist_.vx = -airjoy_data_.left_x * 6;
            else
                target_chassis_twist_.vx = 0.0f;

            if(_tool_Abs(airjoy_data_.left_y) > 0.05f)
                target_chassis_twist_.vy = airjoy_data_.left_y * 6;
            else
                target_chassis_twist_.vy = 0.0f;

            if(_tool_Abs(airjoy_data_.right_y) > 0.05f)
                target_chassis_twist_.yaw_rate = -airjoy_data_.right_y * 6;
            else
                target_chassis_twist_.yaw_rate = 0.0f;
			
			target_yaw_ = yaw;
            this->setWorldSpeed(target_chassis_twist_);
            this->update();
            break;
        }

        case CHASSIS_MANUAL_CONTROL_B:
        {
            if(_tool_Abs(airjoy_data_.left_x) > 0.05f)
                target_chassis_twist_.vx = -airjoy_data_.left_x * 6;
            else
                target_chassis_twist_.vx = 0.0f;

            if(_tool_Abs(airjoy_data_.left_y) > 0.05f)
                target_chassis_twist_.vy = airjoy_data_.left_y * 6;
            else
                target_chassis_twist_.vy = 0.0f;

            // 获取当前角度
            float yaw_real_angle = yaw;

            yaw_pid_period_count_++;
            if(yaw_pid_period_count_ >= yaw_pid_period_)
            {
                yaw_pid_period_count_ = 0;
                target_chassis_twist_.yaw_rate = yaw_pid_.pid_calc(target_yaw_, yaw_real_angle);
            }
			
            this->setWorldSpeed(target_chassis_twist_);
            this->update();
            break;
        }

        case CHASSIS_LOCK_FORWEAPON:
        {
            float target_yaw_angle = 90.0f;

            float yaw_real_angle = yaw;

            yaw_pid_period_count_++;
            if(yaw_pid_period_count_ >= yaw_pid_period_)
            {
                yaw_pid_period_count_ = 0;
                target_chassis_twist_.yaw_rate = yaw_pid_.pid_calc(target_yaw_angle, yaw_real_angle);
            }

            if(_tool_Abs(airjoy_data_.left_x) > 0.05f)
                target_chassis_twist_.vx = -airjoy_data_.left_x * 6;
            else
                target_chassis_twist_.vx = 0.0f;

            if(_tool_Abs(airjoy_data_.left_y) > 0.05f)
                target_chassis_twist_.vy = airjoy_data_.left_y * 6;
            else
                target_chassis_twist_.vy = 0.0f;

            this->setWorldSpeed(target_chassis_twist_);
            this->update();
            break;
        }

        case CHASSIS_AUTO_CONTROL:
        {
            break;
        }
        case CHASSIS_STOP:
        {
            this->wheels_[0]->setTargetCurrent(0);
            this->wheels_[1]->setTargetCurrent(0);
            this->wheels_[2]->setTargetCurrent(0);
            break;
        }

        case CHASSIS_MANUAL_CONTROL_C:
        {
            target_chassis_twist_.yaw_rate = 0.0f;

            if(_tool_Abs(airjoy_data_.left_x) > 0.05f)
                target_chassis_twist_.vx = -airjoy_data_.left_x * 6;
            else
                target_chassis_twist_.vx = 0.0f;

            if(_tool_Abs(airjoy_data_.left_y) > 0.05f)
                target_chassis_twist_.vy = airjoy_data_.left_y * 6;
            else
                target_chassis_twist_.vy = 0.0f;

            this->setWorldSpeed(target_chassis_twist_);
            this->update();
            break;
        }
        default:
        {
            break;
        }
    }

    //����һ���״����ݴ�ӡһ��
    
    

    #if debug_ladar

    if(Lader_position::GetInstance(&hUsbDeviceHS)->return_coutlar_data() > last_cout_ladar_data)
    {
        debug_uart.Printf_Ladar(ladar_data_.x, ladar_data_.y);    
        last_cout_ladar_data = Lader_position::GetInstance(&hUsbDeviceHS)->return_coutlar_data();
    }

    #endif
    //debug_uart.Printf_Ladar(ladar_data_.x, ladar_data_.y);

    debug_uart.printf_DMA("%f,%f,%f,%f\r\n",
                         target_yaw_,yaw,target_chassis_twist_.yaw_rate,dyaw);

    

    Point2D fk_speed;
    fk_speed.x = this->getWorldSpeed().vx;
    fk_speed.y = this->getWorldSpeed().vy;

    SpeedFK_Queue.send(fk_speed);
}
