#include "omni_chassisSetup.h"

#define vmax 10/500.f  // 线速度
#define wrmax 1/500.f   // 角速度

PID_Param_Config yaw_pid_params = {
    .kp=0.2f,
    .ki=0.0f,
    .kd=0.00006f,
    .I_Outlimit=1.0f,
    .isIOutlimit=false, 
    .output_limit=1.0f,
    .deadband=0.05f
};


void  OmniChassis_Setup::init()
    {
        if(this->wheels_[0] == nullptr ||this->wheels_[1] == nullptr ||
           this->wheels_[2] == nullptr ||this->wheels_[3] == nullptr)
            init_flag = false;
        
        this->start(osPriorityHigh, 256);
        init_flag = true;
					 
        this->yaw_pid.set_params(yaw_pid_params, 0.2f);
        
    }
float OmniChassis_Setup::yaw_adjust(float now_angle, float target_angle_)
{
    real_angle = now_angle;
    target_angle = target_angle_;
    float yaw_output = yaw_pid.pid_calc(target_angle, real_angle);
    return yaw_output;
}


void OmniChassis_Setup::loop()
{
    if (!init_flag)
        return;
    
    switch (chassis_status_)
    {
        case CHASSIS_MANUAL_CONTROL_A://底盘手动控制模式（线速度和角速度都可以控制）
        {
            chassis_control_manualA();
            break;
        }

        case CHASSIS_MANUAL_CONTROL_B://底盘手动控制模式（线速度可控，角度锁定）
        {
            chassis_control_manualB();  
            break;
        }
        case CHASSIS_AUTO_CONTROL://底盘自动控制模式（线速度和角速度都可以控制）
        {
            chassis_control_auto();
            break;
        }

        case CHASSIS_STOP:
        {
            chassis_control_stop();
            break;
        }
        default:
            break;
    }
    
    this->update();
}

void OmniChassis_Setup::chassis_control_manualA()
{
    
         if(_tool_Abs(AirJoy::getinstance().LEFT_X - 1500) < 50)
            {
                target_chassis_twist_.vx = 0.0f;
            }
            else
            {
                target_chassis_twist_.vx = (AirJoy::getinstance().LEFT_X - 1500) * vmax;
            }

            if(_tool_Abs(AirJoy::getinstance().LEFT_Y - 1500) < 50)
            {
                target_chassis_twist_.vy = 0.0f;
            }
            else
            {
                target_chassis_twist_.vy = (AirJoy::getinstance().LEFT_Y - 1500) * vmax;
            }

            if(_tool_Abs(AirJoy::getinstance().RIGHT_X - 1500) < 50)
            {
                target_chassis_twist_.yaw_rate = 0.0f;
            }
            else
            {
                target_chassis_twist_.yaw_rate = (AirJoy::getinstance().RIGHT_X - 1500) * wrmax;
            }
            
						setWorldSpeed(target_chassis_twist_);
            // 记录当前角度作为锁定参考（切换到模式B时使用）
            yaw_lock_angle = RealPosData.world_yaw;
}
        
void OmniChassis_Setup::chassis_control_manualB()
{
    // 获取当前角度
    yaw_real_angle = RealPosData.world_yaw;

    // 计算角度修正量，使底盘保持在锁定角度
    yaw_correction = yaw_adjust(yaw_real_angle, yaw_lock_angle);
    debug_uart.printf_DMA("%f,%f\r\n",yaw_real_angle, yaw_lock_angle);
    target_chassis_twist_.yaw_rate = yaw_correction;
    if(_tool_Abs(AirJoy::getinstance().LEFT_X - 1500) < 50)
     {
        target_chassis_twist_.vx = 0.0f;
     }
    else
    {
        target_chassis_twist_.vx = (AirJoy::getinstance().LEFT_X - 1500) * vmax;
    }
    
    if(_tool_Abs(AirJoy::getinstance().LEFT_Y - 1500) < 50)
     {
        target_chassis_twist_.vy = 0.0f;
     }
    else
    {
        target_chassis_twist_.vy = (AirJoy::getinstance().LEFT_Y - 1500) * vmax;
    }
    
		setWorldSpeed(target_chassis_twist_);
}

void OmniChassis_Setup::chassis_control_auto()
{
    
}

void OmniChassis_Setup::chassis_control_stop()
{  
        target_chassis_twist_.vx = 0.0f;
        target_chassis_twist_.vy = 0.0f;
        target_chassis_twist_.yaw_rate = 0.0f;
        this->world_target_twist_= target_chassis_twist_;
}