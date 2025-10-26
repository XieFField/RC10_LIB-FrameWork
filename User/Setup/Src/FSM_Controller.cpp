#include "FSM_Controller.h"

void FSM_Controller::loop()
{
    if(!init_flag_)
        return;

    //遥控器链接失败
    if(AirJoy::getInstance().LEFT_X == 0 || AirJoy::getInstance().LEFT_Y == 0 || 
        AirJoy::getInstance().RIGHT_X == 0 || AirJoy::getInstance().RIGHT_Y == 0 ||
        AirJoy::getInstance().SWA == 0 || AirJoy::getInstance().SWB == 0 || 
        AirJoy::getInstance().SWC == 0 || AirJoy::getInstance().SWD == 0)
    {
       airjoy_connected_ = false;
       return;
    }
    else
       airjoy_connected_ = true;

    if(_tool_Abs(AirJoy::getInstance().SWB - 1000) < 50)
        robot_status_ = ALL_STOP;
    else if(_tool_Abs(AirJoy::getInstance().SWB - 1500) < 50)
        robot_status_ = MANUAL_CONTROL;
    else if(_tool_Abs(AirJoy::getInstance().SWB - 2000) < 50)
        robot_status_ = AUTO_CONTROL;


   switch (robot_status_)
   {
    case ALL_STOP:
        all_stop();
        break;

    case MANUAL_CONTROL:
        // 手动控制逻辑实现
        manual_ctrl();
        break;

    case AUTO_CONTROL:
        // 自动控制逻辑实现
        auto_ctrl();
        break;

    default:
        break;
   }

   last_robot_status_ = robot_status_;
}

void FSM_Controller::all_stop()
{
   // 停止所有机构动作的实现
   arm_setup_->setArmStatus(ARM_STOP);
   chassis_setup_->setChassisStatus(CHASSIS_STOP);
   
}

void FSM_Controller::manual_ctrl()
{
   


   //底盘线速度控制;
    if(_tool_Abs(AirJoy::getInstance().SWC - 1000) < 50)
        chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
    else
        chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);

    //串联臂关节位置控制
    if(_tool_Abs(AirJoy::getInstance().SWC - 1500) < airjoy_deadzone_ || _tool_Abs(AirJoy::getInstance().SWC - 2000) < airjoy_deadzone_)
        arm_setup_->setArmStatus(ARM_MANUAL_CONTROL);
    else
        arm_setup_->setArmStatus(ARM_IDLE);

}


void FSM_Controller::auto_ctrl()
{
   // 半自动控制模式下的实现
}


