#include "FSM_Controller.h"

void FSM_Controller::loop()
{
    if(!init_flag_)
        return;

    //遥控器链接失败
    if(air_joy.LEFT_X == 0 || air_joy.LEFT_Y == 0 || air_joy.RIGHT_X == 0 || air_joy.RIGHT_Y == 0 ||
       air_joy.SWA == 0 || air_joy.SWB == 0 || air_joy.SWC == 0 || air_joy.SWD == 0)
    {
       airjoy_connected_ = false;
       return;
    }
    else
       airjoy_connected_ = true;
    
    if(_tool_Abs(air_joy.SWB - 1000) < 50)
        robot_status_ = ALL_STOP;
    else if(_tool_Abs(air_joy.SWB - 1500) < 50)
        robot_status_ = MANUAL_CONTROL;
    else if(_tool_Abs(air_joy.SWB - 2000) < 50)
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
   
}

void FSM_Controller::manual_ctrl()
{
   
    if(last_robot_status_ != MANUAL_CONTROL)//若首次非此模式，需复制一下上次状态，免得跳变
    {
        /*串联臂*/
        last_joint_status_ = arm_setup_->get_currentJointStatus();
        target_joint_status_ = last_joint_status_;
        last_robot_status_ = MANUAL_CONTROL;

        /*底盘*/
    }

   //底盘线速度控制;
    if(_tool_Abs(air_joy.SWC - 1000) < 50)
        chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
    else
        chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);

    //串联臂关节位置控制
    if(_tool_Abs(air_joy.SWC - 1500) < airjoy_deadzone_ || _tool_Abs(air_joy.SWC - 2000) < airjoy_deadzone_)
        arm_setup_->setArmStatus(ARM_MANUAL_CONTROL);
    else
        arm_setup_->setArmStatus(ARM_IDLE);

}


void FSM_Controller::auto_ctrl()
{
   // 半自动控制模式下的实现
}


