#include "FSM_Controller.h"



void FSM_Controller::loop()
{
    if(!init_flag_)
        return;



    CrsfReceiver::GetInstance(&huart7)->process();

    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);


    switch(airjoy_data_.SWB)
    {
        case 0x00:
            robot_status_ = ALL_STOP;
            break;

        case 0x01:
            robot_status_ = MANUAL_CONTROL;
            break;  

        case 0x02:
            robot_status_ = AUTO_CONTROL;
            break;
    }

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
    case DEBUG_MODE:
        // 调试模式逻辑实现
        debug();
        break;

    default:
        break;
   }

   last_robot_status_ = robot_status_;
}

void FSM_Controller::all_stop()
{
   // 停止所有机构动作的实现
   arm_setup_->setArmStatus(ARM_IDLE);
   chassis_setup_->setChassisStatus(CHASSIS_STOP);
       
}

void FSM_Controller::manual_ctrl()
{
   


//    //底盘线速度控制;
//     if(_tool_Abs(AirJoy::getinstance().SWC - 1000) < 50)
//         chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
//     else
//         chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);

//     //串联臂关节位置控制
//     if(_tool_Abs(AirJoy::getinstance().SWC - 1500) < airjoy_deadzone_ || _tool_Abs(AirJoy::getinstance().SWC - 2000) < airjoy_deadzone_)
//         arm_setup_->setArmStatus(ARM_MANUAL_CONTROL);
//     else
//         arm_setup_->setArmStatus(ARM_IDLE);
    switch(airjoy_data_.SWC)
    {
        case 0x00:
        {
            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);
            arm_setup_->setArmStatus(ARM_IDLE);
            break;
        }
        case 0x01:
        {
            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);
            arm_setup_->setArmStatus(ARM_MANUAL_CONTROL);
            break;  
        }
        case 0x02:
        {
            chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_B);
            arm_setup_->setArmStatus(ARM_IDLE);
            break;
        }
    }

    

}


void FSM_Controller::auto_ctrl()
{
   // 半自动控制模式下的实现
   arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
   chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL);
}


void FSM_Controller::debug()
{
   // 调试模式下的实现

    // arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
}

