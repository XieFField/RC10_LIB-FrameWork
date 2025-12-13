#include "FSM_Controller.h"


RmPocketData_t test_airjoy;
void FSM_Controller::loop()
{
    if(!init_flag_)
        return;

//     //遥控器链接失败
//     if(AirJoy::getinstance().LEFT_X == 0 || AirJoy::getinstance().LEFT_Y == 0 || 
//         AirJoy::getinstance().RIGHT_X == 0 || AirJoy::getinstance().RIGHT_Y == 0 ||
//         AirJoy::getinstance().SWA == 0 || AirJoy::getinstance().SWB == 0 || 
//         AirJoy::getinstance().SWC == 0 || AirJoy::getinstance().SWD == 0)
//     {
//        airjoy_connected_ = false;
// //       return;
//     }
//     else
//        airjoy_connected_ = true;

//     if(debug_flag_ == 1)
//     {
//         robot_status_ = DEBUG_MODE;
//     }
//     else
//     {
//         if(_tool_Abs(AirJoy::getinstance().SWB - 1000) < 50)
//                 robot_status_ = ALL_STOP;
//         else if(_tool_Abs(AirJoy::getinstance().SWB - 1500) < 50)
//             robot_status_ = MANUAL_CONTROL;
//         else if(_tool_Abs(AirJoy::getinstance().SWB - 2000) < 50)
//             robot_status_ = AUTO_CONTROL;
//     }

   // airjoy->process();
    robot_status_ = MANUAL_CONTROL;
    // 消费 CRSF 接收环形缓冲，推进状态机
//    CrsfReceiver::GetInstance(&huart7)->process();
//    CrsfReceiver::GetInstance(&huart7)->getControlData(&test_airjoy);
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
   arm_setup_->setArmStatus(ARM_STOP);
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

    chassis_setup_->setChassisStatus(CHASSIS_MANUAL_CONTROL_A);

}


void FSM_Controller::auto_ctrl()
{
   // 半自动控制模式下的实现
   arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
   chassis_setup_->setChassisStatus(CHASSIS_AUTO_CONTROL);
}

airjoy_S debug_airjoy;

void FSM_Controller::debug()
{
   // 调试模式下的实现
    debug_airjoy.SWA = AirJoy::getinstance().SWA;
    debug_airjoy.SWB = AirJoy::getinstance().SWB;
    debug_airjoy.SWC = AirJoy::getinstance().SWC;
    debug_airjoy.SWD = AirJoy::getinstance().SWD;

    debug_airjoy.LEFT_X = AirJoy::getinstance().LEFT_X;

    debug_airjoy.LEFT_Y = AirJoy::getinstance().LEFT_Y;
    debug_airjoy.RIGHT_X = AirJoy::getinstance().RIGHT_X;
    debug_airjoy.RIGHT_Y = AirJoy::getinstance().RIGHT_Y;

    arm_setup_->setArmStatus(ARM_AUTO_CONTROL);
}

