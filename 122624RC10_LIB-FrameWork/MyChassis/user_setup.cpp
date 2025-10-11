#include "Module_ChassisBase.h"
#include "Motor_DJI.h"
#include "BSP_fdCAN_Driver.h"
#include "MyChassis.h"
#include "user_setup.h"
#include "stm32h7xx_hal.h" 

fdCANbus* const CAN1_Bus = fdCANbus::getInstance(&hfdcan1); 
extern DJI_Group GroupCAN1_Low;
M3508 M3508_1(1, CAN1_Bus),M3508_2(2, CAN1_Bus),M3508_3(3, CAN1_Bus),M3508_4(4, CAN1_Bus);

MyChassis mychassis(0.05f, 100.0f, 0.2f, 0.2f); // wheel_radius=50mm, max_wheel_rpm=100rpm, wheel_distance_x=200mm, wheel_distance_y=200mm
DJI_Group DJI_Group_1(send_idLow(), CAN1_Bus); 

// 定义 wheel_motor 指针数组，指向四个 M3508 电机
DJI_Motor* wheel_motor[4] = {&M3508_1, &M3508_2, &M3508_3, &M3508_4};

volatile int my_start_signal = 0;

void MyChassisController::init()
{

    PID_Param_Config speed_pid_params = {
    .kp = 18.0f,
    .ki = 0.015f,
    .kd = 0.0f,
    .I_Outlimit = 8000.0f, 
    .isIOutlimit = true, 
    .output_limit = 15000.0f,   
    .deadband = 5.0f 
};

PID_Param_Config angle_pid_params = {
    .kp = 30.0f,
    .ki = 0.0f,
    .kd = 1.1f,
    .I_Outlimit = 0.0f, 
    .isIOutlimit = true, 
    .output_limit = 400.0f,   
    .deadband = 0.8f // 
};
    for(int i=0;i<4;++i)
    {
        DJI_Group_1.addMotor(wheel_motor[i]);
        CAN1_Bus->registerMotor(wheel_motor[i]);
    }
    CAN1_Bus->registerMotor(&DJI_Group_1);
    CAN1_Bus->init();
    mychassis.registerMotor(0, &M3508_1); // 前左
    mychassis.registerMotor(1, &M3508_2); // 前右
    mychassis.registerMotor(2, &M3508_3); // 后左
    mychassis.registerMotor(3, &M3508_4); // 后右

    mychassis.reset_AccLimitStatus(true); // 启用加速度限制
    mychassis.reset_AccValue(1.0f); // 1.0 m/s^2

}

void MyChassisController::loop()
{
    Robot_Twist target_speed;
   if(my_start_signal == 1)
   {
        // 前进0.5 m/s
        target_speed.vx = 0.5f; // m/s
        target_speed.vy = 0.0f; // m/s
        target_speed.yaw_rate = 0.0f; // rad/s
        HAL_Delay(2000);
   }
    else if(my_start_signal == 2)
    {
        //侧移0 .5m/s
        target_speed.vx = 0.0f; // m/s
        target_speed.vy = 0.5f; // m/s
        target_speed.yaw_rate = 0.0f; // rad/s
        HAL_Delay(2000);
    }
    else if(my_start_signal == 3)
    {
        //原地旋转1.0 rad/s
        target_speed.vx = 0.0f; // m/s
        target_speed.vy = 0.0f; // m/s
        target_speed.yaw_rate = 1.0f; // rad/s
        HAL_Delay(2000);
    }
    else
    {
        // 停止
        target_speed.vx = 0.0f; // m/s
        target_speed.vy = 0.0f; // m/s
        target_speed.yaw_rate = 0.0f; // rad/s
    }

    mychassis.inverseKinematics(target_speed);
    mychassis.updateKinematics();


};