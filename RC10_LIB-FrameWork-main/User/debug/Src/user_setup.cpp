//#include "Chassis.h"
//#include "main.h"
//#include "math.h"
//#include "Motor_DJI.h"
//#include "BSP_fdCAN_Driver.h"
//#include "APP_PID.h"
//#include "stm32h7xx_hal.h"

//fdCANbus* CAN1_Bus = fdCANbus::getInstance(&hfdcan1);

//M3508 wheel_motor[4] = {
//    M3508(0x201,CAN1_Bus), // ID 0x201
//    M3508(0x202,CAN1_Bus), // ID 0x202
//    M3508(0x203,CAN1_Bus), // ID 0x203
//    M3508(0x204,CAN1_Bus) // ID 0x204
//};
//DJI_Group DJI_Group1(0x200,CAN1_Bus);
//QuanChassis chassis(1.0f, 0.6); // 3000 RPM, 1.0m wheel circle, 0.6m wheel radius

//// 初始化电机PID参数
//    PID_Param_Config test_m3508_speed_pid_params = {
//        .kp = 18.0f,
//        .ki = 0.015f,
//        .kd = 0.0f,
//        .I_Outlimit = 8000.0f, 
//        .isIOutlimit = true, 
//        .output_limit = 15000.0f,   
//        .deadband = 5.0f 
//    };

//    PID_Param_Config test_m3508_angle_pid_params = {
//        .kp = 30.0f,
//        .ki = 0.0f,
//        .kd = 1.1f,
//        .I_Outlimit = 0.0f, 
//        .isIOutlimit = true, 
//        .output_limit = 400.0f,   
//        .deadband = 0.8f 
//    };

//Wheel wheel[4];
//void user_setup()
//{
//    // 初始化CAN总线
//    CAN1_Bus->init();

//    
//    // 将电机添加到DJI组
//    for(int i = 0; i < 4; i++) {
//        wheel[i].motor = &wheel_motor[i];
//        DJI_Group1.addMotor(&wheel_motor[i]);
//        wheel[i].motor->pid_init(test_m3508_speed_pid_params, 0.0f, test_m3508_angle_pid_params, 0.0f);
//    }

//    // 注册电机组和单个电机到CAN总线
//    CAN1_Bus->registerMotor(&DJI_Group1);
//    CAN1_Bus->init();

//    for(int i = 0; i < 4; i++) {
//        CAN1_Bus->registerMotor(&wheel_motor[i]);
//    }

//    chassis.registerWheelMotor(0, &wheel_motor[0]); // 左前轮
//    chassis.registerWheelMotor(1, &wheel_motor[1]); // 右
//    chassis.registerWheelMotor(2, &wheel_motor[2]); // 左后轮
//    chassis.registerWheelMotor(3, &wheel_motor[3]); // 右
// 
//    
//};

//class ChassisController: public RtosTask {
//public: 
//    ChassisController() : RtosTask("ChassisController", 1){}
//    void init();
//    void loop() override;
//    //Debug_Printf debug_uart;
//};

