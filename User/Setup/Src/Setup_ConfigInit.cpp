#include "Setup_ConfigInit.h"


#if DEBUG_M2006

DJI_MotorDemo dji_motor_demo;
#endif

/*================================ debug  机械吸盘 =============================*/

#if ARM_DEMO_DEBUG

fdCANbus* const CAN1_Bus = fdCANbus::getInstance(&hfdcan1); // 获取FDCAN1的唯一实例
DJI_Group ArmGroupCAN1_Low(send_idLow(), CAN1_Bus); // 1~4号M3508/M2006电机
M3508 m3508_ArmLaunch(1, CAN1_Bus);
M3508 m3508_ArmStretch(2, CAN1_Bus);
M3508 m3508_ArmRotate(3, CAN1_Bus);
M3508 m3508_ArmPitch(4, CAN1_Bus);

Arm_InitData_S arm_demoInit_data={
   .max_launchHeight_ = 0.8f,
   .max_stretchLength_ = 0.3f,
   .arm_length_ = 0.3f,

   .stretch_Ratio_ = 0.01f,
   .launch_Ratio_ = 0.01f,
   .rotate_gearRatio_ = 10.0f
};

Robot_ArmDemo arm_demo(arm_demoInit_data);

void arm_motorInit()
{
   ArmGroupCAN1_Low.addMotor(&m3508_ArmLaunch);
   ArmGroupCAN1_Low.addMotor(&m3508_ArmStretch);
   ArmGroupCAN1_Low.addMotor(&m3508_ArmRotate);
   ArmGroupCAN1_Low.addMotor(&m3508_ArmPitch);

   CAN1_Bus->registerMotor(&ArmGroupCAN1_Low);

   CAN1_Bus->registerMotor(&m3508_ArmLaunch);
   CAN1_Bus->registerMotor(&m3508_ArmStretch);
   CAN1_Bus->registerMotor(&m3508_ArmRotate);
   CAN1_Bus->registerMotor(&m3508_ArmPitch);

   CAN1_Bus->init();

   m3508_ArmLaunch.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   m3508_ArmStretch.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   m3508_ArmRotate.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   m3508_ArmPitch.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   
}
#endif

/*============================== debug  机械吸盘 ===============================*/





void debug_init()
{
   /*============================= debug  机械吸盘 ================================*/
#if ARM_DEMO_DEBUG
   arm_motorInit();
   arm_demo.armInit(&m3508_ArmLaunch, &m3508_ArmStretch, &m3508_ArmRotate, &m3508_ArmPitch);
#endif
/*============================== debug  机械吸盘 ===============================*/


/*============================== debug  M2006 ===============================*/
#if DEBUG_M2006
   dji_motor_demo.init();
#endif
/*============================== debug  M2006 ===============================*/

}


void ALL_Setup_ConfigInit(void)
{
   dji_motor_demo.init();
   

   TimeStamp::getInstance().init(&htim4); // 启用时间戳服务
   debug_init();


   //other init
}

