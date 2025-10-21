#include "Setup_ConfigInit.h" 

#if DEBUG_M2006

DJI_MotorDemo dji_motor_demo;
#endif

/*================================ debug  机械吸盘 =============================*/

#if ARM_DEMO_DEBUG

fdCANbus* const CAN1_Bus = fdCANbus::getInstance(&hfdcan1); // 获取FDCAN1的唯一实例
DJI_Group ArmGroupCAN1_Low(send_idLow(), CAN1_Bus); // 1~4号M3508/M2006电机
M3508 m3508_ArmLaunch(1, CAN1_Bus);
M2006 m2006_ArmStretch(2, CAN1_Bus);
M3508 m3508_ArmRotate(3, CAN1_Bus);
M2006 m2006_ArmPitch(4, CAN1_Bus);

Arm_InitData_S arm_demoInit_data={
   .max_launchHeight_ = 0.8f,
   .max_stretchLength_ = 0.130f,
   .arm_length_ = 0.3f,

   .stretch_Ratio_ = 0.03098f,
   .launch_Ratio_ = 0.1f,
   .rotate_gearRatio_ = 10.0f,
   .pitch_gearRatio_ = 10.0f,
};

Robot_ArmDemo arm_demo(arm_demoInit_data);

void arm_motorInit()
{
   ArmGroupCAN1_Low.addMotor(&m3508_ArmLaunch);
   ArmGroupCAN1_Low.addMotor(&m2006_ArmStretch);
   ArmGroupCAN1_Low.addMotor(&m3508_ArmRotate);
   ArmGroupCAN1_Low.addMotor(&m2006_ArmPitch);

   CAN1_Bus->registerMotor(&ArmGroupCAN1_Low);

   CAN1_Bus->registerMotor(&m3508_ArmLaunch);
   CAN1_Bus->registerMotor(&m2006_ArmStretch);
   CAN1_Bus->registerMotor(&m3508_ArmRotate);
   CAN1_Bus->registerMotor(&m2006_ArmPitch);

   CAN1_Bus->init();

   m3508_ArmLaunch.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   m2006_ArmStretch.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   m3508_ArmRotate.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   m2006_ArmPitch.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);

}
#endif

/*============================== debug  机械吸盘 ===============================*/

/*============================== debug  MyChassis ===============================*/

#if MYCHASSIS_DEMO_DEBUG

fdCANbus* const CAN1_Bus = fdCANbus::getInstance(&hfdcan1); 
DJI_Group DJI_Group_1(send_idLow(), CAN1_Bus); 
M3508 M3508_1(1, CAN1_Bus),M3508_2(2, CAN1_Bus),M3508_3(3, CAN1_Bus),M3508_4(4, CAN1_Bus);

DJI_Motor* wheel_[4] = {&M3508_1, &M3508_2, &M3508_3, &M3508_4}; 

MyChassisController<4> myChassisController(0.085f, 7000.0f, 0.64f);

void MyChassis_Init()
{
   DJI_Group_1.addMotor(&M3508_1);
   DJI_Group_1.addMotor(&M3508_2);
   DJI_Group_1.addMotor(&M3508_3);
   DJI_Group_1.addMotor(&M3508_4);
  
   CAN1_Bus->registerMotor(&DJI_Group_1);

   CAN1_Bus->registerMotor(&M3508_1); // 前左
   CAN1_Bus->registerMotor(&M3508_2); // 前右
   CAN1_Bus->registerMotor(&M3508_3); // 后左
   CAN1_Bus->registerMotor(&M3508_4); // 后右

   
   M3508_1.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   M3508_2.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   M3508_3.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   M3508_4.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);

   myChassisController.init(wheel_);
   
   CAN1_Bus->registerMotor(&DJI_Group_1);
   CAN1_Bus->init();

}

#endif

/*============================== debug  MyChassis ===============================*/



void debug_init()
{
   /*============================= debug  机械吸盘 ================================*/
#if ARM_DEMO_DEBUG
   arm_motorInit();
   arm_demo.armInit(&motors);
#endif
/*============================== debug  机械吸盘 ===============================*/


/*============================== debug  M2006 ===============================*/
#if DEBUG_M2006
   dji_motor_demo.init();
#endif
/*============================== debug  M2006 ===============================*/
#if MYCHASSIS_DEMO_DEBUG
	MyChassis_Init();
#endif
}


void ALL_Setup_ConfigInit(void)
{

   

   TimeStamp::getInstance().init(&htim4); // 启用时间戳服务
   debug_init();


   //other init
}

