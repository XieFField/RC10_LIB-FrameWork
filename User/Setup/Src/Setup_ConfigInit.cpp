  #include "Setup_ConfigInit.h"

fdCANbus* const CAN1_Bus = fdCANbus::getInstance(&hfdcan1); // 获取FDCAN1的唯一实例
DJI_Group DJIGroupCAN1_Low(send_idLow(), CAN1_Bus); // 1~4号M3508/M2006电机
DJI_Group DJIGroupCAN1_High(send_idHigh(), CAN1_Bus); // 5~8号M3508/M2006电机
//crsf_demo crsf_task;
//CrsfReceiver uart_driver_(&huart7);           // 构造 UART_ 成员 uart_driver_;
// CrsfReceiver crsf_rc(&huart7);   // 唯一实例
// crsf_demo  crsf_task;          // RTOS 任务封装


/*==============Controller Instances===========*/
uint8_t laser_rx_buffer[20];
uint8_t laser_rx_buffer1[20];
uint8_t laser_rx_buffer2[20];
//激光测距
LaserPosition laserpos(15,laser_rx_buffer,&huart3);
LaserPosition laserpos1(15,laser_rx_buffer1,&huart6);
LaserPosition laserpos2(15,laser_rx_buffer2,&huart10);
Laser_InstanceManager instance_man;
Locate_Setup set1(&instance_man);
OmniChassis_Setup ChassisOmni(1,2,3); // 轮子半径，最大轮子转速，底盘半径
ArmSetup ARM_Controller(arm_initData);
FSM_Controller Finite_StateMachine;

/*==============Controller Instances===========*/

/*=============================================*/

/*================Motor Instances==============*/

                           /* 底盘 */
M3508 omni_wheel1(1, CAN1_Bus); M3508 omni_wheel2(2, CAN1_Bus); 
M3508 omni_wheel3(3, CAN1_Bus); M3508 omni_wheel4(4, CAN1_Bus);

                           /* 串联臂 */      
M3508 arm_launchMotor(5, CAN1_Bus); M2006 arm_stretchMotor(6, CAN1_Bus);
M3508 arm_rotateMotor(7, CAN1_Bus); M2006 arm_pitchMotor(8, CAN1_Bus);

/*================Motor Instances==============*/


/*============================== debug  DJI_Motor ===============================*/

#if DEBUG_DJI_Motor

#if SPEEDPLANNER_DEMO_DEBUG

SpeedPlanner_Demo speedplanner_demo;
#endif

M2006 m2006_1(5, CAN1_Bus);

DJI_MotorDemo dji_motor_demo;

void dji_motor_Init()
{
   DJIGroupCAN1_High.addMotor(&m2006_1);

   CAN1_Bus->registerMotor(&DJIGroupCAN1_High);

   CAN1_Bus->registerMotor(&m2006_1);

   CAN1_Bus->init();

   m2006_1.pid_init(m2006_speed_pid_params, 0.0f, m2006_angle_pid_params, 0.0f);
}
#endif

/*============================== debug  DJI_Motor ===============================*/


/*================================ debug  机械吸盘 =============================*/

#if ARM_DEMO_DEBUG


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
//激光测距
LaserPosition laserpos(&huart3,&huart6);


void arm_motorInit()
{
   DJIGroupCAN1_Low.addMotor(&m3508_ArmLaunch);
   DJIGroupCAN1_Low.addMotor(&m2006_ArmStretch);
   DJIGroupCAN1_Low.addMotor(&m3508_ArmRotate);
   DJIGroupCAN1_Low.addMotor(&m2006_ArmPitch);

   CAN1_Bus->registerMotor(&DJIGroupCAN1_Low);

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


void debug_init()
{
   /*============================= debug  机械吸盘 ================================*/
#if ARM_DEMO_DEBUG
   arm_motorInit();
   arm_demo.armInit(&m3508_ArmLaunch, &m2006_ArmStretch, &m3508_ArmRotate, &m2006_ArmPitch);
#endif
/*============================== debug  机械吸盘 ===============================*/


/*============================== debug  DJI_Motor ===============================*/
#if DEBUG_DJI_Motor
   dji_motor_Init();
   dji_motor_demo.init(&m2006_1);
#endif
/*============================== debug  M2006 ===============================*/
 
/*============================== debug   speedplanner ===============================*/    
#if SPEEDPLANNER_DEMO_DEBUG

   speedplanner_demo.init();
#endif
/*============================== debug   speedplanner ===============================*/
/*============================== debug  DJI_Motor ===============================*/
#if DEBUG
laserpos.Init();//激光测距
#endif
}

void CAN_Motor_Init(void);

void ALL_Setup_ConfigInit(void)
{

   Position* pos = Position::GetInstance(&huart1);
   pos->InitUART();
   TimeStamp::getInstance().init(&htim4); // 启用时间戳服务
   debug_init();
	
//	crsf_rc.init();
	
  Position* pos = Position::GetInstance(&huart1);
  pos->InitUART();

	 instance_man.RegisterInstance(&laserpos);
	 instance_man.RegisterInstance(&laserpos1);
	 instance_man.RegisterInstance(&laserpos2);
	 instance_man.InstanceManager_Init();
	 set1.locate_setup_init();
}


void CAN_Motor_Init(void)
{
   DJIGroupCAN1_Low.addMotor(&omni_wheel1);
   DJIGroupCAN1_Low.addMotor(&omni_wheel2);
   DJIGroupCAN1_Low.addMotor(&omni_wheel3);
   DJIGroupCAN1_Low.addMotor(&omni_wheel4);

   DJIGroupCAN1_High.addMotor(&arm_launchMotor);
   DJIGroupCAN1_High.addMotor(&arm_stretchMotor);
   DJIGroupCAN1_High.addMotor(&arm_rotateMotor);
   DJIGroupCAN1_High.addMotor(&arm_pitchMotor);

   CAN1_Bus->registerMotor(&DJIGroupCAN1_Low);
   CAN1_Bus->registerMotor(&DJIGroupCAN1_High);

   CAN1_Bus->registerMotor(&omni_wheel1);
   CAN1_Bus->registerMotor(&omni_wheel2);
   CAN1_Bus->registerMotor(&omni_wheel3);
   CAN1_Bus->registerMotor(&omni_wheel4);

   CAN1_Bus->registerMotor(&arm_launchMotor);
   CAN1_Bus->registerMotor(&arm_stretchMotor);
   CAN1_Bus->registerMotor(&arm_rotateMotor);
   CAN1_Bus->registerMotor(&arm_pitchMotor);

   CAN1_Bus->init();

   // 底盘轮子电机PID参数初始化
   omni_wheel1.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   omni_wheel2.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   omni_wheel3.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   omni_wheel4.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);

   // 机械臂电机PID参数初始化
   arm_launchMotor.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   arm_stretchMotor.pid_init(m2006_speed_pid_params, 0.0f, m2006_angle_pid_params, 0.0f);
   arm_rotateMotor.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   arm_pitchMotor.pid_init(m2006_speed_pid_params, 0.0f, m2006_angle_pid_params, 0.0f);
}


