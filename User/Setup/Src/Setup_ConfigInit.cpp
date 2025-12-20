<<<<<<< Updated upstream
  #include "Setup_ConfigInit.h"

fdCANbus* const CAN1_Bus = fdCANbus::getInstance(&hfdcan1); // »ñÈ¡FDCAN1µÄÎ¨Ò»ÊµÀý
DJI_Group DJIGroupCAN1_Low(send_idLow(), CAN1_Bus); // 1~4ºÅM3508/M2006µç»ú
DJI_Group DJIGroupCAN1_High(send_idHigh(), CAN1_Bus); // 5~8ºÅM3508/M2006µç»ú

/*==============Controller Instances===========*/
=======
#include "Setup_ConfigInit.h"
 // ï¿½â²¿ï¿½ï¿½ï¿½ï¿½USBï¿½ï¿½ï¿½ï¿½ï¿½è±¸ï¿½ï¿½ï¿½
extern "C" 
{
        extern USBD_HandleTypeDef hUsbDeviceHS;
}
fdCANbus* const CAN1_Bus = fdCANbus::getInstance(&hfdcan1); // ï¿½ï¿½È¡FDCAN1ï¿½ï¿½Î¨Ò»Êµï¿½ï¿½
DJI_Group DJIGroupCAN1_Low(send_idLow(), CAN1_Bus); // 1~4ï¿½ï¿½M3508/M2006ï¿½ï¿½ï¿½
DJI_Group DJIGroupCAN1_High(send_idHigh(), CAN1_Bus); // 5~8ï¿½ï¿½M3508/M2006ï¿½ï¿½ï¿½

fdCANbus* const CAN2_Bus = fdCANbus::getInstance(&hfdcan2); // ï¿½ï¿½È¡FDCAN2ï¿½ï¿½Î¨Ò»Êµï¿½ï¿½
DJI_Group DJIGroupCAN2_Low(send_idLow(), CAN2_Bus); // 1~4ï¿½ï¿½M3508/M2006ï¿½ï¿½ï¿½
DJI_Group DJIGroupCAN2_High(send_idHigh(), CAN2_Bus); // 5~8ï¿½ï¿½M3508/M2006ï¿½ï¿½ï¿½


/*==============Controller Instances===========*/
uint8_t laser_rx_buffer[20];
uint8_t laser_rx_buffer1[20];
uint8_t laser_rx_buffer2[20];
//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//USB_CDC_ cdc(&hUsbDeviceHS);
LaserPosition laserpos(15,laser_rx_buffer,&huart3);
LaserPosition laserpos1(15,laser_rx_buffer1,&huart6);
LaserPosition laserpos2(15,laser_rx_buffer2,&huart10);
Laser_InstanceManager instance_man;
Locate_Setup set1(&instance_man);

OmniChassis_Setup ChassisOmni(0.442f/2.f,420, 0.74f, 0.8363f, true); // ï¿½ï¿½ï¿½Ó°ë¾¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×ªï¿½Ù£ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ ï¿½ï¿½
>>>>>>> Stashed changes

OmniChassis_Setup ChassisOmni(1,2,3); // ÂÖ×Ó°ë¾¶£¬×î´óÂÖ×Ó×ªËÙ£¬µ×ÅÌ°ë¾¶
ArmSetup ARM_Controller(arm_initData);
FSM_Controller Finite_StateMachine;
Robot_WeaponSage_Setup Weapon_Controller(initData_);
/*==============Controller Instances===========*/

/*=============================================*/

/*================Motor Instances==============*/

                           /* ï¿½ï¿½ï¿½ï¿½ */
M3508 omni_wheel1(1, CAN1_Bus); M3508 omni_wheel2(2, CAN1_Bus); 
M3508 omni_wheel3(3, CAN1_Bus); M3508 omni_wheel4(4, CAN1_Bus);

                           /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ */      
M3508 arm_launchMotor(5, CAN1_Bus); M2006 arm_stretchMotor(6, CAN1_Bus);
M3508 arm_rotateMotor(7, CAN1_Bus); M2006 arm_pitchMotor(8, CAN1_Bus);

M3508 Weapon_launchMotor(1, CAN2_Bus); M2006 Weapon_clawMotor(2, CAN2_Bus);
M2006 Weapon_traverseMotor(3, CAN2_Bus); DM_Motor Weapon_wristMotor(J4310_Type, 0x05,0x05, CAN2_Bus);
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


/*================================ debug  ï¿½ï¿½Ðµï¿½ï¿½ï¿½ï¿½ =============================*/

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
//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
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

/*============================== debug  ï¿½ï¿½Ðµï¿½ï¿½ï¿½ï¿½ ===============================*/


void debug_init()
{
   /*============================= debug  ï¿½ï¿½Ðµï¿½ï¿½ï¿½ï¿½ ================================*/
#if ARM_DEMO_DEBUG
   arm_motorInit();
   arm_demo.armInit(&m3508_ArmLaunch, &m2006_ArmStretch, &m3508_ArmRotate, &m2006_ArmPitch);
#endif
/*============================== debug  ï¿½ï¿½Ðµï¿½ï¿½ï¿½ï¿½ ===============================*/


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
laserpos.Init();//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
#endif
}

void CAN_Motor_Init(void);

void ALL_Setup_ConfigInit(void)
{

<<<<<<< Updated upstream
   CAN_Motor_Init();

   TimeStamp::getInstance().init(&htim4); // ÆôÓÃÊ±¼ä´Á·þÎñ
=======
   Position* pos = Position::GetInstance(&huart1);
   pos->InitUART();
   TimeStamp::getInstance().init(&htim4); // ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
>>>>>>> Stashed changes
   debug_init();

   Position* pos = Position::GetInstance(&huart1);
   pos->InitUART();

   ARM_Controller.init(&arm_launchMotor, &arm_stretchMotor, &arm_rotateMotor, &arm_pitchMotor);
   ARM_Controller.setArmStatus(ARM_IDLE);
   
   Weapon_Controller.init(&Weapon_launchMotor, &Weapon_clawMotor,&Weapon_traverseMotor, &Weapon_wristMotor);
   Weapon_Controller.setWeaponSageStatus(WEAPONSAGE_CALIBRATE);

   ChassisOmni.registerWheelMotor(0, &omni_wheel1);
   ChassisOmni.registerWheelMotor(1, &omni_wheel2);
   ChassisOmni.registerWheelMotor(2, &omni_wheel3);
   ChassisOmni.registerWheelMotor(3, &omni_wheel4);
   ChassisOmni.init();

   ChassisOmni.setChassisStatus(CHASSIS_STOP);

   Finite_StateMachine.registerArmSetup(&ARM_Controller);
   Finite_StateMachine.registerChassisSetup(&ChassisOmni);

   Finite_StateMachine.init();
		    // »ñÈ¡Positionµ¥Àý²¢³õÊ¼»¯UART
   debug_init();
	 

<<<<<<< Updated upstream
	
   //other init
=======
   //  CrsfReceiver* crsf_rc = CrsfReceiver::GetInstance(&huart7);
   

	 instance_man.RegisterInstance(&laserpos);
	 instance_man.RegisterInstance(&laserpos1);
	 instance_man.RegisterInstance(&laserpos2);
	 instance_man.InstanceManager_Init();
//ï¿½ï¿½ï¿½ï¿½ï¿½Ø¶ï¿½Î»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý³ï¿½Ê¼ï¿½ï¿½
   set1.init();	
	 set1.laser_initData_.d=0.5;
	 set1.locate_setup_init();
	 set1.set_startToLRL(true);
//ï¿½×´ï¶¨Î»Êµï¿½ï¿½ï¿½ï¿½
	 Lader_position*ladar=Lader_position::GetInstance(&hUsbDeviceHS);
>>>>>>> Stashed changes
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

   DJIGroupCAN2_Low.addMotor(&Weapon_launchMotor);
   DJIGroupCAN2_Low.addMotor(&Weapon_clawMotor);
   DJIGroupCAN2_Low.addMotor(&Weapon_traverseMotor);

<<<<<<< Updated upstream
   // µ×ÅÌÂÖ×Óµç»úPID²ÎÊý³õÊ¼»¯
   omni_wheel1.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   omni_wheel2.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   omni_wheel3.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   omni_wheel4.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);

   // »úÐµ±Ûµç»úPID²ÎÊý³õÊ¼»¯
   arm_launchMotor.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   arm_stretchMotor.pid_init(m2006_speed_pid_params, 0.0f, m2006_angle_pid_params, 0.0f);
   arm_rotateMotor.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
=======
   CAN2_Bus->registerMotor(&DJIGroupCAN2_Low);

   CAN2_Bus->registerMotor(&Weapon_launchMotor);
   CAN2_Bus->registerMotor(&Weapon_clawMotor);
   CAN2_Bus->registerMotor(&Weapon_traverseMotor);
   CAN2_Bus->registerMotor(&Weapon_wristMotor);

   CAN1_Bus->init();
   CAN2_Bus->init();

   // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Óµï¿½ï¿½PIDï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½
   omni_wheel1.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, m3508_angle_pid_params, 0.0f);
   omni_wheel2.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, m3508_angle_pid_params, 0.0f);
   omni_wheel3.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, m3508_angle_pid_params, 0.0f);
   // omni_wheel4.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);

   // ï¿½ï¿½Ðµï¿½Ûµï¿½ï¿½PIDï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½
   
   PID_Param_Config arm_3508_speedPID = m3508_speed_pid_paramsForSpeedMotor;
   PID_Param_Config arm_3508_anglePID = m3508_angle_pid_params;
   arm_3508_anglePID.output_limit = 200.0f;
   // arm_3508_speedPID.output_limit = 420.0f; // ï¿½ï¿½ï¿½Ý»ï¿½Ðµï¿½ï¿½Òªï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Þ·ï¿½
   
   arm_launchMotor.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, arm_3508_anglePID, 0.0f);

   PID_Param_Config arm_strech_anglePID = m2006_angle_pid_params;
   arm_strech_anglePID.output_limit = 500.0f;
   arm_stretchMotor.pid_init(m2006_speed_pid_params, 0.0f, arm_strech_anglePID, 0.0f);
   arm_rotateMotor.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, arm_3508_anglePID, 0.0f);
>>>>>>> Stashed changes
   arm_pitchMotor.pid_init(m2006_speed_pid_params, 0.0f, m2006_angle_pid_params, 0.0f);

   Weapon_launchMotor.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, arm_3508_anglePID, 0.0f);
   Weapon_clawMotor.pid_init(m2006_speed_pid_params, 0.0f, arm_strech_anglePID, 0.0f);
   Weapon_traverseMotor.pid_init(m2006_speed_pid_params, 0.0f, arm_strech_anglePID, 0.0f);

}


