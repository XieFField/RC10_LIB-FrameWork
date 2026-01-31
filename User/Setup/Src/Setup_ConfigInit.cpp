#include "Setup_ConfigInit.h"
 // 外部声明USB高速设备句柄
extern "C" 
{
   extern USBD_HandleTypeDef hUsbDeviceHS;
}
fdCANbus* const CAN1_Bus = fdCANbus::getInstance(&hfdcan1); // 获取FDCAN1的唯一实例
fdCANbus* const CAN2_Bus = fdCANbus::getInstance(&hfdcan2); // 获取FDCAN2的唯一实例
fdCANbus* const CAN3_Bus = fdCANbus::getInstance(&hfdcan3);

DJI_Group DJIGroupCAN1_Low(send_idLow(), CAN1_Bus); // 1~4号M3508/M2006电机
DJI_Group DJIGroupCAN1_High(send_idHigh(), CAN1_Bus); // 5~8号M3508/M2006电机

DJI_Group DJIGroupCAN2_Low(send_idLow(), CAN2_Bus); // 1~4号M3508/M2006电机

Point2D lader_install_offset = {0.0f, 0.0f}; // 激光雷达安装偏移，单位米
Point2D arm_install_offset = {0.480f, 0.02f};   // 机械臂安装偏移，单位米


/*==============Controller Instances===========*/
uint8_t laser_rx_buffer[20];
uint8_t laser_rx_buffer1[20];
uint8_t laser_rx_buffer2[20];
//激光测距
//USB_CDC_ cdc(&hUsbDeviceHS);
USB_CDC_ usb_1(&hUsbDeviceHS);
LaserPosition laserpos(15,laser_rx_buffer,&huart3);
LaserPosition laserpos1(15,laser_rx_buffer1,&huart6);
LaserPosition laserpos2(15,laser_rx_buffer2,&huart10);
Laser_InstanceManager instance_man;

Chassis_Omni<3>::init_config chassis_initData = {
    .wheel_radius = 0.15f/2.f,
    .max_wheel_rpm = 420,
    .wheels[0] = {
        .x = 0.0f,
        .y = 0.375f,
        .theta = 0.0f  // 单位：度
    },
    .wheels[1] = {
        .x = -0.37f,
        .y = -0.375f,
        .theta = -63.741f + 180.0f  // 单位：度
    },
    .wheels[2] = {
        .x = 0.37f,
        .y = -0.375f,
        .theta = 63.741f + 180.0f  // 单位：度
    }
};
OmniChassis_Setup ChassisOmni(chassis_initData); // 轮子半径，最大轮子转速，底盘 底 腰



FSM_Controller Finite_StateMachine;
ArmSetup ARM_Controller(arm_initData);
Robot_WeaponSage_Setup Weapon_Controller(initData_);
test test_task;
/*==============Controller Instances===========*/

/*=============================================*/

/*================Motor Instances==============*/

                           /* 锟斤拷锟斤拷 */
M3508 omni_wheel1(1, CAN1_Bus); M3508 omni_wheel2(2, CAN1_Bus); 
M3508 omni_wheel3(3, CAN1_Bus); M3508 omni_wheel4(4, CAN1_Bus);

                           /* 串联臂 */      
M3508 arm_launchMotor(5, CAN1_Bus); M2006 arm_stretchMotor(8, CAN1_Bus);
M3508 arm_rotateMotor(7, CAN1_Bus); M2006 arm_pitchMotor(6, CAN1_Bus);

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


/*================================ debug  锟斤拷械锟斤拷锟斤拷 =============================*/

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
//锟斤拷锟斤拷锟斤拷
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

/*============================== debug  锟斤拷械锟斤拷锟斤拷 ===============================*/


void debug_init()
{
   /*============================= debug  锟斤拷械锟斤拷锟斤拷 ================================*/
#if ARM_DEMO_DEBUG
   arm_motorInit();
   arm_demo.armInit(&m3508_ArmLaunch, &m2006_ArmStretch, &m3508_ArmRotate, &m2006_ArmPitch);
#endif
/*============================== debug  锟斤拷械锟斤拷锟斤拷 ===============================*/


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
laserpos.Init();//锟斤拷锟斤拷锟斤拷
#endif


	SystemDetectTaskHandle = osThreadNew(StartSystemDetectTask, NULL, &SystemDetectTask_attributes);
}

void CAN_Motor_Init(void);

Locate_Setup* set1 = Locate_Setup::getInstance();

void ALL_Setup_ConfigInit(void)
{
    test_task.init();
   // Position* pos = Position::GetInstance(&huart1);
   // pos->InitUART();

   HWT101CT* imu = HWT101CT::GetInstance(&huart1);
   imu->InitUART();
   TimeStamp::getInstance().init(&htim4); // 启用时间戳服务
   //debug_init();
	
   CAN_Motor_Init();

   ARM_Controller.init(&arm_launchMotor, &arm_stretchMotor, &arm_rotateMotor, &arm_pitchMotor);
   ARM_Controller.setArmStatus(ARM_IDLE);
   
   Weapon_Controller.init(&Weapon_launchMotor, &Weapon_clawMotor,&Weapon_traverseMotor, &Weapon_wristMotor);
   Weapon_Controller.setWeaponSageControlStatus(WEAPONSAGE_CALIBRATE);

   ChassisOmni.registerWheelMotor(0, &omni_wheel1);
   ChassisOmni.registerWheelMotor(1, &omni_wheel2);
   ChassisOmni.registerWheelMotor(2, &omni_wheel3);
   // ChassisOmni.registerWheelMotor(3, &omni_wheel4);
   ChassisOmni.init();

   ChassisOmni.setChassisStatus(CHASSIS_STOP);

   Finite_StateMachine.registerArmSetup(&ARM_Controller);
   Finite_StateMachine.registerChassisSetup(&ChassisOmni);
   Finite_StateMachine.registerWeaponSageSetup(&Weapon_Controller);

   Finite_StateMachine.init();


   CrsfReceiver* crsf_rc = CrsfReceiver::GetInstance(&huart7);
   crsf_rc->init();



   

	 instance_man.RegisterInstance(&laserpos);
	 instance_man.RegisterInstance(&laserpos1);
	 instance_man.RegisterInstance(&laserpos2);
	 instance_man.InstanceManager_Init();
//激光重定位解析数据初始化
	 
     set1->init(&instance_man,&usb_1,lader_install_offset ,arm_install_offset);	
     set1->laser_initData_.d=0.5;
     set1->locate_setup_init();
     set1->set_startToLRL(true);
//雷达定位实例化
	//  Lader_position*ladar=Lader_position::GetInstance(&hUsbDeviceHS);
   
}


void CAN_Motor_Init(void)
{
   DJIGroupCAN1_Low.addMotor(&omni_wheel1);
   DJIGroupCAN1_Low.addMotor(&omni_wheel2);
   DJIGroupCAN1_Low.addMotor(&omni_wheel3);
//   DJIGroupCAN1_Low.addMotor(&omni_wheel4);

   DJIGroupCAN1_High.addMotor(&arm_launchMotor);
   DJIGroupCAN1_High.addMotor(&arm_stretchMotor);
   DJIGroupCAN1_High.addMotor(&arm_rotateMotor);
   DJIGroupCAN1_High.addMotor(&arm_pitchMotor);

   CAN1_Bus->registerMotor(&DJIGroupCAN1_Low);
   CAN1_Bus->registerMotor(&DJIGroupCAN1_High);

   CAN1_Bus->registerMotor(&omni_wheel1);
   CAN1_Bus->registerMotor(&omni_wheel2);
   CAN1_Bus->registerMotor(&omni_wheel3);
//   CAN1_Bus->registerMotor(&omni_wheel4);

   CAN1_Bus->registerMotor(&arm_launchMotor);
   CAN1_Bus->registerMotor(&arm_stretchMotor);
   CAN1_Bus->registerMotor(&arm_rotateMotor);
   CAN1_Bus->registerMotor(&arm_pitchMotor);

   DJIGroupCAN2_Low.addMotor(&Weapon_launchMotor);
   DJIGroupCAN2_Low.addMotor(&Weapon_clawMotor);
   DJIGroupCAN2_Low.addMotor(&Weapon_traverseMotor);


   CAN2_Bus->registerMotor(&DJIGroupCAN2_Low);

   CAN2_Bus->registerMotor(&Weapon_launchMotor);
   CAN2_Bus->registerMotor(&Weapon_clawMotor);
   CAN2_Bus->registerMotor(&Weapon_traverseMotor);
   
   CAN2_Bus->registerMotor(&Weapon_wristMotor);
   
   

   CAN1_Bus->init();
   CAN2_Bus->init();
   
	CAN3_Bus->init();
   // 底盘轮子电机PID参数初始化
   omni_wheel1.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, m3508_angle_pid_params, 0.0f);
   omni_wheel2.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, m3508_angle_pid_params, 0.0f);
   omni_wheel3.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, m3508_angle_pid_params, 0.0f);
   // omni_wheel4.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);

   // 机械臂电机PID参数初始化
   
   PID_Param_Config arm_3508_speedPID = m3508_speed_pid_paramsForSpeedMotor;
   PID_Param_Config arm_3508_anglePID = m3508_angle_pid_params;
   arm_3508_anglePID.output_limit = 350.0f;
   // arm_3508_speedPID.output_limit = 420.0f; // 根据机械臂要求调整输出限幅
   
   arm_launchMotor.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, arm_3508_anglePID, 0.0f);

   PID_Param_Config arm_strech_anglePID = m2006_angle_pid_params;
   arm_strech_anglePID.output_limit = 400.0f;
   m2006_speed_pid_params.output_limit = 4500.0f;
   arm_stretchMotor.pid_init(m2006_speed_pid_params, 0.0f, arm_strech_anglePID, 0.0f);
   arm_rotateMotor.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, m3508Rotate_angle_pid_params, 0.0f);
   arm_pitchMotor.pid_init(m2006_speed_pid_params, 0.0f, m2006_angle_pid_params, 0.0f);
	
	PID_Param_Config weapon_3508_speedPID = m3508_speed_pid_paramsForSpeedMotor;
   PID_Param_Config weapon_3508_anglePID = m3508_angle_pid_params;
   
   PID_Param_Config weapon_2006_speedPID = m2006_speed_pid_params;
   PID_Param_Config weapon_2006_anglePID =m2006_angle_pid_params;
 
   weapon_3508_anglePID.output_limit=100.0f;
   weapon_3508_speedPID.output_limit=15000.0f;
   weapon_3508_anglePID.output_limit=100.0f;
   weapon_3508_speedPID.output_limit=12000.0f;
   weapon_2006_speedPID.output_limit=4500;
   weapon_2006_anglePID.output_limit=500;
   
   Weapon_launchMotor.pid_init(weapon_3508_speedPID, 0.0f,weapon_3508_anglePID, 0.0f);
   Weapon_clawMotor.pid_init(weapon_2006_speedPID, 0.0f,  weapon_2006_anglePID, 0.0f);
   Weapon_traverseMotor.pid_init(m2006_speed_pid_params, 0.0f, arm_strech_anglePID, 0.0f);

}


