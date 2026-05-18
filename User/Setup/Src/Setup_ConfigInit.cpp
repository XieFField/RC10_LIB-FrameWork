#include "Setup_ConfigInit.h"

// 外部声明 USB 高速设备句柄
extern "C" 
{
    extern USBD_HandleTypeDef hUsbDeviceHS;
}
fdCANbus *const CAN1_Bus = fdCANbus::getInstance(&hfdcan1); // 获取 FDCAN1 唯一实例
fdCANbus *const CAN2_Bus = fdCANbus::getInstance(&hfdcan2); // 获取 FDCAN2 唯一实例
fdCANbus *const CAN3_Bus = fdCANbus::getInstance(&hfdcan3);

DJI_Group DJIGroupCAN1_Low(send_idLow(), CAN1_Bus);   // 1~4号 M3508/M2006 电机组
DJI_Group DJIGroupCAN1_High(send_idHigh(), CAN1_Bus); // 5~8号 M3508/M2006 电机组

DJI_Group DJIGroupCAN2_Low(send_idLow(), CAN2_Bus);   // 1~4号 M3508/M2006 电机组
DJI_Group DJIGroupCAN2_High(send_idHigh(), CAN2_Bus); // 5~8号 M3508/M2006 电机组

DJI_Group DJIGroupCAN3_High(send_idHigh(), CAN3_Bus); // 5~8号 M3508/M2006 电机组
DJI_Group DJIGroupCAN3_Low(send_idLow(), CAN3_Bus);   // 1~4号 M3508/M2006 电机组

Point2D arm_install_offset = {0.480f, 0.02f}; // 机械臂安装偏移，单位 m


/*==============Controller Instances===========*/
//USB_CDC_ cdc(&hUsbDeviceHS);
USB_CDC_ usb_1(&hUsbDeviceHS);
JY61_IMU IMU(JY61_ADDR,&hi2c5);
Chassis_Omni<3>::init_config chassis_initData = {
    .wheel_radius = 0.15f / 2.f,
    .max_wheel_rpm = 420,
    .wheels[0] = {
        .x = 0.0f,
        .y = 0.375f,
        .theta = 0.0f // 单位：度
    },
    .wheels[1] = {
        .x = -0.37f, .y = -0.375f,
        .theta = -63.741f + 180.0f // 单位：度
    },
    .wheels[2] = {
        .x = 0.37f, .y = -0.375f,
        .theta = 63.741f + 180.0f // 单位：度
    }};
OmniChassis_Setup ChassisOmni(chassis_initData); // 底盘半径、最大轮速、轮位安装参数
Chassis chassis;

FSM_Controller Finite_StateMachine;
ArmSetup ARM_Controller(arm_initData);
Robot_WeaponSage_Setup Weapon_Controller(initData_);



/*==============Controller Instances===========*/

/*=============================================*/

/*================Motor Instances==============*/


/**
 * CAN1: 底盘转向 M3508*4（1~4 id）+ VESC*4（101~104 id）
 * CAN2: 武器 M2006*3 + M3508*1（M2006:1~3, M3508:4）+ DM4310*1
 * CAN3: 机械臂 M3508*2 + M2006*2（id6:weapon_wrist, id8:arm_stretch）+ DM4310*1
 */
#define TEST_TEMP 0
                           /* 底盘电机 */
#if !TEST_TEMP
M3508 steer1(1, CAN1_Bus); M3508 steer2(2, CAN1_Bus); M3508 steer3(3, CAN1_Bus); M3508 steer4(4, CAN1_Bus);
VESC_Motor U8_1(101, CAN1_Bus, 21); VESC_Motor U8_2(102, CAN1_Bus, 21); 
VESC_Motor U8_3(103, CAN1_Bus, 21); VESC_Motor U8_4(104, CAN1_Bus, 21);
#endif

                            /* 武器系统电机 */
M2006 Weapon_Claw1(1, CAN2_Bus); M2006 Weapon_Claw2(2, CAN2_Bus); M2006 Weapon_Claw3(3, CAN2_Bus); 
M3508 Weapon_Launch(4, CAN2_Bus);
DM_Motor Weapon_Elbow(J4310_Type, 0x06, 0x06, CAN2_Bus); M2006 Weapon_Wrist(6, CAN3_Bus);

                            /* 机械臂电机 */
#if !TEST_TEMP
M3508 arm_launchMotor(5, CAN3_Bus, true, false); M3508 arm_rotateMotor(7, CAN3_Bus, true, false);
M2006 arm_stretchMotor(8, CAN3_Bus, true, false);  
DM_Motor arm_pitchMotor(J4310_Type, 0x06, 0x06, CAN3_Bus);
#else
M3508 arm_launchMotor(5, CAN1_Bus, true, false); M3508 arm_rotateMotor(7, CAN1_Bus, true, false);
M2006 arm_stretchMotor(8, CAN1_Bus, true, false);  
DM_Motor arm_pitchMotor(J4310_Type, 0x06, 0x06, CAN1_Bus);
#endif
OIDEncoder oid_encoder(91, CAN2_Bus, 4096, 200);

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


void debug_init()
{
/*============================= debug 机械臂 ================================*/
#if ARM_DEMO_DEBUG
    arm_motorInit();
    arm_demo.armInit(&m3508_ArmLaunch, &m2006_ArmStretch, &m3508_ArmRotate, &m2006_ArmPitch);
#endif

/*============================== debug  DJI_Motor ===============================*/
#if DEBUG_DJI_Motor
dji_motor_Init();
dji_motor_demo.init(&m2006_1);
#endif

/*============================== debug   speedplanner ===============================*/
#if SPEEDPLANNER_DEMO_DEBUG
    speedplanner_demo.init();
#endif

#if DEBUG
    laserpos.Init(); // 激光测距
#endif

    // system_detect_task_handle = osThreadNew(startSystemDetectTask, NULL, &system_detect_task_attributes);
}

void CAN_Motor_Init(void);
Point2D lader_install_offset = {0.0f, 0.0f}; // 激光雷达安装偏移，单位 m
Locate_Setup* set1 = Locate_Setup::getInstance();


void ALL_Setup_ConfigInit(void)
{

   HWT101CT* imu = HWT101CT::GetInstance(&huart1);
   imu->InitUART();
   TimeStamp::getInstance().init(&htim4); // ?????????????
	
   CAN_Motor_Init();

   ARM_Controller.init(&arm_launchMotor, &arm_stretchMotor, &arm_rotateMotor, &arm_pitchMotor);
   ARM_Controller.setArmStatus(ARM_IDLE);
   
   // Weapon_Controller.register_launch_Motor_1(&Weapon_launchMotor_1_master, &Weapon_launchMotor_1_slave);
   // Weapon_Controller.register_launch_Motor_2(&Weapon_launchMotor_2_master, &Weapon_launchMotor_2_slave);
   // Weapon_Controller.register_claw_Motor(&Weapon_clawMotor);
   // Weapon_Controller.register_traverse_Motor(&Weapon_traverseMotor);
   // Weapon_Controller.register_wrist_Motor(&Weapon_wristMotor);

   Weapon_Controller.init();
   Weapon_Controller.setWeaponSageControlStatus(WEAPONSAGE_CALIBRATE);

   // ChassisOmni.init();

    ChassisOmni.setChassisStatus(CHASSIS_STOP);

#if JIA_USE_THREE_OMNI_CHASSIS
    Chassis::InitConfig chassis_init_config =
    {
         .motor_handle[0] = &steer1,
         .motor_handle[1] = &steer2,
         .motor_handle[2] = &steer3
    };
    chassis.init(chassis_init_config);
#endif

#if JIA_USE_FOUR_STEER_CHASSIS && !TEST_TEMP
    Chassis::InitConfig chassis_init_config =
        {
            // 转向电机句柄（按轮序 0~3 对应）
            .steer_motor_h[0] = &steer1,
            .steer_motor_h[1] = &steer2,
            .steer_motor_h[2] = &steer3,
            .steer_motor_h[3] = &steer4,

            // 驱动电机句柄（按轮序 0~3 对应）
            .drive_motor_h[0] = &U8_1,
            .drive_motor_h[1] = &U8_2,
            .drive_motor_h[2] = &U8_3,
            .drive_motor_h[3] = &U8_4,
        };
    chassis.init(chassis_init_config);
#endif

   Finite_StateMachine.registerArmSetup(&ARM_Controller);
   Finite_StateMachine.registerChassisSetup(&ChassisOmni);
   Finite_StateMachine.registerWeaponSageSetup(&Weapon_Controller);

   Finite_StateMachine.init();

   oid_encoder.init();

   CrsfReceiver* crsf_rc = CrsfReceiver::GetInstance(&huart7);
   crsf_rc->init();

   set1->init(&usb_1,lader_install_offset ,arm_install_offset);
   set1->locate_setup_init();
   set1->set_startToLRL(true);
}

void CAN_Motor_Init(void)
{
   // CAN1 总线初始化：注册底盘电机
#if !TEST_TEMP
   DJIGroupCAN1_Low.addMotor(&steer1); DJIGroupCAN1_Low.addMotor(&steer2);
   DJIGroupCAN1_Low.addMotor(&steer3); DJIGroupCAN1_Low.addMotor(&steer4);
   CAN1_Bus->registerMotor(&DJIGroupCAN1_Low);
   CAN1_Bus->registerMotor(&steer1); CAN1_Bus->registerMotor(&steer2);
   CAN1_Bus->registerMotor(&steer3); CAN1_Bus->registerMotor(&steer4);
   CAN1_Bus->registerMotor(&U8_1); CAN1_Bus->registerMotor(&U8_2);
   CAN1_Bus->registerMotor(&U8_3); CAN1_Bus->registerMotor(&U8_4);
#else
    DJIGroupCAN1_High.addMotor(&arm_launchMotor); DJIGroupCAN1_High.addMotor(&arm_rotateMotor);
    DJIGroupCAN1_High.addMotor(&arm_stretchMotor); 

    CAN1_Bus->registerMotor(&DJIGroupCAN1_High);
    CAN1_Bus->registerMotor(&arm_launchMotor); CAN1_Bus->registerMotor(&arm_rotateMotor);
    CAN1_Bus->registerMotor(&arm_stretchMotor);
    CAN1_Bus->registerMotor(&arm_pitchMotor);

#endif

   CAN1_Bus->init();

   // CAN2 总线初始化：注册武器系统电机
   DJIGroupCAN2_Low.addMotor(&Weapon_Claw1); DJIGroupCAN2_Low.addMotor(&Weapon_Claw2);
   DJIGroupCAN2_Low.addMotor(&Weapon_Claw3); DJIGroupCAN2_Low.addMotor(&Weapon_Launch);
   CAN2_Bus->registerMotor(&DJIGroupCAN2_Low);
   CAN2_Bus->registerMotor(&Weapon_Claw1); CAN2_Bus->registerMotor(&Weapon_Claw2);
   CAN2_Bus->registerMotor(&Weapon_Claw3); CAN2_Bus->registerMotor(&Weapon_Launch);

   CAN2_Bus->registerMotor(&Weapon_Elbow); 

   CAN2_Bus->registerOIDEncoder(&oid_encoder); // ??????????????

   CAN2_Bus->init();

#if !TEST_TEMP
   // CAN3 总线初始化：注册机械臂电机
   DJIGroupCAN3_High.addMotor(&arm_launchMotor); DJIGroupCAN3_High.addMotor(&arm_rotateMotor);
   DJIGroupCAN3_High.addMotor(&arm_stretchMotor); 

   DJIGroupCAN3_High.addMotor(&Weapon_Wrist);

   CAN3_Bus->registerMotor(&DJIGroupCAN3_High);
   CAN3_Bus->registerMotor(&arm_launchMotor); CAN3_Bus->registerMotor(&arm_rotateMotor);
   CAN3_Bus->registerMotor(&arm_stretchMotor); 
   
   CAN3_Bus->registerMotor(&Weapon_Wrist);
   CAN3_Bus->registerMotor(&arm_pitchMotor);

   

	CAN3_Bus->init();

   // 仅四舵轮舵向电机采用 8:1 减速比，其他 M3508（机械臂/发射等）保持原配置。
   steer1.reset_GearRatio(8.0f);
   steer2.reset_GearRatio(8.0f);
   steer3.reset_GearRatio(8.0f);
   steer4.reset_GearRatio(8.0f);

   // 底盘转向电机 PID 参数初始化
   steer1.pid_init(foursteer_steer_speed_pid_params, 0.0f, foursteer_steer_angle_pid_params, 0.0f);
   steer2.pid_init(foursteer_steer_speed_pid_params, 0.0f, foursteer_steer_angle_pid_params, 0.0f);
   steer3.pid_init(foursteer_steer_speed_pid_params, 0.0f, foursteer_steer_angle_pid_params, 0.0f);
   steer4.pid_init(foursteer_steer_speed_pid_params, 0.0f, foursteer_steer_angle_pid_params, 0.0f);

    steer1.set_anglepid_circular(true);
    steer2.set_anglepid_circular(true);
    steer3.set_anglepid_circular(true);
    steer4.set_anglepid_circular(true);

   U8_1.reset_controlFrequency(500);  U8_2.reset_controlFrequency(500);
   U8_3.reset_controlFrequency(500);  U8_4.reset_controlFrequency(500);
#endif

   // 机械臂电机 PID 参数初始化
   
   PID_Param_Config arm_3508_anglePID = m3508_angle_pid_params;
   arm_3508_anglePID.output_limit = 450.0f;
   arm_launchMotor.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, arm_3508_anglePID, 0.0f);
   PID_Param_Config arm_strech_anglePID = m2006_angle_pid_params;
   arm_strech_anglePID.output_limit = 500.0f;
   m2006_speed_pid_params.output_limit = 8000.0f;
   arm_stretchMotor.pid_init(m2006_speed_pid_params, 0.0f, arm_strech_anglePID, 0.0f);
   arm_rotateMotor.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, m3508Rotate_angle_pid_params, 0.0f);
   arm_pitchMotor.reset_controlFrequency(100); // 俯仰电机降到 100Hz，减轻总线负载


// ======== 武器系统 PID 参数初始化 ========


	// PID_Param_Config weapon_3508_speedPID = m3508_speed_pid_paramsForSpeedMotor;
   // PID_Param_Config weapon_3508_anglePID = m3508_angle_pid_params;
   
   // PID_Param_Config weapon_2006_speedPID = m2006_speed_pid_params;
   // PID_Param_Config weapon_2006_anglePID =m2006_angle_pid_params;

   // weapon_3508_anglePID.output_limit=200.0f;
   // weapon_3508_speedPID.output_limit=15000.0f;
   // weapon_2006_speedPID.output_limit=4500;
   // weapon_2006_anglePID.output_limit=500;
   
   // Weapon_launchMotor_1_master.pid_init(weapon_3508_speedPID, 0.0f,weapon_3508_anglePID, 0.0f);
   // Weapon_launchMotor_2_master.pid_init(weapon_3508_speedPID, 0.0f, weapon_3508_anglePID, 0.0f);
   
   // Weapon_clawMotor.pid_init(weapon_2006_speedPID, 0.0f,  weapon_2006_anglePID, 0.0f);
   // Weapon_traverseMotor.pid_init(m2006_speed_pid_params, 0.0f, arm_strech_anglePID, 0.0f);

   PID_Param_Config weapon_launch_anglePID = m3508_angle_pid_params;
   PID_Param_Config weapon_launch_speedPID = m3508_speed_pid_paramsForSpeedMotor;
   weapon_launch_anglePID.output_limit = 200.0f;
   weapon_launch_speedPID.output_limit = 15000.0f;
   Weapon_Launch.pid_init(weapon_launch_speedPID, 0.0f, weapon_launch_anglePID, 0.0f);

   PID_Param_Config weapon_claw_anglePID = m2006_angle_pid_params;
   PID_Param_Config weapon_claw_speedPID = m2006_speed_pid_params;
   weapon_claw_anglePID.output_limit = 500.0f;
   weapon_claw_speedPID.output_limit = 4500.0f;
   Weapon_Claw1.pid_init(weapon_claw_speedPID, 0.0f, weapon_claw_anglePID, 0.0f);
   Weapon_Claw2.pid_init(weapon_claw_speedPID, 0.0f, weapon_claw_anglePID, 0.0f);
   Weapon_Claw3.pid_init(weapon_claw_speedPID, 0.0f, weapon_claw_anglePID, 0.0f);

   Weapon_Wrist.reset_controlFrequency(100);
}
