#include "Setup_ConfigInit.h"

/**
 * @brief  机械臂和底盘的配置初始化
 *   2代r1电机分配 can1:M3508*2 + M3508*4 留有1个DJI电机余量
 *                 can2:M3508*4 + M2006*2 留有1个DJI电机余量
 *                 can3: vesc*4 DM4310*2  留有1个DJI电机余量
 */ 

 // 外部声明USB高速设备句柄
extern "C" 
{
    extern USBD_HandleTypeDef hUsbDeviceHS;
}
fdCANbus *const CAN1_Bus = fdCANbus::getInstance(&hfdcan1); // 获取FDCAN1的唯一实例
fdCANbus *const CAN2_Bus = fdCANbus::getInstance(&hfdcan2); // 获取FDCAN2的唯一实例
fdCANbus *const CAN3_Bus = fdCANbus::getInstance(&hfdcan3);

DJI_Group DJIGroupCAN1_Low(send_idLow(), CAN1_Bus);   // 1~4号M3508/M2006电机
DJI_Group DJIGroupCAN1_High(send_idHigh(), CAN1_Bus); // 5~8号M3508/M2006电机

DJI_Group DJIGroupCAN2_Low(send_idLow(), CAN2_Bus); // 1~4号M3508/M2006电机
DJI_Group DJIGroupCAN2_High(send_idHigh(), CAN2_Bus); // 5~8号M3508/M2006电机

DJI_Group DJIGroupCAN3_High(send_idHigh(), CAN3_Bus); // 5~8号M3508/M2006电机

Point2D arm_install_offset = {0.480f, 0.02f};   // 机械臂安装偏移，单位米


/*==============Controller Instances===========*/
uint8_t laser_rx_buffer[20];
uint8_t laser_rx_buffer1[20];
uint8_t laser_rx_buffer2[20];
// 激光测距
// USB_CDC_ cdc(&hUsbDeviceHS);
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
OmniChassis_Setup ChassisOmni(chassis_initData); // 轮子半径，最大轮子转速，底盘 底 腰
Chassis chassis;

FSM_Controller Finite_StateMachine;
ArmSetup ARM_Controller(arm_initData);
Robot_WeaponSage_Setup Weapon_Controller(initData_);



/*==============Controller Instances===========*/

/*=============================================*/

/*================Motor Instances==============*/


/**
 * @brief  机械臂和底盘的配置初始化
 *   2代r1电机分配 can1:M3508*2(机械臂云台和升降) + M3508*4(舵向) 留有1个DJI电机余量
 *                 can2:M3508*4(龙门架) + M2006*2(夹杆) 留有1个DJI电机余量
 *                 can3: vesc*4 DM4310*2 M2006*1(机械臂伸缩) 留有1个DJI电机余量
 */ 


                           /* 底盘电机 */
M3508 rudder1(1, CAN1_Bus, true, false); M3508 rudder2(2, CAN1_Bus, true, false); 
M3508 rudder3(3, CAN1_Bus, true, false); M3508 rudder4(4, CAN1_Bus, true, false);

VESC_Motor motor_vesc1(101, CAN3_Bus, 21.0f); VESC_Motor motor_vesc2(102, CAN3_Bus, 21.0f); 
VESC_Motor motor_vesc3(103, CAN3_Bus, 21.0f); VESC_Motor motor_vesc4(104, CAN3_Bus, 21.0f);

//                            /* 串联臂 */      
// M3508 arm_launchMotor(5, CAN1_Bus, true, false); M3508 arm_rotateMotor(7, CAN1_Bus, true, false);

// M2006 arm_stretchMotor(8, CAN3_Bus, true, false);  
// DM_Motor arm_pitchMotor(J4310_Type, 0x06, 0x06, CAN3_Bus);

//                            /* 武器系统 */
M3508 Weapon_launchMotor_1_master(1, CAN2_Bus, true, false); M3508 Weapon_launchMotor_1_slave(2, CAN2_Bus, true, false); 
M3508 Weapon_launchMotor_2_master(3, CAN2_Bus, true, false); M3508 Weapon_launchMotor_2_slave(4, CAN2_Bus, true, false);
M2006 Weapon_traverseMotor(5, CAN2_Bus, true, false); M2006 Weapon_clawMotor(6, CAN2_Bus, true, false);

DM_Motor Weapon_wristMotor(J4310_Type, 0x05,0x05, CAN3_Bus);


M3508 arm_launchMotor(5, CAN1_Bus, true, false); M3508 arm_rotateMotor(7, CAN1_Bus, true, false);

M2006 arm_stretchMotor(8, CAN1_Bus, true, false);  
DM_Motor arm_pitchMotor(J4310_Type, 0x06, 0x06, CAN1_Bus);

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

/*================================ debug  机械臂 =============================*/

#if ARM_DEMO_DEBUG

M3508 m3508_ArmLaunch(1, CAN1_Bus);
M2006 m2006_ArmStretch(2, CAN1_Bus);
M3508 m3508_ArmRotate(3, CAN1_Bus);
M2006 m2006_ArmPitch(4, CAN1_Bus);

Arm_InitData_S arm_demoInit_data = {
    .max_launchHeight_ = 0.8f,
    .max_stretchLength_ = 0.130f,
    .arm_length_ = 0.3f,

    .stretch_Ratio_ = 0.03098f,
    .launch_Ratio_ = 0.1f,
    .rotate_gearRatio_ = 10.0f,
    .pitch_gearRatio_ = 10.0f,
};

Robot_ArmDemo arm_demo(arm_demoInit_data);
// 激光测距
LaserPosition laserpos(&huart3, &huart6);

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

void debug_init()
{
/*============================= debug  机械臂 ================================*/
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
Point2D lader_install_offset = {0.0f, 0.0f}; // 激光雷达安装偏移，单位米
Locate_Setup* set1 = Locate_Setup::getInstance();
Laser_InstanceManager instance_man;
void ALL_Setup_ConfigInit(void)
{
    // 初始化串口6的相机模块
    Module_Camera::GetInstance(&huart6)->InitUART();

    HWT101CT *imu = HWT101CT::GetInstance(&huart1);
    imu->InitUART();
    TimeStamp::getInstance().init(&htim4); // 启用时间戳服务
    // debug_init();

    CAN_Motor_Init();

   ARM_Controller.init(&arm_launchMotor, &arm_stretchMotor, &arm_rotateMotor, &arm_pitchMotor);
   ARM_Controller.setArmStatus(ARM_IDLE);
   
   Weapon_Controller.register_launch_Motor_1(&Weapon_launchMotor_1_master, &Weapon_launchMotor_1_slave);
   Weapon_Controller.register_launch_Motor_2(&Weapon_launchMotor_2_master, &Weapon_launchMotor_2_slave);
   Weapon_Controller.register_claw_Motor(&Weapon_clawMotor);
   Weapon_Controller.register_traverse_Motor(&Weapon_traverseMotor);
   Weapon_Controller.register_wrist_Motor(&Weapon_wristMotor);

   Weapon_Controller.init();
   Weapon_Controller.setWeaponSageControlStatus(WEAPONSAGE_CALIBRATE);

//    ChassisOmni.registerWheelMotor(0, &rudder11);
//    ChassisOmni.registerWheelMotor(1, &rudder2);
//    ChassisOmni.registerWheelMotor(2, &rudder3);
//    ChassisOmni.registerWheelMotor(3, &rudder4);
   ChassisOmni.init();

    ChassisOmni.setChassisStatus(CHASSIS_STOP);

#if JIA_USE_THREE_OMNI_CHASSIS
    Chassis::InitConfig chassis_init_config =
    {
         .motor_handle[0] = &rudder1,
         .motor_handle[1] = &rudder2,
         .motor_handle[2] = &rudder3
    };
    chassis.init(chassis_init_config);
#endif

#if JIA_USE_FOUR_STEER_CHASSIS
    // 四舵轮底盘初始化配置：
    // 1) 先绑定 4 个转向电机 + 4 个驱动电机
    // 2) 再设置整车级运动学/动力学限幅参数
    // 3) 最后配置每个轮组的安装几何、方向符号与回零参数
    Chassis::InitConfig chassis_init_config =
        {
            // 转向电机句柄（按轮序 0~3 对应）
            .steer_motor_h[0] = &rudder1,
            .steer_motor_h[1] = &rudder2,
            .steer_motor_h[2] = &rudder3,
            .steer_motor_h[3] = &rudder4,

            // 驱动电机句柄（按轮序 0~3 对应）
            .drive_motor_h[0] = &motor_vesc1,
            .drive_motor_h[1] = &motor_vesc2,
            .drive_motor_h[2] = &motor_vesc3,
            .drive_motor_h[3] = &motor_vesc4,

            // 整车参数与限幅（用于主控制线程速度规划和模块命令限幅）
            .wheel_radius_m = 0.075f,
            .max_vel_x_m_s = 4.0f,
            .max_vel_y_m_s = 4.0f,
            .max_omega_z_rad_s = 8.0f,
            .max_acc_xy_acc_m_s2 = 4.0f,
            .max_acc_xy_dec_m_s2 = 8.0f,
            .max_alpha_z_acc_rad_s2 = 6.0f,
            .max_alpha_z_dec_rad_s2 = 10.0f,
            .max_drive_omega_rad_s = jia::rpmToRadsF32(35.0f),
            .max_drive_alpha_rad_s2 = 90.0f,
            .max_steer_rate_rad_s = 7.0f,
            .max_steer_alpha_rad_s2 = 40.0f,
            .stationary_speed_epsilon_m_s = 0.01f,
            .enable_cosine_compensation = true,
            .idle_posture_mode = Chassis::IdlePostureMode::kHoldLast,

            // 轮序按实车定义：
            // 1号=左后(rudder1/motor_vesc1)、2号=右后、3号=右前、4号=左前。
            // 车体坐标系采用 x前 y左（右手系），整车尺寸：x向780mm、y向800mm。
            // 所以四轮相对底盘中心坐标分别为：x=±0.39m，y=±0.40m。
            // wheels[0]：1号左后轮（x<0, y>0）
            .wheels[0] = {
                .pos_x_m = -0.39f,
                .pos_y_m = 0.40f,
                .theta_oa_to_owi_deg = -90.0f,    // 1号轮机械安装朝向“向右”
                .steer_motor_sign = 1.0f,         // 转向方向符号：1 不取反，-1 取反
                .drive_motor_sign = 1.0f,         // 驱动方向符号：1 不取反，-1 取反
                .homing_enabled = true,           // 实车接入光电门后启用回零
                .homing_sensor_active_high = true,
                .homing_gpio_port = kPHOTOGATE_1_GPIO_Port,
                .homing_gpio_pin = kPHOTOGATE_1_Pin,
                .homing_falling_edge_mech_deg = 60.0f,  // 原始 GPIO 高->低边沿对应机械 +60°
                .homing_rising_edge_mech_deg = -120.0f, // 原始 GPIO 低->高边沿对应机械 -120°
                .homing_search_rpm = 10.0f,       // 回零搜索阶段转向电机转速（rpm）
                .homing_zero_offset_deg = 0.0f,   // 逻辑零点统一指向车头前方；细调偏差后续再回填
                .homing_timeout_s = 5.0f,         // 单轮回零超时时间（s）
            },
            // wheels[1]：2号右后轮（x<0, y<0）
            .wheels[1] = {
                .pos_x_m = -0.39f,
                .pos_y_m = -0.40f,
                .theta_oa_to_owi_deg = 0.0f,      // 2号轮机械安装朝向“向前”
                .steer_motor_sign = 1.0f,
                .drive_motor_sign = 1.0f,
                .homing_enabled = true,
                .homing_sensor_active_high = true,
                .homing_gpio_port = kPHOTOGATE_2_GPIO_Port,
                .homing_gpio_pin = kPHOTOGATE_2_Pin,
                .homing_falling_edge_mech_deg = 60.0f,
                .homing_rising_edge_mech_deg = -120.0f,
                .homing_search_rpm = 10.0f,
                .homing_zero_offset_deg = 0.0f,
                .homing_timeout_s = 5.0f,
            },
            // wheels[2]：3号右前轮（x>0, y<0）
            .wheels[2] = {
                .pos_x_m = 0.39f,
                .pos_y_m = -0.40f,
                .theta_oa_to_owi_deg = 90.0f,     // 3号轮机械安装朝向“向左”
                .steer_motor_sign = 1.0f,
                .drive_motor_sign = 1.0f,
                .homing_enabled = true,
                .homing_sensor_active_high = true,
                .homing_gpio_port = kPHOTOGATE_3_GPIO_Port,
                .homing_gpio_pin = kPHOTOGATE_3_Pin,
                .homing_falling_edge_mech_deg = 60.0f,
                .homing_rising_edge_mech_deg = -120.0f,
                .homing_search_rpm = 10.0f,
                .homing_zero_offset_deg = 0.0f,
                .homing_timeout_s = 5.0f,
            },
            // wheels[3]：4号左前轮（x>0, y>0）
            .wheels[3] = {
                .pos_x_m = 0.39f,
                .pos_y_m = 0.40f,
                .theta_oa_to_owi_deg = 180.0f,    // 4号轮机械安装朝向“向后”
                .steer_motor_sign = 1.0f,
                .drive_motor_sign = 1.0f,
                .homing_enabled = true,
                .homing_sensor_active_high = true,
                .homing_gpio_port = kPHOTOGATE_4_GPIO_Port,
                .homing_gpio_pin = kPHOTOGATE_4_Pin,
                .homing_falling_edge_mech_deg = 60.0f,
                .homing_rising_edge_mech_deg = -120.0f,
                .homing_search_rpm = 10.0f,
                .homing_zero_offset_deg = 0.0f,
                .homing_timeout_s = 5.0f,
            },
        };
    // 将上述配置写入四舵轮 chassis 运行态（仅初始化数据，不改变 FSM 绑定对象）
    chassis.init(chassis_init_config);
#endif

    Finite_StateMachine.registerArmSetup(&ARM_Controller);
    Finite_StateMachine.registerChassisSetup(&ChassisOmni);
    Finite_StateMachine.registerWeaponSageSetup(&Weapon_Controller);

    Finite_StateMachine.init();


   CrsfReceiver* crsf_rc = CrsfReceiver::GetInstance(&huart7);
   crsf_rc->init();
	set1->init(&instance_man,&usb_1,lader_install_offset ,arm_install_offset);
   set1->laser_initData_.d=0.5;
   set1->locate_setup_init();
   set1->set_startToLRL(true);
   
}

void CAN_Motor_Init(void)
{
   DJIGroupCAN1_Low.addMotor(&rudder1); 
   DJIGroupCAN1_Low.addMotor(&rudder2);
   DJIGroupCAN1_Low.addMotor(&rudder3);
   DJIGroupCAN1_Low.addMotor(&rudder4);

   DJIGroupCAN1_High.addMotor(&arm_launchMotor);
   DJIGroupCAN1_High.addMotor(&arm_rotateMotor);
   DJIGroupCAN1_High.addMotor(&arm_stretchMotor);
   
   CAN1_Bus->registerMotor(&DJIGroupCAN1_Low);
   CAN1_Bus->registerMotor(&DJIGroupCAN1_High);
   CAN1_Bus->registerMotor(&arm_pitchMotor);
   CAN1_Bus->registerMotor(&rudder1);
   CAN1_Bus->registerMotor(&rudder2);
   CAN1_Bus->registerMotor(&rudder3);
   CAN1_Bus->registerMotor(&rudder4);

   CAN1_Bus->registerMotor(&arm_launchMotor);
   CAN1_Bus->registerMotor(&arm_rotateMotor);
   CAN1_Bus->registerMotor(&arm_stretchMotor);

   CAN1_Bus->init();
   CAN2_Bus->init();
	CAN3_Bus->init();

   // 底盘轮子电机PID参数初始化
   rudder1.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, m3508_angle_pid_params, 0.0f);
   rudder2.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, m3508_angle_pid_params, 0.0f);
   rudder3.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, m3508_angle_pid_params, 0.0f);
   rudder4.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);

   // 机械臂电机PID参数初始化
   
   PID_Param_Config arm_3508_anglePID = m3508_angle_pid_params;
   arm_3508_anglePID.output_limit = 450.0f;
   
   arm_launchMotor.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, arm_3508_anglePID, 0.0f);

   PID_Param_Config arm_strech_anglePID = m2006_angle_pid_params;
   arm_strech_anglePID.output_limit = 400.0f;
   m2006_speed_pid_params.output_limit = 4500.0f;
   arm_stretchMotor.pid_init(m2006_speed_pid_params, 0.0f, arm_strech_anglePID, 0.0f);
   arm_rotateMotor.pid_init(m3508_speed_pid_paramsForSpeedMotor, 0.0f, m3508Rotate_angle_pid_params, 0.0f);
   arm_pitchMotor.reset_controlFrequency(100); // 俯仰电机降低控制频率到100Hz，减轻总线负担


	PID_Param_Config weapon_3508_speedPID = m3508_speed_pid_paramsForSpeedMotor;
   PID_Param_Config weapon_3508_anglePID = m3508_angle_pid_params;
   
   PID_Param_Config weapon_2006_speedPID = m2006_speed_pid_params;
   PID_Param_Config weapon_2006_anglePID =m2006_angle_pid_params;

   weapon_3508_anglePID.output_limit=200.0f;
   weapon_3508_speedPID.output_limit=15000.0f;
   weapon_2006_speedPID.output_limit=4500;
   weapon_2006_anglePID.output_limit=500;
   
   Weapon_launchMotor_1_master.pid_init(weapon_3508_speedPID, 0.0f,weapon_3508_anglePID, 0.0f);
   Weapon_launchMotor_2_master.pid_init(weapon_3508_speedPID, 0.0f, weapon_3508_anglePID, 0.0f);
   
   Weapon_clawMotor.pid_init(weapon_2006_speedPID, 0.0f,  weapon_2006_anglePID, 0.0f);
   Weapon_traverseMotor.pid_init(m2006_speed_pid_params, 0.0f, arm_strech_anglePID, 0.0f);
   Weapon_wristMotor.reset_controlFrequency(100);
}
