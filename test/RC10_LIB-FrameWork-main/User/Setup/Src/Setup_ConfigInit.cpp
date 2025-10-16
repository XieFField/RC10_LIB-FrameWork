#include "Setup_ConfigInit.h"


#if DEBUG_M2006

DJI_MotorDemo dji_motor_demo;
#endif

fdCANbus* const CAN1_Bus = fdCANbus::getInstance(&hfdcan1);
/*================================ debug  ��е���� =============================*/

#if ARM_DEMO_DEBUG

//fdCANbus* const CAN1_Bus = fdCANbus::getInstance(&hfdcan1); // ��ȡFDCAN1��Ψһʵ��
DJI_Group ArmGroupCAN1_Low(send_idLow(), CAN1_Bus); // 1~4��M3508/M2006���

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

/*================================ debug  底盘Demo =============================*/

#if ONIM_DEMO_DEBUG

//fdCANbus* CAN1_Bus = fdCANbus::getInstance(&hfdcan1); // 获取FDCAN1的唯一实例
// 底盘电机定义

M3508 wheel_motor1(1, CAN1_Bus);
M3508 wheel_motor2(2, CAN1_Bus);
M3508 wheel_motor3(3, CAN1_Bus);
M3508 wheel_motor4(4, CAN1_Bus);

DJI_Group ChassisGroupCAN1_Low(send_idLow(), CAN1_Bus); // 底盘电机组
// namespace ChassisDem

// 底盘参数
const float wheel_radius = 0.076f;   // 轮子半径 (m)
const float max_wheel_rpm = 7000.0f; // 最大轮速 (RPM)
const float chassis_radius = 0.2f;   // 底盘半径 (m)

// 在全局作用域声明 onim_demo 对象
OnimDemo<4> onim_demo(wheel_radius, max_wheel_rpm, chassis_radius);

void chassis_motorInit()
{

   ChassisGroupCAN1_Low.addMotor(&wheel_motor1);
   ChassisGroupCAN1_Low.addMotor(&wheel_motor2);
   ChassisGroupCAN1_Low.addMotor(&wheel_motor3);
   ChassisGroupCAN1_Low.addMotor(&wheel_motor4);

   CAN1_Bus->registerMotor(&ChassisGroupCAN1_Low);
   
   CAN1_Bus->registerMotor(&wheel_motor1);
   CAN1_Bus->registerMotor(&wheel_motor2);   
   CAN1_Bus->registerMotor(&wheel_motor3);
   CAN1_Bus->registerMotor(&wheel_motor4);   

   wheel_motor1.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   wheel_motor2.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   wheel_motor3.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
   wheel_motor4.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);

   // 注册电机组到总线
   onim_demo.chassisInit(&wheel_motor1); 
   onim_demo.chassisInit(&wheel_motor2);
   onim_demo.chassisInit(&wheel_motor3);
   onim_demo.chassisInit(&wheel_motor4);

   CAN1_Bus->registerMotor(&ChassisGroupCAN1_Low); // 注册底盘对象到总线
   CAN1_Bus->init(); 
}


#endif

/*============================== debug  ��е���� ===============================*/





void debug_init()
{
   /*============================= debug  ��е���� ================================*/
#if ARM_DEMO_DEBUG
   arm_motorInit();
   arm_demo.armInit(&m3508_ArmLaunch, &m2006_ArmStretch, &m3508_ArmRotate, &m2006_ArmPitch);
#endif
/*============================== debug  ��е���� ===============================*/

 /*============================= debug  底盘Demo ================================*/
#if ONIM_DEMO_DEBUG
   chassis_motorInit();
   // 确保 wheel_motor 数组已定义
   // 创建指针数组
   // 调用函数
   //onim_demo.chassisInit(wheel_motor);

#endif


/*============================== debug  M2006 ===============================*/
#if DEBUG_M2006
   dji_motor_demo.init();
#endif
/*============================== debug  M2006 ===============================*/

}


void ALL_Setup_ConfigInit(void)
{

   

   TimeStamp::getInstance().init(&htim4); // ����ʱ�������
   debug_init();


   //other init
}

