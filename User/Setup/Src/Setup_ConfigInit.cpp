#include "Setup_ConfigInit.h"
#include "usb_device.h"
extern USBD_HandleTypeDef hUsbDeviceHS;
#if DEBUG_M2006

DJI_MotorDemo dji_motor_demo;
#endif

/*================================ debug  机械吸盘 =============================*/
extern class USB_USART m;
uint8_t rx_buffer[RX_BUFFER_SIZE];
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
//USB_USART m(UART_MODE, &huart1, rx_buffer, RX_BUFFER_SIZE, Position_UART1_RxCallback);
UART_USB_Struct uart_usb_ = {
	.RxCallback_Fuc = USB_DataReceivedCallback,
	.rx_buffer = rx_buffer,
	.rx_buffer_size = RX_BUFFER_SIZE,
};
UART_USB_Struct uart_usb_1 = {
	.RxCallback_Fuc = Position_UART1_RxCallback,
	.rx_buffer = rx_buffer,
	.rx_buffer_size = RX_BUFFER_SIZE,
};
//USB_USART(USB_UART_mode_E mode,UART_USB_Struct *usb_uart_struct,UART_HandleTypeDef *uart_handle,USBD_HandleTypeDef *usb_handle)

USB_USART m(USB_MODE, &uart_usb_, NULL, &hUsbDeviceHS);
USB_USART n(UART1_MODE,&uart_usb_1,&huart1,NULL);
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





void debug_init()
{
   /*============================= debug  机械吸盘 ================================*/
#if ARM_DEMO_DEBUG
   arm_motorInit();
   arm_demo.armInit(&m3508_ArmLaunch, &m2006_ArmStretch, &m3508_ArmRotate, &m2006_ArmPitch);
	 m.USB_UART_Init();
	 n.USB_UART_Init();
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

   

   TimeStamp::getInstance().init(&htim4); // 启用时间戳服务
   debug_init();

   //other init
}

