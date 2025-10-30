#include "frame_demo.h"




GPIODevice elcdoor(GPIOG,GPIO_PIN_8);

//PID_Param_Config m3508_speed_pid_params = {
//    .kp = 32.0f,
//    .ki = 0.0f,
//    .kd = 0.0f,
//    .I_Outlimit = 0.0f, 
//    .isIOutlimit = true, 
//    .output_limit = 20000.0f,   
//    .deadband = 5.0f 
//};

//PID_Param_Config m3508_angle_pid_params = {
//    .kp = 100.0f,
//    .ki = 0.0f,
//    .kd = 0.005f,
//    .I_Outlimit = 8000.0f, 
//    .isIOutlimit = true, 
//    .output_limit = 20000.0f,   
//    .deadband = 5.0f // 
//};


// 使用 volatile 防止编译器优化，确保在调试时可以观察到值的变化
volatile int counter = 0;
volatile uint8_t start_signal = 0;
volatile float test_rpm=0.0f;
volatile float test_pos=0.0f;
volatile float test_kff =0;
volatile float test_kp =0;
volatile float test_kd =0;
void FrameDemo::loop()
{

}

void FrameDemo::init()
{
    start(osPriorityNormal, 256);
}

volatile float delta_time = 0.0f; //目前使用的单位是微秒
volatile uint64_t last_time = 0;


volatile int16_t left_x = 0;

void DJI_MotorDemo::loop()
{
   uint64_t time_now = TimeStamp::getInstance().getMicroseconds();
   if(last_time > 0)
   {
       delta_time = static_cast<float>(time_now - last_time); 
       // 可以在这里使用 delta_time 进行其他计算
   }
   last_time = time_now;
   debug_uart.printf_DMA("%d,%f,%f\r\n",left_x,motor_->getTargetRPM(), motor_->getRPM());
   
   left_x = (int16_t)AirJoy::getinstance().LEFT_X;
   if(left_x < 950 || left_x > 2050)
   {
	    left_x = 0;
   }
   else 
   {
        left_x -= 1500;
   }

   if(abs(left_x) < 50)
   {
	   left_x = 0;
   }
   
   if(start_signal == 1)
   {
       motor_->setTargetRPM(left_x);
   }
   else if(start_signal == 0)
   {

   }
   else if(start_signal == 2)
   {
       motor_->setTargetRPM(test_rpm);
   }
   else if (start_signal == 3)
   {
       motor_->setTargetAngle(test_pos);
   }
   else if (start_signal == 4)
   {

   }
   else if (start_signal == 5)
   {

   }
   else if (start_signal == 6)
   {

   }
   else if (start_signal == 7)
   {

   }
   else if (start_signal == 8)
   {

   }
   else
   {
       motor_->setTargetCurrent(0.0f);
   }
}


//void DM_MotorDemo::loop()
//{
//	 uint64_t time_now = TimeStamp::getInstance().getMicroseconds();
//    if(last_time > 0)
//    {
//        delta_time = static_cast<float>(time_now - last_time); 
//        // 可以在这里使用 delta_time 进行其他计算
//    }
//    last_time = time_now;
////    debug_uart.printf_DMA("%f,%f\r\n",m3508_1.getRPM(), m3508_1.getTargetRPM());
//    //HAL_UART_Transmit(&huart1, (uint8_t*)"Tick\r\n", 6, HAL_MAX_DELAY);
//    if(start_signal == 1)
//    {
////		dm_motor.setMIT(test_pos,test_rpm,test_kp,test_kd,test_kff);
////		dm_motor.setTargetRPM(test_rpm);
//		dm_motor.setTargetTotalAngle(test_rpm,test_pos);
//    }
//    else if(start_signal == 0)
//    {
//		dm_motor.motorEnable();
////		dm_motor.motorSetZero();
////		start_signal = 1;
//    }
//    else if(start_signal == 2)
//    {
//		dm_motor.motorDisable();
//    }
//    else if (start_signal == 3)
//    {
//		dm_motor.motorSetZero();
//        /* code */

//    }
//    else if (start_signal == 4)
//    {
//        /* code */

//    }
//    else if (start_signal == 5)
//    {
//        /* code */
//    }
//    else if (start_signal == 6)
//    {
//        /* code */
//        
//    }
//    else if (start_signal == 7)
//    {
//        /* code */
//    }
//    else if (start_signal == 8)
//    {
//        /* code */
//        
//    }
//    else
//    {
//        
//    }
//}
bool state=0;
void GPIODemo::loop()
{
	uint64_t time_now = TimeStamp::getInstance().getMicroseconds();
    if(last_time > 0)
    {
        delta_time = static_cast<float>(time_now - last_time); 
        // 可以在这里使用 delta_time 进行其他计算
    }
    last_time = time_now;

    if(start_signal == 1)
    {
		elcdoor.Reset_pin();
    }
    else if(start_signal == 0)
    {
		elcdoor.Set_pin();
    }
    else if(start_signal == 2)
    {
		elcdoor.Toggle_pin();
    }
    else if (start_signal == 3)
    {
		state=elcdoor.Read_pin();
        /* code */

    }
    else if (start_signal == 4)
    {
        /* code */

    }
    else if (start_signal == 5)
    {
        /* code */
    }
    else if (start_signal == 6)
    {
        /* code */
        
    }
    else if (start_signal == 7)
    {
        /* code */
    }
    else if (start_signal == 8)
    {
        /* code */
        
    }
    else
    {
        
    }
}

void GPIODemo::init()
{
	start(osPriorityNormal, 256);
}

void DJI_MotorDemo::init(DJI_Motor *motor)
{
   motor_ = motor;

   start(osPriorityNormal, 256);

   debug_uart.printf_DMA("DJI_MotorDemo init\r\n");
}

//void DM_MotorDemo::init()
//{
//	CAN2_Bus.registerMotor(&dm_motor); // 注册电机本身
////    CAN1_Bus.registerMotor(&DJI_Group_1); // 同时注册Group用于发送
////    m3508_1.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
//    CAN2_Bus.init();
//    start(osPriorityNormal, 256);
//   
//    const char *msg = "Hello UART1 on PB6/PB7\r\n";
//    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//}


