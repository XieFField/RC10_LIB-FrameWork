


#include "Motor_Go_demo.h"

extern fdCANbus CAN1_Bus; // CAN1

GO_Motor GO_Motor_1(0, &CAN1_Bus);


extern volatile uint8_t start_signal;
extern volatile float delta_time; //目前使用的单位是微秒
extern volatile uint64_t last_time;




volatile float GO_demo_Torque = 0.0f;
volatile float GO_demo_RPM = 0.0f;
volatile float GO_demo_Angle = 0.0f;


PID_Param_Config Go_speed_pid_params = {
    .kp = 10.0f,
    .ki = 1.0f,
    .kd = 0.01f,
    .I_Outlimit = 0.05f, 
    .isIOutlimit = true, 
    .output_limit = 0.2f,   
    .deadband = 1.0f 
};

PID_Param_Config Go_angle_pid_params = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .I_Outlimit = 0.0f, 
    .isIOutlimit = true, 
    .output_limit = 0.0f,   
    .deadband = 0.0f 
};


void GO_MotorDemo::init()
{
    CAN1_Bus.registerMotor(&GO_Motor_1); // 注册电机本身
    start(osPriorityNormal, 256);

    // 初始化PID参数
    GO_Motor_1.pid_init(Go_speed_pid_params, 0.0f, Go_angle_pid_params, 0.0f);
    
    const char *msg2 = "Hallo GO_MotorDemo!\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)msg2, strlen(msg2), HAL_MAX_DELAY);
}


void GO_MotorDemo::loop()
{
    uint64_t time_now = TimeStamp::getInstance().getMicroseconds();
    if(last_time > 0)
    {
        delta_time = static_cast<float>(time_now - last_time); 
        // 可以在这里使用 delta_time 进行其他计算
    }
    last_time = time_now;
    debug_uart.printf_DMA("%f,%f\r\n",GO_Motor_1.getRPM(), GO_Motor_1.getTargetRPM());
    // HAL_UART_Transmit(&huart1, (uint8_t*)"Tick\r\n", 6, HAL_MAX_DELAY);
    if(start_signal == 1)
    {
        GO_Motor_1.setTargetTorque(GO_demo_Torque);
    }
    else if(start_signal == 0)
    {

    }
    else if(start_signal == 2)
    {
        GO_Motor_1.setTargetRPM(GO_demo_RPM);
    }
    else if (start_signal == 3)
    {
        GO_Motor_1.setTargetAngle(GO_demo_Angle);
       
    }
    else if (start_signal == 4)
    {
        GO_Motor_1.resetTotalAngle();
    }
    else if (start_signal == 5)
    {

       static uint64_t last_switch_time = 0;
       uint64_t current_time = HAL_GetTick();
       static bool temp = true;
       
       if (current_time - last_switch_time >= 10000) {
           last_switch_time = current_time;
           if(temp)
           {
               temp = false;
               GO_Motor_1.setTargetRPM(250);
           }
           else
           {
               temp = true;
               GO_Motor_1.setTargetRPM(-250);
           }
       }
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
    else if(start_signal == 9)
    {
       
    }
    else
    {
        
    }
}