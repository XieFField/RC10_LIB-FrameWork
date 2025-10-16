#include "frame_demo.h"

fdCANbus* const CAN1_Bus = fdCANbus::getInstance(&hfdcan1); // 获取FDCAN1的唯一实例
DJI_Group GroupCAN1_Low(send_idHigh(), CAN1_Bus); // 1~4号M3508/M2006电机
M3508 m3508_1(7, CAN1_Bus);
//目前不错的参数 by XieFField
PID_Param_Config m3508_speed_pid_params = {
    .kp = 32.0f,
    .ki = 0.085f,
    .kd = 0.0f,
    .I_Outlimit = 8000.0f, 
    .isIOutlimit = true, 
    .output_limit = 15000.0f,   
    .deadband = 0.5f 
};

PID_Param_Config m3508_angle_pid_params = {
    .kp = 32.0f,
    .ki = 0.0f,
    .kd = 1.1f,
    .I_Outlimit = 0.0f, 
    .isIOutlimit = true, 
    .output_limit = 400.0f,   
    .deadband = 0.5f // 
};


// 使用 volatile 防止编译器优化，确保在调试时可以观察到值的变化
volatile int counter = 0;
volatile uint8_t start_signal = 0;

volatile int a = 0;
volatile float delta_time = 0.0f; //目前使用的单位是微秒
volatile uint64_t last_time = 0;


void DJI_MotorDemo::init()
{
    GroupCAN1_Low.addMotor(&m3508_1);
    CAN1_Bus->registerMotor(&GroupCAN1_Low);
    CAN1_Bus->registerMotor(&m3508_1);
    CAN1_Bus->init();
    m3508_1.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
    start_signal = 0;
    start(osPriorityNormal, 256);
}
int cnt = 0;
void DJI_MotorDemo::loop()
{
    // 任务循环
    uint64_t time_now = TimeStamp::getInstance().getMicroseconds();
    if(last_time > 0)
    {
        delta_time = static_cast<float>(time_now - last_time); 
        // 可以在这里使用 delta_time 进行其他计算
    }
    last_time = time_now;

    if(start_signal == 1)
        m3508_1.setTargetCurrent(1000);

    else if(start_signal == 2)
        m3508_1.setTargetRPM(100);
    else if(start_signal == 3)
        m3508_1.setTargetRPM(-100);

    else if(start_signal == 4)
        m3508_1.setTargetRPM(0);

    else if(start_signal == 5)
        m3508_1.setTargetAngle(90.0f);

    else if(start_signal == 6)
        m3508_1.setTargetAngle(270.0f);

    else if(start_signal == 7)
        m3508_1.setTargetAngle(0.0f);

    else if(start_signal == 8)
        m3508_1.setTargetTotalAngle(720.0f);
    
    else if(start_signal == 9)
        m3508_1.setTargetTotalAngle(-720.0f);
    
    else
        m3508_1.setTargetCurrent(0);
    cnt++;
    if(cnt > 3)
    {
        debug_uart.printf_DMA("%f,%f\r\n", m3508_1.getTotalAngle(), m3508_1.getTargetTotalAngle());
        cnt= 0;
    }
    
    
}


