#include "frame_demo.h"

fdCANbus* const demoCAN1_Bus = fdCANbus::getInstance(&hfdcan1); // 获取FDCAN1的唯一实例
DJI_Group GroupCAN1_High(send_idHigh(), demoCAN1_Bus); // 5~8号M3508/M2006电机
M3508 m3508_1(7, demoCAN1_Bus);


DJI_Group GroupCAN1_Low(send_idLow(), demoCAN1_Bus); // 1~4号M3508/M2006电机
M2006 m2006_1(1, demoCAN1_Bus);



// 使用 volatile 防止编译器优化，确保在调试时可以观察到值的变化
volatile int counter = 0;
volatile uint8_t start_signal = 0;

volatile int a = 0;
volatile float delta_time = 0.0f; //目前使用的单位是微秒
volatile uint64_t last_time = 0;


void DJI_MotorDemo::init()
{
    GroupCAN1_High.addMotor(&m3508_1);
    demoCAN1_Bus->registerMotor(&GroupCAN1_High);
    demoCAN1_Bus->registerMotor(&m3508_1);
    
    m3508_1.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);
    start_signal = 0;

    GroupCAN1_Low.addMotor(&m2006_1);
    demoCAN1_Bus->registerMotor(&GroupCAN1_Low);
    demoCAN1_Bus->registerMotor(&m2006_1);

    demoCAN1_Bus->init();
    m2006_1.pid_init(m3508_speed_pid_params, 0.0f, m3508_angle_pid_params, 0.0f);


    start(osPriorityNormal, 256);
}
int cnt = 0;

float m2006_targetAngle = 0.0f;
int m2006_signal = 1;

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
    
    if(m2006_signal == 2)
        m2006_1.setTargetAngle(m2006_targetAngle);
    if(m2006_signal == 1)
        m2006_1.setTargetCurrent(0);

}


