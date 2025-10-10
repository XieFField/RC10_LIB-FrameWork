#include "frame_Chassis.h"
#include "main.h"
#include "math.h"
#include "Motor_DJI.h"
#include "BSP_fdCAN_Driver.h"
#include "APP_PID.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"

// --- 全局对象定义 ---
fdCANbus* CAN1_Bus = fdCANbus::getInstance(&hfdcan1);

// DJI M3508电机
M3508 wheel_motors[4] = {
    M3508(0x201, CAN1_Bus), // ID 0x201 - 左前轮
    M3508(0x202, CAN1_Bus), // ID 0x202 - 右前轮
    M3508(0x203, CAN1_Bus), // ID 0x203 - 左后轮
    M3508(0x204, CAN1_Bus)  // ID 0x204 - 右后轮
};

// DJI电机组
DJI_Group DJI_Group_1(0x200, CAN1_Bus);

// 底盘实例 - 使用QuanChassis，参数：底盘半长, 底盘半宽, 轮子半径, 旋转半径, 减速比
QuanChassis* my_chassis = nullptr;

// M3508电机减速比
const float M3508_GEAR_RATIO = 19.2f;

// 电机PID参数配置
PID_Param_Config test_m3508_speed_pid_params = {
    .kp = 18.0f,
    .ki = 0.015f,
    .kd = 0.0f,
    .I_Outlimit = 8000.0f, 
    .isIOutlimit = true, 
    .output_limit = 15000.0f,   
    .deadband = 5.0f 
};

PID_Param_Config test_m3508_angle_pid_params = {
    .kp = 30.0f,
    .ki = 0.0f,
    .kd = 1.1f,
    .I_Outlimit = 0.0f, 
    .isIOutlimit = true, 
    .output_limit = 400.0f,   
    .deadband = 0.8f 
};

// --- 控制任务 ---
class ChassisControlTask : public RtosTask {
public:
    ChassisControlTask() : RtosTask("ChassisTask", 1) {} // 10ms周期, 100Hz
    
    
    void execute() {
        loop();
    }
    
protected:
    void loop() override {
        
        Robot_Twist target_speed;
        
        static uint32_t control_time = 0;
        static uint8_t test_mode = 0;
        
        control_time++;
        if (control_time >= 200) {  // 200 * 10ms = 2s
            control_time = 0;
            test_mode = (test_mode + 1) % 6;
        }
        
        // 根据测试模式设置目标速度
        switch (test_mode) {
            case 0:  // 停止
                target_speed.vx = 0.0f;
                target_speed.vy = 0.0f;
                target_speed.yaw_rate = 0.0f;
                break;
            case 1:  // 前进
                target_speed.vx = 0.5f;
                target_speed.vy = 0.0f;
                target_speed.yaw_rate = 0.0f;
                break;
            case 2:  // 后退
                target_speed.vx = -0.5f;
                target_speed.vy = 0.0f;
                target_speed.yaw_rate = 0.0f;
                break;
            case 3:  // 左移
                target_speed.vx = 0.0f;
                target_speed.vy = 0.5f;
                target_speed.yaw_rate = 0.0f;
                break;
            case 4:  // 右移
                target_speed.vx = 0.0f;
                target_speed.vy = -0.5f;
                target_speed.yaw_rate = 0.0f;
                break;
            case 5:  // 旋转
                target_speed.vx = 0.0f;
                target_speed.vy = 0.0f;
                target_speed.yaw_rate = 1.0f;
                break;
        }
        
       
        // 3. 执行逆运动学计算
        my_chassis->inverseKinematics(target_speed.vx, target_speed.vy, target_speed.yaw_rate);
        
       
        
        // 4. 将计算出的轮子速度设置给电机
        for (int i = 0; i < 4; i++) {
            float wheel_speed = my_chassis->getWheelSpeed(i);
            // 将rad/s转换为RPM，考虑减速比
            float rpm = wheel_speed * 60.0f / (2.0f * 3.14159265358979323846f) * M3508_GEAR_RATIO;
            wheel_motors[i].setTargetRPM(rpm);
        }
        
        // 5. 发送CAN指令
        //CAN1_Bus->send();
    }
};

// 底盘控制任务实例
ChassisControlTask chassis_control_task;

// --- 初始化函数 ---
void user_setup() {
    // 1. 初始化电机和PID 
    for (int i = 0; i < 4; ++i) {
        wheel_motors[i].pid_init(test_m3508_speed_pid_params, 0.0f, test_m3508_angle_pid_params, 0.0f);
        DJI_Group_1.addMotor(&wheel_motors[i]);
        CAN1_Bus->registerMotor(&wheel_motors[i]);
    }
    CAN1_Bus->registerMotor(&DJI_Group_1);
    CAN1_Bus->init();
    
    // 2. 创建底盘对象 - 参数：底盘半长、底盘半宽、轮子半径、旋转半径、减速比
    // 使用实际测量的旋转半径值，sqrt(0.2^2 + 0.15^2) = 0.25f
    // M3508减速比为19.2:1
    my_chassis = new QuanChassis(0.2f, 0.15f, 0.076f, 0.25f, M3508_GEAR_RATIO);
    
}

// 外部控制接口函数 - 对应模板中的接口
void chassis_set_velocity(float vx, float vy, float wz)
{
    if (my_chassis != nullptr) {
        my_chassis->inverseKinematics(vx, vy, wz);
        my_chassis->updateKinematics();
        
        for (int i = 0; i < 4; i++) {
            float wheel_speed = my_chassis->getWheelSpeed(i);
            // 将rad/s转换为RPM，考虑减速比
            float rpm = wheel_speed * 60.0f / (2.0f * 3.14159265358979323846f) * M3508_GEAR_RATIO;
            wheel_motors[i].setTargetRPM(rpm);
        }
        
         //CAN1_Bus->send();
    }
}

// 获取底盘状态函数
void chassis_get_status(float* vx, float* vy, float* wz)
{
    if (my_chassis != nullptr && vx != nullptr && vy != nullptr && wz != nullptr) {
        
        *vx = my_chassis->getChassisVx();
        *vy = my_chassis->getChassisVy();
        *wz = my_chassis->getChassisYawRate();
    }
}

// 停止底盘
void chassis_stop()
{
    chassis_set_velocity(0.0f, 0.0f, 0.0f);
}

// FreeRTOS任务函数
void ChassisControlTaskFunc(void* argument)
{
    // 用户初始化
    user_setup();
    
    while (1) {
        // 调用底盘控制任务循环
        chassis_control_task.execute();
        
        // 10ms周期
        osDelay(10);
    }
}

// 任务句柄
osThreadId_t chassisControlTaskHandle;

// 创建底盘控制任务
void chassis_control_create()
{
    const osThreadAttr_t chassisControlTask_attributes = {
        .name = "ChassisControl",
        .stack_size = 1024,
        .priority = (osPriority_t) osPriorityNormal,
    };
    chassisControlTaskHandle = osThreadNew(ChassisControlTaskFunc, NULL, &chassisControlTask_attributes);
}
