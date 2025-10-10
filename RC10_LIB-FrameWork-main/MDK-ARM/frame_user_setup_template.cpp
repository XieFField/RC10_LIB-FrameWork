/* user_setup.cpp - 基于RC10_LIB模板的底盘控制实现 */
#include "frame_Chassis.h"
#include "main.h"
#include "math.h"
#include "Motor_DJI.h"
#include "BSP_fdCAN_Driver.h"
#include "APP_PID.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include "BSP_IMU.h" // 假设你有一个IMU模块

// --- 全局对象定义 ---
fdCANbus* CAN1_Bus = fdCANbus::getInstance(&hfdcan1);

// DJI M3508电机实例 - 按照模板的命名方式
M3508 wheel_motors[4] = {
    M3508(0x201, CAN1_Bus), // ID 0x201 - 左前轮
    M3508(0x202, CAN1_Bus), // ID 0x202 - 右前轮
    M3508(0x203, CAN1_Bus), // ID 0x203 - 左后轮
    M3508(0x204, CAN1_Bus)  // ID 0x204 - 右后轮
};

// DJI电机组
DJI_Group DJI_Group_1(0x200, CAN1_Bus);

// 底盘实例 - 使用QuanChassis，参数：轮半径, 最大RPM, x间距, y间距
QuanChassis* my_chassis = nullptr;

// IMU对象 - 假设的IMU模块
IMU_Class my_imu;

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

// 速度和角度结构体 - 对应模板中的Robot_Twist和Angle_Twist
typedef struct {
    float vx;         // X方向速度 (m/s)
    float vy;         // Y方向速度 (m/s)
    float yaw_rate;   // 偏航角速度 (rad/s)
} Robot_Twist;

typedef struct {
    float roll;       // 横滚角 (rad)
    float pitch;      // 俯仰角 (rad)
    float yaw;        // 偏航角 (rad)
} Angle_Twist;

// --- 初始化函数 ---
void user_setup() {
    // 1. 初始化电机和PID - 按照模板的方式
    for (int i = 0; i < 4; ++i) {
        wheel_motors[i].pid_init(test_m3508_speed_pid_params, 0.0f, test_m3508_angle_pid_params, 0.0f);
        DJI_Group_1.addMotor(&wheel_motors[i]);
        CAN1_Bus->registerMotor(&wheel_motors[i]);
    }
    CAN1_Bus->registerMotor(&DJI_Group_1);
    CAN1_Bus->init();
    
    // 2. 创建底盘对象 - 参数：底盘半长、底盘半宽、轮子半径
    my_chassis = new QuanChassis(0.2f, 0.15f, 0.076f);
    
    // 3. 配置加速度限制 (可选) - 如果QuanChassis支持这些方法
    // my_chassis->reset_AccLimitStatus(true); // 启用
    // my_chassis->reset_AccValue(1.0f); // 1.0 m/s^2
    
    // 4. 初始化IMU (如果需要)
    // my_imu.init();
}

// --- 控制任务 ---
class ChassisControlTask : public RtosTask {
public:
    ChassisControlTask() : RtosTask("ChassisTask", 1) {} // 10ms周期, 100Hz
    
protected:
    void loop() override {
        // 1. 从遥控器或上位机获取目标速度
        Robot_Twist target_speed;
        
        // 这里使用测试模式，实际应用中应该从遥控器获取
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
        
        // 2. 从IMU获取当前姿态 (如果需要)
        // Angle_Twist current_angle = my_imu.getAngle();
        // my_chassis->updateAngleData(current_angle);
        
        // 3. 执行逆运动学计算
        my_chassis->inverseKinematics(target_speed.vx, target_speed.vy, target_speed.yaw_rate);
        
        // 4. 更新运动学状态
        my_chassis->updateKinematics();
        
        // 5. 将计算出的轮子速度设置给电机
        for (int i = 0; i < 4; i++) {
            float wheel_speed = my_chassis->getWheelSpeed(i);
            // 将rad/s转换为RPM
            float rpm = wheel_speed * 60.0f / (2.0f * 3.14159265358979323846f);
            wheel_motors[i].set_speed(rpm);
        }
        
        // 6. 发送CAN指令
        CAN1_Bus->send();
        
        // 7. 更新底盘状态 (正运动学)
        my_chassis->forwardKinematics();
    }
};

// 底盘控制任务实例
ChassisControlTask chassis_control_task;

// 外部控制接口函数 - 对应模板中的接口
void chassis_set_velocity(float vx, float vy, float wz)
{
    if (my_chassis != nullptr) {
        my_chassis->inverseKinematics(vx, vy, wz);
        my_chassis->updateKinematics();
        
        for (int i = 0; i < 4; i++) {
            float wheel_speed = my_chassis->getWheelSpeed(i);
            float rpm = wheel_speed * 60.0f / (2.0f * 3.14159265358979323846f);
            wheel_motors[i].set_speed(rpm);
        }
        
        CAN1_Bus->send();
    }
}

// 获取底盘状态函数
void chassis_get_status(float* vx, float* vy, float* wz)
{
    if (my_chassis != nullptr && vx != nullptr && vy != nullptr && wz != nullptr) {
        my_chassis->forwardKinematics();
        // 这里需要添加获取底盘速度的接口
        // 目前暂时返回0
        *vx = 0.0f;
        *vy = 0.0f;
        *wz = 0.0f;
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
        chassis_control_task.loop();
        
        // 10ms周期
        osDelay(10);
    }
}

// 任务句柄
osThreadId_t chassisControlTaskHandle;

// 创建底盘控制任务
void chassis_control_task_create()
{
    const osThreadAttr_t chassisControlTask_attributes = {
        .name = "ChassisControl",
        .stack_size = 2048,
        .priority = (osPriority_t) osPriorityNormal,
    };
    
    chassisControlTaskHandle = osThreadNew(ChassisControlTaskFunc, NULL, &chassisControlTask_attributes);
}