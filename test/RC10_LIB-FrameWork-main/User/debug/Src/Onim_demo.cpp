#include "Onim_demo.h"

template <std::size_t WheelCount>
OnimDemo<WheelCount>::OnimDemo(float wheel_radius, float max_wheel_rpm, float chassis_radius) : 
    Chassis_Demo<WheelCount>(wheel_radius, max_wheel_rpm, chassis_radius),  // 调用第一个基类的构造函数
    RtosTask("OnimDemo", 1)  // 调用第二个基类的构造函数
{                                                                                           
     // 初始化成员变量
    target_twist_.vx = 0.0f;
    target_twist_.vy = 0.0f;
    target_twist_.yaw_rate = 0.0f;
    
    // 初始化电机指针数组
    for(std::size_t i = 0; i < WheelCount; i++) {
        motors_[i] = nullptr;
} 
}

template <std::size_t WheelCount>
// 1. 加模板声明
void OnimDemo<WheelCount>::chassisInit(DJI_Motor* motor)  // 用数组适配任意轮数
{ 
     if (motor_count_ < WheelCount) {  // 防止数组越界
        motors_[motor_count_] = motor;  //
        motor_count_++;  // 计数器+1，下次存到下一个位置
    }

    // 所有电机添加完成后再启动任务
    if (motor_count_ == WheelCount) {
        init_flag = true;
        start(osPriorityNormal, 128);
    }
}

template <std::size_t WheelCount>        
// 2. 加模板声明
void OnimDemo<WheelCount>::loop() 
{ 
    if(!init_flag) 
        return; 
    
    static uint64_t last_us = 0; 
    uint64_t now_us = TimeStamp::getInstance().getMicroseconds(); 
    if(last_us == 0) 
    { 
        last_us = now_us; 
        return; 
    } 
    uint64_t dt_us = (now_us >= last_us) ? (now_us - last_us) : 0; 
    last_us = now_us; 
    if(dt_us == 0) 
        return; 
    if(dt_us > 200000) 
        dt_us = 200000; 
    float dt = dt_us * 1e-6f; 
    
    const float v_max = 0.5f;     // 最大线速度 (m/s)
    const float w_max = 1.0f;     // 最大角速度 (rad/s)
    
		// 测试模式控制
    static uint32_t control_time = 0; 
    
		
		// 根据测试模式设置目标速度
    switch (test_mode) { 
        case 0:  // 停止 
            target_twist_.vx = 0.0f; 
            target_twist_.vy = 0.0f; 
            target_twist_.yaw_rate = 0.0f; 
            break; 
        case 1:  // 前进 
            target_twist_.vx = 0.8f;   
            target_twist_.vy = 0.5f;   
            target_twist_.yaw_rate = 0.0f; 
            break; 
        case 2:  // 后退 
            target_twist_.vx = -0.5f; 
            target_twist_.vy = 0.0f; 
            target_twist_.yaw_rate = 0.0f; 
            break; 
        case 3:  // 左移 
            target_twist_.vx = 0.0f; 
            target_twist_.vy = 0.5f; 
            target_twist_.yaw_rate = 0.0f; 
            break; 
        case 4:  // 右移 
            target_twist_.vx = 0.0f; 
            target_twist_.vy = -0.5f; 
            target_twist_.yaw_rate = 0.0f; 
            break; 
        case 5:  // 旋转 
            target_twist_.vx = 0.0f; 
            target_twist_.vy = 0.0f; 
            target_twist_.yaw_rate = 0.5f;  
            break; 
    } 

		this->robot_twist_ = target_twist_;  
    //this->update();
    this->updateKinematics();  // 使用公共方法更新运动学
    
    // 更新电机速度
    for(std::size_t i = 0; i < WheelCount; i++) {
        if(motors_[i]) {
            // 获取计算出的轮速并设置给电机
            // 假设Chassis_Demo类有获取轮速的方法
            float wheel_rpm = this->getWheelTargetRPM(i);
            motors_[i]->setTargetRPM(wheel_rpm);
        }
    }
}
// 构造函数，需要调用两个基类的构造函数

template class OnimDemo<4>; // 显式实例化4轮版本

