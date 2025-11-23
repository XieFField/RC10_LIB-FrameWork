#include "Onim_demo.h"

template <std::size_t WheelCount>
OnimDemo<WheelCount>::OnimDemo(float wheel_radius, float max_wheel_rpm, float chassis_radius) : 
    Chassis_Onim<WheelCount>(wheel_radius, max_wheel_rpm, chassis_radius),  // 调用第一个基类的构造函数
    RtosTask("OnimDemo", 1)  // 调用第二个基类的构造函数
{                                                           

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
    
    // 遥控器控制逻辑
    // 将PPM信号转换为-1到1之间的值
    if(abs(air_joy.LEFT_Y - 1500) < 60) air_joy.LEFT_Y = 1500;
    if(abs(air_joy.LEFT_X - 1500) < 60) air_joy.LEFT_X = 1500;
    if(abs(air_joy.RIGHT_X - 1500) < 60) air_joy.RIGHT_X = 1500;
    float forward_speed = (air_joy.LEFT_Y - 1500.0f) / 500.0f;  // 前进/后退
    float lateral_speed = (air_joy.LEFT_X - 1500.0f) / 500.0f;  // 左移/右移
    float rotation_speed = (air_joy.RIGHT_X - 1500.0f) / 500.0f; // 旋转
    
    // 限制在-1到1之间
    forward_speed = (forward_speed > 1.0f) ? 1.0f : ((forward_speed < -1.0f) ? -1.0f : forward_speed);
    lateral_speed = (lateral_speed > 1.0f) ? 1.0f : ((lateral_speed < -1.0f) ? -1.0f : lateral_speed);
    rotation_speed = (rotation_speed > 1.0f) ? 1.0f : ((rotation_speed < -1.0f) ? -1.0f : rotation_speed);
    
    // 应用最大速度限制
    this->robot_target_twist_.vx = forward_speed * v_max;   
    this->robot_target_twist_.vy = lateral_speed * v_max;   
    this->robot_target_twist_.yaw_rate = rotation_speed * w_max;  
    
   
    if (air_joy.SWA > 1800) {  // 开关在高位，可能是高速模式
        this->robot_target_twist_.vx *= 1.5f;
        this->robot_target_twist_.vy *= 1.5f;
        this->robot_target_twist_.yaw_rate *= 1.5f;
    } else if (air_joy.SWA < 1200) {  // 开关在低位，可能是低速模式
        this->robot_target_twist_.vx *= 0.5f;
        this->robot_target_twist_.vy *= 0.5f;
        this->robot_target_twist_.yaw_rate *= 0.5f;
    }
//		// 测试模式控制
//    static uint32_t control_time = 0; 
//    
//		
//		// 根据测试模式设置目标速度
//    switch (test_mode) { 
//        case 0:  // 停止 
//            this->robot_target_twist_.vx = 0.0f; 
//            this->robot_target_twist_.vy = 0.0f; 
//            this->robot_target_twist_.yaw_rate = 0.0f; 
//            break; 
//        case 1:  // 前进 
//            this->robot_target_twist_.vx = 0.3f;   
//            this->robot_target_twist_.vy = 0.0f;   
//            this->robot_target_twist_.yaw_rate = 0.0f; 
//            break; 
//        case 2:  // 后退 
//            this->robot_target_twist_.vx = -0.3f;   
//            this->robot_target_twist_.vy = 0.0f; 
//            this->robot_target_twist_.yaw_rate = 0.0f; 
//            break; 
//        case 3:  // 左移 
//            this->robot_target_twist_.vx = 0.0f;   
//            this->robot_target_twist_.vy = 0.3f;   
//            this->robot_target_twist_.yaw_rate = 0.0f; 
//            break; 
//        case 4:  // 右移 
//            this->robot_target_twist_.vx = 0.0f;   
//            this->robot_target_twist_.vy = -0.3f;   
//            this->robot_target_twist_.yaw_rate = 0.0f; 
//            break; 
//        case 5:  // 旋转 
//            this->robot_target_twist_.vx = 0.0f;   
//            this->robot_target_twist_.vy = 0.0f;   
//            this->robot_target_twist_.yaw_rate = 0.5f;  
//            break; 
//    } 
		
    this->update();
}
// 构造函数，需要调用两个基类的构造函数

template class OnimDemo<4>; // 显式实例化4轮版本

