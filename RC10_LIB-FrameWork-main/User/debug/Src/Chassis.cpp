//#include "Chassis.h"
//#include "main.h"
//#include "math.h"
//#include "arm_math.h"
//#include "Motor_DJI.h"
//#include "APP_PID.h"
//#include "Motor_Base.h"

//// 只在你自己的 Chassis.cpp 里加，不用动基类文件
//// 显式告诉编译器：生成 Chassis_Base<4> 的版本（解决构造函数未定义）
//template class Chassis_Base<4>;

//extern Wheel wheel[4];
//QuanChassis::QuanChassis(float wheel_circle, float wheel_radius)
//:Chassis_Base<4>(wheel_circle,wheel_radius)
//{
//    for(int i=0;i<4;i++)
//    {
//        wheel[i].wheel_circle = wheel_circle;
//        wheel[i].wheel_radius = wheel_radius;
//    
//    }
//}     

//void setWheelMaxRPM(float max_rpm)
//{
//    for(int i=0;i<4;i++)
//    {
//        wheel[i].max_rpm = max_rpm;
//    }
//}

//void QuanChassis::inverseKinematics(const Robot_Speed_t& robot_speed)
//{
//    // 机器人参数
//    float L = wheel[0].wheel_circle;      // 轮子到中心距离
//    float wheel_radius = wheel[0].wheel_radius; // 轮子半径

//    // 运动学矩阵（左前、右前、左后、右后）
//    float kinematics_data[4 * 3] = {
//        1,  1,  L,    // 左前轮
//        1, -1, -L,    // 右前轮
//        1,  1, -L,    // 左后轮
//        1, -1,  L     // 右后轮
//    };
//    arm_matrix_instance_f32 kinematics_matrix;
//    arm_mat_init_f32(&kinematics_matrix, 4, 3, kinematics_data);

//    // 输入速度向量
//    float32_t robot_speed_data[3] = {robot_speed.linear.Vx, robot_speed.linear.Vy, robot_speed.angular.wl};
//    arm_matrix_instance_f32 robot_speed_vec;
//    arm_mat_init_f32(&robot_speed_vec, 3, 1, robot_speed_data);

//    // 输出轮子线速度
//    float32_t wheel_speed_data[4];
//    arm_matrix_instance_f32 wheel_speed_vec;
//    arm_mat_init_f32(&wheel_speed_vec, 4, 1, wheel_speed_data);

//    // 矩阵乘法
//    arm_mat_mult_f32(&kinematics_matrix, &robot_speed_vec, &wheel_speed_vec);

//    // 线速度转RPM
//    for (int i = 0; i < 4; i++) {
//        wheel[i].rpm = (wheel_speed_data[i] / (2 * 3.14159f * wheel_radius)) * 60.0f;
//        if(wheel[i].rpm > wheel[i].max_rpm) wheel[i].rpm = wheel[i].max_rpm;
//        if(wheel[i].rpm < -wheel[i].max_rpm) wheel[i].rpm = -wheel[i].max_rpm;
//    }
//}

//void QuanChassis::forwardKinematics(Robot_Speed_t& robot_speed)
//{
//    // 将RPM转换为线速度（单位：m/s）
//    float V1 = (wheel[0].rpm / 60.0f) * (2 * 3.14159f * wheel[0].wheel_radius); // 左前轮
//    float V2 = (wheel[1].rpm / 60.0f) * (2 * 3.14159f * wheel[1].wheel_radius); // 右前轮
//    float V3 = (wheel[2].rpm / 60.0f) * (2 * 3.14159f * wheel[2].wheel_radius); // 左后轮
//    float V4 = (wheel[3].rpm / 60.0f) * (2 * 3.14159f * wheel[3].wheel_radius); // 右后轮

//    // 计算机器人线速度和角速度
//    robot_speed.linear.Vx = (V1 + V2 + V3 + V4) / 4.0f; // x方向线速度
//    robot_speed.linear.Vy = (-V1 + V2 + V3 - V4) / 4.0f; // y方向线速度
//    robot_speed.angular.wl = (-V1 + V2 - V3 + V4) / (4.0f * ((wheel[0].wheel_circle + wheel[1].wheel_circle) / 2.0f)); // 绕z轴角速度
//}

//// 1. 补基类要求的纯虚函数：inverseKinematics（参数必须是 const Robot_Twist&）
//void QuanChassis::inverseKinematics(const Robot_Twist& twist) {
//    
//}

//// 2. 补基类要求的纯虚函数：forwardKinematics（无参数）
//void QuanChassis::forwardKinematics() {
//    
//}


