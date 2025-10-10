//#ifndef CHASSIS_H
//#define CHASSIS_H
//#include "main.h"
//#include "APP_PID.h"
//#include "Module_ChassisBase.h" // 假如 Chassis_Base 定义在这里
//#include "Motor_DJI.h"

//typedef struct {
//    struct {
//        float Vx;
//        float Vy;
//    } linear;
//    struct {
//        float wl;
//    } angular;
//} Robot_Speed_t;

//typedef struct{
//    float value;
//    float rpm;
//    float wheel_circle;
//    float wheel_radius; // 轮子半径
//    float max_rpm;  
//    M3508* motor;
//    //PID_Param_Config rpm_pid;
//} Wheel;

//extern Wheel wheel[4];
//class QuanChassis : public Chassis_Base<4> {
//private:
//    float wheel_circle;
//    float wheel_radius;
//    
//public:

//    virtual void inverseKinematics(const Robot_Twist& twist) override;
//    virtual void forwardKinematics() override;
//    QuanChassis(float wheel_circle, float wheel_radius);
//    void setWheelMaxRPM(float max_rpm);
//    void updateKinematics(); // 去掉 override，除非父类有虚函数
//    virtual void inverseKinematics(const Robot_Speed_t& robot_speed);
//    virtual void forwardKinematics(Robot_Speed_t& robot_speed);
//};

//#endif
