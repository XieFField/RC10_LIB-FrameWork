#include "PathPlanner.h"
#include "Module_ChassisOmni.h"
#include "Motor_Base.h"
#include "Motor_DJI.h"
#include <stdio.h>

// Bridge 示例：把 PathPlanner 输出下发给 Chassis（示例使用 4 轮全向/麦克纳姆接口）
// 注意：此文件是示例 glue 代码，需替换 motor_* 变量为你工程中实际创建的 Motor 对象。

// 配置（按实际底盘参数修改）
static constexpr float kWheelRadius = 0.05f; // m
static constexpr float kMaxWheelRPM = 4000.0f; // 示例
static constexpr float kChassisRadius = 0.2f; // m

// 4 轮全向底盘实例（模板参数为轮数）
static Chassis_Omni<4> chassis(kWheelRadius, kMaxWheelRPM, kChassisRadius);

// 占位电机指针（请在系统初始化中把具体的 Motor 对象注册到这些指针）
static Motor_Base* wheel0 = nullptr;
static Motor_Base* wheel1 = nullptr;
static Motor_Base* wheel2 = nullptr;
static Motor_Base* wheel3 = nullptr;

// 初始化桥接：注册 motor 指针并把它们注册到 chassis
void Bridge_Init(Motor_Base* m0, Motor_Base* m1, Motor_Base* m2, Motor_Base* m3)
{
    wheel0 = m0; wheel1 = m1; wheel2 = m2; wheel3 = m3;
    chassis.registerWheelMotor(0, wheel0);
    chassis.registerWheelMotor(1, wheel1);
    chassis.registerWheelMotor(2, wheel2);
    chassis.registerWheelMotor(3, wheel3);
}

// 主循环调用：把 planner 的速度发送到底盘并触发 update
void Bridge_Step(PathPlanner& planner, float dt)
{
    // 1) advance planner
    planner.executeOneStep(dt);

    // 2) read planner speed
    RobotState st = planner.getRobotState();

    // 3) build twist and send to chassis
    Robot_Twist twist;
    twist.vx = st.linear_velocity; // assume linear_velocity is forward velocity (m/s)
    twist.vy = 0.0f; // for differential-style behavior; change if planner supplies vy
    twist.yaw_rate = st.angular_velocity; // rad/s

    chassis.setRobotSpeed(twist);
    chassis.update(); // this should compute wheel RPM and call wheel->setTargetRPM()
}

// Simple demo: show how to call Bridge_Init and Bridge_Step
void Bridge_Demo()
{
    // NOTE: you must replace these with real motor objects from your setup code
    extern M3508 m3508_1; // placeholder extern declarations; replace with actual symbols
    extern M3508 m3508_2;
    extern M3508 m3508_3;
    extern M3508 m3508_4;

    Bridge_Init(&m3508_1, &m3508_2, &m3508_3, &m3508_4);

    // create a small planner demo
    static Waypoint buf[8];
    PathPlanner planner(buf, 8);
    planner.addWaypoint(0,0,0);
    planner.addWaypoint(1,0,0);
    planner.addWaypoint(1,1,0);
    planner.planPath();

    const float dt = 0.02f;
    for (int i=0;i<500;i++){
        Bridge_Step(planner, dt);
        // add logging if needed
    }
}
