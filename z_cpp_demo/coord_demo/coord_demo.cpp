#include "APP_CoordConvert.h"
#include <cstdio>

// 模拟全局变量或类成员
// 1. 变换对象（复用，不重新创建）
static HomogeneousTransform2D T_lidar_to_robot;   // 雷达 -> 机器人
static HomogeneousTransform2D T_robot_to_arm;     // 机器人 -> 机械臂

// 2. 机器人当前状态
static Point2D current_robot_pose_world = {0, 0, 0}; // 机器人在世界坐标系的位置

void System_Init()
{
    // --- 步骤1：配置固定的硬件安装参数 ---    ...
    
    // 假设雷达安装在机器人中心前方 0.2m 处
    Point2D lidar_install_pose = {0.2f, 0.0f, 0.0f}; 
    T_lidar_to_robot.setTransform(lidar_install_pose);

    // 假设机械臂安装在机器人中心右侧 0.15m，后方 0.1m，且旋转了 90度
    Point2D arm_install_pose = {-0.1f, -0.15f, 1.57f};
    T_robot_to_arm.setTransform(arm_install_pose);

    printf("系统初始化完成：变换矩阵已建立。\n");
}

void Control_Loop_Simulation()
{
    // 模拟时间步长
    float dt = 0.1f; 
    // 模拟机器人运动速度 (vx=1m/s, vy=0.5m/s, w=0.1rad/s)
    float vx = 1.0f, vy = 0.5f, w = 0.1f;

    int loop_count = 0;

    while (loop_count < 10) // 模拟10次循环
    {
        printf("\n--- Loop %d ---\n", loop_count);

        // --- 1. 模拟接收雷达数据 (相对于雷达坐标系) ---
        // 假设雷达扫到了一个障碍物，距离雷达前方 1.0m
        Point2D obstacle_in_lidar = {1.0f, 0.0f, 0.0f};

        // --- 2. 变换1：雷达坐标 -> 机器人底盘坐标 ---
        // 直接复用 T_lidar_to_robot 对象，无需重新创建
        Point2D obstacle_in_robot = T_lidar_to_robot.apply(obstacle_in_lidar);
        
        printf("障碍物(底盘系): x=%.2f, y=%.2f\n", obstacle_in_robot.x, obstacle_in_robot.y);


        // --- 3. 更新机器人世界坐标 (模拟里程计/定位更新) ---
        // 简单积分更新 (实际项目中这里通常是读取定位模块的结果)
        // 注意：这里简化了运动学模型，仅作演示
        float c = cos(current_robot_pose_world.theta);
        float s = sin(current_robot_pose_world.theta);
        current_robot_pose_world.x += (vx * c - vy * s) * dt;
        current_robot_pose_world.y += (vx * s + vy * c) * dt;
        current_robot_pose_world.theta += w * dt;


        // --- 4. 变换2：计算机械臂基座的世界坐标 ---
        // 这里的逻辑是：
        // 机械臂在世界系 = T_world_to_robot * T_robot_to_arm
        // 因为 T_world_to_robot 是每时每刻都在变的，所以这里必须动态计算
        
        // 方法 A: 构造临时对象链式乘法 (标准做法)
        HomogeneousTransform2D T_world_to_robot(current_robot_pose_world);
        HomogeneousTransform2D T_world_to_arm = T_world_to_robot.multiply(T_robot_to_arm);
        
        // 获取机械臂基座原点(0,0)在世界系的位置
        Point2D arm_base_world = T_world_to_arm.apply(Point2D{0,0,0});

        printf("机器人(世界系): x=%.2f, y=%.2f, theta=%.2f\n", 
               current_robot_pose_world.x, current_robot_pose_world.y, current_robot_pose_world.theta);
        printf("机械臂(世界系): x=%.2f, y=%.2f\n", arm_base_world.x, arm_base_world.y);

        loop_count++;
    }
}

int main()
{
    System_Init();
    Control_Loop_Simulation();
    return 0;
}