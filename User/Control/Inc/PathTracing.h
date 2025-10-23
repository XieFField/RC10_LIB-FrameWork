#ifndef PATH_TRACING_H
#define PATH_TRACING_H

#include <math.h>
// 数学常量定义
static const float PI = 3.14159265358979323846f;
// 路径点结构 - 存储世界坐标系中的路径点信息
typedef struct {
    float x;        // 世界坐标系x坐标（米）
    float y;        // 世界坐标系y坐标（米）
    float theta;    // 期望朝向角度（弧度）
} Waypoint;

// 机器人状态 - 存储机器人当前状态信息
typedef struct {
    float current_x;        // 当前x坐标（米）
    float current_y;        // 当前y坐标（米）
    float current_theta;    // 当前朝向角度（弧度）
    float linear_velocity;  // 当前线速度（米/秒）
    float angular_velocity; // 当前角速度（弧度/秒）
} RobotState;

// 路径跟踪配置参数
typedef struct {
    float max_linear_velocity;     // 最大线速度（米/秒）
    float max_angular_velocity;    // 最大角速度（弧度/秒）
    float linear_acceleration;     // 线加速度（米/秒?）
    float angular_acceleration;    // 角加速度（弧度/秒?）
    float goal_tolerance;          // 目标点容差（米）
    float lookahead_distance;      // 前视距离（米）
} PathTracingConfig;

class PathTracing {
private:
    Waypoint* waypoints_;                  // 路径点数组
    unsigned int max_waypoints_;           // 最大路径点数
    unsigned int current_waypoint_count_;  // 当前路径点数
    unsigned int current_target_index_;    // 当前目标点索引

    RobotState robot_state_;               // 机器人当前状态
    PathTracingConfig config_;             // 跟踪配置参数
    // 上一时刻角速度，用于角速度加速度限制（每个实例一份）
    float last_angular_velocity_;          // 存储上一控制周期的角速度

    // 私有方法
    float calculateDistance(float x1, float y1, float x2, float y2); // 计算两点间距离
    float normalizeAngle(float angle);     // 角度归一化到[-π, π]
    float calculateAngleToTarget(float target_x, float target_y); // 计算到目标点的角度
    bool isGoalReached(float target_x, float target_y);           // 检查是否到达目标点
    void purePursuitControl(float target_x, float target_y);      // Pure Pursuit控制算法

public:
    PathTracing();  // 默认构造函数
    explicit PathTracing(unsigned int max_points); // 指定最大路径点数的构造函数
    PathTracing(Waypoint* buffer, unsigned int max_points); // 使用外部缓冲区的构造函数
    ~PathTracing(); // 析构函数

    bool init(Waypoint* buffer, unsigned int max_points); // 初始化路径跟踪器

    // 路径管理
    bool addWaypoint(float x, float y, float theta); // 添加路径点
    bool clearWaypoints();                           // 清空所有路径点
    unsigned int getWaypointCount();                 // 获取路径点数量

    // 配置管理
    void setConfig(float max_linear_vel, float max_angular_vel,
                   float linear_accel, float angular_accel,
                   float tolerance, float lookahead); // 设置配置参数
    PathTracingConfig getConfig();                    // 获取当前配置

    // 状态管理
    void setRobotState(float x, float y, float theta); // 设置机器人状态
    RobotState getRobotState();                        // 获取机器人状态

    // 路径跟踪控制
    bool planPath();                       // "规划"路径（实际是初始化跟踪状态）
    void executeOneStep(float dt_seconds); // 执行一步跟踪控制
    bool isPathCompleted();                // 检查路径是否完成

    // 运动控制输出
    void calculateMotionCommands(float* linear_vel, float* angular_vel); // 获取运动控制命令
    Waypoint getCurrentTarget();          // 获取当前目标点
    float getPathLength();                // 计算路径总长度
};

#endif // PATH_TRACING_H