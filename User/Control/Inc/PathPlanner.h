#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <math.h>

// 定义路径点结构体
typedef struct {
    float x;
    float y;
    float theta; // 朝向角度
} Waypoint;

// 定义机器人状态
typedef struct {
    float current_x;
    float current_y;
    float current_theta;
    float linear_velocity;
    float angular_velocity;
} RobotState;

// 路径规划参数
typedef struct {
    float max_linear_velocity;
    float max_angular_velocity;
    float linear_acceleration;
    float angular_acceleration;
    float goal_tolerance; // 目标点容差
    float lookahead_distance; // 前视距离
} PathPlannerConfig;

class PathPlanner {
private:
    // 路径点数组
    Waypoint* waypoints_;
    unsigned int max_waypoints_;
    unsigned int current_waypoint_count_;
    unsigned int current_target_index_;
    
    // 机器人状态
    RobotState robot_state_;
    
    // 规划器配置
    PathPlannerConfig config_;
    
    // 私有方法
    float calculateDistance(float x1, float y1, float x2, float y2);
    float normalizeAngle(float angle);
    float calculateAngleToTarget(float target_x, float target_y);
    bool isGoalReached(float target_x, float target_y);
    void purePursuitControl(float target_x, float target_y);
    
public:
    // 构造函数和析构函数
    // 注意：为避免运行时动态分配（new/delete），请在创建对象后
    // 通过 init(buffer, max_points) 注入 caller 管理的缓冲区。
    PathPlanner();
    // 保留接受 max_points 的构造函数，但不再进行动态分配，需配合 init 使用
    explicit PathPlanner(unsigned int max_points);
    // 可直接通过构造函数注入缓冲区
    PathPlanner(Waypoint* buffer, unsigned int max_points);
    ~PathPlanner();

    // 初始化：将外部提供的缓冲区用于存储路径点
    // 返回 true 表示初始化成功（buffer 非空且 max_points>0）
    bool init(Waypoint* buffer, unsigned int max_points);
    
    // 路径管理
    bool addWaypoint(float x, float y, float theta);
    bool clearWaypoints();
    unsigned int getWaypointCount();
    
    // 规划器配置
    void setConfig(float max_linear_vel, float max_angular_vel, 
                   float linear_accel, float angular_accel,
                   float tolerance, float lookahead);
    PathPlannerConfig getConfig();
    
    // 状态管理
    void setRobotState(float x, float y, float theta);
    RobotState getRobotState();
    
    // 路径规划
    bool planPath();
    // 执行一步路径跟踪，dt_seconds 为此次步骤经过的秒数（必需）
    void executeOneStep(float dt_seconds);
    bool isPathCompleted();
    
    // 运动控制
    void calculateMotionCommands(float* linear_vel, float* angular_vel);
    
    // 工具函数
    Waypoint getCurrentTarget();
    float getPathLength();
};

#endif // PATH_PLANNER_H