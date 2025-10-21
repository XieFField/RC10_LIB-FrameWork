#include "PathPlanner.h"
#include <stdlib.h>

#define PI 3.14159265359f
#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

// 默认构造函数（不分配缓冲区）
PathPlanner::PathPlanner() {
    waypoints_ = nullptr;
    max_waypoints_ = 0;
    current_waypoint_count_ = 0;
    current_target_index_ = 0;

    // 初始化机器人状态
    robot_state_.current_x = 0.0f;
    robot_state_.current_y = 0.0f;
    robot_state_.current_theta = 0.0f;
    robot_state_.linear_velocity = 0.0f;
    robot_state_.angular_velocity = 0.0f;

    // 默认配置
    config_.max_linear_velocity = 0.5f;
    config_.max_angular_velocity = 1.0f;
    config_.linear_acceleration = 0.1f;
    config_.angular_acceleration = 0.5f;
    config_.goal_tolerance = 0.05f;
    config_.lookahead_distance = 0.3f;
}

// 构造但不分配：保留 max_points 但仍需 init(buffer, max_points) 注入
PathPlanner::PathPlanner(unsigned int max_points) {
    waypoints_ = nullptr;
    max_waypoints_ = max_points;
    current_waypoint_count_ = 0;
    current_target_index_ = 0;

    robot_state_.current_x = 0.0f;
    robot_state_.current_y = 0.0f;
    robot_state_.current_theta = 0.0f;
    robot_state_.linear_velocity = 0.0f;
    robot_state_.angular_velocity = 0.0f;

    config_.max_linear_velocity = 0.5f;
    config_.max_angular_velocity = 1.0f;
    config_.linear_acceleration = 0.1f;
    config_.angular_acceleration = 0.5f;
    config_.goal_tolerance = 0.05f;
    config_.lookahead_distance = 0.3f;
}

// 通过外部缓冲区构造
PathPlanner::PathPlanner(Waypoint* buffer, unsigned int max_points) {
    waypoints_ = buffer;
    max_waypoints_ = max_points;
    current_waypoint_count_ = 0;
    current_target_index_ = 0;

    robot_state_.current_x = 0.0f;
    robot_state_.current_y = 0.0f;
    robot_state_.current_theta = 0.0f;
    robot_state_.linear_velocity = 0.0f;
    robot_state_.angular_velocity = 0.0f;

    config_.max_linear_velocity = 0.5f;
    config_.max_angular_velocity = 1.0f;
    config_.linear_acceleration = 0.1f;
    config_.angular_acceleration = 0.5f;
    config_.goal_tolerance = 0.05f;
    config_.lookahead_distance = 0.3f;
}

// 析构函数：不释放外部 buffer（调用者负责）
PathPlanner::~PathPlanner() {
    // waypoints_ 指向的缓冲区由调用者或创建者管理；不要 delete[]
    waypoints_ = nullptr;
}

// 计算两点间距离
float PathPlanner::calculateDistance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

// 角度归一化到 [-PI, PI]
float PathPlanner::normalizeAngle(float angle) {
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

// 计算到目标点的角度
float PathPlanner::calculateAngleToTarget(float target_x, float target_y) {
    float dx = target_x - robot_state_.current_x;
    float dy = target_y - robot_state_.current_y;
    return atan2f(dy, dx);
}

// 检查是否到达目标点
bool PathPlanner::isGoalReached(float target_x, float target_y) {
    float distance = calculateDistance(robot_state_.current_x, robot_state_.current_y, 
                                      target_x, target_y);
    return distance <= config_.goal_tolerance;
}

// Pure Pursuit 控制算法
void PathPlanner::purePursuitControl(float target_x, float target_y) {
    // 计算到目标点的距离和角度
    float dx = target_x - robot_state_.current_x;
    float dy = target_y - robot_state_.current_y;
    float distance_to_target = sqrtf(dx * dx + dy * dy);
    
    // 计算目标点在机器人坐标系中的位置
    float target_global_angle = atan2f(dy, dx);
    float alpha = normalizeAngle(target_global_angle - robot_state_.current_theta);
    
    // 计算曲率和角速度
    if (distance_to_target > 0.001f) {
        float curvature = 2.0f * sinf(alpha) / distance_to_target;
        robot_state_.angular_velocity = robot_state_.linear_velocity * curvature;
        
        // 限制角速度
        if (robot_state_.angular_velocity > config_.max_angular_velocity) {
            robot_state_.angular_velocity = config_.max_angular_velocity;
        } else if (robot_state_.angular_velocity < -config_.max_angular_velocity) {
            robot_state_.angular_velocity = -config_.max_angular_velocity;
        }
    } else {
        robot_state_.angular_velocity = 0.0f;
    }
}

// 添加路径点
bool PathPlanner::addWaypoint(float x, float y, float theta) {
    if (waypoints_ == nullptr || max_waypoints_ == 0) {
        return false; // 未初始化缓冲区
    }
    if (current_waypoint_count_ >= max_waypoints_) {
        return false; // 路径点已满
    }

    waypoints_[current_waypoint_count_].x = x;
    waypoints_[current_waypoint_count_].y = y;
    waypoints_[current_waypoint_count_].theta = theta;
    current_waypoint_count_++;

    return true;
}

// 清空路径点
bool PathPlanner::clearWaypoints() {
    current_waypoint_count_ = 0;
    current_target_index_ = 0;
    return true;
}

// 获取路径点数量
unsigned int PathPlanner::getWaypointCount() {
    return current_waypoint_count_;
}

// 设置规划器配置
void PathPlanner::setConfig(float max_linear_vel, float max_angular_vel, 
                           float linear_accel, float angular_accel,
                           float tolerance, float lookahead) {
    config_.max_linear_velocity = max_linear_vel;
    config_.max_angular_velocity = max_angular_vel;
    config_.linear_acceleration = linear_accel;
    config_.angular_acceleration = angular_accel;
    config_.goal_tolerance = tolerance;
    config_.lookahead_distance = lookahead;
}

// 获取配置
PathPlannerConfig PathPlanner::getConfig() {
    return config_;
}

// 设置机器人状态
void PathPlanner::setRobotState(float x, float y, float theta) {
    robot_state_.current_x = x;
    robot_state_.current_y = y;
    robot_state_.current_theta = normalizeAngle(theta);
}

// 获取机器人状态
RobotState PathPlanner::getRobotState() {
    return robot_state_;
}

// 路径规划
bool PathPlanner::planPath() {
    if (current_waypoint_count_ == 0) {
        return false;
    }
    
    current_target_index_ = 0;
    return true;
}

// 执行一步路径跟踪
// dt_seconds: 本次步骤经过的时间（秒），必须由调用者提供
void PathPlanner::executeOneStep(float dt_seconds) {
    if (current_waypoint_count_ == 0 || current_target_index_ >= current_waypoint_count_) {
        return;
    }
    
    Waypoint current_target = waypoints_[current_target_index_];
    
    // 检查是否到达当前目标点
    if (isGoalReached(current_target.x, current_target.y)) {
        current_target_index_++;
        
        if (current_target_index_ >= current_waypoint_count_) {
            // 到达最终目标，停止
            robot_state_.linear_velocity = 0.0f;
            robot_state_.angular_velocity = 0.0f;
            return;
        }
        
        current_target = waypoints_[current_target_index_];
    }
    
    // 使用Pure Pursuit算法计算控制命令（该函数会设置 robot_state_.angular_velocity）
    purePursuitControl(current_target.x, current_target.y);

    // 更新线速度（基于 dt_seconds）
    float desired_linear_vel = config_.max_linear_velocity;

    // 根据角速度调整线速度
    if (fabsf(robot_state_.angular_velocity) > config_.max_angular_velocity * 0.5f) {
        desired_linear_vel *= 0.7f; // 转弯时减速
    }

    // 平滑加速 / 减速，基于真实 dt
    if (desired_linear_vel > robot_state_.linear_velocity) {
        robot_state_.linear_velocity += config_.linear_acceleration * dt_seconds;
        if (robot_state_.linear_velocity > desired_linear_vel) {
            robot_state_.linear_velocity = desired_linear_vel;
        }
    } else if (desired_linear_vel < robot_state_.linear_velocity) {
        robot_state_.linear_velocity -= config_.linear_acceleration * dt_seconds;
        if (robot_state_.linear_velocity < desired_linear_vel) {
            robot_state_.linear_velocity = desired_linear_vel;
        }
    }
}

// 检查路径是否完成
bool PathPlanner::isPathCompleted() {
    return current_target_index_ >= current_waypoint_count_;
}

// 计算运动控制命令
void PathPlanner::calculateMotionCommands(float* linear_vel, float* angular_vel) {
    if (linear_vel != nullptr) {
        *linear_vel = robot_state_.linear_velocity;
    }
    if (angular_vel != nullptr) {
        *angular_vel = robot_state_.angular_velocity;
    }
}

// 获取当前目标点
Waypoint PathPlanner::getCurrentTarget() {
    if (current_target_index_ < current_waypoint_count_) {
        return waypoints_[current_target_index_];
    } else if (current_waypoint_count_ > 0) {
        return waypoints_[current_waypoint_count_ - 1];
    } else {
        Waypoint empty = {0, 0, 0};
        return empty;
    }
}

// 计算路径总长度
float PathPlanner::getPathLength() {
    float total_length = 0.0f;
    
    for (unsigned int i = 1; i < current_waypoint_count_; i++) {
        total_length += calculateDistance(waypoints_[i-1].x, waypoints_[i-1].y,
                                         waypoints_[i].x, waypoints_[i].y);
    }
    
    return total_length;
}

// 注入外部缓冲区，避免内部动态分配
bool PathPlanner::init(Waypoint* buffer, unsigned int max_points) {
    if (buffer == nullptr || max_points == 0) return false;
    waypoints_ = buffer;
    max_waypoints_ = max_points;
    current_waypoint_count_ = 0;
    current_target_index_ = 0;
    return true;
}