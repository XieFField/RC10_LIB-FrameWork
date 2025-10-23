#include "PathTracing.h"
#include <cmath>




// 默认构造函数 - 初始化所有成员变量
PathTracing::PathTracing() {
    waypoints_ = nullptr;
    max_waypoints_ = 0;
    current_waypoint_count_ = 0;
    current_target_index_ = 0;

    // 初始化机器人状态为零
    robot_state_.current_x = 0.0f;
    robot_state_.current_y = 0.0f;
    robot_state_.current_theta = 0.0f;
    robot_state_.linear_velocity = 0.0f;
    robot_state_.angular_velocity = 0.0f;
    last_angular_velocity_ = 0.0f;  // 初始化上一时刻角速度

    // 默认配置参数 - 适用于小型移动机器人
    config_.max_linear_velocity = 0.5f;
    config_.max_angular_velocity = 1.0f;
    config_.linear_acceleration = 0.1f;
    config_.angular_acceleration = 0.5f;
    config_.goal_tolerance = 0.05f;     // 5厘米容差
    config_.lookahead_distance = 0.3f;  // 30厘米前视距离
}

// 带最大路径点数的构造函数
PathTracing::PathTracing(unsigned int max_points) {
    waypoints_ = nullptr;
    max_waypoints_ = max_points;
    current_waypoint_count_ = 0;
    current_target_index_ = 0;

    robot_state_.current_x = 0.0f;
    robot_state_.current_y = 0.0f;
    robot_state_.current_theta = 0.0f;
    robot_state_.linear_velocity = 0.0f;
    robot_state_.angular_velocity = 0.0f;
    last_angular_velocity_ = 0.0f;

    config_.max_linear_velocity = 0.5f;
    config_.max_angular_velocity = 1.0f;
    config_.linear_acceleration = 0.1f;
    config_.angular_acceleration = 0.5f;
    config_.goal_tolerance = 0.05f;
    config_.lookahead_distance = 0.3f;
}

// 使用外部缓冲区的构造函数 - 避免动态内存分配
PathTracing::PathTracing(Waypoint* buffer, unsigned int max_points) {
    waypoints_ = buffer;
    max_waypoints_ = max_points;
    current_waypoint_count_ = 0;
    current_target_index_ = 0;

    robot_state_.current_x = 0.0f;
    robot_state_.current_y = 0.0f;
    robot_state_.current_theta = 0.0f;
    robot_state_.linear_velocity = 0.0f;
    robot_state_.angular_velocity = 0.0f;
    last_angular_velocity_ = 0.0f;

    config_.max_linear_velocity = 0.5f;
    config_.max_angular_velocity = 1.0f;
    config_.linear_acceleration = 0.1f;
    config_.angular_acceleration = 0.5f;
    config_.goal_tolerance = 0.05f;
    config_.lookahead_distance = 0.3f;
}

// 析构函数 - 仅重置指针，不释放外部管理的缓冲区
PathTracing::~PathTracing() {
    waypoints_ = nullptr;
}

// 计算两点之间的欧几里得距离
float PathTracing::calculateDistance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

// 角度归一化 - 将角度限制在[-π, π]范围内
float PathTracing::normalizeAngle(float angle) {
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

// 计算从当前位置到目标点的全局角度
float PathTracing::calculateAngleToTarget(float target_x, float target_y) {
    float dx = target_x - robot_state_.current_x;
    float dy = target_y - robot_state_.current_y;
    return atan2f(dy, dx);  // 返回相对于世界坐标系的角度
}

// 检查是否到达目标点 - 基于距离容差判断
bool PathTracing::isGoalReached(float target_x, float target_y) {
    float distance = calculateDistance(robot_state_.current_x, robot_state_.current_y,
                                      target_x, target_y);
    return distance <= config_.goal_tolerance;
}

// Pure Pursuit控制算法核心 - 计算跟踪目标点所需的角速度
void PathTracing::purePursuitControl(float target_x, float target_y) {
    // 计算到目标点的向量
    float dx = target_x - robot_state_.current_x;
    float dy = target_y - robot_state_.current_y;
    float distance_to_target = sqrtf(dx * dx + dy * dy);

    // 计算目标点在全局坐标系中的角度
    float target_global_angle = atan2f(dy, dx);
    // 计算角度误差（当前朝向与目标方向之间的差值）
    float alpha = normalizeAngle(target_global_angle - robot_state_.current_theta);

    if (distance_to_target > 0.001f) {
        // Pure Pursuit核心公式：曲率 = 2*sin(α)/L
        // 其中α是角度误差，L是到目标点的距离
        float curvature = 2.0f * sinf(alpha) / fmaxf(distance_to_target, 1e-6f);
        float desired_omega = robot_state_.linear_velocity * curvature;

        // 角速度限幅，确保不超过最大角速度
        if (desired_omega > config_.max_angular_velocity) desired_omega = config_.max_angular_velocity;
        if (desired_omega < -config_.max_angular_velocity) desired_omega = -config_.max_angular_velocity;
        robot_state_.angular_velocity = desired_omega;
    } else {
        robot_state_.angular_velocity = 0.0f;  // 非常接近目标点时停止转动
    }
}

// 添加路径点到路径序列中
bool PathTracing::addWaypoint(float x, float y, float theta) {
    if (waypoints_ == nullptr || max_waypoints_ == 0) return false;  // 缓冲区未初始化
    if (current_waypoint_count_ >= max_waypoints_) return false;     // 路径点已满

    // 存储路径点数据
    waypoints_[current_waypoint_count_].x = x;
    waypoints_[current_waypoint_count_].y = y;
    waypoints_[current_waypoint_count_].theta = theta;
    current_waypoint_count_++;
    return true;
}

// 清空所有路径点，重置跟踪状态
bool PathTracing::clearWaypoints() {
    current_waypoint_count_ = 0;
    current_target_index_ = 0;
    return true;
}

// 获取当前路径点数量
unsigned int PathTracing::getWaypointCount() {
    return current_waypoint_count_;
}

// 设置路径跟踪配置参数
void PathTracing::setConfig(float max_linear_vel, float max_angular_vel,
                            float linear_accel, float angular_accel,
                            float tolerance, float lookahead) {
    config_.max_linear_velocity = max_linear_vel;
    config_.max_angular_velocity = max_angular_vel;
    config_.linear_acceleration = linear_accel;
    config_.angular_acceleration = angular_accel;
    config_.goal_tolerance = tolerance;
    config_.lookahead_distance = lookahead;
}

// 获取当前配置
PathTracingConfig PathTracing::getConfig() {
    return config_;
}

// 设置机器人当前状态
void PathTracing::setRobotState(float x, float y, float theta) {
    robot_state_.current_x = x;
    robot_state_.current_y = y;
    robot_state_.current_theta = normalizeAngle(theta);  // 归一化角度
}

// 获取机器人当前状态
RobotState PathTracing::getRobotState() {
    return robot_state_;
}

// "规划"路径 - 实际是初始化跟踪状态，从第一个路径点开始跟踪
bool PathTracing::planPath() {
    if (current_waypoint_count_ == 0) return false;  // 没有路径点可用
    current_target_index_ = 0;  // 从第一个路径点开始跟踪
    return true;
}

// 执行一步路径跟踪控制 - 核心控制循环
void PathTracing::executeOneStep(float dt_seconds) {
    // 检查是否有路径点或是否已完成所有路径点
    if (current_waypoint_count_ == 0 || current_target_index_ >= current_waypoint_count_) return;

    Waypoint current_target = waypoints_[current_target_index_];
    
    // 检查是否到达当前目标点，如果到达则切换到下一个目标点
    if (isGoalReached(current_target.x, current_target.y)) {
        current_target_index_++;
        // 检查是否完成所有路径点
        if (current_target_index_ >= current_waypoint_count_) {
            robot_state_.linear_velocity = 0.0f;
            robot_state_.angular_velocity = 0.0f;
            return;  // 路径完成，停止运动
        }
        current_target = waypoints_[current_target_index_];  // 更新当前目标点
    }

    if (dt_seconds <= 0.0f) return;  // 无效时间步长

    // 前视点计算：沿路径从当前目标点向前寻找距离≥前视距离的点
    float lookahead = config_.lookahead_distance;
    float accum = 0.0f;
    uint32_t idx = current_target_index_;
    float lx = waypoints_[idx].x;
    float ly = waypoints_[idx].y;
    
    // 遍历路径点，累加距离直到达到前视距离
    while (idx + 1 < current_waypoint_count_ && accum < lookahead) {
        float seg = calculateDistance(waypoints_[idx].x, waypoints_[idx].y, 
                                     waypoints_[idx+1].x, waypoints_[idx+1].y);
        accum += seg;
        idx++;
        lx = waypoints_[idx].x;
        ly = waypoints_[idx].y;
    }

    // 基于距离最终目标的接近程度调整期望线速度（接近时减速）
    float dist_to_goal = calculateDistance(robot_state_.current_x, robot_state_.current_y, 
                                          waypoints_[current_waypoint_count_-1].x, 
                                          waypoints_[current_waypoint_count_-1].y);
    float desired_linear_vel = config_.max_linear_velocity;
    
    // 接近最终目标时减速
    if (dist_to_goal < config_.goal_tolerance * 10.0f) {
        desired_linear_vel = config_.max_linear_velocity * 0.3f;
    }

    // 根据当前角速度与最大角速度的比例进一步调整线速度（转弯时减速）
    if (fabsf(robot_state_.angular_velocity) > config_.max_angular_velocity * 0.5f) {
        desired_linear_vel *= 0.7f;
    }

    // 线速度加速度限制 - 确保速度平滑变化
    if (desired_linear_vel > robot_state_.linear_velocity) {
        robot_state_.linear_velocity += config_.linear_acceleration * dt_seconds;
        if (robot_state_.linear_velocity > desired_linear_vel) robot_state_.linear_velocity = desired_linear_vel;
    } else if (desired_linear_vel < robot_state_.linear_velocity) {
        robot_state_.linear_velocity -= config_.linear_acceleration * dt_seconds;
        if (robot_state_.linear_velocity < desired_linear_vel) robot_state_.linear_velocity = desired_linear_vel;
    }

    // 使用前视点计算Pure Pursuit角速度
    purePursuitControl(lx, ly);

    // 角速度变化率（加速度）限制 - 确保角速度平滑变化，避免急转
    {
        float max_omega_delta = config_.angular_acceleration * dt_seconds;
        float target_omega = robot_state_.angular_velocity;  // Pure Pursuit计算的目标角速度
        float delta = target_omega - last_angular_velocity_; // 角速度变化量
        
        // 限制角速度变化率
        if (delta > max_omega_delta) delta = max_omega_delta;
        if (delta < -max_omega_delta) delta = -max_omega_delta;
        
        robot_state_.angular_velocity = last_angular_velocity_ + delta;  // 应用限制后的角速度
        last_angular_velocity_ = robot_state_.angular_velocity;          // 更新上一时刻角速度
    }
}

// 检查路径是否已完成跟踪
bool PathTracing::isPathCompleted() {
    return current_target_index_ >= current_waypoint_count_;
}

// 获取当前运动控制命令
void PathTracing::calculateMotionCommands(float* linear_vel, float* angular_vel) {
    if (linear_vel) *linear_vel = robot_state_.linear_velocity;
    if (angular_vel) *angular_vel = robot_state_.angular_velocity;
}

// 获取当前跟踪的目标点
Waypoint PathTracing::getCurrentTarget() {
    if (current_target_index_ < current_waypoint_count_)
        return waypoints_[current_target_index_];
    else if (current_waypoint_count_ > 0)
        return waypoints_[current_waypoint_count_ - 1];  // 返回最后一个点
    return Waypoint{0, 0, 0};  // 默认空点
}

// 计算路径总长度 - 累加所有路径段长度
float PathTracing::getPathLength() {
    float total = 0.0f;
    for (unsigned int i = 1; i < current_waypoint_count_; ++i)
        total += calculateDistance(waypoints_[i - 1].x, waypoints_[i - 1].y,
                                   waypoints_[i].x, waypoints_[i].y);
    return total;
}

// 初始化路径跟踪器 - 设置外部提供的路径点缓冲区
bool PathTracing::init(Waypoint* buffer, unsigned int max_points) {
    if (!buffer || !max_points) return false;  // 缓冲区无效
    waypoints_ = buffer;
    max_waypoints_ = max_points;
    current_waypoint_count_ = 0;
    current_target_index_ = 0;
    return true;
}