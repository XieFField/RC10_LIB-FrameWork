#include "APP_Path_Planner.h"
#include <cmath>

// 构造函数：初始化路径规划器，关联机械臂对象
Path_Planner::Path_Planner(Robot_Arm* robot_arm)
    : robot_arm_(robot_arm)
{
}

// 添加路径点到路径序列
bool Path_Planner::addPathPoint(const Arm_Point_S& point, float velocity, uint32_t dwell_time)
{
    if (path_point_count_ >= MAX_PATH_POINTS) {
        return false; // 路径点已满
    }
    
    PathPoint_S path_point;
    path_point.point = point;
    path_point.velocity = velocity;
    path_point.dwell_time = dwell_time;
    
    path_points_[path_point_count_] = path_point;
    path_point_count_++;
    
    return true;
}

// 清空所有路径点，重置规划器状态
void Path_Planner::clearPathPoints()
{
    path_point_count_ = 0;
    current_point_index_ = 0;
    next_point_index_ = 1;
    current_interp_param_ = 0.0f;
    is_dwelling_ = false;
    dwell_start_time_ms_ = 0;
}

// 开始路径规划执行
void Path_Planner::start()
{
    if (path_point_count_ < 2) {
        // 至少需要2个点才能规划路径
        return;
    }
    
    status_ = PLANNER_RUNNING;
    current_point_index_ = 0;
    next_point_index_ = 1;
    current_interp_param_ = 0.0f;
    is_dwelling_ = false;
    dwell_start_time_ms_ = 0;
    
    // 设置第一个点为起始位置
    robot_arm_->setArmTarget(path_points_[0].point);
}

// 暂停路径规划
void Path_Planner::pause()
{
    if (status_ == PLANNER_RUNNING) {
        status_ = PLANNER_PAUSED;
    }
}

// 继续路径规划
void Path_Planner::resume()
{
    if (status_ == PLANNER_PAUSED) {
        status_ = PLANNER_RUNNING;
    }
}

// 停止路径规划，重置所有状态
void Path_Planner::stop()
{
    status_ = PLANNER_IDLE;
    current_point_index_ = 0;
    next_point_index_ = 1;
    current_interp_param_ = 0.0f;
    is_dwelling_ = false;
    dwell_start_time_ms_ = 0;
}

// 获取RTOS tick的毫秒值（跨平台支持）
static uint64_t get_rtos_tick_ms()
{
#if defined(osKernelGetTickCount)
    return static_cast<uint64_t>(osKernelGetTickCount());
#elif defined(xTaskGetTickCount)
    return static_cast<uint64_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
#else
    return 0;
#endif
}

// 主更新函数：根据时间步长推进路径规划
void Path_Planner::update(float dt_seconds)
{
    if (status_ != PLANNER_RUNNING) {
        return;
    }
    
    if (path_point_count_ < 2) {
        status_ = PLANNER_FINISHED;
        return;
    }
    
    // 检查是否需要停留（使用 RTOS tick，dwell_time 单位为 ms）
    if (is_dwelling_) {
        uint64_t now_ms = get_rtos_tick_ms();
        // 强制要求 RTOS tick 可用
        if (now_ms == 0) {
            // 没有 RTOS tick 支持，直接退出（不进行插补）
            return;
        }
        if (now_ms - dwell_start_time_ms_ >= path_points_[current_point_index_].dwell_time) {
            is_dwelling_ = false;
            dwell_start_time_ms_ = 0;
        } else {
            return; // 还在停留中，不进行插补
        }
    }
    
    // 获取当前线段
    const Arm_Point_S& start_point = path_points_[current_point_index_].point;
    const Arm_Point_S& end_point = path_points_[next_point_index_].point;
    
    // 执行插补
    Arm_Point_S interp_point;
    
    switch (planning_mode_) {
        case LINEAR_INTERPOLATION:
            interp_point = linearInterpolation(start_point, end_point, current_interp_param_);
            break;

        case CIRCULAR_INTERPOLATION: {
            // 需要至少三个点：current_point, mid_point (next), next_point (next+1)
            if (next_point_index_ + 1 < path_point_count_) {
                const Arm_Point_S& mid_point = path_points_[next_point_index_].point;
                const Arm_Point_S& end_point2 = path_points_[next_point_index_ + 1].point;
                // compute total arc length
                float arc_len = arc_length_between_three(start_point, mid_point, end_point2);
                if (arc_len > 0.0f) {
                    // advance along arc based on velocity
                    float v = path_points_[current_point_index_].velocity;
                    if (v <= 0.0f) v = default_velocity_;
                    float delta_s = v * dt_seconds;
                    // use current_interp_param_ as fraction along this arc
                    float t = current_interp_param_;
                    float t_inc = delta_s / arc_len;
                    t += t_inc;
                    if (t > 1.0f) t = 1.0f;
                    Arm_Point_S arc_pt;
                    if (circularInterpolation3(start_point, mid_point, end_point2, t, arc_pt)) {
                        interp_point = arc_pt;
                        current_interp_param_ = t;
                    } else {
                        // fallback to linear if fails
                        interp_point = linearInterpolation(start_point, end_point, current_interp_param_);
                    }
                } else {
                    // fallback to linear
                    interp_point = linearInterpolation(start_point, end_point, current_interp_param_);
                }
            } else {
                // not enough points for circular, fallback to linear
                interp_point = linearInterpolation(start_point, end_point, current_interp_param_);
            }
        } break;
    }
    
    // 设置机械臂目标位置
    robot_arm_->setArmTarget(interp_point);
    
    // 更新插补参数：基于速度与时间推进（delta_s = v * dt）
    float distance = calculateDistance(start_point, end_point);
    if (distance > 0) {
        float v = path_points_[current_point_index_].velocity;
        if (v <= 0.0f) v = default_velocity_;
        float delta_s = v * dt_seconds; // 期望位移 (m)
        current_interp_param_ += delta_s / distance;
    } else {
        current_interp_param_ = 1.0f;
    }
    
    // 检查是否到达线段终点
    if (current_interp_param_ >= 1.0f) {
        current_interp_param_ = 0.0f;
        current_point_index_ = next_point_index_;
        next_point_index_++;
        
        // 检查是否需要停留
        if (path_points_[current_point_index_].dwell_time > 0) {
            is_dwelling_ = true;
            uint64_t now_ms = get_rtos_tick_ms();
            if (now_ms != 0) dwell_start_time_ms_ = now_ms;
            else {
                // 没有 RTOS tick 支持，标记为不支持并停止规划
                is_dwelling_ = false;
                status_ = PLANNER_PAUSED;
            }
        }
        
        // 检查是否到达路径终点
        if (next_point_index_ >= path_point_count_) {
            status_ = PLANNER_FINISHED;
        }
    }
}

// 线性插补计算：在起点和终点之间按比例t进行插值
Arm_Point_S Path_Planner::linearInterpolation(const Arm_Point_S& start, const Arm_Point_S& end, float t)
{
    Arm_Point_S result;
    
    // 限制t在[0,1]范围内
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    
    // 线性插补位置
    result.x = start.x + t * (end.x - start.x);
    result.y = start.y + t * (end.y - start.y);
    result.z = start.z + t * (end.z - start.z);
    
    // 线性插补关节角度
    result.suckerJoint_status_ = start.suckerJoint_status_ + t * (end.suckerJoint_status_ - start.suckerJoint_status_);
    
    return result;
}

// 计算两点间欧几里得距离
float Path_Planner::calculateDistance(const Arm_Point_S& p1, const Arm_Point_S& p2)
{
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float dz = p2.z - p1.z;
    
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

// 检查是否到达目标点（基于距离容差）
bool Path_Planner::isPointReached(const Arm_Point_S& target)
{
    // 简单的距离判断
    Arm_Point_S current;
    if (robot_arm_->forwardKinematics(current)) {
        float distance = calculateDistance(current, target);
        return distance < interpolation_step_ * 2.0f; // 容差为2倍步长
    }
    return false;
}

// 角度归一化：将角度限制在[-π, π)范围内
float normalize_angle(float a) {
    while (a <= -M_PI) a += 2.0f * M_PI;
    while (a > M_PI) a -= 2.0f * M_PI;
    return a;
}

// 三点圆弧插补：通过三个点计算圆弧插值点（在XY平面）
// 如果三点共线或无效，返回false
bool circularInterpolation3(const Arm_Point_S& p1, const Arm_Point_S& p2, const Arm_Point_S& p3, float t, Arm_Point_S& out)
{
    // 投影到XY平面
    float x1 = p1.x, y1 = p1.y;
    float x2 = p2.x, y2 = p2.y;
    float x3 = p3.x, y3 = p3.y;

    // 计算圆心和半径
    float d = 2.0f * (x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2));
    const float eps = 1e-6f;
    if (fabsf(d) < eps) return false; // 三点近似共线

    float x1s = x1*x1 + y1*y1;
    float x2s = x2*x2 + y2*y2;
    float x3s = x3*x3 + y3*y3;

    float ux = (x1s*(y2-y3) + x2s*(y3-y1) + x3s*(y1-y2)) / d;  // 圆心x坐标
    float uy = (x1s*(x3-x2) + x2s*(x1-x3) + x3s*(x2-x1)) / d;  // 圆心y坐标

    float r = sqrtf((x1-ux)*(x1-ux) + (y1-uy)*(y1-uy));  // 圆半径
    if (r <= eps) return false;

    // 计算三个点相对于圆心的角度
    float th1 = atan2f(y1-uy, x1-ux);
    float th2 = atan2f(y2-uy, x2-ux);
    float th3 = atan2f(y3-uy, x3-ux);

    // 计算从起点到终点的角度增量
    float d13 = normalize_angle(th3 - th1);
    float d12 = normalize_angle(th2 - th1);

    // 确保中间点在起点和终点之间的圆弧上
    if ((d12 > 0.0f && d13 < 0.0f) || (d12 < 0.0f && d13 > 0.0f) || (fabsf(d12) > fabsf(d13))) {
        // 通过加减2π翻转圆弧
        if (d13 > 0.0f) d13 -= 2.0f * M_PI;
        else d13 += 2.0f * M_PI;
    }

    // 根据参数t计算圆弧上的点
    float th = th1 + t * d13;
    float cx = ux + r * cosf(th);
    float cy = uy + r * sinf(th);

    // Z坐标和关节角度进行线性插补
    out.x = cx;
    out.y = cy;
    out.z = p1.z + t * (p3.z - p1.z);
    out.suckerJoint_status_ = p1.suckerJoint_status_ + t * (p3.suckerJoint_status_ - p1.suckerJoint_status_);
    return true;
}

// 计算三点间圆弧长度：失败时返回负值
float arc_length_between_three(const Arm_Point_S& p1, const Arm_Point_S& p2, const Arm_Point_S& p3)
{
    // 与圆弧插补相同的圆心和半径计算
    float x1 = p1.x, y1 = p1.y;
    float x2 = p2.x, y2 = p2.y;
    float x3 = p3.x, y3 = p3.y;
    float d = 2.0f * (x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2));
    const float eps = 1e-6f;
    if (fabsf(d) < eps) return -1.0f;
    float x1s = x1*x1 + y1*y1;
    float x2s = x2*x2 + y2*y2;
    float x3s = x3*x3 + y3*y3;
    float ux = (x1s*(y2-y3) + x2s*(y3-y1) + x3s*(y1-y2)) / d;
    float uy = (x1s*(x3-x2) + x2s*(x1-x3) + x3s*(x2-x1)) / d;
    float r = sqrtf((x1-ux)*(x1-ux) + (y1-uy)*(y1-uy));
    if (r <= eps) return -1.0f;
    
    // 计算角度并确定圆弧方向
    float th1 = atan2f(y1-uy, x1-ux);
    float th2 = atan2f(y2-uy, x2-ux);
    float th3 = atan2f(y3-uy, x3-ux);
    float d13 = normalize_angle(th3 - th1);
    float d12 = normalize_angle(th2 - th1);
    if ((d12 > 0.0f && d13 < 0.0f) || (d12 < 0.0f && d13 > 0.0f) || (fabsf(d12) > fabsf(d13))) {
        if (d13 > 0.0f) d13 -= 2.0f * M_PI;
        else d13 += 2.0f * M_PI;
    }
    
    // 圆弧长度 = 半径 × 角度（弧度）
    return fabsf(r * d13);
}