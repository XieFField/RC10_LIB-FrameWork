#ifndef __APP_PATH_PLANNER_H
#define __APP_PATH_PLANNER_H

#pragma once

#ifdef __cplusplus
#include "Robot_Arm.h"
#include <cstdint>
#include <stdint.h>  

extern "C" {
    #define M_PI 3.14159265358979323846f  // 定义圆周率常量
}


/**
 * @brief 最大路径点数
 */
#define MAX_PATH_POINTS 50

/**
 * @brief 路径点类型
 */
typedef struct {
    Arm_Point_S point;    // 路径点位置
    float velocity;       // 该点速度（保留参数）
    uint32_t dwell_time;  // 在该点的停留时间（毫秒）
} PathPoint_S;

/**
 * @brief 路径规划模式
 */
typedef enum {
    LINEAR_INTERPOLATION,  // 直线插补
    CIRCULAR_INTERPOLATION // 圆弧插补（待实现）
} PathPlanningMode_E;

/**
 * @brief 路径规划器状态
 */
typedef enum {
    PLANNER_IDLE,      // 空闲状态
    PLANNER_RUNNING,   // 运行中
    PLANNER_PAUSED,    // 暂停
    PLANNER_FINISHED   // 完成
} PlannerStatus_E;

/**
 * @brief 路径规划器类
 */
class Path_Planner {
public:
    Path_Planner(Robot_Arm* robot_arm);
    ~Path_Planner(){}

    /**
     * @brief 添加单个路径点
     * @return true: 添加成功, false: 路径点已满
     */
    bool addPathPoint(const Arm_Point_S& point, float velocity = 0, uint32_t dwell_time = 0);
    
    /**
     * @brief 清空路径点
     */
    void clearPathPoints();
    
    /**
     * @brief 设置路径规划模式
     */
    void setPlanningMode(PathPlanningMode_E mode) { planning_mode_ = mode; }
    
    /**
     * @brief 设置插补步长（米）
     */
    void setInterpolationStep(float step) { interpolation_step_ = step; }
    
    /**
     * @brief 开始路径规划执行
     */
    void start();
    
    /**
     * @brief 暂停路径规划
     */
    void pause();
    
    /**
     * @brief 继续路径规划
     */
    void resume();
    
    /**
     * @brief 停止路径规划
     */
    void stop();
    
    /**
     * @brief 更新路径规划器状态
     * @note 需要在主循环中定期调用
     */
    // update should be called regularly; dt_seconds is elapsed seconds since last call
    void update(float dt_seconds);
    // 默认速度（m/s），当 PathPoint_S.velocity <= 0 时使用默认值
    void setDefaultVelocity(float v) { default_velocity_ = v; }
    
    /**
     * @brief 获取规划器状态
     */
    PlannerStatus_E getStatus() const { return status_; }
    
    /**
     * @brief 检查是否完成
     */
    bool isFinished() const { return status_ == PLANNER_FINISHED; }
    
    /**
     * @brief 获取当前路径点索引
     */
    uint32_t getCurrentPointIndex() const { return current_point_index_; }
    
    /**
     * @brief 获取总路径点数
     */
    uint32_t getTotalPoints() const { return path_point_count_; }
    
    /**
     * @brief 检查路径点是否已满
     */
    bool isPathFull() const { return path_point_count_ >= MAX_PATH_POINTS; }

private:
    Robot_Arm* robot_arm_;                    // 机械臂对象指针
    PathPoint_S path_points_[MAX_PATH_POINTS]; // 路径点数组
    uint32_t path_point_count_ = 0;           // 路径点数量
    PathPlanningMode_E planning_mode_ = LINEAR_INTERPOLATION; // 规划模式
    PlannerStatus_E status_ = PLANNER_IDLE;   // 规划器状态
    
    uint32_t current_point_index_ = 0;        // 当前路径点索引
    uint32_t next_point_index_ = 1;           // 下一个路径点索引
    float interpolation_step_ = 0.01f;        // 插补步长（米）
    float current_interp_param_ = 0.0f;       // 当前插补参数 [0,1]
    float default_velocity_ = 0.05f;          // 默认速度 m/s,还没用上速度规划,后面会改
    
    uint64_t dwell_start_time_ms_ = 0;        // 停留开始时间（ms，基于 RTOS tick）
    bool is_dwelling_ = false;                // 是否正在停留
    
    /**
     * @brief 直线插补计算
     */
    Arm_Point_S linearInterpolation(const Arm_Point_S& start, const Arm_Point_S& end, float t);
    
    /**
     * @brief 计算两点间距离
     */
    float calculateDistance(const Arm_Point_S& p1, const Arm_Point_S& p2);
    
    /**
     * @brief 检查是否到达目标点
     */
    bool isPointReached(const Arm_Point_S& target);
};

// Path_Planner使用的辅助函数（圆弧插补相关）
// 在此声明，实现在.cpp文件中
float normalize_angle(float a);  // 角度归一化到[-π, π)
bool circularInterpolation3(const Arm_Point_S& p1, const Arm_Point_S& p2, const Arm_Point_S& p3, float t, Arm_Point_S& out);  // 三点圆弧插补
float arc_length_between_three(const Arm_Point_S& p1, const Arm_Point_S& p2, const Arm_Point_S& p3);  // 计算三点间圆弧长度

#endif // __cplusplus

#endif // __APP_PATH_PLANNER_H