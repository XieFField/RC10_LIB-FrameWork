/**
 * @file APP_Speedplanner.h
 * @author naoganlin
 * @brief 速度控制器
 * @version 0.20
 * @date 2025-09-30
 */

#ifndef __APP_SPEEDPLANNER_H
#define __APP_SPEEDPLANNER_H

#pragma once

#ifdef __cplusplus

extern "C"
{

#include "stm32h7xx_hal.h"
}
#include "APP_tool.h"
#include "arm_math.h" // 使用 ARM CMSIS DSP 库
#include <cmath>      // 用于数学运算，如 sqrt, pow 等

// 定义运动阶段枚举
/**
 * @brief 定义运动阶段枚举
 */
enum SPhase
{
    S_ACCEL_JERK_UP_PHASE,   // 加速段：加加速度（Jerk）从0增加到最大值
    S_ACCEL_CONST_PHASE,     // 加速段：加速度保持恒定在最大值
    S_ACCEL_JERK_DOWN_PHASE, // 加速段：加加速度（Jerk）从最大值减小到0
    S_CONST_VEL_PHASE,       // 匀速段：速度保持恒定在最大值
    S_DECEL_JERK_UP_PHASE,   // 减速段：加加速度（Jerk）从0增加到最大负值（开始减速）
    S_DECEL_CONST_PHASE,     // 减速段：加速度保持恒定在最大负值
    S_DECEL_JERK_DOWN_PHASE, // 减速段：加加速度（Jerk）从最大负值减小到0（减速结束）
    S_FINISHED_PHASE         // 规划完成阶段
};

/**
 * @brief 定义运动阶段枚举，新增 PID_PHASE 表示进入 PID 点追踪控制
 */
enum Phase
{
    ACCEL_PHASE,   // 加速段
    CONST_PHASE,   // 匀速段
    DECEL_PHASE,   // 减速段
    PID_PHASE,     // PID 控制段（接近目标点）
    FINISHED_PHASE // 规划结束
};

/**
 * @brief 定义规划类型枚举
 */
enum ProfileType
{
    TRAPEZOIDAL, // 梯形规划：存在加速、匀速、减速三个阶段
    TRIANGULAR   // 三角形规划：仅有加速和减速两个阶段，无法达到设定的最大速度
};

/**
 * @brief 定义速度规划参数结构体，用于初始化规划器的参数
 */
typedef struct
{
    float maxAcc;       // 最大加速度（正值）
    float maxDec;       // 最大减速度（正值）
    float maxJerk;      // 最大加加速度（正值）
    float maxSpeed;     // 最大允许速度
    float initialSpeed; // 起始时的速度
    float finalSpeed;   // 目标点的速度
    float startPos;     // 起始位置
    float targetPos;    // 目标位置
    float deadzone;     // 死区范围（小于该范围视为到达目标点）
} Speedplanner_Param_Config;

/**
 * @brief 基类：一维速度规划器
 */
class Speedplanner1D_Base
{
public:
    /**
     * @brief 根据当前已行驶的距离，规划目标速度
     * @param now_dis 当前已行驶的距离
     * @return 规划的目标速度
     */
    virtual float plan(float now_dis) = 0;

    /**
     * @brief 判断规划是否完成
     * @return 如果规划完成返回 true，否则返回 false
     */
    virtual bool isFinished() = 0;

    /**
     * @brief 重置规划器状态
     */
    virtual void reset() = 0;

    /**
     * @brief 重置规划器参数
     * @param params 速度规划参数
     */
    virtual void param_reset(Speedplanner_Param_Config params) = 0;

protected:
    // 规划参数
    float m_maxAcc_ = 0.0f;        // 最大加速度
    float m_maxDec_ = 0.0f;        // 最大减速度
    float m_maxJerk_ = 0.0f;       // 最大加加速度
    float m_maxSpeed_ = 0.0f;      // 最大速度
    float m_initialSpeed_ = 0.0f;  // 起始速度
    float m_finalSpeed_ = 0.0f;    // 目标速度
    float m_startPos_ = 0.0f;      // 起始位置
    float m_targetPos_ = 0.0f;     // 目标位置
    float m_totalDistance_ = 0.0f; // 总路程
    float m_deadzone_ = 0.0f;      // 死区范围
    float traveled_ = 0.0f;        // 已行驶路程
};

/**
 * @brief 派生类：梯形速度规划器
 */
class TrapePlanner1D : public Speedplanner1D_Base
{
public:
    /**
     * @brief 构造函数：初始化规划器参数
     * @param params 速度规划参数，默认为零
     */
    TrapePlanner1D(Speedplanner_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});

    /**
     * @brief 根据当前已行驶的距离，规划目标速度
     * @param now_dis 当前已行驶的距离
     * @return 规划的目标速度
     */
    float plan(float now_dis);

    /**
     * @brief 判断规划是否完成
     * @return 如果规划完成返回 true，否则返回 false
     */
    bool isFinished() { return m_phase == FINISHED_PHASE; }

    /**
     * @brief 重置规划器状态
     */
    void reset(void);

    /**
     * @brief 重置规划器参数
     * @param params 速度规划参数
     */
    void param_reset(Speedplanner_Param_Config params);

    /**
     * @brief 根据当前已行驶距离判断处于哪个阶段
     * @param traveled 已行驶的距离
     * @return 当前阶段
     */
    Phase determinePhase(float traveled);

    /**
     * @brief 获取当前阶段
     * @return 当前阶段
     */
    Phase getPhase() const { return m_phase; }

protected:
    // 内部状态
    Phase m_phase = FINISHED_PHASE; // 当前阶段
    // 各阶段路程
    float m_accelDistance_ = 0.0f; // 加速段长度
    float m_decelDistance_ = 0.0f; // 减速段长度
    float direction_ = 0.0f;       // 运动方向
    float min_dead_speed_ = 0.0f;  // 最小死区速度
    float v_target_ = 0.0f;        // 目标速度
};

/**
 * @brief S 型速度规划器
 */
class SShapedPlanner1D : public Speedplanner1D_Base
{
public:
    /**
     * @brief 构造函数：初始化所有成员变量为零或默认值
     * @param params 速度规划参数，默认为零
     */
    SShapedPlanner1D(Speedplanner_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});

    /**
     * @brief 根据当前已行驶的距离，规划目标速度
     * @param now_dis 当前已行驶的距离
     * @return 规划的目标速度
     */
    float plan(float now_dis);

    /**
     * @brief 判断规划是否已完成
     * @return 如果规划已完成则返回 true，否则返回 false
     */
    bool isFinished() const { return m_phase == S_FINISHED_PHASE; }

    /**
     * @brief 重置规划器状态
     */
    void reset(void);

    /**
     * @brief 重置规划器参数
     * @param params 速度规划参数
     */
    void param_reset(Speedplanner_Param_Config params);

    /**
     * @brief 获取当前规划所处的阶段
     * @return 当前 S 型规划阶段
     */
    SPhase getPhase() const { return m_phase; }

    /**
     * @brief 根据已行驶的距离确定当前所处的 S 型规划阶段
     * @param traveled 已行驶的距离（从起始位置算起）
     * @return 当前 S 型规划阶段
     */
    SPhase determinePhase(float traveled);

private:
    // 内部状态变量
    SPhase m_phase = S_FINISHED_PHASE; // 当前规划所处的阶段

    // 预计算的 S 型规划各个阶段的距离
    float m_accelJerkUpDistance_ = 0.0f;   // 加速段：Jerk 上升阶段的路程
    float m_accelConstDistance_ = 0.0f;    // 加速段：加速度恒定阶段的路程
    float m_accelJerkDownDistance_ = 0.0f; // 加速段：Jerk 下降阶段的路程
    float m_constVelDistance_ = 0.0f;      // 匀速段：恒定速度阶段的路程
    float m_decelJerkUpDistance_ = 0.0f;   // 减速段：Jerk 上升（减速开始）阶段的路程
    float m_decelConstDistance_ = 0.0f;    // 减速段：加速度恒定（减速中）阶段的路程
    float m_decelJerkDownDistance_ = 0.0f; // 减速段：Jerk 下降（减速结束）阶段的路程

    /**
     * @brief 预计算各阶段的距离
     */
    void cal_PhaseDistances();

    /**
     * @brief 加速段：Jerk 上升阶段的速度
     * @param traveled 已行驶距离
     * @return 当前速度
     */
    float cal_Acc_JerkUpSpeed(float traveled);

    /**
     * @brief 加速段：加速度恒定阶段的速度
     * @param traveled 已行驶距离
     * @return 当前速度
     */
    float cal_Acc_ConstSpeed(float traveled);

    /**
     * @brief 加速段：Jerk 下降阶段的速度
     * @param traveled 已行驶距离
     * @return 当前速度
     */
    float cal_Acc_JerkDownSpeed(float traveled);

    /**
     * @brief 减速段：Jerk 上升阶段的速度
     * @param traveled 已行驶距离
     * @return 当前速度
     */
    float cal_Dec_JerkUpSpeed(float traveled);

    /**
     * @brief 减速段：加速度恒定阶段的速度
     * @param traveled 已行驶距离
     * @return 当前速度
     */
    float cal_Dec_ConstSpeed(float traveled);

    /**
     * @brief 减速段：Jerk 下降阶段的速度
     * @param traveled 已行驶距离
     * @return 当前速度
     */
    float cal_Dec_JerkDownSpeed(float traveled);
};

#endif

#endif