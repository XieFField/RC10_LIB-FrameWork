/**
 * @file APP_Speedplanner.h
 * @author naoganlin
 * @brief 速度控制器
 * @version 1.0
 * @date 2025-09-28
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

// 定义运动阶段枚举，新增 PID_PHASE 表示进入 PID 点追踪控制
enum Phase
{
    ACCEL_PHASE,   // 加速段
    CONST_PHASE,   // 匀速段
    DECEL_PHASE,   // 减速段
    PID_PHASE,     // PID 控制段（接近目标点）
    FINISHED_PHASE // 规划结束
};

// 定义规划类型枚举
enum ProfileType
{
    TRAPEZOIDAL, // 梯形规划：存在加速、匀速、减速三个阶段
    TRIANGULAR   // 三角形规划：仅有加速和减速两个阶段，无法达到设定的最大速度
};

// 定义速度规划参数结构体，用于初始化规划器的参数
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

// 基类：一维速度规划器
class Speedplanner1D_Base
{
public:
    // 纯虚函数：根据当前已行驶的距离，规划目标速度
    virtual float plan(float now_dis) = 0;
    // 纯虚函数：判断规划是否完成
    virtual bool isFinished() = 0;
    // 纯虚函数：重置规划器状态
    virtual void reset() = 0;
    // 重置规划器参数
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

// 派生类：梯形速度规划器
class TrapePlanner1D : public Speedplanner1D_Base
{
public:
    // 构造函数：初始化规划器参数
    TrapePlanner1D(Speedplanner_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});

    // 根据当前已行驶的距离，规划目标速度
    float plan(float now_dis);
    // 判断规划是否完成
    bool isFinished() { return m_phase == FINISHED_PHASE; }
    // 重置规划器状态
    void reset();
    // 重置规划器参数
    void param_reset(Speedplanner_Param_Config params);

    // 辅助接口：根据当前已行驶距离判断处于哪个阶段
    Phase determinePhase(float traveled);

    // 获取当前阶段
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

#endif

#endif