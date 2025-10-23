/**
 * @file APP_Speedplanner.h
 * @author naoganlin
 * @brief 速度控制器(s型暂不可用)
 * @version 0.40
 * @date 2025-10-13
 */

#ifndef __APP_SPEEDPLANNER_H
#define __APP_SPEEDPLANNER_H

#pragma once

#ifdef __cplusplus

extern "C"
{
}
#include "stm32h7xx_hal.h" // STM32 HAL 库头文件
#include "APP_tool.h"      // 工具类头文件，包含通用工具函数
#include "APP_Vector2D.h"  // 二维向量类头文件，定义了 Vector2D 类型
#include "arm_math.h"      // ARM CMSIS DSP 库，用于数学运算优化
#include <cmath>           // 标准数学库，用于 sqrt, pow 等函数

/**
 * @brief 运动阶段枚举，包含PID点追踪控制
 *
 * 用于描述梯形或三角形速度曲线的各个阶段。
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
 * @brief 速度规划类型枚举
 *
 * 用于区分梯形和三角形速度曲线。
 */
enum ProfileType
{
    TRAPEZOIDAL, // 梯形规划：存在加速、匀速、减速三个阶段
    TRIANGULAR   // 三角形规划：仅有加速和减速两个阶段，无法达到设定的最大速度
};

/**
 * @brief 一维速度规划参数结构体
 *
 * 用于初始化一维速度规划器的参数。
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
} Speedplanner_1D_Param_Config;

/**
 * @brief 一维TD平滑器
 *
 * 用于对输入信号进行平滑处理，R参数越小越平滑，越大越灵敏。
 */
class  
{
public:
    /**
     * @brief 构造函数，设置R参数
     * @param td_r_ R参数，越小越平滑
     */
    Td(float td_r_ = 0.0f);

    /**
     * @brief 设置R参数
     * @param td_r_ R参数
     */
    void set_R(float td_r_); // R越小越平滑。越大越猛

    /**
     * @brief TD平滑函数
     * @param input_expect 期望输入
     * @return 平滑输出
     */
    float plan(float input_expect);

private:
    float r_ = 0.0f;             // TD平滑参数R
    float V1_ = 0.0f;            // 内部状态变量1
    float V2_ = 0.0f;            // 内部状态变量2
    float fh_ = 0.0f;            // 辅助变量
    float expect_ = 0.0f;        // 期望值
    float Ts_ = 0.0f;            // 采样周期
    uint32_t previous_time_ = 0; // 上一次调用的时间戳
};

/**
 * @brief 一维速度规划器基类（抽象类）
 *
 * 提供速度规划的基本接口，所有一维速度规划器的派生类都需要实现这些接口。
 */
class Speedplanner1D_Base
{
public:
    /**
     * @brief 规划目标速度（纯虚函数）
     * @param now_dis 当前已行驶的距离
     * @return 规划的目标速度
     */
    virtual float plan(float now_dis) = 0;

    /**
     * @brief 判断规划是否完成（纯虚函数）
     * @return 如果规划完成返回 true，否则返回 false
     */
    virtual bool isFinished() = 0;

    /**
     * @brief 重置规划器状态（纯虚函数）
     */
    virtual void reset() = 0;
};

/**
 * @brief 梯形速度规划器（派生类）
 *
 * 实现一维梯形速度曲线的规划。
 */
class TrapePlanner1D : public Speedplanner1D_Base
{
public:
    /**
     * @brief 构造函数：初始化规划器参数
     * @param params 速度规划参数，默认为零
     */
    TrapePlanner1D(Speedplanner_1D_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});

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
    void param_reset(Speedplanner_1D_Param_Config params);

    /**
     * @brief 判断当前阶段
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
    // 规划参数(原基类protected)
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
 * @brief S型速度规划器（派生类）
 *
 * 实现一维S型速度曲线的规划。
 */
class TdPlanner1D : public Speedplanner1D_Base
{
public:
    /**
     * @brief 构造函数：初始化所有成员变量为零或默认值
     * @param params 速度规划参数，默认为零
     */
    TdPlanner1D(Speedplanner_1D_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, float td_r = 0.0f) : trapeplanner_(params), td_(td_r)
    {
    }

    /**
     * @brief 规划目标速度
     * @param now_dis 当前已行驶的距离
     * @return 规划的目标速度
     */
    float plan(float now_dis)
    {
        // 先使用梯形规划器计算目标速度
        float v_trape = trapeplanner_.plan(now_dis);
        // 再使用TD平滑器对目标速度进行平滑处理
        float v_td = td_.plan(v_trape);
        return v_td;
    }

    /**
     * @brief 判断规划是否已完成
     * @return 如果规划已完成则返回 true，否则返回 false
     */
    bool isFinished() { return trapeplanner_.isFinished(); }

    /**
     * @brief 重置规划器状态
     */
    void reset(void) { trapeplanner_.reset(); }

    /**
     * @brief 重置规划器参数
     * @param params 速度规划参数
     */
    void param_reset(Speedplanner_1D_Param_Config params, float td_r)
    {
        trapeplanner_.param_reset(params);
        td_.set_R(td_r);
    }

    /**
     * @brief 获取当前S型规划阶段
     * @return 当前 S 型规划阶段
     */
    Phase getPhase() const { return trapeplanner_.getPhase(); }

    /**
     * @brief 判断当前S型阶段
     * @param traveled 已行驶的距离
     * @return 当前 S 型规划阶段
     */
    Phase determinePhase(float traveled) { return trapeplanner_.determinePhase(traveled); }

private:
    TrapePlanner1D trapeplanner_; // 内部使用的梯形规划器
    Td td_;                       // 内部使用的td型规划器
};

///////////////////////////////    2D 版本     //////////////////////////

/**
 * @brief 二维速度规划参数结构体
 *
 * 用于初始化二维速度规划器的参数。
 */
typedef struct
{
    float maxAcc;       // 最大加速度（正值）
    float maxDec;       // 最大减速度（正值）
    float maxJerk;      // 最大加加速度（正值）
    float maxSpeed;     // 最大允许速度
    float initialSpeed; // 起始时的速度
    float finalSpeed;   // 目标点的速度
    Vector2D startPos;  // 起始位置
    Vector2D targetPos; // 目标位置
    float deadzone;     // 死区范围（小于该范围视为到达目标点）
} Speedplanner_2D_Param_Config;

/**
 * @brief 基类：二维速度规划器
 */
/**
 * @brief 二维速度规划器基类（抽象类）
 *
 * 提供速度规划的基本接口，所有二维速度规划器的派生类都需要实现这些接口。
 */
class Speedplanner2D_Base
{
public:
    /**
     * @brief 规划目标速度（纯虚函数）
     * @param now_dis 当前已行驶的距离
     * @return 规划的目标速度
     */
    virtual Vector2D plan(const Vector2D &now_dis) = 0;

    /**
     * @brief 判断规划是否完成（纯虚函数）
     * @return 如果规划完成返回 true，否则返回 false
     */
    virtual bool isFinished() = 0;

    /**
     * @brief 重置规划器状态（纯虚函数）
     */
    virtual void reset() = 0;

    /**
     * @brief 重置规划器参数（纯虚函数）
     * @param params 速度规划参数
     */
    virtual void param_reset(Speedplanner_2D_Param_Config params) = 0;

protected:
};

/**
 * @brief 二维梯形速度规划器（派生类）
 *
 * 实现二维梯形速度曲线的规划。
 */
class TrapePlanner2D : public Speedplanner2D_Base
{
public:
    /**
     * @brief 构造函数：初始化规划器参数
     * @param params 速度规划参数，默认为零
     */
    TrapePlanner2D(Speedplanner_2D_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f});

    /**
     * @brief 规划目标速度
     * @param now_dis 当前已行驶的距离
     * @return 规划的目标速度
     */
    Vector2D plan(const Vector2D &now_dis);

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
    void param_reset(Speedplanner_2D_Param_Config params);

    /**
     * @brief 判断当前阶段
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
    // 规划参数(原基类protected)
    float m_maxAcc_ = 0.0f;                       // 最大加速度
    float m_maxDec_ = 0.0f;                       // 最大减速度
    float m_maxJerk_ = 0.0f;                      // 最大加加速度
    float m_maxSpeed_ = 0.0f;                     // 最大速度
    float m_initialSpeed_ = 0.0f;                 // 起始速度
    float m_finalSpeed_ = 0.0f;                   // 目标速度
    Vector2D m_startPos_ = Vector2D(0.0f, 0.0f);  // 起始位置
    Vector2D m_targetPos_ = Vector2D(0.0f, 0.0f); // 目标位置
    float m_totalDistance_ = 0.0f;                // 总路程
    float m_deadzone_ = 0.0f;                     // 死区范围
    float traveled_ = 0.0f;                       // 已行驶路程

    // 内部状态
    ProfileType m_profileType;
    Phase m_phase = FINISHED_PHASE; // 当前阶段
    // 各阶段路程
    float m_accelDistance_ = 0.0f; // 加速段长度
    float m_decelDistance_ = 0.0f; // 减速段长度

    float direction_ = 0.0f;      // 运动方向
    float min_dead_speed_ = 0.0f; // 最小死区速度
    float v_target_ = 0.0f;       // 目标速度
};

class TdPlanner2D : public Speedplanner2D_Base
{
public:
    /**
     * @brief 构造函数：初始化所有成员变量为零或默认值
     * @param params 速度规划参数，默认为零
     */
    TdPlanner2D(Speedplanner_2D_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f}, {0.0f, 0.0f}, 0.0f}, float td_r = 0.0f) : trapeplanner_(params), td_(td_r)
    {
    }

    /**
     * @brief 规划目标速度
     * @param now_dis 当前已行驶的距离
     * @return 规划的目标速度
     */
    Vector2D plan(const Vector2D &now_dis)
    {
        // 先使用梯形规划器计算目标速度
        Vector2D v_trape = trapeplanner_.plan(now_dis);
        // 再使用TD平滑器对目标速度进行平滑处理
        Vector2D v_td;
        v_td.x = td_.plan(v_trape.x);
        v_td.y = td_.plan(v_trape.y);
        return v_td;
    }

    /**
     * @brief 判断规划是否已完成
     * @return 如果规划已完成则返回 true，否则返回 false
     */
    bool isFinished() { return trapeplanner_.isFinished(); }

    /**
     * @brief 重置规划器状态
     */
    void reset(void) { trapeplanner_.reset(); }

    /**
     * @brief 重置规划器参数
     * @param params 速度规划参数
     */
    void param_reset(Speedplanner_2D_Param_Config params, float td_r)
    {
        trapeplanner_.param_reset(params);
        td_.set_R(td_r);
    }

    /**
     * @brief 获取当前S型规划阶段
     * @return 当前 S 型规划阶段
     */
    Phase getPhase() const { return trapeplanner_.getPhase(); }

    /**
     * @brief 判断当前S型阶段
     * @param traveled 已行驶的距离
     * @return 当前 S 型规划阶段
     */
    Phase determinePhase(float traveled) { return trapeplanner_.determinePhase(traveled); }

private:
    TrapePlanner2D trapeplanner_; // 内部使用的梯形规划器
    Td td_;                       // 内部使用的td型规划器
};

/**
 * @brief 一维恒加速度平滑器
 *
 * 用于将目标速度平滑地调整到期望速度，限制加速度变化。
 */
class ConstantAcc
{
public:
    /**
     * @brief 构造函数，传入加速度上限和初始速度
     * @param maxAcceleration 最大加速度（单位 m/s?）
     * @param initialValue 初始速度（单位 m/s）
     */
    ConstantAcc(float maxAcceleration = 0.0f, float initialValue = 0.0f);

    /**
     * @brief 规划函数：传入目标速度，返回平滑输出速度
     * @param targetSpeed 目标速度
     * @return 平滑后的速度
     */
    float plan(float targetSpeed);

    /**
     * @brief 设置新的加速度上限
     * @param acceleration 最大加速度（单位 m/s?）
     */
    void setMaxAcceleration(float acceleration);

    /**
     * @brief 重置规划器状态，可设置初始速度
     * @param maxAcceleration 最大加速度
     * @param initialValue 初始速度
     */
    void reset(float maxAcceleration = 0.0f, float initialValue = 0.0f);

    /**
     * @brief 仅重置速度为0
     */
    void reset_speed();

private:
    float maxAcceleration_; // 加速度上限（单位 m/s?）
    float lastOutput_;      // 上一次输出的速度（单位 m/s）
};


#endif

#endif