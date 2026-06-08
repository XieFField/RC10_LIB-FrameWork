/**
 * @file APP_Speedplanner.h
 * @author naoganlin
 * @brief 速度控制器
 * 1.只有一维的梯形s型测过，二维的没应该场景属于打发时间的产物
 * @version 3.0
 * @date 2026-6-8
 */

#ifndef __APP_SPEEDPLANNER_H
#define __APP_SPEEDPLANNER_H
#include <arm_math.h>
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
 * @brief S型速度规划的运动阶段枚举
 *
 * 用于描述S型速度曲线的各个阶段。
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

typedef struct
{
    float output_limit; // 输出限幅
    float r;            // 快速跟踪因子
    float h;            // 滤波因子，系统调用步长
    float b;            // 系统系数
    float delta;        // fal函数的线性区间宽度
    float beta_01;      // 扩张状态观测器反馈增益1
    float beta_02;      // 扩张状态观测器反馈增益2
    float beta_03;      // 扩张状态观测器反馈增益3
    float alpha_1;      // 非线性因子1
    float alpha_2;      // 非线性因子2
    float beta_1;       // 跟踪输入信号增益1
    float beta_2;       // 跟踪微分信号增益2
} ADRC_Param_Config;

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

    /**
     * @brief 重置规划器参数（纯虚函数）
     * @param params 速度规划参数
     */
    virtual void param_reset(Speedplanner_1D_Param_Config params) = 0;

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
    float m_deadzone_ = 0.00001f;  // 死区范围
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
    TrapePlanner1D(Speedplanner_1D_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.00001f});

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
    // 内部状态
    Phase m_phase = FINISHED_PHASE; // 当前阶段
    // 各阶段路程
    float m_accelDistance_ = 0.0f; // 加速段长度
    float m_decelDistance_ = 0.0f; // 减速段长度
    float direction_ = 0.0f;       // 运动方向
    float min_dead_speed_ = 0.0f;  // 最小死区速度
    float v_target_ = 0.0f;        // 目标速度
};

///**
// * @brief S型速度规划器（派生类）
// *
// * 实现一维S型速度曲线的规划。
// */
//class SShapedPlanner1D : public Speedplanner1D_Base
//{
//public:
//    /**
//     * @brief 构造函数：初始化所有成员变量为零或默认值
//     * @param params 速度规划参数，默认为零
//     */
//    SShapedPlanner1D(Speedplanner_1D_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.00001f});

//    /**
//     * @brief 规划目标速度
//     * @param now_dis 当前已行驶的距离
//     * @return 规划的目标速度
//     */
//    float plan(float now_dis);

//    /**
//     * @brief 判断规划是否已完成
//     * @return 如果规划已完成则返回 true，否则返回 false
//     */
//    bool isFinished() { return m_phase == S_FINISHED_PHASE; }

//    /**
//     * @brief 重置规划器状态
//     */
//    void reset(void);

//    /**
//     * @brief 重置规划器参数
//     * @param params 速度规划参数
//     */
//    void param_reset(Speedplanner_1D_Param_Config params);

//    /**
//     * @brief 获取当前S型规划阶段
//     * @return 当前 S 型规划阶段
//     */
//    SPhase getPhase() const { return m_phase; }

//    /**
//     * @brief 判断当前S型阶段
//     * @param traveled 已行驶的距离
//     * @return 当前 S 型规划阶段
//     */
//    SPhase determinePhase(float traveled);

//private:
//    // 内部状态变量
//    SPhase m_phase = S_FINISHED_PHASE; // 当前规划所处的阶段

//    // 预计算的 S 型规划各个阶段的距离
//    float m_accelJerkUpDistance_ = 0.0f;   // 加速段：Jerk 上升阶段的路程
//    float m_accelConstDistance_ = 0.0f;    // 加速段：加速度恒定阶段的路程
//    float m_accelJerkDownDistance_ = 0.0f; // 加速段：Jerk 下降阶段的路程
//    float m_constVelDistance_ = 0.0f;      // 匀速段：恒定速度阶段的路程
//    float m_decelJerkUpDistance_ = 0.0f;   // 减速段：Jerk 上升（减速开始）阶段的路程
//    float m_decelConstDistance_ = 0.0f;    // 减速段：加速度恒定（减速中）阶段的路程
//    float m_decelJerkDownDistance_ = 0.0f; // 减速段：Jerk 下降（减速结束）阶段的路程

//    float m_t1_ = 0.0f;
//    float m_t2_ = 0.0f;
//    float m_t3_ = 0.0f;
//    float m_t4_ = 0.0f;
//    float m_t5_ = 0.0f;
//    float m_t6_ = 0.0f;
//    float m_t7_ = 0.0f;
//    float m_vlim_ = 0.0f;
//    /**
//     * @brief 预计算各阶段的距离
//     * @return true 计算成功, false 参数无法收敛
//     */
//    bool
//    cal_PhaseDistances();

//    /**
//     * @brief 加速段：Jerk 上升阶段的速度
//     * @param traveled 已行驶距离
//     * @return 当前速度
//     */
//    float cal_Acc_JerkUpSpeed(float traveled);

//    /**
//     * @brief 加速段：加速度恒定阶段的速度
//     * @param traveled 已行驶距离
//     * @return 当前速度
//     */
//    float cal_Acc_ConstSpeed(float traveled);

//    /**
//     * @brief 加速段：Jerk 下降阶段的速度
//     * @param traveled 已行驶距离
//     * @return 当前速度
//     */
//    float cal_Acc_JerkDownSpeed(float traveled);

//    /**
//     * @brief 减速段：Jerk 上升阶段的速度
//     * @param traveled 已行驶距离
//     * @return 当前速度
//     */
//    float cal_Dec_JerkUpSpeed(float traveled);

//    /**
//     * @brief 减速段：加速度恒定阶段的速度
//     * @param traveled 已行驶距离
//     * @return 当前速度
//     */
//    float cal_Dec_ConstSpeed(float traveled);

//    /**
//     * @brief 减速段：Jerk 下降阶段的速度
//     * @param traveled 已行驶距离
//     * @return 当前速度
//     */
//    float cal_Dec_JerkDownSpeed(float traveled);
//};

///**
// * @brief 一维恒加速度平滑器
// *
// * 用于将目标速度平滑地调整到期望速度，限制加速度变化。
// */
//class ConstantAcc
//{
//public:
//    /**
//     * @brief 构造函数，传入加速度上限和初始速度
//     * @param maxAcceleration 最大加速度（单位 m/s?）
//     * @param initialValue 初始速度（单位 m/s）
//     */
//    ConstantAcc(float maxAcceleration = 0.0f, float initialValue = 0.0f);

//    /**
//     * @brief 规划函数：传入目标速度，返回平滑输出速度
//     * @param targetSpeed 目标速度
//     * @return 平滑后的速度
//     */
//    float plan(float targetSpeed);

//    /**
//     * @brief 设置新的加速度上限
//     * @param acceleration 最大加速度（单位 m/s?）
//     */
//    void setMaxAcceleration(float acceleration);

//    /**
//     * @brief 重置规划器状态，可设置初始速度
//     * @param maxAcceleration 最大加速度
//     * @param initialValue 初始速度
//     */
//    void reset(float maxAcceleration = 0.0f, float initialValue = 0.0f);

//    /**
//     * @brief 仅重置速度为0
//     */
//    void reset_speed();

//private:
//    float maxAcceleration_; // 加速度上限（单位 m/s?）
//    float lastOutput_;      // 上一次输出的速度（单位 m/s）
//};

///**
// * @brief 一维TD平滑器
// *
// * 用于对输入信号进行平滑处理，R参数越小越平滑，越大越灵敏。
// */
//class Td
//{
//public:
//    /**
//     * @brief 构造函数，设置R参数
//     * @param td_r_ R参数，越小越平滑
//     */
//    Td(float td_r_ = 0.0f);

//    /**
//     * @brief 设置R参数
//     * @param td_r_ R参数
//     */
//    void set_R(float td_r_); // R越小越平滑。越大越猛

//    /**
//     * @brief TD平滑函数
//     * @param input_expect 期望输入
//     * @return 平滑输出
//     */
//    float plan(float input_expect);

//    void reset(void)
//    {
//        V1_ = 0.0f;
//        V2_ = 0.0f;
//        fh_ = 0.0f;
//        expect_ = 0.0f;
//        Ts_ = 0.0f;
//        previous_time_ = 0;
//    }

//private:
//    float r_ = 0.0f;             // TD平滑参数R
//    float V1_ = 0.0f;            // 内部状态变量1
//    float V2_ = 0.0f;            // 内部状态变量2
//    float fh_ = 0.0f;            // 辅助变量
//    float expect_ = 0.0f;        // 期望值
//    float Ts_ = 0.0f;            // 采样周期
//    uint32_t previous_time_ = 0; // 上一次调用的时间戳
//};

//class ADRC
//{
//public:
//    /**
//     * @brief 自抗扰控制器（Active Disturbance Rejection Control, ADRC）
//     *
//     * ADRC 是一种先进的控制算法，能够在不依赖精确数学模型的情况下实现对系统的高效控制。
//     * 该类实现了 ADRC 的核心功能，包括扩张状态观测器（ESO）、非线性状态误差反馈（NLSEF）
//     * 和跟踪微分器（TD）。
//     */
//    ADRC(ADRC_Param_Config params);

//    /**
//     * @brief 计算 ADRC 输出
//     * @param normalization 是否进行归一化处理
//     * @param unit 归一化单位（默认值为 PI）
//     * @return ADRC 输出值
//     */
//    float ADRC_Calculate(bool normalization = false, float unit = PI);

//    /**
//     * @brief 更新实际反馈值
//     * @param real_ 实际反馈值
//     */
//    void Update_Real(float real_) { y = real_; }

//    /**
//     * @brief 更新目标值
//     * @param target_ 目标值
//     */
//    void Update_Target(float target_) { v = target_; }

//    /**
//     * @brief 获取当前 ADRC 输出值
//     * @return 当前输出值
//     */
//    float Get_Output() { return u; }

//    /**
//     * @brief 初始化 ADRC 参数
//     * @param params ADRC 参数配置结构体
//     */
//    void ADRC_Param_Init(ADRC_Param_Config params);

//protected:
//private:
//    /**
//     * @brief 非线性函数 fal
//     * @param e_ 输入误差
//     * @param alpha_ 非线性因子
//     * @param delta_ 线性区间宽度
//     * @return fal 函数输出值
//     */
//    float fal(float e_, float alpha_, float delta_);

//    /**
//     * @brief 非线性跟踪微分器 fst
//     * @param x1_ 状态变量 1
//     * @param x2_ 状态变量 2
//     * @param r_ 跟踪因子
//     * @param h_ 滤波因子
//     * @return fst 函数输出值
//     */
//    float fst(float x1_, float x2_, float r_, float h_);

//    /**
//     * @brief 符号函数 sgn
//     * @param x_ 输入值
//     * @return 符号函数输出值
//     */
//    float sgn(float x_);

//    /**
//     * @brief 非线性函数 fhan
//     * @param x1 状态变量 1
//     * @param x2 状态变量 2
//     * @param r 跟踪因子
//     * @param h 滤波因子
//     * @return fhan 函数输出值
//     */
//    float fhan(float x1, float x2, float r, float h);

//    float output_limit = 0; // 输出限幅

//    /*---------------TD-----------------*/
//    float r = 0; // 快速跟踪因子
//    float h = 0; // 滤波因子，系统调用步长

//    /*---------------ESO----------------*/
//    float b = 0;       // 系统系数
//    float delta = 0;   // fal函数的线性区间宽度
//    float beta_01 = 0; // 扩张状态观测器反馈增益1
//    float beta_02 = 0; // 扩张状态观测器反馈增益2
//    float beta_03 = 0; // 扩张状态观测器反馈增益3

//    /*---------------NLSEF--------------*/
//    float alpha_1 = 0; // 非线性因子1
//    float alpha_2 = 0; // 非线性因子2
//    float beta_1 = 0;  // 跟踪输入信号增益1
//    float beta_2 = 0;  // 跟踪微分信号增益2

//    /*---------------状态变量------------*/
//    float v = 0; // 输入目标值

//    float v1 = 0;
//    float v2 = 0;

//    float v1_last = 0;
//    float v2_last = 0;

//    float u = 0;
//    float u0 = 0;

//    float z1 = 0;
//    float z2 = 0;
//    float z3 = 0;

//    float e1 = 0;
//    float e2 = 0;
//    float e2_last = 0;

//    float u0_last = 0;

//    float y = 0; // 反馈值
//};

///////////////////////////////    2D 版本 没有应用场景故注释掉    //////////////////////////

///**
// * @brief 二维速度规划参数结构体
// *
// * 用于初始化二维速度规划器的参数。
// */
// typedef struct
//{
//    float maxAcc;       // 最大加速度（正值）
//    float maxDec;       // 最大减速度（正值）
//    float maxJerk;      // 最大加加速度（正值）
//    float maxSpeed;     // 最大允许速度
//    float initialSpeed; // 起始时的速度
//    float finalSpeed;   // 目标点的速度
//    Vector2D startPos;  // 起始位置
//    Vector2D targetPos; // 目标位置
//    float deadzone;     // 死区范围（小于该范围视为到达目标点）
//} Speedplanner_2D_Param_Config;

///**
// * @brief 基类：二维速度规划器
// */
///**
// * @brief 二维速度规划器基类（抽象类）
// *
// * 提供速度规划的基本接口，所有二维速度规划器的派生类都需要实现这些接口。
// */
// class Speedplanner2D_Base
//{
// public:
//    /**
//     * @brief 规划目标速度（纯虚函数）
//     * @param now_dis 当前已行驶的距离
//     * @return 规划的目标速度
//     */
//    virtual Vector2D plan(Vector2D &now_dis) = 0;

//    /**
//     * @brief 判断规划是否完成（纯虚函数）
//     * @return 如果规划完成返回 true，否则返回 false
//     */
//    virtual bool isFinished() = 0;

//    /**
//     * @brief 重置规划器状态（纯虚函数）
//     */
//    virtual void reset() = 0;

//    /**
//     * @brief 重置规划器参数（纯虚函数）
//     * @param params 速度规划参数
//     */
//    virtual void param_reset(Speedplanner_2D_Param_Config params) = 0;

// protected:
//     // 规划参数
//     float m_maxAcc_ = 0.0f;                       // 最大加速度
//     float m_maxDec_ = 0.0f;                       // 最大减速度
//     float m_maxJerk_ = 0.0f;                      // 最大加加速度
//     float m_maxSpeed_ = 0.0f;                     // 最大速度
//     float m_initialSpeed_ = 0.0f;                 // 起始速度
//     float m_finalSpeed_ = 0.0f;                   // 目标速度
//     Vector2D m_startPos_ = Vector2D(0.0f, 0.0f);  // 起始位置
//     Vector2D m_targetPos_ = Vector2D(0.0f, 0.0f); // 目标位置
//     float m_totalDistance_ = 0.0f;                // 总路程
//     float m_deadzone_ = 0.00001f;                 // 死区范围
// };

///**
// * @brief 二维梯形速度规划器（派生类）
// *
// * 实现二维梯形速度曲线的规划。
// */
// class TrapePlanner2D : public Speedplanner2D_Base
//{
// public:
//    /**
//     * @brief 构造函数：初始化规划器参数
//     * @param params 速度规划参数，默认为零
//     */
//    TrapePlanner2D(Speedplanner_2D_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f}, {0.0f, 0.0f}, 0.00001f});

//    /**
//     * @brief 规划目标速度
//     * @param now_dis 当前已行驶的距离
//     * @return 规划的目标速度
//     */
//    Vector2D plan(Vector2D &now_dis);

//    /**
//     * @brief 判断规划是否完成
//     * @return 如果规划完成返回 true，否则返回 false
//     */
//    bool isFinished() { return m_phase == FINISHED_PHASE; }

//    /**
//     * @brief 重置规划器状态
//     */
//    void reset(void);

//    /**
//     * @brief 重置规划器参数
//     * @param params 速度规划参数
//     */
//    void param_reset(Speedplanner_2D_Param_Config params);

//    /**
//     * @brief 判断当前阶段
//     * @param traveled 已行驶的距离
//     * @return 当前阶段
//     */
//    Phase determinePhase(float traveled);

//    /**
//     * @brief 获取当前阶段
//     * @return 当前阶段
//     */
//    Phase getPhase() const { return m_phase; }

// protected:
//     // 内部状态
//     ProfileType m_profileType;
//     Phase m_phase = FINISHED_PHASE; // 当前阶段
//     // 各阶段路程
//     float m_accelDistance_ = 0.0f; // 加速段长度
//     float m_decelDistance_ = 0.0f; // 减速段长度

//    float direction_ = 0.0f;      // 运动方向
//    float min_dead_speed_ = 0.0f; // 最小死区速度
//    float v_target_ = 0.0f;       // 目标速度
//};

///**
// * @brief 二维S型速度规划器（派生类）
// *
// * 实现二维S型速度曲线的规划。
// */
// class SShapedPlanner2D : public Speedplanner2D_Base
//{
// public:
//    /**
//     * @brief 构造函数：初始化所有成员变量为零或默认值
//     * @param params 速度规划参数，默认为零
//     */
//    SShapedPlanner2D(Speedplanner_2D_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, {0.0f, 0.0f}, {0.0f, 0.0f}, 0.00001f});

//    /**
//     * @brief 规划目标速度
//     * @param now_dis 当前已行驶的距离
//     * @return 规划的目标速度
//     */
//    Vector2D plan(Vector2D &now_dis);

//    /**
//     * @brief 判断规划是否已完成
//     * @return 如果规划已完成则返回 true，否则返回 false
//     */
//    bool isFinished() { return m_phase == S_FINISHED_PHASE; }

//    /**
//     * @brief 重置规划器状态
//     */
//    void reset(void);

//    /**
//     * @brief 重置规划器参数
//     * @param params 速度规划参数
//     */
//    void param_reset(Speedplanner_2D_Param_Config params);

//    /**
//     * @brief 获取当前S型规划阶段
//     * @return 当前 S 型规划阶段
//     */
//    SPhase getPhase() const { return m_phase; }

//    /**
//     * @brief 判断当前S型阶段
//     * @param traveled 已行驶的距离
//     * @return 当前 S 型规划阶段
//     */
//    SPhase determinePhase(float traveled);

// private:
//     // 内部状态变量
//     SPhase m_phase = S_FINISHED_PHASE; // 当前规划所处的阶段

//    // 预计算的 S 型规划各个阶段的距离
//    float m_accelJerkUpDistance_ = 0.0f;   // 加速段：Jerk 上升阶段的路程
//    float m_accelConstDistance_ = 0.0f;    // 加速段：加速度恒定阶段的路程
//    float m_accelJerkDownDistance_ = 0.0f; // 加速段：Jerk 下降阶段的路程
//    float m_constVelDistance_ = 0.0f;      // 匀速段：恒定速度阶段的路程
//    float m_decelJerkUpDistance_ = 0.0f;   // 减速段：Jerk 上升（减速开始）阶段的路程
//    float m_decelConstDistance_ = 0.0f;    // 减速段：加速度恒定（减速中）阶段的路程
//    float m_decelJerkDownDistance_ = 0.0f; // 减速段：Jerk 下降（减速结束）阶段的路程

//    Vector2D m_startPos1_ = Vector2D(0.0f, 0.0f);

//    int err_ = 0;         // 错误标志，0表示无错误，1表示参数计算错误
//    float m_t1_ = 0.0f;   //  phase 1 的截至时间
//    float m_t2_ = 0.0f;   //  phase 2 的截至时间
//    float m_t3_ = 0.0f;   //  phase 3 的截至时间
//    float m_t4_ = 0.0f;   //  phase 4 的截至时间
//    float m_t5_ = 0.0f;   //  phase 5 的截至时间
//    float m_t6_ = 0.0f;   //  phase 6 的截至时间
//    float m_t7_ = 0.0f;   //  phase 7 的截至时间
//    float m_vlim_ = 0.0f; //  限制的最大速度

//    /**
//     * @brief 预计算各阶段的距离
//     */
//    void cal_PhaseDistances();

//    /**
//     * @brief 加速段：Jerk 上升阶段的速度
//     * @param traveled 已行驶距离
//     * @return 当前速度
//     */
//    float cal_Acc_JerkUpSpeed(float traveled);

//    /**
//     * @brief 加速段：加速度恒定阶段的速度
//     * @param traveled 已行驶距离
//     * @return 当前速度
//     */
//    float cal_Acc_ConstSpeed(float traveled);

//    /**
//     * @brief 加速段：Jerk 下降阶段的速度
//     * @param traveled 已行驶距离
//     * @return 当前速度
//     */
//    float cal_Acc_JerkDownSpeed(float traveled);

//    /**
//     * @brief 减速段：Jerk 上升阶段的速度
//     * @param traveled 已行驶距离
//     * @return 当前速度
//     */
//    float cal_Dec_JerkUpSpeed(float traveled);

//    /**
//     * @brief 减速段：加速度恒定阶段的速度
//     * @param traveled 已行驶距离
//     * @return 当前速度
//     */
//    float cal_Dec_ConstSpeed(float traveled);

//    /**
//     * @brief 减速段：Jerk 下降阶段的速度
//     * @param traveled 已行驶距离
//     * @return 当前速度
//     */
//    float cal_Dec_JerkDownSpeed(float traveled);
//};

#endif

#endif