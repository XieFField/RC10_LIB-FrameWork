/**
 * @file APP_Path.h
 * @author naoganlin
 * @brief 局部路径规划
 * 1.基于贝塞尔曲线和S型速度规划
 * 2.支持一阶和二阶贝塞尔曲线路径规划
 * 3.速度规划参数得给一定的初速度否则跑的时候不稳定
 * @version 1.0
 * @date 2025-10-30
 */

#ifndef __APP_PATH_H
#define __APP_PATH_H
#include <arm_math.h>
#pragma once

#ifdef __cplusplus

extern "C"
{
}
#include "APP_Bezier_Curve.h" // 包含贝塞尔曲线相关的头文件
#include "APP_Speedplanner.h" // 包含速度规划器相关的头文件

/**
 * @class Path_Bezier
 * @brief 基于贝塞尔曲线的路径规划类
 *
 * 该类实现了路径规划功能，包括路径点的计算、
 * 路径重置以及路径更新。
 */
class Path_Bezier
{
public:
    /**
     * @brief 默认构造函数
     */
    Path_Bezier();

    /**
     * @brief 一阶贝塞尔曲线构造函数
     * @param start_point 起点
     * @param end_point 终点
     * @param params 速度规划参数
     */
    Path_Bezier(Vector2D start_point, Vector2D end_point, Speedplanner_1D_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.00001f}) : bc_(start_point, end_point)
    {
        m_phase = S_ACCEL_JERK_UP_PHASE;  // 初始化阶段为加速阶段
        end_point_ = end_point;           // 设置终点
        params.initialSpeed = 0.001f;     // 设置初始速度
        params.startPos = 0.0f;           // 设置起始位置
        params.targetPos = bc_.Get_len(); // 设置目标位置为曲线长度
        sp_.param_reset(params);          // 重置速度规划参数
        point_last_ = start_point;        // 设置上一个点为起点
    }

    /**
     * @brief 二阶贝塞尔曲线构造函数
     * @param start_point 起点
     * @param control_point 控制点
     * @param end_point 终点
     * @param params 速度规划参数
     */
    Path_Bezier(Vector2D start_point, Vector2D control_point, Vector2D end_point, Speedplanner_1D_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.00001f}) : bc_(start_point, control_point, end_point)
    {
        m_phase = S_ACCEL_JERK_UP_PHASE;  // 初始化阶段为加速阶段
        end_point_ = end_point;           // 设置终点
        params.initialSpeed = 0.001f;     // 设置初始速度
        params.startPos = 0.0f;           // 设置起始位置
        params.targetPos = bc_.Get_len(); // 设置目标位置为曲线长度
        sp_.param_reset(params);          // 重置速度规划参数
        point_last_ = start_point;        // 设置上一个点为起点
    }

    /**
     * @brief 规划路径点
     * @param point 当前点
     * @return Vector2D 返回规划后的速度向量
     */
    Vector2D plan(Vector2D point)
    {
        v_resultant_ = sp_.plan(distance_); // 计算当前速度
        m_phase = sp_.getPhase();           // 获取当前阶段

        bc_.Get_Nearest_Distance(point, &t_);                  // 获取点到曲线的最近距离
        v_tangent_ = (bc_.Get_Tangent_Vector(t_)).normalize(); // 计算切线向量

        distance_ += (point - point_last_).magnitude(); // 更新距离
        point_last_ = point;                            // 更新上一个点

        return (v_tangent_ * v_resultant_); // 返回速度向量
    }

    /**
     * @brief 重置路径规划器
     */
    void reset(void)
    {
        m_phase = S_ACCEL_JERK_UP_PHASE;     // 重置阶段为加速阶段
        point_last_ = bc_.Get_Start_point(); // 重置上一个点为起点
        distance_ = 0.0f;                    // 重置距离
        t_ = 0.0f;                           // 重置参数 t
        v_resultant_ = 0.0f;                 // 重置速度
    }

    /**
     * @brief 更新一阶贝塞尔曲线
     * @param start_point 起点
     * @param end_point 终点
     */
    void update(Vector2D start_point, Vector2D end_point)
    {
        end_point_ = end_point;                    // 更新终点
        bc_.Bezier_Update(start_point, end_point); // 更新贝塞尔曲线
    }

    /**
     * @brief 更新二阶贝塞尔曲线
     * @param start_point 起点
     * @param control_point 控制点
     * @param end_point 终点
     */
    void update(Vector2D start_point, Vector2D control_point, Vector2D end_point)
    {
        end_point_ = end_point;                                   // 更新终点
        bc_.Bezier_Update(start_point, control_point, end_point); // 更新贝塞尔曲线
    }

    /**
     * @brief 判断路径规划是否完成
     * @return true 如果完成
     * @return false 如果未完成
     */
    bool isFinished() { return m_phase == S_FINISHED_PHASE; }

private:
    SPhase m_phase = S_FINISHED_PHASE;           // 当前规划所处的阶段
    BezierCurve bc_;                             // 贝塞尔曲线对象
    SShapedPlanner1D sp_;                        // 一维 S 型速度规划器
    float t_ = 0.0f;                             // 贝塞尔曲线参数 t
    float v_resultant_ = 0.0f;                   // 当前速度
    float distance_ = 0.0001f;                   // 当前距离
    Vector2D v_tangent_ = Vector2D(0.0f, 0.0f);  // 切线向量
    Vector2D point_last_ = Vector2D(0.0f, 0.0f); // 上一个点
    Vector2D end_point_ = Vector2D(0.0f, 0.0f);  // 终点
};
#endif

#endif