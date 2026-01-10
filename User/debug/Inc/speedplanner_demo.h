/**
 * @file speedplanner_demo.h
 * @author naoganlin
 * @brief 速度控制器demo,用宏定义调用开关
 * @version 1.0
 * @date 2025-10-28
 */

#ifndef __SPEEDPLANNER_DEMO_H
#define __SPEEDPLANNER_DEMO_H

#pragma once

#ifdef __cplusplus

extern "C"
{
#endif
#include "cmsis_os.h"
#include "usart.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "BSP_RTOS.h"
#include "APP_debugTool.h"
#include "APP_CoordConvert.h"
#include "BSP_TimeStamp.h"
#include "APP_Speedplanner.h"
#include "debug_setup.h"
#include "APP_Bezier_Curve.h"
#include "APP_Path.h"

#include "Module_ChassisOmni.h"
#include "Motor_Base.h"
#include "FSMstauts_enum.h"
#include "APP_debugTool.h"
#include "APP_CoordConvert.h"
#include "APP_Speedplanner.h"
#include "APP_PID.h"
#include "Module_Position.h"

#if SPEEDPLANNER_DEMO_DEBUG
#define Path_end 1               // 启用路径规划-末端点模式
#define Path_s 0                 // 启用路径规划-S形模式
#define Bezier_Curve 0           // 启用贝塞尔曲线测试
#define trapezoid_Velocitytype 0 // 启用梯形速度规划测试
#define Positionaltype_1D 0      // 启用1D位置规划测试
#define Positionaltype_2D 0      // 启用2D位置规划测试
#endif

class SpeedPlanner_Demo : public RtosTask
{
public:
    SpeedPlanner_Demo() : RtosTask("SpeedPlanner_Demo", 1), debug_uart(&huart8) {}

    /**
     * @brief 初始化函数
     *
     * 根据宏定义选择的模式，初始化路径点、速度规划参数和PID参数。
     */
    void init();

    /**
     * @brief 主循环函数
     *
     * 执行周期性的路径跟踪、速度规划和位置更新逻辑。
     */
    void loop() override;

    Debug_Printf debug_uart; // 调试串口对象

    Robot_Twist last_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};   // 上一次底盘速度状态
    Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // 目标底盘速度状态

    bool is_path_completed_ = false;      // 路径是否完成标志
    const float DIST_TO_END = 0.5f;       // 到达终点的判定距离阈值 (单位: 米)
    Vector2D speed = {0.0f, 0.0f};        // 当前机器人的实际合成速度
    Vector2D planspeed = {0.0f, 0.0f};    // 路径规划器计算出的切向速度
    Vector2D baseVelocity = {0.0f, 0.0f}; // 基础速度向量

    // 角度锁定相关
    float lock_angle = 0.0f;  // 目标锁定角度
    float lock_update = 0.0f; // 角度更新变量

    PID_Position yaw_pid_; // 偏航角PID控制器

    int num = 0;                              // 循环计数器，用于分频打印调试信息
    float tNearest = 0.0f;                    // 最近点在贝塞尔曲线上的参数t (0~1)
    float tLookahead = 0.0f;                  // 前视点在贝塞尔曲线上的参数t (0~1)
    float m_lookaheadDist = 0.4f;             // 前视距离 (单位: 米)
    float lateralError = 0.0f;                // 横向误差 (机器人偏离路径的距离)
    float correctspeed = 0.0f;                // 计算出的横向纠偏速度大小
    Vector2D pos;                             // 机器人当前位置 (仿真或实际)
    Vector2D nearestPt;                       // 路径上距离机器人最近的点
    Vector2D lookaheadPt;                     // 路径上的前视点
    Vector2D lookaheadTangent;                // 前视点处的切线方向向量
    Vector2D pathEnd;                         // 路径终点坐标
    Vector2D corrVelocity = {0.0f, 0.0f};     // 计算出的横向纠偏速度向量
    Speedplanner_1D_Param_Config path_param_; // 速度规划器配置参数
    PID_Position pid_track;                   // 循迹横向误差PID控制器

    /**
     * @brief 获取路径上距离机器人最近的点
     * @param path_ 贝塞尔曲线对象
     * @param robotPos 机器人当前位置
     * @param tNearest 输出参数，返回最近点的t值
     * @return Vector2D 最近点的坐标
     */
    Vector2D GetPathNearestPoint(BezierCurve &path_, const Vector2D &robotPos, float &tNearest);

    /**
     * @brief 寻找前视点
     * @param path_ 贝塞尔曲线对象
     * @param tNearest 最近点的t值
     * @param tLookahead 输出参数，返回前视点的t值
     * @return Vector2D 前视点的坐标
     */
    Vector2D FindLookaheadPoint(BezierCurve &path_, float tNearest, float &tLookahead);

    /**
     * @brief 计算横向误差
     * @param path_ 贝塞尔曲线对象
     * @param robotPos 机器人当前位置
     * @param nearestPt 最近点坐标
     * @param tLookahead 前视点的t值
     * @return float 横向误差值 (带符号，表示偏左或偏右)
     */
    float CalculateLateralError(BezierCurve &path_, const Vector2D &robotPos, const Vector2D &nearestPt, float tLookahead);

private:
};

#endif // __cplusplus

#endif