/**
 * @file APP_Path.h
 * @author naoganlin
 * @brief 局部路径规划
 * 1.基于贝塞尔曲线和S型速度规划
 * 2.支持一阶和二阶贝塞尔曲线路径规划
 * 3.速度规划参数得给一定的初速度否则跑的时候不稳定
 * path类：
 * 两种用法：
 * 1.控制点给值设为1 （当然也可以设为0到1之间的数，但是会取控制点前后两点之间的十分位点作为贝塞尔曲线的起点和终点）
 * 2.全部点都设为0.5除了起点和终点，但是会取控制点前后两点之间的中点作为贝塞尔曲线的起点和终点（当然也可以取别的值效果同理，但是千万不能取大于0.5的值在此用法下）
 * @version 3.0
 * @date 2025-12-14
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
#include "APP_tool.h"
#include "APP_PID.h"
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
        initial_setting(params);
        end_point_ = end_point;    // 设置终点
        point_last_ = start_point; // 设置上一个点为起点
        point_last_ = start_point; // 设置上一个点为起点
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
        initial_setting(params);
        end_point_ = end_point;    // 设置终点
        point_last_ = start_point; // 设置上一个点为起点
        point_last_ = start_point; // 设置上一个点为起点
    }

    /**
     * @brief 规划路径点
     * @param point 当前点
     * @return Vector2D 返回规划后的速度向量
     */
    Vector2D plan(Vector2D point)
    {
        bc_.Get_Nearest_Distance(point, &t_); // 获取点到曲线的最近距离
        distance_ = bc_.Get_Current_Len(t_);
        v_resultant_ = sp_.plan(distance_); // 计算当前速度
        m_phase = sp_.getPhase();           // 获取当前阶段

        v_tangent_ = (bc_.Get_Tangent_Vector(t_)).normalize(); // 计算切线向量
        point_last_ = point;                                   // 更新上一个点
        return (v_tangent_ * v_resultant_);                    // 返回速度向量
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
    void update(Vector2D start_point, Vector2D end_point, Speedplanner_1D_Param_Config params)
    {
        initial_setting(params);
        end_point_ = end_point;                    // 设置终点
        point_last_ = start_point;                 // 设置上一个点为起点
        bc_.Bezier_Update(start_point, end_point); // 更新贝塞尔曲线
    }

    /**
     * @brief 更新二阶贝塞尔曲线
     * @param start_point 起点
     * @param control_point 控制点
     * @param end_point 终点
     */
    void update(Vector2D start_point, Vector2D control_point, Vector2D end_point, Speedplanner_1D_Param_Config params)
    {
        initial_setting(params);
        end_point_ = end_point;                                   // 设置终点
        point_last_ = start_point;                                // 设置上一个点为起点
        bc_.Bezier_Update(start_point, control_point, end_point); // 更新贝塞尔曲线
    }

    /**
     * @brief 判断路径规划是否完成
     * @return true 如果完成
     * @return false 如果未完成
     */
    bool isFinished() { return m_phase == S_FINISHED_PHASE; }

    /**
     * @brief 获取贝塞尔曲线对象
     * @return BezierCurve& 返回贝塞尔曲线的引用
     */
    BezierCurve &get_bezier_curve(void)
    {
        return bc_;
    }

private:
    void initial_setting(Speedplanner_1D_Param_Config params)
    {
        m_phase = S_ACCEL_JERK_UP_PHASE;                         // 初始化阶段为加速阶段
        params.initialSpeed = abs(params.initialSpeed) + 0.001f; // 设置初始速度
        params.startPos = 0.0f;                                  // 设置起始位置
        params.targetPos = bc_.Get_len();                        // 设置目标位置为曲线长度
        sp_.param_reset(params);                                 // 重置速度规划参数
    }
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

typedef enum Generate_Curve_Status
{
    GENERATE_WAIT_FIRST_POINT = 0,
    GENERATE_FINISHED_STRAIGHT,
    GENERATE_WAIT_LAST_CURVE_POINT
} Generate_Curve_Status;

#define MAX_CURVE_NUM 20 // 最大曲线数
#define MAX_PATH_NUM 20  // 最大路径数
// 路径（从静止启动到静止）
class Path
{
public:
    /**
     * @brief 默认构造函数
     */
    Path();
    /**
     * @brief 添加途径点
     * @param point_ 途径点坐标
     * @param smoothness_ 平滑度参数，范围[0, 0.5]
     * @return true 如果添加成功
     * @return false 如果添加失败
     */
    bool Add_Point(Vector2D point_, float smoothness_); // smoothness_ 0~0.5

    /**
     * @brief 添加起始点
     * @param point_ 起始点坐标
     * @param have_start_angle_ 是否有起始角度
     * @param start_angle_ 起始角度值
     * @return true 如果添加成功
     * @return false 如果添加失败
     */
    bool Add_Start_Point(Vector2D point_, bool have_start_angle_, float start_angle_, Speedplanner_1D_Param_Config params);

    /**
     * @brief 添加结束点
     * @param point_ 结束点坐标
     * @param end_angle_ 结束角度值
     * @return true 如果添加成功
     * @return false 如果添加失败
     */
    bool Add_End_Point(Vector2D point_, float end_angle_);

    /**
     * @brief 获取路径误差和向量
     * @param location_ 当前坐标
     * @param yaw 当前航向角
     * @param target_yaw 输出目标航向角
     * @param normal_error 输出法向误差
     * @param tangent_error 输出切向误差
     * @param normal_vector 输出法向量
     * @param tangent_vector 输出切向量
     * @param max_vel 输出最大速度限制
     * @return true 如果获取成功
     * @return false 如果获取失败
     */
    bool Get_Error_And_Vector(
        Vector2D location_,
        float yaw,
        float *target_yaw,
        float *normal_error,
        float *tangent_error,
        Vector2D *normal_vector,
        Vector2D *tangent_vector,
        float *max_vel);

    /**
     * @brief 路径规划计算
     * @param point 当前位置点
     * @return Vector2D 返回期望的速度向量
     */
    Vector2D plan(Vector2D point);

    /**
     * @brief 重置规划状态
     */
    void plan_reset();
    /**
     * @brief 获取路径是否结束
     * @return true 如果路径结束
     * @return false 如果路径未结束
     */
    bool Is_End() { return is_end; }

    /**
     * @brief 重置路径
     */
    void Reset();

    /**
     * @brief 获取当前阶段
     * @return 当前阶段
     */
    SPhase getPhase() const { return m_phase; }

    /**
     * @brief 判断规划是否已完成
     * @return 如果规划已完成则返回 true，否则返回 false
     */
    bool isFinished() { return m_phase == S_FINISHED_PHASE; }

    /**
     * @brief 获取当前贝塞尔曲线对象
     * @return BezierCurve& 返回当前贝塞尔曲线的引用
     */
    BezierCurve &get_bezier_curve(void)
    {
        if (index_ >= bezier_curve_num && bezier_curve_num > 0)
        {
            return bezier_curve_list[bezier_curve_num - 1];
        }
        return bezier_curve_list[index_];
    }

protected:
    float err_end = 0.0f; // 末端误差
    float dead = 0.04f;
    Vector2D v_output_ = Vector2D(0.0f, 0.0f);
    // 旧代码
    BezierCurve bezier_curve_list[MAX_CURVE_NUM]; // 储存各路段曲线
    SShapedPlanner1D sp_;                         // 一维 S 型速度规划器
    float total_len = 0;                          // 路线总长度

    int index_ = 0;
    float total_ = 0.0f;
    float distance_ = 0.0f;
    float t_ = 0.0f;                             // 贝塞尔曲线参数 t
    float v_resultant_ = 0.0f;                   // 当前速度
    Vector2D v_tangent_ = Vector2D(0.0f, 0.0f);  // 切线向量
    Vector2D point_last_ = Vector2D(0.0f, 0.0f); // 上一个点
    SPhase m_phase = S_FINISHED_PHASE;
    Speedplanner_1D_Param_Config params_; // 每条曲线对应的速度规划参数
    uint8_t bezier_curve_num = 0;         // 总曲线数量

    /**
     * @brief 计算各路段的结束速度
     */
    void Calc_End_Vel();

    /**
     * @brief 生成各路段的曲线
     * @param point_ 当前点坐标
     * @param smoothness_ 平滑度参数
     * @return true 如果生成成功
     * @return false 如果生成失败
     */
    bool Generate_Curve(Vector2D point_, float smoothness_);

    /*------------------------------路径参数-----------------------------------*/
    bool have_start_angle = 0; // 是否要求达到起始角度后再启动
    float start_angle = 0;     // 起始角度
    float end_angle = 0;       // 结束角度

    /*------------------------------过程变量-----------------------------------*/
    float currnet_target_angle = 0; // 当前目标角度

    uint8_t current_bezier_curve_dx = 0; // 当前路段对应曲线的索引

    float current_t = 0; // 当前坐标对应当前路段的t值

    float current_finished_len = 0; // 已完成的曲线的总长度

    float current_curve_len = 0; // 当前曲线走过的长度

private:
    /*---------------------------------状态-------------------------------------*/
    bool is_end = false;   // 是否开始
    bool is_start = false; // 是否结束

    bool is_init = false; // 是否初始化

    /*----------------------------生成路径的临时变量-------------------------------------*/
    Generate_Curve_Status generate_status = GENERATE_WAIT_FIRST_POINT; // 生成曲线的状态

    Vector2D point_list[3]; // 生成曲线时临时存放点坐标

    float last_smoothness; // 生成曲线时临时存放上一个点的平滑程度
};

class Path_line
{
public:
    Path_line()
    {
        bezier_curve_num = 0;
        is_init = false;
    }
    bool Add_Point(Vector2D point_, Speedplanner_1D_Param_Config params)
    {
        if (is_init == true)
            return false;
        params_[bezier_curve_num] = params;
        bezier_curve_list[bezier_curve_num].Bezier_Update(point_last_, point_);
        bezier_curve_num++;
        point_last_ = point_;
        params.initialSpeed = abs(params.initialSpeed) + 0.001f; // 设置初始速度

        return true;
    }
    bool Add_Point(Vector2D point_, Vector2D control_point_, Speedplanner_1D_Param_Config params)
    {
        if (is_init == true)
            return false;
        params_[bezier_curve_num] = params;
        bezier_curve_list[bezier_curve_num].Bezier_Update(point_last_, control_point_, point_);
        bezier_curve_num++;
        point_last_ = point_;
        params.initialSpeed = abs(params.initialSpeed) + 0.001f; // 设置初始速度

        return true;
    }

    bool Add_Start_Point(Vector2D point_)
    {

        if (is_init == true)
            return false;
        point_last_ = point_;
        return true;
    }

    bool Add_End_Point(Vector2D point_, Speedplanner_1D_Param_Config params)
    {
        if (is_init == true)
            return false;

        is_init = true;
        is_end = true;
        params_[bezier_curve_num] = params;
        bezier_curve_list[bezier_curve_num].Bezier_Update(point_last_, point_);
        bezier_curve_num++;
        index_ = 0;
        params_[index_].targetPos = (bezier_curve_list[index_].Get_len() - params_[index_].startPos);
        params_[index_].startPos = 0.0f; // 设置起始位置
        sp_.param_reset(params_[index_]);
        m_phase = ACCEL_PHASE; // 初始化阶段为加速阶段
        return true;
    }

    /**
     * @brief 路径规划计算
     * @param point 当前位置点
     * @return Vector2D 返回期望的速度向量
     */
    Vector2D plan(Vector2D point)
    {
        if (Is_End() == true)
        {
            bezier_curve_list[index_].Get_Nearest_Distance(point, &t_); // 获取点到曲线的最近距离
            distance_ = bezier_curve_list[index_].Get_Current_Len(t_);

            v_resultant_ = sp_.plan(distance_); // 速度规划器计算当前目标速度
            m_phase = sp_.getPhase();           // 获取当前速度规划阶段

            v_tangent_ = (bezier_curve_list[index_].Get_Tangent_Vector(t_)).normalize(); // 计算切线向量（单位向量）

            err_end = _tool_Abs((point - bezier_curve_list[index_].Get_End_point()).magnitude());

            // 段切换条件：
            // 1) 近端误差达到阈值可直接切段；
            // 2) t 接近 1 仅作为辅助条件，必须同时离终点不远，避免“投影到段末端但车体仍较远”时误切段。
            const float t_reach_guard = 0.20f;
            bool reach_segment_end = (_tool_Abs(err_end) <= dead) || (t_ >= 0.995f && _tool_Abs(err_end) <= t_reach_guard);
            // bool reach_segment_end = (t_ >= 0.995f);
            if (reach_segment_end)
            {
                index_++; // 切换到下一段曲线
                t_ = 0.0f;
                if (index_ >= bezier_curve_num)
                {
                    is_end = false; // 结束运行
                    m_phase = FINISHED_PHASE;
                }
                else
                {
                    params_[index_].startPos = 0.0f; // 设置起始位置
                    params_[index_].targetPos = (bezier_curve_list[index_].Get_len() - params_[index_].startPos);
                    sp_.param_reset(params_[index_]);
                }
            }
            v_output_ = (v_tangent_ * v_resultant_);
            return v_output_; // 返回 速度向量 = 切向方向 * 目标速率
        }
        else
        {
            return Vector2D{0, 0};
        }
    }

    /**
     * @brief 重置规划状态
     */
    void plan_reset()
    {
        is_init = false; // 重置初始化标志

        bezier_curve_num = 0; // 重置曲线数量

        is_end = false; // 重置路径结束标志
    }

    /**
     * @brief 获取路径是否结束
     * @return true 如果路径结束
     * @return false 如果路径未结束
     */
    bool Is_End() { return is_end; }

    /**
     * @brief 重置路径
     */
    void Reset()
    {
        index_ = 0;
        point_last_ = bezier_curve_list[index_].Get_Start_point(); // 重置上一个点为起点
        m_phase = ACCEL_PHASE;
        distance_ = 0.0f;    // 重置距离
        t_ = 0.0f;           // 重置参数 t
        v_resultant_ = 0.0f; // 重置速度
        is_end = true;
    }

    /**
     * @brief 获取当前阶段
     * @return 当前阶段
     */
    Phase getPhase() const { return m_phase; }

    /**
     * @brief 判断规划是否已完成
     * @return 如果规划已完成则返回 true，否则返回 false
     */
    bool isFinished() { return m_phase == S_FINISHED_PHASE; }

    /**
     * @brief 获取当前贝塞尔曲线对象
     * @return BezierCurve& 返回当前贝塞尔曲线的引用
     */
    BezierCurve &get_bezier_curve(void)
    {
        if (index_ >= bezier_curve_num && bezier_curve_num > 0)
        {
            return bezier_curve_list[bezier_curve_num - 1];
        }
        return bezier_curve_list[index_];
    }
    // int get_pid_end_flag()  { return pid_end_flag; }
    Vector2D Get_Tangent_Vector()
    {
        return v_tangent_;
    }

protected:
    float dead = 0.04f;
    int index_ = 0;
    BezierCurve bezier_curve_list[MAX_CURVE_NUM]; // 储存各路段曲线

    Speedplanner_1D_Param_Config params_[MAX_CURVE_NUM]; // 每条曲线对应的速度规划参数
    TrapePlanner1D sp_;                                  // 一维 S 型速度规划器

    float distance_ = 0.0f;
    float t_ = 0.0f;                             // 贝塞尔曲线参数 t
    float v_resultant_ = 0.0f;                   // 当前速度
    Vector2D v_tangent_ = Vector2D(0.0f, 0.0f);  // 切线向量
    Vector2D point_last_ = Vector2D(0.0f, 0.0f); // 上一个点
    Vector2D v_output_ = Vector2D(0.0f, 0.0f);
    Phase m_phase = FINISHED_PHASE;

    uint8_t bezier_curve_num = 0; // 总曲线数量

private:
    /*---------------------------------状态-------------------------------------*/
    float err_end = 0.0f;
    bool is_end = false;  // 是否开始
    bool is_init = false; // 是否初始化
};

#endif

#endif