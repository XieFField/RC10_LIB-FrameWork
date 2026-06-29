/**
 * @file APP_Path.h
 * @author naoganlin
 * @brief 局部路径�?�划
 * 1.基于贝�?�尔曲线和S型速度规划
 * 2.�?持一阶和二阶贝�?�尔曲线�?径�?�划
 * 3.速度规划参数得给一定的初速度否则跑的时候不稳定
 * path类：
 * 两�?�用法：
 * 1.控制点给值�?�为1 （当然也�?以�?�为0�?1之间的数，但�?会取控制点前后两点之间的十分位点作为贝�?�尔曲线的起点和终点�?
 * 2.全部点都设为0.5除了起点和终点，但是会取控制点前后两点之间的�?点作为贝塞尔曲线的起点和终点（当然也�?以取�?的值效果同理，但是千万不能取大�?0.5的值在此用法下�?
 * @version 5.2
 * @date 2026-6-8
 */

#ifndef __APP_PATH_H
#define __APP_PATH_H
#include <arm_math.h>
#pragma once

#ifdef __cplusplus

extern "C"
{
}
#include "APP_Bezier_Curve.h" // 包含贝�?�尔曲线相关的头文件
#include "APP_Speedplanner.h" // 包含速度规划器相关的头文�?
#include "APP_tool.h"

typedef enum Generate_Curve_Status
{
    GENERATE_WAIT_FIRST_POINT = 0,
    GENERATE_FINISHED_STRAIGHT,
    GENERATE_WAIT_LAST_CURVE_POINT
} Generate_Curve_Status;

#define MAX_CURVE_NUM 20 // 最大曲线数
#define MAX_PATH_NUM 20  // 最大路径数

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
        params.initialSpeed = abs(params.initialSpeed) + 0.001f; // 设置初忋速度

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
        params.initialSpeed = abs(params.initialSpeed) + 0.001f; // 设置初忋速度

        return true;
    }

    bool Add_Start_Point(Vector2D point_,float dead_=0.03f)
    {
        dead = dead_;
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
        params_[index_].startPos = 0.0f; // 设置起忋位罿
        sp_.param_reset(params_[index_]);
        m_phase = ACCEL_PHASE; // 初忋化阶濵为加速阶殿
        end_point = point_;
        return true;
    }

    /**
     * @brief 跿径迄划计算
     * @param point 当前位置炿
     * @return Vector2D 返回期望的速度向量
     */
    Vector2D plan(Vector2D point)
    {
        if (Is_End() == false)
        {
            
            err_end = _tool_Abs((point - bezier_curve_list[index_].Get_End_point()).magnitude());
            
            bezier_curve_list[index_].Get_Nearest_Distance(point, &t_); // 获取点到曲线的最近距禿
            distance_ = bezier_curve_list[index_].Get_Current_Len(t_);
            
            if(distance_>bezier_curve_list[index_].Get_len()-(brake_distance*brake_coefficient))
                v_resultant_ = sp_.plan(bezier_curve_list[index_].Get_len()-err_end); // 速度规划器迡算当前盿标速度
            else
                v_resultant_ = sp_.plan(distance_); // 速度规划器迡算当前盿标速度
            m_phase = sp_.getPhase();           // 获取当前速度规划阶濿

            if (tangent_lock == false)
            {
                v_tangent_ = (bezier_curve_list[index_].Get_Tangent_Vector(t_)).normalize(); // 计算切线向量（单位向量）
            }
           
            // 段切换条件：
            // 1) 近翿诿巿达到阈值可直接切濵；
            // 2) t 接近 1 仅作为辅助条件，必须同时离终点不远，避免“投影到段末竿但车体仍较远”时诿切濵〿
            const float t_reach_guard = 0.20f;
            bool reach_segment_end = (_tool_Abs(err_end) <= dead) || (t_ >= 0.999f && _tool_Abs(err_end) <= t_reach_guard);
            // bool reach_segment_end = (t_ >= 0.995f);
            if (reach_segment_end)
            {
                index_++; // 切换到下一段曲线
                t_ = 0.0f;
                if (index_ >= bezier_curve_num)
                {
                    is_end = false; // 结束运迿
                    m_phase = FINISHED_PHASE;
                }
                else
                {
                    // 舵轮过弯特殊设计
                    if (params_[index_].targetPos == 1.0f)
                    {
                        tangent_lock = true;
                        v_tangent_ = {1.0f, 0.0f};
                    }
                    else if (params_[index_].targetPos == 2.0f)
                    {
                        tangent_lock = true;
                        v_tangent_ = {-1.0f, 0.0f};
                    }
                    else if (params_[index_].targetPos == 3.0f)
                    {
                        tangent_lock = true;
                        v_tangent_ = {0.0f, 1.0f};
                    }
                    else if (params_[index_].targetPos == 4.0f)
                    {
                        tangent_lock = true;
                        v_tangent_ = {0.0f, -1.0f};
                    }
                    else
                    {
                        tangent_lock = false;
                    }
                    brake_distance=params_[index_].startPos;
                    float temp = (bezier_curve_list[index_].Get_len() - brake_distance);
                    if (tangent_lock == false)
                    {
                        if (temp > 0)
                        {
                            params_[index_].targetPos = temp;
                        }
                        else
                        {
                            params_[index_].targetPos = bezier_curve_list[index_].Get_len();
                        }
                    }

                    params_[index_].startPos = 0.0f; // 设置起忋位罿
                    sp_.param_reset(params_[index_]);
                }
            }
            v_output_ = (v_tangent_ * v_resultant_);
            return v_output_; // 返回 速度向量 = 切向方向 * 盿标速率
        }
        else
        {
            return Vector2D{0, 0};
        }
    }

    /**
     * @brief 重置规划状怿
     */
    void plan_reset()
    {
        tangent_lock = false;
        is_init = false; // 重置初忋化标志

        bezier_curve_num = 0; // 重置曲线数量

        is_end = false; // 重置跿径结束标忿
    }

    /**
     * @brief 获取跿径是否结板
     * @return true 如果跿径结板
     * @return false 如果跿径未结束
     */
    bool Is_End() { return (is_end == false); }

    /**
     * @brief 重置跿徿
     */
    void Reset()
    {
        tangent_lock = false;
        index_ = 0;
        point_last_ = bezier_curve_list[index_].Get_Start_point(); // 重置上一丿点为起点
        m_phase = ACCEL_PHASE;
        distance_ = 0.0f;    // 重置距翿
        t_ = 0.0f;           // 重置参数 t
        v_resultant_ = 0.0f; // 重置速度
        is_end = true;
    }

    /**
     * @brief 获取当前阶濿
     * @return 当前阶濿
     */
    Phase getPhase() const { return m_phase; }

    /**
     * @brief 判断规划昿否已完成
     * @return 如果规划已完成则返回 true，否则返囿 false
     */
    bool isFinished() { return m_phase == FINISHED_PHASE; }

    /**
     * @brief 获取当前贝忞尔曲线对象
     * @return BezierCurve& 返回当前贝忞尔曲线的引甿
     */
    BezierCurve &get_bezier_curve(void)
    {
        if (index_ >= bezier_curve_num && bezier_curve_num > 0)
        {
            return bezier_curve_list[bezier_curve_num - 1];
        }
        return bezier_curve_list[index_];
    }

    Vector2D Get_Tangent_Vector()
    {
        return v_tangent_;
    }

    Vector2D Get_End_Point()
    {
        return end_point;
    }

    int Get_Index()
    {
        return index_;
    }

    bool Get_Curve_Flag()
    {
        return tangent_lock;
    }

protected:
    bool tangent_lock = false;
    //float dead = 0.05f;
    float dead = 0.03f;

    float brake_distance=0.0f;
    float brake_coefficient=1.1f;

    int index_ = 0;
    BezierCurve bezier_curve_list[MAX_CURVE_NUM]; // 储存各路段曲线

    Speedplanner_1D_Param_Config params_[MAX_CURVE_NUM]; // 每条曲线对应的速度规划参数
    TrapePlanner1D sp_;                                  // 一绿 S 型速度规划噿

    float distance_ = 0.0f;
    float t_ = 0.0f;                             // 贝忞尔曲线参数 t
    float v_resultant_ = 0.0f;                   // 当前速度
    Vector2D v_tangent_ = Vector2D(0.0f, 0.0f);  // 切线向量
    Vector2D point_last_ = Vector2D(0.0f, 0.0f); // 上一丿炿
    Vector2D v_output_ = Vector2D(0.0f, 0.0f);
    Phase m_phase = FINISHED_PHASE;

    uint8_t bezier_curve_num = 0; // 总曲线数釿

    Vector2D end_point = Vector2D(0.0f, 0.0f);

private:
    /*---------------------------------状怿-------------------------------------*/
    float err_end = 0.0f;
    bool is_end = false;  // 昿否开姿
    bool is_init = false; // 昿否初始化
};

///////////////////////////             前期测试产物                //////////////////////////////////////////
/**
 * @class Path_Bezier
 * @brief 基于贝�?�尔曲线的路径�?�划�?
 *
 * 该类实现了路径�?�划功能，包�?�?径点的�?�算�?
 * �?径重�?以及�?径更新�?
 */
// class Path_Bezier
//{
// public:
//     /**
//      * @brief 默�?�构造函�?
//      */
//     Path_Bezier();

//    /**
//     * @brief 一阶贝塞尔曲线构造函�?
//     * @param start_point 起点
//     * @param end_point 终点
//     * @param params 速度规划参数
//     */
//    Path_Bezier(Vector2D start_point, Vector2D end_point, Speedplanner_1D_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.00001f}) : bc_(start_point, end_point)
//    {
//        initial_setting(params);
//        end_point_ = end_point;    // 设置终点
//        point_last_ = start_point; // 设置上一�?点为起点
//        point_last_ = start_point; // 设置上一�?点为起点
//    }

//    /**
//     * @brief 二阶贝�?�尔曲线构造函�?
//     * @param start_point 起点
//     * @param control_point 控制�?
//     * @param end_point 终点
//     * @param params 速度规划参数
//     */
//    Path_Bezier(Vector2D start_point, Vector2D control_point, Vector2D end_point, Speedplanner_1D_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.00001f}) : bc_(start_point, control_point, end_point)
//    {
//        initial_setting(params);
//        end_point_ = end_point;    // 设置终点
//        point_last_ = start_point; // 设置上一�?点为起点
//        point_last_ = start_point; // 设置上一�?点为起点
//    }

//    /**
//     * @brief 规划�?径点
//     * @param point 当前�?
//     * @return Vector2D 返回规划后的速度向量
//     */
//    Vector2D plan(Vector2D point)
//    {
//        bc_.Get_Nearest_Distance(point, &t_); // 获取点到曲线的最近距�?
//        distance_ = bc_.Get_Current_Len(t_);
//        v_resultant_ = sp_.plan(distance_); // 计算当前速度
//        m_phase = sp_.getPhase();           // 获取当前阶�??

//        v_tangent_ = (bc_.Get_Tangent_Vector(t_)).normalize(); // 计算切线向量
//        point_last_ = point;                                   // 更新上一�?�?
//        return (v_tangent_ * v_resultant_);                    // 返回速度向量
//    }

//    /**
//     * @brief 重置�?径�?�划�?
//     */
//    void reset(void)
//    {
//        m_phase = S_ACCEL_JERK_UP_PHASE;     // 重置阶�?�为加速阶�?
//        point_last_ = bc_.Get_Start_point(); // 重置上一�?点为起点
//        distance_ = 0.0f;                    // 重置距�??
//        t_ = 0.0f;                           // 重置参数 t
//        v_resultant_ = 0.0f;                 // 重置速度
//    }

//    /**
//     * @brief 更新一阶贝塞尔曲线
//     * @param start_point 起点
//     * @param end_point 终点
//     */
//    void update(Vector2D start_point, Vector2D end_point, Speedplanner_1D_Param_Config params)
//    {
//        initial_setting(params);
//        end_point_ = end_point;                    // 设置终点
//        point_last_ = start_point;                 // 设置上一�?点为起点
//        bc_.Bezier_Update(start_point, end_point); // 更新贝�?�尔曲线
//    }

//    /**
//     * @brief 更新二阶贝�?�尔曲线
//     * @param start_point 起点
//     * @param control_point 控制�?
//     * @param end_point 终点
//     */
//    void update(Vector2D start_point, Vector2D control_point, Vector2D end_point, Speedplanner_1D_Param_Config params)
//    {
//        initial_setting(params);
//        end_point_ = end_point;                                   // 设置终点
//        point_last_ = start_point;                                // 设置上一�?点为起点
//        bc_.Bezier_Update(start_point, control_point, end_point); // 更新贝�?�尔曲线
//    }

//    /**
//     * @brief 判断�?径�?�划�?否完�?
//     * @return true 如果完成
//     * @return false 如果�?完成
//     */
//    bool isFinished() { return m_phase == S_FINISHED_PHASE; }

//    /**
//     * @brief 获取贝�?�尔曲线对象
//     * @return BezierCurve& 返回贝�?�尔曲线的引�?
//     */
//    BezierCurve &get_bezier_curve(void)
//    {
//        return bc_;
//    }

// private:
//     void initial_setting(Speedplanner_1D_Param_Config params)
//     {
//         m_phase = S_ACCEL_JERK_UP_PHASE;                         // 初�?�化阶�?�为加速阶�?
//         params.initialSpeed = abs(params.initialSpeed) + 0.001f; // 设置初�?�速度
//         params.startPos = 0.0f;                                  // 设置起�?�位�?
//         params.targetPos = bc_.Get_len();                        // 设置�?标位�?为曲线长�?
//         sp_.param_reset(params);                                 // 重置速度规划参数
//     }
//     SPhase m_phase = S_FINISHED_PHASE;           // 当前规划所处的阶�??
//     BezierCurve bc_;                             // 贝�?�尔曲线对象
//     SShapedPlanner1D sp_;                        // 一�? S 型速度规划�?
//     float t_ = 0.0f;                             // 贝�?�尔曲线参数 t
//     float v_resultant_ = 0.0f;                   // 当前速度
//     float distance_ = 0.0001f;                   // 当前距�??
//     Vector2D v_tangent_ = Vector2D(0.0f, 0.0f);  // 切线向量
//     Vector2D point_last_ = Vector2D(0.0f, 0.0f); // 上一�?�?
//     Vector2D end_point_ = Vector2D(0.0f, 0.0f);  // 终点
// };

// �?径（从静止启动到静�??�?
// class Path
//{
// public:
//    /**
//     * @brief 默�?�构造函�?
//     */
//    Path();
//    /**
//     * @brief 添加途径�?
//     * @param point_ 途径点坐�?
//     * @param smoothness_ 平滑度参数，范围[0, 0.5]
//     * @return true 如果添加成功
//     * @return false 如果添加失败
//     */
//    bool Add_Point(Vector2D point_, float smoothness_); // smoothness_ 0~0.5

//    /**
//     * @brief 添加起�?�点
//     * @param point_ 起�?�点坐标
//     * @param have_start_angle_ �?否有起�?��?�度
//     * @param start_angle_ 起�?��?�度�?
//     * @return true 如果添加成功
//     * @return false 如果添加失败
//     */
//    bool Add_Start_Point(Vector2D point_, bool have_start_angle_, float start_angle_, Speedplanner_1D_Param_Config params);

//    /**
//     * @brief 添加结束�?
//     * @param point_ 结束点坐�?
//     * @param end_angle_ 结束角度�?
//     * @return true 如果添加成功
//     * @return false 如果添加失败
//     */
//    bool Add_End_Point(Vector2D point_, float end_angle_);

//    /**
//     * @brief 获取�?径�??�?和向�?
//     * @param location_ 当前坐标
//     * @param yaw 当前�?向�??
//     * @param target_yaw 输出�?标航向�??
//     * @param normal_error 输出法向�?�?
//     * @param tangent_error 输出切向�?�?
//     * @param normal_vector 输出法向�?
//     * @param tangent_vector 输出切向�?
//     * @param max_vel 输出最大速度限制
//     * @return true 如果获取成功
//     * @return false 如果获取失败
//     */
//    bool Get_Error_And_Vector(
//        Vector2D location_,
//        float yaw,
//        float *target_yaw,
//        float *normal_error,
//        float *tangent_error,
//        Vector2D *normal_vector,
//        Vector2D *tangent_vector,
//        float *max_vel);

//    /**
//     * @brief �?径�?�划计算
//     * @param point 当前位置�?
//     * @return Vector2D 返回期望的速度向量
//     */
//    Vector2D plan(Vector2D point);

//    /**
//     * @brief 重置规划状�?
//     */
//    void plan_reset();
//    /**
//     * @brief 获取�?径是否结�?
//     * @return true 如果�?径结�?
//     * @return false 如果�?径未结束
//     */
//    bool Is_End() { return is_end; }

//    /**
//     * @brief 重置�?�?
//     */
//    void Reset();

//    /**
//     * @brief 获取当前阶�??
//     * @return 当前阶�??
//     */
//    SPhase getPhase() const { return m_phase; }

//    /**
//     * @brief 判断规划�?否已完成
//     * @return 如果规划已完成则返回 true，否则返�? false
//     */
//    bool isFinished() { return m_phase == S_FINISHED_PHASE; }

//    /**
//     * @brief 获取当前贝�?�尔曲线对象
//     * @return BezierCurve& 返回当前贝�?�尔曲线的引�?
//     */
//    BezierCurve &get_bezier_curve(void)
//    {
//        if (index_ >= bezier_curve_num && bezier_curve_num > 0)
//        {
//            return bezier_curve_list[bezier_curve_num - 1];
//        }
//        return bezier_curve_list[index_];
//    }

// protected:
//     float err_end = 0.0f; // �?�?�?�?
//     float dead = 0.04f;
//     Vector2D v_output_ = Vector2D(0.0f, 0.0f);
//     // 旧代�?
//     BezierCurve bezier_curve_list[MAX_CURVE_NUM]; // 储存各路段曲�?
//     SShapedPlanner1D sp_;                         // 一�? S 型速度规划�?
//     float total_len = 0;                          // �?线总长�?

//    int index_ = 0;
//    float total_ = 0.0f;
//    float distance_ = 0.0f;
//    float t_ = 0.0f;                             // 贝�?�尔曲线参数 t
//    float v_resultant_ = 0.0f;                   // 当前速度
//    Vector2D v_tangent_ = Vector2D(0.0f, 0.0f);  // 切线向量
//    Vector2D point_last_ = Vector2D(0.0f, 0.0f); // 上一�?�?
//    SPhase m_phase = S_FINISHED_PHASE;
//    Speedplanner_1D_Param_Config params_; // 每条曲线对应的速度规划参数
//    uint8_t bezier_curve_num = 0;         // 总曲线数�?

//    /**
//     * @brief 计算各路段的结束速度
//     */
//    void Calc_End_Vel();

//    /**
//     * @brief 生成各路段的曲线
//     * @param point_ 当前点坐�?
//     * @param smoothness_ 平滑度参�?
//     * @return true 如果生成成功
//     * @return false 如果生成失败
//     */
//    bool Generate_Curve(Vector2D point_, float smoothness_);

//    /*------------------------------�?径参�?-----------------------------------*/
//    bool have_start_angle = 0; // �?否�?�求达到起�?��?�度后再�?�?
//    float start_angle = 0;     // 起�?��?�度
//    float end_angle = 0;       // 结束角度

//    /*------------------------------过程变量-----------------------------------*/
//    float currnet_target_angle = 0; // 当前�?标�?�度

//    uint8_t current_bezier_curve_dx = 0; // 当前�?段�?�应曲线的索�?

//    float current_t = 0; // 当前坐标对应当前�?段的t�?

//    float current_finished_len = 0; // 已完成的曲线的总长�?

//    float current_curve_len = 0; // 当前曲线走过的长�?

// private:
//     /*---------------------------------状�?-------------------------------------*/
//     bool is_end = false;   // �?否开�?
//     bool is_start = false; // �?否结�?

//    bool is_init = false; // �?否初始化

//    /*----------------------------生成�?径的临时变量-------------------------------------*/
//    Generate_Curve_Status generate_status = GENERATE_WAIT_FIRST_POINT; // 生成曲线的状�?

//    Vector2D point_list[3]; // 生成曲线时临时存放点坐标

//    float last_smoothness; // 生成曲线时临时存放上一�?点的平滑程度
//};

// class Path_line
//{
// public:
//     Path_line()
//     {
//         bezier_curve_num = 0;
//         is_init = false;
//     }
//     bool Add_Point(Vector2D point_, Speedplanner_1D_Param_Config params)
//     {
//         if (is_init == true)
//             return false;
//         params_[bezier_curve_num] = params;
//         bezier_curve_list[bezier_curve_num].Bezier_Update(point_last_, point_);
//         bezier_curve_num++;
//         point_last_ = point_;
//         params.initialSpeed = abs(params.initialSpeed) + 0.001f; // 设置初�?�速度

//        return true;
//    }
//    bool Add_Point(Vector2D point_, Vector2D control_point_, Speedplanner_1D_Param_Config params)
//    {
//        if (is_init == true)
//            return false;
//        params_[bezier_curve_num] = params;
//        bezier_curve_list[bezier_curve_num].Bezier_Update(point_last_, control_point_, point_);
//        bezier_curve_num++;
//        point_last_ = point_;
//        params.initialSpeed = abs(params.initialSpeed) + 0.001f; // 设置初�?�速度

//        return true;
//    }

//    bool Add_Start_Point(Vector2D point_)
//    {

//        if (is_init == true)
//            return false;
//        point_last_ = point_;
//        return true;
//    }

//    bool Add_End_Point(Vector2D point_, Speedplanner_1D_Param_Config params)
//    {
//        if (is_init == true)
//            return false;

//        is_init = true;
//        is_end = true;
//        params_[bezier_curve_num] = params;
//        bezier_curve_list[bezier_curve_num].Bezier_Update(point_last_, point_);
//        bezier_curve_num++;
//        index_ = 0;
//        params_[index_].targetPos = (bezier_curve_list[index_].Get_len() - params_[index_].startPos);
//        params_[index_].startPos = 0.0f; // 设置起�?�位�?
//        sp_.param_reset(params_[index_]);
//        m_phase = S_ACCEL_JERK_UP_PHASE; // 初�?�化阶�?�为加速阶�?
//        return true;
//    }

//    /**
//     * @brief �?径�?�划计算
//     * @param point 当前位置�?
//     * @return Vector2D 返回期望的速度向量
//     */
//    Vector2D plan(Vector2D point)
//    {
//        if (Is_End() == true)
//        {
//            bezier_curve_list[index_].Get_Nearest_Distance(point, &t_); // 获取点到曲线的最近距�?
//            distance_ = bezier_curve_list[index_].Get_Current_Len(t_);

//            v_resultant_ = sp_.plan(distance_); // 速度规划器�?�算当前�?标速度
//            m_phase = sp_.getPhase();           // 获取当前速度规划阶�??

//            v_tangent_ = (bezier_curve_list[index_].Get_Tangent_Vector(t_)).normalize(); // 计算切线向量（单位向量）

//            err_end = _tool_Abs((point - bezier_curve_list[index_].Get_End_point()).magnitude());

//            // 段切换条件：
//            // 1) 近�??�?�?达到阈值可直接切�?�；
//            // 2) t 接近 1 仅作为辅助条件，必须同时离终点不远，避免“投影到段末�?但车体仍较远”时�?切�?��?
//            const float t_reach_guard = 0.20f;
//            bool reach_segment_end = (_tool_Abs(err_end) <= dead) || (t_ >= 0.995f && _tool_Abs(err_end) <= t_reach_guard);
//            // bool reach_segment_end = (t_ >= 0.995f);
//            if (reach_segment_end)
//            {
//                index_++; // 切换到下一段曲�?
//                t_ = 0.0f;
//                if (index_ >= bezier_curve_num)
//                {
//                    is_end = false; // 结束运�??
//                    m_phase = S_FINISHED_PHASE;
//                }
//                else
//                {
//                    params_[index_].targetPos = (bezier_curve_list[index_].Get_len() - params_[index_].startPos);
//                    params_[index_].startPos = 0.0f; // 设置起�?�位�?
//                    sp_.param_reset(params_[index_]);
//                }
//            }
//            v_output_ = (v_tangent_ * v_resultant_);
//            return v_output_; // 返回 速度向量 = 切向方向 * �?标速率
//        }
//        else
//        {
//            return Vector2D{0, 0};
//        }
//    }

//    /**
//     * @brief 重置规划状�?
//     */
//    void plan_reset()
//    {
//        is_init = false; // 重置初�?�化标志

//        bezier_curve_num = 0; // 重置曲线数量

//        is_end = false; // 重置�?径结束标�?
//    }

//    /**
//     * @brief 获取�?径是否结�?
//     * @return true 如果�?径结�?
//     * @return false 如果�?径未结束
//     */
//    bool Is_End() { return is_end; }

//    /**
//     * @brief 重置�?�?
//     */
//    void Reset()
//    {
//        index_ = 0;
//        point_last_ = bezier_curve_list[index_].Get_Start_point(); // 重置上一�?点为起点
//        m_phase = S_ACCEL_JERK_UP_PHASE;
//        distance_ = 0.0f;    // 重置距�??
//        t_ = 0.0f;           // 重置参数 t
//        v_resultant_ = 0.0f; // 重置速度
//        is_end = true;
//    }

//    /**
//     * @brief 获取当前阶�??
//     * @return 当前阶�??
//     */
//    SPhase getPhase() const { return m_phase; }

//    /**
//     * @brief 判断规划�?否已完成
//     * @return 如果规划已完成则返回 true，否则返�? false
//     */
//    bool isFinished() { return m_phase == S_FINISHED_PHASE; }

//    /**
//     * @brief 获取当前贝�?�尔曲线对象
//     * @return BezierCurve& 返回当前贝�?�尔曲线的引�?
//     */
//    BezierCurve &get_bezier_curve(void)
//    {
//        if (index_ >= bezier_curve_num && bezier_curve_num > 0)
//        {
//            return bezier_curve_list[bezier_curve_num - 1];
//        }
//        return bezier_curve_list[index_];
//    }

//    Vector2D Get_Tangent_Vector()
//    {
//        return v_tangent_;
//    }

// protected:
//     float dead = 0.05f;
//     int index_ = 0;
//     BezierCurve bezier_curve_list[MAX_CURVE_NUM]; // 储存各路段曲�?

//    Speedplanner_1D_Param_Config params_[MAX_CURVE_NUM]; // 每条曲线对应的速度规划参数
//    SShapedPlanner1D sp_;                                // 一�? S 型速度规划�?

//    float distance_ = 0.0f;
//    float t_ = 0.0f;                             // 贝�?�尔曲线参数 t
//    float v_resultant_ = 0.0f;                   // 当前速度
//    Vector2D v_tangent_ = Vector2D(0.0f, 0.0f);  // 切线向量
//    Vector2D point_last_ = Vector2D(0.0f, 0.0f); // 上一�?�?
//    Vector2D v_output_ = Vector2D(0.0f, 0.0f);
//    SPhase m_phase = S_FINISHED_PHASE;

//    uint8_t bezier_curve_num = 0; // 总曲线数�?

// private:
//     float err_end = 0.0f;
//     bool is_end = false;  // �?否开�?
//     bool is_init = false; // �?否初始化
// };

#endif

#endif