/**
 * @file APP_Path.h
 * @author naoganlin
 * @brief 贝塞尔曲线库
 * @version 1.0
 * @date 2025-10-30
 */

#ifndef __APP_BEZIER_CURVE_H
#define __APP_BEZIER_CURVE_H

#pragma once
#include "APP_Vector2D.h" // 包含向量类的定义

#ifdef __cplusplus

// 常量定义
#define FIND_NEAREST_DISTANCE_STEP_COUNT 100  // 查找最近点的迭代次数
#define BEZIER_SAMPLE_NUM 100				  // 贝塞尔曲线采样点数量（不包含起点）
#define GOLDEN_RATIO (sqrtf(5.f) - 1.f) / 2.f // 黄金分割比例 (~0.618)

/**
 * @enum BezierOrder
 * @brief 贝塞尔曲线的阶数
 */
typedef enum BezierOrder : uint8_t
{
	FIRST_ORDER_BEZIER, // 一阶贝塞尔曲线（直线）
	SECOND_ORDER_BEZIER // 二阶贝塞尔曲线
} BezierOrder;

/**
 * @class BezierCurve
 * @brief 贝塞尔曲线类
 *
 * 该类实现了贝塞尔曲线的相关计算，包括点、切向量、曲率等。
 */
class BezierCurve
{
public:
	/**
	 * @brief 默认构造函数
	 */
	BezierCurve();

	/**
	 * @brief 一阶贝塞尔曲线构造函数
	 * @param start_point_ 起点
	 * @param end_point_ 终点
	 */
	BezierCurve(Vector2D start_point_, Vector2D end_point_);

	/**
	 * @brief 二阶贝塞尔曲线构造函数
	 * @param start_point_ 起点
	 * @param control_point_ 控制点
	 * @param end_point_ 终点
	 */
	BezierCurve(Vector2D start_point_, Vector2D control_point_, Vector2D end_point_);

	/**
	 * @brief 更新一阶贝塞尔曲线
	 * @param start_point_ 起点
	 * @param end_point_ 终点
	 */
	void Bezier_Update(Vector2D start_point_, Vector2D end_point_);

	/**
	 * @brief 更新二阶贝塞尔曲线
	 * @param start_point_ 起点
	 * @param control_point_ 控制点
	 * @param end_point_ 终点
	 */
	void Bezier_Update(Vector2D start_point_, Vector2D control_point_, Vector2D end_point_);

	/**
	 * @brief 获取控制点（仅适用于二阶曲线）
	 * @return Vector2D 控制点
	 */
	Vector2D Get_Control_Point() const { return control_point; }

	/**
	 * @brief 获取曲线上的点
	 * @param t 曲线参数，范围[0, 1]
	 * @return Vector2D 曲线上的点
	 */
	Vector2D Get_Point(float t);

	/**
	 * @brief 获取最近点的距离并输出对应的 t 值
	 * @param point 目标点
	 * @param t 输出参数，对应最近点的曲线参数
	 * @return float 最近点的距离
	 */
	float Get_Nearest_Distance(Vector2D point, float *t);

	/**
	 * @brief 获取当前位置走过的长度
	 * @param t 曲线参数，范围[0, 1]
	 * @return float 当前长度
	 */
	float Get_Current_Len(float t);

	/**
	 * @brief 获取单位切向量
	 * @param t 曲线参数，范围[0, 1]
	 * @return Vector2D 切向量
	 */
	Vector2D Get_Tangent_Vector(float t);

	/**
	 * @brief 获取法向量
	 * @param point 目标点
	 * @param t 曲线参数，范围[0, 1]
	 * @return Vector2D 法向量
	 */
	Vector2D Get_Normal_Vector(const Vector2D &point, const float t);

	/**
	 * @brief 获取起始点
	 * @return Vector2D 起始点
	 */
	Vector2D Get_Start_point() { return start_point; }

	/**
	 * @brief 获取结束点
	 * @return Vector2D 结束点
	 */
	Vector2D Get_End_point() { return end_point; }

	/**
	 * @brief 获取曲率
	 * @param t 曲线参数，范围[0, 1]
	 * @return float 曲率
	 */
	float Get_Curvature(float t);

	/**
	 * @brief 获取曲线总长度
	 * @return float 曲线总长度
	 */
	float Get_len() const { return len; }

	/**
	 * @brief 获取贝塞尔曲线的阶数
	 * @return BezierOrder 曲线阶数
	 */
	BezierOrder Get_Bezier_Order() { return order; }

	/**
	 * @brief 设置结束速度
	 * @param end_vel_ 结束速度
	 */
	void Set_End_Vel(float end_vel_)
	{
		end_vel = fabsf(end_vel_);
	}

	/**
	 * @brief 获取最大速度
	 * @param t 曲线参数，范围[0, 1]
	 * @return float 最大速度
	 */
	float Get_Max_Vel(float t);
    
    void Rest(void)
    {
        start_point={0.0f,0.0f};
        end_point={0.0f,0.0f};
        control_point={0.0f,0.0f};
        Bezier_Update(start_point,control_point,end_point);
    }

protected:
	float len = 0.0f;										   // 曲线总长度
	float distance_list[BEZIER_SAMPLE_NUM];					   // 储存采样点的累计长度
	float bezier_sample_step = 1.f / (float)BEZIER_SAMPLE_NUM; // 采样步长

	BezierOrder order; // 贝塞尔曲线的阶数

	Vector2D start_point, end_point; // 起始点和结束点
	Vector2D control_point;			 // 控制点（仅二阶曲线使用）

	Vector2D tangent_vector; // 切向量
	Vector2D normal_vector;	 // 法向量

private:
	float end_vel = 0;						   // 结束速度
	float max_vel_list[BEZIER_SAMPLE_NUM + 1]; // 储存每个采样点的最大速度

	float max_curvature_len;	 // 最大曲率对应的长度
	float max_curvature_max_vel; // 最大曲率对应的最大速度

	float current_max_vel; // 当前最大速度
};

#endif
#endif