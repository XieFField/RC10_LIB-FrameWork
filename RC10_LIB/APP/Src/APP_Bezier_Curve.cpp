// 包含贝塞尔曲线类的定义
#include "APP_Bezier_Curve.h"

/**
 * @brief 默认构造函数
 */
BezierCurve::BezierCurve()
{
	// 空实现
}

/**
 * @brief 一阶贝塞尔曲线构造函数
 * @param start_point_ 起始点
 * @param end_point_ 结束点
 */
BezierCurve::BezierCurve(Vector2D start_point_, Vector2D end_point_)
{
	Bezier_Update(start_point_, end_point_); // 初始化一阶贝塞尔曲线
}

/**
 * @brief 二阶贝塞尔曲线构造函数
 * @param start_point_ 起始点
 * @param control_point_ 控制点
 * @param end_point_ 结束点
 */
BezierCurve::BezierCurve(Vector2D start_point_, Vector2D control_point_, Vector2D end_point_)
{
	Bezier_Update(start_point_, control_point_, end_point_); // 初始化二阶贝塞尔曲线
}

/**
 * @brief 更新一阶贝塞尔曲线
 * @param start_point_ 起始点
 * @param end_point_ 结束点
 */
void BezierCurve::Bezier_Update(Vector2D start_point_, Vector2D end_point_)
{
	order = FIRST_ORDER_BEZIER; // 设置曲线阶数为一阶

	start_point = start_point_; // 设置起始点
	end_point = end_point_;		// 设置结束点

	len = 0; // 初始化曲线长度

	tangent_vector = (end_point - start_point).normalize(); // 计算单位切向量

	len = (end_point - start_point).magnitude(); // 计算曲线长度
}

/**
 * @brief 更新二阶贝塞尔曲线
 * @param start_point_ 起始点
 * @param control_point_ 控制点
 * @param end_point_ 结束点
 */
void BezierCurve::Bezier_Update(Vector2D start_point_, Vector2D control_point_, Vector2D end_point_)
{
	order = SECOND_ORDER_BEZIER; // 设置曲线阶数为二阶

	start_point = start_point_;		// 设置起始点
	control_point = control_point_; // 设置控制点
	end_point = end_point_;			// 设置结束点

	len = 0;	 // 初始化曲线长度
	end_vel = 0; // 初始化结束速度

	float temp_t = 0;	// 临时变量 t
	float temp_len = 0; // 临时变量长度

	// 计算初始曲率和最大速度
	float temp_curature = Get_Curvature(0.f);

	if (temp_curature < 1e-6f)
	{
		max_vel_list[0] = 1e6f; // 曲率接近 0，速度无限制
	}
	else if (temp_curature > 1e6f)
	{
		max_vel_list[0] = 0.1f; // 曲率过大，限制速度
	}
	else
	{
		arm_sqrt_f32(1.f / temp_curature, &max_vel_list[0]); // 根据曲率计算最大速度
	}

	max_curvature_len = 0.f;				 // 初始化最大曲率对应的长度
	max_curvature_max_vel = max_vel_list[0]; // 初始化最大曲率对应的最大速度

	// 采样计算曲线长度和最大速度
	for (uint8_t i = 0; i < BEZIER_SAMPLE_NUM; i++)
	{
		if (i < BEZIER_SAMPLE_NUM - 1)
		{
			temp_len = (Get_Point(temp_t + bezier_sample_step) - Get_Point(temp_t)).magnitude();
			temp_t += bezier_sample_step;
			temp_curature = Get_Curvature(temp_t);
		}
		else
		{
			temp_len = (end_point - Get_Point(temp_t)).magnitude();
			temp_curature = Get_Curvature(1.f);
		}

		len += temp_len;		// 累加曲线长度
		distance_list[i] = len; // 储存采样点的累计长度

		if (temp_curature < 1e-6f)
		{
			max_vel_list[i + 1] = 1e6f; // 曲率接近 0，速度无限制
		}
		else if (temp_curature > 1e6f)
		{
			max_vel_list[i + 1] = 0.1f; // 曲率过大，限制速度
		}
		else
		{
			arm_sqrt_f32(1.f / temp_curature, &max_vel_list[i + 1]);
		}

		// 更新最大曲率对应的速度和长度
		if (max_vel_list[i + 1] < max_curvature_max_vel)
		{
			max_curvature_len = len;
			max_curvature_max_vel = max_vel_list[i + 1];
		}
	}
}

/**
 * @brief 获取曲线上的点
 * @param t 曲线参数，范围 [0, 1]
 * @return Vector2D 曲线上的点
 */
Vector2D BezierCurve::Get_Point(const float t)
{
	if (t <= 0.f)
		return start_point; // 返回起始点
	else if (t >= 1.f)
		return end_point; // 返回结束点

	if (order == FIRST_ORDER_BEZIER)
	{
		return Vector2D::lerp(start_point, end_point, t); // 一阶贝塞尔曲线
	}
	else
	{
		// 二阶贝塞尔曲线
		return Vector2D::lerp(
			Vector2D::lerp(start_point, control_point, t),
			Vector2D::lerp(control_point, end_point, t),
			t);
	}
}

/**
 * @brief 获取最近点的距离并输出对应的 t 值
 * @param point 目标点
 * @param t 输出参数，对应最近点的曲线参数
 * @return float 最近点的距离
 */
float BezierCurve::Get_Nearest_Distance(const Vector2D point, float *t)
{
	if (order == FIRST_ORDER_BEZIER)
	{
		const Vector2D d = end_point - start_point; // 直线方向向量
		const Vector2D v = point - start_point;		// 起点到目标点的向量

		const float dot_vd = v * d; // 点积
		const float dot_dd = d * d; // 方向向量的模平方

		if (dot_dd < 1e-9f)
		{
			if (t != nullptr)
			{
				*t = 0.f;
			}
			return v.magnitude(); // 线段长度接近 0，返回目标点到起点的距离
		}

		const float t_val = dot_vd / dot_dd; // 计算 t 值

		if (t_val <= 0.0f)
		{
			if (t != nullptr)
			{
				*t = 0.0f;
			}
			return v.magnitude(); // t 小于 0，返回目标点到起点的距离
		}
		else if (t_val >= 1.0f)
		{
			if (t != nullptr)
			{
				*t = 1.0f;
			}
			return (point - end_point).magnitude(); // t 大于 1，返回目标点到结束点的距离
		}

		// 设置输出参数 t
		if (t != nullptr)
		{
			*t = t_val;
		}

		const Vector2D offset = v - d * t_val; // 计算目标点到最近点的偏移量

		return offset.magnitude(); // 返回距离
	}
	else
	{
		// 黄金分割法查找最近点
		float left = 0.f;
		float right = 1.f;

		// 初始化黄金分割点
		float t1 = 1.f - GOLDEN_RATIO;
		float t2 = GOLDEN_RATIO;

		// 计算初始点的距离平方
		float d1 = Vector2D::distanceSquared(Get_Point(t1), point);
		float d2 = Vector2D::distanceSquared(Get_Point(t2), point);

		// 迭代收缩区间
		for (uint8_t i = 0; i < FIND_NEAREST_DISTANCE_STEP_COUNT; i++)
		{
			if (d1 < d2)
			{
				// 左侧区间更优，收缩右边界
				right = t2;
				t2 = t1;
				d2 = d1;
				t1 = right - GOLDEN_RATIO * (right - left);
				d1 = Vector2D::distanceSquared(Get_Point(t1), point);
			}
			else
			{
				// 右侧区间更优，收缩左边界
				left = t1;
				t1 = t2;
				d1 = d2;
				t2 = left + GOLDEN_RATIO * (right - left);
				d2 = Vector2D::distanceSquared(Get_Point(t2), point);
			}
		}

		// 找到最优值 t1 和 d1（重复利用）
		if (d1 > d2)
		{
			d1 = d2;
			t1 = t2;
		}

		d2 = Vector2D::distanceSquared(Get_Point(left), point);

		if (d2 < d1)
		{
			d1 = d2;
			t1 = left;
		}

		d2 = Vector2D::distanceSquared(Get_Point(right), point);

		if (d2 < d1)
		{
			d1 = d2;
			t1 = right;
		}

		if (t != nullptr)
		{
			*t = t1;
		}

		// 返回实际距离（开方得到真实距离）
		return sqrtf(d1);
	}
}

/**
 * @brief 获取当前位置走过的长度
 * @param t 曲线参数，范围 [0, 1]
 * @return float 当前长度
 */
float BezierCurve::Get_Current_Len(float t)
{
	if (t <= 0.f)
		return 0.f;
	else if (t >= 1.f)
		return len;

	if (order == FIRST_ORDER_BEZIER)
	{
		return (Get_Point(t) - start_point).magnitude(); // 一阶贝塞尔曲线，直接计算起始点到当前点的距离
	}
	else
	{
		uint8_t distance_list_dx = (uint8_t)(t / bezier_sample_step); // 获取索引

		float p = (t - (float)distance_list_dx * bezier_sample_step) / bezier_sample_step; // 线性插值比例

		if (distance_list_dx == 0)
		{
			return distance_list[0] * p; // 特殊处理第一个区间
		}
		else if (distance_list_dx < BEZIER_SAMPLE_NUM)
		{
			// 在两个采样点之间，进行线性插值
			return distance_list[distance_list_dx - 1] * (1.f - p) + distance_list[distance_list_dx] * p;
		}
		else
		{
			return len; // 超出范围，返回总长度
		}
	}
}

/**
 * @brief 获取单位切向量
 * @param t 曲线参数，范围 [0, 1]
 * @return Vector2D 单位切向量
 */
Vector2D BezierCurve::Get_Tangent_Vector(const float t)
{
	// 一阶直线的方向向量已经提前储存
	if (order != FIRST_ORDER_BEZIER)
	{
		tangent_vector = (Get_Point(t + 0.01f) - Get_Point(t - 0.01f)).normalize(); // 通过前后点计算切向量
	}

	return tangent_vector;
}

/**
 * @brief 获取曲率
 * @param t 曲线参数，范围 [0, 1]
 * @return float 曲率值
 */
float BezierCurve::Get_Curvature(float t)
{
	// 处理 t 值边界情况
	if (t < 0.f)
		t = 0.f;
	else if (t > 1.f)
		t = 1.f;

	if (order == FIRST_ORDER_BEZIER)
	{
		return 0.f; // 一阶贝塞尔曲线(直线)的曲率恒为0
	}
	else // 二阶贝塞尔曲线曲率计算
	{
		// p0 = start_point;
		// p1 = control_point;
		// p2 = end_point;

		// 一阶导数：B'(t) = 2(1-t)(P?-P?) + 2t(P?-P?) = 2(P?-P?) + 2t(P?-2P?+P?)
		Vector2D A = (control_point - start_point) * 2.f;
		Vector2D B = (end_point - (control_point * 2.f) + start_point) * 2.f;

		// 计算t处的一阶导数(dx, dy)
		Vector2D first_deriv = A + B * t;
		float dx = first_deriv.x;
		float dy = first_deriv.y;

		// 检查一阶导数是否为零
		if (fabsf(dx) < 1e-6f && fabsf(dy) < 1e-6f)
		{
			return 0.f; // 导数为零，曲率为0
		}

		// 计算二阶导数(ddx, ddy) - 二阶导数是常数
		Vector2D second_deriv = (end_point - control_point * 2.f + start_point) * 2.f; // B''(t) = 2(P?-2P?+P?)
		float ddx = second_deriv.x;
		float ddy = second_deriv.y;

		// 曲率公式分子: |dx*ddy - dy*ddx|
		float numerator = fabsf(dx * ddy - dy * ddx);

		// 曲率公式分母: (dx2 + dy2)^(3/2)
		float len_squared = dx * dx + dy * dy;

		// 避免除以零 (处理奇点情况)
		if (len_squared < 1e-6f)
		{
			return 1e6f; // 返回一个大数表示曲率很大
		}

		float denominator = powf(len_squared, 1.5f);

		float curvature = numerator / denominator;

		// 限制曲率在合理范围内
		if (curvature > 1e6f)
			curvature = 1e6f;
		if (curvature < 0.f)
			curvature = 0.f;

		return curvature;
	}
}

/**
 * @brief 获取最大速度
 * @param t 曲线参数，范围 [0, 1]
 * @return float 最大速度
 */
float BezierCurve::Get_Max_Vel(float t)
{
	if (t < 0.f)
		t = 0.f;
	else if (t > 1.f)
		t = 1.f;

	float temp_current_len = Get_Current_Len(t);

	arm_sqrt_f32(fabsf((len - temp_current_len) * 2.f + end_vel * end_vel), &current_max_vel); // 获取当前规划到结束点的最大速度

	if (order == FIRST_ORDER_BEZIER)
	{
		// 曲率为0
	}
	else
	{
		uint8_t max_vel_list_dx = (uint8_t)(t / bezier_sample_step); // 获取索引

		float p = (t - (float)max_vel_list_dx * bezier_sample_step) / bezier_sample_step; // 线性插值比例

		float temp_max_vel;

		if (max_vel_list_dx < BEZIER_SAMPLE_NUM)
		{
			// 获取当前曲率下最大速度
			temp_max_vel = max_vel_list[max_vel_list_dx] * (1.f - p) + max_vel_list[max_vel_list_dx + 1] * p;
		}
		else
		{
			temp_max_vel = max_vel_list[BEZIER_SAMPLE_NUM]; // 超出范围，使用最后一个最大速度
		}

		current_max_vel = current_max_vel > temp_max_vel ? temp_max_vel : current_max_vel; // 取最小值

		if (temp_current_len < max_curvature_len)
		{
			arm_sqrt_f32(fabsf((max_curvature_len - temp_current_len) * 2.f + max_curvature_max_vel * max_curvature_max_vel), &temp_max_vel); // 获取当前规划到曲率最大点的最大速度

			current_max_vel = current_max_vel > temp_max_vel ? temp_max_vel : current_max_vel; // 取最小值
		}
	}

	return current_max_vel;
}

/**
 * @brief 获取法向量
 * @param point 目标点
 * @param t 曲线参数，范围[0, 1]
 * @return Vector2D 法向量
 */
Vector2D BezierCurve::Get_Normal_Vector(const Vector2D &point, const float t)
{
	if (order != FIRST_ORDER_BEZIER)
	{
		normal_vector = Get_Tangent_Vector(t).perpendicular(); // 同时更新tangent_vector;
	}

	float cross_result; // 叉乘结果

	if (order == FIRST_ORDER_BEZIER)
	{
		cross_result = tangent_vector.cross(point - start_point);
	}
	else
	{
		cross_result = tangent_vector.cross(point - Get_Point(t));
	}

	if (cross_result > 0)
	{
		return normal_vector;
	}
	else if (cross_result < 0)
	{
		return -normal_vector;
	}
	else
	{
		return Vector2D(0, 0);
	}
}