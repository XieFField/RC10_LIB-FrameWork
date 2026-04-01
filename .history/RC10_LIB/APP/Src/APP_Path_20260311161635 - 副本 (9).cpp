#include "APP_Path.h"
#define TWO_PI 6.2831853071795864769f
#define HALF_PI 1.570796326794896619f
#define TWO_THIRD_PI 4.71238898038468985769f
/**
 * @brief 默认构造函数
 */
Path::Path()
{
	bezier_curve_num = 0;
	generate_status = GENERATE_WAIT_FIRST_POINT;
	have_start_angle = 0;
	start_angle = 0;
	end_angle = 0;
	total_len = 0;
	is_init = false;
	last_smoothness = 0;
	
}

/**
 * @brief 添加途径点
 * @param point_ 途径点坐标
 * @param smoothness_ 平滑度参数，范围[0, 0.5]
 * @return true 如果添加成功
 * @return false 如果添加失败
 */
bool Path::Add_Point(Vector2D point_, float smoothness_) // 0~0.5)
{
	if (is_init == true)
		return false;

	return Generate_Curve(point_, smoothness_);
}

/**
 * @brief 添加起始点
 * @param point_ 起始点坐标
 * @param have_start_angle_ 是否有起始角度
 * @param start_angle_ 起始角度值
 * @return true 如果添加成功
 * @return false 如果添加失败
 */
bool Path::Add_Start_Point(Vector2D point_, bool have_start_angle_, float start_angle_,Speedplanner_1D_Param_Config params)
{
	if (is_init == true)
		return false;

	have_start_angle = have_start_angle_;
	start_angle = start_angle_;
    
    params.initialSpeed = abs(params.initialSpeed) + 0.001f; // 设置初始速度
	params_ = params;

	return Generate_Curve(point_, 0);
}

/**
 * @brief 添加结束点
 * @param point_ 结束点坐标
 * @param end_angle_ 结束角度值
 * @return true 如果添加成功
 * @return false 如果添加失败
 */
bool Path::Add_End_Point(Vector2D point_, float end_angle_)
{
	if (is_init == true)
		return false;

	end_angle = end_angle_;

	is_init = true;

	if (Generate_Curve(point_, 0) == false)
		return false;

	Calc_End_Vel();
	is_end = true;
	params_.targetPos = (total_len - params_.startPos);
	params_.startPos = 0.0f; // 设置起始位置
	sp_.param_reset(params_);
	m_phase = S_ACCEL_JERK_UP_PHASE; // 初始化阶段为加速阶段
	point_last_ = bezier_curve_list[index_].Get_Start_point();
	// total_ = bezier_curve_list[index_].Get_len();
	total_ = 0.0f;
	return true;
}

/**
 * @brief 生成各路段的曲线
 * @param point_ 当前点坐标
 * @param smoothness_ 平滑度参数
 * @return true 如果生成成功
 * @return false 如果生成失败
 */
bool Path::Generate_Curve(Vector2D point_, float smoothness_)
{
	if (bezier_curve_num >= MAX_CURVE_NUM - 2)
		return false;

	if (smoothness_ < 0.f)
		smoothness_ = 0;
	else if (smoothness_ > 1.0f)
		smoothness_ = 1.0f;

	switch (generate_status)
	{
	case GENERATE_WAIT_FIRST_POINT: // 等待第一个点
		point_list[0] = point_;
		generate_status = GENERATE_FINISHED_STRAIGHT;
		break;

	case GENERATE_FINISHED_STRAIGHT: // 刚生成完直线
		if (smoothness_ == 0)
		{
			// 如果平滑度为0，直接生成直线（一阶贝塞尔曲线）连接上一个点和当前点
			bezier_curve_list[bezier_curve_num].Bezier_Update(point_list[0], point_); // 直线（一阶贝塞尔）
			total_len += bezier_curve_list[bezier_curve_num].Get_len();
			// 如果生成的线段长度有效，增加曲线计数
			if (bezier_curve_list[bezier_curve_num].Get_len() > 0.00001f)
			{
				bezier_curve_num++;
			}

			point_list[0] = point_; // 更新上一个点为当前点

			generate_status = GENERATE_FINISHED_STRAIGHT; // 状态保持为完成直线生成
		}
		else
		{
			// 如果平滑度不为0，计算过渡点
			Vector2D temp_point = Vector2D::lerp(point_, point_list[0], smoothness_); // 直线衔接曲线的过渡点（曲线起始点）
			last_smoothness = smoothness_;

			// 生成上一段直线到过渡点的直线段
			bezier_curve_list[bezier_curve_num].Bezier_Update(point_list[0], temp_point); // 曲线前的衔接直线
			total_len += bezier_curve_list[bezier_curve_num].Get_len();
			if (bezier_curve_list[bezier_curve_num].Get_len() > 0.00001f)
			{
				bezier_curve_num++;
			}

			point_list[0] = temp_point; // 记录曲线起始点
			point_list[1] = point_;		// 记录曲线控制点（即当前输入的点）

			generate_status = GENERATE_WAIT_LAST_CURVE_POINT; // 状态变更为等待曲线结束点
		}
		break;

	case GENERATE_WAIT_LAST_CURVE_POINT: // 等待曲线最后的结束点
		if (smoothness_ == 0)
		{
			if (last_smoothness == 1.0f) // 曲线间无需直线过渡
			{
				// 计算曲线结束点（下一段曲线起始点）
				Vector2D temp_point = Vector2D::lerp(point_list[1], point_, last_smoothness); // 曲线结束点（下一段曲线起始点）

				// 生成二阶贝塞尔曲线
				bezier_curve_list[bezier_curve_num].Bezier_Update(point_list[0], point_list[1], temp_point); // 曲线
				total_len += bezier_curve_list[bezier_curve_num].Get_len();
				if (bezier_curve_list[bezier_curve_num].Get_len() > 0.00001f)
				{
					bezier_curve_num++;
				}

				point_list[0] = point_; // 更新点
				generate_status = GENERATE_FINISHED_STRAIGHT; // 状态变更为完成直线
			}
			else
			{
				// 计算曲线结束点，也是衔接直线的起点
				Vector2D temp_point = Vector2D::lerp(point_list[1], point_, last_smoothness); // 曲线结束点，衔接直线过渡点

				// 生成二阶贝塞尔曲线
				bezier_curve_list[bezier_curve_num].Bezier_Update(point_list[0], point_list[1], temp_point); // 曲线（二阶贝塞尔）
				total_len += bezier_curve_list[bezier_curve_num].Get_len();
				if (bezier_curve_list[bezier_curve_num].Get_len() > 0.00001f)
				{
					bezier_curve_num++;
				}

				// 生成衔接直线
				bezier_curve_list[bezier_curve_num].Bezier_Update(temp_point, point_); // 直线
				total_len += bezier_curve_list[bezier_curve_num].Get_len();
				if (bezier_curve_list[bezier_curve_num].Get_len() > 0.00001f)
				{
					bezier_curve_num++;
				}
				point_list[0] = point_;
				generate_status = GENERATE_FINISHED_STRAIGHT;
			}
		}
		else
		{

			// 计算曲线衔接直线的过渡点
			Vector2D temp_point = Vector2D::lerp(point_list[1], point_, last_smoothness); // 曲线衔接直线的过渡点

			// 生成二阶贝塞尔曲线
			bezier_curve_list[bezier_curve_num].Bezier_Update(point_list[0], point_list[1], temp_point); // 曲线
			total_len += bezier_curve_list[bezier_curve_num].Get_len();
			if (bezier_curve_list[bezier_curve_num].Get_len() > 0.00001f)
			{
				bezier_curve_num++;
			}

			// 计算直线衔接下一段曲线的过渡点
			Vector2D temp_point_1 = Vector2D::lerp(point_, point_list[1], smoothness_); // 直线衔接下一段曲线的过渡点
			last_smoothness = smoothness_;

			// 生成曲线间的直线过渡
			bezier_curve_list[bezier_curve_num].Bezier_Update(temp_point, temp_point_1); // 曲线间的直线过渡
			total_len += bezier_curve_list[bezier_curve_num].Get_len();
			if (bezier_curve_list[bezier_curve_num].Get_len() > 0.00001f)
			{
				bezier_curve_num++;
			}

			point_list[0] = temp_point_1; // 下一段曲线起始点
			point_list[1] = point_;		  // 下一段曲线控制点

			generate_status = GENERATE_WAIT_LAST_CURVE_POINT;
		}
		break;

	default:
		generate_status = GENERATE_FINISHED_STRAIGHT;
	}
	return true;
}

#define CURVE_FINISHED_THRESHOLD 0.05f			   // m
#define START_ANGLE_THRESHOLD 2.f / 360.f * TWO_PI // 4度

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
 */h
bool Path::Get_Error_And_Vector(Vector2D location_, float yaw, float *target_yaw, float *normal_error, float *tangent_error, Vector2D *normal_vector, Vector2D *tangent_vector, float *max_vel)
{
	if (is_init == false)
		return false;

	if (have_start_angle == true)
	{
		if (is_start == false && fabsf(yaw - start_angle) > START_ANGLE_THRESHOLD)
		{
			currnet_target_angle = start_angle;
		}
		else
		{
			is_start = true;
		}
	}
	else
	{
		is_start = true;
	}

	if (is_start == true)
	{
		currnet_target_angle = end_angle;
	}

	*target_yaw = currnet_target_angle;

	*normal_error = 0.f - bezier_curve_list[current_bezier_curve_dx].Get_Nearest_Distance(location_, &current_t); // 获取最近点的t值和最近点距离
	current_curve_len = bezier_curve_list[current_bezier_curve_dx].Get_Current_Len(current_t);					  // 计算当前路程

	// 判断是否切换曲线
	while (current_bezier_curve_dx < bezier_curve_num && bezier_curve_list[current_bezier_curve_dx].Get_len() - current_curve_len < CURVE_FINISHED_THRESHOLD && is_end == false)
	{
		if (current_bezier_curve_dx < bezier_curve_num - 1) //
		{
			current_finished_len += bezier_curve_list[current_bezier_curve_dx].Get_len(); // 更新已完成曲线的累计路程

			current_bezier_curve_dx++; // 切换到下一段曲线

			*normal_error = bezier_curve_list[current_bezier_curve_dx].Get_Nearest_Distance(location_, &current_t); // 重新获取最近点的t值和最近点距离
			current_curve_len = bezier_curve_list[current_bezier_curve_dx].Get_Current_Len(current_t);				// 重新计算当前路程
		}
		else // 路径结束
		{
			is_end = true;
		}
	}

	if (is_start == false) // 直接锁定起点
	{
		*tangent_error = 0;
		*tangent_vector = Vector2D(0, 0); // 切向量为0

		*normal_vector = bezier_curve_list[current_bezier_curve_dx].Get_Start_point() - location_; // 法向量指向起点
		*normal_error = (*normal_vector).magnitude();

		*normal_vector = (*normal_vector).normalize();
	}
	else if (current_t >= 1) // 直接锁定终点
	{
		*tangent_error = 0;
		*tangent_vector = Vector2D(0, 0); // 切向量为0

		*normal_vector = bezier_curve_list[current_bezier_curve_dx].Get_End_point() - location_; // 法向量指向终点
		*normal_error = (*normal_vector).magnitude();

		*normal_vector = (*normal_vector).normalize();
	}
	else // 在曲线中
	{
		current_curve_len = bezier_curve_list[current_bezier_curve_dx].Get_Current_Len(current_t); // 计算当前路程

		// 切向误差 = 总路程 - 当前路段已走路程 - 之前路段总路程
		*tangent_error = total_len - current_curve_len - current_finished_len;
		*tangent_vector = bezier_curve_list[current_bezier_curve_dx].Get_Tangent_Vector(current_t);

		*normal_vector = bezier_curve_list[current_bezier_curve_dx].Get_Normal_Vector(location_, current_t);
	}

	*max_vel = bezier_curve_list[current_bezier_curve_dx].Get_Max_Vel(current_t);

	return true;
}

#define CURVATURE_SAMPLE_STEP 0.02 // m 计算曲率时的三个点采样步长

/**
 * @brief 计算每一段曲线的结束速度
 *
 * 根据曲线的类型和曲率，计算每一段曲线结束时的最大速度。
 */
void Path::Calc_End_Vel()
{
	for (int16_t i = bezier_curve_num - 1; i >= 0; i--)
	{
		if (i == bezier_curve_num - 1) // 如果是最后一段曲线
		{
			bezier_curve_list[i].Set_End_Vel(0.f); // 设置结束速度为 0
		}
		else
		{
			// 如果当前段和下一段都是直线
			if (bezier_curve_list[i + 1].Get_Bezier_Order() == FIRST_ORDER_BEZIER && bezier_curve_list[i].Get_Bezier_Order() == FIRST_ORDER_BEZIER)
			{
				float temp_vel_1, temp_vel_2;

				temp_vel_1 = bezier_curve_list[i + 1].Get_Max_Vel(0.f); // 获取下一段的最大速度

				// 计算连接处的曲率，采样点：当前段末尾前一点，当前段末尾（即下一段开始），下一段开始后一点
				float temp_curvature = Vector2D::curvatureFromThreePoints(
					bezier_curve_list[i].Get_Point((bezier_curve_list[i].Get_len() - CURVATURE_SAMPLE_STEP) / bezier_curve_list[i].Get_len()),
					bezier_curve_list[i].Get_Point(1.f),
					bezier_curve_list[i + 1].Get_Point(CURVATURE_SAMPLE_STEP / bezier_curve_list[i + 1].Get_len()));

				arm_sqrt_f32(1.f / temp_curvature, &temp_vel_2); // 根据曲率计算允许的最大速度 (v = sqrt(a_n / k))，假设最大向心加速度为 1m/s^2

				bezier_curve_list[i].Set_End_Vel(temp_vel_1 > temp_vel_2 ? temp_vel_2 : temp_vel_1); // 设置较小的速度为结束速度
			}
			else // 直线与曲线过渡，曲率可忽略
			{
				bezier_curve_list[i].Set_End_Vel(bezier_curve_list[i + 1].Get_Max_Vel(0.f)); // 设置结束速度为下一段的最大速度
			}
		}
	}
}

/**
 * @brief 重置路径规划器
 *
 * 重置路径规划器的所有状态和参数。
 */
void Path::Reset()
{
	is_init = false; // 重置初始化标志

	bezier_curve_num = 0; // 重置曲线数量

	generate_status = GENERATE_WAIT_FIRST_POINT; // 重置生成状态

	have_start_angle = 0; // 重置起始角度标志
	start_angle = 0;	  // 重置起始角度
	end_angle = 0;		  // 重置结束角度
	total_len = 0;		  // 重置总路径长度

	currnet_target_angle = 0; // 重置当前目标角度

	current_bezier_curve_dx = 0; // 重置当前曲线索引
	current_t = 0;				 // 重置当前曲线参数 t

	current_finished_len = 0; // 重置已完成的曲线长度
	current_curve_len = 0;	  // 重置当前曲线的长度

	last_smoothness = 0; // 重置平滑度

	is_end = false;	  // 重置路径结束标志
	is_start = false; // 重置路径开始标志
}

Vector2D Path::plan(Vector2D point)
{
	if (m_phase == S_FINISHED_PHASE)
	{
		return Vector2D{0, 0};
	}
	if (Is_End() == true)
	{
		bezier_curve_list[index_].Get_Nearest_Distance(point, &t_); // 获取点到曲线的最近距离
		distance_ = bezier_curve_list[index_].Get_Current_Len(t_);
		distance_ = total_ + distance_; // 计算当前总距离

		v_resultant_ = sp_.plan(distance_); // 速度规划器计算当前目标速度
		m_phase = sp_.getPhase();			// 获取当前速度规划阶段

		bezier_curve_list[index_].Get_Nearest_Distance(point, &t_);					 // 获取点到曲线的最近距离
		v_tangent_ = (bezier_curve_list[index_].Get_Tangent_Vector(t_)).normalize(); // 计算切线向量（单位向量）

		point_last_ = point; // 更新上一个点

		// 如果当前曲线段走完（t接近1）或者规划完成
		if (t_ >= 0.999f || m_phase == S_FINISHED_PHASE)
		{
			total_ += bezier_curve_list[index_].Get_len(); // 累加已完成路程
			index_++; // 切换到下一段曲线
			if (index_ >= bezier_curve_num || m_phase == S_FINISHED_PHASE)
			{
				is_end = false; // 结束运行
				m_phase = S_FINISHED_PHASE;
			}
		}

		return (v_tangent_ * v_resultant_); // 返回 速度向量 = 切向方向 * 目标速率
	}
	else
	{
		return Vector2D{0, 0};
	}
}

void Path::plan_reset()
{
	index_ = 0;
	total_ = 0.0f;
	point_last_ = bezier_curve_list[index_].Get_Start_point(); // 重置上一个点为起点
	m_phase = S_ACCEL_JERK_UP_PHASE;
	distance_ = 0.0f;	 // 重置距离
	t_ = 0.0f;			 // 重置参数 t
	v_resultant_ = 0.0f; // 重置速度
	is_end = true;
}