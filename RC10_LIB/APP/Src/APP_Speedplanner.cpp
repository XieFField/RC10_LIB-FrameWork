#include "APP_Speedplanner.h" // 速度规划器头文件


// 重置速度规划器参数
/**
 * @brief 重置速度规划器的参数。
 * @details 根据输入的参数结构体，初始化速度规划器的内部状态，包括最大加速度、最大减速度、目标位置等。
 * @param params 包含速度规划器参数的结构体。
 */
void TrapePlanner1D::param_reset(Speedplanner_1D_Param_Config params)
{
    // 保存用户参数并初始化成员变量
    m_maxAcc_ = abs(params.maxAcc);                             // 最大加速度
    m_maxDec_ = abs(params.maxDec);                             // 最大减速度
    m_maxJerk_ = abs(params.maxJerk);                           // 最大加加速度
    m_maxSpeed_ = abs(params.maxSpeed);                         // 最大速度
    m_initialSpeed_ = abs(params.initialSpeed);                 // 起始速度
    m_finalSpeed_ = abs(params.finalSpeed);                     // 目标速度
    m_startPos_ = params.startPos;                              // 起始位置
    m_targetPos_ = params.targetPos;                            // 目标位置
    m_totalDistance_ = abs(params.targetPos - params.startPos); // 总路程
    m_deadzone_ = abs(params.deadzone);                         // 死区范围

    // 根据目标位置与起始位置计算运动方向
    if (params.targetPos - params.startPos > 0.0f)
    {
        direction_ = 1.0f; // 如果目标位置大于起始位置，方向为正
    }
    else if (params.targetPos - params.startPos < 0.0f)
    {
        direction_ = -1.0f; // 如果目标位置小于起始位置，方向为负
    }

    // 计算加速和减速所需的路程
    float d_acc = (m_maxSpeed_ * m_maxSpeed_ - m_initialSpeed_ * m_initialSpeed_) / (2.0f * m_maxAcc_); // 加速段所需的路程
    float d_dec = (m_maxSpeed_ * m_maxSpeed_ - m_finalSpeed_ * m_finalSpeed_) / (2.0f * m_maxDec_);     // 减速段所需的路程

    // 判断是否能够达到设定最大速度
    if (d_acc + d_dec <= m_totalDistance_)
    {
        // 梯形规划：存在加速、匀速、减速三个阶段
        m_accelDistance_ = d_acc; // 保存加速段路程
        m_decelDistance_ = d_dec; // 保存减速段路程
    }
    else
    {
        // 三角形规划：无法达到设定最大速度，计算可达到的峰值速度 v_peak
        float v_peak_sq = (m_maxDec_ * m_initialSpeed_ * m_initialSpeed_ +
                           m_maxAcc_ * m_finalSpeed_ * m_finalSpeed_ +
                           2 * m_maxAcc_ * m_maxDec_ * m_totalDistance_) /
                          (m_maxAcc_ + m_maxDec_); // 计算峰值速度的平方
        float v_peak = 0.0f;
        arm_sqrt_f32(v_peak_sq, &v_peak);                                                              // 计算峰值速度
        m_accelDistance_ = (v_peak * v_peak - m_initialSpeed_ * m_initialSpeed_) / (2.0f * m_maxAcc_); // 重新计算加速段路程
        m_decelDistance_ = (v_peak * v_peak - m_finalSpeed_ * m_finalSpeed_) / (2.0f * m_maxDec_);     // 重新计算减速段路程
    }
    // 初始化阶段为加速段
    m_phase = ACCEL_PHASE;
}

/**
 * @brief 构造函数，初始化速度规划器。
 * @param params 包含速度规划器参数的结构体。
 */
TrapePlanner1D::TrapePlanner1D(Speedplanner_1D_Param_Config params)
{
    param_reset(params); // 调用参数重置函数
}

/**
 * @brief 根据当前已行驶的距离，判断当前所处的运动阶段。
 * @param traveled 当前已行驶的距离。
 * @return 当前的运动阶段。
 */
Phase TrapePlanner1D::determinePhase(float traveled)
{
    if (traveled >= m_totalDistance_)
        return FINISHED_PHASE; // 规划结束

    if (traveled < m_accelDistance_)
        return ACCEL_PHASE; // 加速阶段
    else if (traveled < (m_totalDistance_ - m_decelDistance_))
        return CONST_PHASE; // 匀速阶段
    else
        return DECEL_PHASE; // 减速阶段
}

/**
 * @brief 根据当前已行驶的距离，计算目标速度。
 * @param now_dis 当前已行驶的距离。
 * @return 规划的目标速度。
 */
float TrapePlanner1D::plan(float now_dis)
{
    // 计算已行驶距离
    traveled_ = abs(now_dis - m_startPos_);
    if (abs(m_targetPos_-now_dis) < m_deadzone_)
    {
        m_phase = FINISHED_PHASE;
        return 0.0f; // 如果距离小于死区范围，返回速度为0
    }
    if (traveled_ >= m_totalDistance_)
    {
        traveled_ = m_totalDistance_;      // 限制最大行驶距离
        m_phase = FINISHED_PHASE;          // 设置阶段为规划结束
        return m_finalSpeed_ * direction_; // 返回目标速度
    }

    // 判断当前阶段
    m_phase = determinePhase(traveled_);

    switch (m_phase)
    {
    case ACCEL_PHASE:
    {
        // 加速段：v^2 = v0^2 + 2*a*s
        float expr = m_initialSpeed_ * m_initialSpeed_ + 2.0f * m_maxAcc_ * traveled_;
        float sqrt_val = 0.0f;
        arm_sqrt_f32(expr, &sqrt_val); // 计算加速阶段目标速度
        v_target_ = sqrt_val;

        break;
    }
    case CONST_PHASE:
        v_target_ = m_maxSpeed_; // 匀速阶段目标速度
        break;
    case DECEL_PHASE:
    {
        // 减速阶段：根据公式计算目标速度
        float expr = m_finalSpeed_ * m_finalSpeed_ + 2 * m_maxDec_ * (m_totalDistance_ - traveled_);
        float sqrt_val = 0;
        arm_sqrt_f32(expr, &sqrt_val); // 计算减速阶段目标速度
        v_target_ = sqrt_val;
        break;
    }
    case FINISHED_PHASE:
    default:
        v_target_ = m_finalSpeed_; // 规划结束阶段目标速度
        break;
    }

    return v_target_ * direction_; // 返回目标速度
}

/**
 * @brief 重置速度规划器状态。
 */
/**
 * @brief 重置一维梯形速度规划器状态。
 */
void TrapePlanner1D::reset()
{
    traveled_ = 0;            // 重置已行驶路程
    m_phase = FINISHED_PHASE; // 设置阶段为规划结束
    m_totalDistance_ = 0;     // 重置总路程
    m_accelDistance_ = 0;     // 重置加速段长度
    m_decelDistance_ = 0;     // 重置减速段长度
    direction_ = 0;           // 重置运动方向
}


// ---------------------------- TrapePlanner2D ----------------------------

/**
 * @brief 重置二维梯形速度规划器的参数。
 * @param params 包含速度规划器参数的结构体。
 */
void TrapePlanner2D::param_reset(Speedplanner_2D_Param_Config params)
{
    // 保存用户参数
    m_maxAcc_ = abs(params.maxAcc);
    m_maxDec_ = abs(params.maxDec);
    m_maxSpeed_ = abs(params.maxSpeed);
    m_initialSpeed_ = abs(params.initialSpeed);
    m_finalSpeed_ = abs(params.finalSpeed);
    m_startPos_ = params.startPos;
    m_targetPos_ = params.targetPos;

    // 计算总路程：起点到目标点的直线距离
    Vector2D diff =  params.targetPos - params.startPos;
    m_totalDistance_ = diff.magnitude();

    // 计算若能达到设定最大速度时的加速和减速路程
    float d_acc = 0;
    if (m_maxSpeed_ > m_initialSpeed_)
        d_acc = (m_maxSpeed_ * m_maxSpeed_ - m_initialSpeed_ * m_initialSpeed_) / (2.0f * m_maxAcc_);
    float d_dec = 0;
    if (m_maxSpeed_ > m_finalSpeed_)
        d_dec = (m_maxSpeed_ * m_maxSpeed_ - m_finalSpeed_ * m_finalSpeed_) / (2.0f * m_maxDec_);

    // 判断是否能够达到设定最大速度
    if (d_acc + d_dec <= m_totalDistance_)
    {
        // 梯形规划：存在加速、匀速、减速三个阶段
        m_profileType = TRAPEZOIDAL;
        m_accelDistance_ = d_acc;
        m_decelDistance_ = d_dec;
    }
    else
    {
        // 三角形规划：无法达到设定最大速度，计算可达到的峰值速度 v_peak
        m_profileType = TRIANGULAR;
        float v_peak_sq = (m_maxDec_ * m_initialSpeed_ * m_initialSpeed_ +
                           m_maxAcc_ * m_finalSpeed_ * m_finalSpeed_ +
                           2 * m_maxAcc_ * m_maxDec_ * m_totalDistance_) /
                          (m_maxAcc_ + m_maxDec_);
        float v_peak = 0.0f;
        arm_sqrt_f32(v_peak_sq, &v_peak);
        m_accelDistance_ = (v_peak * v_peak - m_initialSpeed_ * m_initialSpeed_) / (2.0f * m_maxAcc_);
        m_decelDistance_ = (v_peak * v_peak - m_finalSpeed_ * m_finalSpeed_) / (2.0f * m_maxDec_);
    }

    // 初始化阶段为加速段
    m_phase = ACCEL_PHASE;
}

/**
 * @brief 构造函数，初始化二维梯形速度规划器。
 * @param params 包含速度规划器参数的结构体。
 */
TrapePlanner2D::TrapePlanner2D(Speedplanner_2D_Param_Config params)
{
    param_reset(params); // 调用参数重置函数
}

/**
 * @brief 根据当前已行驶的距离，计算目标速度。
 * @param now_dis 当前已行驶的距离。
 * @return 规划的目标速度。
 */
Vector2D TrapePlanner2D::plan(const Vector2D &now_dis)
{
    // 计算路径及单位方向
    Vector2D path = m_targetPos_ - now_dis;
	if (m_totalDistance_ < 0.0001f)
    {
        m_phase = FINISHED_PHASE;
        return Vector2D(0, 0);
    }

    Vector2D direction = path.normalize();

    // 计算当前位置在规划路径上的投影距离
    Vector2D delta = now_dis - m_startPos_;
    float traveled = delta * direction;
    if (traveled < 0)
        traveled = 0;
    if (traveled >= m_totalDistance_)
    {
        return m_finalSpeed_ * (m_targetPos_ - m_startPos_).normalize();
    }
    // 计算当前位置与目标点之间的直线距离
	
    float distanceToTarget = (m_targetPos_ - now_dis).magnitude();

    // 未进入 PID 控制则继续采用梯形规划，根据 traveled 判断当前阶段
    m_phase = determinePhase(traveled);
    float v_target = 0;
    switch (m_phase)
    {
    case ACCEL_PHASE:
    {
        float expr = m_initialSpeed_ * m_initialSpeed_ + 2 * m_maxAcc_ * traveled;
        float sqrt_val = 0;
        arm_sqrt_f32(expr, &sqrt_val);
        v_target = sqrt_val;
        break;
    }
    case CONST_PHASE:
        v_target = m_maxSpeed_;
        break;
    case DECEL_PHASE:
    {
        float expr = m_finalSpeed_ * m_finalSpeed_ + 2 * m_maxDec_ * (m_totalDistance_ - traveled);
        float sqrt_val = 0;
        arm_sqrt_f32(expr, &sqrt_val);
        v_target = sqrt_val;
        break;
    }
    case FINISHED_PHASE:
    default:
        v_target = m_finalSpeed_;
        break;
    }

    return direction * v_target;
}

/**
 * @brief 根据当前已行驶的距离，判断当前所处的运动阶段。
 * @param traveled 当前已行驶的距离。
 * @return 当前的运动阶段。
 */
Phase TrapePlanner2D::determinePhase(float traveled)
{
    if ((traveled-0.1f) >= m_totalDistance_)
        return FINISHED_PHASE;

    if (m_profileType == TRAPEZOIDAL)
    {
        if (traveled < m_accelDistance_)
            return ACCEL_PHASE;
        else if (traveled < (m_totalDistance_ - m_decelDistance_))
            return CONST_PHASE;
        else
            return DECEL_PHASE;
    }
    else
    { // TRIANGULAR
        if (traveled < m_accelDistance_)
            return ACCEL_PHASE;
        else
            return DECEL_PHASE;
    }
}

/**
 * @brief 重置二维梯形速度规划器状态。
 */
void TrapePlanner2D::reset()
{
    traveled_ = 0;            // 重置已行驶路程
    m_phase = FINISHED_PHASE; // 设置阶段为规划结束
    m_totalDistance_ = 0;     // 重置总路程
    m_accelDistance_ = 0;     // 重置加速段长度
    m_decelDistance_ = 0;     // 重置减速段长度
    direction_ = 0;           // 重置运动方向
}

// ---------------------------- ConstantAcc ----------------------------
/**
 * @brief 一维恒加速度平滑器构造函数。
 * @param maxAcceleration 最大加速度。
 * @param initialValue 初始速度。
 */
ConstantAcc::ConstantAcc(float maxAcceleration, float initialValue)
{
    this->maxAcceleration_ = maxAcceleration;
    lastOutput_ = initialValue;
}

/**
 * @brief 重置恒加速度平滑器。
 * @param maxAcceleration 最大加速度。
 * @param initialValue 初始速度。
 */
void ConstantAcc::reset(float maxAcceleration, float initialValue)
{
    lastOutput_ = initialValue;
    this->maxAcceleration_ = maxAcceleration;
}

/**
 * @brief 规划输出速度，限制加速度变化。
 * @param targetSpeed 目标速度。
 * @return 平滑后的速度。
 */
float ConstantAcc::plan(float targetSpeed)
{
    float diff = targetSpeed - lastOutput_;
    // 限制速度变化量不超过最大加速度
    if (fabs(diff) > maxAcceleration_)
    {
        if (diff > 0)
        {
            diff = maxAcceleration_;
        }
        else
        {
            diff = -maxAcceleration_;
        }
    }
    lastOutput_ += diff;
    return lastOutput_;
}

/**
 * @brief 设置最大加速度。
 * @param acceleration 最大加速度。
 */
void ConstantAcc::setMaxAcceleration(float acceleration)
{
    maxAcceleration_ = acceleration;
}

/**
 * @brief 仅重置平滑器输出速度为0。
 */
void ConstantAcc::reset_speed()
{
    lastOutput_ = 0.0f;
}


// ---------------------------- Td ----------------------------
/**
 * @brief 一维TD平滑器构造函数。
 * @param td_r_ TD平滑参数R。
 */
Td::Td(float td_r_)
{
    r_ = td_r_;
}

/**
 * @brief 设置TD平滑参数R。
 * @param td_r_ R参数。
 */
void Td::set_R(float td_r_)
{
    r_ = td_r_;
}

/**
 * @brief TD平滑函数。
 * @param input_expect 期望输入。
 * @return 平滑输出。
 * @details 采用二阶TD算法对输入信号进行平滑处理，R越小越平滑。
 */
float Td::plan(float input_expect)
{
    expect_ = input_expect;

    uint32_t current_time = HAL_GetTick(); // 获取当前时间，单位ms
    if (previous_time_ != 0)
    { // 确保上一次时间不为0
        Ts_ = float(current_time - previous_time_) / 1000.0f;
    }
    previous_time_ = current_time;
    // 二阶TD算法核心：fh_为加速度项
    fh_ = -r_ * r_ * (V1_ - expect_) - 2.0f * r_ * V2_;

    V1_ += V2_ * Ts_; // 速度积分
    V2_ += fh_ * Ts_; // 加速度积分

    return V1_;
}




