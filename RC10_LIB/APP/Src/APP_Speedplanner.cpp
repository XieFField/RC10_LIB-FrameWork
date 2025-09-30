#include "APP_Speedplanner.h"

// 重置速度规划器参数
/**
 * @brief 重置速度规划器的参数。
 * @details 根据输入的参数结构体，初始化速度规划器的内部状态，包括最大加速度、最大减速度、目标位置等。
 * @param params 包含速度规划器参数的结构体。
 */
void TrapePlanner1D::param_reset(Speedplanner_Param_Config params)
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

// 构造函数：初始化速度规划器参数
/**
 * @brief 构造函数，初始化速度规划器。
 * @param params 包含速度规划器参数的结构体。
 */
TrapePlanner1D::TrapePlanner1D(Speedplanner_Param_Config params)
{
    param_reset(params); // 调用参数重置函数
}

// 根据当前已行驶距离判断处于哪个阶段
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

// 根据当前已行驶的距离，规划目标速度
/**
 * @brief 根据当前已行驶的距离，计算目标速度。
 * @param now_dis 当前已行驶的距离。
 * @return 规划的目标速度。
 */
float TrapePlanner1D::plan(float now_dis)
{
    // 计算已行驶距离
    traveled_ = abs(now_dis - m_startPos_);
    if (traveled_ < m_deadzone_)
    {
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
        // 加速阶段：根据公式计算目标速度
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

// 重置速度规划器状态
void TrapePlanner1D::reset()
{
    traveled_ = 0;            // 重置已行驶路程
    m_phase = FINISHED_PHASE; // 设置阶段为规划结束
    m_totalDistance_ = 0;     // 重置总路程
    m_accelDistance_ = 0;     // 重置加速段长度
    m_decelDistance_ = 0;     // 重置减速段长度
    direction_ = 0;           // 重置运动方向
}

void SShapedPlanner1D::param_reset(Speedplanner_Param_Config params)
{
    m_maxAcc_ = params.maxAcc;
    m_maxDec_ = params.maxDec;
    m_maxJerk_ = params.maxJerk;
    m_maxSpeed_ = params.maxSpeed;
    m_initialSpeed_ = params.initialSpeed;
    m_finalSpeed_ = params.finalSpeed;
    m_startPos_ = params.startPos;
    m_targetPos_ = params.targetPos;
    m_deadzone_ = params.deadzone;
    // 计算总距离
    m_totalDistance_ = std::abs(m_targetPos_ - m_startPos_);

    // 预计算各个阶段的路程
    cal_PhaseDistances();

    // 初始化当前阶段
    m_phase = S_ACCEL_JERK_UP_PHASE;
}

// 构造函数：初始化速度规划器参数
/**
 * @brief 构造函数，初始化 S 型速度规划器。
 * @param params 包含速度规划器参数的结构体。
 */
SShapedPlanner1D::SShapedPlanner1D(Speedplanner_Param_Config params)
{
    param_reset(params); // 调用参数重置函数
}

float SShapedPlanner1D::plan(float now_dis)
{
    // 计算已行驶的距离
    float traveled_ = std::abs(now_dis - m_startPos_);

    // 确定当前阶段
    m_phase = determinePhase(traveled_);

    // 根据阶段计算当前速度
    float currentSpeed = 0.0f;

    switch (m_phase)
    {
    case S_ACCEL_JERK_UP_PHASE:
        currentSpeed = cal_Acc_JerkUpSpeed(traveled_);
        break;
    case S_ACCEL_CONST_PHASE:
        currentSpeed = cal_Acc_ConstSpeed(traveled_);
        break;
    case S_ACCEL_JERK_DOWN_PHASE:
        currentSpeed = cal_Acc_JerkDownSpeed(traveled_);
        break;
    case S_CONST_VEL_PHASE:
        currentSpeed = m_maxSpeed_;
        break;
    case S_DECEL_JERK_UP_PHASE:
        currentSpeed = cal_Dec_JerkUpSpeed(traveled_);
        break;
    case S_DECEL_CONST_PHASE:
        currentSpeed = cal_Dec_ConstSpeed(traveled_);
        break;
    case S_DECEL_JERK_DOWN_PHASE:
        currentSpeed = cal_Dec_JerkDownSpeed(traveled_);
        break;
    case S_FINISHED_PHASE:
        currentSpeed = m_finalSpeed_;
        break;
    }

    return currentSpeed;
}

/**
 * @brief 根据已行驶的距离确定当前所处的 S 型规划阶段。
 * @param traveled 已行驶的距离（从起始位置算起）。
 * @return 当前 S 型规划阶段。
 */
SPhase SShapedPlanner1D::determinePhase(float traveled)
{
    // 检查是否已经完成
    if (traveled >= m_totalDistance_)
    {
        return S_FINISHED_PHASE;
    }

    // 累计距离判断当前阶段
    float cumulative = 0.0f;

    // 加速段：Jerk 上升
    cumulative += m_accelJerkUpDistance_;
    if (traveled < cumulative)
        return S_ACCEL_JERK_UP_PHASE;

    // 加速段：加速度恒定
    cumulative += m_accelConstDistance_;
    if (traveled < cumulative)
        return S_ACCEL_CONST_PHASE;

    // 加速段：Jerk 下降
    cumulative += m_accelJerkDownDistance_;
    if (traveled < cumulative)
        return S_ACCEL_JERK_DOWN_PHASE;

    // 匀速段
    cumulative += m_constVelDistance_;
    if (traveled < cumulative)
        return S_CONST_VEL_PHASE;

    // 减速段：Jerk 上升
    cumulative += m_decelJerkUpDistance_;
    if (traveled < cumulative)
        return S_DECEL_JERK_UP_PHASE;

    // 减速段：加速度恒定
    cumulative += m_decelConstDistance_;
    if (traveled < cumulative)
        return S_DECEL_CONST_PHASE;

    // 减速段：Jerk 下降
    return S_DECEL_JERK_DOWN_PHASE;
}

// ---------------------------- 内部辅助函数 ----------------------------

/**
 * @brief 计算各阶段的距离
 */
void SShapedPlanner1D::cal_PhaseDistances()
{
    // 先计算加速阶段的 Jerk 时间
    float t_jerk_up = m_maxAcc_ / m_maxJerk_;
    float t_jerk_up_2 = t_jerk_up * t_jerk_up;
    float t_jerk_up_3 = t_jerk_up_2 * t_jerk_up;
    m_accelJerkUpDistance_ = m_initialSpeed_ * t_jerk_up + 0.5f * m_maxJerk_ * t_jerk_up_3 / 3.0f;

    // 加速段：加速度保持恒定的时间
    float speed_after_jerk_up = m_initialSpeed_ + 0.5f * m_maxJerk_ * t_jerk_up_2;
    float time_accel_const = (m_maxSpeed_ - speed_after_jerk_up) / m_maxAcc_;
    float time_accel_const_2 = time_accel_const * time_accel_const;
    m_accelConstDistance_ = speed_after_jerk_up * time_accel_const + 0.5f * m_maxAcc_ * time_accel_const_2;

    // Jerk 下降阶段（加速度回到 0）
    float t_jerk_down = m_maxAcc_ / m_maxJerk_;
    float t_jerk_down_2 = t_jerk_down * t_jerk_down;
    float t_jerk_down_3 = t_jerk_down_2 * t_jerk_down;
    float speed_after_accel_const = speed_after_jerk_up + m_maxAcc_ * time_accel_const;
    m_accelJerkDownDistance_ = speed_after_accel_const * t_jerk_down - (m_maxJerk_ * t_jerk_down_3) / 6.0f;

    // 匀速阶段距离
    float speed_during_const = speed_after_accel_const + 0.5f * m_maxJerk_ * t_jerk_down_2;
    float distance_remaining = m_totalDistance_ - (m_accelJerkUpDistance_ + m_accelConstDistance_ + m_accelJerkDownDistance_);
    m_constVelDistance_ = distance_remaining * (speed_during_const >= m_maxSpeed_ ? 1.0f : 0.0f);

    // 减速阶段（与加速阶段对称）
    float decel_time_jerk_up = m_maxDec_ / m_maxJerk_;
    float decel_time_jerk_up_2 = decel_time_jerk_up * decel_time_jerk_up;
    float decel_time_jerk_up_3 = decel_time_jerk_up_2 * decel_time_jerk_up;
    m_decelJerkUpDistance_ = m_maxSpeed_ * decel_time_jerk_up - 0.5f * m_maxJerk_ * decel_time_jerk_up_3 / 3.0f;

    float speed_after_decel_jerk_up = m_maxSpeed_ - 0.5f * m_maxJerk_ * decel_time_jerk_up_2;
    float decel_time_const = (speed_after_decel_jerk_up - m_finalSpeed_) / m_maxDec_;
    float decel_time_const_2 = decel_time_const * decel_time_const;
    m_decelConstDistance_ = speed_after_decel_jerk_up * decel_time_const - 0.5f * m_maxDec_ * decel_time_const_2;

    float decel_time_jerk_down = m_maxDec_ / m_maxJerk_;
    float decel_time_jerk_down_2 = decel_time_jerk_down * decel_time_jerk_down;
    float decel_time_jerk_down_3 = decel_time_jerk_down_2 * decel_time_jerk_down;
    float speed_after_decel_const = speed_after_decel_jerk_up - m_maxDec_ * decel_time_const;
    m_decelJerkDownDistance_ = speed_after_decel_const * decel_time_jerk_down - (m_maxJerk_ * decel_time_jerk_down_3) / 6.0f;
}

/**
 * @brief 加速段：Jerk 上升阶段的速度
 */
float SShapedPlanner1D::cal_Acc_JerkUpSpeed(float traveled)
{
    float t;
    arm_sqrt_f32(2.0f * traveled / m_maxJerk_, &t);
    return m_initialSpeed_ + 0.5f * m_maxJerk_ * t * t;
}

/**
 * @brief 加速段：加速度恒定阶段的速度
 */
float SShapedPlanner1D::cal_Acc_ConstSpeed(float traveled)
{
    float t = (traveled - m_accelJerkUpDistance_) / m_maxAcc_;
    return m_initialSpeed_ + m_maxAcc_ * t;
}

/**
 * @brief 加速段：Jerk 下降阶段的速度
 */
float SShapedPlanner1D::cal_Acc_JerkDownSpeed(float traveled)
{
    float distance = traveled - m_accelJerkUpDistance_ - m_accelConstDistance_;
    float t;
    arm_sqrt_f32(2.0f * distance / m_maxJerk_, &t);
    return m_maxSpeed_ - 0.5f * m_maxJerk_ * t * t;
}

/**
 * @brief 减速段：Jerk 上升阶段的速度
 */
float SShapedPlanner1D::cal_Dec_JerkUpSpeed(float traveled)
{
    float remaining = m_totalDistance_ - traveled;
    float t;
    arm_sqrt_f32(2.0f * remaining / m_maxJerk_, &t);
    return m_maxSpeed_ - 0.5f * m_maxJerk_ * t * t;
}

/**
 * @brief 减速段：加速度恒定阶段的速度
 */
float SShapedPlanner1D::cal_Dec_ConstSpeed(float traveled)
{
    float remaining = m_totalDistance_ - traveled;
    float t = remaining / m_maxDec_;
    return m_maxSpeed_ - m_maxDec_ * t;
}

/**
 * @brief 减速段：Jerk 下降阶段的速度
 */
float SShapedPlanner1D::cal_Dec_JerkDownSpeed(float traveled)
{
    float remaining = m_totalDistance_ - traveled;
    float t;
    arm_sqrt_f32(2.0f * remaining / m_maxJerk_, &t);
    return m_finalSpeed_ + 0.5f * m_maxJerk_ * t * t;
}

// 重置速度规划器状态
/**
 * @brief 重置速度规划器的内部状态。
 */
void SShapedPlanner1D::reset()
{
    traveled_ = 0;        // 重置已行驶路程
    m_totalDistance_ = 0; // 重置总路程
    // 内部状态变量
    m_phase = S_FINISHED_PHASE; // 当前规划所处的阶段
    // 预计算的 S 型规划各个阶段的距离
    m_accelJerkUpDistance_ = 0.0f;   // 加速段：Jerk 上升阶段的路程
    m_accelConstDistance_ = 0.0f;    // 加速段：加速度恒定阶段的路程
    m_accelJerkDownDistance_ = 0.0f; // 加速段：Jerk 下降阶段的路程
    m_constVelDistance_ = 0.0f;      // 匀速段：恒定速度阶段的路程
    m_decelJerkUpDistance_ = 0.0f;   // 减速段：Jerk 上升（减速开始）阶段的路程
    m_decelConstDistance_ = 0.0f;    // 减速段：加速度恒定（减速中）阶段的路程
    m_decelJerkDownDistance_ = 0.0f; // 减速段：Jerk 下降（减速结束）阶段的路程
}
