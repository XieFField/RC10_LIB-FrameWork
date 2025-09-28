#include "APP_Speedplanner.h"

// 重置速度规划器参数
void TrapePlanner1D::param_reset(Speedplanner_Param_Config params)
{
    // 保存用户参数并初始化成员变量
    m_phase = FINISHED_PHASE; // 初始化阶段为规划结束
    m_maxAcc_ = abs(params.maxAcc); // 最大加速度
    m_maxDec_ = abs(params.maxDec); // 最大减速度
    m_maxJerk_ = abs(params.maxJerk); // 最大加加速度
    m_maxSpeed_ = abs(params.maxSpeed); // 最大速度
    m_initialSpeed_ = abs(params.initialSpeed); // 起始速度
    m_finalSpeed_ = abs(params.finalSpeed); // 目标速度
    m_startPos_ = params.startPos; // 起始位置
    m_targetPos_ = params.targetPos; // 目标位置
    m_totalDistance_ = abs(params.targetPos - params.startPos); // 总路程
    m_deadzone_ = abs(params.deadzone); // 死区范围

    // 根据目标位置与起始位置计算运动方向
    if (params.targetPos - params.startPos > 0.0f)
    {
        direction_ = 1.0f; // 正方向
    }
    else if (params.targetPos - params.startPos < 0.0f)
    {
        direction_ = -1.0f; // 负方向
    }

    // 计算加速和减速所需的路程
    float d_acc = (m_maxSpeed_ * m_maxSpeed_ - m_initialSpeed_ * m_initialSpeed_) / (2.0f * m_maxAcc_);
    float d_dec = (m_maxSpeed_ * m_maxSpeed_ - m_finalSpeed_ * m_finalSpeed_) / (2.0f * m_maxDec_);

    // 判断是否能够达到设定最大速度
    if (d_acc + d_dec <= m_totalDistance_)
    {
        // 梯形规划：存在加速、匀速、减速三个阶段
        m_accelDistance_ = d_acc;
        m_decelDistance_ = d_dec;
    }
    else
    {
        // 三角形规划：无法达到设定最大速度，计算可达到的峰值速度 v_peak
        float v_peak_sq = (m_maxDec_ * m_initialSpeed_ * m_initialSpeed_ +
                           m_maxAcc_ * m_finalSpeed_ * m_finalSpeed_ +
                           2 * m_maxAcc_ * m_maxDec_ * m_totalDistance_) /
                          (m_maxAcc_ + m_maxDec_);
        float v_peak = 0.0f;
        arm_sqrt_f32(v_peak_sq, &v_peak); // 计算峰值速度
        m_accelDistance_ = (v_peak * v_peak - m_initialSpeed_ * m_initialSpeed_) / (2.0f * m_maxAcc_);
        m_decelDistance_ = (v_peak * v_peak - m_finalSpeed_ * m_finalSpeed_) / (2.0f * m_maxDec_);
    }
    // 初始化阶段为加速段
    m_phase = ACCEL_PHASE;
}

// 构造函数：初始化速度规划器参数
TrapePlanner1D::TrapePlanner1D(Speedplanner_Param_Config params)
{
    param_reset(params); // 调用参数重置函数
}

// 根据当前已行驶距离判断处于哪个阶段
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
        traveled_ = m_totalDistance_; // 限制最大行驶距离
        m_phase = FINISHED_PHASE; // 设置阶段为规划结束
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
    m_phase = FINISHED_PHASE; // 设置阶段为规划结束
    m_totalDistance_ = 0; // 重置总路程
    m_accelDistance_ = 0; // 重置加速段长度
    m_decelDistance_ = 0; // 重置减速段长度
    direction_ = 0; // 重置运动方向
}

