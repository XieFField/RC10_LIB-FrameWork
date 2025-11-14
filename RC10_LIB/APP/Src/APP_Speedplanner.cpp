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
    if (abs(traveled - m_totalDistance_) < m_deadzone_)
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
    float traveled_ = abs(now_dis - m_startPos_);
    if (abs(m_targetPos_ - now_dis) < m_deadzone_)
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
    m_phase = FINISHED_PHASE; // 设置阶段为规划结束
    m_totalDistance_ = 0;     // 重置总路程
    m_accelDistance_ = 0;     // 重置加速段长度
    m_decelDistance_ = 0;     // 重置减速段长度
    direction_ = 0;           // 重置运动方向
}

// ---------------------------- SShapedPlanner1D ----------------------------

/**
 * @brief 重置 S 型速度规划器的参数。
 * @param params 包含速度规划器参数的结构体。
 */
void SShapedPlanner1D::param_reset(Speedplanner_1D_Param_Config params)
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
    m_totalDistance_ = abs(m_targetPos_ - m_startPos_);

    // 预计算各个阶段的路程
    cal_PhaseDistances();

    // 初始化当前阶段
    m_phase = S_ACCEL_JERK_UP_PHASE;
}

/**
 * @brief 构造函数，初始化 S 型速度规划器。
 * @param params 包含速度规划器参数的结构体。
 */
SShapedPlanner1D::SShapedPlanner1D(Speedplanner_1D_Param_Config params)
{
    param_reset(params); // 调用参数重置函数
}

/**
 * @brief 根据当前已行驶的距离，计算目标速度。
 * @param now_dis 当前已行驶的距离。
 * @return 规划的目标速度。
 */
float SShapedPlanner1D::plan(float now_dis)
{
    // 计算已行驶的距离
    float traveled_ = abs(now_dis - m_startPos_);

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
    if (abs(traveled - m_totalDistance_) < m_deadzone_)
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
 * @brief 计算各阶段的距离。
 */
void SShapedPlanner1D::cal_PhaseDistances()
{
    // == == == == == 计算加速段参数 == == == == ==
    // 判断是否能达到最大加速度
    float Tj1, Ta, alima, Tj2, Td, alimd, Tv, vlim, T;
    if ((m_maxSpeed_ - m_initialSpeed_) * m_maxJerk_ < m_maxAcc_ * m_maxAcc_) // 不能达到最大加速度的情况
    {
        if (m_initialSpeed_ > m_maxSpeed_) // 如果初始速度已经大于最大速度
        {
            Tj1 = 0;   // 加加速时间为0
            Ta = 0;    // 总加速时间为0
            alima = 0; // 实际最大加速度为0
        }

        else // 正常情况下的计算
        {
            arm_sqrt_f32((m_maxSpeed_ - m_initialSpeed_) / m_maxJerk_, &Tj1); // 加加速时间
            Ta = 2 * Tj1;                                                     // 总加速时间（对称的加加速和减加速）
            alima = Tj1 * m_maxJerk_;                                         // 实际能达到的最大加速度
        }
    }
    else // 能达到最大加速度的情况
    {
        Tj1 = m_maxAcc_ / m_maxJerk_;                           // 加加速时间
        Ta = Tj1 + (m_maxSpeed_ - m_initialSpeed_) / m_maxAcc_; // 总加速时间
        alima = m_maxAcc_;                                      // 实际最大加速度等于设定最大加速度
    }

    //========== 计算减速段参数 ==========
    // 判断是否能达到最大减速度
    if ((m_maxSpeed_ - m_finalSpeed_) * m_maxJerk_ < m_maxDec_ * m_maxDec_) // 不能达到最大减速度的情况
    {
        arm_sqrt_f32((m_maxSpeed_ - m_finalSpeed_) / m_maxJerk_, &Tj2); // 加减速时间
        Td = 2 * Tj2;                                                   // 总减速时间
        alimd = Tj2 * m_maxJerk_;                                       // 实际最大减速度
    }
    else // 能达到最大减速度的情况
    {
        Tj2 = m_maxDec_ / m_maxJerk_;                         // 加减速时间
        Td = Tj2 + (m_maxSpeed_ - m_finalSpeed_) / m_maxDec_; // 总减速时间
        alimd = m_maxDec_;                                    // 实际最大减速度等于设定最大减速度
    }

    // ========== 计算匀速段时间 ==========
    Tv = (m_targetPos_ - m_startPos_) / m_maxSpeed_ - Ta / 2 * (1 + m_initialSpeed_ / m_maxSpeed_) - Td / 2 * (1 + m_finalSpeed_ / m_maxSpeed_);

    // ========== 处理不存在匀速阶段的情况 ==========
    if (Tv > 0) // 存在匀速阶段
    {
        vlim = m_maxSpeed_; // 限制速度等于最大速度
        T = Tv + Ta + Td;   // 总时间
    }
    else // 不存在匀速阶段
    {
        Tv = 0;                           // 匀速时间为0
        float amax_accel_org = m_maxAcc_; // 保存原始加速段最大加速度值
        float amax_decel_org = m_maxDec_; // 保存原始减速段最大加速度值
        int count = 0;                    // 调整次数计数器
        // 计算delta值，用于求解时间参数
        // 由于现在有两个不同的加速度，需要分别计算加速段和减速段
        // 这里使用平均加速度来近似计算
        float a_avg = (m_maxAcc_ + m_maxDec_) / 2;
        float delta = (a_avg * a_avg * a_avg * a_avg) / (m_maxJerk_ * m_maxJerk_) + 2 * (m_initialSpeed_ * m_initialSpeed_ + m_finalSpeed_ * m_finalSpeed_) + a_avg * (4 * (m_targetPos_ - m_startPos_) - 2 * a_avg / m_maxJerk_ * (m_initialSpeed_ + m_finalSpeed_));

        // 初始时间参数计算（使用平均加速度）
        Tj1 = m_maxAcc_ / m_maxJerk_;
        Ta = (a_avg * a_avg / m_maxJerk_ - 2 * m_initialSpeed_ + sqrt(delta)) / (2 * a_avg);
        Tj2 = m_maxDec_ / m_maxJerk_;
        Td = (a_avg * a_avg / m_maxJerk_ - 2 * m_finalSpeed_ + sqrt(delta)) / (2 * a_avg);
        vlim = m_initialSpeed_ + (Ta - Tj1) * alima; // 计算实际达到的最大速度

        // 逐渐减少加速度，直到找到可行的解
        while (Ta < 2 * Tj1 || Td < 2 * Tj2)
        {
            count += 1;
            // 同时减少加速段和减速段的加速度，保持比例关系
            float reduction_factor = 0.9;             // 每次减少10%
            m_maxAcc_ = m_maxAcc_ * reduction_factor; // 保持最小加速度
            m_maxDec_ = m_maxDec_ * reduction_factor; // 保持最小加速度

            // 重新计算加速段参数
            if ((m_maxSpeed_ - m_initialSpeed_) * m_maxJerk_ < m_maxAcc_ * m_maxAcc_)
            {
                arm_sqrt_f32((m_maxSpeed_ - m_initialSpeed_) / m_maxJerk_, &Tj1);
                Ta = 2 * Tj1;
                alima = Tj1 * m_maxJerk_;
            }
            else
            {
                Tj1 = m_maxAcc_ / m_maxJerk_;
                Ta = Tj1 + (m_maxSpeed_ - m_initialSpeed_) / m_maxAcc_;
                alima = m_maxAcc_;
            }

            // 重新计算减速段参数
            if ((m_maxSpeed_ - m_finalSpeed_) * m_maxJerk_ < m_maxDec_ * m_maxDec_)
            {
                arm_sqrt_f32((m_maxSpeed_ - m_finalSpeed_) / m_maxJerk_, &Tj2);
                Td = 2 * Tj2;
                alimd = Tj2 * m_maxJerk_;
            }
            else
            {
                Tj2 = m_maxDec_ / m_maxJerk_;
                Td = Tj2 + (m_maxSpeed_ - m_finalSpeed_) / m_maxDec_;
                alimd = m_maxDec_;
            }
            // 重新计算平均加速度和delta值
            a_avg = (m_maxAcc_ + m_maxDec_) / 2;
            if (a_avg > 0)
            {
                delta = (a_avg * a_avg * a_avg * a_avg) / (m_maxJerk_ * m_maxJerk_) + 2 * (m_initialSpeed_ * m_initialSpeed_ + m_finalSpeed_ * m_finalSpeed_) + a_avg * (4 * (m_targetPos_ - m_startPos_) - 2 * a_avg / m_maxJerk_ * (m_initialSpeed_ + m_finalSpeed_));
            }

            else
            {
                delta = (a_avg * a_avg * a_avg * a_avg) / (m_maxJerk_ * m_maxJerk_) + 2 * (m_initialSpeed_ * m_initialSpeed_ + m_finalSpeed_ * m_finalSpeed_) - a_avg * (4 * (m_targetPos_ - m_startPos_) - 2 * a_avg / m_maxJerk_ * (m_initialSpeed_ + m_finalSpeed_));
            }

            // 重新计算时间参数
            Ta = (a_avg * a_avg / m_maxJerk_ - 2 * m_initialSpeed_ + sqrt(delta)) / (2 * a_avg);
            Td = (a_avg * a_avg / m_maxJerk_ - 2 * m_finalSpeed_ + sqrt(delta)) / (2 * a_avg);
            vlim = m_initialSpeed_ + (Ta - Tj1) * alima; // 重新计算实际最大速度

            // 防止无限循环
            if (count > 100)
            {
                m_maxAcc_ = 0.0f;        // 最大加速度
                m_maxDec_ = 0.0f;        // 最大减速度
                m_maxJerk_ = 0.0f;       // 最大加加速度
                m_maxSpeed_ = 0.0f;      // 最大速度
                m_initialSpeed_ = 0.0f;  // 起始速度
                m_finalSpeed_ = 0.0f;    // 目标速度
                m_startPos_ = 0.0f;      // 起始位置
                m_targetPos_ = 0.0f;     // 目标位置
                m_totalDistance_ = 0.0f; // 总路程
                m_deadzone_ = 0.0f;      // 死区范围
                err_ = 1;                // 设置错误标志
                break;
            }
        }
        // 处理加速或减速时间为负的情况
        if (Ta < 0 || Td < 0)
        {
            if (m_initialSpeed_ > m_finalSpeed_) // 初始速度大于目标速度，主要是减速
            {
                Ta = 0;
                Tj1 = 0;
                alima = 0;
                Td = 2 * (m_targetPos_ - m_startPos_) / (m_finalSpeed_ + m_initialSpeed_);
                Tj2 = (m_maxJerk_ * (m_targetPos_ - m_startPos_) - sqrt(m_maxJerk_ * (m_maxJerk_ * (m_targetPos_ - m_startPos_) * (m_targetPos_ - m_startPos_) + (m_finalSpeed_ + m_initialSpeed_) * (m_finalSpeed_ + m_initialSpeed_) * (m_finalSpeed_ - m_initialSpeed_)))) / (m_maxJerk_ * (m_finalSpeed_ + m_initialSpeed_));
                alimd = -m_maxJerk_ * Tj2;
                vlim = m_finalSpeed_ - (Td - Tj2) * alimd;
                alimd = -alimd;
            }

            else // 主要是加速
            {
                Td = 0;
                Tj2 = 0;
                Ta = 2 * (m_targetPos_ - m_startPos_) / (m_finalSpeed_ + m_initialSpeed_);
                Tj1 = (m_maxJerk_ * (m_targetPos_ - m_startPos_) - sqrt(m_maxJerk_ * (m_maxJerk_ * (m_targetPos_ - m_startPos_) * (m_targetPos_ - m_startPos_) - (m_finalSpeed_ + m_initialSpeed_) * (m_finalSpeed_ + m_initialSpeed_) * (m_finalSpeed_ - m_initialSpeed_)))) / (m_maxJerk_ * (m_finalSpeed_ + m_initialSpeed_));
                alima = m_maxJerk_ * Tj1;
                vlim = m_initialSpeed_ + (Ta - Tj1) * alima;
            }
        }

        T = Tv + Ta + Td; // 计算总时间
    }

    float T2_duration, v_start_stage2, T6_duration, v_start_stage6;
    // ========== 计算各阶段路程 ==========
    // 阶段1: 加加速阶段 (0 <= t < Tj1)
    m_accelJerkUpDistance_ = m_initialSpeed_ * Tj1 + m_maxJerk_ * Tj1 * Tj1 * Tj1 / 6;

    // 阶段2: 匀加速阶段 (Tj1 <= t < (Ta - Tj1))
    T2_duration = Ta - 2 * Tj1;
    v_start_stage2 = m_initialSpeed_ + m_maxJerk_ * Tj1 * Tj1 / 2;
    m_accelConstDistance_ = v_start_stage2 * T2_duration + 0.5 * alima * T2_duration * T2_duration;

    // 阶段3: 减加速阶段 ((Ta - Tj1) <= t < Ta)
    m_accelJerkDownDistance_ = vlim * Tj1 - m_maxJerk_ * Tj1 * Tj1 * Tj1 / 6;

    // 阶段4: 匀速阶段 (Ta <= t < (Ta + Tv))
    m_constVelDistance_ = vlim * Tv;

    // 阶段5: 加减速阶段 ((T - Td) <= t < (T - Td + Tj2))
    m_decelJerkUpDistance_ = vlim * Tj2 - m_maxJerk_ * Tj2 * Tj2 * Tj2 / 6;

    // 阶段6: 匀减速阶段 ((T - Td + Tj2) <= t < (T - Tj2))
    T6_duration = Td - 2 * Tj2;
    v_start_stage6 = vlim - m_maxJerk_ * Tj2 * Tj2 / 2;
    m_decelConstDistance_ = v_start_stage6 * T6_duration - 0.5 * alimd * T6_duration * T6_duration;

    // 阶段7: 减减速阶段 ((T - Tj2) <= t < T)
    m_decelJerkDownDistance_ = m_finalSpeed_ * Tj2 + m_maxJerk_ * Tj2 * Tj2 * Tj2 / 6;

    m_t1_ = Tj1;
    m_t2_ = (Ta - Tj1);
    m_t3_ = Ta;
    m_t4_ = (Ta + Tv);
    m_t5_ = (T - Td + Tj2);
    m_t6_ = (T - Tj2);
    m_t7_ = T;
    m_vlim_ = vlim;
}
/**
 * @brief 加速段：Jerk 上升阶段的速度
 */
float SShapedPlanner1D::cal_Acc_JerkUpSpeed(float traveled)
{
    float t_low = 0.0f;
    float t_high = m_t1_;
    float t_mid, x_mid;
    for (int i = 0; i < 50; i++) // 二分法求解时间 t
    {
        t_mid = (t_low + t_high) / 2;
        x_mid = m_initialSpeed_ * t_mid + m_maxJerk_ * t_mid * t_mid * t_mid / 6;

        if (abs(x_mid - traveled) < 1e-10)
        {
            break;
        }
        else if (x_mid < traveled)
        {
            t_low = t_mid;
        }
        else
        {
            t_high = t_mid;
        }
    }
    return m_initialSpeed_ + 0.5f * m_maxJerk_ * t_mid * t_mid;
}

/**
 * @brief 加速段：加速度恒定阶段的速度
 */
float SShapedPlanner1D::cal_Acc_ConstSpeed(float traveled)
{
    float v_0 = m_initialSpeed_ + 0.5f * m_maxJerk_ * m_t1_ * m_t1_; // 加速段匀加速阶段初速度
    float v;
    arm_sqrt_f32((2.0f * (traveled - m_accelJerkUpDistance_) * m_maxAcc_ + v_0 * v_0), &v);
    return v;
}

/**
 * @brief 加速段：Jerk 下降阶段的速度
 */
float SShapedPlanner1D::cal_Acc_JerkDownSpeed(float traveled)
{
    float t_low = m_t2_;
    float t_high = m_t3_;
    float t_mid, x_mid;
    for (int i = 0; i < 50; i++) // 二分法求解时间 t
    {
        t_mid = (t_low + t_high) / 2;
        x_mid = (m_vlim_ + m_initialSpeed_) * m_t3_ / 2 - m_vlim_ * (m_t3_ - t_mid) + m_maxJerk_ * (m_t3_ - t_mid) * (m_t3_ - t_mid) * (m_t3_ - t_mid) / 6;

        if (abs(x_mid - traveled) < 1e-10)
        {
            break;
        }
        else if (x_mid < traveled)
        {
            t_low = t_mid;
        }
        else
        {
            t_high = t_mid;
        }
    }
    return m_vlim_ - m_maxJerk_ * (m_t3_ - t_mid) * (m_t3_ - t_mid) / 2;
}

/**
 * @brief 减速段：Jerk 上升阶段的速度
 */
float SShapedPlanner1D::cal_Dec_JerkUpSpeed(float traveled)
{

    float t_low = m_t4_;
    float t_high = m_t5_;
    float t_mid, x_mid;
    float Td = m_t7_ - m_t4_;
    for (int i = 0; i < 50; i++) // 二分法求解时间 t
    {
        t_mid = (t_low + t_high) / 2;
        x_mid = m_targetPos_ - (m_vlim_ + m_finalSpeed_) * Td / 2 + m_vlim_ * (t_mid - m_t4_) - m_maxJerk_ * (t_mid - m_t4_) * (t_mid - m_t4_) * (t_mid - m_t4_) / 6;

        if (abs(x_mid - traveled) < 1e-10)
        {
            break;
        }
        else if (x_mid < traveled)
        {
            t_low = t_mid;
        }
        else
        {
            t_high = t_mid;
        }
    }
    return m_vlim_ - m_maxJerk_ * (t_mid - m_t4_) * (t_mid - m_t4_) / 2;
}

/**
 * @brief 减速段：加速度恒定阶段的速度
 */
float SShapedPlanner1D::cal_Dec_ConstSpeed(float traveled)
{
    float v_0 = m_vlim_ - m_maxJerk_ * (m_t5_ - m_t4_) * (m_t5_ - m_t4_) / 2;
    float v;
    arm_sqrt_f32((v_0 * v_0 - 2.0f * (traveled - m_accelJerkUpDistance_ - m_accelConstDistance_ - m_accelJerkDownDistance_ - m_constVelDistance_ - m_decelJerkUpDistance_) * m_maxDec_), &v);
    return v;
}

/**
 * @brief 减速段：Jerk 下降阶段的速度
 */
float SShapedPlanner1D::cal_Dec_JerkDownSpeed(float traveled)
{
    float t_low = m_t6_;
    float t_high = m_t7_;
    float t_mid, x_mid;
    float Td = m_t7_ - m_t4_;
    for (int i = 0; i < 50; i++) // 二分法求解时间 t
    {
        t_mid = (t_low + t_high) / 2;
        x_mid = m_targetPos_ - m_finalSpeed_ * (m_t7_ - t_mid) - m_maxJerk_ * (m_t7_ - t_mid) * (m_t7_ - t_mid) * (m_t7_ - t_mid) / 6;

        if (abs(x_mid - traveled) < 1e-10)
        {
            break;
        }
        else if (x_mid < traveled)
        {
            t_low = t_mid;
        }
        else
        {
            t_high = t_mid;
        }
    }
    return m_finalSpeed_ + m_maxJerk_ * (m_t7_ - t_mid) * (m_t7_ - t_mid) / 2;
}

/**
 * @brief 重置 S 型速度规划器的内部状态。
 */
void SShapedPlanner1D::reset()
{
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
    m_deadzone_ = abs(params.deadzone);
    // 计算总路程：起点到目标点的直线距离
    Vector2D diff = params.targetPos - params.startPos;
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
Vector2D TrapePlanner2D::plan(Vector2D &now_dis)
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
    if (abs(traveled - m_totalDistance_) < m_deadzone_)
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
    m_phase = FINISHED_PHASE; // 设置阶段为规划结束
    m_totalDistance_ = 0;     // 重置总路程
    m_accelDistance_ = 0;     // 重置加速段长度
    m_decelDistance_ = 0;     // 重置减速段长度
    direction_ = 0;           // 重置运动方向
}

// ---------------------------- SShapedPlanner2D ----------------------------

/**
 * @brief 重置二维 S 型速度规划器的参数。
 * @param params 包含速度规划器参数的结构体。
 */
void SShapedPlanner2D::param_reset(Speedplanner_2D_Param_Config params)
{
    m_maxAcc_ = abs(params.maxAcc);
    m_maxDec_ = abs(params.maxDec);
    m_maxJerk_ = abs(params.maxJerk);
    m_maxSpeed_ = abs(params.maxSpeed);
    m_initialSpeed_ = abs(params.initialSpeed);
    m_finalSpeed_ = abs(params.finalSpeed);
    m_startPos_.x = 0.0f;
    m_startPos_.y = 0.0f;
    m_startPos1_ = params.startPos;
    m_targetPos_ = params.targetPos - params.startPos;
    m_deadzone_ = abs(params.deadzone);
    // 计算总距离
    Vector2D diff = params.targetPos - params.startPos;
    m_totalDistance_ = diff.magnitude();

    // 预计算各个阶段的路程
    cal_PhaseDistances();

    // 初始化当前阶段
    m_phase = S_ACCEL_JERK_UP_PHASE;
}

/**
 * @brief 构造函数，初始化二维 S 型速度规划器。
 * @param params 包含速度规划器参数的结构体。
 */
SShapedPlanner2D::SShapedPlanner2D(Speedplanner_2D_Param_Config params)
{
    param_reset(params); // 调用参数重置函数
}

/**
 * @brief 根据当前已行驶的距离，计算目标速度。
 * @param now_dis 当前已行驶的距离。
 * @return 规划的目标速度。
 */
Vector2D SShapedPlanner2D::plan(Vector2D &now_dis1)
{
    // 计算路径及单位方向
    Vector2D now_dis = now_dis1 - m_startPos1_;
    if (now_dis.x < 0.000001f)
    {
        now_dis.x = 0.000001f;
    }
    if (now_dis.y < 0.000001f)
    {
        now_dis.y = 0.000001f;
    }

    Vector2D path = m_targetPos_ - now_dis;
    Vector2D direction = path.normalize();

    if (m_totalDistance_ < 0.0001f)
    {
        m_phase = S_FINISHED_PHASE;
        return Vector2D(0, 0);
    }

    // 计算当前位置在规划路径上的投影距离
    Vector2D delta = now_dis - m_startPos_;
    float traveled = delta * direction;
    // float traveled = delta.magnitude();

    if (traveled < 0)
        traveled = 0;
    if (traveled >= m_totalDistance_)
    {
        return m_finalSpeed_ * (m_targetPos_ - m_startPos_).normalize();
    }

    // 计算当前位置与目标点之间的直线距离
    // float distanceToTarget = (m_targetPos_ - now_dis).magnitude();

    // 确定当前阶段
    m_phase = determinePhase(traveled);

    // 根据阶段计算当前速度
    float currentSpeed = 0.0f;

    switch (m_phase)
    {
    case S_ACCEL_JERK_UP_PHASE:
        currentSpeed = cal_Acc_JerkUpSpeed(traveled);
        break;
    case S_ACCEL_CONST_PHASE:
        currentSpeed = cal_Acc_ConstSpeed(traveled);
        break;
    case S_ACCEL_JERK_DOWN_PHASE:
        currentSpeed = cal_Acc_JerkDownSpeed(traveled);
        break;
    case S_CONST_VEL_PHASE:
        currentSpeed = m_maxSpeed_;
        break;
    case S_DECEL_JERK_UP_PHASE:
        currentSpeed = cal_Dec_JerkUpSpeed(traveled);
        break;
    case S_DECEL_CONST_PHASE:
        currentSpeed = cal_Dec_ConstSpeed(traveled);
        break;
    case S_DECEL_JERK_DOWN_PHASE:
        currentSpeed = cal_Dec_JerkDownSpeed(traveled);
        break;
    case S_FINISHED_PHASE:
        currentSpeed = m_finalSpeed_;
        break;
    }

    return direction * currentSpeed;
}

/**
 * @brief 根据已行驶的距离确定当前所处的 S 型规划阶段。
 * @param traveled 已行驶的距离（从起始位置算起）。
 * @return 当前 S 型规划阶段。
 */
SPhase SShapedPlanner2D::determinePhase(float traveled)
{
    // 检查是否已经完成
    if (abs(traveled - m_totalDistance_) < m_deadzone_)
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
 * @brief 计算各阶段的距离。
 */
void SShapedPlanner2D::cal_PhaseDistances()
{
    // == == == == == 计算加速段参数 == == == == ==
    // 判断是否能达到最大加速度
    float Tj1, Ta, alima, Tj2, Td, alimd, Tv, vlim, T;
    if ((m_maxSpeed_ - m_initialSpeed_) * m_maxJerk_ < m_maxAcc_ * m_maxAcc_) // 不能达到最大加速度的情况
    {
        if (m_initialSpeed_ > m_maxSpeed_) // 如果初始速度已经大于最大速度
        {
            Tj1 = 0;   // 加加速时间为0
            Ta = 0;    // 总加速时间为0
            alima = 0; // 实际最大加速度为0
        }

        else // 正常情况下的计算
        {
            arm_sqrt_f32((m_maxSpeed_ - m_initialSpeed_) / m_maxJerk_, &Tj1); // 加加速时间
            Ta = 2 * Tj1;                                                     // 总加速时间（对称的加加速和减加速）
            alima = Tj1 * m_maxJerk_;                                         // 实际能达到的最大加速度
        }
    }
    else // 能达到最大加速度的情况
    {
        Tj1 = m_maxAcc_ / m_maxJerk_;                           // 加加速时间
        Ta = Tj1 + (m_maxSpeed_ - m_initialSpeed_) / m_maxAcc_; // 总加速时间
        alima = m_maxAcc_;                                      // 实际最大加速度等于设定最大加速度
    }

    //========== 计算减速段参数 ==========
    // 判断是否能达到最大减速度
    if ((m_maxSpeed_ - m_finalSpeed_) * m_maxJerk_ < m_maxDec_ * m_maxDec_) // 不能达到最大减速度的情况
    {
        arm_sqrt_f32((m_maxSpeed_ - m_finalSpeed_) / m_maxJerk_, &Tj2); // 加减速时间
        Td = 2 * Tj2;                                                   // 总减速时间
        alimd = Tj2 * m_maxJerk_;                                       // 实际最大减速度
    }
    else // 能达到最大减速度的情况
    {
        Tj2 = m_maxDec_ / m_maxJerk_;                         // 加减速时间
        Td = Tj2 + (m_maxSpeed_ - m_finalSpeed_) / m_maxDec_; // 总减速时间
        alimd = m_maxDec_;                                    // 实际最大减速度等于设定最大减速度
    }

    // ========== 计算匀速段时间 ==========
    Tv = (m_targetPos_ - m_startPos_).magnitude() / m_maxSpeed_ - Ta / 2 * (1 + m_initialSpeed_ / m_maxSpeed_) - Td / 2 * (1 + m_finalSpeed_ / m_maxSpeed_);

    // ========== 处理不存在匀速阶段的情况 ==========
    if (Tv > 0) // 存在匀速阶段
    {
        vlim = m_maxSpeed_; // 限制速度等于最大速度
        T = Tv + Ta + Td;   // 总时间
    }
    else // 不存在匀速阶段
    {
        Tv = 0;                           // 匀速时间为0
        float amax_accel_org = m_maxAcc_; // 保存原始加速段最大加速度值
        float amax_decel_org = m_maxDec_; // 保存原始减速段最大加速度值
        int count = 0;                    // 调整次数计数器
        // 计算delta值，用于求解时间参数
        // 由于现在有两个不同的加速度，需要分别计算加速段和减速段
        // 这里使用平均加速度来近似计算
        float a_avg = (m_maxAcc_ + m_maxDec_) / 2;
        float delta = (a_avg * a_avg * a_avg * a_avg) / (m_maxJerk_ * m_maxJerk_) + 2 * (m_initialSpeed_ * m_initialSpeed_ + m_finalSpeed_ * m_finalSpeed_) + a_avg * (4 * (m_targetPos_ - m_startPos_).magnitude() - 2 * a_avg / m_maxJerk_ * (m_initialSpeed_ + m_finalSpeed_));

        // 初始时间参数计算（使用平均加速度）
        Tj1 = m_maxAcc_ / m_maxJerk_;
        Ta = (a_avg * a_avg / m_maxJerk_ - 2 * m_initialSpeed_ + sqrt(delta)) / (2 * a_avg);
        Tj2 = m_maxDec_ / m_maxJerk_;
        Td = (a_avg * a_avg / m_maxJerk_ - 2 * m_finalSpeed_ + sqrt(delta)) / (2 * a_avg);
        vlim = m_initialSpeed_ + (Ta - Tj1) * alima; // 计算实际达到的最大速度

        // 逐渐减少加速度，直到找到可行的解
        while (Ta < 2 * Tj1 || Td < 2 * Tj2)
        {
            count += 1;
            // 同时减少加速段和减速段的加速度，保持比例关系
            float reduction_factor = 0.9;             // 每次减少10%
            m_maxAcc_ = m_maxAcc_ * reduction_factor; // 保持最小加速度
            m_maxDec_ = m_maxDec_ * reduction_factor; // 保持最小加速度

            // 重新计算加速段参数
            if ((m_maxSpeed_ - m_initialSpeed_) * m_maxJerk_ < m_maxAcc_ * m_maxAcc_)
            {
                arm_sqrt_f32((m_maxSpeed_ - m_initialSpeed_) / m_maxJerk_, &Tj1);
                Ta = 2 * Tj1;
                alima = Tj1 * m_maxJerk_;
            }
            else
            {
                Tj1 = m_maxAcc_ / m_maxJerk_;
                Ta = Tj1 + (m_maxSpeed_ - m_initialSpeed_) / m_maxAcc_;
                alima = m_maxAcc_;
            }

            // 重新计算减速段参数
            if ((m_maxSpeed_ - m_finalSpeed_) * m_maxJerk_ < m_maxDec_ * m_maxDec_)
            {
                arm_sqrt_f32((m_maxSpeed_ - m_finalSpeed_) / m_maxJerk_, &Tj2);
                Td = 2 * Tj2;
                alimd = Tj2 * m_maxJerk_;
            }
            else
            {
                Tj2 = m_maxDec_ / m_maxJerk_;
                Td = Tj2 + (m_maxSpeed_ - m_finalSpeed_) / m_maxDec_;
                alimd = m_maxDec_;
            }
            // 重新计算平均加速度和delta值
            a_avg = (m_maxAcc_ + m_maxDec_) / 2;
            if (a_avg > 0)
            {
                delta = (a_avg * a_avg * a_avg * a_avg) / (m_maxJerk_ * m_maxJerk_) + 2 * (m_initialSpeed_ * m_initialSpeed_ + m_finalSpeed_ * m_finalSpeed_) + a_avg * (4 * (m_targetPos_ - m_startPos_).magnitude() - 2 * a_avg / m_maxJerk_ * (m_initialSpeed_ + m_finalSpeed_));
            }

            else
            {
                delta = (a_avg * a_avg * a_avg * a_avg) / (m_maxJerk_ * m_maxJerk_) + 2 * (m_initialSpeed_ * m_initialSpeed_ + m_finalSpeed_ * m_finalSpeed_) - a_avg * (4 * (m_targetPos_ - m_startPos_).magnitude() - 2 * a_avg / m_maxJerk_ * (m_initialSpeed_ + m_finalSpeed_));
            }

            // 重新计算时间参数
            Ta = (a_avg * a_avg / m_maxJerk_ - 2 * m_initialSpeed_ + sqrt(delta)) / (2 * a_avg);
            Td = (a_avg * a_avg / m_maxJerk_ - 2 * m_finalSpeed_ + sqrt(delta)) / (2 * a_avg);
            vlim = m_initialSpeed_ + (Ta - Tj1) * alima; // 重新计算实际最大速度

            // 防止无限循环
            if (count > 100)
            {
                m_maxAcc_ = 0.0f;            // 最大加速度
                m_maxDec_ = 0.0f;            // 最大减速度
                m_maxJerk_ = 0.0f;           // 最大加加速度
                m_maxSpeed_ = 0.0f;          // 最大速度
                m_initialSpeed_ = 0.0f;      // 起始速度
                m_finalSpeed_ = 0.0f;        // 目标速度
                m_startPos_ = {0.0f, 0.0f};  // 起始位置
                m_targetPos_ = {0.0f, 0.0f}; // 目标位置
                m_totalDistance_ = 0.0f;     // 总路程
                m_deadzone_ = 0.0f;          // 死区范围
                err_ = 1;                    // 设置错误标志
                break;
            }
        }
        // 处理加速或减速时间为负的情况
        if (Ta < 0 || Td < 0)
        {
            if (m_initialSpeed_ > m_finalSpeed_) // 初始速度大于目标速度，主要是减速
            {
                Ta = 0;
                Tj1 = 0;
                alima = 0;
                Td = 2 * (m_targetPos_ - m_startPos_).magnitude() / (m_finalSpeed_ + m_initialSpeed_);
                Tj2 = (m_maxJerk_ * (m_targetPos_ - m_startPos_).magnitude() - sqrt(m_maxJerk_ * (m_maxJerk_ * (m_targetPos_ - m_startPos_).magnitude() * (m_targetPos_ - m_startPos_).magnitude() + (m_finalSpeed_ + m_initialSpeed_) * (m_finalSpeed_ + m_initialSpeed_) * (m_finalSpeed_ - m_initialSpeed_)))) / (m_maxJerk_ * (m_finalSpeed_ + m_initialSpeed_));
                alimd = -m_maxJerk_ * Tj2;
                vlim = m_finalSpeed_ - (Td - Tj2) * alimd;
                alimd = -alimd;
            }

            else // 主要是加速
            {
                Td = 0;
                Tj2 = 0;
                Ta = 2 * (m_targetPos_ - m_startPos_).magnitude() / (m_finalSpeed_ + m_initialSpeed_);
                Tj1 = (m_maxJerk_ * (m_targetPos_ - m_startPos_).magnitude() - sqrt(m_maxJerk_ * (m_maxJerk_ * (m_targetPos_ - m_startPos_).magnitude() * (m_targetPos_ - m_startPos_).magnitude() - (m_finalSpeed_ + m_initialSpeed_) * (m_finalSpeed_ + m_initialSpeed_) * (m_finalSpeed_ - m_initialSpeed_)))) / (m_maxJerk_ * (m_finalSpeed_ + m_initialSpeed_));
                alima = m_maxJerk_ * Tj1;
                vlim = m_initialSpeed_ + (Ta - Tj1) * alima;
            }
        }

        T = Tv + Ta + Td; // 计算总时间
    }

    float T2_duration, v_start_stage2, T6_duration, v_start_stage6;
    // ========== 计算各阶段路程 ==========
    // 阶段1: 加加速阶段 (0 <= t < Tj1)
    m_accelJerkUpDistance_ = m_initialSpeed_ * Tj1 + m_maxJerk_ * Tj1 * Tj1 * Tj1 / 6;

    // 阶段2: 匀加速阶段 (Tj1 <= t < (Ta - Tj1))
    T2_duration = Ta - 2 * Tj1;
    v_start_stage2 = m_initialSpeed_ + m_maxJerk_ * Tj1 * Tj1 / 2;
    m_accelConstDistance_ = v_start_stage2 * T2_duration + 0.5 * alima * T2_duration * T2_duration;

    // 阶段3: 减加速阶段 ((Ta - Tj1) <= t < Ta)
    m_accelJerkDownDistance_ = vlim * Tj1 - m_maxJerk_ * Tj1 * Tj1 * Tj1 / 6;

    // 阶段4: 匀速阶段 (Ta <= t < (Ta + Tv))
    m_constVelDistance_ = vlim * Tv;

    // 阶段5: 加减速阶段 ((T - Td) <= t < (T - Td + Tj2))
    m_decelJerkUpDistance_ = vlim * Tj2 - m_maxJerk_ * Tj2 * Tj2 * Tj2 / 6;

    // 阶段6: 匀减速阶段 ((T - Td + Tj2) <= t < (T - Tj2))
    T6_duration = Td - 2 * Tj2;
    v_start_stage6 = vlim - m_maxJerk_ * Tj2 * Tj2 / 2;
    m_decelConstDistance_ = v_start_stage6 * T6_duration - 0.5 * alimd * T6_duration * T6_duration;

    // 阶段7: 减减速阶段 ((T - Tj2) <= t < T)
    m_decelJerkDownDistance_ = m_finalSpeed_ * Tj2 + m_maxJerk_ * Tj2 * Tj2 * Tj2 / 6;

    m_t1_ = Tj1;
    m_t2_ = (Ta - Tj1);
    m_t3_ = Ta;
    m_t4_ = (Ta + Tv);
    m_t5_ = (T - Td + Tj2);
    m_t6_ = (T - Tj2);
    m_t7_ = T;
    m_vlim_ = vlim;
}

/**
 * @brief 加速段：Jerk 上升阶段的速度
 */
float SShapedPlanner2D::cal_Acc_JerkUpSpeed(float traveled)
{
    float t_low = 0.0f;
    float t_high = m_t1_;
    float t_mid, x_mid;
    for (int i = 0; i < 50; i++) // 二分法求解时间 t
    {
        t_mid = (t_low + t_high) / 2;
        x_mid = m_initialSpeed_ * t_mid + m_maxJerk_ * t_mid * t_mid * t_mid / 6;

        if (abs(x_mid - traveled) < 1e-10)
        {
            break;
        }
        else if (x_mid < traveled)
        {
            t_low = t_mid;
        }
        else
        {
            t_high = t_mid;
        }
    }
    return m_initialSpeed_ + 0.5f * m_maxJerk_ * t_mid * t_mid;
}

/**
 * @brief 加速段：加速度恒定阶段的速度
 */
float SShapedPlanner2D::cal_Acc_ConstSpeed(float traveled)
{
    float v_0 = m_initialSpeed_ + 0.5f * m_maxJerk_ * m_t1_ * m_t1_; // 加速段匀加速阶段初速度
    float v;
    arm_sqrt_f32((2.0f * (traveled - m_accelJerkUpDistance_) * m_maxAcc_ + v_0 * v_0), &v);
    return v;
}

/**
 * @brief 加速段：Jerk 下降阶段的速度
 */
float SShapedPlanner2D::cal_Acc_JerkDownSpeed(float traveled)
{
    float t_low = m_t2_;
    float t_high = m_t3_;
    float t_mid, x_mid;
    for (int i = 0; i < 50; i++) // 二分法求解时间 t
    {
        t_mid = (t_low + t_high) / 2;
        x_mid = (m_vlim_ + m_initialSpeed_) * m_t3_ / 2 - m_vlim_ * (m_t3_ - t_mid) + m_maxJerk_ * (m_t3_ - t_mid) * (m_t3_ - t_mid) * (m_t3_ - t_mid) / 6;

        if (abs(x_mid - traveled) < 1e-10)
        {
            break;
        }
        else if (x_mid < traveled)
        {
            t_low = t_mid;
        }
        else
        {
            t_high = t_mid;
        }
    }
    return m_vlim_ - m_maxJerk_ * (m_t3_ - t_mid) * (m_t3_ - t_mid) / 2;
}

/**
 * @brief 减速段：Jerk 上升阶段的速度
 */
float SShapedPlanner2D::cal_Dec_JerkUpSpeed(float traveled)
{

    float t_low = m_t4_;
    float t_high = m_t5_;
    float t_mid, x_mid;
    float Td = m_t7_ - m_t4_;
    for (int i = 0; i < 50; i++) // 二分法求解时间 t
    {
        t_mid = (t_low + t_high) / 2;
        x_mid = (m_targetPos_ - m_startPos_).magnitude() - (m_vlim_ + m_finalSpeed_) * Td / 2 + m_vlim_ * (t_mid - m_t4_) - m_maxJerk_ * (t_mid - m_t4_) * (t_mid - m_t4_) * (t_mid - m_t4_) / 6;

        if (abs(x_mid - traveled) < 1e-10)
        {
            break;
        }
        else if (x_mid < traveled)
        {
            t_low = t_mid;
        }
        else
        {
            t_high = t_mid;
        }
    }
    return m_vlim_ - m_maxJerk_ * (t_mid - m_t4_) * (t_mid - m_t4_) / 2;
}

/**
 * @brief 减速段：加速度恒定阶段的速度
 */
float SShapedPlanner2D::cal_Dec_ConstSpeed(float traveled)
{
    float v_0 = m_vlim_ - m_maxJerk_ * (m_t5_ - m_t4_) * (m_t5_ - m_t4_) / 2;
    float v;
    arm_sqrt_f32((v_0 * v_0 - 2.0f * (traveled - m_accelJerkUpDistance_ - m_accelConstDistance_ - m_accelJerkDownDistance_ - m_constVelDistance_ - m_decelJerkUpDistance_) * m_maxDec_), &v);
    return v;
}

/**
 * @brief 减速段：Jerk 下降阶段的速度
 */
float SShapedPlanner2D::cal_Dec_JerkDownSpeed(float traveled)
{
    float t_low = m_t6_;
    float t_high = m_t7_;
    float t_mid, x_mid;
    float Td = m_t7_ - m_t4_;
    for (int i = 0; i < 50; i++) // 二分法求解时间 t
    {
        t_mid = (t_low + t_high) / 2;
        x_mid = (m_targetPos_ - m_startPos_).magnitude() - m_finalSpeed_ * (m_t7_ - t_mid) - m_maxJerk_ * (m_t7_ - t_mid) * (m_t7_ - t_mid) * (m_t7_ - t_mid) / 6;

        if (abs(x_mid - traveled) < 1e-10)
        {
            break;
        }
        else if (x_mid < traveled)
        {
            t_low = t_mid;
        }
        else
        {
            t_high = t_mid;
        }
    }
    return m_finalSpeed_ + m_maxJerk_ * (m_t7_ - t_mid) * (m_t7_ - t_mid) / 2;
}

/**
 * @brief 重置 S 型速度规划器的内部状态。
 */
void SShapedPlanner2D::reset()
{
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
// ---------------------------- Td平滑器 ----------------------------
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

ADRC::ADRC(ADRC_Param_Config params)
{
    ADRC_Param_Init(params);
}

void ADRC::ADRC_Param_Init(ADRC_Param_Config params)
{
    output_limit = params.output_limit;

    r = params.r;
    h = params.h;

    b = params.b;
    delta = params.delta;
    beta_01 = params.beta_01;
    beta_02 = params.beta_02;
    beta_03 = params.beta_03;

    alpha_1 = params.alpha_1;
    alpha_2 = params.alpha_2;
    beta_1 = params.beta_1;
    beta_2 = params.beta_2;
}

float ADRC::ADRC_Calculate(bool normalization, float unit)
{

    // 跟踪微分器TD
    v1 = v1_last + h * v2_last;
    v2 = v2_last + h * fst(v1_last - v, v2_last, r, h);

    v1_last = v1;
    v2_last = v2;

    // 扩张观测器ESO
    float e = z1 - y;

    z1 = z1 + h * (z2 - beta_01 * e);
    z2 = z2 + h * (z3 - beta_02 * fal(e, alpha_1, delta) + b * u);
    z3 = z3 - h * beta_03 * fal(e, alpha_2, delta);

    // 非线性组合NLSEF
    e1 = v1 - z1;
    e2 = v2 - z2;

    e2 = e2_last * 0.3f + e2 * 0.7f;

    e2_last = e2;

    u = beta_1 * fal(e1, alpha_1, delta) + beta_2 * fal(e2, alpha_2, delta);

    // 扰动补偿
    u0 = u - z3 / b;

    // 输出限幅
    if (u0 > output_limit)
        u0 = output_limit;
    else if (u0 < -output_limit)
        u0 = -output_limit;

    u0 = u0_last * 0.3f + u0 * 0.7f;

    u0_last = u0;

    return u0;
}

/**
 * @brief 最速控制综合函数
 * @note
 * @param x:
 * @retval
 */
float ADRC::fst(float x1_, float x2_, float r_, float h_)
{
    float d = r_ * h_;
    float d0 = h_ * d;
    float y = x1_ + h_ * x2_;

    float a0 = sqrtf(d * d + 8.f * r_ * fabsf(y));
    float a;

    if (fabsf(y) > d0)
    {
        a = x2_ + (a0 - d) / 2.f * sgn(y);
    }
    else
    {
        a = x2_ + y / h_;
    }

    if (fabsf(a) > d)
    {
        return -r_ * sgn(a);
    }
    else
    {
        return -r_ * a / d;
    }
}

float ADRC::fhan(float x1, float x2, float r, float h)
{
    float d = r * h * h;
    float a0 = h * x2;
    float y = x1 + a0;
    float a1 = sqrtf(d * (d + 8.f * fabsf(y)));
    float a2 = a0 + sgn(y) * (a1 - d) / 2.f;
    return -r * (a2 / d) * (fabsf(a2) <= d) + -r * sgn(a2) * (fabsf(a2) > d);
}

/**
 * @brief  饱和函数
 * @note
 * @param delta_:线性段区间长度
 * @retval
 */
float ADRC::fal(float e_, float alpha_, float delta_)
{
    if (fabsf(e_) <= delta_)
    {
        return e_ / powf(delta_, alpha_ - 1.f);
    }
    else
    {
        return powf(fabsf(e_), alpha_) * sgn(e_);
    }
}

/**
 * @brief  符号函数
 * @note
 * @param x_:
 * @retval
 */
float ADRC::sgn(float x_)
{
    if (x_ > 0.f)
    {
        return 1.f;
    }
    else if (x_ == 0.f)
    {
        return 0.f;
    }
    else
    {
        return -1.f;
    }
}
