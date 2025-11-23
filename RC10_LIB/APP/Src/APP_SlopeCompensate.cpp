#include "APP_SlopeCompensate.h"
#include <cmath>
#include <cstring>


SlopeCompensator::SlopeCompensator() {
    inited_ = false;
    last_comp_.fill(0.0f);
}

/**
 * @brief 初始化
 * @param cfg 补偿配置
 * @param control_period_s 控制周期 (s)
 * @return 0 表示成功, -1 表示失败
 */
int SlopeCompensator::init(const SlopeCompensateConfig& cfg, float control_period_s) {
    if (cfg.motor_k_t <= 1e-6f) return -1; // 核心参数校验
    cfg_ = cfg;
    inited_ = true;
    control_period_s_ = (control_period_s > 0.0f) ? control_period_s : 0.001f; // 安全回退
    last_comp_.fill(0.0f);
    return 0;
}

/**
 * @brief 计算单轮分配的等效质量
 * @details 根据重心后移模型，在上坡时为后轮分配更多质量，以更好地模拟重力效应。
 * @param wheel_idx 车轮索引 (0-3)
 * @param sin_theta 坡度角的正弦值
 * @return 单轮分配的质量 (kg)
 */
float SlopeCompensator::calc_wheel_mass(uint8_t wheel_idx, float sin_theta) const {
    float base_mass = cfg_.chassis_mass / 4.0f;
    if (wheel_idx >= 2) { // 后两轮
        return base_mass * (1.0f + cfg_.k_rear * sin_theta);
    } else { // 前两轮
        return base_mass * (1.0f - cfg_.k_rear * sin_theta);
    }
}

/**
 * @brief 计算单轮的基础补偿电流
 * @details
 *  1.  **重力分力**: 克服重力沿坡道向下的分力所需的主要扭矩。
 *  2.  **滚动阻力**: 克服麦克纳姆轮滚动时产生的摩擦阻力。
 *  3.  **斜向损耗**: 当车身航向与坡道方向不一致时，部分驱动力会损耗，需要额外补偿。
 *  最后将总阻力扭矩根据电机扭矩常数转换为电流。
 * @param wheel_idx 车轮索引
 * @param sin_theta sin(坡度角)
 * @param cos_theta cos(坡度角)
 * @param tan_yaw tan(航向角)
 * @return 单轮所需的基础补偿电流 (A)
 */
float SlopeCompensator::calc_single_base_comp(uint8_t wheel_idx, float sin_theta, float cos_theta, float tan_yaw) const {
    float m_i = calc_wheel_mass(wheel_idx, sin_theta);
    float gravity_force = m_i * cfg_.g * sin_theta;// 重力分力
    float roll_force = m_i * cfg_.g * cos_theta * cfg_.f_mec;// 滚动阻力
    float decomp_force = gravity_force * std::fabs(tan_yaw);// 斜向损耗
    float total_torque = (gravity_force + roll_force + decomp_force) * cfg_.wheel_radius;// 总扭矩
    if (cfg_.motor_k_t <= 1e-6f) return 0.0f;
    return total_torque / cfg_.motor_k_t;// 转换为电流
}

/**
 * @brief 对目标补偿电流进行平滑处理
 * @details 限制补偿电流的单周期变化率，防止电流突变对电机和电源造成冲击。
 * @param target_comp 目标补偿电流
 * @param last_comp 上一周期的补偿电流
 * @return 平滑后的补偿电流
 */
float SlopeCompensator::smooth_comp_current(float target_comp, float last_comp) const {
    // 使用通用的 ramp 函数进行平滑，保持与配置的 comp_smooth_rate 一致。
    float cur = last_comp;
    float rate = cfg_.comp_smooth_rate;
    float dt = (control_period_s_ > 0.0f) ? control_period_s_ : 0.001f;
    ramp(target_comp, cur, rate, dt);
    return cur;
}

/**
 * @brief 将补偿电流平滑地减小到零
 * @details 当补偿条件不再满足时（如离开坡道），用于平滑地撤销补偿电流。
 * @param comp_current [out] 输出的补偿电流数组
 */
void SlopeCompensator::smooth_comp_to_zero(float comp_current[4]) {
    // 使用通用 ramp 将每个轮的补偿平滑回 0，速率由 cfg_.comp_smooth_rate 控制
    float dt = (control_period_s_ > 0.0f) ? control_period_s_ : 0.001f;
    for (uint8_t i = 0; i < 4; ++i) {
        ramp(0.0f, last_comp_[i], cfg_.comp_smooth_rate, dt);
        comp_current[i] = last_comp_[i];
    }
}

/**
 * @brief 计算斜坡补偿电流
 * @details
 *  - 首先检查是否需要补偿（坡度是否超过阈值）。
 *  - 遍历四轮，计算每个车轮的基础补偿电流。
 *  - 根据车轮速度判断是否处于低速状态，若是则增强补偿以防溜坡。
 *  - 对计算出的补偿电流进行限幅，确保不超过设定的最大值，并防止总电流超限。
 *  - 最后进行平滑处理并输出。
 * @param input 输入数据
 * @param comp_current [out] 输出补偿电流
 */
void SlopeCompensator::calc(const SlopeInputData& input, float comp_current[4]) {
    if (!inited_ || comp_current == nullptr) {
        if (comp_current) std::memset(comp_current, 0, sizeof(float)*4);
        return;
    }

    float pitch = input.pitch;
    float sin_theta = std::sinf(pitch);
    float cos_theta = std::cosf(pitch);
    float tan_yaw = 0.0f;
    if (std::fabs(std::cosf(input.yaw)) > 1e-3f) {
        tan_yaw = std::tanf(input.yaw);
    }
    tan_yaw = constrain(tan_yaw, -10.0f, 10.0f);

    if (sin_theta <= cfg_.slope_threshold || pitch <= 0.0f) {
        smooth_comp_to_zero(comp_current);
        return;
    }

    for (uint8_t i = 0; i < 4; ++i) {
        float base_comp = calc_single_base_comp(i, sin_theta, cos_theta, tan_yaw);
        bool is_low_speed = (std::fabs(input.motor_speed[i]) < cfg_.low_speed_thr);
        if (is_low_speed) base_comp *= cfg_.low_speed_ratio;
        float max_allow_comp = cfg_.max_comp_current;
        float current_base = input.motor_current[i] - last_comp_[i];
        float max_comp_by_current = (cfg_.max_comp_current * 1.2f) - current_base;
        if (max_comp_by_current < max_allow_comp) max_allow_comp = max_comp_by_current;
        if (base_comp > max_allow_comp) base_comp = max_allow_comp;
        if (base_comp < 0.0f) base_comp = 0.0f;
        float sm = smooth_comp_current(base_comp, last_comp_[i]);
        comp_current[i] = sm;
        last_comp_[i] = sm;
    }
}
