#ifndef APP_SLOPECOMPENSATE_H
#define APP_SLOPECOMPENSATE_H

#include <stdint.h>
#include <stdbool.h>
#include "APP_tool.h"

typedef struct {
    float wheel_radius;
    float gear_ratio;
    float chassis_mass;
    float k_rear;// 重心后移系数
    float f_mec;// 麦克纳姆轮滚动阻力系数
    float motor_k_t;// 电机扭矩常数
    float max_comp_current;// 最大补偿电流
    float g;
    float slope_threshold;// 坡度阈值（sin值）
    float low_speed_thr;// 低速阈值
    float low_speed_ratio;// 低速补偿比例
    float stop_comp_ratio;// 静止补偿比例
    float comp_smooth_rate;// 补偿电流平滑速率（A/s）
} SlopeCompensateConfig;

typedef struct {
    float pitch;
    float yaw;
    float motor_speed[4];
    float motor_current[4];
    float target_speed[4];
} SlopeInputData;


#ifdef __cplusplus

#include <array>

class SlopeCompensator {
public:
    SlopeCompensator();
    int init(const SlopeCompensateConfig& cfg, float control_period_s = 0.001f);
    // 计算补偿电流
    void calc(const SlopeInputData& input, float comp_current[4]);
   
    // 可选：访问当前配置
    const SlopeCompensateConfig& config() const { return cfg_; }

private:
    SlopeCompensateConfig cfg_{};
    std::array<float, 4> last_comp_{}; // 上一周期的补偿电流
    bool inited_ = false;
    float control_period_s_ = 0.001f;

    float calc_wheel_mass(uint8_t wheel_idx, float sin_theta) const;// 计算单轮分配的等效质量
    float calc_single_base_comp(uint8_t wheel_idx, float sin_theta, float cos_theta, float tan_yaw) const;
    float smooth_comp_current(float target_comp, float last_comp) const;
    void smooth_comp_to_zero(float comp_current[4]);
};

#endif // __cplusplus

#endif // APP_SLOPECOMPENSATE_H