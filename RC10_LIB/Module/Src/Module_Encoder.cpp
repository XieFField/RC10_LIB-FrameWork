#include "Module_Encoder.h"

void Encoder::update(uint16_t raw_value)
{
    const float deg_per_count = 360.0f / static_cast<float>(range_);

    if (!is_init_)
    {
        offset_   = raw_value;
        last_raw_ = raw_value;
        is_init_  = true;

        // 以当前读数为零相位，初始化单圈角与连续角
        int32_t rel_cnt = static_cast<int32_t>(raw_value) - static_cast<int32_t>(offset_);
        float mod_deg = fmodf(rel_cnt * deg_per_count, 360.0f);
        if (mod_deg < 0) mod_deg += 360.0f;

        last_mod_deg_ = mod_deg;
        cont_deg_     = mod_deg;     // 连续角从当前位置起步
        total_angle_  = cont_deg_ + bias_deg_;
        angle_        = normalize_deg_0_360(total_angle_);
        return;
    }

    // 计算当前单圈角(相对上电原点)
    int32_t rel_cnt = static_cast<int32_t>(raw_value) - static_cast<int32_t>(offset_);
    float mod_deg = fmodf(rel_cnt * deg_per_count, 360.0f);
    if (mod_deg < 0) mod_deg += 360.0f;

    // 单圈角差取最短路（±180），积分到连续角
    float delta = normalize_deg_pm180(mod_deg - last_mod_deg_);
    cont_deg_   += delta;
    last_mod_deg_ = mod_deg;
    last_raw_     = raw_value;

    // 连续角 + 偏置 = 总角；显示角做 0..360 归一化
    total_angle_ = cont_deg_ + bias_deg_;
    angle_       = normalize_deg_0_360(total_angle_);
}

void Encoder::relocate_totalAngle(float now_totalAngle)
{
    // 仅调整偏置，使 total_angle_ 立即等于指定值；不重置连续态，避免跳变
    bias_deg_   = now_totalAngle - cont_deg_;
    total_angle_ = now_totalAngle;
    angle_       = normalize_deg_0_360(total_angle_);
}
