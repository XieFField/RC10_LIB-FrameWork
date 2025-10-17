#include "Module_Encoder.h"

void Encoder::update(uint16_t raw_value)
{
    const float deg_per_count = 360.0f / static_cast<float>(range_);

    if (!is_init_) 
    {
        offset_ = raw_value;
        last_ = raw_value;
        is_init_ = true;
        round_cnt_ = 0;
        return;
    }

    int32_t delta = static_cast<int32_t>(raw_value) - static_cast<int32_t>(last_);

    if (delta > range_ / 2) 
        round_cnt_--;

    else if (delta < -static_cast<int32_t>(range_) / 2) 
        round_cnt_++;
    
    last_ = raw_value;
    int32_t total_encoder = round_cnt_ * static_cast<int32_t>(range_) +
                            static_cast<int32_t>(raw_value) - static_cast<int32_t>(offset_);

    float total_angle_nobias = static_cast<float>(total_encoder) * deg_per_count;

    total_angle_ = total_angle_nobias + bias_deg_;
    
    

    // 使用 fmodf 保证 angle_ 在 [0, 360) 或 (-360, 0) 范围内，然后处理负数情况
    angle_ = fmodf(total_angle_, 360.0f);
    if (angle_ < 0) 
        angle_ += 360.0f;
    
}

void Encoder::relocate_totalAngle(float now_totalAngle)
{
    const float deg_per_count = 360.0f / static_cast<float>(range_);

    int32_t total_encoder = round_cnt_ * static_cast<int32_t>(range_) +
                            static_cast<int32_t>(last_) - static_cast<int32_t>(offset_);

    float total_angle_nobias = static_cast<float>(total_encoder) * deg_per_count;

    bias_deg_ = now_totalAngle - total_angle_nobias;

    total_angle_ = now_totalAngle;

    angle_ = fmodf(total_angle_, 360.0f);
    if (angle_ < 0) 
        angle_ += 360.0f;
    
}
