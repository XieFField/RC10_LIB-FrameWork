#include "Module_Encoder.h"

void Encoder::update(uint16_t raw_value)
{
    if (!is_init_) 
    {
        offset_ = raw_value;
        last_ = raw_value;
        is_init_ = true;
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
    total_angle_ = static_cast<float>(total_encoder) / (static_cast<float>(range_) / 360.0f);
    
    // 使用 fmodf 保证 angle_ 在 [0, 360) 或 (-360, 0) 范围内，然后处理负数情况
    angle_ = fmodf(total_angle_, 360.0f);
    if (angle_ < 0) {
        angle_ += 360.0f;
    }
}