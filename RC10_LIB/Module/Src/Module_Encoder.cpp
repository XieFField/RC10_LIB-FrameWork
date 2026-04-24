#include "Module_Encoder.h"

void Encoder::update(uint16_t raw_value)
{

    float current_angle = static_cast<float>(raw_value) * 360.0f / static_cast<float>(range_);

    if (!is_init_)
    {
        offset_      = raw_value; // 记录初始 raw 值 (可选，仅作记录)
        start_angle_ = current_angle; // 记录初始角度，作为 0 点基准
        last_angle_  = current_angle;
        
        round_cnt_        = 0;
        precision_offset_ = 0.0f;
        
        total_angle_ = 0.0f;
        angle_       = normalize_deg_0_360(current_angle - start_angle_); // 显示角归零

        // 若重定位在首帧前发生，则在初始化完成当拍应用，避免被首帧清零
        if (has_pending_relocate_)
        {
            precision_offset_ = pending_relocate_total_angle_;
            total_angle_ = pending_relocate_total_angle_;
            angle_ = normalize_deg_0_360(total_angle_);
            has_pending_relocate_ = false;
        }

        is_init_     = true;
        return;
    }
    float delta = current_angle - last_angle_;

    // 跨越 0/360 边界的逻辑判断
    // 如果两帧之间跳变超过 180 度，认为发生了过圈
    if (delta > 180.0f)
    {
        round_cnt_--;
    }
    else if (delta < -180.0f)
    {
        round_cnt_++;
    }

    // 更新历史
    last_angle_ = current_angle;

    float abs_total_angle = round_cnt_ * 360.0f + current_angle;

    //    Total = (AbsTotal - Start) + PrecisionOffset
    total_angle_ = (abs_total_angle - start_angle_) + precision_offset_;

    angle_ = normalize_deg_0_360(total_angle_);

    const int32_t RESET_THRESHOLD = 5000;
    if (abs(round_cnt_) > RESET_THRESHOLD)
    {
        precision_offset_ += round_cnt_ * 360.0f;
        round_cnt_ = 0;

    }
}

void Encoder::relocate_totalAngle(float now_totalAngle)
{
    if (!is_init_)
    {
        // 尚未收到首帧反馈时，先缓存目标，待初始化后立即应用
        has_pending_relocate_ = true;
        pending_relocate_total_angle_ = now_totalAngle;
        total_angle_ = now_totalAngle;
        angle_ = normalize_deg_0_360(total_angle_);
        return;
    }

    
    // 当前计算出的部分 (不含 Offset)
    float current_calc = (round_cnt_ * 360.0f + last_angle_ - start_angle_);
    
    // Reverse solve for Off:
    // Target = CurrentCalc + Off_New
    // Off_New = Target - CurrentCalc
    precision_offset_ = now_totalAngle - current_calc;

    // Update result immediately
    total_angle_ = now_totalAngle;
    angle_       = normalize_deg_0_360(total_angle_);
}