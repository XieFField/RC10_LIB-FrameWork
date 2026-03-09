#include "Module_Encoder.h"

void Encoder::update(uint16_t raw_value)
{
    // 1. 计算当前的绝对单圈角度 (0 ~ 360)
    //    这里使用 float 计算，对于 8192 分辨率精度足够
    float current_angle = static_cast<float>(raw_value) * 360.0f / static_cast<float>(range_);

    if (!is_init_)
    {
        offset_      = raw_value; // 记录初始 raw 值 (可选，仅作记录)
        start_angle_ = current_angle; // 记录初始角度，作为 0 点基准
        last_angle_  = current_angle;
        
        round_cnt_        = 0;
        // precision_offset_ = 0.0f; // [Fix] 不要清零可能早已设置的 relocate 偏移量
        
        // 初始化时，Total = (0 + Current - Start) + Offset = Offset
        // total_angle_ = 0.0f;
        total_angle_ = precision_offset_;

        angle_       = normalize_deg_0_360(total_angle_); // 显示角必须跟随 total
        is_init_     = true;
        return;
    }

    // 2. 也是核心：通过前后两帧角度差，判断是否 "过零/跨圈"
    float delta = current_angle - last_angle_;

    // 跨越 0/360 边界的逻辑判断
    // 如果两帧之间跳变超过 180 度，认为发生了过圈
    if (delta > 180.0f)
    {
        // 例如：上一帧 5 度，这一帧 355 度 -> delta = +350 -> 说明是反转跨过了 0 点
        round_cnt_--;
    }
    else if (delta < -180.0f)
    {
        // 例如：上一帧 355 度，这一帧 5 度 -> delta = -350 -> 说明是正转跨过了 360 点
        round_cnt_++;
    }

    // 更新历史
    last_angle_ = current_angle;

    // 3. 计算 "未偏置" 的物理总角度 AbsTotal = Turns * 360 + Current
    //    相比积分法(total += delta)，这种算法没有累积误差，且 round_cnt_ 是整数，非常稳健
    float abs_total_angle = round_cnt_ * 360.0f + current_angle;

    // 4. 计算最终输出的总角度 (减去初始角度，加上精度偏置)
    //    Total = (AbsTotal - Start) + PrecisionOffset
    total_angle_ = (abs_total_angle - start_angle_) + precision_offset_;

    // 5. 计算单圈输出角度 (总是映射到 0~360)
    //    这里我们直接用 total_angle_ 取模，保证和 total_angle_ 是对应的
    angle_ = normalize_deg_0_360(total_angle_);


    // [精度保护/防溢出逻辑]
    // 类似于你同事的 "rotor_pos > 5000" 逻辑
    // 如果总圈数太大，float 精度会下降 (23位尾数，在 10000 圈时分辨率只有 ~0.4 度)
    // 我们在圈数超过一定阈值 (例如 5000 圈) 时，进行一次 "坐标系重置"
    // 把当前的圈数 "吸收" 到 precision_offset_ 中，并将 round_cnt_ 归零
    const int32_t RESET_THRESHOLD = 5000;
    if (abs(round_cnt_) > RESET_THRESHOLD)
    {
        // 将当前的圈数部分转移到 precision_offset_
        precision_offset_ += round_cnt_ * 360.0f;
        // 归零圈数，防止 float 失真
        round_cnt_ = 0;
        // 注意：无需修改 last_angle_ 或 start_angle_，因为公式仍然平衡
        // Test: 
        // Before: Total = (5001*360 + Curr - Start) + Off
        // After:  Off' = Off + 5001*360; Round' = 0
        // NewTotal = (0*360 + Curr - Start) + Off' 
        //          = (Curr - Start) + Off + 5001*360 -> 等价 -> 正确
    }
}

void Encoder::relocate_totalAngle(float now_totalAngle)
{
    // 重定位核心：改变 "坐标原点" 使得计算出的 total_angle_ 等于目标值
    // Formula: Total = (Round*360 + Curr - Start) + Off
    // 我们保持 Round, Curr, Start 不变 (因为这些是物理事实)，只调整 Off
    
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