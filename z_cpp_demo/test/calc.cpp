#include <iostream>
#include "math.h"
float calc_legal_rotate_target(float current_0_360, float target_0_360);
int main()
{
    float target = 360.0f;
    float current = 180.0f;
    float result = calc_legal_rotate_target(current, target);
    std::cout << "Legal rotate target: " << result << std::endl;
}
float prev_rotate_target_ = 0.0f;
float prev_norm_target_ = 0.0f;
typedef enum {
    ROTATE_PATH_SHORTEST,   // 最短路径
    ROTATE_PATH_POSITIVE,   // 正向路径
    ROTATE_PATH_NEGATIVE    // 负向路径
}Rotate_Strategy_E;

Rotate_Strategy_E rotate_strategy_;

float calc_legal_rotate_target(float current_0_360, float target_0_360)
{
    float re = 270.0f;
    if (re < 250.0f || re > 270.0f)
        re = 265.0f;

    target_0_360 = fmodf(target_0_360, 360.0f);
    if (target_0_360 < 0.0f) target_0_360 += 360.0f;
    current_0_360 = fmodf(current_0_360, 360.0f);
    if (current_0_360 < 0.0f) current_0_360 += 360.0f;

    if (target_0_360 > 180.0f && target_0_360 < re)
    {
        float dist_to_180 = target_0_360 - 180.0f;
        float dist_to_re = re - target_0_360;
        target_0_360 = (dist_to_180 < dist_to_re) ? 180.0f : re;
    }

    bool target_changed = (fabsf(target_0_360 - prev_norm_target_) > 0.01f);
    prev_norm_target_ = target_0_360;

    float diff = target_0_360 - current_0_360;

    float shortest_diff = diff;
    if (shortest_diff > 180.0f)
        shortest_diff -= 360.0f;
    else if (shortest_diff <= -180.0f)
        shortest_diff += 360.0f;

    if (target_changed || std::fabs(shortest_diff) < 10.0f)
    {
        if (std::fabs(shortest_diff) < 10.0f)
        {
            rotate_strategy_ = ROTATE_PATH_SHORTEST;
        }
        else
        {
            bool crosses = false;
            if (shortest_diff > 0.0f)
            {
                if (current_0_360 <= 180.0f && (current_0_360 + shortest_diff) >= re)
                    crosses = true;
            }
            else if (shortest_diff < 0.0f)
            {
                if (current_0_360 >= re && (current_0_360 + shortest_diff) <= 180.0f)
                    crosses = true;
            }

            if (crosses)
                rotate_strategy_ = (shortest_diff > 0.0f) ? ROTATE_PATH_NEGATIVE : ROTATE_PATH_POSITIVE;
            else
                rotate_strategy_ = ROTATE_PATH_SHORTEST;
        }
    }

    if (diff > 180.0f)
        diff -= 360.0f;
    else if (diff <= -180.0f)
        diff += 360.0f;

    if (std::fabs(diff) >= 10.0f)
    {
        switch (rotate_strategy_)
        {
            case ROTATE_PATH_POSITIVE:
                if (diff < 0.0f) diff += 360.0f;
                break;
            case ROTATE_PATH_NEGATIVE:
                if (diff > 0.0f) diff -= 360.0f;
                break;
            default:
                break;
        }
    }

    float result = current_0_360 + diff;

    if (!target_changed)
    {
        float gap = result - prev_rotate_target_;
        float k = roundf(gap / 360.0f);
        result = result - k * 360.0f;
    }
    prev_rotate_target_ = result;

    result = fmodf(result, 360.0f);
    if (result < 0.0f) result += 360.0f;
    return result;
}