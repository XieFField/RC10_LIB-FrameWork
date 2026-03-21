#ifndef APP_UTILS_H_
#define APP_UTILS_H_

#include <stdint.h>

#include <algorithm>

namespace jia
{
    using u8 = uint8_t;
    using u16 = uint16_t;
    using u32 = uint32_t;
    using u64 = uint64_t;

    using i8 = int8_t;
    using i16 = int16_t;
    using i32 = int32_t;
    using i64 = int64_t;

    using f32 = float;
    using f64 = double;

    constexpr f32 kPi = 3.14159265358979323846f;

    // 计算三角函数（角度制）
    f32 sinDegF32(f32 deg);
    f32 cosDegF32(f32 deg);

    /**
     * @brief 基于时间的一维信号速率限幅函数
     * @param target       目标值
     * @param current      当前值
     * @param dt           时间步长（秒）
     * @param maxRate      最大速率（单位/秒）
     * @return             限幅后的下一时刻值
     */
    f32 limit1DSignalRateByTimeF32(f32 target, f32 current, f32 dt, f32 max_rate);

    /**
     * @brief 基于时间的一维信号速率限幅函数（分离方向）
     * @param target       目标值
     * @param current      当前值
     * @param dt           时间步长（秒）
     * @param max_pos_rate 最大正速率（单位/秒）
     * @param max_neg_rate 最大负速率（单位/秒）
     * @return             限幅后的下一时刻值
     */
    f32 limit1DSignalRateByTimeSeparateIncAndDecF32(f32 target, f32 current, f32 dt, f32 max_pos_rate, f32 max_neg_rate);

    // 三值取小
    template <typename T>
    constexpr T minOfThree(const T &a, const T &b, const T &c);

    // 数值范围限制
    template <typename T>
    constexpr T clampValue(const T &val, const T &min_val, const T &max_val);

    // 转速和角速度转换
    constexpr f32 rpmToRadsF32(f32 rpm);
    constexpr f32 radsToRpmF32(f32 omega);

    // 角速度和线速度转换
    constexpr f32 omegaToVelF32(f32 omega, f32 radius);
    constexpr f32 velToOmegaF32(f32 vel, f32 radius);

    /**
     * @brief 生成正弦波信号（float类型输出）
     * @param t 输入时间（单位：秒，float类型）
     * @param amplitude 振幅（默认1.0f，输出范围[-amplitude, amplitude]）
     * @param frequency 频率（默认1.0Hz，每秒振荡次数）
     * @param phase 相位偏移（默认0.0f，单位：弧度）
     * @return float 正弦波当前时刻的幅值
     */
    f32 sineWaveGeneratorF32(f32 time, f32 amplitude = 1.0f, f32 frequency = 1.0f, f32 phase = 0.0f);

    inline f32 sinDegF32(f32 deg)
    {
        f32 sinf_result = sinf(deg * (kPi / 180.0f));

        // if (sinf_result > 1.0f)
        // {
        //     sinf_result = 1.0f;
        // }
        // else if (sinf_result < -1.0f)
        // {
        //     sinf_result = -1.0f;
        // }

        return sinf_result;
    }

    inline f32 cosDegF32(f32 deg)
    {
        f32 cosf_result = cosf(deg * (kPi / 180.0f));

        // if (cosf_result > 1.0f)
        // {
        //     cosf_result = 1.0f;
        // }
        // else if (cosf_result < -1.0f)
        // {
        //     cosf_result = -1.0f;
        // }

        return cosf_result;
    }

    inline f32 limit1DSignalRateByTimeF32(f32 target, f32 current, f32 dt, f32 max_rate)
    {
        f32 diff = target - current;
        f32 maxStep = max_rate * dt;
        if (diff > maxStep)
            return current + maxStep;
        else if (diff < -maxStep)
            return current - maxStep;
        else
            return target;
    }

    inline f32 limit1DSignalRateByTimeSeparateIncAndDecF32(f32 target, f32 current, f32 dt, f32 max_pos_rate, f32 max_neg_rate)
    {
        f32 diff = target - current;
        f32 max_pos_step = max_pos_rate * dt;
        f32 max_neg_step = max_neg_rate * dt;
        if (diff > max_pos_step)
            return current + max_pos_step;
        else if (diff < -max_neg_step)
            return current - max_neg_step;
        else
            return target;
    }

    /**
     * @brief 基于时间的一维信号速率限幅函数（分离方向）
     * @param target       目标值
     * @param current      当前值
     * @param dt           时间步长（秒）
     * @param max_retreat_rate 最大退避速率（单位/秒）
     * @param max_approach_rate 最大接近速率（单位/秒）
     * @return             限幅后的下一时刻值
     */
    inline f32 limit1DSignalRateByTimeSeparateApproachAndRetreatF32(f32 target, f32 current, f32 dt, f32 max_retreat_rate, f32 max_approach_rate)
    {
        f32 diff = target - current;
        if (diff > 0.0f && current > 0.0f || diff < 0.0f && current < 0.0f || current == 0.0f)
        {
            f32 max_retreat_step = max_retreat_rate * dt;
            if (diff > max_retreat_step)
                return current + max_retreat_step;
            else if (diff < -max_retreat_step)
                return current - max_retreat_step;
            else
                return target;
        }
        else if (diff < 0.0f && current > 0.0f || diff > 0.0f && current < 0.0f)
        {
            f32 max_approach_step = max_approach_rate * dt;
            if (diff < -max_approach_step)
                return current - max_approach_step;
            else if (diff > max_approach_step)
                return current + max_approach_step;
            else
                return target;
        }
        else
        {
            return target;
        }
    }

    template <typename T>
    constexpr inline T minOfThree(const T &a, const T &b, const T &c)
    {
        return std::min(std::min(a, b), c);
    }

    template <typename T>
    constexpr inline T clampValue(const T &val, const T &min_val, const T &max_val)
    {
        if (val < min_val)
            return min_val;
        if (val > max_val)
            return max_val;
        return val;
    }

    constexpr inline f32 rpmToRadsF32(f32 rpm)
    {
        return rpm * (2.0f * kPi) / 60.0f;
    }

    constexpr inline f32 radsToRpmF32(f32 omega)
    {
        return omega * 60.0f / (2.0f * kPi);
    }

    constexpr inline f32 omegaToVelF32(f32 omega, f32 radius)
    {
        return omega * radius;
    }

    constexpr inline f32 velToOmegaF32(f32 vel, f32 radius)
    {
        return vel / radius;
    }

    inline f32 sineWaveGeneratorF32(f32 time, f32 amplitude, f32 frequency, f32 phase)
    {
        return amplitude * sinf(2.0f * kPi * frequency * time + phase);
    }

    /**
     * @brief 按比例缩放三个数值，确保其绝对值不超过各自的最大值限制
     * @param val1 第一个输入值（可正可负）
     * @param val2 第二个输入值（可正可负）
     * @param val3 第三个输入值（可正可负）
     * @param max1 第一个值的最大绝对值限制（必须为正数）
     * @param max2 第二个值的最大绝对值限制（必须为正数）
     * @param max3 第三个值的最大绝对值限制（必须为正数）
     * @param out1 输出：处理后的第一个值
     * @param out2 输出：处理后的第二个值
     * @param out3 输出：处理后的第三个值
     * @return f32 缩放比例：
     *         - 若所有值都符合限制，返回1.0f
     *         - 若最大值为负数，返回-1.0f
     *         - 若有一个或多个值超限制，返回缩放比例（确保所有值绝对值均≤各自的最大值）
     */
    inline f32 scaleThreeValuesToMaxF32(f32 val1, f32 val2, f32 val3,
                                        f32 max1, f32 max2, f32 max3,
                                        f32 &out1, f32 &out2, f32 &out3)
    {
        // 安全校验：最大值必须为非负数
        if (max1 < 0.0f || max2 < 0.0f || max3 < 0.0f)
        {
            return -1.0f;
        }

        // 特殊情况：若存在最大值为0，直接全部返回0（避免除以0）
        if (max1 == 0.0f && max2 == 0.0f && max3 == 0.0f)
        {
            out1 = 0.0f;
            out2 = 0.0f;
            out3 = 0.0f;
            return 0.0f;
        }

        // 计算每个值的"超限倍数"（当前值绝对值 / 对应最大值）
        // 倍数>1表示超限，倍数≤1表示合规
        f32 ratio1 = fabsf(val1) / max1;
        f32 ratio2 = fabsf(val2) / max2;
        f32 ratio3 = fabsf(val3) / max3;

        // 找到最大的超限倍数
        f32 maxRatio = ratio1;
        if (ratio2 > maxRatio)
            maxRatio = ratio2;
        if (ratio3 > maxRatio)
            maxRatio = ratio3;

        // 确定缩放比例：若最大倍数≤1，缩放比例为1（不缩放）；否则为1/maxRatio
        f32 scaleFactor = (maxRatio > 1.0f) ? (1.0f / maxRatio) : 1.0f;

        // 按比例缩放并保留符号
        out1 = val1 * scaleFactor;
        out2 = val2 * scaleFactor;
        out3 = val3 * scaleFactor;

        return scaleFactor;
    }

    constexpr inline f32 radToDegF32(f32 rad)
    {
        return rad * 360.0f / (2.0f * kPi);
    }

    constexpr inline f32 degToRadF32(f32 deg)
    {
        return deg * (2.0f * kPi) / 360.0f;
    }

} // namespace jia

#endif
