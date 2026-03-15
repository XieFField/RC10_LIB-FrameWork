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
    f32 limit1DSignalRateByTimeF32(f32 target, f32 current, f32 dt, f32 maxRate);

    // 三值取小
    template <typename T>
    constexpr T minOfThree(const T &a, const T &b, const T &c);

    // 数值范围限制
    template <typename T>
    constexpr T clampValue(const T &val, const T &min_val, const T &max_val);

    // 转速和角速度转换（单位：rad/s）
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

    inline f32 limit1DSignalRateByTimeF32(f32 target, f32 current, f32 dt, f32 maxRate)
    {
        f32 diff = target - current;
        f32 maxStep = maxRate * dt;
        if (diff > maxStep)
            return current + maxStep;
        else if (diff < -maxStep)
            return current - maxStep;
        else
            return target;
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

} // namespace jia

#endif
