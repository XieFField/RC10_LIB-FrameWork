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
    constexpr T clampValue(const T &value, const T &min_val, const T &max_val);

    template <typename T>
    constexpr inline T minOfThree(const T &a, const T &b, const T &c)
    {
        return std::min(std::min(a, b), c);
    }

    template <typename T>
    constexpr inline T clampValue(const T &value, const T &min_val, const T &max_val)
    {
        if (value < min_val)
            return min_val;
        if (value > max_val)
            return max_val;
        return value;
    }

} // namespace jia

#endif
