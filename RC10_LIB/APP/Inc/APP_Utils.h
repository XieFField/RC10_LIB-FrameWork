#ifndef APP_UTILS_H_
#define APP_UTILS_H_

#include <stdint.h>

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

    f32 sinDeg(f32 deg);
    f32 cosDeg(f32 deg);

    /**
     * @brief 基于时间的一维信号速率限幅函数
     * @param target       目标值
     * @param current      当前值
     * @param dt           时间步长（秒）
     * @param maxRate      最大速率（单位/秒）
     * @return             限幅后的下一时刻值
     */
    f32 limit1DSignalRateByTime(f32 target, f32 current, f32 dt, f32 maxRate);

} // namespace jia

#endif
