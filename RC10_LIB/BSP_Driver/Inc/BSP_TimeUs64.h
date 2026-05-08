/**
 * @file BSP_TimeUs64.h
 * @brief 基于 RTOS tick 的 64 位微秒时间戳接口
 */

#ifndef __BSP_TIMEUS64_H
#define __BSP_TIMEUS64_H

#include <cstdint>

namespace jia
{
class TimeStampUs64
{
public:
    static std::uint64_t GetTimeUs();
    static std::uint64_t TicksToUs64(std::uint32_t ticks, std::uint32_t tick_rate_hz);
    static std::uint64_t ComposeTimeUs64(std::uint32_t ticks, std::uint32_t tick_rate_hz, std::uint32_t sub_tick_us);

private:
    TimeStampUs64() = delete;
};
} // namespace jia

#endif // __BSP_TIMEUS64_H
