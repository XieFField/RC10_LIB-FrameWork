#ifndef TEST_BSP_TIMEUS64_H
#define TEST_BSP_TIMEUS64_H

#include <cstdint>

namespace jia
{
class TimeStampUs64
{
public:
    static std::uint64_t GetTimeUs() { return 0ULL; }
    static std::uint64_t TicksToUs64(std::uint32_t ticks, std::uint32_t tick_rate_hz)
    {
        return tick_rate_hz == 0U ? 0ULL : (static_cast<std::uint64_t>(ticks) * 1000000ULL) / tick_rate_hz;
    }
    static std::uint64_t ComposeTimeUs64(std::uint32_t ticks, std::uint32_t tick_rate_hz, std::uint32_t sub_tick_us)
    {
        return TicksToUs64(ticks, tick_rate_hz) + sub_tick_us;
    }
};
} // namespace jia

#endif
