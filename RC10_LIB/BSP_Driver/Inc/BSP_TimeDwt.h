/**
 * @file BSP_TimeDwt.h
 * @brief DWT CYCCNT 高精度时间功能模块接口
 */

#ifndef __BSP_TIMEDWT_H
#define __BSP_TIMEDWT_H

#include <cstdint>

namespace jia
{
class TimeDwt
{
public:
    static void Init(std::uint32_t core_clock_hz);
    static std::uint32_t GetCycle32();
    static std::uint32_t GetElapsedCycles32(std::uint32_t start_cycle32, std::uint32_t end_cycle32);
    static std::uint32_t GetElapsedCycles32(std::uint32_t start_cycle32);
    static std::uint32_t CyclesToUs32(std::uint32_t cycles);
    static std::uint32_t GetElapsedUs32(std::uint32_t start_cycle32, std::uint32_t end_cycle32);
    static std::uint32_t GetElapsedUs32(std::uint32_t start_cycle32);

private:
    static std::uint32_t core_clock_hz_;
    static std::uint32_t cycles_per_us_;
};
} // namespace jia

#endif // __BSP_TIMEDWT_H
