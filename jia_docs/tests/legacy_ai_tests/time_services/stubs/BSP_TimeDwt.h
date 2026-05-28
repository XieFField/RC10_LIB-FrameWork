#ifndef TEST_BSP_TIMEDWT_H
#define TEST_BSP_TIMEDWT_H

#include <cstdint>

namespace jia
{
class TimeDwt
{
public:
    static void Init(std::uint32_t) {}
    static std::uint32_t GetCycle32() { return 0U; }
    static std::uint32_t GetElapsedCycles32(std::uint32_t start_cycle32, std::uint32_t end_cycle32)
    {
        return end_cycle32 - start_cycle32;
    }
    static std::uint32_t GetElapsedCycles32(std::uint32_t start_cycle32)
    {
        return GetElapsedCycles32(start_cycle32, GetCycle32());
    }
    static std::uint32_t CyclesToUs32(std::uint32_t cycles) { return cycles; }
    static std::uint32_t GetElapsedUs32(std::uint32_t start_cycle32, std::uint32_t end_cycle32)
    {
        return GetElapsedCycles32(start_cycle32, end_cycle32);
    }
    static std::uint32_t GetElapsedUs32(std::uint32_t start_cycle32)
    {
        return GetElapsedUs32(start_cycle32, GetCycle32());
    }
};
} // namespace jia

#endif
