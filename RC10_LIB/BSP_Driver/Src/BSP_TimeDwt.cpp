#include "BSP_TimeDwt.h"

#if defined(USE_HAL_DRIVER)
#if defined(STM32H723xx) || defined(STM32H7xx) || defined(__CORTEX_M)
#include "stm32h7xx.h"
#endif
#if defined(DWT) && defined(CoreDebug) && defined(CoreDebug_DEMCR_TRCENA_Msk) && defined(DWT_CTRL_CYCCNTENA_Msk)
#define JIA_HAS_DWT_CYCCNT 1
#else
#define JIA_HAS_DWT_CYCCNT 0
#endif
#else
#define JIA_HAS_DWT_CYCCNT 0
#endif

namespace jia
{
std::uint32_t TimeDwt::core_clock_hz_ = 0U;
std::uint32_t TimeDwt::cycles_per_us_ = 1U;

void TimeDwt::Init(std::uint32_t core_clock_hz)
{
    core_clock_hz_ = core_clock_hz;

    // 仅当主频能整除 1 MHz 时，才启用 DWT 的 us 换算。
    // 这样可保证 cycles_per_us_ 为整数，避免 us 换算失真。
    if (core_clock_hz_ % 1000000U == 0U)
    {
        cycles_per_us_ = core_clock_hz_ / 1000000U;

#if JIA_HAS_DWT_CYCCNT
        // 打开跟踪功能，否则 DWT 的周期计数器不会开始计数。
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        // 先清零计数器，避免带入上一次运行或调试残留值。
        DWT->CYCCNT = 0U;
        // 使能 CYCCNT，之后即可按 CPU 周期读取递增计数。
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
    }
    else
    {
        // 不能整除时保持降级路径，不启用 DWT 的 us 换算。
        // GetCycle32 / GetElapsedCycles32 的原始周期接口仍可用。
        (void)core_clock_hz_;
    }
}

std::uint32_t TimeDwt::GetCycle32()
{
#if JIA_HAS_DWT_CYCCNT
    return DWT->CYCCNT;
#else
    return 0U;
#endif
}

std::uint32_t TimeDwt::GetElapsedCycles32(std::uint32_t start_cycle32, std::uint32_t end_cycle32)
{
    return end_cycle32 - start_cycle32;
}

std::uint32_t TimeDwt::GetElapsedCycles32(std::uint32_t start_cycle32)
{
    return GetElapsedCycles32(start_cycle32, GetCycle32());
}

std::uint32_t TimeDwt::CyclesToUs32(std::uint32_t cycles)
{
    if (cycles_per_us_ == 0U)
    {
        return 0U;
    }
    return cycles / cycles_per_us_;
}

std::uint32_t TimeDwt::GetElapsedUs32(std::uint32_t start_cycle32, std::uint32_t end_cycle32)
{
    return CyclesToUs32(GetElapsedCycles32(start_cycle32, end_cycle32));
}

std::uint32_t TimeDwt::GetElapsedUs32(std::uint32_t start_cycle32)
{
    return GetElapsedUs32(start_cycle32, GetCycle32());
}
} // namespace jia
