#include "BSP_TimeUs64.h"

#if defined(STM32H723xx) && defined(USE_HAL_DRIVER)
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx.h"
#define JIA_HAS_FREERTOS_SYSTICK_TIMESTAMP 1
#else
#define JIA_HAS_FREERTOS_SYSTICK_TIMESTAMP 0
#endif

namespace jia
{
std::uint64_t TimeStampUs64::TicksToUs64(std::uint32_t ticks, std::uint32_t tick_rate_hz)
{
    if (tick_rate_hz == 0U)
    {
        return 0ULL;
    }
    return (static_cast<std::uint64_t>(ticks) * 1000000ULL) / static_cast<std::uint64_t>(tick_rate_hz);
}

std::uint64_t TimeStampUs64::ComposeTimeUs64(std::uint32_t ticks,
                                             std::uint32_t tick_rate_hz,
                                             std::uint32_t sub_tick_us)
{
    const std::uint64_t tick_period_us = (tick_rate_hz == 0U) ? 0ULL : (1000000ULL / tick_rate_hz);
    if (tick_period_us != 0ULL && sub_tick_us > tick_period_us)
    {
        sub_tick_us = static_cast<std::uint32_t>(tick_period_us);
    }
    return TicksToUs64(ticks, tick_rate_hz) + static_cast<std::uint64_t>(sub_tick_us);
}

/**
 * @brief 获取当前系统时间（微秒级）
 * 
 * 基于 FreeRTOS SysTick 定时器实现的高精度微秒时间戳。
 * 通过读取当前 tick 计数和 SysTick 计数器的当前值，计算出微秒级时间戳。
 * 
 * 精度说明：
 * - 基础精度由 configTICK_RATE_HZ 决定（通常为 1kHz，即 1ms 精度）
 * - 亚 tick 精度通过读取 SysTick->VAL 实现，可达 CPU 周期级精度
 * 
 * @return 当前时间（微秒），64 位无符号整数
 */
std::uint64_t TimeStampUs64::GetTimeUs()
{
#if JIA_HAS_FREERTOS_SYSTICK_TIMESTAMP
    // 调度器未启动时返回 0
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return 0ULL;
    }

    // 进入临界区，防止 tick 在读取过程中被更新
    taskENTER_CRITICAL();
    
    // 获取当前 FreeRTOS tick 计数
    std::uint32_t ticks = xTaskGetTickCount();
    
    // 读取 SysTick 当前计数值（向下计数）
    std::uint32_t systick_value = SysTick->VAL & SysTick_VAL_CURRENT_Msk;
    
    // 获取 SysTick 重载值（计数器最大值）
    const std::uint32_t systick_load = SysTick->LOAD & SysTick_LOAD_RELOAD_Msk;
    
    // 检查是否有未处理的 tick（COUNTFLAG 位表示计数器已归零）
    const bool pending_tick = (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0U;
    if (pending_tick)
    {
        // 如果有挂起的 tick，tick 计数需要 +1
        // 并重新读取 SysTick->VAL，因为可能已重新开始计数
        ++ticks;
        systick_value = SysTick->VAL & SysTick_VAL_CURRENT_Msk;
    }
    
    // 退出临界区
    taskEXIT_CRITICAL();

    // 计算亚 tick 时间（微秒）
    std::uint32_t sub_tick_us = 0U;
    const std::uint32_t tick_cycles = systick_load + 1U;  // 一个 tick 的总周期数
    if (tick_cycles != 0U)
    {
        // 计算当前 tick 内已过去的周期数
        // SysTick 向下计数，所以已过周期 = 总周期 - 当前值
        const std::uint32_t elapsed_cycles = tick_cycles - systick_value;
        
        // 换算为微秒：(已过周期数 * 1000000) / (总周期数 * tick频率)
        sub_tick_us = static_cast<std::uint32_t>(
            (static_cast<std::uint64_t>(elapsed_cycles) * 1000000ULL) /
            (static_cast<std::uint64_t>(tick_cycles) * configTICK_RATE_HZ)
        );
    }

    // 组合 tick 和亚 tick 时间，返回完整的微秒时间戳
    return ComposeTimeUs64(ticks, static_cast<std::uint32_t>(configTICK_RATE_HZ), sub_tick_us);
#else
    // 不支持 FreeRTOS 时返回 0
    return 0ULL;
#endif
}
} // namespace jia
