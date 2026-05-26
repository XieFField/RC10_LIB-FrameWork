/**
 * @file BSP_RtosTimeStampUs64.cpp
 * @author 桑叁年
 * @brief RTOS 微秒时间戳实现
 */

#include "BSP_RtosTimeStampUs64.h"

#if defined(STM32H723xx) && defined(USE_HAL_DRIVER)
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx.h"
#define JIA_HAS_FREERTOS_SYSTICK_TIMESTAMP 1
#else
#define JIA_HAS_FREERTOS_SYSTICK_TIMESTAMP 0
#endif

namespace {

std::uint64_t ticksToUs64(std::uint32_t ticks, std::uint32_t tick_rate_hz) {
    if (tick_rate_hz == 0U) {
        return 0ULL;
    }
    return (static_cast<std::uint64_t>(ticks) * 1000000ULL) / static_cast<std::uint64_t>(tick_rate_hz);
}

std::uint64_t composeTimeUs64(std::uint32_t ticks,
                              std::uint32_t tick_rate_hz,
                              std::uint32_t sub_tick_us) {
    const std::uint64_t tick_period_us = (tick_rate_hz == 0U) ? 0ULL : (1000000ULL / tick_rate_hz);
    if (tick_period_us != 0ULL && sub_tick_us > tick_period_us) {
        sub_tick_us = static_cast<std::uint32_t>(tick_period_us);
    }
    return ticksToUs64(ticks, tick_rate_hz) + static_cast<std::uint64_t>(sub_tick_us);
}

}  // namespace

namespace jia {

std::uint64_t RtosTimeStampUs64::getTimeUs() {
#if JIA_HAS_FREERTOS_SYSTICK_TIMESTAMP
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return 0ULL;
    }

    taskENTER_CRITICAL();
    std::uint32_t ticks = xTaskGetTickCount();
    std::uint32_t systick_value = SysTick->VAL & SysTick_VAL_CURRENT_Msk;
    const std::uint32_t systick_load = SysTick->LOAD & SysTick_LOAD_RELOAD_Msk;
    const bool pending_tick = (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0U;
    if (pending_tick) {
        ++ticks;
        systick_value = SysTick->VAL & SysTick_VAL_CURRENT_Msk;
    }
    taskEXIT_CRITICAL();

    std::uint32_t sub_tick_us = 0U;
    const std::uint32_t tick_cycles = systick_load + 1U;
    if (tick_cycles != 0U) {
        const std::uint32_t elapsed_cycles = tick_cycles - systick_value;
        sub_tick_us = static_cast<std::uint32_t>(
            (static_cast<std::uint64_t>(elapsed_cycles) * 1000000ULL) /
            (static_cast<std::uint64_t>(tick_cycles) * configTICK_RATE_HZ));
    }

    return composeTimeUs64(ticks, static_cast<std::uint32_t>(configTICK_RATE_HZ), sub_tick_us);
#else
    return 0ULL;
#endif
}
}  // namespace jia
