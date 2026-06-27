#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H

#include <cstdint>

#define configTICK_RATE_HZ 1000U

using TickType_t = std::uint32_t;

static inline int xTaskGetSchedulerState()
{
    return 0;
}

static inline TickType_t xTaskGetTickCount()
{
    return 0U;
}

static inline void taskENTER_CRITICAL() {}
static inline void taskEXIT_CRITICAL() {}

#define taskSCHEDULER_NOT_STARTED 0

#endif
