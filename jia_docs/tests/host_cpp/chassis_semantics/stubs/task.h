#ifndef TEST_TDD_TASK_H
#define TEST_TDD_TASK_H

#include "FreeRTOS.h"

inline TickType_t xTaskGetTickCount()
{
    return 0;
}

inline void vTaskDelayUntil(TickType_t *, TickType_t)
{
}

#endif
