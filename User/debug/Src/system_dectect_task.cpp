/**
 *******************************************************************************
 * @file    system_detect_task.cpp
 * @author  ZhangJiaJia
 * @date    2026-02-02
 * @version V1.1
 * @brief   系统检测任务源文件
 *******************************************************************************
 */

#include "system_detect_task.h"

#include <cstdint>

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include "BSP_TimeStamp.h"
#include "omni_chassisSetup.h"
#include "Setup_ConfigInit.h"

extern OmniChassis_Setup ChassisOmni;

osThreadId_t system_detect_task_handle;
const osThreadAttr_t system_detect_task_attributes = {
    .name = "system_detect_task",
    .stack_size = 200 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

UBaseType_t system_detect_task_water_mark = 0;

uint64_t system_detect_task_time = 0;

void startSystemDetectTask(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        system_detect_task_water_mark = uxTaskGetStackHighWaterMark(NULL);
        system_detect_task_time = TimeStamp::getInstance().getMicroseconds();

        vTaskDelayUntil(&last_wake_time, 50);
    }
}