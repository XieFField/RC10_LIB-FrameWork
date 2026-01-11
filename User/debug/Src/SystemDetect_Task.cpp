/**
  ******************************************************************************
  * @file    SystemDetect_Task.cpp
  * @author  ZhangJiaJia
  * @date    2026-01-12
  * @brief   系统检测任务源文件
  ******************************************************************************
  */


#include "SystemDetect_Task.h"


osThreadId_t SystemDetectTaskHandle;
const osThreadAttr_t SystemDetectTask_attributes = {
  .name = "SystemDetectTask",
  .stack_size = 200 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
UBaseType_t SystemDetectTaskWaterMark = 0;


uint64_t LastSystemDetectTaskTime = 0;
uint64_t SystemDetectTaskTime = 0;  


void StartSystemDetectTask(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
   
    while(1)
    {
		
        SystemDetectTaskWaterMark = uxTaskGetStackHighWaterMark(NULL);
        SystemDetectTaskTime = TimeStamp::getInstance().getMicroseconds();

        // ChassisOmni.debug_uart.printf_DMA("%llu,%llu\r\n", SystemDetectTaskTime - LastSystemDetectTaskTime, SystemDetectTaskTime);

        LastSystemDetectTaskTime = SystemDetectTaskTime;



        vTaskDelayUntil(&xLastWakeTime, 1);
    }
}