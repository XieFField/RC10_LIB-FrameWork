/**
 *******************************************************************************
 * @file    system_detect_task.h
 * @author  ZhangJiaJia
 * @date    2026-02-02
 * @version V1.1
 * @brief   系统检测任务头文件
 *******************************************************************************
 */

#ifndef SYSTEM_DETECT_TASK_H_
#define SYSTEM_DETECT_TASK_H_

#include "cmsis_os2.h"

extern osThreadId_t system_detect_task_handle;
extern const osThreadAttr_t system_detect_task_attributes;

void startSystemDetectTask(void *argument);

#endif // SYSTEM_DETECT_TASK_H_
