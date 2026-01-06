#ifndef ONIM_DEMO_H
#define ONIM_DEMO_H


#include <cstdint>
#include <cstdlib> 
#include <cstring>
#include <cstdio> 
#include <cmath>
#include <string>
#include <type_traits>
#include <cstdarg>
#include <cstdio>

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "BSP_TimeStamp.h"
#include "omni_chassisSetup.h"
#include "Setup_ConfigInit.h"


extern OmniChassis_Setup ChassisOmni;

extern osThreadId_t SystemDetectTaskHandle;
extern const osThreadAttr_t SystemDetectTask_attributes;

void StartSystemDetectTask(void *argument);


# endif // __ONIM_DEMO_H