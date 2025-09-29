#ifndef __MOTOR_GO_DEMO_H__
#define __MOTOR_GO_DEMO_H__

#pragma once    // 再次冗余保证不重复包含

#if defined(__cplusplus) && __cplusplus < 201103L
#error "此文件需要支持C++11及以上编译环境,请确保编译器支持C++11或更高版本。"
#elif !defined(__cplusplus)
#error "此文件需要支持C++编译环境,请确保编译器支持__cplusplus宏。"
#endif


#include "BSP_RTOS.h"
#include "APP_debugTool.h"
#include "frame_demo.h"

#include "Motor_GO.h"



class GO_MotorDemo: public RtosTask {
public:
    GO_MotorDemo() : RtosTask("GO_MotorDemo", 1), debug_uart(&huart1) {}
    void init();
    void loop() override;
    Debug_Printf debug_uart;
};



#endif // __MOTOR_GO_DEMO_H__
