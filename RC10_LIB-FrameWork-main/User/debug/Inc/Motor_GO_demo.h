/**
 * @file		Motor_GO_demo.h
 * @brief       宇树GO-M8010-6电机驱动测试
 * @author      ZhangJiaJia (Zhang643328686@163.com)
 * @date        2025-09 (创建日期)
 * @date        2025-10-14 (最后修改日期)
 * @platform	学院STM32H723ZGT6核心板
 * @version     0.1.0
 * @details     暂无
 * @todo        暂无
 * @note        暂无
 * @warning		暂无
 * @license     WTFPL License
 *
 * @par 版本修订历史
 * @{
 *  @li 版本号: 0.1.0
 *      - 修订日期: 2025-10-14
 *      - 主要变更:
 *			- 完成GO电机驱动测试
 *      - 作者: ZhangJiaJia
 */


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
    GO_MotorDemo() : RtosTask("GO_MotorDemo", 10), debug_uart(&huart1) {}
    void init();
    void loop() override;
    Debug_Printf debug_uart;
};



#endif // __MOTOR_GO_DEMO_H__
