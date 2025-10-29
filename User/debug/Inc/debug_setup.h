/**
 * @brief 绕开状态机的单独demo测试
 */

#ifndef DEBUG_SETUP_H
#define DEBUG_SETUP_H

/**
 * @brief 一些相关的debug启动宏定义可以放这里
 */

#define DEMO_DEBUG_TEST 0

#if DEMO_DEBUG_TEST
    #define ARM_DEMO_DEBUG 0
    #define DEBUG_M2006 0
    #define SPEEDPLANNER_DEMO_DEBUG 0
#endif

#endif // DEBUG_SETUP_H
