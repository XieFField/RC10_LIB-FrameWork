/**
 * @brief �ƿ�״̬���ĵ���demo����
 */

#ifndef DEBUG_SETUP_H
#define DEBUG_SETUP_H

/**
 * @brief һЩ��ص�debug�����궨����Է�����
 */

#define DEMO_DEBUG_TEST 1

#if DEMO_DEBUG_TEST
    #define ARM_DEMO_DEBUG 0
    #define DEBUG_M2006 0
    #define SPEEDPLANNER_DEMO_DEBUG 1
    #define DEBUG_DJI_Motor 1
#endif

#endif // DEBUG_SETUP_H
