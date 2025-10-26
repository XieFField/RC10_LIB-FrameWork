/**
 * @file   Module_Air_joy.h
 * @brief  遥控器PPM解码（单例）
 */

#ifndef MODULE_AIR_JOY_H
#define MODULE_AIR_JOY_H

#pragma once
#include <stddef.h>
#include <limits.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "string.h"
#include "BSP_TimeStamp.h"

#define DEBUG_AIR_JOY 1

class AirJoy;

#if DEBUG_AIR_JOY
extern AirJoy *air_joy_debug; // 供调试使用的全局指针
#endif

#ifdef __cplusplus
class AirJoy
{
public:
    // 单例访问
    static AirJoy& getInstance();

    // 中断中调用，更新PPM
    void data_update(uint16_t GPIO_Pin, uint16_t GPIO_EXTI_USED_PIN);

    // 保持兼容：通道数据仍可直接访问（也可改用 getter）
    volatile uint16_t SWA=0,SWB=0,SWC=0,SWD=0;
    volatile uint16_t LEFT_X=0,LEFT_Y=0,RIGHT_X=0,RIGHT_Y=0;

private:
    // 构造/析构私有化，禁止拷贝
    AirJoy() = default;
    ~AirJoy() = default;
    AirJoy(const AirJoy&) = delete;
    AirJoy& operator=(const AirJoy&) = delete;

    // 常量定义
    static constexpr uint16_t FRAME_END_MIN = 2100;    // 帧结束最小时间
    static constexpr uint16_t PWM_MIN = 950;           // PWM最小脉宽
    static constexpr uint16_t PWM_MAX = 2050;          // PWM最大脉宽
    static constexpr uint8_t  MAX_CHANNELS = 8;        // 最大通道数
    static constexpr uint16_t FILTER_THRESHOLD_PERCENT = 15; // 滤波阈值百分比

    volatile uint32_t last_ppm_time=0, now_ppm_time=0;
    uint8_t ppm_ready=0, ppm_sample_cnt=0;
    uint8_t ppm_update_flag=0;
    volatile uint16_t ppm_time_delta=0;   // 脉宽(us)
    uint16_t PPM_buf[10]={0};
    static uint16_t last_valid[8];
};
#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
#ifdef __cplusplus
}
#endif

#endif // MODULE_AIR_JOY_H