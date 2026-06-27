/**
 * @file   Module_Air_joy.h
 * @author Zhan HongLi
 * @brief  遥控器 PPM 解码
 * @version 1.0
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
#include "APP_tool.h"


#ifdef __cplusplus
class AirJoy
{
public:
    // 单例访问（回调里直接调用 AirJoy::instance().data_update(...)）
    static AirJoy& getinstance();

    void data_update(uint16_t GPIO_Pin, uint16_t GPIO_EXTI_USED_PIN);

    // 通道数据
    volatile uint16_t SWA=0,SWB=0,SWC=0,SWD=0;
    volatile uint16_t LEFT_X=0,LEFT_Y=0,RIGHT_X=0,RIGHT_Y=0;
    
private:
    // 构造私有化，禁止拷贝
    AirJoy() = default;
    AirJoy(const AirJoy&) = delete;
    AirJoy& operator=(const AirJoy&) = delete;

    // 常量定义
    static constexpr uint16_t FRAME_END_MIN = 2100;    // 帧结束最小时长(us)
    static constexpr uint16_t PWM_MIN       = 950;     // PWM最小脉宽(us)
    static constexpr uint16_t PWM_MAX       = 2050;    // PWM最大脉宽(us)
    static constexpr uint8_t  MAX_CHANNELS  = 8;       // 最大通道数
    static constexpr uint16_t FILTER_THRESHOLD_PERCENT = 15; // 滤波阈值百分比（未使用示例）
    
    // 时序/解析状态
    volatile uint32_t last_ppm_time=0, now_ppm_time=0;
    uint8_t  ppm_ready=0,              // 一般初始化时，PPM解码状态未就绪
             ppm_sample_cnt=0;
    uint8_t  ppm_update_flag=0;
    volatile uint16_t ppm_time_delta=0;   // 上一次与本次沿间的时间差(us)
    uint16_t PPM_buf[10]={0};             // 临时缓冲
    static uint16_t last_valid[8];        // 上次有效通道值
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