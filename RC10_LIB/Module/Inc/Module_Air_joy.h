/**
 * @file   Module_Air_joy.h
 * @author Zhan HongLi
 * @brief  閬ユ帶鍣≒PM瑙ｇ爜
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

#ifdef __cplusplus
class AirJoy
{
public:
    void data_update(uint16_t GPIO_Pin, uint16_t GPIO_EXTI_USED_PIN);
    volatile uint16_t SWA=0,SWB=0,SWC=0,SWD=0;
    volatile uint16_t LEFT_X=0,LEFT_Y=0,RIGHT_X=0,RIGHT_Y=0;
    
private:
    // 甯搁噺瀹氫箟
    static constexpr uint16_t FRAME_END_MIN = 2100;    // 甯х粨鏉熸渶灏忔椂闂�
    static constexpr uint16_t PWM_MIN = 950;           // PWM鏈€灏忚剦瀹�
    static constexpr uint16_t PWM_MAX = 2050;          // PWM鏈€澶ц剦瀹�
    static constexpr uint8_t MAX_CHANNELS = 8;         // 鏈€澶ч€氶亾鏁�
    static constexpr uint16_t FILTER_THRESHOLD_PERCENT = 15; // 婊ゆ尝闃堝€肩櫨鍒嗘瘮
    
    volatile uint32_t last_ppm_time=0, now_ppm_time=0;
    uint8_t ppm_ready=0, //涓€鑸�鍒濆�嬪寲鏃跺€欙紝PPM瑙ｇ爜鐘舵€佷负鏈�鍑嗗�囧ソ
                 ppm_sample_cnt=0;
    uint8_t ppm_update_flag=0;
    volatile uint16_t ppm_time_delta=0;   // 寰楀埌涓婂崌娌夸笌涓嬮檷娌跨殑鏃堕棿
    uint16_t PPM_buf[10]={0};   
    static uint16_t last_valid[8];
};




extern "C" {
#endif
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
#ifdef __cplusplus
}
extern AirJoy air_joy;
#endif

#endif // MODULE_AIR_JOY_H

//123123