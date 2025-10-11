#include "Module_Air_joy.h"
#include <cstdlib>  // 用于abs函数

// 定义静态成员变量
uint16_t AirJoy::last_valid[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

AirJoy air_joy;

volatile int cnt_ = 0;

void AirJoy::data_update(uint16_t GPIO_Pin, uint16_t GPIO_EXTI_USED_PIN)
{
    if(GPIO_Pin != GPIO_EXTI_USED_PIN) 
        return;
    
    // 获取时间戳
    last_ppm_time = now_ppm_time;
    now_ppm_time = TimeStamp::getInstance().getMicroseconds();
    ppm_time_delta = now_ppm_time - last_ppm_time;

    // 开始解包PPM信号
    if(ppm_ready == 1)    // 判断帧结束后，进行下一轮解析
    {
        // 帧结束电平至少2ms=2000us(留点余量)
        if(ppm_time_delta >= FRAME_END_MIN)  // 帧头
        {
            ppm_ready = 1;
            ppm_sample_cnt = 0;
            ppm_update_flag = 1;
        } 
        else if(ppm_time_delta >= PWM_MIN && ppm_time_delta <= PWM_MAX) // 单个PWM脉宽在1000-2000us
        {         
            // 直接存入缓冲区，不做任何滤波
            PPM_buf[ppm_sample_cnt] = ppm_time_delta;
            last_valid[ppm_sample_cnt] = ppm_time_delta;
            ppm_sample_cnt++;
            cnt_ = ppm_sample_cnt;
            
            // 边界检查
            if(ppm_sample_cnt >= MAX_CHANNELS)   
            {
                // 通道映射：根据实际遥控器调整
                LEFT_X  = PPM_buf[0]; 
                LEFT_Y  = PPM_buf[1]; 
                RIGHT_X = PPM_buf[3]; 
                RIGHT_Y = PPM_buf[2];
                SWA     = PPM_buf[4]; 
                SWB     = PPM_buf[5]; 
                SWC     = PPM_buf[6]; 
                SWD     = PPM_buf[7];
                
                ppm_ready = 0;
                ppm_sample_cnt = 0;
            }
        }
        else 
        {
            // 无效脉冲宽度，重置状态
            ppm_ready = 0;
            ppm_sample_cnt = 0;
        }
    }
    else if(ppm_time_delta >= FRAME_END_MIN) // 帧尾电平至少2ms=2000us
    {
        ppm_ready = 1;
        ppm_sample_cnt = 0;
        ppm_update_flag = 0;
    }
}

/**
 * @brief GPIO 的外部中断回调函数
 * 
 * @param GPIO_Pin 触发的IO口
 */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    air_joy.data_update(GPIO_Pin, GPIO_PIN_8);
}