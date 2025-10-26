#include "Module_Air_joy.h"
#include <cstdlib>  // abs

#if DEBUG_AIR_JOY
AirJoy *air_joy_debug = nullptr; // 供调试使用的全局
#endif

// 静态成员定义
uint16_t AirJoy::last_valid[8] = {1500,1500,1500,1500,1500,1500,1500,1500};

// 全局实例删除，改为单例
// AirJoy air_joy;
// volatile int cnt_ = 0;

// 单例实现
AirJoy& AirJoy::getInstance()
{
    static AirJoy instance;
    return instance;
}

void AirJoy::data_update(uint16_t GPIO_Pin, uint16_t GPIO_EXTI_USED_PIN)
{
    air_joy_debug = this; // 供调试使用的全局指针
    // 仅处理绑定的 PPM EXTI 引脚，忽略其他中断源
    if(GPIO_Pin != GPIO_EXTI_USED_PIN) return;

    last_ppm_time = now_ppm_time;
    now_ppm_time = TimeStamp::getInstance().getMicroseconds();
    ppm_time_delta = now_ppm_time - last_ppm_time;

    // 开始解包PPM信号
    if(ppm_ready == 1)
    {
        if(ppm_time_delta >= FRAME_END_MIN)  // 帧头
        {
            ppm_ready = 1;
            ppm_sample_cnt = 0;
            ppm_update_flag = 1;
        }
        else if(ppm_time_delta >= PWM_MIN && ppm_time_delta <= PWM_MAX)
        {
            PPM_buf[ppm_sample_cnt] = ppm_time_delta;
            last_valid[ppm_sample_cnt] = ppm_time_delta;
            ppm_sample_cnt++;

            if(ppm_sample_cnt >= MAX_CHANNELS)
            {
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
            ppm_ready = 0;
            ppm_sample_cnt = 0;
        }
    }
    else if(ppm_time_delta >= FRAME_END_MIN)
    {
        ppm_ready = 1;
        ppm_sample_cnt = 0;
        ppm_update_flag = 0;
    }
}

/**
 * @brief GPIO 的外部中断回调函数
 */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 这里替换为单例调用，指定使用的引脚（示例：GPIO_PIN_8）
    AirJoy::getInstance().data_update(GPIO_Pin, GPIO_PIN_8);
}