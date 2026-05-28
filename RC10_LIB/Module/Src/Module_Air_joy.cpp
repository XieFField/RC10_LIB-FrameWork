//#include "Module_Air_joy.h"
//#include <cstdlib>  // 用于 abs 函数

//// 定义静态成员变量
//uint16_t AirJoy::last_valid[8] = {1500,1500,1500,1500,1500,1500,1500,1500};

//AirJoy* debug_airjoy_ = nullptr;

//// 可选调试计数
//volatile int cnt_ = 0;

//// 单例实现（函数内静态）
//// 建议在中断使能前，先在系统初始化处调用一次 AirJoy::instance();
//AirJoy& AirJoy::getinstance()
//{
//    static AirJoy s;
//    return s;
//}

//void AirJoy::data_update(uint16_t GPIO_Pin, uint16_t GPIO_EXTI_USED_PIN)
//{
//    // 仅处理绑定的 PPM EXTI 引脚
//    if(GPIO_Pin != GPIO_EXTI_USED_PIN) 
//        return;
//    
//    // 获取时间戳
//    last_ppm_time = now_ppm_time;
//    now_ppm_time  = TimeStamp::getInstance().getMicroseconds();
//    ppm_time_delta = now_ppm_time - last_ppm_time;

//    // 解码 PPM 帧
//    if(ppm_ready == 1)    // 已进入采样流程
//    {
//        // 帧头：低/高电平间隔大于最小时长判定为新帧
//        if(ppm_time_delta >= FRAME_END_MIN)
//        {
//            ppm_ready = 1;
//            ppm_sample_cnt = 0;
//            ppm_update_flag = 1;
//        } 
//        // 有效通道脉宽
//        else if(ppm_time_delta >= PWM_MIN && ppm_time_delta <= PWM_MAX)
//        {         
//            // 缓存，不做滤波
//            PPM_buf[ppm_sample_cnt]    = ppm_time_delta;
//            last_valid[ppm_sample_cnt] = ppm_time_delta;
//            ppm_sample_cnt++;
//            cnt_ = ppm_sample_cnt;
//            
//            // 通道采满，统一映射更新
//            if(ppm_sample_cnt >= MAX_CHANNELS)
//            {
//                // 通道映射：按你的接收机实际顺序调整
//                LEFT_X  = PPM_buf[0]; 
//                LEFT_Y  = PPM_buf[1]; 
//                RIGHT_X = PPM_buf[3]; 
//                RIGHT_Y = PPM_buf[2];
//                SWA     = PPM_buf[4]; 
//                SWB     = PPM_buf[5]; 
//                SWC     = PPM_buf[6]; 
//                SWD     = PPM_buf[7];
//                
//                if(_tool_Abs(SWD - 1500) < 50)
//                    SWD = 1000;
//                
//                ppm_ready = 0;
//                ppm_sample_cnt = 0;
//            }
//        }
//        else 
//        {
//            // 无效脉宽，重置等待帧头
//            ppm_ready = 0;
//            ppm_sample_cnt = 0;
//        }
//    }
//    else if(ppm_time_delta >= FRAME_END_MIN) // 帧尾/帧间隔足够，视作新帧开始
//    {
//        ppm_ready = 1;
//        ppm_sample_cnt = 0;
//        ppm_update_flag = 0;
//    }
//    debug_airjoy_ = this;
//}

///**
// * @brief GPIO 外部中断回调（保持旧入口不变）
// */
//extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    // 将 GPIO_PIN_8 改为你的实际 PPM EXTI 引脚
//    AirJoy::getinstance().data_update(GPIO_Pin, GPIO_PIN_8);
//}