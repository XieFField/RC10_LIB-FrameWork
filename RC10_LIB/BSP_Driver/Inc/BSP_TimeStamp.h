/**
 * @file BSP_TimeStamp.h
 * @author XieFField
 * @brief 时间服务，可以提供微秒级时间戳
 *        主要还是参考杨哥代码，不过C/C++耦合度较高，全局变量过多
 *        在此做出改善
 * @version 1.0
 * 
 * @version 2.0
 *       修正了64位溢出导致的随机无穷大bug
 *       优化了临界区的处理，改为原子操作
 */

/*
  _______               ___________                    
 /_  __(_)___ ___  ___ / ___/_  __/___ _____ ___  ____ 
  / / / / __ `__ \/ _ \\__ \ / / / __ `/ __ `__ \/ __ \
 / / / / / / / / /  __/__/ // / / /_/ / / / / / / /_/ /
/_/ /_/_/ /_/ /_/\___/____//_/  \__,_/_/ /_/ /_/ .___/ 
                                              /_/      
*/

#ifndef __BSP_TIMESTAMP_H
#define __BSP_TIMESTAMP_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32h7xx_hal.h"
#include "tim.h"

#ifdef __cplusplus
}
#endif // __cplusplus

#ifdef __cplusplus

#include <cstdint>


class TimeStamp{
public:
    static TimeStamp& getInstance(); //获取唯一实例

    /**
     * @brief 初始化时间戳服务
     * @param htim 指向已配置为基本计数器模式的定时 1us 的句柄
     */
    void init(TIM_HandleTypeDef* htim);

    /**
     * @brief 获取当前时间戳(自系统启动以来)，单位微秒
     */
    uint64_t getMicroseconds() const;

    /**
     * @brief 获取当前时间戳(自系统启动以来)，单位毫秒
     */
    uint64_t getMilliseconds() const;

    /**
     * @brief 获取当前时间戳(自系统启动以来)，单位秒
     */
    float getSeconds() const;

    static void overflowCallback(); // 定时器溢出回调函数
private:
    //保持单例
    TimeStamp();

    ~TimeStamp() = default;

    TimeStamp(const TimeStamp&) = delete;

    TimeStamp& operator = (const TimeStamp&) = delete;

    static TIM_HandleTypeDef* s_htim_; // 定时器句柄

    static volatile uint64_t s_overflow_count_; // 溢出计数
};

#endif // __cplusplus

#endif // __BSP_TIMESTAMP_H