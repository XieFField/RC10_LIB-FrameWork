/**
 * @file    APP_debugTool.h
 * @author  XieFField
 * @brief   用于临时调试用的 UART printf 封装
 *          之后其他debug工具也可以放进这里面     
 * @version 1.0
 */

#ifndef __APP_DEBUG_TOOL_H
#define __APP_DEBUG_TOOL_H

#pragma once


#ifdef __cplusplus
extern "C" {
    #include "stm32h7xx_hal.h"
    #include <stdarg.h> 
    #include <string.h>
    #include "usart.h"
}
#endif

#ifdef __cplusplus

#include <cstdint>
#include <cstdio>
#define SEND_BUF_SIZE 100
/**
 * @brief 串口打印调试助手
 */
class Debug_Printf {
public:
    Debug_Printf(UART_HandleTypeDef *huart) : huart_(huart) {}
    ~Debug_Printf() {}

    void printf_DMA(char *fmt, ...)
    {
        memset(Sendbuf, 0, SEND_BUF_SIZE);  // 清空发送缓冲区
        
        va_list arg;
        va_start(arg, fmt);
        vsnprintf((char*)Sendbuf, SEND_BUF_SIZE, fmt, arg);  // 安全的格式化输出，防止缓冲区溢出
        va_end(arg);
        
        uint8_t len = strlen((char*)Sendbuf);  // 计算实际字符串长度
        if(len > 0)
        {
            HAL_UART_Transmit_DMA(huart_, Sendbuf, len);  // 通过发送字符串
        }
    }

    void printf_UART(char *fmt, ...) 
    {  // 函数名修改以区分非DMA版本
        if (fmt == NULL) 
            return;  // 防止空指针崩溃
        
        va_list arg;
        va_start(arg, fmt);
        
        // 1. 填充缓冲区并获取实际需要的长度（不含终止符）
        int ret = vsnprintf((char*)Sendbuf, SEND_BUF_SIZE, fmt, arg);
        va_end(arg);
        
        // 2. 校验填充结果，过滤无效情况
        if (ret <= 0 || ret >= SEND_BUF_SIZE) 
            return;  // 填充失败或内容超长
        
        uint16_t send_len = (uint16_t)ret;  // 有效发送长度
        
        // 3. 检查UART状态，确保就绪
        if (HAL_UART_GetState(huart_) != HAL_UART_STATE_READY) 
            // 阻塞式发送无需等待DMA，直接尝试重置UART
            HAL_UART_Abort(huart_);
        
        
        // 4. 阻塞式发送（等待发送完成）
        if (send_len > 0) 
            // 使用HAL_UART_Transmit（阻塞式），超时时间设为100ms
            HAL_UART_Transmit(huart_, Sendbuf, send_len, time_out);
    }

    void reset_timeout(uint32_t timeout_ms) 
    {
        time_out = timeout_ms;
    }

private:
    uint32_t time_out = 100;
    UART_HandleTypeDef *huart_;
    uint8_t Sendbuf[SEND_BUF_SIZE];
};

#endif

#endif // __DEBUG_UART_H