/**
 * @file position.h
 * @author  Zhuang Ji cao
 * @brief UART驱动文件
 */

#ifndef BSP_USART_DRIVER_H
#define BSP_USART_DRIVER_H
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif 
	//定义函数指针
typedef uint32_t (*RxCallback)(uint8_t *buf, uint16_t len);
/** 
* @brief define the uart struct
*/
typedef struct
{
    UART_HandleTypeDef *uart_handle;
    uint16_t rx_buffer_size;
    uint8_t *rx_buffer;
    RxCallback RxCallback_Fuc;
}usart_struct;
extern usart_struct usart1_struct;
void Uart_Init(UART_HandleTypeDef *huart, uint8_t *Rxbuffer, uint16_t len, RxCallback RxCallback_Fuction);
void Uart_Receive_Callback(usart_struct *uart);


#ifdef __cplusplus
}
#endif
#endif
