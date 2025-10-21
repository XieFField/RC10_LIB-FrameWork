/**
 * @file position.h
 * @author  Zhuang Ji cao
 * @brief UART头文件
 */

#ifndef __RC10_UART_H
#define __RC10_UART_H

#pragma once
#include "usart.h"	
#include "usb_device.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef enum{
    USB_MODE,   // USB模式
    UART1_MODE, // UART1模式
    UART2_MODE, // UART2模式
    UART3_MODE  // 可扩展更多UART
}USB_UART_mode_E;	
	//定义函数被指针
typedef void (*RxCallback)(uint8_t *buf, uint16_t len);
	//数据结构体
typedef struct
{
    uint16_t rx_buffer_size;
    uint8_t *rx_buffer;
    RxCallback RxCallback_Fuc;
		
} UART_USB_Struct;
//USB和UART回调函数
void Uart_USB_Receive_Callback_Global(uint8_t* Buf, uint32_t Len,USB_UART_mode_E mode);	
/** 
* @brief define the uart struct
*/

#ifdef __cplusplus
}

#endif

#ifdef __cplusplus


class USB_USART {
public:
    
    USB_USART(USB_UART_mode_E mode,UART_USB_Struct *usb_uart_struct,UART_HandleTypeDef *uart_handle,USBD_HandleTypeDef *usb_hand);
    ~USB_USART(){}
		void Receive_Callback(uint8_t* Buf, uint32_t Len);
	  void USB_UART_Init();
		    USB_UART_mode_E GetMode() const { return control_mode_; }
    UART_HandleTypeDef* GetUartHandle() const { return uarthandle_; }
private:
	  UART_USB_Struct *usb_uart;//结构体指针
    USB_UART_mode_E control_mode_; //模式
		UART_HandleTypeDef *uarthandle_;//UART句柄
		USBD_HandleTypeDef *usbhandle_;//USB句柄
};

// 实例管理器
class InstanceManager {
public:
    static void RegisterInstance(USB_UART_mode_E mode, USB_USART* instance);//注册
    static USB_USART* GetInstance(USB_UART_mode_E mode);
    static USB_USART* GetInstanceByUartHandle(UART_HandleTypeDef *huart);
    static void UnregisterInstance(USB_UART_mode_E mode);
    
private:
    static USB_USART* instances[4]; // 支持最多4个实例
};

#endif // __cplusplus

#endif