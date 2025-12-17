/**
 * @file BSP_USB_UART_Driver.cpp
 * @author Zhuang Ji cao
 * @brief USB UART???????
 * @attention ?????????USB UART
 * @date 2025-10-1
 */

#ifndef __BSP_USB_UART_Driver_H
#define __BSP_USB_UART_Driver_H

#pragma once
#include "usart.h"	
#include "usb_device.h"
#include "stm32h7xx_hal.h"
#include "usbd_cdc.h"
#ifdef __cplusplus
extern "C" {
#endif

//???庯???????
typedef void (*RxCallback)(uint8_t *buf, uint16_t len);
extern void CDC_Receive_(uint8_t* buf, uint32_t *len);	
//USB和UART回调函数
/** 
* @brief define the uart struct
*/

#ifdef __cplusplus
}

#endif
#ifdef __cplusplus

#define UART_MAX 10
#define USB_MAX 1

class UART_{
public:
    
    UART_(uint16_t rx_buffer_size,uint8_t *rx_buffer,UART_HandleTypeDef *uart_handle);
    ~UART_(){}
			//?????麯??
		virtual void Callback_Fuc(uint8_t *buf, uint16_t len);
		void SetCallback(RxCallback callback) {RxCallback_Fuc = callback;}// ??????????
		void UART_Receive_Callback(uint8_t* Buf, uint32_t Len);
    UART_HandleTypeDef* GetUartHandle() const { return uarthandle_;}
		void UART_Init();
		uint16_t rx_buffer_size;
		uint8_t *rx_buffer;
private:
    RxCallback RxCallback_Fuc;	  
		UART_HandleTypeDef *uarthandle_;//UART???
};

class USB_CDC_{
	public:
    USB_CDC_(USBD_HandleTypeDef *usb_handle);
    ~USB_CDC_(){}
    virtual void Callback_DCD_Fuc(uint8_t *buf, uint16_t len);
    USBD_HandleTypeDef* GetUSBHandle() const { return usbhandle_; }
private:
    RxCallback RxCallback_Fuc;	 
		USBD_HandleTypeDef *usbhandle_;//USB???
};
// ?????????
class InstanceManager {
public:
    static void RegisterInstance(UART_* uart_instance,USB_CDC_* usb_instance);//???
    static USB_CDC_* GetInstanceByUSBHandle();
    static UART_* GetInstanceByUartHandle(UART_HandleTypeDef *huart);
private:
		static USB_CDC_* usb_instances[2];
    static UART_* uart_instances[UART_MAX]; // 支持最多10个实例
};

#endif // __cplusplus

#endif