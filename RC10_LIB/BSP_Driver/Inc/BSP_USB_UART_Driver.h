/**
 * @file BSP_USB_UART_Driver.cpp
 * @author Zhuang Ji cao
 * @brief USB UART驱动文件
 * @attention 此文件用于USB UART
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
#define USB_MAX 10
#define MAX_SEND_BUF_SIZE 128// 发送缓冲区大小

#define MAX_RECEIVE_BUF_SIZE 512// 接收缓冲区大小

#define MAX_RECEIVE_ID 10// 最大id

#define MAX_RECEIVE_DATA_LEN 64
//雷达接收
	typedef enum RECEIVE_FLAG
	{
		WAIT_HEAD_1,// 0xaa
		WAIT_HEAD_2,// 0x55
		WAIT_ID,// 1~max
		WAIT_LEN,// 1~64
		WAIT_DATA,
		WAIT_CHECK,//xor
		WAIT_TAIL// 0xee
	} RECEIVE_FLAG;

	
	typedef struct 
{
  /* data */
	  float data1[MAX_RECEIVE_DATA_LEN];
    float data2[MAX_RECEIVE_DATA_LEN];
    float data3[MAX_RECEIVE_DATA_LEN];
}USB_Data;
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
    void Callback_DCD_Fuc(uint8_t *buf, uint16_t len);
    USBD_HandleTypeDef* GetUSBHandle() const { return usbhandle_; }
		void CDC_Send_(uint8_t id_, uint8_t *data, uint16_t len);
		uint8_t xor_check(const uint8_t *data, uint32_t length);
	  RECEIVE_FLAG receive_flag = WAIT_HEAD_1;
		uint8_t receive_id;
		uint8_t receive_len;
		uint8_t receive_check;
		uint8_t receive_data[MAX_RECEIVE_DATA_LEN] = {0};
		uint16_t receive_data_dx = 0;
		USB_Data Data_;
		uint8_t head_1 = 0xaa;
		uint8_t head_2 = 0x55;
		uint8_t tail=0xee;
	protected:
		uint8_t send_buf[MAX_SEND_BUF_SIZE];
	
		
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