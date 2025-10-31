/**
 * @file BSP_USB_UART_Driver.cpp
 * @author Zhuang Ji cao
 * @brief USB UART驱动文件
 * @attention 此文件用于USB UART
 * @date 2025-10-1
 */
#include "BSP_USB_UART_Driver.h"

// 实例管理器实现
UART_* InstanceManager::uart_instances[UART_MAX] = {nullptr};
USB_CDC_* InstanceManager::usb_instances[2]={nullptr};
uint8_t n=0;
uint8_t m=0;
extern uint8_t large_data_buffer[];
void InstanceManager::RegisterInstance(UART_* uart_instance,USB_CDC_* usb_instance) {
	if(n<UART_MAX&&uart_instance!=NULL)
	{
        uart_instances[n] = uart_instance;
				n++;
	}else if(m<USB_MAX&&usb_instance!=NULL)
{
				usb_instances[m]=usb_instance;
				m++;
}
}

UART_* InstanceManager::GetInstanceByUartHandle(UART_HandleTypeDef *huart) {
    for (int i = 0; i < UART_MAX; i++) {
            if (uart_instances[i]->GetUartHandle() == huart) {
                return uart_instances[i];
            }
    }
    return nullptr;
}
//USART
// UART_ 类实现
UART_::UART_(uint16_t rx_buffer_size,uint8_t *rx_buffer,UART_HandleTypeDef *uart_handle)
{
				this->rx_buffer= rx_buffer;
				this->rx_buffer_size=rx_buffer_size;
        this->uarthandle_ = uart_handle;
        if(this->uarthandle_ == NULL)
				{
            Error_Handler();
				}
				else
				{
				InstanceManager::RegisterInstance(this,NULL);
				// 注册到实例管理器
				
				}
}

void UART_::UART_Init()
{
				if(uarthandle_ == NULL){
									Error_Handler();}
       else {
				__HAL_UART_CLEAR_IDLEFLAG(uarthandle_);
        __HAL_UART_CLEAR_FLAG(uarthandle_, UART_FLAG_RXNE);
				__HAL_UART_CLEAR_IDLEFLAG(uarthandle_);
			  __HAL_UART_ENABLE_IT(uarthandle_, UART_IT_PE);     // 奇偶错误中断
        __HAL_UART_ENABLE_IT(uarthandle_, UART_IT_ERR);
				__HAL_UART_ENABLE_IT(uarthandle_, UART_IT_IDLE);
				HAL_UART_Receive_DMA(uarthandle_,this->rx_buffer,this->rx_buffer_size);
}
}

//虚拟串口
USB_CDC_* InstanceManager::GetInstanceByUSBHandle(USBD_HandleTypeDef *usb_handle) {
    for (int i = 0; i < USB_MAX; i++) {
            if (usb_instances[i]->GetUSBHandle() == usb_handle) {
                return usb_instances[i];
            }
						 else{
    return nullptr;
						 }
  }
}
USB_CDC_::USB_CDC_(USBD_HandleTypeDef *usb_handle)
{
        this->usbhandle_ = usb_handle;
        if(this->usbhandle_ == NULL)
				{
            Error_Handler();
				}
				else
				{
				// 注册到实例管理器
				InstanceManager::RegisterInstance(NULL,this);
				}
}

// C 接口的全局回调函数
#ifdef __cplusplus
extern "C" {
#endif
//hUsbDeviceHS
void CDC_Receive_Callback(uint8_t *buf, uint16_t len,USBD_HandleTypeDef *usb_handle) {
	  USB_CDC_* instance = InstanceManager::GetInstanceByUSBHandle(usb_handle);
    if (instance != nullptr) {
        instance->Callback_Fuc(buf,len);
}
}
void UART_Receive_Callback(UART_HandleTypeDef *huart)
{
    UART_* instance = InstanceManager::GetInstanceByUartHandle(huart);
    if (instance != nullptr) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        
        // 停止DMA
        HAL_UART_DMAStop(huart);
        
        // 计算实际接收的数据长度
        uint16_t received_len;
        
        // 方法1：使用RX XferCount（如果HAL库维护了此计数）
        if (huart->RxXferCount > 0) {
            received_len = instance->rx_buffer_size - huart->RxXferCount;
        } 
        // 方法2：使用DMA计数器（带保护）
        else if (huart->hdmarx != NULL) {
            received_len = instance->rx_buffer_size - __HAL_DMA_GET_COUNTER(huart->hdmarx);
            
            // 验证长度合理性
            if (received_len > instance->rx_buffer_size) {
                received_len = instance->rx_buffer_size; // 限制到最大缓冲区大小
            }
        } else {
            received_len = instance->rx_buffer_size; // 回退方案
        }        
        if (received_len > 0) {
            instance->Callback_Fuc(instance->rx_buffer, received_len);
        }
        
        // 重新启动DMA
        HAL_UART_Receive_DMA(huart, instance->rx_buffer, instance->rx_buffer_size);
    }
}
#ifdef __cplusplus
}
#endif












