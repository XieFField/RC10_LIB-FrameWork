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
            HAL_UARTEx_ReceiveToIdle_DMA(uarthandle_, rx_buffer, rx_buffer_size);
        } 
}


//虚拟串口
USB_CDC_* InstanceManager::GetInstanceByUSBHandle() {
    for (int i = 0; i < USB_MAX; i++) {
             if(usb_instances[i]!=NULL){
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
void UART_::Callback_Fuc(uint8_t *buf, uint16_t len){
    if (this->RxCallback_Fuc != nullptr) {
        RxCallback_Fuc(buf, len);
    }
}
void USB_CDC_::Callback_DCD_Fuc(uint8_t *buf, uint16_t len)
{
    if (this->RxCallback_Fuc != nullptr) {
        RxCallback_Fuc(buf, len);
    }
}
// C 接口的全局回调函数
#ifdef __cplusplus
extern "C" {
#endif
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    UART_* instance = InstanceManager::GetInstanceByUartHandle(huart);
    if (instance != nullptr) {
        // 调用实例的接收处理，使用HAL提供的Size参数
      instance->Callback_Fuc(huart->pRxBuffPtr,instance->rx_buffer_size);        
        // 重新启动DMA接收
			HAL_UARTEx_ReceiveToIdle_DMA(huart, instance->rx_buffer, instance->rx_buffer_size);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	  UART_* instance = InstanceManager::GetInstanceByUartHandle(huart);
	// 清除所有可能的错误标志
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);// 清除奇偶校验错误标志
    }
    
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);// 清除帧错误标志
    }
    
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);// 清除噪声错误标志
    }
    
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);// 清除溢出错误标志
    }
	
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_LBDF))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_LBDF);// LIN断点检测标志处理
    }
	
	HAL_UARTEx_ReceiveToIdle_DMA(huart, instance->rx_buffer, instance->rx_buffer_size);
}


void CDC_Receive_(uint8_t* Buf, uint32_t *Len)
{
 {
    USB_CDC_* instance = InstanceManager::GetInstanceByUSBHandle();
    if (instance != nullptr && Buf != nullptr && Len != nullptr) {
        // 调用实例的接收处理，使用传入的缓冲区和长度
        instance->Callback_DCD_Fuc(Buf, *Len);
        
        // USB CDC 接收处理完成，通常需要重新启动接收
        // 注意：USB CDC 使用不同的机制，不是 HAL_UARTEx_ReceiveToIdle_DMA
        // 通常使用 USBD_CDC_ReceivePacket 或类似函数
        USBD_CDC_SetRxBuffer(instance->GetUSBHandle(), Buf);
        USBD_CDC_ReceivePacket(instance->GetUSBHandle());
    }
 }
}
#ifdef __cplusplus
}
#endif












