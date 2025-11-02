/**
 * @file BSP_USB_UART_Driver.cpp
 * @author Zhuang Ji cao
 * @brief USB UART驱动文件
 * @attention 此文件用于USB UART
 * @date 2025-10-1
 */
#include "BSP_USB_UART_Driver.h"
// 若你选择方案A，可额外包含 DMA 头；方案B不强制需要
// #include "stm32h7xx_hal_dma.h"

// 实例管理器实现
UART_* InstanceManager::uart_instances[UART_MAX] = {nullptr};
USB_CDC_* InstanceManager::usb_instances[2]={nullptr};
uint8_t n=0;
uint8_t m=0;
void InstanceManager::RegisterInstance(UART_* uart_instance,USB_CDC_* usb_instance) 
{
	if(n<UART_MAX&&uart_instance!=NULL)
	{
      uart_instances[n] = uart_instance;
      n++;
	}
  else if(m<USB_MAX&&usb_instance!=NULL)
  {
      usb_instances[m]=usb_instance;
      m++;
  }
}

UART_* InstanceManager::GetInstanceByUartHandle(UART_HandleTypeDef *huart) 
{
    for (int i = 0; i < UART_MAX; i++) 
    {
        if (uart_instances[i] != nullptr && uart_instances[i]->GetUartHandle() == huart) 
            return uart_instances[i];
        
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
            Error_Handler();
				
				else
				{
				InstanceManager::RegisterInstance(this,NULL);
				// 注册到实例管理器
				
				}
}

void UART_::UART_Init()
{
    if (uarthandle_ == NULL) { Error_Handler(); return; }

    // 0) 确保 DMA RX 已停止
    if (uarthandle_->hdmarx) {
        HAL_DMA_Abort(uarthandle_->hdmarx);
    }

    // 1) 清错误与 IDLE 标志，抽干 RDR（顺序很重要）
    __HAL_UART_CLEAR_OREFLAG(uarthandle_);
    __HAL_UART_CLEAR_FEFLAG(uarthandle_);
    __HAL_UART_CLEAR_NEFLAG(uarthandle_);
    __HAL_UART_CLEAR_PEFLAG(uarthandle_);
    __HAL_UART_CLEAR_IDLEFLAG(uarthandle_);
    while (__HAL_UART_GET_FLAG(uarthandle_, UART_FLAG_RXNE)) {
        volatile uint8_t dump = (uint8_t)uarthandle_->Instance->RDR;
        (void)dump;
    }
    __HAL_UART_CLEAR_IDLEFLAG(uarthandle_);

#ifdef USART_CR3_OVRDIS
    // 2) 可选：禁用 Overrun，容忍初始化前洪泛
    SET_BIT(uarthandle_->Instance->CR3, USART_CR3_OVRDIS);
#endif

    // 3) 启动 ToIdle + DMA（只启一次，用实例缓冲）
    HAL_UARTEx_ReceiveToIdle_DMA(uarthandle_, rx_buffer, rx_buffer_size);

    // 4) 触发源选择：若上位机可能持续发送，建议保留半传中断；否则可以关闭
#if 1
    // 留下半传以保证“无空闲也能回调”
    // 默认 HAL 已开 HT/TC，无需额外动作
#else
    if (uarthandle_->hdmarx) {
        __HAL_DMA_DISABLE_IT(uarthandle_->hdmarx, DMA_IT_HT); // 若明确只想靠 IDLE/TC
    }
#endif

}

//虚拟串口
USB_CDC_* InstanceManager::GetInstanceByUSBHandle() 
{
    for (int i = 0; i < USB_MAX; i++) 
    {
      if(usb_instances[i]!=NULL )
        return usb_instances[i];
    
      else
        return nullptr;	 
    }
}
USB_CDC_::USB_CDC_(RxCallback RxCallback_Fuc,USBD_HandleTypeDef *usb_handle)
{
				this->RxCallback_Fuc=RxCallback_Fuc;
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
void USB_CDC_::CDC_Receive_Callback(uint8_t* Buf, uint32_t Len) 
{
    RxCallback_Fuc(Buf, Len);
}




// C 接口的全局回调函数
#ifdef __cplusplus
extern "C" 
{
#endif

void USB_Receive_Callback_Global(uint8_t* Buf, uint32_t Len) 
{
     InstanceManager::GetInstanceByUSBHandle()->CDC_Receive_Callback(Buf,Len);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    UART_* inst = InstanceManager::GetInstanceByUartHandle(huart);
    if (!inst) return;

    // 用实例缓冲 + 实际 Size
    inst->Callback_Fuc(inst->rx_buffer, Size);

    // 重新启动接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, inst->rx_buffer, inst->rx_buffer_size);
    // 若选择关闭半传，这里再关一次以防 HAL 重置
    // __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}



#ifdef __cplusplus
}
#endif












