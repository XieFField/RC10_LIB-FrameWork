#include "Rc10_Uart.h"
// 实例管理器实现
USB_USART* InstanceManager::instances[4] = {nullptr};

void InstanceManager::RegisterInstance(USB_UART_mode_E mode, USB_USART* instance) {
    if (mode >= 0 && mode < 4) {
        instances[mode] = instance;
    }
}

USB_USART* InstanceManager::GetInstance(USB_UART_mode_E mode) {
    if (mode >= 0 && mode < 4) {
        return instances[mode];
    }
    return nullptr;
}

USB_USART* InstanceManager::GetInstanceByUartHandle(UART_HandleTypeDef *huart) {
    for (int i = 0; i < 4; i++) {
        if (instances[i] && instances[i]->GetMode() != USB_MODE) {
            if (instances[i]->GetUartHandle() == huart) {
                return instances[i];
            }
        }
    }
    return nullptr;
}

void InstanceManager::UnregisterInstance(USB_UART_mode_E mode) {
    if (mode >= 0 && mode < 4) {
        instances[mode] = nullptr;
    }
}

// USB_USART 类实现
USB_USART::USB_USART(USB_UART_mode_E mode, UART_USB_Struct *usb_uart_struct, 
                     UART_HandleTypeDef *uart_handle, USBD_HandleTypeDef *usb_handle)
{
    usb_uart = usb_uart_struct;
    control_mode_ = mode;
    
    if(control_mode_ == USB_MODE) {
        usbhandle_ = usb_handle;
        if(usbhandle_ == NULL)
            Error_Handler();
    } else {
        // UART1_MODE, UART2_MODE 等
        uarthandle_ = uart_handle;
        if(uarthandle_ == NULL)
            Error_Handler();
    }
    
    // 注册到实例管理器
    InstanceManager::RegisterInstance(mode, this);
}

void USB_USART::USB_UART_Init() {
    if(control_mode_ == USB_MODE) {
        // USB模式初始化
    } else {
        // UART模式初始化
        if(uarthandle_ == NULL)
            Error_Handler();

        if(uarthandle_->Instance == USART1 || uarthandle_->Instance == USART2) {
            __HAL_UART_CLEAR_IDLEFLAG(uarthandle_);
            __HAL_UART_ENABLE_IT(uarthandle_, UART_IT_IDLE);
            HAL_UART_Receive_DMA(uarthandle_, usb_uart->rx_buffer, usb_uart->rx_buffer_size);
        } else {
            Error_Handler();
        }
    }
}

void USB_USART::Receive_Callback(uint8_t* Buf, uint32_t Len) {
    if(control_mode_ == USB_MODE) {
        usb_uart->RxCallback_Fuc(Buf, Len);
    } else {
        // UART模式
        usb_uart->rx_buffer_size = usb_uart->rx_buffer_size - 
                                  ((DMA_Stream_TypeDef*)uarthandle_->hdmarx->Instance)->NDTR;
        usb_uart->RxCallback_Fuc(usb_uart->rx_buffer, usb_uart->rx_buffer_size);
    }
}

// C 接口的全局回调函数
#ifdef __cplusplus
extern "C" {
#endif

void Uart_USB_Receive_Callback_Global(uint8_t* Buf, uint32_t Len,USB_UART_mode_E mode) {
    // 这个函数可能需要根据具体上下文决定调用哪个实例
    // 或者可以移除，使用更具体的中断回调
	    if(InstanceManager::GetInstance(mode) != nullptr)
    {
        InstanceManager::GetInstance(mode)->Receive_Callback(Buf,Len);
    }
}

// UART 空闲中断回调
/*void HAL_UART_IdleCallback(UART_HandleTypeDef *huart) {
    USB_USART* instance = InstanceManager::GetInstanceByUartHandle(huart);
    if (instance != nullptr) {
        // 调用实例的接收处理
        // 注意：这里需要根据实际数据传递方式调整
        instance->Receive_Callback(nullptr, 0);
    }
}*/

#ifdef __cplusplus
}
#endif













/*
static USB_USART* g_usb_uart_instance = nullptr;

USB_USART::USB_USART(USB_UART_mode_E mode,UART_USB_Struct *usb_uart_struct,UART_HandleTypeDef *uart_handle,USBD_HandleTypeDef *usb_handle)
{
	usb_uart = usb_uart_struct;
	control_mode_=mode;
	if(control_mode_==UART_MODE)
	{
		uarthandle_ =uart_handle;
    if(uarthandle_ == NULL)
        Error_Handler();
    else{}
	}
	else if(control_mode_==USB_MODE)
	{
		usbhandle_=usb_handle;
		if(usbhandle_ == NULL)
        Error_Handler();
    else{}
	}
}
void USB_USART::USB_UART_Init()
{
	if(control_mode_==UART_MODE)
	{
    if(uarthandle_ == NULL)
        Error_Handler();
    else{}

    if(uarthandle_->Instance == USART1)
    {
        __HAL_UART_CLEAR_IDLEFLAG(uarthandle_);
				__HAL_UART_ENABLE_IT(uarthandle_, UART_IT_IDLE);
				HAL_UART_Receive_DMA(uarthandle_, usb_uart->rx_buffer, usb_uart->rx_buffer_size);
    }
    else
    {
        Error_Handler();
    }
	}
	else if(control_mode_==USB_MODE){
		
	};
	g_usb_uart_instance = this;
}
void USB_USART::Receive_Callback(uint8_t* Buf, uint32_t Len)
{
	if(control_mode_==UART_MODE)
	{
	usb_uart->rx_buffer_size=usb_uart->rx_buffer_size - ((DMA_Stream_TypeDef*)uarthandle_->hdmarx->Instance)->NDTR;
	usb_uart->RxCallback_Fuc(usb_uart->rx_buffer,usb_uart->rx_buffer_size);
	}
	else if(control_mode_==USB_MODE)
	{
	usb_uart->RxCallback_Fuc(Buf,Len);
	}
}

// C 兼容的全局函数实现
#ifdef __cplusplus
extern "C" {
#endif

void Uart_USB_Receive_Callback_Global(uint8_t* Buf, uint32_t Len)
{
    if(g_usb_uart_instance != nullptr)
    {
        g_usb_uart_instance->Receive_Callback(Buf,Len);
    }
}

#ifdef __cplusplus
}
#endif*/