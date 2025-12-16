/**
 * @file BSP_USB_UART_Driver.cpp
 * @author Zhuang Ji cao
 * @brief USB UART�����ļ�
 * @attention ���ļ�����USB UART
 * @date 2025-10-1
 */
#include "BSP_USB_UART_Driver.h"

// ʵ��������ʵ��
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
// UART_ ��ʵ��
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
				// ע�ᵽʵ��������
				
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


//���⴮��
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
				// ע�ᵽʵ��������
				InstanceManager::RegisterInstance(NULL,this);
				}
}
void USB_CDC_::CDC_Receive_Callback(uint8_t* Buf, uint32_t Len) {
        RxCallback_Fuc(Buf, Len);
}
void UART_::Callback_Fuc(uint8_t *buf, uint16_t len){
    if (RxCallback_Fuc != nullptr) {
        RxCallback_Fuc(buf, len);
    }
}

// C �ӿڵ�ȫ�ֻص�����
#ifdef __cplusplus
extern "C" {
#endif

void USB_Receive_Callback_Global(uint8_t* Buf, uint32_t Len) {
     InstanceManager::GetInstanceByUSBHandle()->CDC_Receive_Callback(Buf,Len);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    UART_* instance = InstanceManager::GetInstanceByUartHandle(huart);
    if (instance != nullptr) {
       // 使用 HAL 提供的 Size 参数，它表示实际收到的字节数
        instance->Callback_Fuc(huart->pRxBuffPtr, Size);
        // 重新启动接收（DMA）
        HAL_UARTEx_ReceiveToIdle_DMA(huart, instance->rx_buffer, instance->rx_buffer_size);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	  UART_* instance = InstanceManager::GetInstanceByUartHandle(huart);
	// ������п��ܵĴ����־
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);// �����żУ������־
    }
    
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);// ���֡�����־
    }
    
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);// ������������־
    }
    
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);// �����������־
    }
	
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_LBDF))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_LBDF);// LIN�ϵ����־����
    }
	
	HAL_UARTEx_ReceiveToIdle_DMA(huart, instance->rx_buffer, instance->rx_buffer_size);
}
#ifdef __cplusplus
}
#endif












