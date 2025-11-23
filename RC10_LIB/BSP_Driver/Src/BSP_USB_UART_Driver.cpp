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
//				  __HAL_UART_DISABLE_IT(&huart1,UART_IT_RXNE);
//					__HAL_UART_DISABLE_IT(&huart1,UART_IT_PE);
//					__HAL_UART_DISABLE_IT(&huart1,UART_IT_ERR);
//					__HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
//					// 2)清错误与空闲标志(顺序很重要)
//					__HAL_UART_CLEAR_OREFLAG(&huart1),
//					__HAL_UART_CLEAR_OREFLAG(&huart1);
//					__HAL_UART_CLEAR_OREFLAG(&huart1),
//					__HAL_UART_CLEAR_OREFLAG(&huart1);
//					__HAL_UART_CLEAR_OREFLAG(&huart1);
//					//3)抽干接收FIFO/RDR
//					while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)){
//						volatile uint8_t dump =(uint8_t)huart1.Instance->RDR;
//						(void)dump;}
//					__HAL_UART_CLEAR_IDLEFLAG(&huart1);
//					// 4)启动连续接收(推荐 IDLE + DMA)
//					static uint8_t uart1_rxbuf[512];
//					
//					__HAL_DMA_DISABLE_IT(huart1.hdmarx,DMA_IT_HT);// 关半传中断
				 HAL_UART_AbortReceive_IT(&huart1);	
         HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, rx_buffer_size);
				 memset(large_data_buffer, 1,large_data_size);
				 HAL_UART_Transmit_DMA(&huart1, large_data_buffer, large_data_size);
				 for(int i=0;i<100000;i++)
				 {};
				 			
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
void USB_CDC_::CDC_Receive_Callback(uint8_t* Buf, uint32_t Len) {
        RxCallback_Fuc(Buf, Len);
}




// C 接口的全局回调函数
#ifdef __cplusplus
extern "C" {
#endif

void USB_Receive_Callback_Global(uint8_t* Buf, uint32_t Len) {
     InstanceManager::GetInstanceByUSBHandle()->CDC_Receive_Callback(Buf,Len);
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{    UART_* instance = InstanceManager::GetInstanceByUartHandle(huart);
//    if (instance != nullptr) {

//				/* clear DMA transfer complete flag */
//			HAL_UART_DMAStop(huart);
//        // 调用实例的接收处理，使用HAL提供的Size参数
//      instance->Callback_Fuc(huart->pRxBuffPtr,instance->rx_buffer_size);
//   
//        
//        // 可选：重新启动DMA接收，实现连续接收
//         HAL_UART_Receive_DMA(&huart1, huart->pRxBuffPtr,instance->rx_buffer_size);
//    }
//}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    UART_* instance = InstanceManager::GetInstanceByUartHandle(huart);
    if (instance != nullptr) {
			
			    /* clear idle it flag avoid idle interrupt all the time */
			__HAL_UART_CLEAR_IDLEFLAG(huart);

				/* clear DMA transfer complete flag */
			//HAL_UART_DMAStop(huart);
        // 调用实例的接收处理，使用HAL提供的Size参数
      instance->Callback_Fuc(huart->pRxBuffPtr,instance->rx_buffer_size);        
        // 重新启动DMA接收
			
			HAL_UARTEx_ReceiveToIdle_DMA(huart, instance->rx_buffer, instance->rx_buffer_size);
			if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
			{
				  Error_Handler();
			}
      HAL_DMA_Abort(huart->hdmatx);
    }
}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	UART_* instance = InstanceManager::GetInstanceByUartHandle(huart);
//    if(instance != nullptr)
//    {
//        // 1. 最高优先级：保护接收
//        instance->Callback_Fuc(huart->pRxBuffPtr,instance->rx_buffer_size);  
//        HAL_UART_Transmit_DMA(huart, large_data_buffer,1);
//			
//				HAL_UART_Receive_IT(huart, instance->rx_buffer, instance->rx_buffer_size);
//    }
//}

#ifdef __cplusplus
}
#endif












