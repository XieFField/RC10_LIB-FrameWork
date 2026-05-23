/**
 * @file BSP_USB_UART_Driver.cpp
 * @author Zhuang Ji cao
 * @brief USB UART??????????
 * @attention ?????????????USB UART
 * @date 2025-10-1
 */
#include "BSP_USB_UART_Driver.h"

// ?????????????????
UART_* InstanceManager::uart_instances[UART_MAX] = {nullptr};
USB_CDC_* InstanceManager::usb_instances[2]={nullptr};
uint8_t n=0;
uint8_t m=0;
extern "C" 
{
   uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);
}
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
// UART_ ???????
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
	}
}

void UART_::UART_Init()
{
	if(uarthandle_ == NULL)
	{
		Error_Handler();
	}
    else 
	{
        HAL_UARTEx_ReceiveToIdle_DMA(uarthandle_, rx_buffer, rx_buffer_size);
    } 
}


//?????????
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
				// ?????????????????
				InstanceManager::RegisterInstance(NULL,this);
				}
}
void UART_::Callback_Fuc(uint8_t *buf, uint16_t len)
{
    if (this->RxCallback_Fuc != nullptr) 
	{
        RxCallback_Fuc(buf, len);
    }
}
void USB_CDC_::Callback_DCD_Fuc(uint8_t *buf, uint16_t len)
{
{
	uint8_t i = 0;
	uint8_t break_flag = 1;
	while (i < len && break_flag == 1)
		{
			/*-----------------------------------------????--------------------*/
			switch (receive_flag)
			{
			case WAIT_HEAD_1:// 0xaa
				if (buf[i] == 0xaa) 
				{
					receive_flag = WAIT_HEAD_2;
				}
				i++;
				break;
				
			case WAIT_HEAD_2:// 0x55
				if (buf[i] == 0x55) receive_flag = WAIT_ID;
				else receive_flag = WAIT_HEAD_1;
			  i++;
				break;
				
			case WAIT_ID:// 1~MAX
				if (buf[i] > MAX_RECEIVE_ID || buf[i] == 0) receive_flag = WAIT_HEAD_1;
				else 
				{
					receive_id = buf[i];
					receive_flag = WAIT_LEN;
				}
				i++;
				break;
				
			case WAIT_LEN:
				if (buf[i] > MAX_RECEIVE_DATA_LEN) receive_flag = WAIT_HEAD_1;
				else
				{
					receive_len = buf[i];
					receive_flag = WAIT_DATA;
					receive_data_dx = 0;
				}
				i++;
				break;
				
			case WAIT_DATA:
				receive_data[receive_data_dx] = buf[i];
				receive_data_dx++;
				if (receive_data_dx >= receive_len) 
				{
					receive_flag = WAIT_CHECK;
					receive_data_dx = 0;
				}
				i++;
				break;
			
			case WAIT_CHECK:
				if (buf[i] == xor_check(receive_data, receive_len)) receive_flag = WAIT_TAIL;
				else receive_flag = WAIT_HEAD_1;
			  	i++;
				break;
				
			case WAIT_TAIL:// 0xee
				if (buf[i] == 0xee)
				{
					/*-----------------------????-------------------------*/
				if (receive_len != 0)
				{
					if(receive_id==1)
					{
						uint8_t m=0;
						for(int i=0;i<=receive_len-4;i=i+4)
						{
							Data_.data1[m]=*(float*)(&receive_data[i]);
							m++;
						}
					}
					else if(receive_id == 5)
					{
						relocate_suceed_cnt++;
					}
				}
//				uint8_t a=0x11;
//			  CDC_Send_(0x04,&a,0x01);
					/*-----------------------????-------------------------*/
				  
				}
				break_flag = 0;
				receive_flag = WAIT_HEAD_1;
				break;
			
			default:
				receive_flag = WAIT_HEAD_1;
		    i++;
				break;
			}
			/*-------------------------------------????--------------------*/
		}
	}
}


void USB_CDC_::CDC_Send_(uint8_t id_, uint8_t *data, uint16_t len)
{
		send_buf[0] = head_1;
		send_buf[1] = head_2;
		send_buf[2] = id_;
		send_buf[3] = len;
		memcpy(&send_buf[4], data, len);
		send_buf[len + 4] = xor_check(data, len);
		send_buf[len + 5] = tail;
		uint8_t send_ret = USBD_BUSY;
		send_ret = CDC_Transmit_HS(send_buf, sizeof(send_buf));
}
uint8_t USB_CDC_::xor_check(const uint8_t *data, uint32_t length)
{
	uint8_t xor_val = 0;
	for (uint16_t i = 0; i < length; i++)
	{
		xor_val ^= data[i]; // ??
	}
	return xor_val;
}
// C ?????????
#ifdef __cplusplus
extern "C" {
#endif
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    UART_* instance = InstanceManager::GetInstanceByUartHandle(huart);
    if (instance != nullptr) {
       // ??? HAL ????? Size ?????????????????????
        instance->Callback_Fuc(huart->pRxBuffPtr, Size);
        // ???????????MA??
        HAL_UARTEx_ReceiveToIdle_DMA(huart, instance->rx_buffer, instance->rx_buffer_size);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	  UART_* instance = InstanceManager::GetInstanceByUartHandle(huart);
	// ??????????????????????
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF);// ????????????????????
    }
    
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);// ??????????????
    }
    
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_NEF);// ???????????????????
    }
    
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);// ??????????????????
    }
	
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_LBDF))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_LBDF);// LIN????????????????
    }
	
	HAL_UARTEx_ReceiveToIdle_DMA(huart, instance->rx_buffer, instance->rx_buffer_size);
}


void CDC_Receive_(uint8_t* Buf, uint32_t *Len)
{
 {
    USB_CDC_* instance = InstanceManager::GetInstanceByUSBHandle();
    if (instance != nullptr && Buf != nullptr && Len != nullptr) {
        // ?????????????????????
        instance->Callback_DCD_Fuc(Buf, *Len);
        
        // USB CDC ?????????????????
        // ???USB CDC ?????????? HAL_UARTEx_ReceiveToIdle_DMA
        // ???? USBD_CDC_ReceivePacket ?????
        USBD_CDC_SetRxBuffer(instance->GetUSBHandle(), Buf);
        USBD_CDC_ReceivePacket(instance->GetUSBHandle());
    }
 }
}
#ifdef __cplusplus
}
#endif












