#include "Rc10_Uart.h"

usart_struct usart1_struct;
void Uart_Init(UART_HandleTypeDef *huart, uint8_t *Rxbuffer, uint16_t len, RxCallback RxCallback_Fuction)
{
    if(huart == NULL)
        Error_Handler();
    else{}

    if(huart->Instance == USART1)
    {
        usart1_struct.uart_handle = huart;
        usart1_struct.rx_buffer = Rxbuffer;
        usart1_struct.rx_buffer_size = len;
        usart1_struct.RxCallback_Fuc = RxCallback_Fuction;
        __HAL_UART_CLEAR_IDLEFLAG(huart);
				__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
				HAL_UART_Receive_DMA(huart, Rxbuffer, len);
    }
    else
    {
        Error_Handler();
    }
}
void Uart_Receive_Callback(usart_struct *uart)
{
	uart->rx_buffer_size=uart->rx_buffer_size - ((DMA_Stream_TypeDef*)uart->uart_handle->hdmarx->Instance)->NDTR;
	uart->RxCallback_Fuc(uart->rx_buffer,uart->rx_buffer_size);
}