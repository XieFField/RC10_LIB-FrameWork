#pragma once

#include "RC_communication.h"
#include "BSP_USB_UART_Driver.h"
#include "RC_gpio_exti.h"

namespace tim { class Tim; }

#ifdef __cplusplus

namespace communication {

class Lora_communication : public Communication, public gpio::GpioExti {
public:
    Lora_communication(UART_HandleTypeDef* tx_huart, UART_HandleTypeDef* rx_huart,
         GPIO_TypeDef* tx_aux_gpio_port, uint16_t tx_aux_gpio_pin,
          GPIO_TypeDef* rx_aux_gpio_port, uint16_t rx_aux_gpio_pin,
           tim::Tim* timer);
    ~Lora_communication();

    void Init();

protected:
    virtual void Comm_TxUseTxDMA(UART_HandleTypeDef* huart, uint8_t* data, uint16_t size) override;
//    virtual void Uart_Rx_It_Process(uint8_t* buf_, uint16_t len_);
    virtual void Task_Process();
    virtual void EXTI_Prosess() override;
    void Tim_It_Process();

private:
    UART_HandleTypeDef* lora_tx_huart;
    UART_HandleTypeDef* lora_rx_huart;
    GPIO_TypeDef* lora_aux_port;
    uint16_t lora_aux_pin;
    uint32_t timer_tick_count;
    
    uint8_t tx_ring_buffer[RING_BUF_SIZE];
    uint8_t rx_ring_buffer[RING_BUF_SIZE];
    alignas(32) uint8_t tx_dma_buffer[DMA_BUF_SIZE];
    alignas(32) uint8_t rx_dma_buffer[DMA_BUF_SIZE];

    UART_ bsp_rx;
    tim::Tim* attached_timer;

    static Lora_communication* s_instance;
    static void RxCallback(uint8_t* buf, uint16_t len);
};

}

#endif