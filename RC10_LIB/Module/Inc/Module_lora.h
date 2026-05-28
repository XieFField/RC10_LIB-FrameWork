#pragma once

#include "Module_communication.h"
#include "BSP_USB_UART_Driver.h"

#define MAX_GPIO_EXTI_NUM 16

namespace tim { class Tim; }

#ifdef __cplusplus

namespace communication {

class Lora_communication : public Communication {
public:
    static Lora_communication* GetInstance(
        UART_HandleTypeDef* tx_huart = nullptr,
        UART_HandleTypeDef* rx_huart = nullptr,
        GPIO_TypeDef* tx_aux_port = nullptr,
        uint16_t tx_aux_pin = 0,
        GPIO_TypeDef* rx_aux_port = nullptr,
        uint16_t rx_aux_pin = 0,
        tim::Tim* timer = nullptr);

    void Init();
    void Task_Process();        // ← public：主循环调用
    void Tim_It_Process();      // ← public：定时器中断调用

    static void All_EXTI_Prosess(uint16_t gpio_pin_);

protected:
    virtual void Comm_TxUseTxDMA(UART_HandleTypeDef* huart, uint8_t* data, uint16_t size) override;
    void EXTI_Prosess();        // ← protected：只由 All_EXTI_Prosess 内部调用

private:
    Lora_communication(UART_HandleTypeDef* tx_huart, UART_HandleTypeDef* rx_huart,
         GPIO_TypeDef* tx_aux_gpio_port, uint16_t tx_aux_gpio_pin,
          GPIO_TypeDef* rx_aux_gpio_port, uint16_t rx_aux_gpio_pin,
           tim::Tim* timer);
    ~Lora_communication();

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
    static Lora_communication* gpio_exti_list[MAX_GPIO_EXTI_NUM];
    static void RxCallback(uint8_t* buf, uint16_t len);
};

}

#endif