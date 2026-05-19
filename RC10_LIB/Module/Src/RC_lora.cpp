#include "RC_lora.h"

namespace communication {



/* ========== 构造 / 析构 ========== */
Lora_communication::Lora_communication(UART_HandleTypeDef* tx_huart, UART_HandleTypeDef* rx_huart,
     GPIO_TypeDef* tx_aux_gpio_port, uint16_t tx_aux_gpio_pin,
      GPIO_TypeDef* rx_aux_gpio_port, uint16_t rx_aux_gpio_pin,
       tim::Tim* timer)
    : Communication(tx_huart, rx_huart,
        tx_ring_buffer, tx_dma_buffer, rx_ring_buffer, rx_dma_buffer,
        tx_aux_gpio_port, tx_aux_gpio_pin, rx_aux_gpio_port, rx_aux_gpio_pin),
      gpio::GpioExti(tx_aux_gpio_pin),
      bsp_rx(DMA_BUF_SIZE, rx_dma_buffer, rx_huart),
      attached_timer(timer),
      timer_tick_count(0)
{
    lora_tx_huart = tx_huart;
    lora_rx_huart = rx_huart;
    lora_aux_port = tx_aux_gpio_port;
    lora_aux_pin = tx_aux_gpio_pin;
}

Lora_communication::~Lora_communication() {
}

/* ========== 初始化 ========== */
Lora_communication* Lora_communication::s_instance = nullptr;
void Lora_communication::Init() {
    s_instance = this;
    bsp_rx.SetCallback(RxCallback);
    bsp_rx.UART_Init();
}

/* ========== 静态回调：BSP UART 接收 ========== */
void Lora_communication::RxCallback(uint8_t* buf, uint16_t len) {
    if (s_instance) {
        s_instance->Comm_RxDMAToRxBuffer(s_instance->lora_rx_huart, len);
    }
}

/* ========== 虚函数实现 ========== */
void Lora_communication::Comm_TxUseTxDMA(UART_HandleTypeDef* huart, uint8_t* data, uint16_t size) {
    if (huart == lora_tx_huart) {
        HAL_UART_Transmit_DMA(huart, data, size);
    }
}

//void Lora_communication::Uart_Rx_It_Process(uint8_t* buf_, uint16_t len_) {
////    Comm_RxDMAToRxBuffer(lora_rx_huart, len_);
//}

void Lora_communication::Task_Process() {
    if (Comm_Task_Loop()) {
        static uint16_t joystick[4];
        static uint16_t key;
        GetRecvData(joystick, key);
        static uint8_t command, load1, load2;
        GetSettingData(command, load1, load2);
    }
}

/* ========== 定时器中断========== */
void Lora_communication::Tim_It_Process() {
    timer_tick_count++;
    if (timer_tick_count >= 20) {
        timer_tick_count = 0;
        Comm_SendAxisDataToTxBuffer(1, 2, 5, 1, 1, 1, 0xBB, 0xCC, 0xDD);
    }
}

/* ========== GPIO 中断自动触发 ========== */
void Lora_communication::EXTI_Prosess() {
    Comm_TxBufferToTxDMA(lora_tx_huart);
}

} // namespace communication