#ifndef TEST_SERIAL1_STM32H7XX_HAL_H
#define TEST_SERIAL1_STM32H7XX_HAL_H

#include <cstdint>
#include <vector>

struct USART_TypeDef
{
};

struct UART_HandleTypeDef
{
    USART_TypeDef *Instance = nullptr;
    uint8_t *pRxBuffPtr = nullptr;
};

using HAL_StatusTypeDef = int;

constexpr HAL_StatusTypeDef HAL_OK = 0;

extern std::vector<std::vector<uint8_t>> g_test_uart_tx_frames;
extern int g_test_receive_to_idle_calls;
extern uint16_t g_test_last_receive_to_idle_len;
extern int g_test_clear_idle_calls;
extern int g_test_delay_calls;
extern UART_HandleTypeDef *g_test_last_receive_uart;
extern UART_HandleTypeDef *g_test_last_tx_uart;

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t len);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t len);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t len, uint32_t timeout);
void HAL_Delay(uint32_t ms);
void testHostResetHalState();

inline void __HAL_UART_CLEAR_IDLEFLAG(UART_HandleTypeDef *)
{
    ++g_test_clear_idle_calls;
}

#endif
