#include "stm32h7xx_hal.h"

#include <vector>

std::vector<std::vector<uint8_t>> g_test_uart_tx_frames;
int g_test_receive_to_idle_calls = 0;
uint16_t g_test_last_receive_to_idle_len = 0;
int g_test_clear_idle_calls = 0;
int g_test_delay_calls = 0;
UART_HandleTypeDef *g_test_last_receive_uart = nullptr;
UART_HandleTypeDef *g_test_last_tx_uart = nullptr;
uint32_t g_test_time_ms = 0;

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t len)
{
    ++g_test_receive_to_idle_calls;
    g_test_last_receive_uart = huart;
    g_test_last_receive_to_idle_len = len;
    if (huart != nullptr) {
        huart->pRxBuffPtr = buffer;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t len)
{
    g_test_last_tx_uart = huart;
    g_test_uart_tx_frames.emplace_back(buffer, buffer + len);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t len, uint32_t)
{
    g_test_last_tx_uart = huart;
    g_test_uart_tx_frames.emplace_back(buffer, buffer + len);
    return HAL_OK;
}

void HAL_Delay(uint32_t)
{
    ++g_test_delay_calls;
}

void testHostResetHalState()
{
    g_test_uart_tx_frames.clear();
    g_test_receive_to_idle_calls = 0;
    g_test_last_receive_to_idle_len = 0;
    g_test_clear_idle_calls = 0;
    g_test_delay_calls = 0;
    g_test_last_receive_uart = nullptr;
    g_test_last_tx_uart = nullptr;
}
