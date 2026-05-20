#ifndef TEST_TDD_MAIN_H
#define TEST_TDD_MAIN_H

#include "APP_debugTool.h"

struct GPIO_TypeDef
{
};

inline UART_HandleTypeDef huart7{};

inline GPIO_TypeDef test_gpio_port_1{};
inline GPIO_TypeDef test_gpio_port_2{};
inline GPIO_TypeDef test_gpio_port_3{};
inline GPIO_TypeDef test_gpio_port_4{};

#define kPHOTOGATE_1_GPIO_Port (&test_gpio_port_1)
#define kPHOTOGATE_2_GPIO_Port (&test_gpio_port_2)
#define kPHOTOGATE_3_GPIO_Port (&test_gpio_port_3)
#define kPHOTOGATE_4_GPIO_Port (&test_gpio_port_4)

#define kPHOTOGATE_1_Pin 1U
#define kPHOTOGATE_2_Pin 2U
#define kPHOTOGATE_3_Pin 3U
#define kPHOTOGATE_4_Pin 4U

using HAL_StatusTypeDef = int;
using GPIO_PinState = int;

#define HAL_OK 0
#define HAL_UART_STATE_READY 0
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1

inline int HAL_UART_GetState(UART_HandleTypeDef *)
{
    return HAL_UART_STATE_READY;
}

inline HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, unsigned char *, unsigned short)
{
    return HAL_OK;
}

inline GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, unsigned short)
{
    return GPIO_PIN_RESET;
}

inline void Error_Handler()
{
}

#endif
