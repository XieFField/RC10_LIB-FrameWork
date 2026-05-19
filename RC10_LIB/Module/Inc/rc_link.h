#pragma once
#include <stdint.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void RC_Init(void);
bool RC_Process(uint16_t joy[4], uint16_t* key);
void RC_Send(uint16_t x, uint16_t y, uint16_t z,
			 uint8_t gripper, uint8_t suction, uint8_t auto_mode,
			 uint8_t mode, uint8_t cmd1, uint8_t cmd2);

/* 供已有 HAL 回调调用的接口 */
void RC_OnUartTxCplt(UART_HandleTypeDef *huart);
void RC_OnTxAuxRising(void);
void RC_OnTimPeriodElapsed(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif