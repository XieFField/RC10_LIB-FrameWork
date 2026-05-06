/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
void MX_DMA_Init(void);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void UART_IDLE_Callback(uint16_t received_length);
void parse_uart_data(uint8_t data);
//void UART_IdleCallback(UART_HandleTypeDef *huart);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AirJoy_Pin GPIO_PIN_8
#define AirJoy_GPIO_Port GPIOF
#define AirJoy_EXTI_IRQn EXTI9_5_IRQn
#define PPM_Pin GPIO_PIN_9
#define PPM_GPIO_Port GPIOF
#define SWITCH1_Pin GPIO_PIN_14
#define SWITCH1_GPIO_Port GPIOD
#define SWTICH2_Pin GPIO_PIN_15
#define SWTICH2_GPIO_Port GPIOD
#define EN_IN1_Pin GPIO_PIN_3
#define EN_IN1_GPIO_Port GPIOG
#define PH_IN2_Pin GPIO_PIN_4
#define PH_IN2_GPIO_Port GPIOG
#define DRVOFF_Pin GPIO_PIN_5
#define DRVOFF_GPIO_Port GPIOG
#define nSLEEP_Pin GPIO_PIN_6
#define nSLEEP_GPIO_Port GPIOG
#define nFAULT_Pin GPIO_PIN_7
#define nFAULT_GPIO_Port GPIOG
#define SUCKER_P1_Pin GPIO_PIN_5
#define SUCKER_P1_GPIO_Port GPIOD
#define SUCKER_P2_Pin GPIO_PIN_6
#define SUCKER_P2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
