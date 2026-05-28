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
#define kPHOTOGATE_4_Pin GPIO_PIN_8
#define kPHOTOGATE_4_GPIO_Port GPIOF
#define kPHOTOGATE_3_Pin GPIO_PIN_9
#define kPHOTOGATE_3_GPIO_Port GPIOF
#define Lora_IO1_Pin GPIO_PIN_10
#define Lora_IO1_GPIO_Port GPIOB
#define Lora_IO2_Pin GPIO_PIN_11
#define Lora_IO2_GPIO_Port GPIOB
#define kPHOTOGATE_1_Pin GPIO_PIN_14
#define kPHOTOGATE_1_GPIO_Port GPIOD
#define kPHOTOGATE_2_Pin GPIO_PIN_15
#define kPHOTOGATE_2_GPIO_Port GPIOD
#define SUCKER_error_Pin GPIO_PIN_3
#define SUCKER_error_GPIO_Port GPIOG
#define SUCKER_2_Pin GPIO_PIN_4
#define SUCKER_2_GPIO_Port GPIOG
#define SUCKER_3_Pin GPIO_PIN_5
#define SUCKER_3_GPIO_Port GPIOG
#define SUCKER_4_Pin GPIO_PIN_6
#define SUCKER_4_GPIO_Port GPIOG
#define SUCKER_5_Pin GPIO_PIN_7
#define SUCKER_5_GPIO_Port GPIOG
#define SUCKER_6_Pin GPIO_PIN_8
#define SUCKER_6_GPIO_Port GPIOG
#define RXLora_uart2_Pin GPIO_PIN_6
#define RXLora_uart2_GPIO_Port GPIOC
#define RXLora_uart2C7_Pin GPIO_PIN_7
#define RXLora_uart2C7_GPIO_Port GPIOC
#define TXLora_uart1_Pin GPIO_PIN_12
#define TXLora_uart1_GPIO_Port GPIOC
#define TXLora_uart1D2_Pin GPIO_PIN_2
#define TXLora_uart1D2_GPIO_Port GPIOD
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
