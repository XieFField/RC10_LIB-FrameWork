/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/*FRAMEDEMO_BEGIN*/
#include "frame_demo.h"
/*FRAMEDEMO_END*/

#include "UsbFrameReceiver.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// USB帧接收器实例
static UsbFrameReceiver* usbReceiver = nullptr;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 定义是否使用STL（根据项目配置选择）
// #define USE_STL

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#include "Setup_ConfigInit.h"
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern void fdcan_global_scheduler_tick_isr(void);

// USB接收统计信息打印计时器
static uint32_t usbStatsTimer = 0;
int cnt=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __cplusplus
extern "C"{
#endif
void MX_FREERTOS_Init(void);
    
// USB帧接收回调函数声明
void usbFrameCallback(uint8_t id, const uint8_t* data, uint8_t length);
void usbErrorCallback(const char* error);

#ifdef __cplusplus
}
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// CanTest test_demo(&hfdcan1, 0x001); // CAN1 测试实例

/**
  * @brief USB帧数据接收回调函数
  * @param id 帧ID
  * @param data 数据指针
  * @param length 数据长度
  */
void usbFrameCallback(uint8_t id, const uint8_t* data, uint8_t length)
{
    // 在这里处理接收到的USB帧数据
  //  printf("USB Frame Received - ID: 0x%02X, Length: %d, Data: ", id, length);
    cnt=cnt+1;
    for (uint8_t i = 0; i < length; i++) {
       // printf("%02X ", data[i]);
    }
  //  printf("\n");
    
    // 示例：根据ID处理不同命令
    switch (id) {
        case 0x01:
            // 处理命令1
            //printf("Command 1 received\n");
            break;
        case 0x02:
            // 处理命令2
           // printf("Command 2 received\n");
            break;
        default:
          //  printf("Unknown command ID: 0x%02X\n", id);
            break;
    }
    
    // 示例：发送响应帧
    uint8_t response[] = {0xAA, 0xBB, 0xCC};
    if (usbReceiver) {
			  cnt=cnt+1;
        usbReceiver->sendFrame(0x80, response, sizeof(response));
			
    }
}

/**
  * @brief USB错误回调函数
  * @param error 错误信息
  */
void usbErrorCallback(const char* error)
{
  //  printf("USB Error: %s\n", error);
}

/**
  * @brief 打印USB统计信息
  */
void printUsbStatistics()
{
    if (usbReceiver) {
        UsbFrameReceiver::Statistics stats = usbReceiver->getStatistics();
      //  printf("USB Statistics - Frames: %lu, Bytes: %lu, Errors: %lu, Last Error: %lu\n",
               stats.totalFramesReceived,
               stats.totalBytesReceived,
               stats.errorCount,
               stats.lastErrorCode;
    }
}

/**
  * @brief 发送测试USB帧
  */
void sendTestUsbFrame()
{
    if (usbReceiver) {
        uint8_t testData[] = {0x11, 0x22, 0x33, 0x44, 0x55};
        bool success = usbReceiver->sendFrame(0x01, testData, sizeof(testData));
        if (success) {
         //   printf("Test USB frame sent successfully\n");
        } else {
          //  printf("Failed to send test USB frame\n");
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    MX_DMA_Init();
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_FDCAN1_Init();
    MX_FDCAN2_Init();
    MX_FDCAN3_Init();
    MX_USART1_UART_Init();
    MX_TIM6_Init();
    MX_TIM4_Init();
    
    /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim6); //启动定时器否则CAN任务不会跑的
    HAL_TIM_Base_Start_IT(&htim4); // 启动TIM4用于时间戳
    
    // 初始化所有配置
    ALL_Setup_ConfigInit();

    // 初始化USB帧接收器
   // printf("Initializing USB Frame Receiver...\n");
    
    usbReceiver = new UsbFrameReceiver(&hUsbDeviceHS, 
                                      usbFrameCallback, 
                                      usbErrorCallback);
    
    if (usbReceiver->init()) {
      //  printf("USB Frame Receiver initialized successfully\n");
        
        // 发送测试帧
        sendTestUsbFrame();
    } else {
      //  printf("Failed to initialize USB Frame Receiver\n");
			  int s=1;
        Error_Handler();
    }
    
    usbStatsTimer = HAL_GetTick();

    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
    MX_FREERTOS_Init();

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        // 定期打印USB统计信息（每5秒）
        uint32_t currentTime = HAL_GetTick();
        if (currentTime - usbStatsTimer >= 5000) {
            printUsbStatistics();
            usbStatsTimer = currentTime;
            
            // 定期发送心跳帧
            static uint8_t heartbeatCounter = 0;
            uint8_t heartbeatData[] = {heartbeatCounter++};
            if (usbReceiver) {
                usbReceiver->sendFrame(0xFF, heartbeatData, sizeof(heartbeatData));
            }
        }
        
        // 短暂延时，让出CPU时间
        osDelay(100);
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                                |RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;  // 改为HSE
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 160;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)  // 改为2
    {
        Error_Handler();
    }

    /** Configure USB clock
    */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;  // 使用HSI48
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
    */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM8 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM8)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */
    if(htim->Instance == TIM6)
    {
        fdcan_global_scheduler_tick_isr();
    }
    
    if (htim->Instance == TIM4) // 假设你使用的是 TIM4
    {
        TimeStamp::overflowCallback();
    }
    /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

// 重定向printf到串口（如果使用串口调试）
#ifdef __GNUC__
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}