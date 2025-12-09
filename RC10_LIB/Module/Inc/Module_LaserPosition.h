/**
* @file Module_LaserPosition.
* @author Zhuang Ji cao  Zhang Jia jia
* @brief USB UART驱动文件
* @attention 此文件用于USB UART
* @date 2025-10-1
* 
* 
* @brief 类封装，只对应一个模块
* 
* 		 激光类，实例化后对应一个激光模块
* 		 对外的功能接口，只需要一个， return_LaserDate()->返回此激光模块的测距数据(单位/米)
*/

#ifndef __MODULE_LaserPosition_H
#define __MODULE_LaserPosition_H


#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
	
#include <stdint.h>
#include "BSP_USB_UART_Driver.h"
#include "main.h"
#include <string.h>
#include <math.h>
#include "usart.h"	
#include "stm32h7xx_hal.h"
#define RX_BUFFER_SIZE_Laser 15 
#define PI							3.14159265358979323846f			// 定义圆周率常量PI


#define LaserModule_1_UartHandle &huart6		// 激光测距模块1串口句柄
#define LaserModule_2_UartHandle &huart4		// 激光测距模块2串口句柄

#define LaserModule1Address				0x00							// 激光测距模块1地址
#define LaserModule1ReadAddress			(LaserModule1Address | 0x80)	// 激光测距模块1读地址
#define LaserModule1WriteAddress		LaserModule1Address				// 激光测距模块1写地址

#define LaserModule2Address				0x00							// 激光测距模块2地址
#define LaserModule2ReadAddress			(LaserModule2Address | 0x80)	// 激光测距模块2读地址
#define LaserModule2WriteAddress		LaserModule2Address 			// 激光测距模块2写地址

//float delta_hoop_x = 3.884f;
//float delta_hoop_y = 0.746f;

typedef struct LaserModuleConfigurationData
{
	UART_HandleTypeDef* UartHandle;			// 串口句柄
//	QueueHandle_t ReceiveQueue;		// 串口DMA接收队列句柄
	uint8_t Address;			// 激光模块原始地址
	uint8_t ReadAddress;
	uint8_t WriteAddress;
}LaserModuleConfigurationDataTypedef;

typedef struct LaserModuleMeasurementData
{
	uint32_t Distance;
	uint16_t SignalQuality;
	uint16_t State;
}LaserModuleMeasurementDataTypedef;

typedef struct LaserModuleData
{
	LaserModuleConfigurationDataTypedef ConfigurationData;
	LaserModuleMeasurementDataTypedef MeasurementData;
}LaserModuleDataTypedef;

typedef struct LaserModuleDataGroup
{
	LaserModuleDataTypedef LaserModule1;
	LaserModuleDataTypedef LaserModule2;
}LaserModuleDataGroupTypedef;

typedef struct WorldXYCoordinates
{
	float X;		// 单位：m
	float Y;		// 单位：m
}WorldXYCoordinatesTypedef;
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
#include "BSP_RTOS.h"

class LaserPosition :public RtosTask
{
public:
	uint32_t Laser_Y;
	uint32_t Laser_X;
	float Laser_Y_return;
	float Laser_X_return;
	static constexpr float delta_hoop_x = 4.050f;
	static constexpr float delta_hoop_y = 0.967f;
	static constexpr uint16_t deltaX = 224;
	static constexpr uint16_t deltaY = 220;
	float YYY;
	float XXX;
 LaserPosition(UART_HandleTypeDef *uart_handle1, UART_HandleTypeDef *uart_handle2);
 uint8_t LaserPositioningState = 0;	// 激光定位状态变量
 WorldXYCoordinatesTypedef WorldXYCoordinates;	// 世界坐标系XY坐标变量，在场地内面向正北，场地右上角顶点为坐标原点，正西为X轴，正南为Y轴
 float Yaw = (3.0f / 2.0f) * PI;					// 偏航角变量，单位弧度，0表示世界坐标系正X轴方向，逆时针为正方向，范围是-PI到PI之间
 TickType_t LastTimestamp = xTaskGetTickCount();			// 上次时间戳变量，用于vTaskDelayUntil()函数的绝对
 uint8_t LaserModuleGroup_Init(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
 //uint8_t LaserModule_TurnOnTheLaserPointer(LaserModuleDataTypedef* LaserModuleData);
 uint8_t LaserModule_StateContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData);
 uint8_t LaserModule_StopContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData);
 uint8_t LaserModuleGroup_AnalysisModulesMeasurementResults(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
 uint8_t LaserModule_AnalysisModulesMeasurementResults(LaserModuleDataTypedef* LaserModuleData);
 uint8_t LaserPositioning_YawJudgment(float* Yaw);
 void LaserPositioning_XYWorldCoordinatesCalculate(WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw, uint32_t FrontLaser, uint32_t RightLaser);
 void LaserPositioning_GetYaw(float* Yaw);
 void GetPositionYaw(float* Yaw);
 uint8_t LaserPositioning_XYWorldCoordinatesVerification(const WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw);
 void LaserPositioning_SendXYWorldCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates);
 void SendPositionXYCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates);
	void Init();
	void Config(LaserModuleDataGroupTypedef* LaserModuleDataGroup);
protected:
   void loop() override;

private:
	 static LaserModuleDataGroupTypedef LaserModuleDataGroup;
	 static constexpr uint32_t TX_TIMEOUT_MS = 10;  // 发送超时时间
	 UART_ uart1_;  
  UART_ uart2_;
  UART_HandleTypeDef *uart1_handle;
  UART_HandleTypeDef *uart2_handle;
  uint8_t rx_buffer1[RX_BUFFER_SIZE_Laser];
  uint8_t rx_buffer2[RX_BUFFER_SIZE_Laser];	
  bool init_flag = false;
// 实例成员函数
 void Uart1Callback(uint8_t *buf, uint16_t len);
 void Uart2Callback(uint8_t *buf, uint16_t len);
	void ResetCallbackStatus(); 
 volatile uint8_t uart1_callback_executed;
	volatile uint8_t uart1_callback_result;
	volatile uint8_t uart2_callback_executed; 
	volatile uint8_t uart2_callback_result;
 // 静态成员函数
 static void StaticUart1Callback(uint8_t *buf, uint16_t len);
 static void StaticUart2Callback(uint8_t *buf, uint16_t len);
	UART_* uart_instance_1;
 UART_* uart_instance_2;
	// 保存实例指针用于静态函数访
 static LaserPosition* instance_; 

};


#endif // __cplusplus

#endif