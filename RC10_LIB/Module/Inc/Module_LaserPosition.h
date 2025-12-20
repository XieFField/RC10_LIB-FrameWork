/**
 * @file Module_LaserPosition.
 * @author Zhuang Ji cao  Zhang Jia jia
 * @brief USB UART驱动文件
 * @attention 此文件用于USB UART
 * @date 2025-12-1
 */

#ifndef __MODULE_LaserPosition_H
#define __MODULE_LaserPosition_H


#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
	
#include <stdint.h>

#include "main.h"
#include <string.h>
#include <math.h>
#include "usart.h"	
#include "stm32h7xx_hal.h"
#define RX_BUFFER_SIZE_Laser 15 

#define LaserModule1Address				0x00							// 激光测距模块1地址
#define LaserModule1ReadAddress			(LaserModule1Address | 0x80)	// 激光测距模块1读地址
#define LaserModule1WriteAddress		LaserModule1Address				// 激光测距模块1写地址

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
#include "BSP_USB_UART_Driver.h"

class LaserPosition :public UART_
{
public:
	float Data;
  LaserPosition(uint16_t rx_buffer_size,uint8_t *rx_buffer,UART_HandleTypeDef *uart_handle);
  void Callback_Fuc(uint8_t *buf, uint16_t len) override;
  uint8_t LaserPositioningState = 0;	// 激光定位状态变量
  WorldXYCoordinatesTypedef WorldXYCoordinates;	// 世界坐标系XY坐标变量，在场地内面向正北，场地右上角顶点为坐标原点，正西为X轴，正南为Y轴
//  float Yaw = (3.0f / 2.0f) * PI;					// 偏航角变量，单位弧度，0表示世界坐标系正X轴方向，逆时针为正方向，范围是-PI到PI之间
  uint8_t LaserModuleGroup_Init(LaserModuleDataTypedef* LaserModuleData);
  uint8_t LaserModule_StateContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData);
  uint8_t LaserModule_StopContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData);
  uint8_t LaserModule_AnalysisModulesMeasurementResults(LaserModuleDataTypedef* LaserModuleData);
	void Init();
  float Get_data(){return Data;}
	void Config(LaserModuleDataTypedef* LaserModuleData);
  LaserModuleDataTypedef LaserModule1;
protected:
  
private:
   UART_HandleTypeDef *uart_handle;
   bool init_flag = false;
// 实例成员函数
	void ResetCallbackStatus(); 
  volatile uint8_t uart_callback_executed;
	volatile uint8_t uart_callback_result;
};

class Laser_InstanceManager:public RtosTask
{
public:
	  Laser_InstanceManager();
    void InstanceManager_Init();
    static void RegisterInstance(LaserPosition* Laser_instance);//注册
    TickType_t LastTimestamp = xTaskGetTickCount();
    //static LaserPosition** get_instance(){return laser_instances;}
protected:
	  static LaserPosition* laser_instances[4]; // 支持最多4个实例
    void loop() override;
    friend class Locate_Setup;
private:
   
};

#endif // __cplusplus

#endif