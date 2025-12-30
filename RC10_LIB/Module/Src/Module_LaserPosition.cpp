/**
 * @file Module_LaserPosition.
 * @author Ha Ji cao ++
 * @brief USB UART驱动文件
 * @attention 此文件用于USB UART
 * @date 2025-12-1
 */
#include "Module_LaserPosition.h"
LaserPosition* Laser_InstanceManager::laser_instances[4] = {nullptr, nullptr, nullptr, nullptr};  
uint8_t count=0;
LaserPosition::LaserPosition(uint16_t rx_buffer_size,uint8_t *rx_buffer,UART_HandleTypeDef *uart_handle)
    :	UART_(rx_buffer_size,rx_buffer,uart_handle),
			uart_callback_executed(0),
      uart_callback_result(0)
{
	this->uart_handle=uart_handle;
	//Laser_InstanceManager::RegisterInstance(this);
}
Laser_InstanceManager::Laser_InstanceManager()
: RtosTask("LaserTask", 1)
{
	
}
void Laser_InstanceManager::InstanceManager_Init()
{
	for(int i=0;i<4;i++)
	{
		if(laser_instances[i]!=nullptr){
		laser_instances[i]->Init();
		}
	}
	Laser_InstanceManager::start(osPriorityNormal, 512);
}
	
void Laser_InstanceManager::RegisterInstance(LaserPosition* Laser_instance)
{
	if(count<4)
	{
		laser_instances[count]=Laser_instance;
		count++;
	}
}
void LaserPosition::Config(LaserModuleDataTypedef* LaserModuleData)
{
	LaserModuleData->ConfigurationData.UartHandle = uart_handle;		// 设置激光测距模块1的串口句柄
	LaserModuleData->ConfigurationData.Address = LaserModule1Address;
	LaserModuleData->ConfigurationData.ReadAddress = LaserModule1ReadAddress;
	LaserModuleData->ConfigurationData.WriteAddress = LaserModule1WriteAddress;
	LaserModuleData->MeasurementData.Distance = 0;	// 激光测距模块1距离数据初始化
	LaserModuleData->MeasurementData.SignalQuality = 0;	// 激光测距模块1信号质量数据初始化
	LaserModuleData->MeasurementData.State = 0;	// 激光测距模块1状态数据初始化
}
void LaserPosition::Init() 
{   
		LaserPosition::UART_::UART_Init();
	  Config(&(this->LaserModule1));
}

// 处理激光模块1的数据
void LaserPosition::Callback_Fuc(uint8_t *buf, uint16_t len) {
    uint8_t result = 0;
    // 设置回调执行状态
    uart_callback_executed = 1;
    uart_callback_result = result;
}

uint8_t LaserPosition::LaserModuleGroup_Init(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleGroupState = 0;		// 激光测距模块状态变量
	TickType_t Timestamp = 0;
	vTaskDelayUntil(&Timestamp, pdMS_TO_TICKS(3000));	// 确保自上电以来已经延时3000ms，确保激光测距模块已完成模块内部初始化
 // osDelay(100);//16V 电池只用100
	LaserModuleGroupState |= LaserModule_StateContinuousAutomaticMeasurement(LaserModuleData);	// 激光测距模块1连续自动测量状态设置
	return LaserModuleGroupState;			// 返回激光测距模块状态
}

void Laser_InstanceManager::loop() 
{
	uint8_t LaserModuleGroupState = 0;	// 激光测距模块状态变量
	uint8_t LaserPositioningState = 0;	// 激光定位状态变量
	static bool initialized = false;
    if (!initialized) {
        for(int i=0;i<4;i++) {
            if(laser_instances[i]!=nullptr){
                laser_instances[i]->LaserModuleGroup_Init(&(laser_instances[i]->LaserModule1));
            }
        }
        initialized = true;
    }
	for(int i=0;i<4;i++)
	{
		if(laser_instances[i]!=nullptr){
		LaserModuleGroupState = 0;	// 激光测距模块状态重置
		LaserPositioningState = 0;  // 激光定位状态重置
		(laser_instances[i]->LaserModule1).MeasurementData.State = 0;	// 激光测距模块1状态重置
		LaserModuleGroupState |= laser_instances[i]->LaserModule_AnalysisModulesMeasurementResults(&(laser_instances[i]->LaserModule1));			// 激光测距模块组读取测量结果
    laser_instances[i]->Data = (laser_instances[i]->LaserModule1).MeasurementData.Distance/1000.f;
		if(laser_instances[i]->Data == 0)
		{
			//osDelay(100);
			laser_instances[i]->LaserModuleGroup_Init(&(laser_instances[i]->LaserModule1));		// 激光测距模块组初始化
		//	osDelay(100);
		}		
	}
}
		vTaskDelayUntil(&LastTimestamp, pdMS_TO_TICKS(100));		// 每40ms执行一次任务
}

uint8_t LaserPosition::LaserModule_StateContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleState = 0;	// 激光测距模块状态变量

	// 设置连续自动测量的命令0xAA, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x25
	
	uint8_t CMD[9] = { 0xAA, LaserModuleData->ConfigurationData.WriteAddress, 0x00, 0x20, 0x00, 0x01, 0x00, 0x04, 0x00 };
	uint8_t CheckValueCalculation = CMD[1] + CMD[2] + CMD[3] + CMD[4] + CMD[5] + CMD[6] + CMD[7];
	CMD[8] = CheckValueCalculation;
	LaserModuleState |= HAL_UART_Transmit_DMA(LaserModuleData->ConfigurationData.UartHandle, CMD, sizeof(CMD));	
	// 恢复调度器
	for(int i=0;i<100000;i++)
	{}
	return LaserModuleState;			// 返回激光测距模块状态
}

uint8_t LaserPosition::LaserModule_StopContinuousAutomaticMeasurement(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleState = 0;	// 激光测距模块状态变量
  uint8_t CMD[1] = { 0x58 };
	LaserModuleState |= HAL_UART_Transmit_DMA(LaserModuleData->ConfigurationData.UartHandle, CMD, sizeof(CMD));		// 发送设置停止连续自动测量模块的命令
	for(int i=0;i<100000;i++)
	{}
	return LaserModuleState;			// 返回激光测距模块状态
}

uint8_t LaserPosition::LaserModule_AnalysisModulesMeasurementResults(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleState = 0;		// 激光测距模块状态变量
		uint32_t Distance =
			(this->rx_buffer[6] << 24) |
			(this->rx_buffer[7] << 16) |
			(this->rx_buffer[8] << 8) |
			(this->rx_buffer[9] << 0);		// 接收并计算距离
	
		uint16_t SignalQuality =
			(this->rx_buffer[10] << 8) |
			(this->rx_buffer[11] << 0);		// 接收并计算信号质量
	
		uint8_t CheckValueReceive = this->rx_buffer[12];	// 接收校验值
	
		uint8_t CheckValueCalculation = 0;
		for (uint8_t i = 1; i < 12; i++)
		{
			CheckValueCalculation += this->rx_buffer[i];		// 计算校验值
		}
	
		if (CheckValueReceive == CheckValueCalculation)
		{
			LaserModuleData->MeasurementData.Distance = Distance;				// 更新激光测距模块1的距离数据
			LaserModuleData->MeasurementData.SignalQuality = SignalQuality;
		}
		else
		{
			LaserModuleData->MeasurementData.State |= 0x04;		// 激光测距模块测量错误，错误原因，接收数据包校验位不通过
			LaserModuleState |= 0x01;							// 激光测距模块状态异常
		}
	return LaserModuleState;			// 返回激光测距模块状态
}

