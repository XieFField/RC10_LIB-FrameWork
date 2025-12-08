/**
 * @file Module_LaserPosition.
 * @author Ha Ji cao ++
 * @brief USB UART驱动文件
 * @attention 此文件用于USB UART
 * @date 2025-10-1
 */
#include "Module_LaserPosition.h"
uint8_t count=0;
// 静态成员变量定义
LaserPosition* LaserPosition::instance_ = nullptr;
LaserModuleDataGroupTypedef LaserPosition::LaserModuleDataGroup;
LaserPosition::LaserPosition(UART_HandleTypeDef *uart_handle1, UART_HandleTypeDef *uart_handle2) 
    : uart1_(RX_BUFFER_SIZE_Laser, rx_buffer1, uart_handle1),
      uart2_(RX_BUFFER_SIZE_Laser, rx_buffer2, uart_handle2),
      RtosTask("LaserTask", 1),
			uart1_handle(uart_handle1),
			uart2_handle(uart_handle2),
			uart1_callback_executed(0),
      uart1_callback_result(0),
      uart2_callback_executed(0),
      uart2_callback_result(0)
{
    instance_ = this;  // 设置静态实例指针
    // 为每个UART实例设置回调函数
		uart1_.SetCallback(StaticUart1Callback);
    uart2_.SetCallback(StaticUart2Callback);
}
void LaserPosition::Config(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	LaserModuleDataGroup->LaserModule1.ConfigurationData.UartHandle = uart1_handle;		// 设置激光测距模块1的串口句柄
	LaserModuleDataGroup->LaserModule1.ConfigurationData.Address = LaserModule1Address;
	LaserModuleDataGroup->LaserModule1.ConfigurationData.ReadAddress = LaserModule1ReadAddress;
	LaserModuleDataGroup->LaserModule1.ConfigurationData.WriteAddress = LaserModule1WriteAddress;
	LaserModuleDataGroup->LaserModule1.MeasurementData.Distance = 0;	// 激光测距模块1距离数据初始化
	LaserModuleDataGroup->LaserModule1.MeasurementData.SignalQuality = 0;	// 激光测距模块1信号质量数据初始化
	LaserModuleDataGroup->LaserModule1.MeasurementData.State = 0;	// 激光测距模块1状态数据初始化

	// 激光测距模块2初始化
	LaserModuleDataGroup->LaserModule2.ConfigurationData.UartHandle = uart2_handle;		// 设置激光测距模块2的串口句柄
	LaserModuleDataGroup->LaserModule2.ConfigurationData.Address = LaserModule2Address;
	LaserModuleDataGroup->LaserModule2.ConfigurationData.ReadAddress = LaserModule2ReadAddress;
	LaserModuleDataGroup->LaserModule2.ConfigurationData.WriteAddress = LaserModule2WriteAddress;
	LaserModuleDataGroup->LaserModule2.MeasurementData.Distance = 0;	// 激光测距模块2距离数据初始化
	LaserModuleDataGroup->LaserModule2.MeasurementData.SignalQuality = 0;	// 激光测距模块2信号质量数据初始化
	LaserModuleDataGroup->LaserModule2.MeasurementData.State = 0;	// 激光测距模块2状态数据初始化

}
void LaserPosition::Init() 
{
    uart_instance_1 = InstanceManager::GetInstanceByUartHandle(uart1_handle);
    uart_instance_2 = InstanceManager::GetInstanceByUartHandle(uart2_handle);
    // 初始化UART
    uart_instance_1->UART_Init();
    uart_instance_2->UART_Init();
	  start(osPriorityNormal, 256);
	  Config(&LaserModuleDataGroup);

}
// 静态回调函数 - 转发到实例函数
void LaserPosition::StaticUart1Callback(uint8_t *buf, uint16_t len) {
    if (instance_ != nullptr) {
        instance_->Uart1Callback(buf, len);
    }
}

void LaserPosition::StaticUart2Callback(uint8_t *buf, uint16_t len) {
    if (instance_ != nullptr) {
        instance_->Uart2Callback(buf, len);
    }
}
// 处理激光模块1的数据
void LaserPosition::Uart1Callback(uint8_t *buf, uint16_t len) {
    uint8_t result = 0;
    // 设置回调执行状态
    uart1_callback_executed = 1;
    uart1_callback_result = result;
}
// 处理激光模块2的数据  
void LaserPosition::Uart2Callback(uint8_t *buf, uint16_t len) 
{
    // 在这里处理激光模块2接收到的数据
	   uint8_t result = 0;
    // 设置回调执行状态
    uart2_callback_executed = 1;
    uart2_callback_result = result;

}

uint8_t LaserPosition::LaserModuleGroup_Init(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	uint8_t LaserModuleGroupState = 0;		// 激光测距模块状态变量
	// 激光测距模块1初始化
	TickType_t Timestamp = 0;
	vTaskDelayUntil(&Timestamp, pdMS_TO_TICKS(1000));	// 确保自上电以来已经延时3000ms，确保激光测距模块已完成模块内部初始化
  osDelay(100);

	LaserModuleGroupState |= LaserModule_StateContinuousAutomaticMeasurement(&LaserModuleDataGroup->LaserModule1);	// 激光测距模块1连续自动测量状态设置
	LaserModuleGroupState |= LaserModule_StateContinuousAutomaticMeasurement(&LaserModuleDataGroup->LaserModule2);	// 激光测距模块2连续自动测量状态设置

	return LaserModuleGroupState;			// 返回激光测距模块状态
}

void LaserPosition::loop() 
{
	uint8_t LaserModuleGroupState = 0;	// 激光测距模块状态变量
	uint8_t LaserPositioningState = 0;	// 激光定位状态变量

	if(count==0){
  LaserModuleGroupState |= LaserModuleGroup_Init(&LaserModuleDataGroup);
	count++;
	}
//	LaserModuleGroupState |= LaserModule_StateContinuousAutomaticMeasurement(&(LaserModuleDataGroup.LaserModule1 ));	// 激光测距模块1连续自动测量状态设置
//	LaserModuleGroupState |= LaserModule_StateContinuousAutomaticMeasurement(&(LaserModuleDataGroup.LaserModule2));	// 激光测距模块2连续自动测量状态设置

	  osDelay(1000);	// 延时100ms，等待激光测距模块第一次数据接收完毕

		LaserModuleGroupState = 0;	// 激光测距模块状态重置
		LaserPositioningState = 0;  // 激光定位状态重置
		LaserModuleDataGroup.LaserModule1.MeasurementData.State = 0;	// 激光测距模块1状态重置
		LaserModuleDataGroup.LaserModule2.MeasurementData.State = 0;	// 激光测距模块2状态重置

		LaserModuleGroupState |= LaserModuleGroup_AnalysisModulesMeasurementResults(&LaserModuleDataGroup);			// 激光测距模块组读取测量结果

		Laser_X = LaserModuleDataGroup.LaserModule2.MeasurementData.Distance;
    	Laser_Y = LaserModuleDataGroup.LaserModule1.MeasurementData.Distance;

		if(Laser_X == 0 || Laser_Y == 0)
		{
			osDelay(100);
			LaserModuleGroup_Init(&LaserModuleDataGroup);		// 激光测距模块组初始化
			osDelay(1000);
		}		
        // Laser_X_return = -(float)(Laser_X + 257) / 1000.f + delta_hoop_x; //不需要
        // Laser_Y_return = (float)(Laser_Y + 374) / 1000.f - delta_hoop_y;	
		vTaskDelayUntil(&LastTimestamp, pdMS_TO_TICKS(40));		// 每40ms执行一次任务
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

uint8_t LaserPosition::LaserModuleGroup_AnalysisModulesMeasurementResults(LaserModuleDataGroupTypedef* LaserModuleDataGroup)
{
	uint8_t LaserModuleGroupState = 0;		// 激光测距模块状态变量

	LaserModuleGroupState |= LaserModule_AnalysisModulesMeasurementResults(&LaserModuleDataGroup->LaserModule1);	// 激光测距模块1读取测量结果
LaserModuleGroupState |= LaserModule_AnalysisModulesMeasurementResults(&LaserModuleDataGroup->LaserModule2);	// 激光测距模块2读取测量结果

	return LaserModuleGroupState;			// 返回激光测距模块状态
}

uint8_t LaserPosition::LaserModule_AnalysisModulesMeasurementResults(LaserModuleDataTypedef* LaserModuleData)
{
	uint8_t LaserModuleState = 0;		// 激光测距模块状态变量

	if(LaserModuleData->ConfigurationData.UartHandle==uart1_handle)
	{
		uint32_t Distance =
			(rx_buffer1[6] << 24) |
			(rx_buffer1[7] << 16) |
			(rx_buffer1[8] << 8) |
			(rx_buffer1[9] << 0);		// 接收并计算距离
	
		uint16_t SignalQuality =
			(rx_buffer1[10] << 8) |
			(rx_buffer1[11] << 0);		// 接收并计算信号质量
	
		uint8_t CheckValueReceive = rx_buffer1[12];	// 接收校验值
	
		uint8_t CheckValueCalculation = 0;
		for (uint8_t i = 1; i < 12; i++)
		{
			CheckValueCalculation += rx_buffer1[i];		// 计算校验值
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
	}
	else if(LaserModuleData->ConfigurationData.UartHandle==uart2_handle)
	{
		uint32_t Distance =
			(rx_buffer2[6] << 24) |
			(rx_buffer2[7] << 16) |
			(rx_buffer2[8] << 8) |
			(rx_buffer2[9] << 0);		// 接收并计算距离
	
		uint16_t SignalQuality =
			(rx_buffer2[10] << 8) |
			(rx_buffer2[11] << 0);		// 接收并计算信号质量
	
		uint8_t CheckValueReceive = rx_buffer2[12];	// 接收校验值
	
		uint8_t CheckValueCalculation = 0;
		for (uint8_t i = 1; i < 12; i++)
		{
			CheckValueCalculation += rx_buffer2[i];		// 计算校验值
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
	}

	return LaserModuleState;			// 返回激光测距模块状态
}

uint8_t LaserPosition::LaserPositioning_YawJudgment(float* Yaw)
{
	// TODO
}

static void LaserPositioning_XYWorldCoordinatesCalculate(WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw, uint32_t FrontLaser, uint32_t RightLaser)
{
#define FrontLaserDistanceOffset_X	0							// 前激光X轴安装距离偏移量，单位：mm
#define FrontLaserDistanceOffset_Y	0							// 前激光Y轴安装距离偏移量，单位：mm
#define RightLaserDistanceOffset_X	0							// 右激光X轴安装距离偏移量，单位：mm
#define RightLaserDistanceOffset_Y	0							// 右激光Y轴安装距离偏移量，单位：mm
#define YawOffset					0.f						// 偏航角偏移量，单位：度
//#define FrontLaserAngleOffset_ActualDistance		0		// 前激光安装角度偏移量_实际距离，单位：mm
#define FrontLaserAngleOffset_OffsetDistance		0			// 前激光安装角度偏移量_偏移距离，单位：mm
#define FrontLaserAngleOffset_MeasurementDistance	0			// 前激光安装角度偏移量_测量距离，单位：mm
//#define RightLaserAngleOffset_ActualDistance		0			// 右激光安装角度偏移量_实际距离，单位：mm
#define RightLaserAngleOffset_OffsetDistance		0			// 右激光安装角度偏移量_偏移距离，单位：mm
#define RightLaserAngleOffset_MeasurementDistance	0			// 前激光安装角度偏移量_测量距离，单位：mm

	Yaw += ((float)YawOffset * PI / 180.0f);	// 偏航角偏移量校正
	
	//float FrontLaserAngleOffset = atan((float)FrontLaserAngleOffset_OffsetDistance / (float)FrontLaserAngleOffset_ActualDistance);
	//float RightLaserAngleOffset = atan(((float)(-RightLaserAngleOffset_OffsetDistance)) / (float)RightLaserAngleOffset_ActualDistance);

	float FrontLaserAngleOffset = asinf((float)FrontLaserAngleOffset_OffsetDistance / (float)FrontLaserAngleOffset_MeasurementDistance);
	float RightLaserAngleOffset = asinf(((float)(-RightLaserAngleOffset_OffsetDistance)) / (float)RightLaserAngleOffset_MeasurementDistance);

	WorldXYCoordinates->Y = -(((float)FrontLaser * sinf(Yaw - FrontLaserAngleOffset)) / 1000.0f);
	WorldXYCoordinates->X = -(((float)RightLaser * sinf(Yaw - RightLaserAngleOffset)) / 1000.0f);

	WorldXYCoordinates->X += (float)RightLaserDistanceOffset_X / 1000.0f;
	WorldXYCoordinates->Y += (float)FrontLaserDistanceOffset_Y / 1000.0f;
}

void LaserPosition::LaserPositioning_GetYaw(float* Yaw)
{
#define PositionYaw_PositiveDirection 1		// Position偏航角正方向，1表示和激光定位正方向相同，-1表示和激光定位正方向相反
#define PositionYaw_Offset 0.0f				// Position偏航角偏移量，单位角度，以激光定位正方向为0度，正方向逆时针为正方向，范围是-180到180之间

	float PositionYaw = 0.0f;		// 偏航角变量，单位弧度

	GetPositionYaw(&PositionYaw);		// 获取Position的偏航角，单位弧度

	// 坐标系转换
	*Yaw = (PositionYaw_PositiveDirection * PositionYaw) + (PositionYaw_Offset * PI / 180.0f);		// 将Position的偏航角转换为激光定位的偏航角，单位弧度
}

/**
 * @brief		获得Position的偏航角
 * @param[in]	float* Yaw 偏航角指针，单位弧度
 * @return		无
 * @note		偏航角的单位是弧度，范围是-PI到PI之间
 */
void LaserPosition::GetPositionYaw(float* Yaw)
{
	// 待实现
}

uint8_t LaserPosition::LaserPositioning_XYWorldCoordinatesVerification(const WorldXYCoordinatesTypedef* WorldXYCoordinates, float Yaw)
{
	// TODO
}

void LaserPosition::LaserPositioning_SendXYWorldCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates)
{
#define PositionXYCoordinates_Direction 1   // Position的XY坐标系方向，1表示与激光定位相同采用右手坐标系，-1表示与激光定位相反采用左手坐标系
#define PositionXYCoordinates_XAngleOffset 0.0f	// Position的XY坐标系X轴角度偏移量，单位角度，以激光定位正方向为0度，正方向逆时针为正方向，范围是-180到180之间
#define PositionXYCoordinates_OriginOffset_X 0.0f	// Position的坐标原点X坐标偏移量，单位：m，以激光定位坐标原点为参考点
#define PositionXYCoordinates_OriginOffset_Y 0.0f	// Position的坐标原点Y坐标偏移量，单位：m，以激光定位坐标原点为参考点

	float Position_X;		// 单位：m
	float Position_Y;		// 单位：m

	// 将激光定位的世界坐标系XY坐标转换为Position的世界坐标系XY坐标
	Position_X = cosf(PositionXYCoordinates_XAngleOffset) * (WorldXYCoordinates->X + PositionXYCoordinates_OriginOffset_X) + sinf(PositionXYCoordinates_XAngleOffset) * (WorldXYCoordinates->Y + PositionXYCoordinates_OriginOffset_Y);	// Position的X坐标计算
	Position_Y = -PositionXYCoordinates_Direction * sinf(PositionXYCoordinates_XAngleOffset) * (WorldXYCoordinates->X + PositionXYCoordinates_OriginOffset_X) + PositionXYCoordinates_Direction * cosf(PositionXYCoordinates_XAngleOffset) * (WorldXYCoordinates->Y + PositionXYCoordinates_OriginOffset_Y);		// Position的Y坐标计算

	// 发送Position的世界坐标系XY坐标数据
 WorldXYCoordinatesTypedef tempCoords = {Position_X, Position_Y};
 SendPositionXYCoordinates(&tempCoords);	// 发送Position的世界坐标系XY坐标数据
}

/**
 * @brief		发送Position的世界坐标系XY坐标数据
 * @param[in]	WorldXYCoordinatesTypedef* WorldXYCoordinates 世界坐标系XY坐标数据指针
 * @return		无
 * @note		世界坐标系XY坐标数据的单位是m
 */
void LaserPosition::SendPositionXYCoordinates(const WorldXYCoordinatesTypedef* WorldXYCoordinates)
{
	// 待实现
}
void LaserPosition::ResetCallbackStatus()
{
    uart1_callback_executed = 0;
    uart2_callback_executed = 0;
}


// 在USER CODE BEGIN 4区域添加回调函数
