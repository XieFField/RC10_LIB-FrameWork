/**
 * @file Module_Position.cpp
 * @author XieFField HA Ji cao 
 * @brief position驱动文件
 * @attention 此文件用于position而非action
 * @date 2025-10-22
 */

/*

  Reposition_SendData函数用于重定位，id为1则仅重定位X,Y坐标；id2可以
  额外重定位yaw, id3可将imu断电重启
  
*/
#include "Module_Position.h"
// 联合体用于将20字节的浮点数接收到 float 数组中
#include <math.h>
#include "usbd_cdc_if.h"


#define NEW_OR_OLD 1

union
{
	uint8_t data[24];
	float ActVal[6];
} posture;

// 获取单例实例
//Position* Position::instance_ = nullptr;

// 获取单例实例


Position::Position(uint16_t rx_buffer_size,uint8_t *rx_buffer,UART_HandleTypeDef *uart_handle) 
    :UART_(rx_buffer_size,rx_buffer,uart_handle),
		uart_instance_(nullptr)
    , uart_initialized_(false)
    , rx_buffer_{0}
{
}

Position* Position::GetInstance(UART_HandleTypeDef *uart_handle) 
{
	  static uint8_t static_rx_buffer[RX_BUFFER_SIZE] = {0};
	  static Position instance(RX_BUFFER_SIZE,static_rx_buffer,uart_handle);
    return &instance;
}

// 初始化UART
void Position::InitUART() 
{
    if (uart_initialized_) {
        return; // 已经初始化过
    }
    UART_HandleTypeDef *uart_handle=Position::UART_::GetUartHandle();

    uart_instance_ = InstanceManager::GetInstanceByUartHandle(uart_handle);
    
    // 初始化UART
    uart_instance_->UART_Init();
    
    uart_initialized_ = true;
}

void Position::Callback_Fuc(uint8_t *buf, uint16_t len)
{
  uint8_t count = 0;
	uint8_t i = 0;
	uint8_t CRC_check[2];//CRC校验位，此文件未启用
	
	
	
	uint8_t break_flag = 1;
	while(i < len && break_flag == 1)
	{
		switch (count)
		{
			case 0:
			{
				if (buf[i] == FRAME_HEAD_POSITION_0)   //接收包头1
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			case 1:
			{
				if (buf[i] == FRAME_HEAD_POSITION_1) //接收包头2
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			case 2://接收帧ID和数据长度
			{
				if (buf[i] == 0x01) 
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			case 3:
			{
				if (buf[i] == 0x18) //0x0c
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			case 4://开始接收数据
			{
				uint8_t j;
				
				#if NEW_OR_OLD
				if (i > len - 24)
				{
					break_flag = 0;
					break;
				}
				
				for(j = 0; j < 24; j++)
				{
					posture.data[j] = buf[i];
					i++;
				}
                
                #else
                if (i > len - 24)
				{
					break_flag = 0;
				}
				
				for(j = 0; j < 20; j++)
				{
					posture.data[j] = buf[i];
					i++;
				}
                
                #endif
				count++;
				break;
			}
			
			//接收CRC校验码
			case 5:
			{
				uint8_t j;
				
				for(j = 0; j < 2; j++)
				{
					CRC_check[j] = buf[i];
					i++;
				}
				count++;
				break;
			}
			
			case 6:
			{
				if (buf[i] == FRAME_TAIL_POSITION_0)  //接收包尾1
				{
					count++;
				}
				else
				{
					count = 0;
				}
				i++;
				break;
			}
			
			case 7:
			{
				if (buf[i] == FRAME_TAIL_POSITION_1)  //接收包尾2
				{	
					//在接收包尾2后才开始启动回调
					//UART_IdleCallback(&huart1);
					Update_RawPosition(posture.ActVal);
				}
				count = 0;
				
				break_flag = 0;
				
				break;
			}
			
			default:
			{
				count = 0;
				break;
			}
		}
		
	}
	
}


// 数据更新函数：将解析后的值存入 RawPos 和 RealPos
void Position::Update_RawPosition(float value[5])
{
	RawPosData.Pos_X = value[0] / 1000.f; 
	RawPosData.Pos_Y = value[1] / 1000.f; 
	RawPosData.angle_Z = value[2];
	RawPosData.Speed_Yaw = value[3];
	RawPosData.Speed_Y = value[4];

   //世界坐标
	RealPosData.world_yaw = RawPosData.angle_Z;
    RealPosData.world_x   =  RawPosData.Pos_X + RealPosData.dx;
	RealPosData.world_y   =  RawPosData.Pos_Y + RealPosData.dy;

	RealPosData.dyaw = RawPosData.Speed_Yaw;

}



void Position::Reposition_SendData(float X, float Y)
{
	uint8_t txBuffer[16] = {0};

	union
	{
        float f;
        uint8_t bytes[4];
    } floatUnion;

	//包头
	txBuffer[0] = FRAME_HEAD_POSITION_0;
	txBuffer[1] = FRAME_HEAD_POSITION_1;
    txBuffer[2]=0x01;

	//数据长度
	txBuffer[3] = 0x08;

	//数据
	floatUnion.f = X;
	txBuffer[4] = floatUnion.bytes[0];
    txBuffer[5] = floatUnion.bytes[1];
    txBuffer[6] = floatUnion.bytes[2];
    txBuffer[7] = floatUnion.bytes[3];

    floatUnion.f = Y;
    txBuffer[8] = floatUnion.bytes[0];
    txBuffer[9] = floatUnion.bytes[1];
    txBuffer[10] = floatUnion.bytes[2];
    txBuffer[11] = floatUnion.bytes[3];

	//CRC
	txBuffer[12] = 0;
	txBuffer[13] = 0;
	//包尾
	txBuffer[14] = FRAME_TAIL_POSITION_0;
	txBuffer[15] = FRAME_TAIL_POSITION_1;

	HAL_UART_Transmit(&huart1, txBuffer, 16, HAL_MAX_DELAY);
}


/*调试USB用的
void USB_DataReceivedCallback(uint8_t* buf, uint16_t len)
{
    // 接收到数据后立即回传（echo功能）
    if(len > 0 && len <= RX_BUFFER_SIZE)
    {
        CDC_Transmit_HS(buf, len);
    }
}*/