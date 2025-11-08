/**
 * @file position.cpp
 * @author HA Ji cao 
 * @brief position驱动文件
 * @attention 此文件用于position而非action
 * @date 2025-10-22
 */

/*

  Reposition_SendData函数用于重定位，id为1则仅重定位X,Y坐标；id2可以
  额外重定位yaw, id3可将imu断电重启
  
*/

// 联合体用于将20字节的浮点数接收到 float 数组中
#include "position.h"
#include <math.h>
#include "usbd_cdc_if.h"


#define NEW_OR_OLD 1


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
		

    uart_instance_ = InstanceManager::GetInstanceByUartHandle(uart_handle);;
    
    // 清空接收缓冲区
    memset(rx_buffer_, 0, RX_BUFFER_SIZE);
    
    __HAL_UART_ENABLE(uart_handle);
    // 初始化UART
    uart_instance_->UART_Init();
    
    uart_initialized_ = true;
}
uint8_t count = 0;

void Position::Callback_Fuc(uint8_t byte)
{
	 switch (state_)
    {
    case WAITING_FOR_HEADER_0:
        if (byte == FRAME_HEAD_POSITION_0)
        {
            state_ = WAITING_FOR_HEADER_1;
        }
        break;
    case WAITING_FOR_HEADER_1:
        if (byte == FRAME_HEAD_POSITION_1)
        {
            state_ = WAITING_FOR_ID;
        }
        else
        {
            state_ = WAITING_FOR_HEADER_0;
        }
        break;
    case WAITING_FOR_ID:
        state_ = WAITING_FOR_LENGTH;
        break;
    case WAITING_FOR_LENGTH:
        rx_frame_mat.data_length = byte; // 存储数据长度
        rxIndex_ = 0;
        state_ = WAITING_FOR_DATA;
        break;
		case WAITING_FOR_DATA:
        rx_frame_mat.data.buff_msg[rxIndex_++] = byte; // 存储接收到的数据
        if (rxIndex_ >= rx_frame_mat.data_length)
        {

            state_ = WAITING_FOR_CRC_0;
        }
        break;
    case WAITING_FOR_CRC_0:
        state_ = WAITING_FOR_CRC_1;
        break;
    case WAITING_FOR_CRC_1:
        state_ = WAITING_FOR_END_0;
        break;
    case WAITING_FOR_END_0:
        if (byte == FRAME_TAIL_POSITION_0)
        {
            state_ = WAITING_FOR_END_1;
        }
        else
        {
            state_ = WAITING_FOR_HEADER_0;
        }
        break;
    case WAITING_FOR_END_1:
        if (byte == FRAME_TAIL_POSITION_1)
        {
				Update_RawPosition(rx_frame_mat.data.msg_get);
        state_ = WAITING_FOR_HEADER_0;
        break;
				}
    default:
        state_ = WAITING_FOR_HEADER_0;
        break;
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



void Reposition_SendData(float X, float Y)
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