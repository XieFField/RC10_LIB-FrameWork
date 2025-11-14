/**
 * @file Module_Position.cpp
 * @author XieFField HA Ji cao 
 * @brief position�����ļ�
 * @attention ���ļ�����position����action
 * @date 2025-10-22
 */

/*

  Reposition_SendData���������ض�λ��idΪ1����ض�λX,Y���ꣻid2����
  �����ض�λyaw, id3�ɽ�imu�ϵ�����
  
*/
#include "Module_Position.h"
// ���������ڽ�20�ֽڵĸ��������յ� float ������
#include <math.h>
#include "usbd_cdc_if.h"


#define NEW_OR_OLD 1

union
{
	uint8_t data[24];
	float ActVal[6];
} posture;

// ��ȡ����ʵ��
//Position* Position::instance_ = nullptr;

// ��ȡ����ʵ��

RealPos RealPosData;

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

// ��ʼ��UART
void Position::InitUART() 
{
    if (uart_initialized_) {
        return; // �Ѿ���ʼ����
    }
    UART_HandleTypeDef *uart_handle=Position::UART_::GetUartHandle();

    uart_instance_ = InstanceManager::GetInstanceByUartHandle(uart_handle);
    
    // ��ʼ��UART
    uart_instance_->UART_Init();
    
    uart_initialized_ = true;
}

void Position::Callback_Fuc(uint8_t *buf, uint16_t len)
{
    uint8_t count = 0;
	uint8_t i = 0;
	uint8_t CRC_check[2];//CRCУ��λ�����ļ�δ����
	
	
	
	uint8_t break_flag = 1;
	while(i < len && break_flag == 1)
	{
		switch (count)
		{
			case 0:
			{
				if (buf[i] == FRAME_HEAD_POSITION_0)   //���հ�ͷ1
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
				if (buf[i] == FRAME_HEAD_POSITION_1) //���հ�ͷ2
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
			case 2://����֡ID�����ݳ���
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
			case 4://��ʼ��������
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
			
			//����CRCУ����
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
				if (buf[i] == FRAME_TAIL_POSITION_0)  //���հ�β1
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
				if (buf[i] == FRAME_TAIL_POSITION_1)  //���հ�β2
				{	
					//�ڽ��հ�β2��ſ�ʼ�����ص�
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


// ���ݸ��º��������������ֵ���� RawPos �� RealPos
void Position::Update_RawPosition(float value[5])
{
	RawPosData.Pos_X = value[0] / 1000.f; 
	RawPosData.Pos_Y = value[1] / 1000.f; 
	RawPosData.angle_Z = value[2];
	RawPosData.Speed_Yaw = value[3];
	RawPosData.Speed_Y = value[4];

   //��������
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

	//��ͷ
	txBuffer[0] = FRAME_HEAD_POSITION_0;
	txBuffer[1] = FRAME_HEAD_POSITION_1;
    txBuffer[2]=0x01;

	//���ݳ���
	txBuffer[3] = 0x08;

	//����
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
	//��β
	txBuffer[14] = FRAME_TAIL_POSITION_0;
	txBuffer[15] = FRAME_TAIL_POSITION_1;

	HAL_UART_Transmit(&huart1, txBuffer, 16, HAL_MAX_DELAY);
}


/*����USB�õ�
void USB_DataReceivedCallback(uint8_t* buf, uint16_t len)
{
    // ���յ����ݺ������ش���echo���ܣ�
    if(len > 0 && len <= RX_BUFFER_SIZE)
    {
        CDC_Transmit_HS(buf, len);
    }
}*/