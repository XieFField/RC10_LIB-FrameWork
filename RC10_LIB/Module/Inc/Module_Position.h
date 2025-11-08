/**
 * @file position.h
 * @author WU Jia Zhuang Ji cao
 * @brief position驱动文件
 * @attention 此文件用于position而非action
 */

#ifndef POSITION_H
#define POSITION_H


#ifdef __cplusplus
extern "C" {
#endif 

#include "usart.h"
#include <stdint.h>
#include "math.h"
#include "BSP_USB_UART_Driver.h"

#define MAX_DATA_LENGTH 64//接收数据最大大小
	
#define PI 3.14159265358979f
#define FRAME_HEAD_POSITION_0 0xfc  //包头
#define FRAME_HEAD_POSITION_1 0xfb

#define FRAME_TAIL_POSITION_0 0xfd  //包尾
#define FRAME_TAIL_POSITION_1 0xfe

#define INSTALL_ERROR_X		0.0     //安装误差
#define INSTALL_ERROR_Y		0.209
#define RX_BUFFER_SIZE 1  //UART每次接收一个数据

// 全局变量
extern uint8_t rx_buffer[RX_BUFFER_SIZE];


void Reposition_SendData(float X, float Y);
void POS_Relocate_ByDiff(float X, float Y, float yaw);
void Update_RawPosition(float value[5]);
typedef struct RealPos  //处理后
{
  float world_x;
  float world_y;     
  float world_yaw;

	float dx;
	float dy;
	float dyaw;

}RealPos;

typedef struct RawPos   //处理前
{
	float angle_Z;
	float Pos_X;
	float Pos_Y;
	float Speed_X;
	float Speed_Y;
	
	float Speed_Yaw;

	float LAST_Pos_X;
	float LAST_Pos_Y;

	float DELTA_Pos_X;
	float DELTA_Pos_Y;
	
	float REAL_X;
	float REAL_Y;
}RawPos;
//储存数据帧
typedef struct serial_frame_mat
{
    uint8_t data_length = 0; // 数据载荷的字节数
    uint16_t crc_calculated = 0;
    uint8_t rx_temp_data_mat[MAX_DATA_LENGTH];
    union data
    {
			  uint8_t buff_msg[MAX_DATA_LENGTH];           // 用于字节流的接收
        float msg_get[MAX_DATA_LENGTH / 4] = {0.0f}; // 用于浮点数的接收 
    } data;
} serial_frame_mat_t;



#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Position:public UART_{
public:
    // 获取单例实例
    static Position* GetInstance(UART_HandleTypeDef *uart_handle);
    serial_frame_mat_t rx_frame_mat;
    // 初始化UART
    void InitUART();
		//重写虚函数.cpp
    void Callback_Fuc(uint8_t byte) override;
    // 删除拷贝构造函数和赋值运算符
    Position(const Position&) = delete;
    Position& operator=(const Position&) = delete;
		RealPos get_position() const { return RealPosData;}
		void Update_RawPosition(float value[5]);
private:
    Position(uint16_t rx_buffer_size,uint8_t *rx_buffer,UART_HandleTypeDef *uart_handle); // 私有构造函数
    ~Position() = default;
     uint8_t rxIndex_; // 当前接收到的字节的索引
    // UART实例
    UART_* uart_instance_;
		RawPos RawPosData = {0};
		RealPos RealPosData = {0};
    enum rxState
    {
        WAITING_FOR_HEADER_0,
        WAITING_FOR_HEADER_1,
        WAITING_FOR_ID,
        WAITING_FOR_LENGTH,
        WAITING_FOR_DATA,
        WAITING_FOR_CRC_0,
        WAITING_FOR_CRC_1,
        WAITING_FOR_END_0,
        WAITING_FOR_END_1
    } state_;
    
    // 初始化标志
    bool uart_initialized_;

		uint8_t rx_buffer_[RX_BUFFER_SIZE];
};

#endif // __cplusplus

#endif
