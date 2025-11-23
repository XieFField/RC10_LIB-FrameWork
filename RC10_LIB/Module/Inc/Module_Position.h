/**
 * @file Module_Position.h
 * @author XieFField HA Ji cao
 * @brief position????
 * @attention ?????position??action
 */

#ifndef MODULE_POSITION_H
#define MODULE_POSITION_H


#ifdef __cplusplus
extern "C" {
#endif 

#include "usart.h"
#include <stdint.h>
#include "math.h"
#include "BSP_USB_UART_Driver.h"
	
	
#define PI 3.14159265358979f
#define FRAME_HEAD_POSITION_0 0xfc  //??
#define FRAME_HEAD_POSITION_1 0xfb

#define FRAME_TAIL_POSITION_0 0xfd  //??
#define FRAME_TAIL_POSITION_1 0xfe

#define INSTALL_ERROR_X		0.0     //????
#define INSTALL_ERROR_Y		0.209

typedef struct RealPos  //???
{
  float world_x;
  float world_y;     
  float world_yaw;

	float dx;
	float dy;
	float dyaw;

}RealPos;


typedef struct RawPos   //???
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

#define RX_BUFFER_SIZE 64     

// ????
extern uint8_t rx_buffer[RX_BUFFER_SIZE];
extern RealPos RealPosData;




#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Position:public UART_{
public:
    // ??????
    static Position* GetInstance(UART_HandleTypeDef *uart_handle);
    
    // ???UART
    void InitUART();
		//?????.cpp
    void Callback_Fuc(uint8_t *buf, uint16_t len) override;
    // ??????????????
    Position(const Position&) = delete;
    Position& operator=(const Position&) = delete;

    void Reposition_SendData(float X, float Y);

  RealPos getRealPosData() const { return RealPosData; }

private:
    Position(uint16_t rx_buffer_size,uint8_t *rx_buffer,UART_HandleTypeDef *uart_handle); // ??????
    ~Position() = default;
    
    void Update_RawPosition(float value[5]);

    // UART??
    UART_* uart_instance_;
    RealPos    RealPosData;
    RawPos   RawPosData;
    
    // ?????
    bool uart_initialized_;

		uint8_t rx_buffer_[RX_BUFFER_SIZE];
};

#endif // __cplusplus

#endif