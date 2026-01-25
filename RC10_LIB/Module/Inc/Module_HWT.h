/**
 * @file Locate_Setup.h
 * @brief 定位 主要是雷达接收 位姿变换 激光重定位等功能
 * @author XieFField    HaJiCao
 */
#ifndef MODULE_HWT_H
#define MODULE_HWT_H



#ifdef __cplusplus
extern "C"
{
#endif
#include "BSP_USB_UART_Driver.h"
#include "APP_Vector2D.h"
#include "usart.h"
#include <stdint.h>
#include "math.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#define FRAME_HEADER_1 0x55
#define FRAME_HEADER_2 0x53
typedef struct
{
    uint8_t reserved[4]; // 前4个保留字节
    uint8_t YawL;
    uint8_t YawH;
    uint8_t VL;
    uint8_t VH;
    uint8_t checksum;
} HWT101CT_Frame_t;

typedef enum RxState
{
		WAITING_FOR_HEADER_1,
		WAITING_FOR_HEADER_2,
		WAITING_FOR_RESERVED_1,
		WAITING_FOR_RESERVED_2,
		WAITING_FOR_RESERVED_3,
		WAITING_FOR_RESERVED_4,
		WAITING_FOR_YAWL,
		WAITING_FOR_YAWH,
		WAITING_FOR_VL,
		WAITING_FOR_VH,
		WAITING_FOR_CHECKSUM
}RxState;

class HWT101CT:public UART_
{

public:
			static HWT101CT* GetInstance(UART_HandleTypeDef *uart_handle);
     void InitUART();
		//?????.cpp
     void Callback_Fuc(uint8_t *buf, uint16_t len) override;
    // ??????????????
     HWT101CT(const HWT101CT&) = delete;
     HWT101CT& operator=(const HWT101CT&) = delete;
      float get_heading();
      float get_yaw_rad();//获取角度弧度制
      float get_yaw_speed_rad();//获取角速度
      void handleReceiveData(uint8_t byte);
      void processDecodedData(float yaw);
      HWT101CT(UART_HandleTypeDef *huart_);
      void add_io(GPIO_TypeDef *port, uint16_t pin);
      void process_data();
      void imu_rst();
      void imu_reset_heading(float reheading);
      uint32_t get_update_time();
      
private:
	   HWT101CT_Frame_t frame;
	   RxState rx_state=WAITING_FOR_HEADER_1;
	   HWT101CT(uint16_t rx_buffer_size,uint8_t *rx_buffer,UART_HandleTypeDef *uart_handle); 
    ~HWT101CT() = default;
     UART_* uart_instance_;
     bool uart_initialized_;
		 uint8_t rx_buffer_[64];
     uint8_t reserved_index=0;
     uint8_t calculated_checksum=0, init_count = 0;
     float orin_yaw = 0.0f, init_yaw = 0.0f, delta_angle = 0.0f, real_yaw = 0.0f, yaw_rad = 0.0f;
     float calculateYaw(uint8_t YawH, uint8_t YawL);
     uint8_t calculateChecksum();
     void yaw_tf(float nowyaw);
     bool if_init = true;
     uint32_t last_update_time = 0, now_time = 0;
     float delta_time = 0.0f, yaw_speed_rad = 0.0f, last_yaw = 0.0f;
};

#endif
















#endif