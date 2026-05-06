#ifndef MODULE_JY61_H
#define MODULE_JY61_H



extern "C"
{
	
}

#include "BSP_USB_UART_Driver.h"
#include "APP_Vector2D.h"
#include "usart.h"
#include <stdint.h>
#include "math.h"
#include "BSP_I2C.h"
#include "BSP_RTOS.h"
#include "i2c.h"
#include "APP_debugTool.h"
#include "Locate_setup.h"
#ifdef __cplusplus
#define ACCEL_RANGE  16.0f * 9.80665f
#define GYRO_RANGE  2000.0f
	
#define REG_ACC_GYRO_START 0x34
#define REG_ANGLE_START 0x3D

#define JY61_ADDR 0x50

typedef struct 
{
	float ax ,ay ,az;
	float gx ,gy ,gz;
	float roll,pitch,yaw;
}JY61_data;


class JY61_IMU :public I2C_User
{
	
	public:
	JY61_IMU(uint16_t addr,I2C_HandleTypeDef* i2c_handle);
	~JY61_IMU(){};
	static inline int16_t u8ToS16(uint8_t lo, uint8_t hi) {
		return (int16_t)((hi << 8) | lo);
}
	bool jy61ReadAngles(JY61_data &d);
	bool jy61ReadAccGyro(JY61_data &d);
	JY61_data GetImuData(){return JY61_data_;}
	
	private:
	JY61_data JY61_data_;
	uint8_t angle_buf[6];
	uint8_t gyro_buf[12];
	HAL_StatusTypeDef status;
	
};



class IMU_test :public RtosTask
{
public:
	IMU_test():RtosTask("IMU_Test",50){};
	void init()
	{
		this->start(osPriorityNormal, 256);
	}
	void loop()override;
private:
	uint32_t errorcode;
	uint32_t test_addr;
	uint8_t buf[6];
	HAL_StatusTypeDef statusm[128];

	Debug_Printf debug_uart = Debug_Printf(&huart8);
	
};

#endif
#endif