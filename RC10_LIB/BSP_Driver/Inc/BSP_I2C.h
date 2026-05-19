#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32h7xx_hal.h"
#include "tim.h"
#include "main.h"
#include "i2c.h"
#ifdef __cplusplus
}
#endif // __cplusplus

#ifdef __cplusplus

#include <cstdint>


class I2C_User 
{
	
	
	public:
	I2C_User(uint16_t addr,I2C_HandleTypeDef* i2c_handle);
	~I2C_User()=default;
	 HAL_StatusTypeDef I2C_ReadReg(uint8_t devaddr,uint8_t* pdata,uint8_t size);   // 读取寄存器
	 HAL_StatusTypeDef I2C_WriteReg(uint8_t devaddr,uint8_t* pdata,uint8_t size);  // 写入寄存器
	
	 HAL_StatusTypeDef readData(uint8_t *pdata,uint8_t size);      // 纯对数据
	 HAL_StatusTypeDef writeData();      // 纯写数据
	void setTimeout(uint32_t timeout) { timeout_ = timeout; }
	
	
	private:
	
	uint16_t addr_;
	I2C_HandleTypeDef* i2c_handle_;
	uint32_t timeout_=20;
	
};










#endif // __cplusplus

#endif // __BSP_I2C_H