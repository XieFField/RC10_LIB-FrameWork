#include "BSP_I2C.h"

I2C_User::I2C_User(uint16_t addr,I2C_HandleTypeDef* i2c_handle)
{
	addr_=(addr<<1);
	i2c_handle_=i2c_handle;
}

HAL_StatusTypeDef I2C_User::I2C_ReadReg(uint8_t devaddr,uint8_t* pdata,uint8_t size)
{
	
	return HAL_I2C_Mem_Read(i2c_handle_,addr_,devaddr,I2C_MEMADD_SIZE_8BIT,pdata,size,timeout_);
}

HAL_StatusTypeDef I2C_User::I2C_WriteReg(uint8_t devaddr,uint8_t* pdata,uint8_t size)
{
	return HAL_I2C_Mem_Write(i2c_handle_,addr_,devaddr,I2C_MEMADD_SIZE_8BIT,pdata,size,timeout_);
}

HAL_StatusTypeDef I2C_User::readData(uint8_t *pdata,uint8_t size)
{
	 return HAL_I2C_Master_Receive(i2c_handle_, addr_, pdata, size, timeout_);
}

HAL_StatusTypeDef I2C_User::writeData(uint8_t *pdata,uint8_t size)
{
	return HAL_I2C_Master_Transmit(i2c_handle_, addr_, pdata, size, timeout_);
}
/*-----------------------------------´ı²âÊÔ---------------------------------------------*/
void I2C_User::Scanf_addr()
{
	 for(uint8_t i = 0; i < 128; i++)
    {
     addr_statusm[i] = HAL_I2C_IsDeviceReady(i2c_handle_, i << 1, 1, 10);	
//		if(addr_statusm[i]==HAL_OK)
//		{
//			addr_=(i<<1);
//		}
	}
}