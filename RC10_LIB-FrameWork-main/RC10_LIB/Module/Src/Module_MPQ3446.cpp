#include "Module_MPQ3446.h"

MPQ3446::MPQ3446(uint8_t addr,I2C_HandleTypeDef* port)
{
	addr_=addr;
	port_=port;
}

uint8_t MPQ3446:: MPQ3446_WriteReg(MPQ3446_Reg reg_addr, uint16_t data)
{
	uint8_t tx_buff[3];
	tx_buff[0]=(uint8_t)reg_addr;
	tx_buff[1]=(data>>8)&0xFF;
	tx_buff[2]=(data)&0xFF;
	return HAL_I2C_Master_Transmit(port_,addr_,tx_buff,3,HAL_MAX_DELAY);
}
void MPQ3446:: SetUSER00(uint16_t SendData)
{
	reg_=USER00;
	state= MPQ3446_WriteReg(reg_, SendData);
}

void MPQ3446:: SetUSER01(uint16_t SendData)
{
	reg_=USER01;
	state= MPQ3446_WriteReg(reg_, SendData);
}

void MPQ3446:: SetUSER02(uint16_t SendData)
{
	reg_=USER02;
	state= MPQ3446_WriteReg(reg_, SendData);
}

void MPQ3446:: SetUSER03(uint16_t SendData)
{
	reg_=USER03;
	state= MPQ3446_WriteReg(reg_, SendData);
}

void MPQ3446:: SetSYSTEM00(uint16_t SendData)
{
	reg_=SYSTEM00;
	state= MPQ3446_WriteReg(reg_, SendData);
}

void MPQ3446::SetSYSTEM01(uint16_t SendData)
{
	reg_=SYSTEM01;
	state= MPQ3446_WriteReg(reg_, SendData);
}

void MPQ3446:: SetMASTER_SLAVE_KEY(uint16_t SendData)
{
	reg_=MASTER_SLAVE_KEY;
	state= MPQ3446_WriteReg(reg_, SendData);
}

void MPQ3446:: SetOTP_USER_RESERVE(uint16_t SendData)
{
	reg_=OTP_USER_RESERVE;
	state= MPQ3446_WriteReg(reg_, SendData);
}
uint8_t MPQ3446::MPQ3446_ReadReg(MPQ3446_Reg reg_addr, uint16_t* data)
{
	uint8_t rx_buf[2];
	uint8_t reg=(uint8_t)reg_addr;
	 if (HAL_I2C_Master_Transmit(port_,addr_, &reg, 1,HAL_MAX_DELAY) != 0) 
	 {
        return 0; // 发送失败
    }
	 if (HAL_I2C_Master_Receive(port_,addr_, rx_buf, 2,HAL_MAX_DELAY) != 0) 
	 {
        return 0; // 接收失败
    }
	*data = (rx_buf[0] << 8) | rx_buf[1]; // 拼接16位数据
    return 1;
}
