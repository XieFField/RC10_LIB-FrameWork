#ifndef __MODULE_MPQ3446_H
#define __MODULE_MPQ3446_H
#ifdef __cplusplus
extern "C" {
#endif
#include "arm_math.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
#include "main.h"
#include "i2c.h"
#endif __cplusplus

#ifdef __cplusplus
typedef enum {
    USER00       = 0x00,
    USER01       = 0x01,
    USER02       = 0x02,
    USER03       = 0x03,
    SYSTEM00     = 0x06,
    SYSTEM01     = 0x07,
    MASTER_SLAVE_KEY = 0x08,
    OTP_USER_RESERVE = 0x1E
} MPQ3446_Reg;




class MPQ3446
{
public:
	MPQ3446(uint8_t addr,I2C_HandleTypeDef* port);
	~MPQ3446(){}
	uint8_t MPQ3446_WriteReg(MPQ3446_Reg reg_addr, uint16_t data);
	void SetUSER00(uint16_t SendData);
	void SetUSER01(uint16_t SendData);
	void SetUSER02(uint16_t SendData);
	void SetUSER03(uint16_t SendData);
	void SetSYSTEM00(uint16_t SendData);
	void SetSYSTEM01(uint16_t SendData);
	void SetMASTER_SLAVE_KEY(uint16_t SendData);
	void SetOTP_USER_RESERVE(uint16_t SendData);
	uint8_t MPQ3446_ReadReg(MPQ3446_Reg reg_addr, uint16_t* data);	
private:
	uint8_t addr_;
	I2C_HandleTypeDef* port_;
	MPQ3446_Reg reg_;
	uint8_t state;
};






#endif 

#endif 