#include "Module_JY61.h"

JY61_IMU::JY61_IMU(uint16_t addr,I2C_HandleTypeDef* i2c_handle)
	:I2C_User(addr,i2c_handle){}
		
bool JY61_IMU::jy61ReadAngles(JY61_data &d)
{
//	 uint8_t buf[6];
	if (I2C_ReadReg(REG_ANGLE_START, angle_buf,6)!=HAL_OK) return false;
	d.roll = (float)u8ToS16(angle_buf[0],angle_buf[1]) / 32768.0f * 180.0f;
	d.pitch = (float)u8ToS16(angle_buf[2], angle_buf[3]) / 32768.0f * 180.0f;
	d.yaw = (float)u8ToS16(angle_buf[4], angle_buf[5]) / 32768.0f * 180.0f;
	return true;
}

bool JY61_IMU::jy61ReadAccGyro(JY61_data &d)
{
//  uint8_t buf[12];
	if (I2C_ReadReg(REG_ACC_GYRO_START, gyro_buf,12)!=HAL_OK) return false;
  int16_t axRaw = u8ToS16(gyro_buf[0], gyro_buf[1]);
  int16_t ayRaw = u8ToS16(gyro_buf[2], gyro_buf[3]);
  int16_t azRaw = u8ToS16(gyro_buf[4], gyro_buf[5]);
  int16_t gxRaw = u8ToS16(gyro_buf[6], gyro_buf[7]);
  int16_t gyRaw = u8ToS16(gyro_buf[8], gyro_buf[9]);
  int16_t gzRaw = u8ToS16(gyro_buf[10], gyro_buf[11]);
  d.ax = (float)axRaw / 32768.0f * ACCEL_RANGE;
  d.ay = (float)ayRaw / 32768.0f * ACCEL_RANGE;
  d.az = (float)azRaw / 32768.0f * ACCEL_RANGE;
  d.gx = (float)gxRaw / 32768.0f * GYRO_RANGE;
  d.gy = (float)gyRaw / 32768.0f * GYRO_RANGE;
  d.gz = (float)gzRaw / 32768.0f * GYRO_RANGE;
  return true;
}




	
	
/*-----------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------*/


bool ok1,ok2;
JY61_IMU imu(JY61_ADDR,&hi2c5); 
 JY61_data imu_data;
HAL_StatusTypeDef ok3;
uint8_t reg=REG_ANGLE_START;
void IMU_test:: loop()
{
	  HAL_StatusTypeDef status;
  
	// 注：devAddr需左移1位（HAL库要求7位地址<<1，硬件自动处理读写位）
  ok1 = HAL_I2C_Master_Transmit(&hi2c5, (JY61_ADDR << 1), &reg, 1, 100);

  osDelay(10);
  // 步骤2：从JY61读取指定长度的数据
  ok2 = HAL_I2C_Master_Receive(&hi2c5, (JY61_ADDR << 1), buf,6, 100);

  


  ok1=imu.jy61ReadAngles(imu_data);
  ok2=imu.jy61ReadAccGyro(imu_data);
  if (!ok1 || !ok2)
  {
    errorcode=hi2c5.ErrorCode;
    ok3 = HAL_I2C_Master_Transmit(&hi2c5, (JY61_ADDR << 1), &reg, 1, 10);
  }
  for(uint8_t i = 1; i < 127; i++)
    {
      statusm[i] = HAL_I2C_IsDeviceReady(&hi2c5, i << 1, 1, 10); 
  }
}
	
	
	
	