#ifndef MODULE_HBRIDGE_H
#define MODULE_HBRIDGE_H



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
#include "GPIO.h"
#include "BSP_TimeStamp.h"

#ifdef __cplusplus

#define DRV8245_EN_GPIO_Port    GPIOG
#define DRV8245_EN_Pin          GPIO_PIN_3   //EN/IN1

#define DRV8245_PH_GPIO_Port    GPIOG
#define DRV8245_PH_Pin          GPIO_PIN_4   //PH/IN2

#define DRV8245_DRVOFF_GPIO_Port GPIOG
#define DRV8245_DRVOFF_Pin       GPIO_PIN_5  //DRVOFF

#define DRV8245_nSLEEP_GPIO_Port GPIOG
#define DRV8245_nSLEEP_Pin       GPIO_PIN_6  //nSleep

#define DRV8245_nFAULT_GPIO_Port GPIOG
#define DRV8245_nFAULT_Pin       GPIO_PIN_7  //nFault

typedef enum {
    DRV8245_OK        = 0,
    DRV8245_FAULT     = 1,    // 通用故障
    DRV8245_OUT1_ERR  = 2,    // OUT1 开路/短路
    DRV8245_OUT2_ERR  = 3,    // OUT2 开路/短路
    DRV8245_VM_SHORT  = 4,    // 输出对电源短路
    DRV8245_NO_FAULT  = 5
} DRV8245_StatusTypeDef;


typedef struct
{	
	bool init_flag=0;
	bool weak_init_flag=0;
	bool is_waiting_flag=0;
	
}Runing_flag;

typedef struct
{	
	uint64_t current_time;
	uint64_t plus_start_time;
}Time_Stamp;




class H_Bridge
{
public:
	H_Bridge(GPIO_TypeDef* en_port,uint16_t en_pin,GPIO_TypeDef* ph_port,uint16_t ph_pin,GPIO_TypeDef* drvoff_port,uint16_t drvoff_pin,
			 GPIO_TypeDef* nsleep_port, uint16_t nsleep_pin ,GPIO_TypeDef* nfault_port, uint16_t nfault_pin);
void DRV8245_Init(void);

// 进入待机模式 (用于 OLP 诊断)
void DRV8245_EnterStandby(void);

// 进入运行模式 (正常驱动电机)
void DRV8245_EnterRunMode(void);

// 电机正转
void DRV8245_Forward(void);

// 电机反转
void DRV8245_Backward(void);

// 电机停止 (高阻)
void DRV8245_Stop(void);

// 读取故障引脚状态
void DRV8245_ReadFault(void);

// 故障复位 (nSLEEP 脉冲)
void DRV8245_ResetFault(void);

// OLP 关断状态诊断 (必须在待机模式调用)
void DRV8245_OLP_Diagnosis(void);

uint8_t Get_ErrorState(){return Error_State;}
DRV8245_StatusTypeDef Get_ErrorNum(){ return Error_num;}

private:
	uint8_t Error_State=1;
	DRV8245_StatusTypeDef Error_num=DRV8245_NO_FAULT;
	Time_Stamp time_stamp;
	Runing_flag running_flag;


	GPIO_TypeDef* drv8245_en_gpio_Port_;
	uint16_t drv8245_en_pin_;
	GPIO_TypeDef* drv8245_ph_gpio_port_;
	uint16_t drv8245_ph_pin_;
	GPIO_TypeDef* drv8245_drvoff_gpio_port_;
	uint16_t drv8245_drvoff_pin_;
	GPIO_TypeDef* drv8245_nsleep_gpio_port_;
	uint16_t drv8245_nsleep_pin_ ;
	GPIO_TypeDef*  drv8245_nfault_gpio_port_;
	uint16_t drv8245_nfault_pin_ ;

};
	





class H_Bridge_test :public RtosTask 
{
	public:
	H_Bridge_test():RtosTask("H_Bridge_Test",1){};
	void init()
	{
		this->start(osPriorityNormal, 256);
	}
	
	void loop()override;
	private:

	
};




		
	


#endif
#endif