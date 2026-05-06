#include "Module_HBridge.h"

H_Bridge::H_Bridge(GPIO_TypeDef* en_port,uint16_t en_pin,GPIO_TypeDef* ph_port,uint16_t ph_pin,GPIO_TypeDef* drvoff_port,
				  uint16_t drvoff_pin,GPIO_TypeDef* nsleep_port, uint16_t nsleep_pin ,GPIO_TypeDef* nfault_port, uint16_t nfault_pin)
{
	drv8245_en_gpio_Port_=en_port;
	drv8245_en_pin_=en_pin;
	drv8245_ph_gpio_port_=ph_port;
	drv8245_ph_pin_=ph_pin;
	drv8245_drvoff_gpio_port_=drvoff_port;
	drv8245_drvoff_pin_=drvoff_pin;
	drv8245_nsleep_gpio_port_=nsleep_port;
	drv8245_nsleep_pin_ =nsleep_pin;
	drv8245_nfault_gpio_port_=nfault_port;
	drv8245_nfault_pin_ =nfault_pin;
}

void H_Bridge::DRV8245_Init()
{
	if(!running_flag.init_flag)
	{ 
		if(!running_flag.is_waiting_flag)
		{
			osDelay(1500);
			running_flag.is_waiting_flag=1;
		}
		HAL_GPIO_WritePin(DRV8245_nSLEEP_GPIO_Port, DRV8245_nSLEEP_Pin, GPIO_PIN_RESET);
	  if(!running_flag.weak_init_flag)
	  {
		
		time_stamp.plus_start_time=TimeStamp::getInstance().getMicroseconds();
		  running_flag.weak_init_flag=1;
	  }
	  while(TimeStamp::getInstance().getMicroseconds()-time_stamp.plus_start_time<=30)
	  {
		  
	  }
	
		
	//	  DRV8245_EnterStandby();
		HAL_GPIO_WritePin(DRV8245_nSLEEP_GPIO_Port, DRV8245_nSLEEP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DRV8245_DRVOFF_GPIO_Port, DRV8245_DRVOFF_Pin, GPIO_PIN_RESET);
	    osDelay(2);  // µÈ´ýÐ¾Æ¬Æô¶¯
		running_flag.init_flag=1;
	  

	}

}

void H_Bridge::DRV8245_EnterStandby()
{
    HAL_GPIO_WritePin(DRV8245_DRVOFF_GPIO_Port, DRV8245_DRVOFF_Pin, GPIO_PIN_SET);
}

void H_Bridge::DRV8245_EnterRunMode()
{
    HAL_GPIO_WritePin(DRV8245_DRVOFF_GPIO_Port, DRV8245_DRVOFF_Pin, GPIO_PIN_RESET);
}


void H_Bridge::DRV8245_Forward(void)
{
    DRV8245_EnterRunMode();
    HAL_GPIO_WritePin(DRV8245_PH_GPIO_Port, DRV8245_PH_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DRV8245_EN_GPIO_Port, DRV8245_EN_Pin, GPIO_PIN_SET);
}


void H_Bridge::DRV8245_Backward(void)
{
    DRV8245_EnterRunMode();
    HAL_GPIO_WritePin(DRV8245_PH_GPIO_Port, DRV8245_PH_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV8245_EN_GPIO_Port, DRV8245_EN_Pin, GPIO_PIN_SET);
}

void H_Bridge::DRV8245_Stop(void)
{
    HAL_GPIO_WritePin(DRV8245_EN_GPIO_Port, DRV8245_EN_Pin, GPIO_PIN_RESET);
}


void H_Bridge::DRV8245_ReadFault(void)
{
    if(HAL_GPIO_ReadPin(DRV8245_nFAULT_GPIO_Port, DRV8245_nFAULT_Pin) == GPIO_PIN_RESET)
    {
        Error_State= 1; // ¹ÊÕÏ
    }
	if(HAL_GPIO_ReadPin(DRV8245_nFAULT_GPIO_Port, DRV8245_nFAULT_Pin) == GPIO_PIN_SET)
    {
		Error_State= 0; // Õý³£
	}else
	{
		Error_State=3;
	}
}

void H_Bridge::DRV8245_ResetFault(void)
{
    HAL_GPIO_WritePin(DRV8245_nSLEEP_GPIO_Port, DRV8245_nSLEEP_Pin, GPIO_PIN_RESET);
    osDelay(1); 
    HAL_GPIO_WritePin(DRV8245_nSLEEP_GPIO_Port, DRV8245_nSLEEP_Pin, GPIO_PIN_SET);
    osDelay(2);
}

void H_Bridge::DRV8245_OLP_Diagnosis(void)
{
    DRV8245_EnterStandby();
    DRV8245_Stop();
    osDelay(1);

    // ²½Öè1£º¼ì²â OUT1
    HAL_GPIO_WritePin(DRV8245_EN_GPIO_Port, DRV8245_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DRV8245_PH_GPIO_Port, DRV8245_PH_Pin, GPIO_PIN_RESET);
    osDelay(1);
    if(Error_State == 1)    Error_num=DRV8245_OUT1_ERR;

    // ²½Öè2£º¼ì²â OUT2
    HAL_GPIO_WritePin(DRV8245_EN_GPIO_Port, DRV8245_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV8245_PH_GPIO_Port, DRV8245_PH_Pin, GPIO_PIN_SET);
    osDelay(1);
    if(Error_State == 1)    Error_num= DRV8245_OUT2_ERR;

    // ²½Öè3£º¼ì²â¶Ô VM ¶ÌÂ·
    HAL_GPIO_WritePin(DRV8245_EN_GPIO_Port, DRV8245_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DRV8245_PH_GPIO_Port, DRV8245_PH_Pin, GPIO_PIN_SET);
    osDelay(1);
    if(Error_State== 1)     Error_num= DRV8245_VM_SHORT;

    Error_num= DRV8245_NO_FAULT;
}

/*---------------------------------------------------------------------------------------------------*/ //test

H_Bridge h_bridge(DRV8245_EN_GPIO_Port,DRV8245_EN_Pin,DRV8245_PH_GPIO_Port,DRV8245_PH_Pin ,DRV8245_DRVOFF_GPIO_Port,
				DRV8245_DRVOFF_Pin,DRV8245_nSLEEP_GPIO_Port,DRV8245_nSLEEP_Pin ,DRV8245_nFAULT_GPIO_Port,DRV8245_nFAULT_Pin );
uint8_t state;


void H_Bridge_test::loop()
{
	
//	  HAL_GPIO_WritePin(DRV8245_nSLEEP_GPIO_Port, DRV8245_nSLEEP_Pin, GPIO_PIN_SET);
		h_bridge.DRV8245_Init();
	
	h_bridge.DRV8245_ReadFault();
//	h_bridge.DRV8245_OLP_Diagnosis();

	switch(state)
	{
		case 1:
		{	
			h_bridge.DRV8245_EnterStandby();
			break;
		}
		case 2:
		{
			if(h_bridge.Get_ErrorState()==0)
			h_bridge.DRV8245_Forward();
			break;
		}
		case 3:
		{	if(h_bridge.Get_ErrorState()==0)
			h_bridge.DRV8245_Backward();
			break;
		}
		case 4:
		{	
			break;
		}
		case 5:
		{	
			break;
		}
		case 6:
		{	
			break;
		}
		default:
			break;
	}
}
