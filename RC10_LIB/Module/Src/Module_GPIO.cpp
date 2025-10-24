#include "Module_GPIO.h"


GPIODevice::GPIODevice(GPIO_TypeDef* port,uint16_t pin)
{
	port_=port;
	pin_=pin;
}
void GPIODevice::Set_pin()
{
	HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
}

void GPIODevice::Reset_pin()
{
	HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
}

void GPIODevice::Toggle_pin()
{
	 HAL_GPIO_TogglePin(port_, pin_);
}

bool GPIODevice::Read_pin()
{
	return HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET;
}


