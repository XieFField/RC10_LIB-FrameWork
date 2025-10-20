#ifndef __MODULE_GPIO_H
#define __MODULE_GPIO_H
#ifdef __cplusplus
extern "C" {
#endif
#include "arm_math.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
#include "main.h"
#endif __cplusplus

#ifdef __cplusplus


class GPIO {
public:
	GPIO(GPIO_TypeDef* port,uint16_t pin);
	~GPIO();
void Set_pin();    
void Reset_pin();
void Toggle_pin();
bool Read_pin();


private:
    GPIO_TypeDef* port_; // GPIO 端口，如 GPIOA、GPIOB 等
    uint16_t pin_;       // GPIO 引脚，如 GPIO_PIN_0、GPIO_PIN_1 等
};




#endif 

#endif 