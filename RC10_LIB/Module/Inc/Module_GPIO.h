/**
 * @file Module_GPIO.h
 * @author 70er66
 * @brief 达妙电机类
 * @version 1.0
 * 
 * 此文件包含达妙J4310封装
 */
 
 
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

//-------------------------------------------------------------------------
//*GPIO口的类封装；
//*使用前需要在Cubemx上配置好对应的GPIO口
//*在实例化创建对象的时候输入对应的GPIO口
//*GPIO口的中断管理后续添加；
class GPIODevice {
public:
	GPIODevice(GPIO_TypeDef* port,uint16_t pin);
	~GPIODevice(){};
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