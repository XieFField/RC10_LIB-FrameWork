#ifndef ONIM_DEMO_H
#define ONIM_DEMO_H

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "BSP_RTOS.h"
#include "BSP_fdCAN_Driver.h"
#include "Motor_DJI.h"
#include "APP_PID.h"
#include "APP_debugTool.h"
#include "BSP_TimeStamp.h"
#include "APP_Tool.h"
#include "Module_Chassisdemo.h"
#include "Module_Air_joy.h"

extern AirJoy air_joy;

template <std::size_t WheelCount>
class OnimDemo:public Chassis_Demo<WheelCount>, public RtosTask{
public:
   OnimDemo(float wheel_radius, float max_wheel_rpm, float chassis_radius); 
   void init()
	 {
		 start(osPriorityNormal, 128);
		 init_flag = true;
	 }
protected: 
    void loop() override; 

    static inline float step_pm(uint16_t us, uint16_t mid=1500, uint16_t dead=60, float rate=0.25f) 
    { 
        // 返回单位比例系数，范围在-rate..+rate之间 
        if(us > mid + dead) return +rate; 
        if(us < mid - dead) return -rate; 
        return 0.0f; 
    } 

private: 
    Debug_Printf debug_uart = Debug_Printf(&huart1);
    bool init_flag = false; 
    
    std::size_t motor_count_ = 0;
    uint8_t test_mode = 0;

}; 



#endif // __cplusplus 

#endif
  
// Chassis_Demo.cpp 末尾
// 显式实例化4轮底盘模板类 

