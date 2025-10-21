#ifndef __MYCHASSIS_DEMO_H
#define __MYCHASSIS_DEMO_H

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
#include "Module_ChassisBase.h"
#include "Motor_DJI.h"
#include "BSP_fdCAN_Driver.h"
#include "Module_MyChassis.h"
#include "Motor_VESC.h"
#include "BSP_TimeStamp.h"
#include "Setup_ConfigInit.h"
#include <cstddef>
#include "Module_Air_Joy.h"

extern AirJoy air_joy;

template <std::size_t WheelCount>
class MyChassisController : public MyChassis<WheelCount>, public RtosTask
{
public:
    MyChassisController(float wheel_radius, float max_wheel_rpm, float chassis_radius) ;
    void init(DJI_Motor *input_motors[4]);
	static inline float ppm_to_norm_pm(const uint16_t us, uint16_t mid=1500,float span=500.0f)
	 {
	// [-1, +1]，1500 为中值，±500us 为满量程
 	return (static_cast<float>(us) - static_cast<float>(mid)) / span;
	}
protected:    
    void loop() override;
	bool my_init_flag = false;
    
private:
    DJI_Motor* motors_[WheelCount];
		std::size_t motor_count_ = 0;
		Robot_Twist target_speed;
		uint8_t my_start_signal = 0;
		float my_delta_time = 0.0f; 
		uint64_t my_last_time = 0;
};

#endif
#endif
