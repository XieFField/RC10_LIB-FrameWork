#ifndef __MODULE_MYCHASSIS_H
#define __MODULE_MYCHASSIS_H

#define  PI  3.14159265358979323846f
#include <cstddef> 
#include "Module_ChassisBase.h"
#include "Motor_DJI.h"
#include "arm_math.h"

#ifdef __cplusplus

template <std::size_t WheelCount>
class MyChassis : public Chassis_Base<WheelCount>
{
public:
	MyChassis(float wheel_radius, float max_wheel_rpm, float chassis_radius);
	void updateKinematics()override;
private:	
float wheel_radius;
DJI_Motor* motors_[WheelCount];
	float chassis_radius_; 
	void inverseKinematics(const Robot_Twist& twist);
  void forwardKinematics();

	arm_matrix_instance_f32 mat_;
	arm_matrix_instance_f32 in;    // 初始化输入矩阵
  arm_matrix_instance_f32 out;
  float32_t output[4]; 
	float32_t input[3] ;
	float32_t d[12] ;
};

#endif
#endif