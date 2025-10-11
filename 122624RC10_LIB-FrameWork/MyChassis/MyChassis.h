#ifndef __MY_CHASSIS_H
#define __MY_CHASSIS_H

#define  PI  3.14159265358979323846f

#include "Module_ChassisBase.h"
#include "Motor_DJI.h"
#include "arm_math.h"

class MyChassis : public Chassis_Base<4>
//在Module_ChassisBase.cpp里添加了template class Chassis_Base<4>;这句话，否则会报undefined reference to `Chassis_Base<4>::Chassis_Base(float, float)'
//解释说是链接器找不到类的实例化
{
public:
	MyChassis(float wheel_radius, float max_wheel_rpm, float wheel_distance_x, float wheel_distance_y);
	void registerMotor(int idx, DJI_Motor* motor) {
		if(idx >= 0 && idx < 4) {
			motors_[idx] = motor;
		}//注册电机
	}

	void updateKinematics();
	void inverseKinematics_init(float wheel_radius, float wheel_distance_x, float wheel_distance_y);//初始化矩阵实现逆解需要的一些数据
	void inverseKinematics(const Robot_Twist& twist);
	void forwardKinematics();

private:
	float wheel_distance_x;
	float wheel_distance_y;
	float mat_data_[12] = {0};
	arm_matrix_instance_f32 mat_;
	DJI_Motor* motors_[4] = {nullptr};
	float wheel_speed_[4] = {0};
	Robot_Twist twist;
};

#endif