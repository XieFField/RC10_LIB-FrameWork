#include "Module_MyChassis.h"


template <std::size_t WheelCount>
void MyChassis<WheelCount>::inverseKinematics(const Robot_Twist& twist)
{
    input[0] = twist.vx;
    input[1] = twist.vy;
    input[2] = twist.yaw_rate;

    arm_mat_mult_f32(&mat_, &in, &out);

	for(uint8_t i	= 0;i < WheelCount; ++i){       
				this->wheel_target_rpm_[i] = this->wheelSpeedToMotorRPM(output[i]);
    }

}

template <std::size_t WheelCount>
void MyChassis<WheelCount>::updateKinematics()
{
    inverseKinematics(this->robot_twist_);
	forwardKinematics();
}


template <std::size_t WheelCount>
MyChassis<WheelCount>::MyChassis(float wheel_radius, float max_wheel_rpm, float chassis_radius)
    : Chassis_Base<WheelCount>(wheel_radius, max_wheel_rpm),
 chassis_radius_(chassis_radius)
{ 
    
	//float *d = mat_data_;
    d[0] = 1; d[1] = -1; d[2] = -chassis_radius;
    d[3] = -1; d[4] =  -1; d[5] =  -chassis_radius;
    d[6] = -1; d[7] =  -1; d[8] = chassis_radius;
    d[9] = 1; d[10]= -1; d[11]=  chassis_radius;
    arm_mat_init_f32(&mat_, 4, 3, d);


    arm_mat_init_f32(&in, 3, 1, input);
    arm_mat_init_f32(&out, 4, 1, output);
   
}

template <std::size_t WheelCount>
void MyChassis<WheelCount>::forwardKinematics()
{
    Robot_Twist twist;
    const float wheel_radius_m = wheel_radius / 1000.0f;// wheel_radius单位为m
    const float rpm_to_rad_per_s = (2.0f * PI) / 60.0f;
    float wheel_rpm[4];
    for(int i = 0;i < 4; i++)
    {
        wheel_rpm[i]=motors_[i] ? motors_[i]->getRPM() : 0.0f;
    }
    twist.vx = wheel_radius_m * (wheel_rpm[0] + wheel_rpm[1] + wheel_rpm[2] + wheel_rpm[3]) * rpm_to_rad_per_s / 4.0f;
    twist.vy = wheel_radius_m * (-wheel_rpm[0] + wheel_rpm[1] + wheel_rpm[2] - wheel_rpm[3]) * rpm_to_rad_per_s / 4.0f;
    twist.yaw_rate = wheel_radius_m * (-wheel_rpm[0] + wheel_rpm[1] - wheel_rpm[2] + wheel_rpm[3]) * rpm_to_rad_per_s / (4.0f * chassis_radius_);
}

template class Chassis_Base<4>;
template class MyChassis<4>;