#include "MyChassis.h"

MyChassis::MyChassis(float wheel_radius, float max_wheel_rpm, float wheel_distance_x, float wheel_distance_y)
    : Chassis_Base<4>(wheel_radius, max_wheel_rpm),
 wheel_distance_x(wheel_distance_x),
 wheel_distance_y(wheel_distance_y)
{
	
}

void MyChassis::updateKinematics()
{
    inverseKinematics(twist);
    forwardKinematics();
}

void MyChassis::inverseKinematics_init(float wheel_radius, float wheel_distance_x, float wheel_distance_y)
{
    float sum = wheel_distance_x + wheel_distance_y;
    float *d = mat_data_;
    d[0] = 1; d[1] = -1; d[2] = -sum;
    d[3] = 1; d[4] =  1; d[5] =  sum;
    d[6] = 1; d[7] =  1; d[8] = -sum;
    d[9] = 1; d[10]= -1; d[11]=  sum;
    arm_mat_init_f32(&mat_, 4, 3, mat_data_);
}

void MyChassis::inverseKinematics(const Robot_Twist& twist)
{
    const float sum = wheel_distance_x + wheel_distance_y;
    const float rad_per_s_to_rpm = 60.0f / (2.0f * PI);
    
    float input[3] = {twist.vx , twist.vy, twist.yaw_rate};//输入向量
    float output[4];//输出向量为四个轮子的速度
    
    arm_matrix_instance_f32 in, out;
    arm_mat_init_f32(&in, 3, 1, input);
    arm_mat_init_f32(&out, 4, 1, output);
    arm_mat_mult_f32(&mat_, &in, &out);//矩阵结果运算得到轮子线速度
    
    for (int i = 0; i < 4; i++) {
        wheel_speed_[i] = output[i] / wheel_radius_;//线速度转化为转速
    }
    for (int i=0;i<4;++i)//限速
    {
        if(wheel_speed_[i]>max_wheel_rpm_)
            wheel_speed_[i]=max_wheel_rpm_;
        else if(wheel_speed_[i]<-max_wheel_rpm_)
            wheel_speed_[i]=-max_wheel_rpm_;
    }
    for (int i = 0; i < 4; i++) {  
        if(motors_[i])            
            motors_[i]->setTargetRPM(wheel_speed_[i]); 
}
}

void MyChassis::forwardKinematics()
{
    const float wheel_radius_m = wheel_radius_ / 1000.0f;
    const float lx_plus_ly = wheel_distance_x + wheel_distance_y;
    const float rpm_to_rad_per_s = (2.0f * PI) / 60.0f;
    float wheel_rpm[4];
    for(int i=0;i<4;++i)

    {
        wheel_rpm[i]=motors_[i] ? motors_[i]->getRPM() : 0.0f;
    }
    twist.vx = wheel_radius_m * (wheel_rpm[0] + wheel_rpm[1] + wheel_rpm[2] + wheel_rpm[3]) * rpm_to_rad_per_s / 4.0f;
    twist.vy = wheel_radius_m * (-wheel_rpm[0] + wheel_rpm[1] + wheel_rpm[2] - wheel_rpm[3]) * rpm_to_rad_per_s / 4.0f;
    twist.yaw_rate = wheel_radius_m * (-wheel_rpm[0] + wheel_rpm[1] - wheel_rpm[2] + wheel_rpm[3]) * rpm_to_rad_per_s / (4.0f * lx_plus_ly);
}
