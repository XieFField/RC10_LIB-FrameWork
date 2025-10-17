#include "mychassis_demo.h"

volatile uint8_t my_start_signal = 0;
volatile float my_delta_time = 0.0f; 
volatile uint64_t my_last_time = 0;

template <std::size_t WheelCount>
MyChassisController<WheelCount>::MyChassisController(float wheel_radius, float max_wheel_rpm, float chassis_radius)
    :MyChassis<4>(wheel_radius, max_wheel_rpm, chassis_radius),
    RtosTask("MyChassisController", 1)
{
		target_speed.vx = 0.0f; 
    target_speed.vy = 0.0f; 
    target_speed.yaw_rate = 0.0f; 
	
    for(std::size_t i = 0; i < WheelCount; i++) {
        motors_[i] = nullptr;
    }
}

template <std::size_t WheelCount>
void MyChassisController<WheelCount>::init(DJI_Motor *input_motors[4])
{
    for(std::size_t i = 0; i < WheelCount; i++) {
        motors_[i] = input_motors[i];
    }

    start(osPriorityNormal, 256);
	  my_init_flag = true;
    my_start_signal = 0;

}

template <std::size_t WheelCount>
void MyChassisController<WheelCount>::loop()
{
	if(!my_init_flag)
        return;
	
	uint64_t time_now = TimeStamp::getInstance().getMicroseconds();
    if(my_last_time > 0)
    {
        my_delta_time = static_cast<float>(time_now - my_last_time); 
    }
		my_last_time = time_now;

   if(my_start_signal == 1)
   {
        // 前进0.5 m/s
        target_speed.vx = 0.8f; // m/s
        target_speed.vy = 0.5f; // m/s
        target_speed.yaw_rate = 0.0f; // rad/s
   }
    else if(my_start_signal == 2)
    {
        //侧移0 .5m/s
        target_speed.vx = 0.0f; // m/s
        target_speed.vy = 0.5f; // m/s
        target_speed.yaw_rate = 0.0f; // rad/s
    }
    else if(my_start_signal == 3)
    {
        //原地旋转1.0 rad/s
        target_speed.vx = 0.0f; // m/s
        target_speed.vy = 0.0f; // m/s
        target_speed.yaw_rate = 1.0f; // rad/s
    }
    else
    {
        // 停止
        target_speed.vx = 0.0f; // m/s
        target_speed.vy = 0.0f; // m/s
        target_speed.yaw_rate = 0.0f; // rad/s
    }
		this->robot_twist_ = target_speed ;
    this->updateKinematics();

		for(std::size_t i = 0; i < WheelCount; i++) {
        if(motors_[i]) {
            float wheel_rpm = this->getWheelTargetRPM(i);
            motors_[i]->setTargetRPM(wheel_rpm);
        }
    }
};
template class MyChassisController<4>;