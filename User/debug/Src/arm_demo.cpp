#include "arm_demo.h"



Robot_ArmDemo::Robot_ArmDemo(Arm_InitData_S init_Data) : 
        Robot_Arm(init_Data), RtosTask("Robot_ArmDemo",1) 
{
}

void Robot_ArmDemo::armInit(DJI_Motor *motor_ArmLaunch, DJI_Motor *motor_ArmStretch, 
    DJI_Motor *motor_ArmRotate, DJI_Motor *motor_ArmPitch)
{
    this->registerMotor_Launch(motor_ArmLaunch);
    this->registerMotor_Stretch(motor_ArmStretch);
    this->registerMotor_Rotate(motor_ArmRotate);
    this->registerMotor_Pitch(motor_ArmPitch);


    start(osPriorityNormal, 256);

    init_flag = true;
}

Arm_Point_S test_point = {
    .x = 0.3f,
    .y = 0.0f,
    .z = 0.0f,

    .suckerJoint_status_ = 0.0f
};



void Robot_ArmDemo::loop()
{
    if(!init_flag)
        return;

    
    static uint64_t last_us = 0;
    uint64_t now_us = TimeStamp::getInstance().getMicroseconds();
    if(last_us == 0) 
    { 
        last_us = now_us; 
        return; 
    }
    uint64_t dt_us = (now_us >= last_us) ? (now_us - last_us) : 0;
    last_us = now_us;
    if(dt_us == 0) 
        return;
    if(dt_us > 200000) 
        dt_us = 200000; 
    float dt = dt_us * 1e-6f;

    
    const float v_xyz = 0.2f;     
    const float v_aux = 0.5f;     

//    test_point.z += step_pm(air_joy.LEFT_Y)  * v_xyz * dt;
//    test_point.x += step_pm(air_joy.LEFT_X)  * v_xyz * dt;
//    test_point.y += step_pm(air_joy.RIGHT_Y) * v_xyz * dt;
//    test_point.suckerJoint_status_ += step_pm(air_joy.RIGHT_X) * v_aux * dt;

    this->setArmTarget(test_point);
    this->update();
    // Main loop code here
}


