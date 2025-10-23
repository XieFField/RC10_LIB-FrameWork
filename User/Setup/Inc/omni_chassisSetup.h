/**
 * @file omni_chassisSetup.h
 * @brief È«Ïòµ×ÅÌ¿ØÖÆ
 */

class OmniChassis_Setup:public RtosTask, public Chassis_Omni<4>{
public:
OmniChassis_Setup(float wheel_radius, float max_wheel_rpm)
    : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<4>(wheel_radius, max_wheel_rpm)
{}

void registerMotors(Motor_Base* wheel1, Motor_Base* wheel2, Motor_Base* wheel3, Motor_Base* wheel4)
{
    if( motor_registered )
        return;
    this->registerWheelMotor(0, wheel1);
    this->registerWheelMotor(1, wheel2);
    this->registerWheelMotor(2, wheel3);
    this->registerWheelMotor(3, wheel4);
    motor_registered = true;
}

void init() 
{
    if(!motor_registered)
        init_flag = false;
    start(osPriorityHigh, 256);
    init_flag = true;
}
private:
    void loop() override;
    bool init_flag = false;
    bool motor_registered = false;
};