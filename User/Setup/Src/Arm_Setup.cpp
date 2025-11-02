#include "Arm_setup.h"

void ArmSetup::loop()
{
    if(!arm_ctrlStatus.init_flag)
        return;

    if(!arm_ctrlStatus.is_calibrating)
    {
        calibrateM2006();
        arm_status_ = ARM_CALIBRATE;
    }


    switch(arm_status_)
    {
        case ARM_MANUAL_CONTROL:
            {
                manualControl();
            }
            break;

        case ARM_AUTO_CONTROL:
            {
                autoControl();
            }
            break;

        case ARM_STOP: 
            {
                // 停止状态, 将各个关节回归初始位置后，将电流置零
                stop();
            }
            break;
        case ARM_IDLE:
            {
                // 空闲状态，维持当前状态
                idle();
            }

        case ARM_DEBUG:
            {
                // 调试状态
                if(arm_ctrlStatus.debug_start == 1)
                    debug();
            }
            break;

        case ARM_CALIBRATE:
            {
                // 校准状态
                // 上电校准M2006电机位置
                break;
            }
        default:
            break;
    }

    this->update(); //将控制信息发送给电机
    last_arm_status_ = arm_status_;
}
    

void ArmSetup::manualControl()
{
    // 手动控制函数
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    
    if(last_arm_status_ != ARM_MANUAL_CONTROL)//若首次非此模式，需复制一下上次状态，免得跳变
    {
        /*串联臂*/
        last_joint_status_ = this->get_currentJointStatus();
        target_joint_status_ = last_joint_status_;

        last_arm_status_ = ARM_MANUAL_CONTROL;
    }

    // 读取遥控器输入，计算目标关节位置

    if(AirJoy::getinstance().RIGHT_X < 1450)
        target_joint_status_.rotateJoint_angle_ -= 5.0f; // 旋转关节逆时针
    else if(AirJoy::getinstance().RIGHT_X > 1550)
        target_joint_status_.rotateJoint_angle_ += 5.0f; // 旋转关节顺时针
    else
        target_joint_status_ = target_joint_status_; // 保持不变



    if(AirJoy::getinstance().RIGHT_Y < 1450)
        target_joint_status_.launchJoint_Height_ -= 0.005f; // 伸展关节收回
    else if(AirJoy::getinstance().RIGHT_Y > 1550)
        target_joint_status_.launchJoint_Height_ += 0.005f; // 伸展关节伸出
    else
        target_joint_status_.launchJoint_Height_ = target_joint_status_.launchJoint_Height_; // 保持不变

    if(_tool_Abs(AirJoy::getinstance().SWA - 1000) < 50)
        target_joint_status_.stretchJoint_Length_ = 0.0f; // 伸展关节收回到最小位置
    else if(_tool_Abs(AirJoy::getinstance().SWA - 2000) < 50)
        target_joint_status_.stretchJoint_Length_ = this->init_data_.max_stretchLength_; // 伸展关节伸出到最大位置
    else 
        target_joint_status_.stretchJoint_Length_ = target_joint_status_.stretchJoint_Length_; // 保持不变



    if(_tool_Abs(AirJoy::getinstance().SWD - 1000) < 50)
        target_joint_status_.suckerJoint_angle_ = 0.0f; // 末端关节收
    else if(_tool_Abs(AirJoy::getinstance().SWD - 2000) < 50)
        target_joint_status_.suckerJoint_angle_ = 90.0f; // 末端关节开
    else 
        target_joint_status_.suckerJoint_angle_ = target_joint_status_.suckerJoint_angle_; // 保持不变

    this->set_LaunchHeight(target_joint_status_.launchJoint_Height_);
    this->set_StretchLength(target_joint_status_.stretchJoint_Length_);
    this->set_RotateAngle(target_joint_status_.rotateJoint_angle_);
    this->set_PitchAngle(target_joint_status_.suckerJoint_angle_);

    if(_tool_Abs(AirJoy::getinstance().SWC - 2000) < 50)
        this->setSuckerStatus(Sucker_Status_E::SUCK);
    else
        this->setSuckerStatus(Sucker_Status_E::STOP);
}


void ArmSetup::autoControl()
{
    // 自动控制函数
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);

    //暂时应该是只写无梅花桩的自动；这就比较简单了，都不需要关心避障，只需要做吸取的预判而已；


}

void ArmSetup::stop()
{
    // 停止控制函数
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    this->setSuckerStatus(Sucker_Status_E::STOP);

    this->set_LaunchHeight(0.0f);
    this->set_StretchLength(0.0f);
    this->set_RotateAngle(0.0f);
    this->set_PitchAngle(0.0f);

    if(_tool_Abs(this->motor_launch_->getTotalAngle() - 0.0f) < 0.1f)
        this->motor_launch_->setTargetCurrent(0.0f);
    
    if(_tool_Abs(this->motor_stretch_->getTotalAngle() - 0.0f) < 0.1f)
        this->motor_stretch_->setTargetCurrent(0.0f);

    if(_tool_Abs(this->motor_rotate_->getTotalAngle() - 0.0f) < 0.1f)
        this->motor_rotate_->setTargetCurrent(0.0f);

    if(_tool_Abs(this->motor_pitch_->getTotalAngle() - 0.0f) < 0.1f)
        this->motor_pitch_->setTargetCurrent(0.0f);
    if((_tool_Abs(this->motor_launch_->getTotalAngle() - 0.0f) < 0.1f) && 
       (_tool_Abs(this->motor_stretch_->getTotalAngle() - 0.0f) < 0.1f) &&
       (_tool_Abs(this->motor_rotate_->getTotalAngle() - 0.0f) < 0.1f) &&
       (_tool_Abs(this->motor_pitch_->getTotalAngle() - 0.0f) < 0.1f))
    {
        //全部到位后，切换到空闲模式
        this->set_controlMode(CURRENT_CONTROL_MODE);
        this->motor_launch_->setTargetCurrent(0.0f);
        this->motor_stretch_->setTargetCurrent(0.0f);
        this->motor_rotate_->setTargetCurrent(0.0f);
        this->motor_pitch_->setTargetCurrent(0.0f);
    }
}

void ArmSetup::calibrateM2006()
{
    this->set_controlMode(CURRENT_CONTROL_MODE);
    // 上电校准M2006电机位置
    // 给予M2006一个小电流顶住限位，然后计时1s，将当前位置重定位为0度
    if(!arm_ctrlStatus.calibrate_start)
    {
        arm_ctrlStatus.calibrate_startTime = TimeStamp::getInstance().getSeconds();
        arm_ctrlStatus.calibrate_start = true;
    }
    this->motor_stretch_->setTargetCurrent(-1000.0f); // 给予一个小电流顶住限位
    this->motor_pitch_->setTargetCurrent(1000.0f); // 给予一个小电流顶住限位

    if(this->now_time_s_ - arm_ctrlStatus.calibrate_startTime > 1.5f)
    {
        this->motor_stretch_->relocate_totalAngle(0.0f);
        this->motor_pitch_->relocate_totalAngle(0.0f);
        this->motor_stretch_->setTargetCurrent(0.0f);
        this->motor_pitch_->setTargetCurrent(0.0f);

        arm_ctrlStatus.is_calibrating = true;
    }
}

void ArmSetup::idle()
{
    // 空闲控制函数，若上一时刻非此模式，则记忆上一时刻位置，并维持不变
    this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
    if(last_arm_status_ != ARM_IDLE)
    {
        last_joint_status_ = this->get_currentJointStatus();
        target_joint_status_ = last_joint_status_;

        last_arm_status_ = ARM_IDLE;
    }

    this->set_LaunchHeight(target_joint_status_.launchJoint_Height_);
    this->set_StretchLength(target_joint_status_.stretchJoint_Length_);
    this->set_RotateAngle(target_joint_status_.rotateJoint_angle_);
    this->set_PitchAngle(target_joint_status_.suckerJoint_angle_);

    this->setSuckerStatus(Sucker_Status_E::STOP);
}

uint8_t test_signal = 0;
float test_current = 0.0f;
float rotate_rate = 5.0f;
float launch_rate = 0.005f;

float stretch_starttime = 0;
bool stretch_flag = false;
float pitch_starttime = 0;
bool pitch_flag = false;
float stretch_usetime = 0;
float pitch_usetime = 0;
void ArmSetup::debug()
{
    //测试
    if(test_signal == 0) //所有电机电流强制为0；检查电机转动方向是否需要置反
    {
        this->set_controlMode(CURRENT_CONTROL_MODE);
        this->motor_launch_->setTargetCurrent(0.0f);
        this->motor_stretch_->setTargetCurrent(0.0f);
        this->motor_rotate_->setTargetCurrent(0.0f);
        this->motor_pitch_->setTargetCurrent(0.0f);
    }

    //1~4 signal test用于测试电机电流方向和电机转动方向是否同相
    else if(test_signal == 1)
        this->motor_launch_->setTargetCurrent(test_current);

    else if(test_signal == 2)
        this->motor_stretch_->setTargetCurrent(test_current);

    else if(test_signal == 3)
        this->motor_rotate_->setTargetCurrent(test_current);
        
    else if(test_signal == 4)
        this->motor_pitch_->setTargetCurrent(test_current);

    //航模遥控操纵测试
    else if(test_signal == 5)
    {
        this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
        // 读取遥控器输入，计算目标关节位置

        if(AirJoy::getinstance().RIGHT_X < 1450)
            target_joint_status_.rotateJoint_angle_ -= rotate_rate; // 旋转关节逆时针
        else if(AirJoy::getinstance().RIGHT_X > 1550)
            target_joint_status_.rotateJoint_angle_ += rotate_rate; // 旋转关节顺时针
        else
            target_joint_status_ = target_joint_status_; // 保持不变



        if(AirJoy::getinstance().RIGHT_Y < 1450)
            target_joint_status_.launchJoint_Height_ -= launch_rate; // 伸展关节收回
        else if(AirJoy::getinstance().RIGHT_Y > 1550)
            target_joint_status_.launchJoint_Height_ += launch_rate; // 伸展关节伸出
        else
            target_joint_status_.launchJoint_Height_ = target_joint_status_.launchJoint_Height_; // 保持不变

        if(_tool_Abs(AirJoy::getinstance().SWA - 1000) < 50)
            target_joint_status_.stretchJoint_Length_ = 0.0f; // 伸展关节收回到最小位置
        else if(_tool_Abs(AirJoy::getinstance().SWA - 2000) < 50)
            target_joint_status_.stretchJoint_Length_ = this->init_data_.max_stretchLength_; // 伸展关节伸出到最大位置
        else 
            target_joint_status_.stretchJoint_Length_ = target_joint_status_.stretchJoint_Length_; // 保持不变



        if(_tool_Abs(AirJoy::getinstance().SWD - 1000) < 50)
            target_joint_status_.suckerJoint_angle_ = 0.0f; // 末端关节收
        else if(_tool_Abs(AirJoy::getinstance().SWD - 2000) < 50)
            target_joint_status_.suckerJoint_angle_ = 95.0f; // 末端关节开
        else 
            target_joint_status_.suckerJoint_angle_ = target_joint_status_.suckerJoint_angle_; // 保持不变

        this->set_LaunchHeight(target_joint_status_.launchJoint_Height_);
        this->set_StretchLength(target_joint_status_.stretchJoint_Length_);
        this->set_RotateAngle(target_joint_status_.rotateJoint_angle_);
        this->set_PitchAngle(target_joint_status_.suckerJoint_angle_);

        if(_tool_Abs(AirJoy::getinstance().SWC - 2000) < 50)
            this->setSuckerStatus(Sucker_Status_E::SUCK);
        else
            this->setSuckerStatus(Sucker_Status_E::STOP);
    }

    else if(test_signal == 6) //测试stop功能
    {
        this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
        this->stop();
    }

    else if(test_signal == 7) //测试idle功能
    {
        this->set_controlMode(MANUAL_MOTOR_POSITION_MODE);
        this->idle();
    }

    else //empty
    {
        this->set_controlMode(CURRENT_CONTROL_MODE);
        this->set_LaunchHeight(0.0f);
        this->set_StretchLength(0.0f);
        this->set_RotateAngle(0.0f);
        this->set_PitchAngle(0.0f);
    }
}

Arm_InitData_S arm_initData = {
   .max_launchHeight_ = 0.4f,
   .max_stretchLength_ = 0.130f,
   .arm_length_ = 0.6f,
   .end_link_length_ = 0.08f,

   .stretch_Ratio_ = 0.08417f,
   .launch_Ratio_ = 0.01099f,
   .rotate_gearRatio_ = 144.878f,
   .pitch_gearRatio_ = 360.0f,

   .min_rotate_angle_ = 0.0f,
   .max_rotate_angle_ = 359.999f,
};


