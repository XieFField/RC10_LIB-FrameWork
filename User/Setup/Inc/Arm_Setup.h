/**
 * @file Arm_setup.h
 * @author XieFField
 * @brief 串联臂运动控制实现
 * @version 1.0
 */

#ifndef __ARM_SETUP_H
#define __ARM_SETUP_H

#ifdef __cplusplus
extern "C" {
#include "stdint.h"
}
#endif



#ifdef __cplusplus
#include "BSP_RTOS.h"
#include "Robot_Arm.h"
#include "APP_Tool.h"
#include "Module_Air_joy.h"
#include "Motor_DJI.h"
#include "BSP_TimeStamp.h"
#include "APP_debugTool.h"
#include "FSMstauts_enum.h"

typedef struct{
    bool init_flag = false;

    
    uint8_t debug_start = 0; //调试开始标志 == 1 开始调试

    float calibrate_startTime = 0; 
    bool calibrate_start = false;
    bool is_calibrating = false;

}arm_ctrl_status_S;

const float MF_high[12] = 
{
    40.0f, 20.0f, 40.0f,
    20.0f, 40.0f, 60.0f,
    40.0f, 60.0f, 40.0f,
    20.0f, 40.0f, 20.0f
};

class ArmSetup: public RtosTask ,public Robot_Arm {
public:
    ArmSetup(Arm_InitData_S init_Data)
        : Robot_Arm(init_Data), RtosTask("ArmSetup", 1) 
    {
        
    }

    void init(M3508 *motor_ArmLaunch, M2006 *motor_ArmStretch, 
        M3508 *motor_ArmRotate, M2006 *motor_ArmPitch)
    {
        this->registerMotor_Launch(motor_ArmLaunch);
        this->registerMotor_Stretch(motor_ArmStretch);
        this->registerMotor_Rotate(motor_ArmRotate);
        this->registerMotor_Pitch(motor_ArmPitch);

        this->setPitchReversed(true); //俯仰电机反向
        this->setStretchReversed(false); //伸展电机不反向

        start(osPriorityNormal, 256);

        arm_ctrlStatus.init_flag = true;
    }

    void setArmStatus(ARM_Status_E status)
    {
        arm_status_ = status;
    }
    
    
private:


    Debug_Printf debug_uart = Debug_Printf(&huart1);

    //控制函数
    void manualControl();
    void autoControl();
    void stop();
    void idle();
    void debug();

    //上电校准M2006电机位置
    void calibrateM2006();

protected:
    void loop() override;

    arm_ctrl_status_S arm_ctrlStatus = {
        .init_flag = false,
        .debug_start = 1,
        .calibrate_startTime = 0,
        .calibrate_start = false,
        .is_calibrating = false,
    };

    ARM_Status_E arm_status_ = ARM_MANUAL_CONTROL;
    ARM_Status_E last_arm_status_ = ARM_MANUAL_CONTROL;

    Joint_Status_S last_joint_status_ = {0.0f, 0.0f, 0.0f, 0.0f};
    Joint_Status_S target_joint_status_ = {0.0f, 0.0f, 0.0f, 0.0f};

    float launch20cm_time = 0.5f;
    float launch40cm_time = 1.0f;
};


extern Arm_InitData_S arm_initData;

#endif //__cplusplus


#endif // __ARM_SETUP_H
