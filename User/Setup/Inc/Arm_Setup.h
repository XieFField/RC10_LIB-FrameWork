/**
 * @file Arm_setup.h
 * @author XieFField
 * @brief 串联臂控制实现
 * @version 1.0
 * @date 2025-09-17
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

        start(osPriorityNormal, 256);

        init_flag = true;
    }

    void setArmStatus(ARM_Status_E status)
    {
        arm_status_ = status;
    }
    
    
private:
    bool init_flag = false;

    bool is_calibrating = false;

    Debug_Printf debug_uart = Debug_Printf(&huart1);

    //控制函数
    void manualControl();
    void autoControl();
    void stop();
    void idle();
    void debug();

    //上电校准M2006电机位置
    void calibrateM2006(); float calibrate_startTime = 0; bool calibrate_start = false;

    uint8_t debug_start = 0;
protected:
    void loop() override;



    ARM_Status_E arm_status_ = ARM_MANUAL_CONTROL;
    ARM_Status_E last_arm_status_ = ARM_MANUAL_CONTROL;

    Joint_Status_S last_joint_status_ = {0.0f, 0.0f, 0.0f, 0.0f};
    Joint_Status_S target_joint_status_ = {0.0f, 0.0f, 0.0f, 0.0f};

    
    float now_time_s_ = 0; float last_time_s_ = 0;
};


extern Arm_InitData_S arm_initData;

#endif //__cplusplus


#endif // __ARM_SETUP_H
