/**
 * @file omni_chassisSetup.h
 * @brief 全锟斤拷锟斤拷炭锟斤拷锟�
 */
#ifndef __OMNI_CHASSISSETUP_H
#define __OMNI_CHASSISSETUP_H

#pragma once



#ifdef __cplusplus
#include "BSP_RTOS.h"   
#include "Module_ChassisOmni.h"
#include "Motor_Base.h"
#include "FSMstauts_enum.h"
#include "Module_Position.h"
#include "APP_PID.h"
#include "Motor_VESC.h"
#include "Module_Air_Joy.h"
#include "APP_debugTool.h"
#include "AutoCtrler.h"
#include "APP_Vector2D.h"
#include "APP_Speedplanner.h"
#include "PathPlanner.h"
#include "APP_Bezier_Curve.h"
#include <vector>

extern AirJoy air_joy;

#define PI 3.14159265358979323846f

class OmniChassis_Setup:public RtosTask, public Chassis_Omni<4>{
public:
    OmniChassis_Setup(float wheel_radius, float max_wheel_rpm, float chassis_radius)
        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<4>(wheel_radius, max_wheel_rpm, chassis_radius)
    {}

    void setChassisStatus(CHASSIS_Status_E status)
    {
        chassis_status_ = status;
    }

    void init() 
    {
        if(this->wheels_[0] == nullptr ||this->wheels_[1] == nullptr ||
           this->wheels_[2] == nullptr ||this->wheels_[3] == nullptr)
            init_flag = false;
        
        this->start(osPriorityHigh, 256);
        init_flag = true;
    }
   

    // 直接使用世界坐标设置测试起点/终点（覆盖索引方式）
    void setAutoTestPoints(const Vector2D &start, const Vector2D &goal);
 
    

     
private:

	float locked_yaw = 0.0f;
    float now_yaw = 0.0f;
    float yaw_ctrl = 0.0f;
    Robot_Twist last_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        void loop() override;
        bool init_flag = false; 
     Position *position = nullptr;
void chassis_manual_control_A();
void chassis_manual_control_B();
void chassis_stop();
void chassis_auto_control();
        Debug_Printf debug_uart = Debug_Printf(&huart2);
        CHASSIS_Status_E chassis_status_ = CHASSIS_STOP;

        Vector2D auto_start_point_ = Vector2D(0.0f, 0.0f);
        Vector2D auto_goal_point_ = Vector2D(0.0f, 0.0f);
};
#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H