#ifndef CHASSIS_SWERVE_TASK_DEMO_H
#define CHASSIS_SWERVE_TASK_DEMO_H

#pragma once

#ifdef __cplusplus
extern "C" {
}
#endif

#ifdef __cplusplus

#include "Motor_DJI.h"
#include "chassis_swerve_demo.h"
#include "APP_Tool.h"
#include "Module_CrsfReceiver.h"
#include "BSP_TimeStamp.h"
#include "BSP_RTOS.h"
#include "Locate_Setup.h"
#define DEBUG_SHIT 0


class Swerve_Task_Demo: public RtosTask, public chassis_swerve_demo::Chassis_Swerve
{
public:
    Swerve_Task_Demo() : RtosTask("SwerveDemo", 1) {}
    ~Swerve_Task_Demo() = default;

    void init()
    {
        this->set_max_linear_vel(3.0f);
        this->set_max_angular_vel(1.0f);
        this->init_swerve();
        start(osPriorityHigh, 512); // 启动任务，优先级2，栈大小512字
    }

private:

    void loop() override;
    
    RmPocketData_t airjoy_data_;
};



#endif


#endif
