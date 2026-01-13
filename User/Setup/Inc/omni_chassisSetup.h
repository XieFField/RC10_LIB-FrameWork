/**
 * @file omni_chassisSetup.h
 * @brief 鍏ㄩ敓鏂ゆ嫹閿熸枻鎷风偔閿熸枻鎷烽敓锟?
 */
#ifndef __OMNI_CHASSISSETUP_H
#define __OMNI_CHASSISSETUP_H

#pragma once



#ifdef __cplusplus


extern "C"{
    #include "stm32h7xx_hal.h"
    #include "cmsis_os.h"
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"
    #include "semphr.h"
};


#include "BSP_RTOS.h"   
#include "Module_ChassisOmni.h"
#include "Motor_Base.h"
#include "FSMstauts_enum.h"
#include "Module_CrsfReceiver.h"
#include "APP_debugTool.h"
#include "usart.h"
#include "Module_Position.h"
#include "APP_PID.h"
#include "Locate_Setup.h"
#include "BSP_USB_UART_Driver.h"
#include "usb_device.h"
#include "RTOS_QueueSetup.h"
#include "AutoCtrler.h"

#define debug_ladar 0
class OmniChassis_Setup:public RtosTask, public Chassis_Omni<3>{
public:
    OmniChassis_Setup(float wheel_radius, float max_wheel_rpm, float base_length, float side_length, bool three_wheel)
        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<3>(wheel_radius, max_wheel_rpm, base_length, side_length, three_wheel)
        ,debug_uart(&huart8)
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
        
        yaw_pid_.set_params(lock_angle_pid_params, 10000.0f); 
        
            this->setThreeWheelSolver(true);

        #if debug_ladar
            this->setThreeWheelSolver(false);
        #endif

        this->start(osPriorityHigh, 512);
        init_flag = true;
    }

    void setChassisReverse(bool isReverse)
    {
        if(!isReverse)
            this->is_chassis_reverse_ = 1.0f;
        else
            this->is_chassis_reverse_ = -1.0f;
    }

    /**
     * @brief 取值范围0~12
     * 0 表示没有要抓取的KFS
     */
    bool set_TargetKFS(int KFS)
    {
        target_KFS = KFS;
        if(target_KFS <0 || target_KFS >12)
            return false;
        MF_AutoCtrler::PathNode_S temp = MF_AutoCtrler::PathNodeResult_calc({0.0f, 0.0f, 0.0f},
                                                    MF_AutoCtrler::MFNum_TransforMapNum(target_KFS - 1),
                                                    0);
        path_node_.bestB1 = temp.bestB1;
        path_node_.bestBMF1 = temp.bestBMF1;

        path_node_.entranceMap = temp.entranceMap;
        path_node_.bestB2 = temp.bestB2;
        path_node_.bestBMF2 = temp.bestBMF2;
        return true;
    }

private:
        void loop() override;
        bool init_flag = false;
          
        Robot_Twist last_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        
        // Robot_Twist chassis_maxSpeed_ = {0};
        float target_yaw_ = 0.0f;
        uint8_t yaw_pid_period_ = 3;
        uint8_t yaw_pid_period_count_ = 0;
        PID_Position yaw_pid_;

        const float LINESPEED_LIMIT = 10/500.f; // 线速度限制
        const float YAWSPEED_LIMIT = 1/500.f; // yaw速度限制

        float is_chassis_reverse_ = 1.0f;
        CHASSIS_Status_E chassis_status_ = CHASSIS_STOP;
        RmPocketData_t airjoy_data_; //遥控器数据，范围 -1 ~ 1

        Debug_Printf debug_uart = Debug_Printf(&huart8); // 调试串口
        Point3D ladar_data_;

        int8_t target_KFS = 0;
        MF_AutoCtrler::PathNode_S path_node_; //路径节点数据
};
#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H