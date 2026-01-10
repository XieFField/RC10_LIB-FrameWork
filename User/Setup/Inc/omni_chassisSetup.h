/**
 * @file omni_chassisSetup.h
 * @brief 鍏ㄩ敓鏂ゆ嫹閿熸枻鎷风偔閿熸枻鎷烽敓锟?
 */
#ifndef __OMNI_CHASSISSETUP_H
#define __OMNI_CHASSISSETUP_H

#pragma once

#ifdef __cplusplus

extern "C"
{
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
#include "APP_Path.h"
#include "APP_Speedplanner.h"
#include "APP_Bezier_Curve.h"
#include "AutoCtrler.h"

#define debug_ladar 0
class OmniChassis_Setup : public RtosTask, public Chassis_Omni<3>
{
public:
    OmniChassis_Setup(float wheel_radius, float max_wheel_rpm, float base_length, float side_length, bool three_wheel)
        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<3>(wheel_radius, max_wheel_rpm, base_length, side_length, three_wheel),
          debug_uart(&huart8)
    {
    }

    void setChassisStatus(CHASSIS_Status_E status)
    {
        chassis_status_ = status;
    }

    void init()
    {
        if (this->wheels_[0] == nullptr || this->wheels_[1] == nullptr ||
            this->wheels_[2] == nullptr || this->wheels_[3] == nullptr)
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
        if (!isReverse)
            this->is_chassis_reverse_ = 1.0f;
        else
            this->is_chassis_reverse_ = -1.0f;
    }

private:
    int flag = 0;
    int flag_run = 0;
    CHASSIS_Status_E chassis_status_ = CHASSIS_STOP;

    Path path_;
    Path_line path_line_;

    float a=2.0f;
    float b=0.5f;
    
    Point3D ladar_data_;
    Vector2D robot_pos_ = {0.0f, 0.0f};
    
    Robot_Twist last_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; 
    Vector2D planspeed;
    Vector2D speed;    
    
    int8_t point_map=0;
    int8_t path_point_[20];
    int8_t path_key_point_[10];
    int8_t KFS=4;

    float target_yaw_ = 0.0f;
    uint8_t yaw_pid_period_ = 3;
    uint8_t yaw_pid_period_count_ = 0;
    PID_Position yaw_pid_;
    
    void loop() override;
    bool init_flag = false;

    // Robot_Twist chassis_maxSpeed_ = {0};
    const float LINESPEED_LIMIT = 10 / 500.f; // 线速度限制
    const float YAWSPEED_LIMIT = 1 / 500.f;   // yaw速度限制

    float is_chassis_reverse_ = 1.0f;
    
    RmPocketData_t airjoy_data_; // 遥控器数据，范围 -1 ~ 1

    Debug_Printf debug_uart = Debug_Printf(&huart8); // 调试串口
    
    Speedplanner_1D_Param_Config path_param_={.maxAcc = 3.0f, .maxDec = 3.0f, .maxJerk = 4.0f, .maxSpeed = 0.5f, .initialSpeed = 0.05f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f}; 
    MF_AutoCtrler::PathNode_S KFS_result_ = {0, 0, 0, 0, 0, 26};
    /**
     * @brief 获取路径上距离机器人最近的点
     * @param path_ 贝塞尔曲线对象
     * @param robotPos 机器人当前位置
     * @param tNearest 输出参数，返回最近点的t值
     * @return Vector2D 最近点的坐标
     */
    Vector2D GetPathNearestPoint(BezierCurve &path_, const Vector2D &robotPos, float &tNearest);

    /**
     * @brief 寻找前视点
     * @param path_ 贝塞尔曲线对象
     * @param tNearest 最近点的t值
     * @param tLookahead 输出参数，返回前视点的t值
     * @return Vector2D 前视点的坐标
     */
    Vector2D FindLookaheadPoint(BezierCurve &path_, float tNearest, float &tLookahead);

    /**
     * @brief 计算横向误差
     * @param path_ 贝塞尔曲线对象
     * @param robotPos 机器人当前位置
     * @param nearestPt 最近点坐标
     * @param tLookahead 前视点的t值
     * @return float 横向误差值 (带符号，表示偏左或偏右)
     */
    float CalculateLateralError(BezierCurve &path_, const Vector2D &robotPos, const Vector2D &nearestPt, float tLookahead);

    int num = 0;
    float tNearest = 0.0f;                // 最近点在贝塞尔曲线上的参数t (0~1)
    float tLookahead = 0.0f;              // 前视点在贝塞尔曲线上的参数t (0~1)
    float m_lookaheadDist = 0.4f;         // 前视距离 (单位: 米)
    float lateralError = 0.0f;            // 横向误差 (机器人偏离路径的距离)
    float correctspeed = 0.0f;            // 计算出的横向纠偏速度大小
    Vector2D nearestPt;                   // 路径上距离机器人最近的点
    Vector2D lookaheadPt;                 // 路径上的前视点
    Vector2D lookaheadTangent;            // 前视点处的切线方向向量
    Vector2D pathEnd;                     // 路径终点坐标
    Vector2D corrVelocity = {0.0f, 0.0f}; // 计算出的横向纠偏速度向量
    PID_Position pid_track;               // 循迹横向误差PID控制器
};
#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H