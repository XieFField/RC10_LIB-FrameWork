/**
 * @file omni_chassisSetup.h
 * @brief 底盘应用类
 * @author @XieFField @naoganlin @GaGiaa
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
#include "chassis.h"

#define debug_ladar 0

class OmniChassis_Setup : public RtosTask, public Chassis_Omni<3>
{
public:
    OmniChassis_Setup(float wheel_radius, float max_wheel_rpm, float base_length, float side_length, bool three_wheel)
        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<3>(wheel_radius, max_wheel_rpm, base_length, side_length, three_wheel)
        ,debug_uart(&huart8)
    {
        yaw_pid_.set_as_circular();
    }

    OmniChassis_Setup(Chassis_Omni<3>::init_config& config)
        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<3>(config)
        ,debug_uart(&huart8)
    {
        yaw_pid_.set_as_circular();
    }

    void setChassisStatus(CHASSIS_Status_E status)
    {
        chassis_status_ = status;
    }

    void init()
    {
        init_flag = false;
        for (uint8_t i = 0; i < 3; ++i)
        {
            if (this->wheels_[i] == nullptr)
                return;
        }

        yaw_pid_.set_params(lock_angle_pid_params, 10000.0f);

        this->setThreeWheelSolver(true);

#if debug_ladar
        this->setThreeWheelSolver(false);
#endif
        pid_pos_x.set_params(track_pid_params, 0.0f);
        pid_pos_y.set_params(track_pid_params, 0.0f);

        this->start(osPriorityHigh, 1024);
        init_flag = true;
    }

    void setChassisReverse(bool isReverse)
    {
        if (!isReverse)
            this->is_chassis_reverse_ = 1.0f;
        else
            this->is_chassis_reverse_ = -1.0f;
    }

    void setPathAutoStart(uint8_t start)
    {
        if (start == 1)
            flag = 1;
        else
            flag = 0;

        if (start == 0)
        {
            flag_run = 0;
        }
    }

    void setTargetKFS(int targetKFS)
    {
        KFS = targetKFS;
    }

    bool GetReach_flag()
    {
        return WeaponSage_END;
    }

    bool Get_Arm_Start_flag()
    {
        return Arm_Start;
    }

    void Receive_Arm_End_flag(bool arm_end)
    {
        Arm_Start = arm_end;
    }

    void set_KFS(int8_t KFS1, int8_t KFS2)
    {
        MF1 = KFS1;
        MF2 = KFS2;
    }

private:
    bool WeaponSage_END = 0;
    bool Arm_Start = false;
    CHASSIS_Status_E chassis_status_ = CHASSIS_STOP;

    int flag = 0;
    int flag_run = 0;

    Path_line path_line_;
    Vector2D Clamping_Bar_Selection_pos_ = {1.925f + 0.48f, 0.19f + 0.50f};

    Speedplanner_1D_Param_Config path_param_KFS_ = {.maxAcc = 30.0f, .maxDec = 40.0f, .maxJerk = 100.0f, .maxSpeed = 0.6f, .initialSpeed = 0.3f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f};
    Speedplanner_1D_Param_Config path_param_CB_ = {.maxAcc = 5.0f, .maxDec = 5.0f, .maxJerk = 0.0f, .maxSpeed = 0.75f, .initialSpeed = 0.3f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f};

    Point3D ladar_data_;
    Vector2D robot_pos_ = {0.0f, 0.0f};
    Point2D robot_point_ = {0.0f, 0.0f};

    Vector2D planspeed;
    Vector2D speed;
    Vector2D corrVelocity = {0.0f, 0.0f};

    PID_Position pid_pos_x;
    PID_Position pid_pos_y;

    Robot_Twist last_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    float tNearest = 0.0f;
    float tLookahead = 0.0f;

    float max_robot_speed_ = 1.5f;
    float max_robot_speed_end_ = 0.4f;
    float t_deadzone = 0.93f;
    float max_corr_end_ = 0.5f;

    Vector2D nearestPt;
    Vector2D lookaheadPt;
    Vector2D lookaheadTangent;
    Vector2D pathEnd;

    BezierCurve curve;

    int KFS = 0;
    MF_AutoCtrler::PathInformation_S KFS_KeyPoint_;

    int8_t MF1 = 0;
    int8_t MF2 = 0;
    int8_t MF1_Point_ = 0;
    int8_t MF2_Point_ = 0;

    Vector2D MF1_pos_ = {0.0f, 0.0f};
    Vector2D MF2_pos_ = {0.0f, 0.0f};

    int index_exit = 0;

    float MF2_target_yaw_ = 0.0f;
    bool spin_flag = false;
    bool spin_up_flag = false;
    bool spin_down_flag = false;
    bool MF1_flag = false;
    bool MF2_flag = false;
    bool MF1_finish = false;

    Vector2D spin_point_ = {3.6f, 8.72f};
    float spin_skew_ = -0.1f;
    bool get_spin_flag = false;
    bool Spin_Start = false;

    float yaw = 0.0f;
    float target_yaw_ = 0.0f;

    uint8_t yaw_pid_period_ = 3;
    uint8_t yaw_pid_period_count_ = 0;
    PID_Position yaw_pid_;

    void loop() override;

    bool init_flag = false;
    float is_chassis_reverse_ = 1.0f;

    Vector2D ff_ref_point_ = {0.0f, 0.0f};
    Vector2D ff_ref_point_last_ = {0.0f, 0.0f};
    Vector2D ff_velocity_lpf_ = {0.0f, 0.0f};
    bool ff_diff_inited_ = false;

    float m_lookaheadDist = 0.3f;
    float kff_la_ = 0.0f;
    float ff_lpf_alpha_ = 0.20f;
    float control_period_s_ = 0.001f;
    float ff_dt_min_s_ = 0.0009f;
    float ff_dt_max_s_ = 0.010f;
    float max_ff_speed_ = 1.0f;
    Vector2D v_robot_last_cmd_ = {0.0f, 0.0f};
    float k_damp_ = 0.0f;
    float end_ff_scale_ = 0.35f;
    float end_pid_scale_ = 0.7f;

    RmPocketData_t airjoy_data_;
    Debug_Printf debug_uart = Debug_Printf(&huart8);

    Vector2D GetPathNearestPoint(BezierCurve &path_, const Vector2D &robotPos, float &tNearest);
    Vector2D FindLookaheadPoint(BezierCurve &path_, float tNearest, float &tLookahead);
    void Path_correction(void);
    Vector2D ComputeLookaheadDiffFeedforward(bool near_end);
    void ResetAutoControlStates(void);
    Vector2D ComposeRobotVelocity(const Vector2D &v_pid, const Vector2D &v_ff_ref, bool near_end);
    void KFS_Selection_Planning(void);
    void Clamping_Bar_Selection_Planning(void);
    void flag_reset(void);
};

#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H
