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
        if (this->wheels_[0] == nullptr || this->wheels_[1] == nullptr ||
            this->wheels_[2] == nullptr || this->wheels_[3] == nullptr)
            init_flag = false;

        yaw_pid_.set_params(lock_angle_pid_params, 10000.0f);

        this->setThreeWheelSolver(true);

#if debug_ladar
        this->setThreeWheelSolver(false);
#endif
        pid_pos_x.set_params(track_pid_params, 0.0f);
        pid_pos_y.set_params(track_pid_params, 0.0f);

        this->start(osPriorityHigh, 1024);
        //        setTargetKFS(3);
        init_flag = true;
    }

    void setChassisReverse(bool isReverse)
    {
        if (!isReverse)
            this->is_chassis_reverse_ = 1.0f;
        else
            this->is_chassis_reverse_ = -1.0f;
    }

    /**
     * @brief 设置路径自动开始标志
     * @param start 1表示开始，0表示停止
     * @param path_flagIndex 路径标志索引，0或1
     */
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
    //-----------------------------------通讯标志位-----------------------------------------//
    bool WeaponSage_END = 0;

    bool init_flag = false;

    bool Arm_Start = false;

    CHASSIS_Status_E chassis_status_ = CHASSIS_STOP;
    //-----------------------------------速度规划参数-----------------------------------------//

    int flag = 0;
    int flag_run = 0;

    Path_line path_line_;
    Vector2D Clamping_Bar_Selection_pos_ = {1.925f + 0.48f, 0.19f + 0.50f};

    Speedplanner_1D_Param_Config path_param_KFS_ = {.maxAcc = 30.0f, .maxDec = 40.0f, .maxJerk = 100.0f, .maxSpeed = 0.6f, .initialSpeed = 0.3f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f};
    Speedplanner_1D_Param_Config path_param_CB_ = {.maxAcc = 5.0f, .maxDec = 5.0f, .maxJerk = 0.0f, .maxSpeed = 0.75f, .initialSpeed = 0.3f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f};

    // float Acc_target_yaw_ = 0.0f;
    // ConstantAcc Acc_yaw_{0.1f,0.0f}; // 注意代码运行系统的周期
    // Vector2D original_point_={-0.48f,-0.50f};

    //-----------------------------------接口监视参数-----------------------------------------//

    Point3D ladar_data_;
    Vector2D robot_pos_ = {0.0f, 0.0f};
    Point2D robot_point_ = {0.0f, 0.0f};

    Vector2D planspeed;
    Vector2D speed;
    Vector2D corrVelocity = {0.0f, 0.0f}; // 计算出的横向纠偏速度向量

    PID_Position pid_pos_x; // x轴绝对位置PID控制器
    PID_Position pid_pos_y; // y轴绝对位置PID控制器

    Robot_Twist last_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    //-----------------------------------前视pid参数-----------------------------------------//

    float tNearest = 0.0f;   // 最近点在贝塞尔曲线上的参数t (0~1)
    float tLookahead = 0.0f; // 前视点在贝塞尔曲线上的参数t (0~1)

    float max_robot_speed_ = 1.5f;
    float max_robot_speed_end_ = 0.4f;
    float t_deadzone = 0.93f;
    float max_corr_end_ = 0.5f;

    Vector2D nearestPt;        // 路径上距离机器人最近的点
    Vector2D lookaheadPt;      // 路径上的前视点
    Vector2D lookaheadTangent; // 前视点处的切线方向向量
    Vector2D pathEnd;          // 路径终点坐标

    BezierCurve curve;

    //-----------------------------------梅林规划参数-----------------------------------------//
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

    Vector2D spin_point_ = {3.6f, 8.72f}; // 上方旋转点
    float spin_skew_ = -0.1f;             // 下方旋转位置y轴偏移量
    bool get_spin_flag = false;
    bool Spin_Start = false;
    //-----------------------------------yaw角控制参数-----------------------------------------//

    float yaw = 0.0f;
    float target_yaw_ = 0.0f;

    uint8_t yaw_pid_period_ = 3;
    uint8_t yaw_pid_period_count_ = 0;
    PID_Position yaw_pid_;

    const float LINESPEED_LIMIT = 10 / 500.f; // 线速度限制
    const float YAWSPEED_LIMIT = 1 / 500.f;   // yaw速度限制

    float is_chassis_reverse_ = 1.0f;
    
    
    //-----------------------------------前馈参数-----------------------------------------//

    // 用于前视点差分前馈的“参考点”：
    // 正常跟踪阶段等于 lookaheadPt，终点阶段等于 endPt。
    Vector2D ff_ref_point_ = {0.0f, 0.0f};
    // 保存上一周期参考点，做离散差分 (p[k]-p[k-1]) / dt。
    Vector2D ff_ref_point_last_ = {0.0f, 0.0f};
    // 低通后的前馈速度，抑制 t 跳变和离散噪声导致的尖峰。
    Vector2D ff_velocity_lpf_ = {0.0f, 0.0f};
    // 前馈差分初始化标志，避免首周期使用无效差分。
    bool ff_diff_inited_ = false;

    float m_lookaheadDist = 0.3f; // 前视距离 (单位: 米)
    // 前视点差分前馈增益（越大越“冲”，也更容易抖）。
    float kff_la_ = 0.0f;
    // 一阶低通系数，范围(0,1]：越小越平滑，越大越灵敏。
    float ff_lpf_alpha_ = 0.20f;
    // 控制任务周期（当前系统 1ms 调度）。
    float control_period_s_ = 0.001f;
    // 差分最小时间，避免 dt 太小导致数值爆发。
    float ff_dt_min_s_ = 0.0009f;
    // 差分最大时间，避免任务异常延迟后一次性放大速度脉冲。
    float ff_dt_max_s_ = 0.010f;
    // 前馈限幅（m/s），用于约束尖峰。
    float max_ff_speed_ = 1.0f;
    Vector2D v_robot_last_cmd_ = {0.0f, 0.0f};
    // float kff_ref_ = 0.8f;
    float k_damp_ = 0.0f;
    float end_ff_scale_ = 0.35f;
    float end_pid_scale_ = 0.7f;
    
    //-----------------------------------其他参数-----------------------------------------//

    RmPocketData_t airjoy_data_; // 遥控器数据，范围 -1 ~ 1

    Debug_Printf debug_uart = Debug_Printf(&huart8); // 调试串口

    //-----------------------------------内部控制函数-----------------------------------------//

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

    void KFS_Selection_Planning(void);

    void Path_correction(void);

    // 基于“前视参考点差分”的前馈计算：
    // v_ff_raw = kff_la_ * (p_ref[k]-p_ref[k-1]) / dt
    // 并叠加低通、限幅和终点段衰减。
    Vector2D ComputeLookaheadDiffFeedforward(bool near_end);

    // 统一清空自动控制相关内部状态（速度命令记忆与前馈差分状态）。
    void ResetAutoControlStates(void);

    Vector2D ComposeRobotVelocity(const Vector2D &v_pid, const Vector2D &v_ff_ref, bool near_end);

    void Clamping_Bar_Selection_Planning(void);

    void flag_reset(void);

    void loop() override;
};
#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H