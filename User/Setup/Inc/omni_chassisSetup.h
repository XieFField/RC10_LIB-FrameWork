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
#include "Module_Camera.h"
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

#define s_debug 0
#define FF_V 0

typedef struct
{
    bool spin_flag = false; // 是否需要执行中途转向。

    bool get_spin_flag = false; // 旋转触发过渡标志。

    bool Spin_Start = false; // 当前正在执行旋转。

    bool spin_up_flag = false;   // 上路段旋转流程使能。
    bool spin_down_flag = false; // 下路段旋转流程使能。

    bool MF1_flag = false;   // 进入 MF1 目标点标志。
    bool MF2_flag = false;   // 进入 MF2 目标点标志。
    bool MF1_finish = false; // MF1 阶段已完成标志。
} KFS_FLAG;

typedef struct
{
    bool Selection_flag = false; // 进入 CB 目标点标志
    bool Retreat_flag = false;   // 进入 CB 停止点标志
} CB_FLAG;

class OmniChassis_Setup : public RtosTask, public Chassis_Omni<3>
{
public:
    // 通过轮系几何参数构造底盘任务对象。
    OmniChassis_Setup(float wheel_radius, float max_wheel_rpm, float base_length, float side_length, bool three_wheel)
        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<3>(wheel_radius, max_wheel_rpm, base_length, side_length, three_wheel), debug_uart(&huart8)
    {
        // yaw 环使用角度循环误差（跨 ±180 度连续）。
        yaw_pid_.set_as_circular();
    }

    // 通过配置结构体构造底盘任务对象。
    OmniChassis_Setup(Chassis_Omni<3>::init_config &config)
        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<3>(config), debug_uart(&huart8)
    {
        // yaw 环使用角度循环误差（跨 ±180 度连续）。
        yaw_pid_.set_as_circular();
    }



    // 初始化底盘控制器参数并启动 RTOS 任务。
    void init()
    {
        if (this->wheels_[0] == nullptr || this->wheels_[1] == nullptr ||
            this->wheels_[2] == nullptr || this->wheels_[3] == nullptr)
            init_flag = false;

        yaw_pid_.set_params(lock_angle_pid_params, 10000.0f);

        this->setThreeWheelSolver(true);

        pid_pos_x.set_params(track_pid_params, 0.0f);
        pid_pos_y.set_params(track_pid_params, 0.0f);
        path_lock.set_params(path_lock_end, 0.0f);

        this->start(osPriorityHigh, 1024);
        //        setTargetKFS(3);
        init_flag = true;
#ifdef s_debug
        TP_1d.param_reset(Param_1d);
#endif
    }

    // 设置底盘正反向映射系数（用于手动控制方向翻转）。
    void setChassisReverse(bool isReverse)
    {
        if (!isReverse)
            this->is_chassis_reverse_ = 1.0f;
        else
            this->is_chassis_reverse_ = -1.0f;
    }

private:
    Vector2D test_point = {0.6f, 6.0f};
    Vector2D control_point = {0.0f, 2.5f};
    float err_curve = 0.0f;
    int num = 0;

    //-----------------------------------通讯标志位-----------------------------------------//
    CHASSIS_Status_E chassis_status_ = CHASSIS_STOP; // 当前底盘总状态机状态。

    bool WeaponSage_Start = false; // 夹杆流程开始标志。
    bool WeaponSage_End = false;   // 夹杆流程完成标志。

    bool Arm_Start = false; // 机械臂动作触发标志。

    int flag = 0;     // 自动流程起始触发位（边沿触发）。
    int flag_run = 0; // 自动流程运行中标志位。

    //-----------------------------------接口监视参数-----------------------------------------//
    
    
    int8_t MF1 = 0; // 目标点 1 编号。
    int8_t MF2 = 0; // 目标点 2 编号。

    Vector2D planspeed = {0.0f, 0.0f};    // 路径规划输出的最大速度。
    Vector2D corrVelocity = {0.0f, 0.0f}; // 计算出的横向纠偏速度向量
    Vector2D speed = {0.0f, 0.0f};        // 合成后的底盘平移速度。

    Vector2D robot_pos_ = {0.0f, 0.0f}; // 当前机器人世界坐标。
    float yaw = 0.0f;                   // 当前机器人航向角（度）。

    Point3D ladar_data_; // 定位系统输出的原始位姿数据。
    
    float PID_coefficient = 0.8;
    float FF_coefficient = 0.8;

    PID_Position pid_pos_x; // x轴绝对位置PID控制器
    PID_Position pid_pos_y; // y轴绝对位置PID控制器
    PID_Position path_lock; // 停止锁点

    //---------------------------接口调试参数（需要修改时复制过来）---------------------------------------------//

    float v_normal_max = 0.5f; // 常规段地板速度限制。

    // 夹取kfs前的速度限制，用来控制变量
    // float robot_speed_end_ = 0.3f;  // 终点段最大速度限制。
    // float deadzone_max_end_ = 0.1f; // 判定“近终点”阈值。

    float m_lookaheadDist = 0.5f;       // 前视距离 (单位: 米)
    float m_lookaheadDist_line = 0.5f;  // 前视距离 (单位: 米)
    float m_lookaheadDist_curve = 0.07f; // 前视距离 (单位: 米)
    
    //-----------------------------------速度规划参数----------------------------------------------------//
    
    BezierCurve curve; // 当前路径曲线缓存。

    Path_line path_line_; // 路径规划器对象。
    
    Vector2D CB_Selection_start_point_ = {1.0f, 1.0f}; // 夹杆流程默认目标点。
    Vector2D CB_Selection_control_point_ = {2.555f, 2.03f}; // 夹杆流程默认目标点。
    Vector2D Clamping_Bar_Selection_pos_ = {2.455f, 0.83f}; // 夹杆流程默认目标点。
    Vector2D Clamping_Bar_Retreat_pos_ = {2.4f, 1.0f};    // 夹杆流程默认目标点。

    Speedplanner_1D_Param_Config path_param_KFS_ = {.maxAcc = 30.0f, .maxDec = 40.0f, .maxJerk = 0.0f, .maxSpeed = 0.6f, .initialSpeed = 0.3f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.001f}; // KFS 速度规划参数。
    Speedplanner_1D_Param_Config path_param_CB_ = {.maxAcc = 999.0f, .maxDec = 1.0f, .maxJerk = 0.0f, .maxSpeed = 2.5f, .initialSpeed = 0.5f, .finalSpeed = 0.0f, .startPos = 0.10f, .targetPos = 0.0f, .deadzone = 0.001f}; // 夹杆流程速度规划参数。

    Speedplanner_1D_Param_Config path_param_start_ = {.maxAcc = 0.5f, .maxDec = 0.5f, .maxJerk = 0.0f, .maxSpeed = 1.0f, .initialSpeed = 0.01f, .finalSpeed = 0.5f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.001f}; // KFS 速度规划参数。
    Speedplanner_1D_Param_Config path_param_line_ = {.maxAcc = 0.5f, .maxDec = 0.5f, .maxJerk = 0.0f, .maxSpeed = 1.0f, .initialSpeed = 0.5f, .finalSpeed = 0.5f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.001f};   // KFS 速度规划参数。
    Speedplanner_1D_Param_Config path_param_curve_ = {.maxAcc = 0.0f, .maxDec = 0.0f, .maxJerk = 0.0f, .maxSpeed = 0.5f, .initialSpeed = 0.5f, .finalSpeed = 0.5f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.001f};  // KFS 速度规划参数。
    Speedplanner_1D_Param_Config path_param_end_ = {.maxAcc = 0.5f, .maxDec = 0.5f, .maxJerk = 0.0f, .maxSpeed = 1.0f, .initialSpeed = 0.5f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.001f};    // KFS 速度规划参数。

    //-----------------------------------梅林规划参数-----------------------------------------//
    CB_FLAG CB_flag;
    KFS_FLAG KFS_flag;

    MF_AutoCtrler::PathInformation_S KFS_KeyPoint_; // 自动规划输出的关键路径信息。

    Vector2D MF1_pos_ = {0.0f, 0.0f};
    Vector2D MF2_pos_ = {0.0f, 0.0f};

    float MF2_target_yaw_ = 0.0f; // 第二目标点对应目标朝向。

    Vector2D spin_point_ = {3.0f, 8.72f}; // 上方旋转点

    float spin_skew_ = -0.1f; // 下方旋转位置y轴偏移量

    //-----------------------------------yaw角控制参数-----------------------------------------//

    float target_yaw_ = 0.0f; // 底盘锁角目标（度）。

    PID_Position yaw_pid_; // yaw 角度环控制器。

    float is_chassis_reverse_ = 1.0f; // 手动控制正反向系数。
    //-----------------------------------其他参数-----------------------------------------//
    void loop() override; // RTOS 主循环。

    bool init_flag = false; // 初始化完成标志。

    RmPocketData_t airjoy_data_; // 遥控器数据，范围 -1 ~ 1

    Debug_Printf debug_uart = Debug_Printf(&huart8);                          // 调试串口
    Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // 当前周期底盘目标姿态。

    //-----------------------------------内部控制函数-----------------------------------------//

    Vector2D GetPathNearestPoint(BezierCurve &path_, const Vector2D &robotPos, float &tNearest);

    Vector2D FindLookaheadPoint(BezierCurve &path_, float tNearest, float &tLookahead);

    void KFS_Selection_Planning(void); // 生成 KFS 自动路径。

    void Path_correction(void); // 基于当前位置执行路径纠偏。

    void Path_spin_check(void); // 检查并执行路径中旋转逻辑。

    Vector2D v_limit(Vector2D &plan_v,Vector2D &pid_v); // 速度限幅函数。

    void flag_reset(void); // 复位自动流程相关标志位。

    void Clamping_Bar_Selection_Planning(void); // 生成夹杆流程路径。

    void Path_CB_check(void);

#ifdef s_debug
    int a = 0;
    float tp_speed_now = 0.0f;
    float tp_pos_now = 0.0f;
    // 1D的位置式
    Speedplanner_1D_Param_Config Param_1d = {.maxAcc = 60.0f, .maxDec = 1.2f, .maxJerk = 20.0f, .maxSpeed = 3.0f, .initialSpeed = 0.5f, .finalSpeed = 0.0f, .startPos = 0.15f, .targetPos = 0.0f, .deadzone = 0.001f}; // 夹杆流程速度规划参数。
    SShapedPlanner1D TP_1d;
#endif

    //-----------------------------------前馈参数-----------------------------------------//
#if FF_V
    float k_damp_ = 0.0f; // 历史速度阻尼系数。
    // 前视点差分前馈增益（越大越“冲”，也更容易抖）。
    float kff_la_ = 0.0f;
    // 前馈限幅（m/s），用于约束尖峰。
    float max_ff_speed_ = 1.0f;
    // 一阶低通系数，范围(0,1]：越小越平滑，越大越灵敏。
    float ff_lpf_alpha_ = 0.20f;

    float end_ff_scale_ = 0.35f; // 终点段前馈缩放系数。
    float end_pid_scale_ = 0.7f; // 终点段 PID 缩放系数。

    // 用于前视点差分前馈的“参考点”：
    // 正常跟踪阶段等于 lookaheadPt，终点阶段等于 endPt。
    Vector2D ff_ref_point_ = {0.0f, 0.0f};
    // 保存上一周期参考点，做离散差分 (p[k]-p[k-1]) / dt。
    Vector2D ff_ref_point_last_ = {0.0f, 0.0f};
    // 低通后的前馈速度，抑制 t 跳变和离散噪声导致的尖峰。
    Vector2D ff_velocity_lpf_ = {0.0f, 0.0f};
    // 前馈差分初始化标志，避免首周期使用无效差分。
    bool ff_diff_inited_ = false;

    // 控制任务周期（当前系统 1ms 调度）。
    float control_period_s_ = 0.001f;
    // 差分最小时间，避免 dt 太小导致数值爆发。
    float ff_dt_min_s_ = 0.0009f;
    // 差分最大时间，避免任务异常延迟后一次性放大速度脉冲。
    float ff_dt_max_s_ = 0.010f;

    Vector2D v_robot_last_cmd_ = {0.0f, 0.0f}; // 上一周期底盘速度命令。
    // 统一清空自动控制相关内部状态（速度命令记忆与前馈差分状态）。
    void ResetAutoControlStates(void);

    Vector2D ComposeRobotVelocity(const Vector2D &v_pid); // 合成 PID、前馈、阻尼后的速度命令。

#endif
public:
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

    bool GetReach_flag()
    {
        // 读取夹杆流程完成标志。
        return WeaponSage_Start;
    }
    
    bool GetEnd_flag()
    {
        // 读取夹杆流程完成标志。
        return WeaponSage_End;
    }
    
    void ReceiveReach_flag(bool weapon_end)
    {
        // 写入机械臂流程反馈标志。
        WeaponSage_Start = weapon_end;
    }

    bool Get_Arm_Start_flag()
    {
        // 读取机械臂触发标志。
        return Arm_Start;
    }

    void Receive_Arm_End_flag(bool arm_end)
    {
        // 写入机械臂流程反馈标志。
        Arm_Start = arm_end;
    }

    void set_KFS(int8_t KFS1, int8_t KFS2)
    {
        // 更新自动规划目标点编号。
        MF1 = KFS1;
        MF2 = KFS2;
    }
    // 统一切换底盘状态，并在相机流程切入/切出时清理相关内部状态。
    void setChassisStatus(CHASSIS_Status_E status)
    {
        // 最后写入底盘总状态。
        chassis_status_ = status;
    }
};
#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H