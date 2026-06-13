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

#define opti 1

typedef struct
{
    Speedplanner_1D_Param_Config line = {.maxAcc = 20.0f, .maxDec = 0.9f, .maxJerk = 0.0f, .maxSpeed = 2.5f, .initialSpeed = 0.8f, .finalSpeed = 1.0f, .startPos = 0.05f, .targetPos = 0.0f, .deadzone = 0.001f};

    Speedplanner_1D_Param_Config start = {.maxAcc = 20.0f, .maxDec = 0.9f, .maxJerk = 0.0f, .maxSpeed = 2.5f, .initialSpeed = 0.8f, .finalSpeed = 0.8f, .startPos = 0.05f, .targetPos = 0.0f, .deadzone = 0.001f};
    Speedplanner_1D_Param_Config curve = {.maxAcc = 0.0f, .maxDec = 0.0f, .maxJerk = 0.0f, .maxSpeed = 0.8f, .initialSpeed = 0.8f, .finalSpeed = 0.8f, .startPos = 0.0f, .targetPos = 999.0f, .deadzone = 0.001f};
    Speedplanner_1D_Param_Config end = {.maxAcc = 20.0f, .maxDec = 0.9f, .maxJerk = 0.0f, .maxSpeed = 2.5f, .initialSpeed = 0.8f, .finalSpeed = 0.001f, .startPos = 0.05f, .targetPos = 0.0f, .deadzone = 0.001f};

    Speedplanner_1D_Param_Config up = {.maxAcc = 20.0f, .maxDec = 0.9f, .maxJerk = 0.0f, .maxSpeed = 2.0f, .initialSpeed = 2.0f, .finalSpeed = 0.6f, .startPos = 0.05f, .targetPos = 0.0f, .deadzone = 0.001f};

    // 没用的
    //Speedplanner_1D_Param_Config KFS = {.maxAcc = 999.0f, .maxDec = 0.8f, .maxJerk = 0.0f, .maxSpeed = 2.5f, .initialSpeed = 0.5f, .finalSpeed = 0.15f, .startPos = 0.25f, .targetPos = 0.0f, .deadzone = 0.001f};
    // 原始的测试数据
    //Speedplanner_1D_Param_Config CB = {.maxAcc = 20.0f, .maxDec = 0.9f, .maxJerk = 0.0f, .maxSpeed = 2.5f, .initialSpeed = 0.8f, .finalSpeed = 0.001f, .startPos = 0.05f, .targetPos = 0.0f, .deadzone = 0.001f};

} PATH_PARAM;

typedef struct
{
    Vector2D planspeed = {0.0f, 0.0f};    // 路径规划输出的最大速度。
    Vector2D corrVelocity = {0.0f, 0.0f}; // 计算出的横向纠偏速度向量

    float PID_coefficient = 1.0;
    float FF_coefficient = 0.0;

    float v_normal_max = 0.5f;
    float m_lookaheadDist = 0.4f; // 前视距离
} SPEED_PARAM;

typedef struct
{
    float CB_spiw =0.5f;
    float CB_pole_L =-0.18f;
    float CB_pole_M =-0.02f;
    float CB_pole_R =0.22f;
    Vector2D CB_Start_pos = {1.0f, 0.9f};        // 夹杆起点。
    Vector2D CB_Selection_pos = {2.47f, 0.815f}; // 夹杆流程默认目标点。
    Vector2D CB_End_pos = {2.745f, 1.185f};
    //Vector2D CB_End_pos = {4.0f-CB_spiw-0.4f-0.49f, 2.0f+CB_pole_L};  // 夹杆终点对接默认目标点。(4.0为R2对接车体位置，0.5为夹杆预留长度，0.4为R2车体半径，0.49为R1车体半径）
} CB_POINT;

typedef struct
{
    int8_t MF1 = 0; // 目标点 1 编号。
    int8_t MF2 = 0; // 目标点 2 编号。
    int8_t MF3 = 0; // 目标点 2 编号。

    Vector2D MF1_pos_ = {0.0f, 0.0f};
    Vector2D MF2_pos_ = {0.0f, 0.0f};
    Vector2D MF3_pos_ = {0.0f, 0.0f};

    Vector2D spin_pos = {0.0f, 0.0f};   // 是否需要执行中途转向。
    Vector2D spin_pos_2 = {0.0f, 0.0f}; // 是否需要执行中途转向。

    float MF2_target_yaw_ = 0.0f; // 第二目标点对应目标朝向。
    float MF3_target_yaw_ = 0.0f; // 第二目标点对应目标朝向。

    float spin_skew = 0.1f; // 旋转位置y轴偏移量

    float coner_ahead = 0.17f;
    float coner_behind = 0.4f;

} KFS_POINT;

typedef struct
{
    Vector2D uphill_pos = {0.6f, 11.4f};
    Vector2D fit_pos = {4.83f, 11.5f};

    float fit_yaw = 180.0f;

    // 左中右
    Vector2D R1_pos[3] = {{4.535f, 11.285f}, {4.535f, 10.705f}, {4.535f, 10.185f}};
    Vector2D R2_pos[3] = {{4.535f, 11.285f}, {4.535f, 10.705f}, {4.535f, 10.185f}};

} CZ_POINT;

typedef struct
{
    bool get_spin_flag = false; // 旋转触发过渡标志。

    bool spin_flag = false;   // 是否需要执行中途转向。
    bool spin_flag_2 = false; // 是否需要执行中途转向。

    bool MF1_flag = false; // 进入 MF1 目标点标志。
    bool MF2_flag = false; // 进入 MF2 目标点标志。
    bool MF3_flag = false; // 进入 MF3 目标点标志。

    bool MF1_finish = false; // MF1 阶段已完成标志。
    bool MF2_finish = false; // MF1 阶段已完成标志。
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
    }

    // 通过配置结构体构造底盘任务对象。
    OmniChassis_Setup(Chassis_Omni<3>::init_config &config)
        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<3>(config), debug_uart(&huart8)
    {
    }

    // 初始化底盘控制器参数并启动 RTOS 任务。
    void init()
    {
        if (this->wheels_[0] == nullptr || this->wheels_[1] == nullptr ||
            this->wheels_[2] == nullptr || this->wheels_[3] == nullptr)
            init_flag = false;

        this->setThreeWheelSolver(true);

        pid_pos_x.set_params(track_pid_params, 0.0f);
        pid_pos_y.set_params(track_pid_params, 0.0f);
        path_lock.set_params(path_lock_end, 0.0f);

        this->start(osPriorityHigh, 1024);
        //        setTargetKFS(3);
        init_flag = true;
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
    // Vector2D test_point = {3.0f, 2.0f};
    //-----------------------------------通讯标志位-----------------------------------------//
    CHASSIS_Status_E chassis_status_ = CHASSIS_STOP; // 当前底盘总状态机状态。
    CHASSIS_Status_E chassis_status_last_ = CHASSIS_STOP; // 当前底盘总状态机状态。（依旧是每个模式都赋值，用于进入自动模式时进行初始化）

    bool WeaponSage_Start = false; // 夹杆流程开始标志。
    bool WeaponSage_End = false;   // 夹杆流程完成标志。

    bool Arm_Start = false; // 机械臂动作触发标志。

    bool RB_Flag = true; // 红蓝方标志位，默认true为现场地

    bool pid_dead_flag = false; // pid完成标志

    int flag = 0;     // 自动流程起始触发位（边沿触发）。
    //int flag_run = 0; // 自动流程运行中标志位。

    //-----------------------------------接口监视参数-----------------------------------------//

    Vector2D speed = {0.0f, 0.0f};      // 合成后的底盘平移速度。
    Vector2D robot_pos_ = {0.0f, 0.0f}; // 当前机器人世界坐标。
    float yaw = 0.0f;                   // 当前机器人航向角（度）。
    float target_yaw = 0.0f;           // 底盘锁角目标（度）。（手操模式要把当前值赋值进来，以便于自动切换时不会抽风）

    //-----------------------------------规划参数-----------------------------------------//
    CB_FLAG CB_flag;
    CB_POINT CB_point;

    KFS_FLAG KFS_flag;
    KFS_POINT KFS_point;

    CZ_POINT CZ_point;

    //-----------------------------------速度规划参数--------------------------------------------//

    SPEED_PARAM V;
    PATH_PARAM path_param;

    PID_Position pid_pos_x; // x轴绝对位置PID控制器
    PID_Position pid_pos_y; // y轴绝对位置PID控制器
    PID_Position path_lock; // 停止锁点

    BezierCurve curve; // 当前路径曲线缓存。

    Path_line path_line_; // 路径规划器对象。

    //-----------------------------------其他参数-----------------------------------------//
    int num = 0;
    //int start_num = 0;

    Point3D ladar_data_; // 定位系统输出的原始位姿数据。

    Vector2D Path_end_point = {0.0f, 0.0f};

    float is_chassis_reverse_ = 1.0f; // 手动控制正反向系数。

    bool init_flag = false; // 初始化完成标志。

    RmPocketData_t airjoy_data_;                    // 遥控器数据，范围 -1 ~ 1
    MF_AutoCtrler::PathInformation_S KFS_KeyPoint_; // 自动规划输出的关键路径信息。

    Debug_Printf debug_uart = Debug_Printf(&huart8);                          // 调试串口
    Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // 当前周期底盘目标姿态。

    //-----------------------------------内部控制函数-----------------------------------------//
    void loop() override; // RTOS 主循环。

    bool KFS_Selection_Planning(void); // 生成 KFS 自动路径。

    void Path_correction(void); // 基于当前位置执行路径纠偏。

    void Path_KFS_check(void); // 检查并执行路径中旋转逻辑。

    Vector2D v_limit(void); // 速度限幅函数。

    void flag_reset(void); // 复位自动流程相关标志位。

    void Clamping_Bar_Selection_Planning(void); // 生成夹杆流程路径。

    void Path_CB_check(void);

    Vector2D spinodal_path(Vector2D last_vector, Vector2D temp_vector, int i, float spin_flag);

    float rotation_path(float MF_Point);
    
    void Path_lock_point(Vector2D lock_point);

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

//        if (start == 0)
//        {
//            start_num++;
//            if (start_num == 3)
//                flag_run = 0;
//        }
//        else
//        {
//            start_num = 0;
//        }
    }

    bool GetReach_flag()
    {
        // 读取夹杆流程完成标志。
        if (pid_dead_flag == true && WeaponSage_Start == true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool GetEnd_flag()
    {
        // 读取夹杆退后流程完成标志。
        if (pid_dead_flag == true && WeaponSage_End == true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void ReceiveReach_flag(bool weapon_end)
    {
        // 写入机械臂流程反馈标志。
        WeaponSage_Start = weapon_end;
    }

    bool Get_Arm_Start_flag()
    {
        // 读取机械臂触发标志。
        if (pid_dead_flag == true && Arm_Start == true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void Receive_Arm_End_flag(bool arm_end)
    {
        // 写入机械臂流程反馈标志。
        Arm_Start = arm_end;
    }

    void set_KFS(int8_t KFS1, int8_t KFS2, int8_t KFS3)
    {
        // 更新自动规划目标点编号。
        KFS_point.MF1 = KFS1;
        KFS_point.MF2 = KFS2;
        KFS_point.MF3 = KFS3;
    }
    // 统一切换底盘状态
    void setChassisStatus(CHASSIS_Status_E status)
    {
        // 最后写入底盘总状态。
        chassis_status_ = status;
    }
};
#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H