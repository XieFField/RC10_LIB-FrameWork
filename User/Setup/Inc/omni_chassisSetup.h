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
#include "Module_lora.h"

typedef struct
{
    float VX = 0.0f;
    float VY = 0.0f;
    float yaw_rate = 0.0f;
} CHASSIS_TARGET;

typedef struct
{  
    Speedplanner_1D_Param_Config line = {.maxAcc = 20.0f, .maxDec = 0.9f, .maxJerk = 0.0f, .maxSpeed = 1.2f, .initialSpeed = 0.6f, .finalSpeed = 1.2f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.001f};
	Speedplanner_1D_Param_Config cb = {.maxAcc = 20.0f, .maxDec = 0.9f, .maxJerk = 0.0f, .maxSpeed = 1.2f, .initialSpeed = 1.2f, .finalSpeed = 0.001f, .startPos = 0.08f, .targetPos = 0.0f, .deadzone = 0.001f};
	Speedplanner_1D_Param_Config speed = {.maxAcc = 20.0f, .maxDec = 1.3f, .maxJerk = 0.0f, .maxSpeed = 4.0f, .initialSpeed = 0.6f, .finalSpeed = 0.01f, .startPos = 0.02f, .targetPos = 0.0f, .deadzone = 0.001f};

    Speedplanner_1D_Param_Config start = {.maxAcc = 20.0f, .maxDec = 0.9f, .maxJerk = 0.0f, .maxSpeed = 2.5f, .initialSpeed = 0.6f, .finalSpeed = 0.8f, .startPos = 0.05f, .targetPos = 0.0f, .deadzone = 0.001f};
    Speedplanner_1D_Param_Config curve = {.maxAcc = 0.0f, .maxDec = 0.0f, .maxJerk = 0.0f, .maxSpeed = 0.8f, .initialSpeed = 0.8f, .finalSpeed = 0.8f, .startPos = 0.0f, .targetPos = 999.0f, .deadzone = 0.001f};
    Speedplanner_1D_Param_Config end = {.maxAcc = 20.0f, .maxDec = 0.9f, .maxJerk = 0.0f, .maxSpeed = 2.5f, .initialSpeed = 0.8f, .finalSpeed = 0.001f, .startPos = 0.06f, .targetPos = 0.0f, .deadzone = 0.001f};

    Speedplanner_1D_Param_Config up = {.maxAcc = 20.0f, .maxDec = 0.9f, .maxJerk = 0.0f, .maxSpeed = 2.0f, .initialSpeed = 2.0f, .finalSpeed = 0.6f, .startPos = 0.05f, .targetPos = 0.0f, .deadzone = 0.001f};
    
	Speedplanner_1D_Param_Config R2 = {.maxAcc = 20.0f, .maxDec = 0.9f, .maxJerk = 0.0f, .maxSpeed = 2.5f, .initialSpeed = 0.8f, .finalSpeed = 0.001f, .startPos = 0.08f, .targetPos = 0.0f, .deadzone = 0.001f};
    
} PATH_PARAM;

typedef struct
{
    Vector2D planspeed = {0.0f, 0.0f};    // 路径规划输出的最大速度。
    Vector2D corrVelocity = {0.0f, 0.0f}; // 计算出的横向纠偏速度向量

    float PID_coefficient = 1.0f;
    float FF_coefficient = 1.0f;
    float spinodal_coefficient = 1.2f;

    float v_normal_max = 0.5f;
    float m_lookaheadDist = 0.4f; // 前视距离
} SPEED_PARAM;

typedef struct
{
    //float spin_y=1.0f;
	float spin_y=0.86f;
    
    // 第数组第零为红场
    Vector2D CB_Start_pos[2] = {{5.0f, 0.9f}, {1.0f, 0.85f}};             // 夹杆起点。
    Vector2D CB_Selection_pos[2] = {{3.466f, 0.835f}, {2.455f, 0.835f}}; // 夹杆流程默认目标点。

    // 相机流程
    Vector2D CB_End_pos[2] = {{3.539f, 1.085f}, {2.461f, 1.085f}};

    // 贴边流程
    Vector2D CB_transition_pos[2] = {{3.539f, 1.01f}, {3.0f, 1.04f}};
    //Vector2D CB_transition_pos[2] = {{3.539f, 1.1f}, {3.0f, 1.4f}};
	
    // Vector2D CB_welt_pos[2] = {{3.3f, 0.5f}, {3.98f, 0.5f}};
    Vector2D CB_welt_pos[2] = {{3.3f, 0.52f}, {3.98f, 0.495f}};

} CB_POINT;

typedef struct
{
    // 内部的KFS位置，用于退出保存功能
    int8_t MF1 = 0; // 目标点 1 编号。
    int8_t MF2 = 0; // 目标点 2 编号。
    int8_t MF3 = 0; // 目标点 3 编号。
    
    // 接收外部的KFS位置，如果没有变化则不对MF进行赋值
    int8_t KFS1 = 0; // 目标点 1 编号。
    int8_t KFS2 = 0; // 目标点 2 编号。
    int8_t KFS3 = 0; // 目标点 3 编号。

    Vector2D MF1_pos_ = {0.0f, 0.0f};
    Vector2D MF2_pos_ = {0.0f, 0.0f};
    Vector2D MF3_pos_ = {0.0f, 0.0f};

    Vector2D spin_pos_0 = {0.0f, 0.0f}; // 是否需要执行中途转向。
    Vector2D spin_pos = {0.0f, 0.0f};   // 是否需要执行中途转向。
    Vector2D spin_pos_2 = {0.0f, 0.0f}; // 是否需要执行中途转向。

    float MF1_target_yaw_ = 0.0f; // 第二目标点对应目标朝向。
    float MF2_target_yaw_ = 0.0f; // 第二目标点对应目标朝向。
    float MF3_target_yaw_ = 0.0f; // 第二目标点对应目标朝向。

    float spin_skew = 0.1f;   // 旋转位置y轴偏移量
    float point_skew = 0.06f; // 旋转位置y轴偏移量

    float coner_ahead = 0.2f;
    float coner_behind = 0.5f;

} KFS_POINT;

typedef struct
{
    Vector2D uphill_pos[2] = {{5.4f, 11.4f}, {0.6f, 11.4f}};
    float skew_yaw = 1.7f;
    // 下界10.02f上界是11.52f
    Vector2D fit_wait_pos = {2.17f, 10.05f};
    Vector2D fit_transition_pos = {3.0f, 11.5f};
    Vector2D fit_end_pos = {4.83f, 11.5f};

    // 左中右   或者   先后
    float set_skew = 0.215f;
    Vector2D R1_pos[3][2] = {{{1.465f, 11.318f}, {4.43f, 11.318f}}, {{1.465f, 10.795f}, {4.43f, 10.795f}}, {{1.465f, 10.225f}, {4.43f, 10.225f}}};
    Vector2D fit_pos[3][2] = {{{6.0f - fit_wait_pos.x, fit_wait_pos.y}, fit_wait_pos}, {{6.0f - fit_end_pos.x, fit_end_pos.y}, fit_end_pos}, {{6.0f - fit_transition_pos.x, fit_transition_pos.y}, fit_transition_pos}};
    Vector2D R2_pos[3][2] = {{{1.17f, 11.318f}, {4.83f, 11.318f}}, {{1.17f, 10.795f}, {4.83f, 10.795f}}, {{1.17f, 10.225f}, {4.83f, 10.225f}}};

} CZ_POINT;

typedef struct
{
    bool get_spin_flag = false; // 旋转触发过渡标志。

    bool spin_flag_0 = false; // 是否需要执行中途转向。
    bool spin_flag = false;   // 是否需要执行中途转向。
    bool spin_flag_2 = false; // 是否需要执行中途转向。

    bool MF1_flag = false; // 进入 MF1 目标点标志。
    bool MF2_flag = false; // 进入 MF2 目标点标志。
    bool MF3_flag = false; // 进入 MF3 目标点标志。

    bool MF1_finish = false; // MF1 阶段已完成标志。
    bool MF2_finish = false; // MF2 阶段已完成标志。
    bool MF3_finish = false; // MF2 阶段已完成标志。

    bool pause_flag = false; // 暂停标志位

    // 为全局默认参数，不需要重置
    bool uphill_flag = true; // 默认KFS自动后上坡进入三区
} KFS_FLAG;

typedef struct
{
    bool Selection_flag = false;
    bool Retreat_flag = false;
} CB_FLAG;

typedef struct
{
    // 远中近的索引
    int R1_RL_index = 1;
    int R1_FB_index = 0;

    int fit_pos_index = 1;
    int R2_pos_index = -1;

} CZ_FLAG;

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
        path_lock_r2.set_params(path_lock_R2, 0.0f);

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

    void reset_CB_point(float x, float y)
    {
        CB_point.CB_Selection_pos[MF_AutoCtrler::get_color()].x = x;
        CB_point.CB_Selection_pos[MF_AutoCtrler::get_color()].y = y;
    }

private:
    // Vector2D test_point = {3.0f, 2.0f};
    //-----------------------------------通讯标志位-----------------------------------------//
    bool WeaponSage_Start = false; // 夹杆流程开始标志。
    bool WeaponSage_End = false;   // 夹杆流程完成标志。

    bool Arm_Start = false; // 机械臂动作触发标志。

    bool CZ_Arm = false; // 机械臂动作触发标志。

    bool RB_Flag = true; // 红蓝方标志位，默认true为蓝场

    bool pid_dead_flag = false; // pid完成标志

    int flag = 0; // 自动流程起始触发位（边沿触发）。

    //-----------------------------------接口监视参数-----------------------------------------//

    Vector2D speed = {0.0f, 0.0f};      // 合成后的底盘平移速度。
    Vector2D robot_pos_ = {0.0f, 0.0f}; // 当前机器人世界坐标。
    float yaw = 0.0f;                   // 当前机器人航向角（度）。
    float target_yaw = 0.0f;            // 底盘锁角目标（度）。（手操模式要把当前值赋值进来，以便于自动切换时不会抽风）

    //-----------------------------------规划参数-----------------------------------------//
    CB_FLAG CB_flag;
    CB_POINT CB_point;

    KFS_FLAG KFS_flag;
    KFS_POINT KFS_point;

    CZ_FLAG CZ_flag;
    CZ_POINT CZ_point;

    BezierCurve curve; // 当前路径曲线缓存。

    Path_line path_line_; // 路径规划器对象。
    //-----------------------------------速度规划参数--------------------------------------------//

    SPEED_PARAM V;
    PATH_PARAM path_param;

    PID_Position pid_pos_x;    // x轴绝对位置PID控制器
    PID_Position pid_pos_y;    // y轴绝对位置PID控制器
    PID_Position path_lock;    // 停止锁点
    PID_Position path_lock_r2; // 停止R2锁点

    //-----------------------------------其他参数-----------------------------------------//

    Vector2D Path_end_point = {0.0f, 0.0f};

    float is_chassis_reverse_ = 1.0f; // 手动控制正反向系数。

    bool init_flag = false; // 初始化完成标志。

    MF_AutoCtrler::PathInformation_S KFS_KeyPoint_; // 自动规划输出的关键路径信息。

    Debug_Printf debug_uart = Debug_Printf(&huart8);    // 调试串口
    CHASSIS_TARGET Chassis_Target = {0.0f, 0.0f, 0.0f}; // 底层对接接口缓冲区

    CHASSIS_Status_E chassis_status_ = CHASSIS_STOP;      // 当前底盘总状态机状态。
    CHASSIS_Status_E chassis_status_last_ = CHASSIS_STOP; // 当前底盘总状态机状态。（依旧是每个模式都赋值，用于进入自动模式时进行初始化）
	
	

#if !USE_RC10_AIRJOY
    RmPocketData_t airjoy_data_; // 摇杆值为 -1 ~ 1
#else
    communication::RC10_AirJoy_Data_S airjoy_data_;
#endif

    //-----------------------------------内部控制函数-----------------------------------------//
    void loop() override; // RTOS 主循环。

    void CB_Selection_Planning(void); // 生成夹杆流程路径。

    void CB_Path_Check(void);

    void CZ_R1_Selection_Planning(void);

    void CZ_R2_Selection_Planning(void);

    void CZ_FIT_Path_Init(void);

    void CZ_ARM_Path_Init(void);

    void CZ_index_reset(void);

    void CZ_FIT_WAIT_Selection_Planning(void);

    void CZ_FIT_R2_Selection_Planning(void);

    void CZ_ARM_Selection_Planning(void);
	
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
    }

    bool GetReach_flag()
    {
        // 读取夹杆流程完成标志。
        if (pid_dead_flag == true && WeaponSage_Start == true && (_tool_Abs(yaw - target_yaw) < 5.0f))
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
        if (WeaponSage_End == true && (_tool_Abs(yaw - target_yaw) < 7.0f))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void ReceiveReach_flag(bool weapon_start)
    {
        // 写入机械臂流程反馈标志。
        WeaponSage_Start = weapon_start;
    }

    void ReceiveEnd_flag(bool weapon_end)
    {
        // 写入机械臂流程反馈标志。
        WeaponSage_End = weapon_end;
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

    bool Get_CZ_Arm_flag()
    {
        // 读取机械臂触发标志。
        if (pid_dead_flag == true && CZ_Arm == true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void Receive_CZ_Arm_flag(bool CZ_end)
    {
        // 写入机械臂流程反馈标志。
        CZ_Arm = CZ_end;
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

    // 写一些辅助函数和标志位函数
private:
    //////////////////////////////////////////       路径纠偏      //////////////////////////////////////////////////////
    void Path_lock_point(Vector2D lock_point)
    {
        float lock_err = (robot_pos_ - lock_point).magnitude();
        if (airjoy_data_.SWA == 0x00 && chassis_status_ == CHASSIS_AUTO_CONTROL_CZ_R2)
        {
            speed = path_lock_r2.pid_calc(0.0f, lock_err) * (robot_pos_ - lock_point).normalize();
			pid_dead_flag = path_lock_r2.get_is_in_dead_zone();
			Chassis_Target.VX = speed.x;
			Chassis_Target.VY = speed.y;
        }
		else if(chassis_status_ == CHASSIS_AUTO_CONTROL_CB && WeaponSage_End == true)
		{
			Chassis_Target.VX = 0.0f;
			Chassis_Target.VY = 0.0f;
		}
		else if(chassis_status_ == CHASSIS_AUTO_CONTROL_CB && path_line_.Is_End() == true)
		{
			Chassis_Target.VX = 0.0f;
			Chassis_Target.VY = 0.0f;
		}		
        else
        {
            speed = path_lock.pid_calc(0.0f, lock_err) * (robot_pos_ - lock_point).normalize();
			pid_dead_flag = path_lock.get_is_in_dead_zone();
			Chassis_Target.VX = speed.x;
			Chassis_Target.VY = speed.y;
        }
        
    }

    void Path_correction(void)
    {
        float tNearest = 0.0f;   // 最近点在贝塞尔曲线上的参数t (0~1)
        float tLookahead = 0.0f; // 前视点在贝塞尔曲线上的参数t (0~1)

        curve.Get_Nearest_Distance(robot_pos_, &tNearest);

        Vector2D nearestPt = curve.Get_Point(tNearest);

        float obj_dis = _tool_Abs((curve.Get_End_point() - robot_pos_).magnitude());

        // ======== 终点纠偏（新架构下平滑退化为终点位置吸附）========
        if (obj_dis < V.m_lookaheadDist)
        {
            Vector2D endPt = curve.Get_End_point();

            if (curve.Get_len() < 0.0001f)
            {
                V.corrVelocity = {0.0f, 0.0f};
                return;
            }
            V.corrVelocity.x = pid_pos_x.pid_calc(endPt.x, robot_pos_.x);
            V.corrVelocity.y = pid_pos_y.pid_calc(endPt.y, robot_pos_.y);
            return;
        }

        Vector2D lookaheadPt; // 路径上的前视点

        tLookahead = tNearest; // 前视点的编号，先从最近点的编号开始（比如t=0.3）

        // 弧长表二分查找：利用 BezierCurve 已有的 distance_list[200]
        // Get_Current_Len 内部做 O(1) 查表+线性插值，二分替代逐点步进
        float current_len = curve.Get_Current_Len(tNearest);
        float target_len = current_len + V.m_lookaheadDist;

        if (target_len >= curve.Get_len())
        {
            // 前视距离超出曲线总长 → 直接用终点
            tLookahead = 1.0f;
            lookaheadPt = curve.Get_Point(1.0f);
        }
        else
        {
            float lo = tNearest, hi = 1.0f;
            for (int i = 0; i < 8; i++)
            {
                float mid = (lo + hi) * 0.5f;
                if (curve.Get_Current_Len(mid) < target_len)
                    lo = mid;
                else
                    hi = mid;
            }
            tLookahead = hi;
            lookaheadPt = curve.Get_Point(tLookahead);
        }
        pid_dead_flag = (pid_pos_y.get_is_in_dead_zone()&&pid_pos_y.get_is_in_dead_zone());

        // 3. 在绝对世界坐标系下，独立计算X轴和Y轴的纠偏向速度
        // 将不再计算切法向，直接基于XY差值PID
        V.corrVelocity.x = pid_pos_x.pid_calc(lookaheadPt.x, robot_pos_.x);
        V.corrVelocity.y = pid_pos_y.pid_calc(lookaheadPt.y, robot_pos_.y);
    }

    Vector2D v_limit(void)
    {
        static int num = 0;
        // 使用单位向量做正交分解，避免 |normal|² 缩放
        Vector2D tangent = path_line_.Get_Tangent_Vector();
        Vector2D tangent_dir = tangent.normalize();
        Vector2D normal_dir = tangent_dir.perpendicular();
        Vector2D v_tangent={0.0f,0.0f};
         
        if (path_line_.Get_Curve_Flag() == true && chassis_status_==CHASSIS_AUTO_CONTROL_KFS)
            V.corrVelocity = V.corrVelocity * V.spinodal_coefficient;

        // 切向 = 前馈 + PID纠偏沿切向分量（PID不限幅，终点 planspeed=0 时保留切向纠偏）
        v_tangent = V.planspeed * V.FF_coefficient + V.corrVelocity.project_onto(tangent_dir);

                
        if (v_tangent.magnitude() > V.planspeed.magnitude())
            v_tangent = v_tangent.normalize() * V.planspeed.magnitude();

        // 法向 = PID纠偏沿法向分量（限幅防止侧向过冲）
        Vector2D v_normal = V.corrVelocity.project_onto(normal_dir);
        if (v_normal.magnitude() > V.v_normal_max)
            v_normal = v_normal.normalize() * V.v_normal_max;

        Vector2D v_end = v_tangent + v_normal;

        num++;
        if (num > 5)
        {
            // debug_uart.printf_DMA("%f,%f,%f,%f\n", robot_pos_.x, robot_pos_.y, speed.magnitude(), v_tangent.magnitude());
            num = 0;
        }
        return v_end;
    }
    ///////////////////////////////////       KFS路径生成            ////////////////////////////////

    void KFS_Path_Check(void)
    {
        // KFS拾取判断MF1
        if (KFS_point.MF1_pos_.x == curve.Get_End_point().x && KFS_point.MF1_pos_.y == curve.Get_End_point().y)
        {
            KFS_flag.MF1_flag = true;
        }
        else if (KFS_flag.MF1_flag == true)
        {
            KFS_flag.MF1_flag = false;
            pid_dead_flag = false;
            Arm_Start = true;
            KFS_flag.MF1_finish = true;
        }

        // KFS拾取判断MF2
        if (KFS_point.MF2_pos_.x == curve.Get_End_point().x && KFS_point.MF2_pos_.y == curve.Get_End_point().y)
        {
            KFS_flag.MF2_flag = true;
        }
        else if (KFS_flag.MF2_flag == true)
        {
            KFS_flag.MF2_flag = false;
            pid_dead_flag = false;
            Arm_Start = true;
            KFS_flag.MF2_finish = true;
        }

        // KFS拾取判断MF3
        if (KFS_point.MF3_pos_.x == curve.Get_End_point().x && KFS_point.MF3_pos_.y == curve.Get_End_point().y)
        {
            KFS_flag.MF3_flag = true;
        }
        else if (KFS_flag.MF3_flag == true)
        {
            KFS_flag.MF3_flag = false;
            pid_dead_flag = false;
            Arm_Start = true;
            KFS_flag.MF3_finish = true;
        }

        if (KFS_flag.spin_flag_0 == true && Arm_Start == false)
        {
            // 两侧旋转判断
            if (KFS_point.spin_pos_0.x == curve.Get_End_point().x && KFS_point.spin_pos_0.y == curve.Get_End_point().y)
            {
                KFS_flag.get_spin_flag = true;
            }
            // 两侧开始旋转
            else if (KFS_flag.get_spin_flag == true)
            {
                target_yaw = KFS_point.MF1_target_yaw_;
                KFS_flag.spin_flag_0 = false;
                KFS_flag.get_spin_flag = false;
            }
        }

        if (KFS_flag.spin_flag == true && KFS_flag.MF1_finish == true && Arm_Start == false)
        {
            // 第一排和最后一排旋转
            if (target_yaw == -90.0f || target_yaw == 90.0f)
            {
                target_yaw = KFS_point.MF2_target_yaw_;
                KFS_flag.spin_flag = false;
            }
            // 两侧旋转判断
            else if (KFS_point.spin_pos.x == curve.Get_End_point().x && KFS_point.spin_pos.y == curve.Get_End_point().y)
            {
                KFS_flag.get_spin_flag = true;
            }
            // 两侧开始旋转
            else if (KFS_flag.get_spin_flag == true)
            {
                target_yaw = KFS_point.MF2_target_yaw_;
                KFS_flag.spin_flag = false;
                KFS_flag.get_spin_flag = false;
            }
        }

        if (KFS_flag.spin_flag_2 == true && KFS_flag.spin_flag == false && KFS_flag.MF2_finish == true && Arm_Start == false)
        {
            // 第一排旋转
            if (target_yaw == -90.0f || target_yaw == 90.0f)
            {
                target_yaw = KFS_point.MF3_target_yaw_;
                KFS_flag.spin_flag_2 = false;
            }
            // 两侧旋转判断
            else if (KFS_point.spin_pos_2.x == curve.Get_End_point().x && KFS_point.spin_pos_2.y == curve.Get_End_point().y)
            {
                KFS_flag.get_spin_flag = true;
            }
            // 两侧开始旋转
            else if (KFS_flag.get_spin_flag == true)
            {
                target_yaw = KFS_point.MF3_target_yaw_;
                KFS_flag.spin_flag_2 = false;
                KFS_flag.get_spin_flag = false;
            }
        }

        if (KFS_flag.uphill_flag == true)
        {
            // 上坡后旋转判断
            if (CZ_point.uphill_pos[RB_Flag].x == curve.Get_Start_point().x && CZ_point.uphill_pos[RB_Flag].y == curve.Get_Start_point().y)
            {
                if (robot_pos_.x > CZ_point.skew_yaw)
                    target_yaw = RB_Flag ? 180.0f : 0.0f;
            }
        }
    }
    ////////////////////////                  梅林路径规划                           ///////////////////////////
    bool KFS_Selection_Planning(void)
    {
        // 只能在一区和二区进行启动
        if (robot_pos_.x < 0.0f || robot_pos_.x > 6.0f || robot_pos_.y > 10.0f || robot_pos_.y < 0.0f)
        {
            return false;
        }

        // 对kfs夹取数量进行判断，并进行合法判断
        int KFS_num = 0;
        if (KFS_point.KFS1 > 0 && KFS_point.KFS2 == 0 && KFS_point.KFS3 == 0)
        {
            KFS_num = 1;
        }
        else if (KFS_point.KFS1 > 0 && KFS_point.KFS2 > 0 && KFS_point.KFS3 == 0)
        {
            KFS_num = 2;
        }
        else if (KFS_point.KFS1 > 0 && KFS_point.KFS2 > 0 && KFS_point.KFS3 > 0)
        {
            KFS_num = 3;
        }
        else
        {
            return false;
        }

        // 自动规划接口转换
        Point2D robot_point_ = {robot_pos_.x, robot_pos_.y};

        // 计算理想的KFS路径
        KFS_KeyPoint_ = MF_AutoCtrler::PathInformation_calc(robot_point_, KFS_point.KFS1, KFS_point.KFS2, KFS_point.KFS3);

        int8_t MF1_Index_ = KFS_KeyPoint_.Index_MFroad[0]; // MF1 对应索引
        int8_t MF2_Index_ = KFS_KeyPoint_.Index_MFroad[1]; // MF2 对应索引
        int8_t MF3_Index_ = KFS_KeyPoint_.Index_MFroad[2]; // MF3 对应索引

        // 寻找MF拾取车辆点位
        int8_t MF1_Point_ = KFS_KeyPoint_.mustPastMap[MF1_Index_]; // MF1 对应地图点编号。
        int8_t MF2_Point_ = KFS_KeyPoint_.mustPastMap[MF2_Index_]; // MF2 对应地图点编号。
        int8_t MF3_Point_ = KFS_KeyPoint_.mustPastMap[MF3_Index_]; // MF3 对应地图点编号。

        // 判断MF1的车子朝向
        KFS_point.MF1_target_yaw_ = rotation_path(MF1_Point_);
        // 判断MF2的车子朝向
        if (KFS_num > 1)
        {
            KFS_point.MF2_target_yaw_ = rotation_path(MF2_Point_);
        }
        // 判断MF3的车子朝向
        if (KFS_num > 2)
        {
            KFS_point.MF3_target_yaw_ = rotation_path(MF3_Point_);
        }

        // 写入MF地图对应坐标
        Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(MF1_Point_);
        if (MF_AutoCtrler::GetMFHeight(KFS_point.MF1) == 0.2f && KFS_point.MF3 != 0)
        {
            if (KFS_point.MF1_target_yaw_ == 0.0f)
            {
                KFS_point.MF1_pos_ = {temp_vector.x - KFS_point.point_skew, temp_vector.y};
            }
            else if (KFS_point.MF1_target_yaw_ == 180.0f)
            {
                KFS_point.MF1_pos_ = {temp_vector.x + KFS_point.point_skew, temp_vector.y};
            }
            else if (KFS_point.MF1_target_yaw_ == 90.0f)
            {
                KFS_point.MF1_pos_ = {temp_vector.x, temp_vector.y - KFS_point.point_skew};
            }
            else if (KFS_point.MF1_target_yaw_ == -90.0f)
            {
                KFS_point.MF1_pos_ = {temp_vector.x, temp_vector.y + KFS_point.point_skew};
            }
        }
        else
        {
            KFS_point.MF1_pos_ = {temp_vector.x, temp_vector.y};
        }

        if (KFS_num > 1)
        {
            KFS_point.MF2_pos_ = MF_AutoCtrler::MapCenterWorld_Vector2D(MF2_Point_);
        }
        else
        {
            KFS_point.MF2_pos_ = {0.0f, 0.0f};
        }
        if (KFS_num > 2)
        {
            KFS_point.MF3_pos_ = MF_AutoCtrler::MapCenterWorld_Vector2D(MF3_Point_);
        }
        else
        {
            KFS_point.MF3_pos_ = {0.0f, 0.0f};
        }

        // 判断第一次是否需要转向
        if (KFS_point.MF1_target_yaw_ == KFS_point.MF2_target_yaw_ || KFS_num < 2.0f)
        {
            KFS_flag.spin_flag = false;
        }
        else if (KFS_point.MF1_target_yaw_ != KFS_point.MF2_target_yaw_)
        {
            KFS_flag.spin_flag = true;
        }
        else
        {
            KFS_flag.spin_flag = false;
        }

        // 判断第二次是否需要转向
        if (KFS_point.MF2_target_yaw_ == KFS_point.MF3_target_yaw_ || KFS_num < 3.0f)
        {
            KFS_flag.spin_flag_2 = false;
        }
        else if (KFS_point.MF2_target_yaw_ != KFS_point.MF3_target_yaw_)
        {
            KFS_flag.spin_flag_2 = true;
        }
        else
        {
            KFS_flag.spin_flag_2 = false;
        }

        // 计算出口索引
        int index_exit = 0; // 当前路径出口索引（有效路径点长度）。
        while (index_exit < 15 && KFS_KeyPoint_.mustPastMap[index_exit] != 0)
        {
            index_exit++;
        }

        // 写入路径点的临时变量
        Vector2D last_vector = robot_pos_;
        Vector2D spin_vector = {0.0f, 0.0f};
        int temp_point = 0;
        int i = 0;

        // 重置路径规划器
        path_line_.Reset();
        path_line_.plan_reset();

        path_line_.Add_Start_Point(robot_pos_);

        // 在梅林内的情况处理，如果需要在外面旋转会先生成路径，如果需要拿同左右同列的会退出
        if (MF_AutoCtrler::GetMapNumFromPos(robot_point_))
        {
            i = 1;
            temp_point = KFS_KeyPoint_.mustPastMap[0];
            // 拐角无法处理防止撞车
            if (temp_point == 1 || temp_point == 5 || temp_point == 26 || temp_point == 30)
            {
                if (_tool_Abs(yaw - KFS_point.MF1_target_yaw_) > 20.0f)
                {
                    if (robot_pos_.y > 2.7f && robot_pos_.y < 8.4f)
                        return false;
                }
                else
                {
                    target_yaw = KFS_point.MF1_target_yaw_;
                }
            }
            else if (temp_point == 27 || temp_point == 28 || temp_point == 29 || temp_point == 30 || temp_point == 2 || temp_point == 3 || temp_point == 4 || temp_point == 5)
            {
                // 在上下两排就直接转
                target_yaw = KFS_point.MF1_target_yaw_;
            }
            else if (temp_point == 21 || temp_point == 16 || temp_point == 11 || temp_point == 6 || temp_point == 25 || temp_point == 20 || temp_point == 15 || temp_point == 10)
            {
                // 左右两排且接下来不为终点
                if (i != (index_exit - 1))
                {
                    // 下一个点为拐点
                    if (KFS_KeyPoint_.mustPastMap[1] == 1 || KFS_KeyPoint_.mustPastMap[1] == 5 || KFS_KeyPoint_.mustPastMap[1] == 26 || KFS_KeyPoint_.mustPastMap[1] == 30)
                    {

                        if (_tool_Abs(yaw - KFS_point.MF1_target_yaw_) < 10.0f)
                        {
                            // 角度差距小直接转当做无事发生
                            target_yaw = KFS_point.MF1_target_yaw_;
                        }
                        else
                        {
                            // 需要旋转的则生成路径并将索引改到2
                            KFS_flag.spin_flag_0 = true;
                            float spin_delay = 1.0f;
                            if (KFS_KeyPoint_.mustPastMap[1] == 1 || KFS_KeyPoint_.mustPastMap[1] == 5)
                            {
                                spin_delay *= (-1.0f);
                            }
                            spin_vector = spinodal_path(last_vector, MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[1]), i, (KFS_point.spin_skew * spin_delay));
                            if (spin_vector.x == 0.0f && spin_vector.x == 0.0f)
                                return false;
                            KFS_point.spin_pos_0 = spin_vector;
                            i = 2;
                            last_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[1]);
                        }
                    }
                    else if (KFS_KeyPoint_.mustPastMap[1] == MF1_Point_) // 下一个为KFS1
                    {
                        if (_tool_Abs(yaw - KFS_point.MF1_target_yaw_) < 10.0f)
                        {
                            // 角度差距小直接转当做无事发生
                            target_yaw = KFS_point.MF1_target_yaw_;
                        }
                        else
                        {
                            return false;
                        }
                    }
                    else
                    {
                        return false;
                    }
                }
            }
        }
        else
        {
            target_yaw = KFS_point.MF1_target_yaw_;
        }

        // 写入起点到MF2路径点坐标（不包含MF2）
        bool FINSH = false;
        for (; i < (KFS_num == 1 ? index_exit : MF2_Index_); i++)
        {
            if (KFS_num == 1)
            {
                if (i == (index_exit - 1)) // 终点
                {
                    temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);
                    // 梅林自动规划后是否直接上三区
                    if (KFS_flag.uphill_flag == false)
                    {
                        path_line_.Add_End_Point(temp_vector, path_param.end);
                    }
                    else if (KFS_flag.uphill_flag == true)
                    {
                        if ((last_vector.x == 0.6f && RB_Flag == true) || (last_vector.x == 5.4f && RB_Flag == false))
                        {
                            path_line_.Add_Point(temp_vector, path_param.start);
                            path_line_.Add_Point(CZ_point.uphill_pos[RB_Flag], path_param.up);
                            path_line_.Add_End_Point(CZ_point.R1_pos[1][RB_Flag], path_param.end);
                        }
                        else if (last_vector.y == 8.6f)
                        {
                            path_line_.Add_Point((temp_vector + ((last_vector - temp_vector).normalize() * KFS_point.coner_ahead)), path_param.start);
                            path_line_.Add_Point((temp_vector + (Vector2D{0.0f, 1.0f} * KFS_point.coner_ahead)), path_param.curve);
                            path_line_.Add_Point(CZ_point.uphill_pos[RB_Flag], path_param.up);
                            path_line_.Add_End_Point(CZ_point.R1_pos[1][RB_Flag], path_param.end);
                        }
                    }
                    // 取末端点进行路径退出后的锁点pid
                    Path_end_point = path_line_.Get_End_Point();
                    return true;
                }
            }
            temp_point = KFS_KeyPoint_.mustPastMap[i];
            temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(temp_point);
            // 四个拐点的顺滑处理
            if (temp_point == 1 || temp_point == 5 || temp_point == 26 || temp_point == 30)
            {
                float spin_delay = KFS_flag.spin_flag == true && (KFS_point.MF1_target_yaw_ == 180.0f || KFS_point.MF1_target_yaw_ == 0.0f);
                if (temp_point == 1 || temp_point == 5)
                {
                    spin_delay *= (-1.0f);
                }
                spin_vector = spinodal_path(last_vector, temp_vector, i, (KFS_point.spin_skew * spin_delay));
                if (spin_vector.x == 0.0f && spin_vector.x == 0.0f)
                    return false;
                if (spin_delay != 0)
                {
                    if (FINSH == true)
                    {
                        KFS_point.spin_pos = spin_vector;
                        FINSH = false;
                    }
                }
            }
            else if (temp_point == MF1_Point_) // MF停止点
            {
                path_line_.Add_Point(KFS_point.MF1_pos_, path_param.end);
                FINSH = true;
            }
            else // 衔接路径
            {
                path_line_.Add_Point(temp_vector, path_param.start);
            }
            last_vector = temp_vector;
        }

        // 写入MF2到终点路径点坐标
        FINSH = false;
        for (i = MF2_Index_; i < index_exit; i++)
        {
            if (i == (index_exit - 1))
            {
                temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i]);

                // 梅林自动规划后是否直接上三区
                if (KFS_flag.uphill_flag == false)
                {
                    path_line_.Add_End_Point(temp_vector, path_param.end);
                }
                else if (KFS_flag.uphill_flag == true)
                {
                    if ((last_vector.x == 0.6f && RB_Flag == true) || (last_vector.x == 5.4f && RB_Flag == false))
                    {
                        path_line_.Add_Point(temp_vector, path_param.start);
                        path_line_.Add_Point(CZ_point.uphill_pos[RB_Flag], path_param.up);
                        path_line_.Add_End_Point(CZ_point.R1_pos[1][RB_Flag], path_param.end);
                    }
                    else if (last_vector.y == 8.6f)
                    {
                        path_line_.Add_Point((temp_vector + ((last_vector - temp_vector).normalize() * KFS_point.coner_ahead)), path_param.start);
                        path_line_.Add_Point((temp_vector + (Vector2D{0.0f, 1.0f} * KFS_point.coner_ahead)), path_param.curve);
                        path_line_.Add_Point(CZ_point.uphill_pos[RB_Flag], path_param.up);
                        path_line_.Add_End_Point(CZ_point.R1_pos[1][RB_Flag], path_param.end);
                    }
                }
            }
            else
            {
                temp_point = KFS_KeyPoint_.mustPastMap[i];
                temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(temp_point);
                // 四个拐点的顺滑处理
                if (temp_point == 1 || temp_point == 5 || temp_point == 26 || temp_point == 30)
                {
                    float spin_delay = KFS_flag.spin_flag_2 == true && (KFS_point.MF2_target_yaw_ == 180.0f || KFS_point.MF2_target_yaw_ == 0.0f);
                    if (temp_point == 1 || temp_point == 5)
                    {
                        spin_delay *= (-1.0f);
                    }
                    spin_vector = spinodal_path(last_vector, temp_vector, i, (KFS_point.spin_skew * spin_delay));
                    if (spin_vector.x == 0.0f && spin_vector.x == 0.0f)
                        return false;
                    if (spin_delay != 0)
                    {
                        if (FINSH == true)
                        {
                            KFS_point.spin_pos_2 = spin_vector;
                            FINSH = false;
                        }
                    }
                }
                else if (((temp_point == MF3_Point_) && KFS_num > 2) || ((temp_point == MF2_Point_) && KFS_num > 1)) // MF停止点
                {
                    if (temp_point == MF2_Point_)
                        FINSH = true;
                    path_line_.Add_Point(temp_vector, path_param.end);
                }
                else // 衔接路径
                {
                    path_line_.Add_Point(temp_vector, path_param.start);
                }
                last_vector = temp_vector;
            }
        }
        // 取末端点进行路径退出后的锁点pid
        Path_end_point = path_line_.Get_End_Point();
        return true;
    }

    Vector2D spinodal_path(Vector2D last_vector, Vector2D temp_vector, int i, float spin_flag)
    {
        Vector2D forward_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(KFS_KeyPoint_.mustPastMap[i + 1]);
        // 拐点前偏移点
        path_line_.Add_Point((temp_vector + ((last_vector - temp_vector).normalize() * KFS_point.coner_ahead)), path_param.start);
        /*
        1->(1,0)
        2->(-1,0)
        3->(0,1)
        4->(0,-1)
        */
        // 拐点方向判断
        Vector2D tangent_vector = (forward_vector - temp_vector).normalize();
        if (tangent_vector.x == 1.0f)
            path_param.curve.targetPos = 1.0f;
        else if (tangent_vector.x == -1.0f)
            path_param.curve.targetPos = 2.0f;
        else if (tangent_vector.y == 1.0f)
            path_param.curve.targetPos = 3.0f;
        else if (tangent_vector.y == -1.0f)
            path_param.curve.targetPos = 4.0f;
        else
            return Vector2D{0.0f, 0.0f};

        // 拐点偏移
        temp_vector.y = temp_vector.y + spin_flag;

        // 拐点后偏移点
        Vector2D result = (temp_vector + (tangent_vector * KFS_point.coner_behind));
        path_line_.Add_Point(result, path_param.curve);
        path_param.curve.targetPos = 999.0f;
        return result;
    }

    float rotation_path(float MF_Point)
    {
        if (MF_Point == 21 || MF_Point == 16 || MF_Point == 11 || MF_Point == 6)
        {
            return 180.0f;
        }
        else if (MF_Point == 25 || MF_Point == 20 || MF_Point == 15 || MF_Point == 10)
        {
            return 0.0f;
        }
        else if (MF_Point == 27 || MF_Point == 28 || MF_Point == 29 || MF_Point == 30)
        {
            return 90.0f;
        }
        else if (MF_Point == 2 || MF_Point == 3 || MF_Point == 4 || MF_Point == 5)
        {
            return -90.0f;
        }
    }
	void KFS_Path_Init(void)
    {
		static bool pause_click = false;
        if (flag == 1)
        {
            KFS_point.KFS1 = KFS_point.MF1; // 目标点 1 编号。
            KFS_point.KFS2 = KFS_point.MF2; // 目标点 2 编号。
            KFS_point.KFS3 = KFS_point.MF3; // 目标点 3 编号。
            flag = 0;
            flag_reset();
            KFS_Selection_Planning();
        }
        
        // 暂停自动
        if (airjoy_data_.RT == 1 && pause_click == false)
        {
            pause_click = true;
            KFS_flag.pause_flag = !KFS_flag.pause_flag;
            if (KFS_flag.pause_flag == true)
            {
                if (KFS_flag.MF1_finish == true)
                {
                    KFS_point.KFS1 = KFS_point.KFS2;
                    KFS_point.KFS2 = KFS_point.KFS3;
                    KFS_point.KFS3 = 0;
                }
                if (KFS_flag.MF2_finish == true)
                {
                    KFS_point.KFS1 = KFS_point.KFS2;
                    KFS_point.KFS2 = 0;
                }
                if (KFS_flag.MF3_finish == true)
                {
                    KFS_point.KFS1 = 0;
                }
            }
            else if (KFS_flag.pause_flag == false)
            {
                Path_end_point = robot_pos_;
                flag_reset();
                KFS_Selection_Planning();
            }
        }
        if (airjoy_data_.RT == 0)
        {
            pause_click = false;
        }
    }

    void v_plan(void)
    {
        V.planspeed = path_line_.plan(robot_pos_);
        Path_correction();
        V.corrVelocity = V.PID_coefficient * V.corrVelocity;
        
        speed = v_limit();

        Chassis_Target.VX = speed.x;
        Chassis_Target.VY = speed.y;
    }
    // 当需要所目标角时第四个参数给false
    void CHASSIS_MANUAL(float vx_ratio, float vy_ratio, float yaw_ratio = 0.0f, bool yaw_update = true)
    {
        if (_tool_Abs(airjoy_data_.left_x) > 0.1f)
            Chassis_Target.VX = airjoy_data_.left_x * vx_ratio * this->is_chassis_reverse_;
        else
            Chassis_Target.VX = 0.0f;
        if (_tool_Abs(airjoy_data_.left_y) > 0.1f)
            Chassis_Target.VY = airjoy_data_.left_y * vy_ratio * this->is_chassis_reverse_;
        else
            Chassis_Target.VY = 0.0f;
        if (_tool_Abs(airjoy_data_.right_x) > 0.1f)
            Chassis_Target.yaw_rate = airjoy_data_.right_x * yaw_ratio * (-1.0f);
        else
            Chassis_Target.yaw_rate = 0.0f;

        if (yaw_update)
            target_yaw = yaw;
    }
    // 复位自动流程相关标志位。
    void flag_reset(void)
    {
        // 统一清空自动流程的阶段标志与旋转状态。
        WeaponSage_Start = false;
        WeaponSage_End = false;
        Arm_Start = false;
        CZ_Arm = false;
        pid_dead_flag = false;

        KFS_flag.MF1_flag = false;
        KFS_flag.MF2_flag = false;
        KFS_flag.MF3_flag = false;

        KFS_flag.spin_flag_0 = false;
        KFS_flag.spin_flag = false;
        KFS_flag.spin_flag_2 = false;

        KFS_flag.MF1_finish = false;
        KFS_flag.MF2_finish = false;
        KFS_flag.MF3_finish = false;

        KFS_flag.get_spin_flag = false;
        KFS_flag.pause_flag = false;

        CB_flag.Retreat_flag = false;
        CB_flag.Selection_flag = false;
        
        curve.Rest();
    }
    void mode_init(void)
    {
        if (chassis_status_last_ != chassis_status_)
        {
            flag_reset();
            CZ_index_reset();
            path_line_.Reset();
            path_line_.plan_reset();
            Path_end_point = robot_pos_;
        }
        chassis_status_last_ = chassis_status_;
    }
};

#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H