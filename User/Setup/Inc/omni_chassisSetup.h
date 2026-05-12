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

#define debug_ladar 0

#define FF_V 0

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

    // 统一切换底盘状态，并在相机流程切入/切出时清理相关内部状态。
    void setChassisStatus(CHASSIS_Status_E status)
    {

        // 最后写入底盘总状态。
        chassis_status_ = status;
    }

    // 初始化底盘控制器参数并启动 RTOS 任务。
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
        path_lock.set_params(path_lock_end, 0.0f);

        // 相机模式独立 PID，参数使用 APP_PID 中独立配置对象。
        camera_pid_x_.set_params(camera_x_pid_params, 0.0f);
        camera_pid_y_.set_params(camera_y_pid_params, 0.0f);
        camera_pid_vec_.set_params(camera_vec_pid_params, 0.0f);
        camera_pid_yaw_.set_params(camera_yaw_pid_params, 10000.0f);
        camera_pid_yaw_.set_as_circular();

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
    int num = 0;
    //-----------------------------------通讯标志位-----------------------------------------//
    CHASSIS_Status_E chassis_status_ = CHASSIS_STOP; // 当前底盘总状态机状态。

    bool WeaponSage_END = false; // 夹杆流程完成标志。

    bool Arm_Start = false; // 机械臂动作触发标志。

    //-----------------------------------接口监视参数-----------------------------------------//
    int flag = 0;     // 自动流程起始触发位（边沿触发）。
    int flag_run = 0; // 自动流程运行中标志位。

    int8_t MF1 = 0; // 目标点 1 编号。
    int8_t MF2 = 0; // 目标点 2 编号。

    Vector2D planspeed = {0.0f, 0.0f};    // 路径规划输出的参考速度。
    Vector2D corrVelocity = {0.0f, 0.0f}; // 计算出的横向纠偏速度向量
    Vector2D speed = {0.0f, 0.0f};        // 合成后的底盘平移速度。

    Vector2D robot_pos_ = {0.0f, 0.0f}; // 当前机器人世界坐标。
    Vector2D test_pos_ = Vector2D{2.2, 0};
    float yaw = 0.0f;    // 当前机器人航向角（度）。
    Point3D ladar_data_; // 定位系统输出的原始位姿数据。

    PID_Position pid_pos_x; // x轴绝对位置PID控制器
    PID_Position pid_pos_y; // y轴绝对位置PID控制器
    PID_Position path_lock; // 停止锁点

    BezierCurve curve; // 当前路径曲线缓存。

    //---------------------------接口调试参数（需要修改时复制过来）---------------------------------------------//

    float max_robot_speed_ = 1.0f; // 常规段底盘最大速度限制。
    float min_robot_speed_ = 0.4f; // 常规段底盘最大速度限制。

    float gradient_start_ = 1.2f; // 终点梯度衰减起始距离。
    float gradient_end_ = 0.2f;   // 终点梯度衰减结束距离。
    float min_gradient_ = 0.8f;   // 终点最小速度缩放比例。

    float robot_speed_end_ = 0.3f;  // 终点段最大速度限制。
    float deadzone_max_end_ = 0.1f; // 判定“近终点”阈值。

    float m_lookaheadDist = 0.3f;        // 前视距离 (单位: 米)
    float m_lookaheadDist_line = 0.3f;   // 前视距离 (单位: 米)
    float m_lookaheadDist_curve = 0.05f; // 前视距离 (单位: 米)
    //-----------------------------------速度规划参数----------------------------------------------------//

    Path_line path_line_; // 路径规划器对象。

    Vector2D Clamping_Bar_Selection_pos_ = {2.405f, 0.69f}; // 夹杆流程默认目标点。

    Speedplanner_1D_Param_Config path_param_KFS_ = {.maxAcc = 30.0f, .maxDec = 40.0f, .maxJerk = 100.0f, .maxSpeed = 0.6f, .initialSpeed = 0.3f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f}; // KFS 速度规划参数。
    Speedplanner_1D_Param_Config path_param_CB_ = {.maxAcc = 5.0f, .maxDec = 5.0f, .maxJerk = 0.0f, .maxSpeed = 0.75f, .initialSpeed = 0.3f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f};     // 夹杆流程速度规划参数。

    Speedplanner_1D_Param_Config path_param_start_ = {.maxAcc = 0.5f, .maxDec = 0.5f, .maxJerk = 0.0f, .maxSpeed = 1.0f, .initialSpeed = 0.01f, .finalSpeed = 0.5f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f}; // KFS 速度规划参数。
    Speedplanner_1D_Param_Config path_param_line_ = {.maxAcc = 0.5f, .maxDec = 0.5f, .maxJerk = 0.0f, .maxSpeed = 1.0f, .initialSpeed = 0.5f, .finalSpeed = 0.5f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f};   // KFS 速度规划参数。
    Speedplanner_1D_Param_Config path_param_curve_ = {.maxAcc = 0.0f, .maxDec = 0.0f, .maxJerk = 0.0f, .maxSpeed = 0.5f, .initialSpeed = 0.5f, .finalSpeed = 0.5f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f};  // KFS 速度规划参数。
    Speedplanner_1D_Param_Config path_param_end_ = {.maxAcc = 0.5f, .maxDec = 0.5f, .maxJerk = 0.0f, .maxSpeed = 1.0f, .initialSpeed = 0.5f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f};    // KFS 速度规划参数。

    //-----------------------------------前视pid参数-----------------------------------------//

    float tNearest = 0.0f;   // 最近点在贝塞尔曲线上的参数t (0~1)
    float tLookahead = 0.0f; // 前视点在贝塞尔曲线上的参数t (0~1)

    // Vector2D lookaheadTangent; // 前视点处的切线方向向量
    // Vector2D pathEnd;          // 路径终点坐标

    //-----------------------------------梅林规划参数-----------------------------------------//

    MF_AutoCtrler::PathInformation_S KFS_KeyPoint_; // 自动规划输出的关键路径信息。

    Vector2D MF1_pos_ = {0.0f, 0.0f};
    Vector2D MF2_pos_ = {0.0f, 0.0f};

    float MF2_target_yaw_ = 0.0f; // 第二目标点对应目标朝向。
    bool spin_flag = false;       // 是否需要执行中途转向。

    bool spin_up_flag = false;   // 上路段旋转流程使能。
    bool spin_down_flag = false; // 下路段旋转流程使能。

    bool MF1_flag = false;   // 进入 MF1 目标点标志。
    bool MF2_flag = false;   // 进入 MF2 目标点标志。
    bool MF1_finish = false; // MF1 阶段已完成标志。

    Vector2D spin_point_ = {3.6f, 8.72f}; // 上方旋转点
    float spin_skew_ = -0.1f;             // 下方旋转位置y轴偏移量
    bool get_spin_flag = false;           // 旋转触发过渡标志。
    bool Spin_Start = false;              // 当前正在执行旋转。

    //-----------------------------------yaw角控制参数-----------------------------------------//

    float target_yaw_ = 0.0f; // 底盘锁角目标（度）。

    uint8_t yaw_pid_period_ = 3;       // yaw 环下采样周期（预留）。
    uint8_t yaw_pid_period_count_ = 0; // yaw 环下采样计数（预留）。
    PID_Position yaw_pid_;             // yaw 角度环控制器。

    float is_chassis_reverse_ = 1.0f; // 手动控制正反向系数。

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
#endif
    //-----------------------------------其他参数-----------------------------------------//
    void loop() override; // RTOS 主循环。

    bool init_flag = false; // 初始化完成标志。

    RmPocketData_t airjoy_data_;                            // 遥控器数据，范围 -1 ~ 1
    Camera_Data_t cam_data_dbg_ = {0.0f, 0.0f, 0.0f, 0.0f}; // 调试用相机数据缓存

    Debug_Printf debug_uart = Debug_Printf(&huart8); // 调试串口

    PID_Position camera_pid_x_; // 相机模式专用 x 轴位置环。

    PID_Position camera_pid_y_; // 相机模式专用 y 轴位置环（预留）。

    PID_Position camera_pid_vec_; // 相机模式专用向量模长位置环。

    PID_Position camera_pid_yaw_; // 相机模式专用 yaw 位置环。

    Robot_Twist last_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};   // 上一周期底盘目标姿态（预留）。
    Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // 当前周期底盘目标姿态。
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

    void KFS_Selection_Planning(void); // 生成 KFS 自动路径。

    void Path_correction(void); // 基于当前位置执行路径纠偏。

    void Path_spin_check(void); // 检查并执行路径中旋转逻辑。
#if FF_V
    // 统一清空自动控制相关内部状态（速度命令记忆与前馈差分状态）。
    void ResetAutoControlStates(void);

    Vector2D ComposeRobotVelocity(const Vector2D &v_pid); // 合成 PID、前馈、阻尼后的速度命令。

#endif

    Vector2D v_limit(Vector2D &v);

    void flag_reset(void);                      // 复位自动流程相关标志位。
    void Clamping_Bar_Selection_Planning(void); // 生成夹杆流程路径。
    /*
>>>>>>> main
    // ———————————————————       相机接口函数        —————————————————————————————//




    void camera_ctrl(void); // 相机闭环主流程状态机。

    bool check_stable(float error, float limit, uint8_t &count); // 连续计数判稳函数。

    Vector2D calc_vector(float x_err, float y_err, float max_vel); // x/y 误差向量合成速度指令。

    float clamp_value(float value, float low, float high); // 标量限幅工具函数。

    float avg_z(float z_now); // z 轴 20 点滑动平均滤波。

    enum Camera_State_E
    {
        CAMERA_WEAPON,  // 流程一：武器预对接姿态阶段。
        CAMERA_Z_ROUGH, // 流程二：z 粗调阶段。
        CAMERA_X_ROUGH, // 流程三：x 粗调阶段。
        CAMERA_Z_FINE,  // 流程三补充：z 精锁阶段。
        CAMERA_YAW,     // 流程四：yaw 锁定阶段。
        CAMERA_DOCK,    // 流程五：锁角有头对接阶段。
        CAMERA_DONE,    // 流程结束阶段。
    };

    Camera_State_E camera_state_ = CAMERA_WEAPON; // 相机流程当前阶段。

    Module_Camera *camera_ = nullptr; // 相机模块实例指针。

    UART_HandleTypeDef *camera_uart_ = &huart6; // 相机串口句柄（默认 huart6）。

    bool weapon_cameraStart = false; // 主状态机触发相机流程的标志位。

    bool camera_init_ = false; // 相机串口初始化完成标志。

    bool weapon_req_ = false; // 底盘到武器：预对接动作请求位。

    bool z_req_ = false; // 底盘到武器：z 调整请求位。

    bool weapon_done_ = false; // 武器到地盘：预对接完成反馈位。

    bool z_done_ = false; // 武器到底盘：z 调整完成反馈位。

    bool dock_done_ = false; // 外部到底盘：对接完成反馈位。

    float z_ref_ = 0.0f; // 底盘透传给武器的 z 参考值。

    float camera_x_ref_ = 0.0f; // 相机流程 x 轴目标值（米）。

    float camera_y_ref_ = 0.90f; // 相机流程 y 轴目标值（米）。

    float yaw_lock_ = 0.0f; // 相机流程期间的航向锁定目标（度）。

    float speed_max_ = 0.5f; // 相机流程平移最大模长（m/s）。

    float omega_max_ = 0.25f; // 相机流程角速度最大值（rad/s）。

    float pos_scale_ = 1.0f; // 位置环输出缩放系数。

    float yaw_scale_ = 1.0f; // 航向环输出缩放系数。

    uint8_t z_rough_count_ = 0; // z 粗调判稳计数。

    uint8_t x_count_ = 0; // x 粗调判稳计数。

    uint8_t z_fine_count_ = 0; // z 精锁判稳计数。

    uint8_t yaw_count_ = 0; // yaw 判稳计数。

    float z_buf_[20] = {0.0f}; // z 轴滑动平均环形缓冲区。

    float z_sum_ = 0.0f; // z 轴滑动平均累计和。

    uint8_t z_idx_ = 0; // z 缓冲区当前写入下标。

    uint8_t z_num_ = 0; // z 缓冲区当前有效样本数。

    float fake_x = 0.0f; // 调试假数据：x 误差输入（米）。

    float fake_y = 0.9f; // 调试假数据：y 误差输入（米）。

    float fake_z = 0.08f; // 调试假数据：z 误差输入（米）。

    float fake_yaw = 0.0f; // 调试假数据：yaw 误差输入（度）。

    // 外部接口函数
    */
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
        return WeaponSage_END;
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
    /*
    void set_camera_uart(UART_HandleTypeDef *uart)
    {
        camera_uart_ = uart;  // 设置相机串口句柄。
        camera_init_ = false; // 强制下次进入相机模式时重新初始化串口。
    }

    void set_camera_limit(float speed_max, float omega_max)
    {
        speed_max_ = speed_max; // 设置相机模式平移速度上限（m/s）。
        omega_max_ = omega_max; // 设置相机模式角速度上限（rad/s）。
    }

    void set_camera_scale(float pos_scale, float yaw_scale)
    {
        pos_scale_ = pos_scale; // 设置相机模式位置环输出缩放系数。
        yaw_scale_ = yaw_scale; // 设置相机模式航向环输出缩放系数。
    }

    void set_camera_xy_ref(float x_ref, float y_ref)
    {
        camera_x_ref_ = x_ref; // 设置相机模式 x 轴目标值（米）。
        camera_y_ref_ = y_ref; // 设置相机模式 y 轴目标值（米）。
    }

    void set_camera_y_ref(float y_ref)
    {
        camera_y_ref_ = y_ref; // 设置相机模式 y 轴目标值（米）。
    }

    float get_camera_y_ref() const
    {
        return camera_y_ref_; // 读取相机模式 y 轴目标值（米）。
    }

    void set_weapon_done(bool done)
    {
        weapon_done_ = done; // 写入武器预对接完成反馈位。
    }

    void set_z_done(bool done)
    {
        z_done_ = done; // 写入武器 z 调整完成反馈位。
    }

    void set_dock_done(bool done)
    {
        dock_done_ = done; // 写入外部对接完成标志位。
    }

    bool get_weapon_req() const
    {
        return weapon_req_; // 读取武器预对接请求位。
    }

    bool get_z_req() const
    {
        return z_req_; // 读取武器 z 调整请求位。
    }

    float get_z_ref() const
    {
        return z_ref_; // 读取透传给武器层的 z 参考值。
    }

    // 由主状态机调用，设置开启武器对接流程
    void setWeaponStart(bool isstart)
    {
        weapon_cameraStart = isstart;
    }
    */
};
#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H