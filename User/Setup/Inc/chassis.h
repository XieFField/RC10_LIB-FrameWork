#ifndef CHASSIS_H_
#define CHASSIS_H_

#include "RC10_LIB/APP/Inc/APP_Utils.h"

#include "Motor_DJI.h"
#include "Module_CrsfReceiver.h"
#include "APP_debugTool.h"
#include "APP_PID.h"

namespace jia
{
    class Chassis
    {
    public:
        struct init_config
        {
            M3508 *motor_handle[3];
        };

        enum class Result
        {
            kOk,
            kError,
        };

        enum class Mode
        {
            kWheelTorqueFreeMode,
            kBodySpeedMode,
            kBodySpeedLockNowRotZMode,
            kBodySpeedLockToRotZMode,
            kWorldSpeedMode,
            kWorldSpeedLockNowRotZMode,
            kWorldSpeedLockToRotZMode,
        };

        struct TargetSpeedModeData
        {
            f32 vel_x;
            f32 vel_y;
            f32 omega_z;
        };

        struct TargetSpeedLockNowRotZModeData
        {
            f32 vel_x;
            f32 vel_y;
        };

        struct TargetSpeedLockToRotZModeData
        {
            f32 vel_x;
            f32 vel_y;
            f32 rot_z;
        };

        // 默认构造和析构函数
        Chassis() = default;
        ~Chassis() = default;

        // 初始化
        void init(init_config &config);
        // 设置轮子扭矩自由模式
        Result setWheelTorqueFreeMode();
        // 设置目标速度模式
        //  // 自身坐标系
        //  //  // 速度
        Result setTargetBodySpeedMode(f32 vel_x, f32 vel_y, f32 omega_z);
        Result setTargetBodySpeedMode(const TargetSpeedModeData &target);
        //  //  // 固定当前rot_z
        Result setTargetBodySpeedLockNowRotZMode(f32 vel_x, f32 vel_y);
        Result setTargetBodySpeedLockNowRotZMode(const TargetSpeedLockNowRotZModeData &target);
        //  //  // 固定到rot_z
        Result setTargetBodySpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z);
        Result setTargetBodySpeedLockToRotZMode(const TargetSpeedLockToRotZModeData &target);
        //  // 世界坐标系
        //  //  // 速度
        Result setTargetWorldSpeedMode(f32 vel_x, f32 vel_y, f32 omega_z);
        Result setTargetWorldSpeedMode(const TargetSpeedModeData &target);
        //  //  // 固定当前rot_z
        Result setTargetWorldSpeedLockNowRotZMode(f32 vel_x, f32 vel_y);
        Result setTargetWorldSpeedLockNowRotZMode(const TargetSpeedLockNowRotZModeData &target);
        //  //  // 固定到rot_z
        Result setTargetWorldSpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z);
        Result setTargetWorldSpeedLockToRotZMode(const TargetSpeedLockToRotZModeData &target);
        // 读取目标速度
        f32 getTargetWorldVelX() const;
        f32 getTargetWorldVelY() const;
        f32 getTargetOmegaZ() const;
        // 读取当前速度
        f32 getCurrentWorldVelX() const;
        f32 getCurrentWorldVelY() const;
        f32 getCurrentOmegaZ() const;

    private:
        struct wheel_config
        {
            f32 pos_x;                     // 单位：米
            f32 pos_y;                     // 单位：米
            f32 rot_z_deg;                 // 单位：度
            M3508 *motor_handle = nullptr; // 电机句柄
            f32 sin_rot_z;
            f32 cos_rot_z;
            f32 eq_radius;     // 等效半径，equivalent radius，可以是负值，单位：米
            f32 abs_sin_rot_z; // 正弦值的绝对值
            f32 abs_cos_rot_z; // 余弦值的绝对值
            f32 abs_eq_radius; // 等效半径的绝对值，单位：米

            M3508 *&h = motor_handle;
            f32 &s = sin_rot_z;
            f32 &c = cos_rot_z;
            f32 &eqr = eq_radius;
            f32 &as = abs_sin_rot_z;
            f32 &ac = abs_cos_rot_z;
            f32 &aeqr = abs_eq_radius;
        };

        struct InputTargetData
        {
            f32 vel_x;
            f32 vel_y;
            f32 omega_z;
            f32 rot_z;
        };

        struct TargetData
        {
            f32 vel_x;    // x轴速度，单位：米/秒
            f32 vel_y;    // y轴速度，单位：米/秒
            f32 omega_z;  // z轴角速度，单位：rad/s
            f32 w1_omega; // 轮子1的角速度，单位：rad/s
            f32 w2_omega; // 轮子2的角速度，单位：rad/s
            f32 w3_omega; // 轮子3的角速度，单位：rad/s
        };

        struct TargetPidData
        {
            f32 omega_z; // z轴角速度，单位：rad/s
        };

        struct PlannedData
        {
            f32 vel_x;    // x轴速度，单位：米/秒
            f32 vel_y;    // y轴速度，单位：米/秒
            f32 omega_z;  // z轴角速度，单位：rad/s
            f32 acc_x;    // x轴加速度，单位：米/秒^2
            f32 acc_y;    // y轴加速度，单位：米/秒^2
            f32 alpha_z;  // z轴角加速度，单位：rad/s^2
            f32 w1_alpha; // 轮子1的角加速度，单位：rad/s^2
            f32 w2_alpha; // 轮子2的角加速度，单位：rad/s^2
            f32 w3_alpha; // 轮子3的角加速度，单位：rad/s^2
            f32 w1_omega; // 轮子1的角速度，单位：rad/s
            f32 w2_omega; // 轮子2的角速度，单位：rad/s
            f32 w3_omega; // 轮子3的角速度，单位：rad/s
        };

        struct CurrentData
        {
            f32 w1_omega; // 轮子1的角速度，单位：rad/s
            f32 w2_omega; // 轮子2的角速度，单位：rad/s
            f32 w3_omega; // 轮子3的角速度，单位：rad/s
        };

        // 创建线程
        static void createThread(void *arg);
        // 运行线程函数
        void runThread(void *arg);

        // 轮子配置
        wheel_config wheel_config_[3];
        wheel_config &w1_ = wheel_config_[0];
        wheel_config &w2_ = wheel_config_[1];
        wheel_config &w3_ = wheel_config_[2];
        // 当前运行模式
        Mode mode_ = Mode::kBodySpeedMode;
        // 目标数据
        TargetData target_data_;
        // 规划数据
        PlannedData planned_data_;      // 规划数据
        PlannedData last_planned_data_; // 上一次规划数据
        // 当前数据
        CurrentData current_data_;

        bool is_world_speed_mode; // 是否为世界速度模式
        bool is_lock_rot_z;       // 是否固定到rot_z

    private:
        void inverseKinematics(f32 in_x, f32 in_y, f32 in_z, f32 &out_w1, f32 &out_w2, f32 &out_w3);

    private:
        // 设定量
        constexpr static u8 period_ms = 1;                 // 控制周期，单位：毫秒
        constexpr static f32 period = period_ms / 1000.0f; // 控制周期，单位：秒
        constexpr static f32 wheel_radius = 0.075f;        // 轮子半径（单位：米）

        bool is_wheel_omega_limit = true;           // 是否进行轮端角速度限制
        f32 max_wheel_omega = rpmToRadsF32(350.0f); // 最大轮子角速度，单位：rad/s
        f32 max_wheel_vel = 0.0f;                   // 最大轮子线速度，单位：米/秒

        // 车端速度限制参数
        f32 max_vel_x_radio = 1.0f;   // x轴速度比例系数
        f32 max_vel_y_radio = 1.0f;   // y轴速度比例系数
        f32 max_omega_z_radio = 1.0f; // z轴角速度比例系数

        f32 max_vel_x = 0.0f;   // 最大x轴速度，单位：米/秒
        f32 max_vel_y = 0.0f;   // 最大y轴速度，单位：米/秒
        f32 max_omega_z = 0.0f; // 最大z轴角速度，单位：rad/s

        bool is_chassis_acc_limit = false; // 是否进行车端加速度限制
        f32 max_acc_xy_acc = 2.0f;         // 最大XY轴线加速度，单位：m/s^2
        f32 max_acc_xy_dec = 20.0f;        // 最大XY轴线减速度，单位：m/s^2
        f32 max_alpha_z_acc = 4.0f;        // 最大z轴角加速度，单位：rad/s^2
        f32 max_alpha_z_dec = 6.0f;        // 最大z轴角减速度，单位：rad/s^2

        bool is_wheel_alpha_limit = false; // 是否进行轮端角加速度限制
        f32 max_wheel_alpha = 2.0f * kPi;  // 最大轮子角加速度，单位：rad/s^2

        const f32 &wr = wheel_radius;

        InputTargetData input_target_data; // 输入目标数据

        Debug_Printf debug_uart = Debug_Printf(&huart8); // 调试串口
        u8 printf_period_ms = 4;                         // 串口调试打印周期，单位：毫秒
        u8 printf_period_count = 0;                      // 串口调试打印周期计数器

        RmPocketData_t airjoy_data;

    private:
        f32 wheel_input_speed_radio = 300.0f;

        bool is_sine = false;
        f32 sine_amplitude = 0.0f;
        f32 sine_frequency = 0.1f;
        f32 sine_offset = 0.0f;

        bool is_phase_step = false;

    private:
        f32 input_hwt_rot_z;
        f32 input_hwt_omega_z;

        TargetPidData target_pid_data_;

        PID_Incremental omega_z_pid;
        uint8_t omega_z_pid_period = 1;
        uint8_t omega_z_pid_count = 0;
        bool is_omega_z_close_loop = false;

        PID_Position rot_z_pid;
        uint8_t rot_z_pid_period = 1;
        uint8_t rot_z_pid_count = 0;

    private:
        bool is_debug = true;
        
        u8 debug_mode = 0;
        f32 debug_lock_rot_z = 0.0f;
    };
}

using jia::Chassis;

#endif // CHASSIS_H_
