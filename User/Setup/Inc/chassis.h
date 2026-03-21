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
            kBodySpeedMode,
        };

        struct TargetBodySpeedModeData
        {
            f32 vel_x;
            f32 vel_y;
            f32 omega_z;
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

        // 默认构造和析构函数
        Chassis() = default;
        ~Chassis() = default;

        // 初始化
        void init(init_config &config);
        // 设置目标速度模式
        Result setTargetBodySpeedMode(const TargetBodySpeedModeData &target);

    private:
        void inverseKinematics(f32 in_x, f32 in_y, f32 in_z, f32 &out_w1, f32 &out_w2, f32 &out_w3);

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

    private:
        // 设定量
        constexpr static f32 period = 0.001f;       // 控制周期，单位：秒
        constexpr static f32 wheel_radius = 0.075f; // 轮子半径（单位：米）

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
        f32 max_acc_xy_acc = 2.0f;        // 最大XY轴线加速度，单位：m/s^2
        f32 max_acc_xy_dec = 20.0f;       // 最大XY轴线减速度，单位：m/s^2
        f32 max_alpha_z_acc = 4.0f;       // 最大z轴角加速度，单位：rad/s^2
        f32 max_alpha_z_dec = 6.0f;       // 最大z轴角减速度，单位：rad/s^2

        bool is_wheel_alpha_limit = false; // 是否进行轮端角加速度限制
        f32 max_wheel_alpha = 2.0f * kPi;  // 最大轮子角加速度，单位：rad/s^2

        const f32 &wr = wheel_radius;

        TargetBodySpeedModeData input_target_data; // 输入目标数据

        Debug_Printf debug_uart = Debug_Printf(&huart8); // 调试串口
        RmPocketData_t input_airjoy_data;

    private:
        f32 wheel_input_speed_radio = 300.0f;
        f32 wheel_speed_input;

        f32 sine_amplitude = 100.0f;
        f32 sine_frequency = 0.1f;

    private:
        f32 input_hwt_rot_z;
        f32 input_hwt_omega_z;

        PID_Position omega_z_pid;
        bool is_omega_z_close_loop = true;
        
        // PID_Position rot_z_pid;
    };
}

using jia::Chassis;

#endif // CHASSIS_H_
