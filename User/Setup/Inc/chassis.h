#ifndef CHASSIS_H_
#define CHASSIS_H_

#include "RC10_LIB/APP/Inc/APP_Utils.h"

#include "Motor_DJI.h"
#include "Module_CrsfReceiver.h"
#include "APP_debugTool.h"

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

        struct Data
        {
            f32 vel_x;    // x轴速度（单位：米/秒）
            f32 vel_y;    // y轴速度（单位：米/秒）
            f32 omega_z;  // z轴角速度（单位：rad/s）
            f32 acc_x;    // x轴加速度，单位：m/s^2
            f32 acc_y;    // y轴加速度，单位：m/s^2
            f32 alpha_z;  // z轴角加速度，单位：rad/s^2
            f32 w1_alpha; // 轮子1的角加速度（单位：rad/s^2）
            f32 w2_alpha; // 轮子2的角加速度（单位：rad/s^2）
            f32 w3_alpha; // 轮子3的角加速度（单位：rad/s^2）
            f32 w1_omega; // 轮子1的角速度（单位：rad/s）
            f32 w2_omega; // 轮子2的角速度（单位：rad/s）
            f32 w3_omega; // 轮子3的角速度（单位：rad/s）
        };

        // 默认构造和析构函数
        Chassis() = default;
        ~Chassis() = default;

        // 初始化
        void init(init_config &config);
        // 设置目标速度模式
        Result setTargetBodySpeedMode(const TargetBodySpeedModeData &target);

    private:
        struct wheel_config
        {
            f32 pos_x;   // （单位：米）
            f32 pos_y;   // （单位：米）
            f32 yaw_deg; // （单位：度）
            M3508 *motor_handle = nullptr;
            f32 sin_yaw;
            f32 cos_yaw;
            f32 eq_radius;     // 等效半径，equivalent radius，可以是负值（单位：米）
            f32 abs_sin_yaw;   // 正弦值的绝对值
            f32 abs_cos_yaw;   // 余弦值的绝对值
            f32 abs_eq_radius; // 等效半径的绝对值（单位：米）

            f32 &s = sin_yaw;
            f32 &c = cos_yaw;
            f32 &eqr = eq_radius;
            f32 &as = abs_sin_yaw;
            f32 &ac = abs_cos_yaw;
            f32 &aeqr = abs_eq_radius;
        };

        // 创建线程
        static void createThread(void *arg);
        // 运行线程函数
        void runThread(void *arg);

        // 轮子配置
        wheel_config wheel_config_[3];
        // 当前运行模式
        Mode mode_ = Mode::kBodySpeedMode;
        // 目标数据
        Data target_data_;
        // 规划数据
        Data planned_data_;
        // 当前数据
        Data current_data_;

    private:
        // 设定量
        constexpr static float period = 0.001f; // 控制周期，单位：秒
        f32 max_acc = 2.0f;                     // 最大线加速度，单位：m/s^2
        f32 max_alpha_deg = 360.0f;             // 最大角加速度，单位：deg/s^2

        f32 max_set_vel_x = 1.0f;         // 最大设定目标x轴速度，单位：米/秒
        f32 max_set_vel_y = 1.0f;         // 最大设定目标y轴速度，单位：米/秒
        f32 max_set_omega_z_deg = 360.0f; // 最大设定目标z轴角速度，单位：deg/s

        f32 wheel_radius = 0.075f;        // 轮子半径（单位：米）
        f32 max_wheel_omega_rpm = 100.0f; // 最大轮子转速，单位：rpm
        f32 vel_x_radio = 0.4f;           // x轴速度比例系数
        f32 vel_y_radio = 0.4f;           // y轴速度比例系数
        f32 omega_z_radio = 0.2f;         // z轴角速度比例系数

        f32 &wr = wheel_radius;

        // 自动计算量
        f32 max_vel_x = 0.0f;   // 最大x轴速度，单位：米/秒
        f32 max_vel_y = 0.0f;   // 最大y轴速度，单位：米/秒
        f32 max_omega_z = 0.0f; // 最大z轴角速度，单位：rad/s

        f32 max_wheel_vel = 0.0f; // 最大轮子线速度，单位：米/秒

        Data input_target_data; // 输入目标数据

        Debug_Printf debug_uart = Debug_Printf(&huart8); // 调试串口
        RmPocketData_t airjoy_data_input;

        private:
        f32 wheel_speed_radio = 300.0f;
        f32 wheel_speed_input;

        f32 sine_amplitude = 100.0f;
        f32 sine_frequency = 0.1f;
    };

}

using jia::Chassis;

#endif // CHASSIS_H_
