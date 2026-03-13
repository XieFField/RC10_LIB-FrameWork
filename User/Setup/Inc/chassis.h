#ifndef CHASSIS_H_
#define CHASSIS_H_

#include "RC10_LIB/APP/Inc/APP_Utils.h"

#include "Motor_Base.h"

namespace jia
{
    class Chassis
    {
    public:
        u8 period_ms = 1;           // 控制周期，单位：毫秒
        f32 max_v_acc = 2.0f;       // 最大线加速度，单位：m/s^2
        f32 max_w_acc_deg = 360.0f; // 最大角加速度，单位：deg/s^2

        f32 max_set_vx = 1.0f;       // 最大设定目标x轴速度，单位：米/秒
        f32 max_set_vy = 1.0f;       // 最大设定目标y轴速度，单位：米/秒
        f32 max_set_wz_deg = 360.0f; // 最大设定目标z轴角速度，单位：deg/s

        f32 max_wheel_rpm = 100.0f; // 最大轮子转速，单位：rpm
        u8 vx_radio = 30;           // x轴速度比例系数，单位：%
        u8 vy_radio = 30;           // y轴速度比例系数，单位：%
        u8 wz_radio = 40;           // z轴角速度比例系数，单位：%

        // 自动计算量
        f32 max_vx = 0.0f; // 最大x轴速度，单位：米/秒
        f32 max_vy = 0.0f; // 最大y轴速度，单位：米/秒
        f32 max_wz = 0.0f; // 最大z轴角速度，单位：rad/s

    public:
        struct init_config
        {
            Motor_Base *motor_handle[3];
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
            f32 vx;
            f32 vy;
            f32 wz;
        };

        struct Data
        {
            f32 vx;     // x轴速度（单位：米/秒）
            f32 vy;     // y轴速度（单位：米/秒）
            f32 wz;     // z轴角速度（单位：rad/s）
            f32 w1_rpm; // 轮子1的转速（单位：rpm）
            f32 w2_rpm; // 轮子2的转速（单位：rpm）
            f32 w3_rpm; // 轮子3的转速（单位：rpm）
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
            f32 x;         // （单位：米）
            f32 y;         // （单位：米）
            f32 theta_deg; // （单位：度）
            f32 radius;    // （单位：米）
            Motor_Base *motor_handle = nullptr;
            f32 sin_theta;
            f32 cos_theta;
            f32 equivalent_radius; // （单位：米）
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
    };
}

using jia::Chassis;

#endif // CHASSIS_H_
