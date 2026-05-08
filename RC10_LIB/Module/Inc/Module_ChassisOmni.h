/**
 * @file Module_ChassisOmni.h
 * @author XieFField
 * @brief 全向轮底盘运动学模块
 * @version 1.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"
#include "cmsis_os.h"

#ifdef __cplusplus
}
#endif

#include "Module_ChassisBase.h"
#include "APP_tool.h"

#ifdef __cplusplus

/*
 * 坐标系：世界坐标系 XY 平面速度分量遵循右手定则，逆时针为正方向。
 *
 * 仅支持 4 轮 / 3 轮全向轮底盘，不应用于其他轮数的全向轮底盘。
 */

// 三全向轮 120° 均布布局的三角函数常量
#define COS_30 0.86602540378f
#define SIN_30 0.5f
// 三全向轮等腰三角形布局的三角函数常量
#define COS_31_87 0.8493846882f
#define SIN_31_87 0.5278984245f
// 四全向轮 90° 均布布局的三角函数常量
#define COS_45 0.70710678118f
#define SIN_45 0.70710678118f

/*
 * 三轮布局（正视图）：
 *         | 1
 *     2 /     \ 3   对应等腰三角形底盘布局
 *
 * 四轮布局（俯视图）：
 *      2 /     \ 3   对应矩形底盘布局
 *      1 \     / 4
 */

template <std::size_t WheelCount>
class Chassis_Omni : public Chassis_Base<WheelCount> {
public:
    // 单个轮子的初始化配置
    struct wheel_init_config
    {
        float theta; // 轮子安装角度（单位：度）
        float x;     // 轮子X坐标（单位：米）
        float y;     // 轮子Y坐标（单位：米）
    };

    // 底盘总体初始化配置
    struct init_config
    {
        float wheel_radius;                         // 轮子半径 (m)
        float max_wheel_rpm;                        // 轮端最大转速 RPM
        wheel_init_config wheels[WheelCount];       // 各轮子配置
    };

private:
    // 轮子运动学预计算量（初始化时计算，运行时直接使用）
    struct wheel_calculate_config
    {
        float cos_theta;
        float sin_theta;
        float radius; // 等效半径 (m)：r = x*sinθ - y*cosθ
    };

public:
    // 圆形底盘（对称布局）
    Chassis_Omni(float wheel_radius, float max_wheel_rpm, float chassis_radius);
    // 等腰三角形底盘（三轮），base=底边长度，side=腰长
    Chassis_Omni(float wheel_radius, float max_wheel_rpm, float base_length, float side_length, bool three_wheel);
    // 通用配置初始化
    Chassis_Omni(init_config& config);

    // 更新运动学（逆向 + 限幅 + 正向）
    void updateKinematics() override;

    // 设置三轮求解器模式（仅 WheelCount==3 时有效）
    void setThreeWheelSolver(bool use_three_solver)
    {
        use_three_solver_ = use_three_solver;
    }

private:
    // 逆运动学：目标速度 → 各轮 RPM
    void inverseKinematics(const Robot_Twist& twist) override;

    // 正运动学：各轮 RPM → 底盘当前速度
    void forwardKinematics() override;

    // 根据等腰三角形几何计算顶点半径和底边半径
    void computeIsoscelesRadii(float base_length, float side_length, float& top_radius, float& bottom_radius);

    float chassis_radius_;        // 底盘半径（顶点到中心的距离）(m)
    float chassis_radius_bottom_; // 底盘底部到中心的距离 (m)

    // 三轮求解器选择标志（仅 WheelCount==3 时有效）
    bool use_three_solver_ = true;

    wheel_init_config wheel_config_[WheelCount];               // 轮子布局配置
    wheel_calculate_config wheel_calculate_config_[WheelCount]; // 轮子运动学预计算量
};

#endif // __cplusplus
