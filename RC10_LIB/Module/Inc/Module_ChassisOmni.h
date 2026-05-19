/**
 * @file Module_ChassisOmni.h
 * @author XieFField
 * @brief 全向底盘模块
 * @version 1.0
 */
#ifndef __MODULE_CHASSISOMNI_H
#define __MODULE_CHASSISOMNI_H

/*

   ________                    _         ____                  _ 
  / ____/ /_  ____ ___________(_)____   / __ \____ ___  ____  (_)
 / /   / __ \/ __ `/ ___/ ___/ / ___/  / / / / __ `__ \/ __ \/ / 
/ /___/ / / / /_/ (__  |__  ) (__  )  / /_/ / / / / / / / / / /  
\____/_/ /_/\__,_/____/____/_/____/   \____/_/ /_/ /_/_/ /_/_/   
                                                                 

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
    坐标系采用右手系，角速度正方向遵循右手定则，即逆时针为正方向

    只包含4/3轮全向底盘，应该不会用到其他轮数的全向轮底盘吧
*/

#define COS_30 0.86602540378f
#define SIN_30 0.5f
#define COS_45 0.70710678118f
#define SIN_45 0.70710678118f
#define SIN_31_87 0.5278984245f   
#define COS_31_87 0.8493846882f

/*
三轮： 
        |1

    2 /    \ 3   对应的底盘电机编号

四轮:     2 /     \  3 对应的底盘电机编号
                         
          1 \     / 4
*/

template <std::size_t WheelCount>
class Chassis_Omni : public Chassis_Base<WheelCount> {
public:
    struct wheel_init_config
    {
        float theta; // （单位：度）
        float x;     // （单位：米）
        float y;     // （单位：米）
    };

    struct init_config
    {
        float wheel_radius; // 轮子半径 (m)
        float max_wheel_rpm; // 轮子最大RPM
        wheel_init_config wheels[WheelCount]; // 轮子配置
    };

private:
    struct wheel_calculate_config
    {
        float cos_theta;
        float sin_theta;
        float radius; // 等效半径 (m)
    };
    
public:
    Chassis_Omni(float wheel_radius, float max_wheel_rpm, float chassis_radius);
    // 等腰三角形参数构造（仅三轮）：base=底边长度，side=腰长
    Chassis_Omni(float wheel_radius, float max_wheel_rpm, float base_length, float side_length, bool three_wheel);
    Chassis_Omni(init_config& config);

    void updateKinematics() override; // 更新运动学，调用逆解和正解

    void setThreeWheelSolver(bool use_three_solver)
    {
        use_three_solver_ = use_three_solver;
    }
private:
    void inverseKinematics(const Robot_Twist& twist) override; // 逆解，根据目标速度计算轮速
    float chassis_radius_; // 底盘半径 (m)
    float chassis_radius_bottom_; // 底盘底部到中心的距离 (m)
    void forwardKinematics() override;
    // 依据等腰三角形几何计算两个半径：顶点半径与底边半径
    void computeIsoscelesRadii(float base_length, float side_length, float& top_radius, float& bottom_radius);

    // 三轮解算器选择标志（只在 WheelCount==3 时有效）
    bool use_three_solver_ = true;
    wheel_init_config wheel_config_[WheelCount]; // 轮子配置
    wheel_calculate_config wheel_calculate_config_[WheelCount]; // 轮子计算配置
};



#endif // __cplusplus

#endif // __MODULE_OMNICHASSIS_H
