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
#define SIN_31_87 0.52799374f
#define COS_31_87 0.84924826f

/*
三轮：   2 /    \ 3   对应的底盘电机编号
            ___
             1

四轮:     2 /     \  3 对应的底盘电机编号
                         
          1 \     / 4
*/

template <std::size_t WheelCount>
class Chassis_Omni : public Chassis_Base<WheelCount> {
public:
    Chassis_Omni(float wheel_radius, float max_wheel_rpm, float chassis_radius);

    void updateKinematics() override; // 更新运动学，调用逆解和正解

private:
    void inverseKinematics(const Robot_Twist& twist) override; // 逆解，根据目标速度计算轮速
    float chassis_radius_; // 底盘半径 (m)

    void forwardKinematics() override;

    // arm_matrix_instance_f32 kinematics_matrix; // WheelCount x 3
    // arm_matrix_instance_f32 input_mat_; // 3x1
    // arm_matrix_instance_f32 output_mat_; // WheelCount x 1

    // float32_t kinematics_matrix_data_[WheelCount * 3];
    // float32_t input_vector_[3]; // 3x1
    // float32_t output_vector_[WheelCount]; // 4x1    
};



#endif // __cplusplus

#endif // __MODULE_OMNICHASSIS_H
