/**
 * @file   Module_Chassis.h
 * @author XieFField/hst
 * @brief  底盘的正逆解
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

#include "arm_math.h"
#include "cmsis_os.h"
#include "Module_ChassisBase.h"
#include "APP_tool.h"

#ifdef __cplusplus

#define COS_30 0.86602540378f
#define SIN_30 0.5f
#define COS_45 0.70710678118f
#define SIN_45 0.70710678118f               

template <std::size_t WheelCount>
class Chassis_Onim : public Chassis_Base<WheelCount> {
public: 
    Chassis_Onim(float wheel_radius, float max_wheel_rpm, float chassis_radius);
    void updateKinematics() override;

protected:
    Robot_Twist robot_twist_foward = {0};
private:
    float chassis_radius_;//?????
    void inverseKinematics(const Robot_Twist& twist);
    void forwardKinematics(Robot_Twist& twist);
    arm_matrix_instance_f32 kinematics_matrix; // WheelCount x 3
    arm_matrix_instance_f32 input_mat_; // 3x1
    arm_matrix_instance_f32 output_mat_; // WheelCount x 1

    // ??????
    float32_t kinematics_matrix_data_[WheelCount * 3];
    float32_t input_vector_[3]; // 3x1
    float32_t output_vector_[WheelCount]; // 4x1    
};

#endif // __cplusplus
#endif // __MODULE_CHASSISDEMO_H__
