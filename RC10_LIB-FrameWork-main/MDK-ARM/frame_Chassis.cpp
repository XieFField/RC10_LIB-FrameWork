#include "frame_Chassis.h"
#include <math.h>

// 数学常量定义
#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* 模板基类构造函数实现 */
template <uint8_t WHEEL_COUNT>
Chassis_Base<WHEEL_COUNT>::Chassis_Base(float L, float W) : L(L), W(W), chassis_vx(0.0f), chassis_vy(0.0f), chassis_wz(0.0f) {
    // 初始化轮子参数
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
        wheels[i].wheel_radius = 0.0f;
        wheels[i].wheel_speed = 0.0f;
        wheels[i].gear_ratio = 1.0f;  // 默认减速比为1:1
    }
    
    // 初始化DSP矩阵
    arm_mat_init_f32(&kinematics_matrix, WHEEL_COUNT, 3, kinematics_data);
}

/* 四轮全向底盘构造函数实现 */
QuanChassis::QuanChassis(float L, float W, float wheel_radius, float rotation_radius, float gear_ratio) 
    : Chassis_Base<4>(L, W), rotation_radius(rotation_radius) {
    
    // 设置轮子参数
    for (uint8_t i = 0; i < 4; i++) {
        wheels[i].wheel_radius = wheel_radius;
        wheels[i].gear_ratio = gear_ratio;
    }
    
    // 初始化运动学矩阵
    initKinematicsMatrix();
}

/* 初始化四轮全向底盘运动学矩阵 */
void QuanChassis::initKinematicsMatrix() {
    // 四轮全向底盘的运动学矩阵 - 使用三角函数计算
    // 轮子角度: 45°, 135°, 225°, 315° (π/4, 3π/4, 5π/4, 7π/4)
    const float angles[4] = {PI/4, 3*PI/4, 5*PI/4, 7*PI/4};
    
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t row = i * 3;
        kinematics_data[row] = cosf(angles[i]);           // cos(θ)
        kinematics_data[row + 1] = sinf(angles[i]);       // sin(θ)
        kinematics_data[row + 2] = rotation_radius;       // 传入的旋转半径
    }
}

/* 逆运动学：从底盘速度到轮子速度 */
void QuanChassis::inverseKinematics(float vx, float vy, float wz) {
    // 输入验证
    if (isnan(vx) || isnan(vy) || isnan(wz)) {
        return;
    }
    
    // 设置输入向量
    input_vector[0] = vx;
    input_vector[1] = vy;
    input_vector[2] = wz;
    
    // 使用DSP矩阵乘法计算轮子速度
    arm_matrix_instance_f32 input_mat;
    arm_matrix_instance_f32 output_mat;
    
    arm_mat_init_f32(&input_mat, 3, 1, input_vector);
    arm_mat_init_f32(&output_mat, 4, 1, output_vector);
    
    // 执行矩阵乘法: output = kinematics_matrix * input
    arm_status status = arm_mat_mult_f32(&kinematics_matrix, &input_mat, &output_mat);
    
    if (status) {
        // 将结果转换为轮子速度 (rad/s)，考虑轮子半径和减速比
        for (uint8_t i = 0; i < 4; i++) {
            if (wheels[i].wheel_radius > 0.0f && wheels[i].gear_ratio > 0.0f) {
                wheels[i].wheel_speed = output_vector[i] / (wheels[i].wheel_radius * wheels[i].gear_ratio);
            } else {
                wheels[i].wheel_speed = 0.0f;
            }
        }
        
        // 应用速度限制
        limitWheelSpeeds();
        
    }
}

/* 更新运动学状态 */
void QuanChassis::updateKinematics() {
   
}

/* 轮子速度限制 */
void QuanChassis::limitWheelSpeeds() {
    const float max_wheel_speed = 100.0f;  // 最大轮子速度 (rad/s)
    
    for (uint8_t i = 0; i < 4; i++) {
        if (fabs(wheels[i].wheel_speed) > max_wheel_speed) {
            wheels[i].wheel_speed = (wheels[i].wheel_speed > 0) ? max_wheel_speed : -max_wheel_speed;
        }
    }
}

/* 获取轮子速度 */
float QuanChassis::getWheelSpeed(uint8_t index) const {
    if (index < 4) {
        return wheels[index].wheel_speed;
    }
    return 0.0f;
}

/* 设置轮子半径 */
void QuanChassis::setWheelRadius(float radius) {
    if (radius > 0.0f) {
        for (uint8_t i = 0; i < 4; i++) {
            wheels[i].wheel_radius = radius;
        }
    }
}

/* 设置减速比 */
void QuanChassis::setGearRatio(float ratio) {
    if (ratio > 0.0f) {
        for (uint8_t i = 0; i < 4; i++) {
            wheels[i].gear_ratio = ratio;
        }
    }
}


template class Chassis_Base<4>;
