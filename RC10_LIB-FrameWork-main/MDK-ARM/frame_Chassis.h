#ifndef __FRAME_CHASSIS_H
#define __FRAME_CHASSIS_H

#include "arm_math.h"
#include <stdint.h>

/* 底盘轮子结构体 */
typedef struct {
    float wheel_radius;     // 轮子半径 (m)
    float wheel_speed;      // 轮子速度 (rad/s)
    float gear_ratio;       // 减速比
} Wheel;

/* 模板基类定义 */
template <uint8_t WHEEL_COUNT>
class Chassis_Base {
public:
    Chassis_Base(float L, float W);
    virtual ~Chassis_Base() = default;
    
    // 纯虚函数，需要在派生类中实现
    virtual void inverseKinematics(float vx, float vy, float wz) = 0;
    virtual void updateKinematics() = 0;
    
    // 获取底盘速度
    virtual float getChassisVx() const { return chassis_vx; }
    virtual float getChassisVy() const { return chassis_vy; }
    virtual float getChassisYawRate() const { return chassis_wz; }
    
protected:
    float L;  // 底盘半长 (m)
    float W;  // 底盘半宽 (m)
    Wheel wheels[WHEEL_COUNT];
    
    // 底盘速度状态
    float chassis_vx;   // 底盘x方向速度 (m/s)
    float chassis_vy;   // 底盘y方向速度 (m/s)
    float chassis_wz;   // 底盘旋转角速度 (rad/s)
    
    // DSP矩阵运算支持
    arm_matrix_instance_f32 kinematics_matrix;  // 运动学矩阵
    float32_t kinematics_data[WHEEL_COUNT * 3]; // 运动学矩阵数据
    float32_t input_vector[3];                 // 输入向量 [vx, vy, wz]
    float32_t output_vector[WHEEL_COUNT];      // 输出向量 [wheel1, wheel2, ...]
    
    // 初始化运动学矩阵
    virtual void initKinematicsMatrix() = 0;
};

/* 四轮全向底盘类 */
class QuanChassis : public Chassis_Base<4> {
public:
    QuanChassis(float L, float W, float wheel_radius, float rotation_radius, float gear_ratio);
    virtual ~QuanChassis() = default;
    
    // 实现基类的纯虚函数
    virtual void inverseKinematics(float vx, float vy, float wz) override;
    virtual void updateKinematics() override;
    
    // 获取轮子速度
    float getWheelSpeed(uint8_t index) const;
    
    // 设置轮子半径
    void setWheelRadius(float radius);
    
    // 设置减速比
    void setGearRatio(float ratio);
    
private:
    float rotation_radius;  // 旋转半径
    
    // 初始化运动学矩阵（四轮全向底盘的运动学矩阵）
    virtual void initKinematicsMatrix() override;
    
    // 轮子速度限制
    void limitWheelSpeeds();
    
};

#endif
