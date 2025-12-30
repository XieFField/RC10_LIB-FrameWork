/**
 * @file   APP_tool.h
 * @brief  通用工具函数头文件
 * @author XieFField
 */

#ifndef __APP_TOOL_H
#define __APP_TOOL_H

#include <stdint.h>
#include <cmath>

#ifdef __cplusplus
extern "C" {
    #include "arm_math.h"
    #define PI 3.14159265358979323846f
}
    

#endif

template<typename Type> 
Type _tool_Abs(Type x) 
{
    return ((x > 0) ? x : -x);
}


/**
 * @brief  将矩阵设置为单位矩阵
 * @param[in,out] M   指向矩阵实例
 * @note   要求矩阵是方阵 (numRows == numCols)
 */
void arm_set_identity_f32(arm_matrix_instance_f32 *M);

/**
 * @brief Perform binary search on a sorted array
 */
int binarySearch(const uint32_t arr[], uint8_t count, uint32_t key);

// 模板函数：将一个值限制在最小和最大值之间
/**
 * @param value 要限制的值
 * @param min 最小值
 * @param max 最大值
 */
template <typename T>
static inline T constrain(T value, T min, T max) 
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

template <typename T>
static inline T m_to_cm(T value_m)
{
    return value_m * 100.0f;
}

template <typename T>
static inline T cm_to_m(T value_cm)
{
    return value_cm / 100.0f;
}

template <typename T>
static inline T mm_to_cm(T value_mm)
{
    return value_mm / 10.0f;
}

template <typename T>
static inline T cm_to_mm(T value_cm)
{
    return value_cm * 10.0f;
}

template <typename T>
static inline T m_to_mm(T value_m)
{
    return value_m * 1000.0f;
}

template <typename T>
static inline T mm_to_m(T value_mm)
{
    return value_mm / 1000.0f;
}



//斜坡函数
void ramp(float target, float& current, float max_change_rate, float dt);

//弧度转换为角度函数
float rad_to_deg(float rad);

//角度转换为弧度函数
float deg_to_rad(float deg);

float normalize_deg_0_360(float a);

float normalize_deg_pm180(float a);
// 将 val_deg 映射到“最接近 ref_deg(0..360)”的等价角，并返回 0..360
float wrap_to_nearest_0_360(float ref_deg_0_360, float val_deg_any);


float wrap_to_nearest_cont(float ref_deg_cont, float val_deg_any);
// 2D点结构体
typedef struct  {
    float x = 0, y = 0;
    float theta = 0; // 旋转角度，单位弧度
} Point2D;

// 3D点结构体
typedef struct {
    float x = 0, y = 0, z = 0;
	float theta;
    float roll = 0, pitch = 0, yaw = 0; // 欧拉角，单位弧度

}Point3D;

typedef struct {
    float vx;
    float vy;
    float vz;

    float yaw_rate;
    float pitch_rate;
    float roll_rate;

    
}Robot_Twist;

typedef struct {
    float yaw_rate;
    float pitch_rate;
    float roll_rate;

    float yaw_angle;
    float pitch_angle;
    float roll_angle;
    
}Angle_Twist;


#ifdef __cplusplus


#endif

#endif /* __APP_TOOL_H */
