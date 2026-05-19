/**
 * @file   APP_tool.h
 * @brief  通用工具函数头文件
 * @author XieFField
 */

#ifndef __APP_TOOL_H
#define __APP_TOOL_H

#include <stdint.h>
#include <cmath>

#include "BSP_TimeStamp.h"

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
    float x = 0.0f, y = 0.0f;
    float theta = 0.0f; // 旋转角度，单位弧度
} Point2D;

// 3D点结构体
typedef struct {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float theta = 0.0f;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f; // 欧拉角，单位弧度
}Point3D;

typedef struct {
    float vx = 0.0f;
    float vy = 0.0f;
    float vz = 0.0f;

    float yaw_rate = 0.0f;
    float pitch_rate = 0.0f;
    float roll_rate = 0.0f;
    
}Robot_Twist;

typedef struct {
    float yaw_rate = 0.0f;
    float pitch_rate = 0.0f;
    float roll_rate = 0.0f;

    float yaw_angle = 0.0f;
    float pitch_angle = 0.0f;
    float roll_angle = 0.0f;
    
}Angle_Twist;


#ifdef __cplusplus

//按钮检测器，目前支持检测单击双击  
class ButtonDetector
{
public:
    /**
     * @param double_click_time 双击判定时间 默认350ms
     * 单位毫秒
     */
    ButtonDetector(float double_click_time = 0.350f) : DOUBLE_CLICK_INTERVAL(double_click_time)
    {

    }

    enum class State{
        Idle, //空闲
        WaitRealse, //等待松开
        WaitNextClick, //等待下一次按下
    };

    /**
     * @param is_press 是否按下 0 没按 1按下
     * @return 事件类型 0 无事件 1 单击 2 双击 3 三击 4 四击...
     */
    uint8_t update(uint8_t is_press)
    {
        uint8_t event = 0;
        float nowtime = TimeStamp::getInstance().getSeconds();

        switch (this->state)
        {
            case State::Idle:
            {
                if(is_press)
                {
                    this->state = State::WaitRealse;
                    last_action_time = nowtime; //第一次按下时间
                    click_count = 1;
                }
                break;
            }
            
            case State::WaitRealse:
            {
                if(!is_press)
                {
                    last_action_time = nowtime; //松开时间
                    this->state = State::WaitNextClick;
                }
                break;  
            }
            
            case State::WaitNextClick:
            {
                bool timeout = (nowtime - last_action_time) > DOUBLE_CLICK_INTERVAL;

                if(timeout)
                {
                    event = click_count; //达到超时时间，结算按键次数
                    this->state = State::Idle;
                    click_count = 0;
                }
                else if(is_press)
                {
                    click_count++;
                    this->state = State::WaitRealse; 
                    last_action_time = nowtime; //再次按下时间
                }
                break;
            }

            default:
                break;
        }
        return event;
    }

private:
    float DOUBLE_CLICK_INTERVAL = 0.350f; //连击判定时间，单位秒
    State state = State::Idle;
    uint8_t click_count = 0;
    float last_action_time = 0;
};


#endif

#endif /* __APP_TOOL_H */
