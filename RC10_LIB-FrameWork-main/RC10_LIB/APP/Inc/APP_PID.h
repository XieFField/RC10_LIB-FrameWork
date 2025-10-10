/**
 * @file APP_PID.h
 * @author XieFField
 * @brief PID 控制器
 * @version 1.0
 * @date 2025-09-17
 */

/*
    此代码在算法上很大程度借鉴了洁宇哥的代码，在此表示感谢
    之所以没有直接使用，是因为里面的数学逻辑虽然合适，但是
    代码风格和结构不像是C++，而是把类当成了结构体来使用，
    我认为破坏了封装性，不够健壮。
    =================================================
    里面在算法上，我认为有些地方可以改进，主要问题在于采样
    时间的计算上，以及积分抗饱和的逻辑，当积分项饱和直接清
    除积分项，有点过于激进，可能引起系统震荡。
    =================================================
*/

#ifndef __APP_PID_H
#define __APP_PID_H

#pragma once

#ifdef __cplusplus

extern "C" {

#include "stm32h7xx_hal.h" 

}
#include <cstdint>
#include <stdbool.h>
#include <cmath>
#include "APP_tool.h"
#include "BSP_TimeStamp.h"

/*
    ______ _            __  _____  ____________     ______
   /  _/ /( )_____     /  |/  /\ \/ / ____/ __ \   / / / /
   / // __/// ___/    / /|_/ /  \  / / __/ / / /  / / / / 
 _/ // /_  (__  )    / /  / /   / / /_/ / /_/ /  /_/_/_/  
/___/\__/ /____/    /_/  /_/   /_/\____/\____/  (_|_|_)   
*/                                                          

typedef struct {
    float kp;
    float ki;
    float kd;
    float I_Outlimit; // 积分限幅
    bool isIOutlimit; // 是否启用积分限幅
    float output_limit;   // 输出限幅
    float deadband;      // 死区
} PID_Param_Config;

// 位置式 PID 控制器，采用了梯形积分，微分先行，积分分离的改进式PID 默认控制频率100Hz
class PID_Position {
public:
    /**
     * @brief 构造函数
     * @param pid_Param_Config PID 参数配置结构体
     * @param I_SeparaThreshold 积分分离阈值，当误差绝对值大于该值时，积分项不累加
     */
    PID_Position(PID_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f}, 
        float I_SeparaThreshold = 0.0f)
        : params_(params), I_SeparaThreshold_(I_SeparaThreshold) 
        {
            reset();
        }

    bool max_output = false; //积分抗饱和

    /**
     * @brief 计算PID输出
     * @param target 目标值
     * @param feedback 当前测量值
     * @return PID控制输出
     */
    float pid_calc(float target, float feedback);
    

    /**
     * @brief 重置PID控制器的内部状态
     *        主要用于重置积分项和历史值。
     */
    void reset()
    {
        I_Term = 0.0f;
        P_Term = 0.0f;
        D_Term = 0.0f;
        output_ = 0.0f;
        error_last_ = 0.0f;
        feedback_last_ = 0.0f;
        isFirst_ = true; // 重置 isFirst_ 标志
    }

    /**
     * @brief 动态修改PID参数
     * @param params 新的PID参数
     */
    void set_params(const PID_Param_Config& params, float I_SeparaThreshold);

    /**
     * @brief 将PID设定为循环模式
     * @param range 循环范围，例如360度
     * @brief 设置为循环模式的时候，PID会自动选择最近的路径，比如
     *        350度到10度，会正着走20度，而不是倒着走340度
     * @param offset 偏移量，默认为0
     */
    void set_as_circular()
    {
        is_circular_ = true;
    }

    /**
     * @brief 将PID设置为单向模式
     */
    void set_as_linear()
    {
        is_circular_ = false;
    }

    float get_P_Term() const { return P_Term; }
    float get_I_Term() const { return I_Term; }
    float get_D_Term() const { return D_Term; }
    float get_dt() const { return dt_; }

private:
    float I_Term = 0;			/* 积分器输出 */
    float P_Term = 0;			/* 比例器输出 */
    float D_Term = 0;			/* 微分器输出 */
    float output_ = 0.0f;     // 输出值
    PID_Param_Config params_;
    float I_SeparaThreshold_;

    float error_ = 0.0f;              // 当前误差
    float error_last_ = 0.0f;       // 上次误差
    float feedback_last_ = 0.0f;    // 上次反馈值

    float dt_ = 0.001f;             // 采样时间，单位秒
    float last_time_s_ = 0.0f;      // 上次调用的时间，单位秒
    bool isFirst_ = true; // 是否为第一次计算

    // 循环设定
    bool is_circular_ = false;
};

// 增量式 PID 控制器 加入微分跟踪器
class PID_Incremental {
public:
    /** 
     * @brief 构造函数
     * @param params PID 参数配置结构体
     * @param td_ratio 微分跟踪器跟踪因子，范围0.0~1.0，0表示不启用微分跟踪器.td_ratio越大跟踪越快。
     *
    */
    PID_Incremental(PID_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f}, float td_ratio = 0.0f)
        : params_(params), td_ratio_(td_ratio) 
    {
        reset();
    }

        /**
         * @brief 计算增量式PID输出
         * @param target 目标值
         * @param feedback 当前测量值
         * @return PID控制输出
         */
        float pid_calc(float target, float feedback);

        /**
         * @brief 重置PID控制器的内部状态
         *        主要用于重置积分项和历史值。
         */
        void reset()
        {
            error_ = 0.0f;
            error_last_ = 0.0f;
            error_earlier_ = 0.0f;
            output_ = 0.0f; // 重置 output_
            output_last_ = 0.0f;
            td_v1_ = 0.0f;
            td_v2_ = 0.0f;
            isFirst_ = true; // 重置 isFirst_ 标志
        }

        /**
         * @brief 动态修改PID参数
         * @param params 新的PID参数
         */
        void set_params(const PID_Param_Config& params, float td_ratio);

        float get_dt() const { return dt_; }
        
private:
    void calc_track_D(float expect, float dt); //微分跟踪器
    bool isFirst_ = true; // 是否为第一次计算

    float error_ = 0.0f, error_last_ = 0.0f, error_earlier_ = 0.0f; // 当前误差，上次误差，上上次误差
    float output_ = 0.0f;       // 当前总输出
    float output_last_ = 0.0f;  // 上次总输出

    PID_Param_Config params_;

    float I_Term = 0.0f; // 积分器输出
    float P_Term = 0.0f; // 比例器输出
    float D_Term = 0.0f; // 微分器输出

    float td_ratio_ = 0.0f; // track_D跟踪因子

    float td_v1_ = 0.0f; //跟踪的目标位置
    float td_v2_ = 0.0f; //跟踪的目标速度

    float dt_ = 0.001f;             // 采样时间，单位秒
    float last_time_s_ = 0.0f;      // 上次调用的时间，单位秒
};


#endif

#endif