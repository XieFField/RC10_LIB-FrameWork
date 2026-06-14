/**
 * @file APP_PID.h
 * @author XieFField
 * @brief PID 类
 * @version 1.0
 * @date 2025-09-17
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
    float I_Outlimit; //  I限幅
    bool isIOutlimit; //  I限幅开关
    float output_limit;   //  输出限幅
    float deadband;      //  死区
} PID_Param_Config;

// PID 位置环，默认100Hz更新频率
class PID_Position {
public:
    /**
     * @brief 构造函数
     * @param pid_Param_Config PID 参数
     * @param I_SeparaThreshold I分离阈值
     */
    PID_Position(PID_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f}, 
        float I_SeparaThreshold = 0.0f)
        : params_(params), I_SeparaThreshold_(I_SeparaThreshold) 
        {
            reset();
        }

    bool max_output = false; // 是否达到输出限幅

    /**
     * @brief PID 计算
     * @param target 目标值
     * @param feedback 反馈值
     * @return PID输出
     */
    float pid_calc(float target, float feedback);
    

    /**
     * @brief 重置PID状态
        *        需要在使用前调用，以清除历史值
        */
    void reset()
    {
        I_Term = 0.0f;
        P_Term = 0.0f;
        D_Term = 0.0f;
        output_ = 0.0f;
        error_last_ = 0.0f;
        feedback_last_ = 0.0f;
        isFirst_ = true; // ���� isFirst_ ��־
    }

    /**
     * @brief 设置PID参数
     * @param params 参数结构体
     * @param I_SeparaThreshold I分离阈值
     */
    void set_params(const PID_Param_Config& params, float I_SeparaThreshold);

    PID_Param_Config get_params() const { return params_; }
    float get_i_separa_threshold() const { return I_SeparaThreshold_; }
    /**
     * @brief 将PID设置为循环模式
     * @param range 循环范围，通常为360度
     * @brief 设置为循环模式时，PID会自动选择最优的偏差路径
     *        例如：350度到10度，会选择经过360度的路径
     * @param offset 偏移量，默认为0
     */
    void set_as_circular()
    {
        is_circular_ = true;
    }

    /**
     * @brief 将PID设置为线性模式
     */
    void set_as_linear()
    {
        is_circular_ = false;
    }

    bool get_is_in_dead_zone() const { return is_in_dead_zone_; }

    float get_P_Term() const { return P_Term; }
    float get_I_Term() const { return I_Term; }
    float get_D_Term() const { return D_Term; }
    float get_dt() const { return dt_; }

    void reset_dt_error(float set)
    {
        dt_error_ = set;
    }
private:
    float I_Term = 0;			/* 积分项 */
    float P_Term = 0;			/* 比例项 */
    float D_Term = 0;			/* 微分项 */
    float output_ = 0.0f;     // 输出
    PID_Param_Config params_;
    float I_SeparaThreshold_;

    float error_ = 0.0f;              // 当前误差
    float error_last_ = 0.0f;       // 上一次误差
    float feedback_last_ = 0.0f;    // 上一次反馈值

    float dt_ = 0.01f;             // 采样时间，单位秒
    float last_time_s_ = 0.0f;      // 上一次采样时间，单位秒
    bool isFirst_ = true; // 是否为第一次
    float dt_error_ = 0.01f; //dt默认值
    // 循环设置
    bool is_circular_ = false;

    bool is_in_dead_zone_ = false;
};

// 增量式PID 默认1kHz更新频率
class PID_Incremental {
public:
    /** 
     * @brief 构造函数
     * @param params PID 参数结构体
     * @param td_ratio 微分系数比例，范围0.0~1.0，0表示不使用微分项。td_ratio越大，微分项影响越大
     *
    */
    PID_Incremental(PID_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f},
                    float td_ratio = 0.0f)
        : params_(params), td_ratio_(td_ratio)
    {
        reset();
    }

        /**
         * @brief 增量式PID计算
         * @param target 目标值
         * @param feedback 当前反馈值
         * @return PID输出
         */
        float pid_calc(float target, float feedback);

        /**
         * @brief 重置PID状态
         *        需要在使用前调用，以清除历史值
         */
        void reset()
        {
            error_ = 0.0f;
            error_last_ = 0.0f;
            error_earlier_ = 0.0f;
            output_ = 0.0f; // output_
            output_last_ = 0.0f;
            td_v1_ = 0.0f;
            td_v2_ = 0.0f;
            feedback_last_ = 0.0f;
            feedback_earlier_ = 0.0f;
            isFirst_ = true; //  isFirst_
        }

        /**
         * @brief 增量式PID参数设置
         * @param params PID参数结构体
         * @param td_ratio 微分系数比例
         */
        void set_params(const PID_Param_Config& params, float td_ratio);
        // 独立开关微分先行，默认关闭，避免把该策略耦合进通用参数初始化入口。
        void set_derivative_first(bool derivative_first) { derivative_first_ = derivative_first; }

        PID_Param_Config get_params() const { return params_; }
        float get_td_ratio() const { return td_ratio_; }
        bool get_derivative_first() const { return derivative_first_; }

        float get_dt() const { return dt_; }
        
private:
    void calc_track_D(float expect, float dt); // 计算跟踪微分项
    bool isFirst_ = true; // 是否为第一次

    float error_ = 0.0f, error_last_ = 0.0f, error_earlier_ = 0.0f; // 当前误差、上一次误差、上上次误差
    float output_ = 0.0f;       // 当前输出
    float output_last_ = 0.0f;  // 上一次输出

    PID_Param_Config params_;

    float I_Term = 0.0f; // I项
    float P_Term = 0.0f; // P项
    float D_Term = 0.0f; // D项

    float td_ratio_ = 0.0f; // track_D占微分项的比例，范围0.0~1.0
    // 微分先行需要保留反馈历史值；关闭时仍沿用原有误差差分路径。
    bool derivative_first_ = false; // 是否启用微分先行（对反馈量做微分）

    float td_v1_ = 0.0f; // 跟踪微分项的第一部分，计算跟踪误差的微分
    float td_v2_ = 0.0f; // 跟踪微分项的第二部分，计算跟踪误差的二阶微分
    float feedback_last_ = 0.0f;    // 上一次反馈值
    float feedback_earlier_ = 0.0f; // 上上次反馈值

    float dt_ = 0.001f;             // 默认采样时间，单位秒
    float last_time_s_ = 0.0f;      // 上一次采样时间，单位秒
};

typedef struct {
    float kp;
    float ki;
    float kv;
    float out_lim;
    float i_lim;
    float i_err;
    float ref_rate;
    float cam_gain;
    float cam_db;
    float cam_gate;
    float cam_delay;
    float done_err;
    float done_vel;
    float done_time;
} CamZ_Param;

class CamZ_Ctrl {
public:
    CamZ_Ctrl(CamZ_Param param = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}) : param_(param) { reset(0.0f); } // ctor

    void set_param(const CamZ_Param& param) { param_ = param; } // set param
    void reset(float z_now); // reset state
    float run_step(float z_ref, float z_cam, bool cam_new, float z_vel); // one step

    bool is_done() const { return done_; } // done flag
    float get_est() const { return z_est_; } // estimated z
    float get_ref() const { return z_ref_; } // smooth ref
    float get_err() const { return z_err_; } // control err

private:
    void step_ref(float z_ref, float dt); // ref slew
    void fuse_cam(float z_cam, float z_vel); // delayed cam fuse
    void step_done(float z_vel, float dt); // done check

    CamZ_Param param_;

    float z_est_ = 0.0f;
    float z_ref_ = 0.0f;
    float z_err_ = 0.0f;
    float i_sum_ = 0.0f;

    float dt_ = 0.01f;
    float last_t_ = 0.0f;
    bool first_ = true;

    bool done_ = false;
    float done_t_ = 0.0f;
};


extern PID_Param_Config m3508_speed_pid_params;
extern PID_Param_Config m3508_angle_pid_params;

extern PID_Param_Config m2006_speed_pid_params;
extern PID_Param_Config m2006_angle_pid_params;
extern PID_Param_Config lock_angle_pid_params;
extern PID_Param_Config track_pid_params;

extern PID_Param_Config camera_x_pid_params;
extern PID_Param_Config camera_y_pid_params;
extern PID_Param_Config camera_vec_pid_params;
extern PID_Param_Config camera_yaw_pid_params;

extern PID_Param_Config m3508Rotate_speed_pid_params;
extern PID_Param_Config m3508Rotate_angle_pid_params;

extern PID_Param_Config path_lock_end;
extern PID_Param_Config path_lock_R2;

extern PID_Param_Config m3508_speed_pid_paramsForSpeedMotor;
extern PID_Param_Config foursteer_steer_speed_pid_params;
extern PID_Param_Config foursteer_steer_angle_pid_params;
extern PID_Param_Config vesc_drive_speed_pid_params;

extern PID_Param_Config omega_z_pid_init_config;
extern PID_Param_Config rot_z_pid_init_config;

extern CamZ_Param camera_z_ctrl_params;
#endif

#endif
