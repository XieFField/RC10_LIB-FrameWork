#include "APP_PID.h"



float PID_Position::pid_calc(float target, float feedback)
{
    float current_time_s = TimeStamp::getInstance().getSeconds();
    dt_ = current_time_s - last_time_s_;

    if (isFirst_)
    {
        isFirst_ = false;
        // 在第一次计算时，dt 可能非常大或不确定，使用默认值
        dt_ = dt_error_; 
        error_last_ = target - feedback; // 初始化上次误差
        feedback_last_ = feedback;
    }

    // 对dt进行异常值处理
    if (dt_ <= 0.0f || dt_ > 0.1f) // 如果dt小于等于0或大于100ms，则认为异常
    {
        dt_ = dt_error_;
    }

    // calc error
    // 计算原始误差
    error_ = target - feedback;

    if (is_circular_)
    {
        // 环形模式：寻找最短路径，将误差限制在 [-180, 180]
        // 这样可以兼容 (-180~180) 和 (0~360) 两种格式
        while (error_ > 180.0f)
            error_ -= 360.0f;
        while (error_ < -180.0f)
            error_ += 360.0f;
    }
    


    if(fabs(error_) < params_.deadband)
        error_ = 0.0f;

    // calc P
    P_Term = params_.kp * error_;

    // calc I (梯形积分)
    if(fabsf(error_) < I_SeparaThreshold_ && I_SeparaThreshold_ > 0)
    {
        I_Term += params_.ki * (error_ + error_last_) * dt_ / 2.0f;
        if(params_.isIOutlimit == true)
            I_Term = constrain(I_Term, -params_.I_Outlimit, params_.I_Outlimit);
    }
    else
    {
        I_Term = 0;
    }

    // calc D (微分先行)
    if (dt_ > 0.0f)
    {
        float diff_feedback = feedback - feedback_last_;
        if (is_circular_)
        {
            // 处理环形
            if (diff_feedback > 180.0f)
                diff_feedback -= 360.0f;
            else if (diff_feedback < -180.0f)
                diff_feedback += 360.0f;
        }
        D_Term = params_.kd * diff_feedback / dt_;
    }
    else
        D_Term = 0.0f;
    

    //update history
    error_last_ = error_;
    feedback_last_ = feedback;
    last_time_s_ = current_time_s;

    float output = P_Term + I_Term - D_Term;
    output = constrain(output, -params_.output_limit, params_.output_limit);

    output_ = output;

    return output_;
}

void PID_Position::set_params(const PID_Param_Config& params, float I_SeparaThreshold)
{
    params_ = params;
    I_SeparaThreshold_ = I_SeparaThreshold;
}


/* =================================================================================== */

//增量式

void PID_Incremental::calc_track_D(float expect, float dt)
{
    //二阶跟踪微分
    float fh = -td_ratio_ * td_ratio_ *(td_v1_ - expect) - 2.0f * td_v2_ * td_ratio_;

    td_v1_ += td_v2_ * dt;
    td_v2_ += fh * dt;
}

void PID_Incremental::set_params(const PID_Param_Config& params, float td_ratio)
{
    params_ = params;
    td_ratio_ = td_ratio;
}

float PID_Incremental::pid_calc(float target, float feedback)
{
    float current_time_s = TimeStamp::getInstance().getSeconds();
    dt_ = current_time_s - last_time_s_;

    // 对dt进行异常值处理
    if (dt_ <= 0.0f)
    {
        dt_ = 0.001f;
    }

    // 1. 如果启用td
    float current_target = target;
    if(td_ratio_ > 0.0f)
    {
        calc_track_D(target, dt_);
        current_target = td_v1_;
    }

    // 2. 计算误差
    error_ = current_target - feedback;
    if(fabs(error_) < params_.deadband)
        error_ = 0.0f;

    if (isFirst_)
    {
        error_last_ = 0;
        error_earlier_ = 0;
        isFirst_ = false;
        output_ = 0.0f; 
    }
    else
    {
        // 3. 计算PID增量
        // P项增量
        P_Term = params_.kp * (error_ - error_last_);

        // I项增量
        I_Term = params_.ki * error_;
        I_Term = constrain(I_Term, -params_.I_Outlimit, params_.I_Outlimit);
        
        // D项增量
        if (dt_ > 0.0f)
        {
            D_Term = params_.kd * (error_ - 2.0f * error_last_ + error_earlier_);
        }
        else
        {
            D_Term = 0.0f;
        }

        // 计算当前总输出 = 上次总输出 + 本次总增量
        output_ = output_last_ + (P_Term + I_Term + D_Term);
    }

    // 输出限幅
    output_ = constrain(output_, -params_.output_limit, params_.output_limit);

    // 更新历史值
    error_earlier_ = error_last_;
    error_last_ = error_;
    output_last_ = output_; // 保存当前总输出，作为下次计算的“上次总输出”
    last_time_s_ = current_time_s;

    return output_;
}

//目前3508不错的参数 
PID_Param_Config m3508_speed_pid_params = {
    .kp = 32.0f,
    .ki = 0.085f,
    .kd = 0.0f,
    .I_Outlimit = 8000.0f, 
    .isIOutlimit = true, 
    .output_limit = 12000.0f,   
    .deadband = 0.5f 
};

PID_Param_Config m3508_angle_pid_params = {
    .kp = 3.5f,
    .ki = 0.0f,
    .kd = 0.05f,
    .I_Outlimit = 0.0f, 
    .isIOutlimit = true, 
    .output_limit = 500.0f,   
    .deadband = 0.03f // 
};

PID_Param_Config m3508Rotate_angle_pid_params = {
    .kp = 3.5f,
    .ki = 0.0f,
    .kd = 0.00f,
    .I_Outlimit = 0.0f, 
    .isIOutlimit = true, 
    .output_limit = 80.0f,   
    .deadband = 0.03f // 
};



PID_Param_Config m2006_speed_pid_params = {
    .kp = 300.0f,  
    .ki = 12.0f, 
    .kd = 0.0f,
    .I_Outlimit = 5000.0f, 
    .isIOutlimit = true, 
    .output_limit = 6000.0f,   
    .deadband = 0.05f 
};

PID_Param_Config m2006_angle_pid_params = {
    .kp = 3.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .I_Outlimit = 0.0f, 
    .isIOutlimit = true, 
    .output_limit = 450.0f,   
    .deadband = 0.03f 
};

PID_Param_Config m3508_speed_pid_paramsForSpeedMotor = {
    .kp =  250.0f,
    .ki = 12.0f,
    .kd = 0.0f,
    .I_Outlimit = 8000.0f, 
    .isIOutlimit = true, 
    .output_limit = 15000.0f,   
    .deadband = 0.1f 
};

PID_Param_Config track_pid_params = {
    .kp = 6.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .I_Outlimit = 0.0f, 
    .isIOutlimit = true, 
    .output_limit = 1.5f,   
    .deadband = 0.0009f 
};

PID_Param_Config lock_angle_pid_params = {
 .kp = 0.075f,
 .ki = 0.0f,
 .kd = 0.010f,
 .I_Outlimit = 0.0f, 
 .isIOutlimit = true, 
 .output_limit = 3.0f, 
 .deadband = 0.1f 
};


PID_Param_Config omega_z_pid_init_config =
{
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .I_Outlimit = 0.0f,
    .isIOutlimit = false,
    .output_limit = 0.0f,
    .deadband = 0.0f,
};

PID_Param_Config rot_z_pid_init_config = {
    .kp = 4.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .I_Outlimit = 0.0f,
    .isIOutlimit = false,
    .output_limit = 4.0f,
    .deadband = 0.0f
};
//PID_Param_Config path_lock_end = {
//    
//    .kp = -0.7f,
//    .ki = 0.0f,
//    .kd = 0.0f,
//    .I_Outlimit = 0.0f, 
//    .isIOutlimit = true, 
//    .output_limit = 1.0f,   
//    .deadband = 0.0009f 
//};