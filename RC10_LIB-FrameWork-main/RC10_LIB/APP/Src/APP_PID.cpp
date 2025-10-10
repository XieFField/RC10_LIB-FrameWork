#include "APP_PID.h"



float PID_Position::pid_calc(float target, float feedback)
{
    float current_time_s = TimeStamp::getInstance().getSeconds();
    dt_ = current_time_s - last_time_s_;

    if (isFirst_)
    {
        isFirst_ = false;
        // 在第一次计算时，dt 可能非常大或不确定，使用默认值
        dt_ = 0.01f; 
        error_last_ = target - feedback; // 初始化上次误差
        feedback_last_ = feedback;
    }

    // 对dt进行异常值处理
    if (dt_ <= 0.0f || dt_ > 0.1f) // 如果dt小于等于0或大于100ms，则认为异常
    {
        dt_ = 0.01f;
    }

    // calc error
    if (is_circular_)
    {
        
        if (feedback * target >= 0)
        {
            error_ = target - feedback;
        }
        else
        {
            if (feedback > 0 && target < 0)
            {
                float positive = (180.0f - feedback) + (180.0f + target); // 正路径
                float negative = target - feedback;
                if (fabsf(positive) <= fabsf(negative))
                    error_ = positive;
                
                    
                else
                    error_ = negative; // 选择一个较短的路径
                
            }
            else // (feedback < 0 && target > 0)
            {
                float positive = target - feedback;
                float negative = -((180.0f + feedback) + (180.0f - target));
                if (fabsf(positive) <= fabsf(negative))
                    error_ = positive;
                
                else
                    error_ = negative; // 选择一个较短的路径
                
            }
        }
    }
    else
        // 线性模式，直接计算误差
        error_ = target - feedback;
    


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
        D_Term = params_.kd * (feedback - feedback_last_) / dt_;
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