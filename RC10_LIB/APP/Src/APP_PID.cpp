#include "APP_PID.h"



float PID_Position::pid_calc(float target, float feedback)
{
    float current_time_s = TimeStamp::getInstance().getSeconds();
    dt_ = current_time_s - last_time_s_;

    if (isFirst_)
    {
        isFirst_ = false;
        // �ڵ�һ�μ���ʱ��dt ���ܷǳ����ȷ����ʹ��Ĭ��ֵ
        dt_ = dt_error_; 
        error_last_ = target - feedback; // ��ʼ���ϴ����
        feedback_last_ = feedback;
    }

    // ��dt�����쳣ֵ����
    if (dt_ <= 0.0f || dt_ > 0.1f) // ���dtС�ڵ���0�����100ms������Ϊ�쳣
    {
        dt_ = dt_error_;
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
                float positive = (180.0f - feedback) + (180.0f + target); // ��·��
                float negative = target - feedback;
                if (fabsf(positive) <= fabsf(negative))
                    error_ = positive;
                
                    
                else
                    error_ = negative; // ѡ��һ���϶̵�·��
                
            }
            else // (feedback < 0 && target > 0)
            {
                float positive = target - feedback;
                float negative = -((180.0f + feedback) + (180.0f - target));
                if (fabsf(positive) <= fabsf(negative))
                    error_ = positive;
                
                else
                    error_ = negative; // ѡ��һ���϶̵�·��
                
            }
        }
    }
    else
        // ����ģʽ��ֱ�Ӽ������
        error_ = target - feedback;
    


    if(fabs(error_) < params_.deadband)
        error_ = 0.0f;

    // calc P
    P_Term = params_.kp * error_;

    // calc I (���λ���)
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

    // calc D (΢������)
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

//����ʽ

void PID_Incremental::calc_track_D(float expect, float dt)
{
    //���׸���΢��
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

    // ��dt�����쳣ֵ����
    if (dt_ <= 0.0f)
    {
        dt_ = 0.001f;
    }

    // 1. �������td
    float current_target = target;
    if(td_ratio_ > 0.0f)
    {
        calc_track_D(target, dt_);
        current_target = td_v1_;
    }

    // 2. �������
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
        // 3. ����PID����
        // P������
        P_Term = params_.kp * (error_ - error_last_);

        // I������
        I_Term = params_.ki * error_;
        
        // D������
        if (dt_ > 0.0f)
        {
            D_Term = params_.kd * (error_ - 2.0f * error_last_ + error_earlier_);
        }
        else
        {
            D_Term = 0.0f;
        }

        // ���㵱ǰ����� = �ϴ������ + ����������
        output_ = output_last_ + (P_Term + I_Term + D_Term);
    }

    // ����޷�
    output_ = constrain(output_, -params_.output_limit, params_.output_limit);

    // ������ʷֵ
    error_earlier_ = error_last_;
    error_last_ = error_;
    output_last_ = output_; // ���浱ǰ���������Ϊ�´μ���ġ��ϴ��������
    last_time_s_ = current_time_s;

    return output_;
}

//Ŀǰ3508�����Ĳ��� 
PID_Param_Config m3508_speed_pid_params = {
    .kp = 32.0f,
    .ki = 0.085f,
    .kd = 0.0f,
    .I_Outlimit = 8000.0f, 
    .isIOutlimit = true, 
    .output_limit = 15000.0f,   
    .deadband = 0.5f 
};

PID_Param_Config m3508_angle_pid_params = {
    .kp = 32.0f,
    .ki = 0.0f,
    .kd = 1.1f,
    .I_Outlimit = 0.0f, 
    .isIOutlimit = true, 
    .output_limit = 400.0f,   
    .deadband = 0.5f // 
};

PID_Param_Config m2006_speed_pid_params = {
    .kp = 67.0f,   
    .ki = 1.0f, 
    .kd = 0.0f,
    .I_Outlimit = 8000.0f, 
    .isIOutlimit = true, 
    .output_limit = 80000.0f,   
    .deadband = 0.05f 
};

PID_Param_Config m2006_angle_pid_params = {
    .kp = 2.3f,
    .ki = 0.0f,
    .kd = 0.24,
    .I_Outlimit = 0.0f, 
    .isIOutlimit = true, 
    .output_limit = 480.0f,   
    .deadband = 0.09f 
};

PID_Param_Config track_pid_params = {
    .kp = 1.0f,
    .ki = 0.085f,
    .kd = 0.0f,
    .I_Outlimit = 8.0f, 
    .isIOutlimit = true, 
    .output_limit = 10.0f,   
    .deadband = 0.5f 
};