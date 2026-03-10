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
    // ����ԭʼ���
    error_ = target - feedback;

    if (is_circular_)
    {
        // ����ģʽ��Ѱ�����·��������������� [-180, 180]
        // �������Լ��� (-180~180) �� (0~360) ���ָ�ʽ
        while (error_ > 180.0f)
            error_ -= 360.0f;
        while (error_ < -180.0f)
            error_ += 360.0f;
    }
    


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
    {
        float diff_feedback = feedback - feedback_last_;
        if (is_circular_)
        {
            // ��������
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

    // PID output (without feedforward). Clamped first so the pure PID
    // contribution stays within the limit before the FF term is overlaid.
    float output = P_Term + I_Term - D_Term;
    output = constrain(output, -params_.output_limit, params_.output_limit);

    // Add feedforward: kf * target. Applied after the PID clamp so the
    // integrator (I_Term) is not inflated by the FF contribution.
    FF_Term = params_.kf * target;
    output += FF_Term;
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
        I_Term = constrain(I_Term, -params_.I_Outlimit, params_.I_Outlimit);
        
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
    output_last_ = output_; // ���浱ǰPID���������Ϊ�´μ���ġ��ϴ��������
    last_time_s_ = current_time_s;

    // ǰ���: kf * Ŀ��ֵ��
    // ʷ���¼(output_last_)ֻ�����PID�������ǰ�ò��֣��Ӷ�
    // ʹincrementalΰ����ǰ��Ь���ۼơ�
    FF_Term = params_.kf * current_target;
    output_ += FF_Term;
    output_ = constrain(output_, -params_.output_limit, params_.output_limit);

    return output_;
}

//Ŀǰ3508�����Ĳ��� 
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
    .output_limit = 100.0f,   
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
    .deadband = 0.01f 
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
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .I_Outlimit = 0.0f, 
    .isIOutlimit = true, 
    .output_limit = 1.0f,   
    .deadband = 0.0009f 
};

PID_Param_Config lock_angle_pid_params = {
 .kp = 0.175f /3.0f,
 .ki = 0.0f,
 .kd = 0.010f,
 .I_Outlimit = 0.0f, 
 .isIOutlimit = true, 
 .output_limit = 6.0f, 
 .deadband = 0.1f 
};

