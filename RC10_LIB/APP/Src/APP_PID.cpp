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

void CamZ_Ctrl::reset(float z_now)
{
    z_est_ = z_now; // 估计值对齐当前高度。
    z_ref_ = z_now; // 平滑参考对齐当前高度。
    z_err_ = 0.0f; // 清误差。
    i_sum_ = 0.0f; // 清积分。

    dt_ = 0.01f; // 重置默认周期。
    last_t_ = TimeStamp::getInstance().getSeconds(); // 记录当前时间。
    first_ = true; // 标记下一次为首帧。

    done_ = false; // 清完成位。
    done_t_ = 0.0f; // 清完成累计时间。
}

void CamZ_Ctrl::step_ref(float z_ref, float dt)
{
    float max_step = param_.ref_rate * dt; // 本周期参考最大变化量。
    float delta = z_ref - z_ref_; // 目标与平滑参考的差值。

    if (delta > max_step)
    {
        z_ref_ += max_step; // 正向限速跟踪。
    }
    else if (delta < -max_step)
    {
        z_ref_ -= max_step; // 反向限速跟踪。
    }
    else
    {
        z_ref_ = z_ref; // 差值在限速内直接对齐。
    }
}

void CamZ_Ctrl::fuse_cam(float z_cam, float z_vel)
{
    float z_now = z_cam + z_vel * param_.cam_delay; // 用速度将延迟样本外推到当前。
    float dz = z_now - z_est_; // 创新量: 相机与估计的差。

    if (fabsf(dz) < param_.cam_db)
    {
        return; // 小于死区视为噪声不融合。
    }

    if (fabsf(dz) > param_.cam_gate)
    {
        return; // 大于门限视为异常值不融合。
    }

    z_est_ += param_.cam_gain * dz; // 小权重纠偏避免跳变。
}

void CamZ_Ctrl::step_done(float z_vel, float dt)
{
    if (fabsf(z_err_) < param_.done_err && fabsf(z_vel) < param_.done_vel)
    {
        done_t_ += dt; // 误差和速度都满足时累计稳定时间。
    }
    else
    {
        done_t_ = 0.0f; // 条件破坏则重新计时。
    }

    done_ = (done_t_ >= param_.done_time); // 达到最小持续时间才判定完成。
}

float CamZ_Ctrl::run_step(float z_ref, float z_cam, bool cam_new, float z_vel)
{
    float now_t = TimeStamp::getInstance().getSeconds(); // 读取当前时间。
    dt_ = now_t - last_t_; // 计算离散周期。

    if (first_)
    {
        first_ = false; // 仅首次进入执行。
        dt_ = 0.01f; // 首帧强制默认周期防突变。
    }

    if (dt_ <= 0.0f || dt_ > 0.1f)
    {
        dt_ = 0.01f; // 周期异常回退默认值。
    }

    last_t_ = now_t; // 更新时间戳。

    z_est_ += z_vel * dt_; // 编码器速度积分做高频预测。

    step_ref(z_ref, dt_); // 目标先做斜坡限速。

    if (cam_new)
    {
        fuse_cam(z_cam, z_vel); // 新相机样本到达时再融合。
    }

    z_err_ = z_ref_ - z_est_; // 位置误差。
    if (fabsf(z_err_) < param_.i_err && param_.i_err > 0.0f)
    {
        i_sum_ += z_err_ * dt_; // 小误差区积分消静差。
    }

    if (param_.i_lim > 0.0f)
    {
        i_sum_ = constrain(i_sum_, -param_.i_lim, param_.i_lim); // 积分限幅防风up。
    }

    float out = param_.kp * z_err_ + param_.ki * i_sum_ - param_.kv * z_vel; // PI加速度阻尼项。
    out = constrain(out, -param_.out_lim, param_.out_lim); // 输出限幅为目标rpm。

    step_done(z_vel, dt_); // 更新到位判定。

    return out; // 返回 launch rpm 指令。
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

    output_last_ = output_; // 保存当前总输出，作为下次计算的“上次总输出”

    return output_;
}


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
    .kp = 5.0f,
    .ki = 0.0f,
    .kd = 0.05f,
    .I_Outlimit = 0.0f, 
    .isIOutlimit = true, 
    .output_limit = 150.0f,   
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

// 四舵轮舵向电机 PID
PID_Param_Config foursteer_steer_speed_pid_params = {
    // .kp = 150.0f,
    // .ki = 0.8f,
    // .kd = 0.0f,
    // .I_Outlimit = 8000.0f,
    // .isIOutlimit = true,
    // .output_limit = 8000.0f,
    // .deadband = 0.0f
    .kp = 250.0f / (3591.0f/187.0f) * 8.0f,
    .ki = 12.0f / (3591.0f/187.0f) * 8.0f,
    .kd = 0.0f / (3591.0f/187.0f) * 8.0f,
    .I_Outlimit = 8000.0f, 
    .isIOutlimit = true, 
    .output_limit = 15000.0f,   
    .deadband = 0.1f * (3591.0f/187.0f) / 8.0f 
};

// 四舵轮 VESC 驱动轮默认速度环 PID
PID_Param_Config vesc_drive_speed_pid_params = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .I_Outlimit = 20000.0f,
    .isIOutlimit = true,
    .output_limit = 20000.0f,
    .deadband = 0.0f
};

PID_Param_Config foursteer_steer_angle_pid_params = {
    // .kp = 5.0f,
    // .ki = 0.0f,
    // .kd = 0.0f,
    // .I_Outlimit = 0.0f,
    // .isIOutlimit = true,
    // .output_limit = 200.0f,
    // .deadband = 0.1f
    .kp = 3.5f / (3591.0f/187.0f) * 8.0f,
    .ki = 0.0f / (3591.0f/187.0f) * 8.0f,
    .kd = 0.05f / (3591.0f/187.0f) * 8.0f,
    .I_Outlimit = 0.0f, 
    .isIOutlimit = true, 
    .output_limit = 1050.0f,   
    // .deadband = 0.03f * (3591.0f/187.0f) / 8.0f
    .deadband = 0.08f
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

PID_Param_Config camera_x_pid_params = {
    .kp = 6.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .I_Outlimit = 0.0f,
    .isIOutlimit = true,
    .output_limit = 0.05f,
    .deadband = 0.002f
};

PID_Param_Config camera_y_pid_params = {
    .kp = 6.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .I_Outlimit = 0.0f,
    .isIOutlimit = true,
    .output_limit = 0.05f,
    .deadband = 0.01f
};

PID_Param_Config camera_vec_pid_params = {
    .kp = 6.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .I_Outlimit = 0.0f,
    .isIOutlimit = true,
    .output_limit = 0.1f,
    .deadband = 0.003f
};

PID_Param_Config lock_angle_pid_params = {
 .kp = 0.10f,
 .ki = 0.0f,
 .kd = 0.00f,
 .I_Outlimit = 0.0f, 
 .isIOutlimit = true, 
 .output_limit = 1.0f, 
 .deadband = 0.0f 
};

PID_Param_Config camera_yaw_pid_params = {
 .kp = 0.075f,
 .ki = 0.0f,
 .kd = 0.010f,
 .I_Outlimit = 0.0f,
 .isIOutlimit = true,
 .output_limit = 0.05f,
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
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .I_Outlimit = 0.0f,
    .isIOutlimit = false,
    .output_limit = 0.0f,
    .deadband = 0.0f
};

CamZ_Param camera_z_ctrl_params = {
    .kp = 1800.0f,
    .ki = 80.0f,
    .kv = 40.0f,
    .out_lim = 4500.0f,
    .i_lim = 3.0f,
    .i_err = 0.004f,
    .ref_rate = 0.03f,
    .cam_gain = 0.1f,
    .cam_db = 0.0015f,
    .cam_gate = 0.008f,
    .cam_delay = 0.06f,
    .done_err = 0.003f,
    .done_vel = 0.004f,
    .done_time = 0.12f,
};

PID_Param_Config path_lock_end = {
    
    .kp = 0.6f,
    .ki = 0.0f,
    .kd = 0.0f,
    .I_Outlimit = 0.0f, 
    .isIOutlimit = true, 
    .output_limit = 0.2f,   
    .deadband = 0.005f 
};

