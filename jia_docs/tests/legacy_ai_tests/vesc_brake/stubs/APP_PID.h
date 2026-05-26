#ifndef TEST_APP_PID_H
#define TEST_APP_PID_H

#include <cstdint>
#include "APP_tool.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float I_Outlimit;
    bool isIOutlimit;
    float output_limit;
    float deadband;
} PID_Param_Config;

class PID_Incremental
{
public:
    PID_Incremental(PID_Param_Config params = {0.0f, 0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f},
                    float td_ratio = 0.0f)
        : params_(params), td_ratio_(td_ratio)
    {
    }

    void set_params(const PID_Param_Config& params, float td_ratio)
    {
        params_ = params;
        td_ratio_ = td_ratio;
    }

    void set_derivative_first(bool derivative_first)
    {
        derivative_first_ = derivative_first;
    }

    PID_Param_Config get_params() const
    {
        return params_;
    }

    float get_td_ratio() const
    {
        return td_ratio_;
    }

    bool get_derivative_first() const
    {
        return derivative_first_;
    }

    float pid_calc(float target, float feedback)
    {
        (void)td_ratio_;
        float error = target - feedback;
        if (error > -params_.deadband && error < params_.deadband)
            error = 0.0f;

        float output = params_.kp * error;
        return constrain(output, -params_.output_limit, params_.output_limit);
    }

private:
    PID_Param_Config params_;
    float td_ratio_ = 0.0f;
    bool derivative_first_ = false;
};

#endif
