#ifndef TEST_TDD_APP_PID_H
#define TEST_TDD_APP_PID_H

#include "APP_tool.h"

struct PID_Param_Config
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float I_Outlimit = 0.0f;
    bool isIOutlimit = false;
    float output_limit = 0.0f;
    float deadband = 0.0f;
};

class PID_Position
{
public:
    PID_Position(PID_Param_Config = {}, float = 0.0f) {}

    void set_params(const PID_Param_Config &, float) {}
    void set_as_circular() {}
    void set_as_linear() {}

    float pid_calc(float target, float feedback)
    {
        last_target = target;
        last_feedback = feedback;
        calc_count += 1U;
        return forced_output;
    }

    float forced_output = 0.0f;
    float last_target = 0.0f;
    float last_feedback = 0.0f;
    unsigned int calc_count = 0U;
};

inline PID_Param_Config lock_angle_pid_params{};

#endif
