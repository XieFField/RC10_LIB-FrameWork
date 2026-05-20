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

    float pid_calc(float, float)
    {
        return 0.0f;
    }
};

inline PID_Param_Config lock_angle_pid_params{};

#endif
