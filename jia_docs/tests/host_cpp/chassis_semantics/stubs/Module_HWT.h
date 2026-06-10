#ifndef TEST_TDD_MODULE_HWT_H
#define TEST_TDD_MODULE_HWT_H

#include "APP_debugTool.h"

class HWT101CT
{
public:
    static HWT101CT *GetInstance(UART_HandleTypeDef *)
    {
        static HWT101CT instance;
        return &instance;
    }

    float get_yaw_rad() const
    {
        return 0.0f;
    }

    float get_yaw_speed_rad() const
    {
        return 0.0f;
    }
};

#endif
