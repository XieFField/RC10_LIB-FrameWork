#ifndef TEST_TDD_MODULE_CRSF_RECEIVER_H
#define TEST_TDD_MODULE_CRSF_RECEIVER_H

#include "APP_debugTool.h"

struct RmPocketData_t
{
    float left_y = 0.0f;
    float left_x = 0.0f;
    float right_x = 0.0f;
    float right_y = 0.0f;
};

class CrsfReceiver
{
public:
    static CrsfReceiver *GetInstance(UART_HandleTypeDef *)
    {
        static CrsfReceiver instance;
        return &instance;
    }

    void getControlData(RmPocketData_t *data)
    {
        if (data != nullptr)
        {
            *data = {};
        }
    }
};

#endif
