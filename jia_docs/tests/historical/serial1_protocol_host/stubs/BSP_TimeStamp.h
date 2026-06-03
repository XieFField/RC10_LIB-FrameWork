#ifndef TEST_SERIAL1_BSP_TIMESTAMP_H
#define TEST_SERIAL1_BSP_TIMESTAMP_H

#include <cstdint>

extern uint32_t g_test_time_ms;

class TimeStamp
{
public:
    static TimeStamp &getInstance()
    {
        static TimeStamp instance;
        return instance;
    }

    uint32_t getMilliseconds() const
    {
        return g_test_time_ms;
    }
};

#endif
