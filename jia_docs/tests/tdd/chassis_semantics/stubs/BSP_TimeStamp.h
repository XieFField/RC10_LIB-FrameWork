#ifndef TEST_TDD_BSP_TIMESTAMP_H
#define TEST_TDD_BSP_TIMESTAMP_H

void testHostSetTimeSeconds(float seconds);
void testHostAdvanceTimeSeconds(float delta_seconds);
void testHostResetTimeSeconds();
float testHostGetTimeSeconds();

class TimeStamp
{
public:
    static TimeStamp &getInstance()
    {
        static TimeStamp instance;
        return instance;
    }

    float getSeconds() const
    {
        return testHostGetTimeSeconds();
    }
};

#endif
