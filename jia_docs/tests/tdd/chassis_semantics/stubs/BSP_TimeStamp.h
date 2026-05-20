#ifndef TEST_TDD_BSP_TIMESTAMP_H
#define TEST_TDD_BSP_TIMESTAMP_H

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
        return 0.0f;
    }
};

#endif
