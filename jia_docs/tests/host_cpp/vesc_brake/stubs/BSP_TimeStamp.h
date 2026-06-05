#ifndef TEST_BSP_TIMESTAMP_H
#define TEST_BSP_TIMESTAMP_H

class TimeStamp
{
public:
    static TimeStamp& getInstance()
    {
        static TimeStamp instance;
        return instance;
    }

    float getSeconds()
    {
        current_time_s_ += 0.001f;
        return current_time_s_;
    }

private:
    float current_time_s_ = 0.0f;
};

#endif
