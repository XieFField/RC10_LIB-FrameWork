#ifndef TEST_BSP_TIMESTAMP_H
#define TEST_BSP_TIMESTAMP_H

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
        return now_s_;
    }

    static void setTestSeconds(float now_s)
    {
        now_s_ = now_s;
    }

private:
    static float now_s_;
};

#endif
