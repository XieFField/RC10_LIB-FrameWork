#ifndef TEST_PID_RECONNECT_BSP_TIMESTAMP_H
#define TEST_PID_RECONNECT_BSP_TIMESTAMP_H

inline float g_pid_reconnect_test_time_seconds = 0.0f;

inline void testPidReconnectSetTimeSeconds(float seconds)
{
    g_pid_reconnect_test_time_seconds = seconds;
}

inline void testPidReconnectAdvanceTimeSeconds(float delta_seconds)
{
    g_pid_reconnect_test_time_seconds += delta_seconds;
}

inline void testPidReconnectResetTimeSeconds()
{
    g_pid_reconnect_test_time_seconds = 0.0f;
}

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
        return g_pid_reconnect_test_time_seconds;
    }
};

#endif
