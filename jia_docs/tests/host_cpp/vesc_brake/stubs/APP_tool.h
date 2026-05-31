#ifndef TEST_APP_TOOL_H
#define TEST_APP_TOOL_H

#include <cmath>
#include <cstdint>

template <typename T>
inline T constrain(T value, T min_value, T max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

#endif
