#ifndef TEST_PID_RECONNECT_ARM_MATH_H
#define TEST_PID_RECONNECT_ARM_MATH_H

#include <cmath>
#include <cstdint>

typedef float float32_t;

typedef struct
{
    std::uint16_t numRows;
    std::uint16_t numCols;
    float *pData;
} arm_matrix_instance_f32;

static inline float arm_sin_f32(float value)
{
    return std::sin(value);
}

static inline float arm_cos_f32(float value)
{
    return std::cos(value);
}

static inline void arm_fill_f32(float value, float *p_dst, std::uint32_t block_size)
{
    for (std::uint32_t i = 0; i < block_size; ++i)
    {
        p_dst[i] = value;
    }
}

#endif
