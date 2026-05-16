#ifndef TEST_ARM_MATH_H
#define TEST_ARM_MATH_H

#include <cmath>
#include <cstdint>

typedef float float32_t;

typedef struct
{
    uint16_t numRows;
    uint16_t numCols;
    float32_t *pData;
} arm_matrix_instance_f32;

static inline float32_t arm_sin_f32(float32_t value)
{
    return std::sin(value);
}

static inline float32_t arm_cos_f32(float32_t value)
{
    return std::cos(value);
}

static inline void arm_fill_f32(float32_t value, float32_t *dest, uint32_t blockSize)
{
    for (uint32_t i = 0; i < blockSize; ++i)
    {
        dest[i] = value;
    }
}

#endif
