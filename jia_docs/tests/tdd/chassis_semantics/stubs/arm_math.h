#ifndef TEST_TDD_ARM_MATH_H
#define TEST_TDD_ARM_MATH_H

#include <cmath>
#include <cstdint>

typedef float float32_t;
typedef int arm_status;
typedef struct
{
    uint16_t numRows;
    uint16_t numCols;
    float32_t *pData;
} arm_matrix_instance_f32;

static constexpr arm_status ARM_MATH_SUCCESS = 0;

static inline float arm_sin_f32(float value)
{
    return std::sin(value);
}

static inline float arm_cos_f32(float value)
{
    return std::cos(value);
}

static inline arm_status arm_sqrt_f32(float value, float32_t *result)
{
    if (result == nullptr)
    {
        return -1;
    }
    if (value < 0.0f)
    {
        *result = 0.0f;
        return -1;
    }
    *result = std::sqrt(value);
    return ARM_MATH_SUCCESS;
}

static inline void arm_fill_f32(float32_t value, float32_t *pDst, uint32_t blockSize)
{
    if (pDst == nullptr)
    {
        return;
    }
    for (uint32_t i = 0; i < blockSize; ++i)
    {
        pDst[i] = value;
    }
}

#endif
