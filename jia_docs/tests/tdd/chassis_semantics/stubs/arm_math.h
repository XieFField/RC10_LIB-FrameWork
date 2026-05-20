#ifndef TEST_TDD_ARM_MATH_H
#define TEST_TDD_ARM_MATH_H

#include <cmath>

typedef float float32_t;

static inline float arm_sin_f32(float value)
{
    return std::sin(value);
}

static inline float arm_cos_f32(float value)
{
    return std::cos(value);
}

#endif
