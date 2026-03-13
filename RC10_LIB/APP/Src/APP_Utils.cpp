#include "RC10_LIB/APP/Inc/APP_Utils.h"

#include <cmath>

namespace jia
{

    f32 sinDegF32(f32 deg)
    {
        f32 sinf_result = sinf(deg * (kPi / 180.0f));

        if (sinf_result > 1.0f)
        {
            sinf_result = 1.0f;
        }
        else if (sinf_result < -1.0f)
        {
            sinf_result = -1.0f;
        }

        return sinf_result;
    }

    f32 cosDegF32(f32 deg)
    {
        f32 cosf_result = cosf(deg * (kPi / 180.0f));

        if (cosf_result > 1.0f)
        {
            cosf_result = 1.0f;
        }
        else if (cosf_result < -1.0f)
        {
            cosf_result = -1.0f;
        }

        return cosf_result;
    }

    f32 limit1DSignalRateByTimeF32(f32 target, f32 current, f32 dt, f32 maxRate)
    {
        f32 diff = target - current;
        f32 maxStep = maxRate * dt;
        if (diff > maxStep)
            return current + maxStep;
        else if (diff < -maxStep)
            return current - maxStep;
        else
            return target;
    }

} // namespace jia
