#include "APP_Utils.h"

#include <cmath>

namespace jia
{
    f32 wrapToPiRuntimeF32(f32 angle_rad)
    {
        while (angle_rad >= kPi)
        {
            angle_rad -= 2.0f * kPi;
        }
        while (angle_rad < -kPi)
        {
            angle_rad += 2.0f * kPi;
        }
        return angle_rad;
    }

    f32 wrapTo2PiRuntimeF32(f32 angle_rad)
    {
        while (angle_rad >= 2.0f * kPi)
        {
            angle_rad -= 2.0f * kPi;
        }
        while (angle_rad < 0.0f)
        {
            angle_rad += 2.0f * kPi;
        }
        return angle_rad;
    }

    f32 shortestAngularDistanceRuntimeF32(f32 from_rad, f32 to_rad)
    {
        return wrapToPiRuntimeF32(to_rad - from_rad);
    }

    f32 nearestEquivalentAngleRuntimeF32(f32 current_rad, f32 target_mod_rad)
    {
        return current_rad + shortestAngularDistanceRuntimeF32(current_rad, target_mod_rad);
    }

    f32 magnitude2DRuntimeF32(f32 x, f32 y)
    {
        return sqrtf(x * x + y * y);
    }

} // namespace jia
