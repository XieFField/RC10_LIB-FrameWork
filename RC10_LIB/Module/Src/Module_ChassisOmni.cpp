#include "Module_ChassisOmni.h"

template <std::size_t WheelCount>
void Chassis_Omni<WheelCount>::inverseKinematics(const Robot_Twist& twist)
{
    if constexpr (WheelCount == 3)
    {
        
    }
    else if constexpr (WheelCount == 4)
    {

    }
    else
    {
        // 其他轮数的全向轮底盘暂不支持
        return;
    }
}


