/**
 * @file Module_ChassisOmni.h
 * @author XieFField
 * @brief 全向底盘模块
 * @version 1.0
 */
#ifndef __MODULE_CHASSISOMNI_H
#define __MODULE_CHASSISOMNI_H

#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "arm_math.h"
#include "cmsis_os.h"

#ifdef __cplusplus
}
#endif

#include "Module_ChassisBase.h"
#include "APP_tool.h"

#ifdef __cplusplus

/*
    坐标系采用右手系，角速度正方向遵循右手定则，即逆时针为正方向

    只包含4/3轮全向底盘，应该不会用到其他轮数的全向轮底盘吧
*/

template <std::size_t WheelCount>
class Chassis_Omni : public Chassis_Base<WheelCount> {
public:
    Chassis_Omni(float wheel_radius, float max_wheel_rpm);

    void updateKinematics() override; // 更新运动学，调用逆解和正解

    void inverseKinematics(const Robot_Twist& twist) override; // 逆解，根据目标速度计算轮速

    void forwardKinematics() override; // 正解，根据轮速计算机器人速度

private:
    
};



#endif // __cplusplus

#endif // __MODULE_OMNICHASSIS_H
