/**
 * @file Module_ChassisOmni.h
 * @author XieFField
 * @brief ȫ�����ģ��
 * @version 1.0
 */
#ifndef __MODULE_CHASSISOMNI_H
#define __MODULE_CHASSISOMNI_H

/*

   ________                    _         ____                  _ 
  / ____/ /_  ____ ___________(_)____   / __ \____ ___  ____  (_)
 / /   / __ \/ __ `/ ___/ ___/ / ___/  / / / / __ `__ \/ __ \/ / 
/ /___/ / / / /_/ (__  |__  ) (__  )  / /_/ / / / / / / / / / /  
\____/_/ /_/\__,_/____/____/_/____/   \____/_/ /_/ /_/_/ /_/_/   
                                                                 

*/

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
#include "Motor_DJI.h"

#ifdef __cplusplus

/*
    ����ϵ��������ϵ�����ٶ���������ѭ���ֶ��򣬼���ʱ��Ϊ������

    ֻ����4/3��ȫ����̣�Ӧ�ò����õ�����������ȫ���ֵ��̰�
*/

#define COS_30 0.86602540378f
#define SIN_30 0.5f
#define COS_45 0.70710678118f
#define SIN_45 0.70710678118f

/*
���֣�   2 /    \ 3   ��Ӧ�ĵ��̵�����
            ___
             1

����:     2 /     \  3 ��Ӧ�ĵ��̵�����
                         
          1 \     / 4
*/

template <std::size_t WheelCount>
class Chassis_Omni : public Chassis_Base<WheelCount> {
public:
    Chassis_Omni(float wheel_radius, float max_wheel_rpm, float chassis_radius);
    void registerChassisMotor(int i, DJI_Motor* motors) {
	  if(i>=0  && i<4)
	  {
		wheel_[i] = motors;
	  }
	}
    void updateKinematics() override; // �����˶�ѧ��������������
    void inverseKinematics(const Robot_Twist& twist) override; // ��⣬����Ŀ���ٶȼ�������
    float chassis_radius_; // ���̰뾶 (m)
private:
    DJI_Motor* wheel_[4] = {nullptr};
};



#endif // __cplusplus

#endif // __MODULE_OMNICHASSIS_H
