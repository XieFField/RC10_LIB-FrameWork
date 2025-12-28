/**
 * @file WeaponSage.h
 * @author XieFField
 * @brief ������ʦ������
 * @version 1.0
 */
#ifndef WEAPONSAGE_H
#define WEAPONSAGE_H

#pragma once

#ifdef __cplusplus
extern "C" {
    #include "arm_math.h"
}

#endif // __cplusplus

#ifdef __cplusplus

#include <iostream> 
#include <cstdint>
#include "Motor_DJI.h"
#include "Motor_DM.h"
#include "APP_tool.h"
#include "BSP_TimeStamp.h"

//һ��ת��������ʱ��Ϊ������
typedef struct 
{
    /* data */
    float max_launchHeight_; // ��������г̣���λ��
    float max_clawAngle_; // ץȡ���Ƕȣ���λ��
    float max_traverseLength_; // ��������г̣���λ��

    float wrist_gearRatio_; // ������ٱȣ�������תһȦ������ת���ٶ�(360����ζ��ֱ��)
    float launch_Ratio_; // �������ʣ��������תһȦ������������
    float claw_gearRatio_; // ץȡ���ٱȣ�ץȡ���תһȦ��ץȡ���ٶ�
    float traverse_Ratio_; // ���Ʊ��ʣ����Ƶ��תһȦ�����ƶ�����

    float max_wristMotorRPM_; // ���������ת�٣���λRPM

}WeaponSage_InitData_S;

namespace WeaponSage
{
    enum Motor_Type_E
    {
        Launch_Motor,
        Claw_Motor,
        Traverse_Motor,
        Wrist_Motor
    };

    typedef struct 
    {
        float launch_reversed_ = -1.0f;
        float claw_reversed_ = -1.0f;
        float traverse_reversed_ = 1.0f;
        float wrist_reversed_ = 1.0f;
    }MotorReversed_S;

    enum WeaponSage_CtrlMode_S 
    {
        /* data */
        CURRENT_CONTROL, // ��������ģʽ
        Join_POSITION_CONTROL, // �ؽ�λ�ÿ���ģʽ
        TOTAL_ANGLE_CONTROL,   // ����ܽǶȿ���ģʽ
    };
    
    typedef struct
    {
        float launch_pos_;
        float claw_pos_;
        float traverse_pos_;
        float wrist_pos_;

        float launch_TotalAngle_;
        float claw_TotalAngle_;
        float traverse_TotalAngle_;
        float wrist_TotalAngle_;
    }WeaponSage_Pos_S;

};

class Robot_WeaponSage {

public:
    Robot_WeaponSage(WeaponSage_InitData_S init_data);
    ~Robot_WeaponSage(){}

    bool register_launch_Motor(M3508* motor)
    { launch_Motor_ = motor; if(launch_Motor_ != nullptr)return true; }
    
    bool register_claw_Motor(M2006* motor)
    { claw_Motor_ = motor; if(claw_Motor_ != nullptr)return true; }

    bool register_traverse_Motor(M2006* motor)
    { traverse_Motor_ = motor; if(traverse_Motor_ != nullptr)return true; }

    bool register_wrist_Motor(DM_Motor* motor)
    { wrist_Motor_ = motor; if(wrist_Motor_ != nullptr)return true; }
    
    /**
     * @brief ���õ������
     * @param reversed �Ƿ��� true���࣬false������(Ĭ�ϲ�����)
     * @param motor_type �������
     */
	
	
    bool setMotorReversed(bool reversed, WeaponSage::Motor_Type_E motor_type);


    bool setTarget(float targetValue, WeaponSage::Motor_Type_E motor_type);

    void setCtrlMode(WeaponSage::WeaponSage_CtrlMode_S mode)
    {
        ctrl_mode_ = mode;
    }
	
	WeaponSage::WeaponSage_Pos_S get_CurrentPos()
	{
		WeaponSage::WeaponSage_Pos_S current_pos;
		current_pos.launch_pos_=MotorTotalAngle_to_Realpos(launch_Motor_->getTotalAngle(), WeaponSage::Launch_Motor);
		current_pos.traverse_pos_=MotorTotalAngle_to_Realpos(traverse_Motor_->getTotalAngle(),WeaponSage::Traverse_Motor);
		current_pos.claw_pos_=MotorTotalAngle_to_Realpos(claw_Motor_->getTotalAngle(),WeaponSage::Claw_Motor);
		current_pos.wrist_pos_=MotorTotalAngle_to_Realpos(wrist_Motor_->getTotalAngle(),WeaponSage::Wrist_Motor);
		return current_pos;
	}
	
	void Weapon_wrist_setzero(){wrist_Motor_->motorSetZero();}
	void Weapon_wrist_enable(){wrist_Motor_->motorEnable();}
private:

    WeaponSage::WeaponSage_CtrlMode_S ctrl_mode_ = WeaponSage::Join_POSITION_CONTROL;
    

    WeaponSage::MotorReversed_S motor_reversed_; 



protected:

    M3508 *launch_Motor_ = nullptr; // �������
    M2006 *claw_Motor_ = nullptr; // ץȡ���
    M2006 *traverse_Motor_ = nullptr; // ���Ƶ��
    DM_Motor *wrist_Motor_ = nullptr; // ������


    WeaponSage::WeaponSage_Pos_S target_pos_;
    WeaponSage::WeaponSage_Pos_S current_pos_;
	WeaponSage::WeaponSage_Pos_S last_pos_;

    /**
     * @brief ��ʵλ��ת��Ϊ����ܽǶ�
     * @param real_pos ��ʵλ�ã���λ�׻�ȣ������initData_˵��
     * @param motor_type �������
     */
    float Realpos_to_MotorTotalAngle(float real_pos, WeaponSage::Motor_Type_E motor_type);

    float MotorTotalAngle_to_Realpos(float motor_angle, WeaponSage::Motor_Type_E motor_type);

    bool setMotorTargetTotalAngle(float total_angle, WeaponSage::Motor_Type_E motor_type);
	WeaponSage_InitData_S initData_;
};


#endif // __cplusplus


#endif // WEAPONSAGE_H
