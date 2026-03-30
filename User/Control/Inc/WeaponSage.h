/**
 * @file WeaponSage.h
 * @author XieFField
 * @brief 锟斤拷锟斤拷锟斤拷师锟斤拷锟斤拷锟斤�?
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

//一锟斤拷转锟斤拷锟斤拷锟斤拷锟斤拷时锟斤拷为锟斤拷锟斤拷锟斤拷
typedef struct 
{
    /* data */
    float max_launchHeight_; // 锟斤拷锟斤拷锟斤拷锟斤拷谐蹋锟斤拷锟轿伙拷锟�?
    float max_clawAngle_; // 抓取锟斤拷锟角度ｏ拷锟斤拷位锟斤�?
    float max_traverseLength_; // 锟斤拷锟斤拷锟斤拷锟斤拷谐蹋锟斤拷锟轿伙拷锟�?

    float wrist_gearRatio_; // 锟斤拷锟斤拷锟斤拷俦龋锟斤拷锟斤拷锟斤拷锟阶蝗︼拷锟斤拷锟斤拷锟阶拷锟斤拷俣锟�(360锟斤拷锟斤拷味锟斤拷直锟斤拷)
    float launch_Ratio_; // 锟斤拷锟斤拷锟斤拷锟绞ｏ拷锟斤拷锟斤拷锟斤拷锟阶蝗︼拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�?
    float claw_gearRatio_; // 抓取锟斤拷锟�?比ｏ拷抓取锟斤拷锟阶蝗︼拷锟阶ト★拷锟斤拷俣锟�?
    float traverse_Ratio_; // 锟斤拷锟狡憋拷锟绞ｏ拷锟斤拷锟狡碉拷锟阶蝗︼拷锟斤拷锟斤拷�?锟斤拷锟斤拷锟�

    float max_wristMotorRPM_; // 锟斤拷锟斤拷锟斤拷锟斤拷锟阶拷伲锟斤拷锟轿籖PM

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
        float traverse_reversed_ = -1.0f;
        float wrist_reversed_ = 1.0f;
    }MotorReversed_S;

    enum WeaponSage_CtrlMode_S 
    {
        /* data */
        CURRENT_CONTROL, // 锟斤拷锟斤拷锟斤拷锟斤拷模式
        Join_POSITION_CONTROL, // 锟截斤拷位锟�?匡拷锟斤拷模�?
        TOTAL_ANGLE_CONTROL,   // 锟斤拷锟斤拷芙嵌瓤锟斤拷锟侥�?�?
        CAMERA_MIX_CONTROL, // ���ģʽ: launch �ٶȻ� + ����ؽ�λ�û�
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
    { 
        launch_Motor_ = motor; 
        if(launch_Motor_ != nullptr)
            return true; 
        else
            return false;
    }
    
    bool register_claw_Motor(M2006* motor)
    { 
        claw_Motor_ = motor; 
        if(claw_Motor_ != nullptr)
            return true; 
        else
            return false;
    }

    bool register_traverse_Motor(M2006* motor)
    { 
        traverse_Motor_ = motor; 
        if(traverse_Motor_ != nullptr)
            return true; 
        else
            return false;
    }

    bool register_wrist_Motor(DM_Motor* motor)
    { 
        wrist_Motor_ = motor; 
        if(wrist_Motor_ != nullptr)
            return true; 
        else
            return false;
    }

    void update();
    
    /**
     * @brief 锟斤拷锟�?碉拷锟斤拷锟斤拷锟�
     * @param reversed 锟�?�凤拷锟斤拷 true锟斤拷锟洁，false锟斤拷锟斤拷锟斤�?(默锟较�?�拷锟斤拷锟斤拷)
     * @param motor_type 锟斤拷锟斤拷锟斤拷锟�?
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

    M3508 *launch_Motor_ = nullptr; // 锟斤拷锟斤拷锟斤拷锟�?
    M2006 *claw_Motor_ = nullptr; // 抓取锟斤拷锟�?
    M2006 *traverse_Motor_ = nullptr; // 锟斤拷锟狡�?�拷锟�
    DM_Motor *wrist_Motor_ = nullptr; // 锟斤拷锟斤拷锟斤�?


    WeaponSage::WeaponSage_Pos_S target_pos_;
    WeaponSage::WeaponSage_Pos_S current_pos_;
	WeaponSage::WeaponSage_Pos_S last_pos_;

    /**
     * @brief 锟斤拷实位锟斤拷�?锟斤拷为锟斤拷锟斤拷芙嵌锟�
     * @param real_pos 锟斤拷实位锟�?ｏ拷锟斤拷位锟阶伙拷龋锟斤拷锟斤拷锟斤拷initData_说锟斤拷
     * @param motor_type 锟斤拷锟斤拷锟斤拷锟�?
     */
    float Realpos_to_MotorTotalAngle(float real_pos, WeaponSage::Motor_Type_E motor_type);

    float MotorTotalAngle_to_Realpos(float motor_angle, WeaponSage::Motor_Type_E motor_type);

    bool setMotorTargetTotalAngle(float total_angle, WeaponSage::Motor_Type_E motor_type);
		void set_launchMotorSpeed(float target)
		{
            launch_target_rpm_ = target;
		}
    float launch_target_rpm_ = 0.0f;
	WeaponSage_InitData_S initData_;
};


#endif // __cplusplus


#endif // WEAPONSAGE_H
