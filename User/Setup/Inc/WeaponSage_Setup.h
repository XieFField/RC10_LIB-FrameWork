/**
 * @file WeaponSage_Setup.h
 * @author XieFField
 * @brief ������ʦӦ�ò�
 */


#ifndef WEAPONSAGE_SETUP_H
#define WEAPONSAGE_SETUP_H

#pragma once

#ifdef __cplusplus
extern "C" {
    #include "arm_math.h"
}
#endif


#ifdef __cplusplus

#include "WeaponSage.h"
#include "FSMstauts_enum.h"
#include "BSP_RTOS.h"
#include "APP_debugTool.h"

namespace WeaponSage_Setup
{
    typedef struct{
        bool init_flag = false;

        float debug_start = 1; //���Կ�ʼ��־ == 1 ��ʼ����

        float calibrate_startTime = 0;
        bool calibrate_start = false;
        bool is_calibrating = false;
    }ctrl_status_S;

    typedef enum{
        //���Զ����̵�ÿ��״̬ö��
        STATE_AIM_POSITION, //��׼λ��
        STATE_LOWER_CLAW,  //�½�צ��
        STATE_GRAB_CLAW,   //ץȡצ��
        STATE_LIFT_POSITION, //����λ��
        STATE_DONE //����
    }auto_GRABstate_S;


    typedef struct{

        struct{
            // bool start
            bool is_matching = false;
            bool grab_start = false;
            float grab_startTime = 0.0f;
            bool is_moving = false;
        }auto_state_bool_S; //�ֲ�״̬�ṹ��
        float Pole_pos[4]={0.0f, 0.0f, 0.0f, 0.0f}; 
        float claw_close_pos = 32.36f;
        float claw_open_pos = 49.58f;
        float tarch_height = 0.0f; 
        float up_height = 0.0f;
        struct{
            bool aimposition_done = false;
            bool lowerclaw_done = false;
            bool grabclaw_done = false;
            bool lift_done = false;
        }flag;
        bool auto_ctrl1 = false;
        int pole_num = 0;
    }auto_ctrl_S;
}





class Robot_WeaponSage_Setup : public RtosTask, public Robot_WeaponSage {
public:
    Robot_WeaponSage_Setup(WeaponSage_InitData_S init_data);
    
    void init(M3508* launch_Motor, M2006* claw_Motor,
        M2006* traverse_Motor, DM_Motor* wrist_Motor)
    {
        this->register_launch_Motor(launch_Motor);
        this->register_claw_Motor(claw_Motor);
        this->register_traverse_Motor(traverse_Motor);
        this->register_wrist_Motor(wrist_Motor);

        if(this->launch_Motor_ == nullptr ||
           this->claw_Motor_ == nullptr ||
           this->traverse_Motor_ == nullptr ||
           this->wrist_Motor_ == nullptr
        )
        {
            ctrl_status_.init_flag = false;
            return;
        }

        start(osPriorityNormal, 256);

        ctrl_status_.init_flag = true;
    }

    void setLowerClawStart(bool start)
    {
        
    }

    Point2D getClawPos()
    {   
        Point2D pos = {0.0f, 0.0f, 0.0f};
        return pos;
    }
    void setWeaponSageStatus(WeaponSage_Status_E status)
    {
        weaponSage_status_ = status;
    }

protected:
    void loop() override;

private:
    WeaponSage_Setup::ctrl_status_S ctrl_status_;
    Debug_Printf debug_uart = Debug_Printf(&huart1);

    void manualControl();
    void idle();
    void stop();
    void debug();
    void autoControl();

    void calibrate();


    bool State_AimPosition(int pole_num);
    void State_LowerClaw();
    bool State_GrabClaw();
    void State_Lift();
	WeaponSage_Setup::auto_ctrl_S auto_ctrl_;
    WeaponSage_Status_E weaponSage_status_ = WEAPONSAGE_IDLE;
	WeaponSage_Setup::auto_GRABstate_S now_state_;
};

extern WeaponSage_InitData_S initData_;

#endif

#endif // WEAPONSAGE_SETUP_H