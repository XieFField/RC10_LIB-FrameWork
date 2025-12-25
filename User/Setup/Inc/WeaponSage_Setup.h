/**
 * @file WeaponSage_Setup.h
 * @author XieFField
 * @brief ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¦Ó¦ï¿½Ã²ï¿½
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

        float debug_start = 1; //ï¿½ï¿½ï¿½Ô¿ï¿½Ê¼ï¿½ï¿½Ö¾ == 1 ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½

        float calibrate_startTime = 0;
        bool calibrate_start = false;
        bool is_calibrating = false;
    }ctrl_status_S;

    typedef enum{
<<<<<<< Updated upstream
        //ï¿½ï¿½ï¿½Ô¶ï¿½ï¿½ï¿½ï¿½Ìµï¿½Ã¿ï¿½ï¿½×´Ì¬Ã¶ï¿½ï¿½
        STATE_AIM_POSITION, //ï¿½ï¿½×¼Î»ï¿½ï¿½
        STATE_LOWER_CLAW,  //ï¿½Â½ï¿½×¦ï¿½ï¿½
        STATE_GRAB_CLAW,   //×¥È¡×¦ï¿½ï¿½
        STATE_LIFT_POSITION, //ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½
        STATE_DONE //ï¿½ï¿½ï¿½ï¿½
=======
        //½«×Ô¶¯¹ý³ÌµÄÃ¿¸ö×´Ì¬Ã¶¾Ù
        STATE_AIM_POSITION, //¶Ô×¼Î»ÖÃ
        STATE_LOWER_CLAW,  //ÏÂ½µ×¦×Ó
        STATE_GRAB_CLAW,   //×¥È¡×¦×Ó
        STATE_LIFT_POSITION, //ÌáÉýÎ»ÖÃ
        STATE_DONE 
>>>>>>> Stashed changes
    }auto_GRABstate_S;


    typedef struct{

         struct{
            // bool start
            bool is_matching = false;
            bool grab_start = false;
            float grab_startTime = 0.0f;
            bool is_moving = false;
<<<<<<< Updated upstream
        }auto_state_bool_S; //ï¿½Ö²ï¿½×´Ì¬ï¿½á¹¹ï¿½ï¿½
        float Pole_pos[4]={0.0f, 0.0f, 0.0f, 0.0f}; 
=======
        }auto_state_bool_S; 
>>>>>>> Stashed changes
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
		
		this->current_pos_= get_CurrentPos();
		
        Point2D pos = {0.0f, 0.0f, 0.0f};
		pos.x=current_pos_.traverse_pos_;
		pos.y=current_pos_.launch_pos_;
		pos.theta=current_pos_.claw_pos_;
		
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
<<<<<<< Updated upstream
    WeaponSage_Status_E weaponSage_status_ = WEAPONSAGE_IDLE;
	WeaponSage_Status_E last_weaponSage_status_ = WEAPONSAGE_IDLE;
	WeaponSage_Setup::auto_GRABstate_S now_state_;
	
	
	
=======
	WeaponSage_Setup::auto_GRABstate_S now_state_;
    
    WeaponSage_Status_E weaponSage_status_ = WEAPONSAGE_IDLE;
	WeaponSage_Status_E weaponSage_last_status_=WEAPONSAGE_IDLE;
    RmPocketData_t airjoy_data_; 

    WeaponSage_Setup::manual_ctrlForgrip_S manual_ctrlForgrip_;
>>>>>>> Stashed changes
};

extern WeaponSage_InitData_S initData_;

#endif

#endif // WEAPONSAGE_SETUP_H