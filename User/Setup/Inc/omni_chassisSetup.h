/**
 * @file omni_chassisSetup.h
 * @brief ??????????????
 */
#ifndef __OMNI_CHASSISSETUP_H
#define __OMNI_CHASSISSETUP_H

// Force rebuild
#pragma once



#ifdef __cplusplus
#include "BSP_RTOS.h"   
#include "Module_ChassisOmni.h"
#include "Motor_Base.h"
#include "FSMstauts_enum.h"
#include "Module_Position.h"
#include "APP_PID.h"
#include "Motor_VESC.h"
#include "Module_Air_Joy.h"
#include "APP_debugTool.h"
//#include "AutoCtrler.h"
#include "APP_Vector2D.h"
#include "APP_Speedplanner.h"
#include "PathPlanner.h"
#include "APP_Bezier_Curve.h"
#include "APP_Path.h"
#include "APP_CoordConvert.h"
#include "PathTracing.h"
#include <vector>

extern AirJoy air_joy;

#define PI 3.14159265358979323846f

class OmniChassis_Setup:public RtosTask, public Chassis_Omni<3>{
public:
    OmniChassis_Setup(float wheel_radius, float max_wheel_rpm, float chassis_radius)
        : RtosTask("OmniChassis_Setup", 1), 
          Chassis_Omni<3>(wheel_radius, max_wheel_rpm, chassis_radius),
          path_planner_(map_data_, nodes_, open_list_, path_buffer_, 5, 6, 30, 30),
          path_tracer_(waypoints_, 30),debug_uart(&huart2)
    {
         initMap();
         
    }
    Debug_Printf debug_uart;
    void setChassisStatus(CHASSIS_Status_E status)
    {
        chassis_status_ = status;
    }

    void init() 
    {
        if(this->wheels_[0] == nullptr ||this->wheels_[1] == nullptr ||
        this->wheels_[2] == nullptr ||this->wheels_[3] == nullptr)
            init_flag = false;
        
        this->start(osPriorityHigh, 256);
        init_flag = true;
        this->initMap(); 
      //  pid_track.set_params(track_pid_params_, 0.0f);
        pid_yaw_.set_params(yaw_pid_params_, 0.0f);
    }
   

     
private:

	float locked_yaw = 0.0f;
    float now_yaw = 0.0f;
    float yaw_ctrl = 0.0f;
    Robot_Twist last_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        void loop() override;
        bool init_flag = false; 
     Position *position = nullptr;
     RealPos rp;
void chassis_manual_control_A();
void chassis_manual_control_B();
void chassis_stop();
void chassis_auto_control(float dt);
       //_uart = Debug_Printf(&huart2);
        CHASSIS_Status_E chassis_status_ = CHASSIS_STOP;

        PID_Position pid_yaw_;
        PID_Param_Config yaw_pid_params_ = {0.2f, 0.0f, 0.00006f, 1.0f, true, 1.0f, 0.05f};

        bool path_finished_ = false;

   
     float vx_body ;
    float vy_body;

    RealPos virtual_rp;
		bool virtual_rp_initialized = false;
		float world_vx;
float world_vy;				
				
				
    uint8_t map_data_[30];
    AStarNode nodes_[30];
    AStarNode* open_list_[30];
    GridPoint path_buffer_[30];
    PathPlanner path_planner_;
    Waypoint waypoints_[30];
    PathTracing path_tracer_;

    inline void initMap()
    {
        for (int i = 0; i < 30; ++i) map_data_[i] = CELL_FREE;

        for (int y = 1; y <= 4; ++y) {
            for (int x = 1; x <= 3; ++x) {
                int idx = y * 5 + x; 
                if (idx >= 0 && idx < 30) map_data_[idx] = CELL_OBSTACLE;
            }
        }

        map_data_[0] = CELL_FREE;
        map_data_[29] = CELL_FREE;

        
        path_planner_.setMapData(map_data_);
    }

public:
   

};
#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H