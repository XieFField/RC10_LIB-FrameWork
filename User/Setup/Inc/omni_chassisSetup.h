/**
 * @file omni_chassisSetup.h
 * @brief 全向底盘控制
 */
#ifndef __OMNI_CHASSISSETUP_H
#define __OMNI_CHASSISSETUP_H

#pragma once




#ifdef __cplusplus
#include "BSP_RTOS.h"   
#include "Module_ChassisOmni.h"
#include "Motor_Base.h"
#include "FSMstauts_enum.h"
#include "APP_debugTool.h"
#include "APP_CoordConvert.h"
#include "BSP_TimeStamp.h"
#include "APP_Speedplanner.h"
#include "debug_setup.h"
#include "APP_Bezier_Curve.h"
#include "APP_Path.h"
#include "APP_PID.h"
#include "Module_Position.h"

class OmniChassis_Setup:public RtosTask, public Chassis_Omni<4>{
public:
    OmniChassis_Setup(float wheel_radius, float max_wheel_rpm, float chassis_radius)
        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<4>(wheel_radius, max_wheel_rpm, chassis_radius),
          path_start_(16.0f,20.0f),
          path_control_(-1.0f,3.0f),
          path_end_(1.0f,10.0f),
          path_param_({1.0f,1.0f,1.5f,2.0f,0.001f,0.0f,0.0f,0.0f,0.0001f}),
          path_(path_start_, path_control_, path_end_, path_param_), // ????
          path_point_(path_start_),debug_uart(&huart2)
    {}
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
        pid_track.set_params(track_pid_params, 0.0f);
    }


private:
        void loop() override;
        bool init_flag = false;

        Robot_Twist last_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        
        bool is_path_completed_ = false;
        const float DIST_TO_END = 0.05f; // 5cm
        Vector2D speed = {0.0f, 0.0f};
        Vector2D planspeed = {0.0f, 0.0f};
        Vector2D baseVelocity = {0.0f, 0.0f}; // ??????????
        int num = 0;
        float tNearest = 0.0f;
        float tLookahead = 0.0f;
        float m_lookaheadDist=0.1f; //????
        float lateralError = 0.0f;
        float correctspeed = 0.0f;
        Vector2D nearestPt;
        Vector2D lookaheadPt;
        Vector2D lookaheadTangent; //????????????
        Vector2D path_start_;
        Vector2D path_control_;
        Vector2D path_end_;
        Vector2D path_point_;
        Vector2D corrVelocity = {0.0f, 0.0f}; // ????
        Speedplanner_1D_Param_Config path_param_;
        Path_Bezier path_; // ?????????
        PID_Position pid_track;
        CHASSIS_Status_E chassis_status_ = CHASSIS_STOP;
        Vector2D getRobotposition()
    {      
        Vector2D pos;
        pos.x=-RealPosData.world_x;
        pos.y=RealPosData.world_y;
        return pos;
    }
        Vector2D GetPathNearestPoint(const Vector2D& robotPos, float& tNearest);
        Vector2D FindLookaheadPoint(float tNearest, float& tLookahead);
        float CalculateLateralError(const Vector2D& robotPos, const Vector2D& nearestPt,float tLookahead);
};
#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H

//class OmniChassis_Setup:public RtosTask, public Chassis_Omni<4>{
//public:
//    OmniChassis_Setup(float wheel_radius, float max_wheel_rpm, float chassis_radius)
//        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<4>(wheel_radius, max_wheel_rpm, chassis_radius),
//          path_start_(16.0f,20.0f),
//          path_control_(-6.0f,16.0f),
//          path_end_(1.0f,10.0f),
//          path_param_({1.0f,1.0f,1.5f,2.0f,0.001f,0.0f,0.0f,0.0f,0.0001f}),
//          path_(path_start_, path_control_, path_end_, path_param_), // ????
//          path_point_(path_start_),debug_uart(&huart2)
//    {}
//    Debug_Printf debug_uart;
//    void setChassisStatus(CHASSIS_Status_E status)
//    {
//        chassis_status_ = status;
//    }

//    void init() 
//    {
//        if(this->wheels_[0] == nullptr ||this->wheels_[1] == nullptr ||
//           this->wheels_[2] == nullptr ||this->wheels_[3] == nullptr)
//            init_flag = false;
//        
//        this->start(osPriorityHigh, 256);
//        init_flag = true;
//        pid_track.set_params(track_pid_params, 0.0f);

//        path_start_ = getRobotposition();
//        path_point_ = path_start_;

//        //使用update更新路径
//        path_.update(path_start_, path_control_, path_end_);
//        path_.reset();
//    }


//private:
//        void loop() override;
//        bool init_flag = false;

//        Robot_Twist last_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
//        Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
//        
//        bool is_path_completed_ = false;
//        const float DIST_TO_END = 0.05f; // 5cm
//        Vector2D speed = {0.0f, 0.0f};
//        Vector2D planspeed = {0.0f, 0.0f};
//        Vector2D baseVelocity = {0.0f, 0.0f}; 
//        int num = 0;
//        float tNearest = 0.0f;
//        float tLookahead = 0.0f;
//        float m_lookaheadDist=0.1f; 
//        float lateralError = 0.0f;
//        float correctspeed = 0.0f;
//        Vector2D nearestPt;
//        Vector2D lookaheadPt;
//        Vector2D lookaheadTangent; 
//        Vector2D path_start_;
//        Vector2D path_control_;
//        Vector2D path_end_;
//        Vector2D path_point_;
//        Vector2D corrVelocity = {0.0f, 0.0f}; 
//        Speedplanner_1D_Param_Config path_param_;
//        Path_Bezier path_; 
//        PID_Position pid_track;
//        CHASSIS_Status_E chassis_status_ = CHASSIS_STOP;
//        Vector2D getRobotposition()
//    {      
//        Vector2D pos;
//        pos.x=-RealPosData.world_x;
//        pos.y=RealPosData.world_y;
//        return pos;
//    }
//        Vector2D GetPathNearestPoint(const Vector2D& robotPos, float& tNearest);
//        Vector2D FindLookaheadPoint(float tNearest, float& tLookahead);
//        float CalculateLateralError(const Vector2D& robotPos, const Vector2D& nearestPt,float tLookahead);
//};
//#endif // __cplusplus

//#endif // __OMNI_CHASSISSETUP_H
