/**
 * @file omni_chassisSetup.h
 * @brief ï¿½ï¿½ï¿½ï¿½Ó¦ï¿½ï¿½ï¿½ï¿½
 */
#ifndef __OMNI_CHASSISSETUP_H
#define __OMNI_CHASSISSETUP_H

#pragma once

#ifdef __cplusplus

extern "C"
{
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
};

#include "BSP_RTOS.h"
#include "Module_ChassisOmni.h"
#include "Motor_Base.h"
#include "FSMstauts_enum.h"
#include "Module_CrsfReceiver.h"
#include "APP_debugTool.h"
#include "usart.h"
#include "Module_Position.h"
#include "APP_PID.h"
#include "Locate_Setup.h"
#include "BSP_USB_UART_Driver.h"
#include "usb_device.h"
#include "RTOS_QueueSetup.h"
#include "APP_Path.h"
#include "APP_Speedplanner.h"
#include "APP_Bezier_Curve.h"
#include "AutoCtrler.h"
#include "Module_Camera.h"
#define debug_ladar 0
class OmniChassis_Setup : public RtosTask, public Chassis_Omni<3>
{
public:
    OmniChassis_Setup(float wheel_radius, float max_wheel_rpm, float base_length, float side_length, bool three_wheel)
        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<3>(wheel_radius, max_wheel_rpm, base_length, side_length, three_wheel)
        ,debug_uart(&huart8)
    {
        yaw_pid_.set_as_circular();
    }


    OmniChassis_Setup(Chassis_Omni<3>::init_config& config)
        : RtosTask("OmniChassis_Setup", 1), Chassis_Omni<3>(config)
        ,debug_uart(&huart8)
    {
        yaw_pid_.set_as_circular();
    }

    void setChassisStatus(CHASSIS_Status_E status)
    {
        chassis_status_ = status;
    }

    void init()
    {
        if (this->wheels_[0] == nullptr || this->wheels_[1] == nullptr ||
            this->wheels_[2] == nullptr || this->wheels_[3] == nullptr)
            init_flag = false;

        yaw_pid_.set_params(lock_angle_pid_params, 10000.0f);

        this->setThreeWheelSolver(true);

    #if debug_ladar
            this->setThreeWheelSolver(false);
    #endif
        pid_track.set_params(track_pid_params, 0.0f);

        // ï¿½Ó¾ï¿½PIDï¿½ï¿½Ê¼ï¿½ï¿½
        pid_vision_x.set_params(track_pid_params, 0.0f);
        pid_vision_y.set_params(track_pid_params, 0.0f);
        pid_vision_yaw.set_params(lock_angle_pid_params, 1000.0f);

        this->start(osPriorityHigh, 1024);
//        setTargetKFS(3);
        init_flag = true;
    }

    void setChassisReverse(bool isReverse)
    {
        if (!isReverse)
            this->is_chassis_reverse_ = 1.0f;
        else
            this->is_chassis_reverse_ = -1.0f;
    }

    /**
     * @brief ÉèÖÃÂ·¾¶×Ô¶¯¿ªÊ¼±êÖ¾
     * @param start 1±íÊ¾¿ªÊ¼£¬0±íÊ¾Í£Ö¹
     * @param path_flagIndex Â·¾¶±êÖ¾Ë÷Òý£¬0»ò1
     */
    void setPathAutoStart(uint8_t start)
    {
        if(start == 1)
            flag = 1;
        else
            flag = 0;
        
        if(start == 0)
        {
            flag_run = 0;
        }
    }


    void setTargetKFS(int targetKFS)
    {
        KFS = targetKFS;
    }
	
	bool GetReach_flag()
	{
		return WeaponSage_END; 
	}
	
private:
    // Vision Staged Locking variables
    int vision_lock_state_ = 0; // 0: Coarse Lateral, 1: Yaw Lock, 2: Fine Lateral (IMU)
    float lock_imu_yaw_target_ = 0.0f;

    int WeaponSage_Start=0;
	bool WeaponSage_END=0;

    int flag = 0;
    int flag_run = 0;

    CHASSIS_Status_E chassis_status_ = CHASSIS_STOP;
    
    float yaw=0.0f;
    float target_yaw_ = 0.0f;
    //float Acc_target_yaw_ = 0.0f;

    //ConstantAcc Acc_yaw_{0.1f,0.0f}; // ×¢Òâ´úÂëÔËÐÐÏµÍ³µÄÖÜÆÚ
    //Path path_;
    Path_line path_line_;
    //Path_line path_line1_;
    
    Speedplanner_1D_Param_Config path_param_={.maxAcc = 30.0f, .maxDec = 40.0f, .maxJerk = 100.0f, .maxSpeed = 0.6f, .initialSpeed = 0.3f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f}; 
    Speedplanner_1D_Param_Config path_param_1={.maxAcc = 4.0f, .maxDec = 4.0f, .maxJerk = 0.0f, .maxSpeed = 0.5f, .initialSpeed = 0.3f, .finalSpeed = 0.0f, .startPos = 0.0f, .targetPos = 0.0f, .deadzone = 0.0001f}; 
 
    Vector2D original_point_={-0.48f,-0.50f};
    Vector2D Clamping_Bar_Selection_pos_ = {1.915f, 0.205f};
    
    Point3D ladar_data_;
    Vector2D robot_pos_ = {0.0f, 0.0f};
    
    Robot_Twist last_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    Robot_Twist target_chassis_twist_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; 
    Vector2D planspeed;
    Vector2D speed;    
    
    int8_t point_map=0;
    int8_t path_point_[20];
    int8_t path_key_point_[10];
    int8_t KFS=0;
    
    uint8_t yaw_pid_period_ = 3;
    uint8_t yaw_pid_period_count_ = 0;
    PID_Position yaw_pid_;
    
    void loop() override;
    bool init_flag = false;

    // Robot_Twist chassis_maxSpeed_ = {0};
    const float LINESPEED_LIMIT = 10 / 500.f; // ï¿½ï¿½ï¿½Ù¶ï¿½ï¿½ï¿½ï¿½ï¿½
    const float YAWSPEED_LIMIT = 1 / 500.f;   // yawï¿½Ù¶ï¿½ï¿½ï¿½ï¿½ï¿½

    float is_chassis_reverse_ = 1.0f;
    
    RmPocketData_t airjoy_data_; // Ò£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý£ï¿½ï¿½ï¿½Î§ -1 ~ 1

    Debug_Printf debug_uart = Debug_Printf(&huart8); // ï¿½ï¿½ï¿½Ô´ï¿½ï¿½ï¿½
    
     MF_AutoCtrler::PathNode_S KFS_result_ = {0, 0, 0, 0, 0, 26};
    /**
     * @brief ï¿½ï¿½È¡Â·ï¿½ï¿½ï¿½Ï¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Äµï¿½
     * @param path_ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ß¶ï¿½ï¿½ï¿½
     * @param robotPos ï¿½ï¿½ï¿½ï¿½ï¿½Ëµï¿½Ç°Î»ï¿½ï¿½
     * @param tNearest ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½tÖµ
     * @return Vector2D ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
     */
    Vector2D GetPathNearestPoint(BezierCurve &path_, const Vector2D &robotPos, float &tNearest);

    /**
     * @brief Ñ°ï¿½ï¿½Ç°ï¿½Óµï¿½
     * @param path_ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ß¶ï¿½ï¿½ï¿½
     * @param tNearest ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½tÖµ
     * @param tLookahead ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ç°ï¿½Óµï¿½ï¿½tÖµ
     * @return Vector2D Ç°ï¿½Óµï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
     */
    Vector2D FindLookaheadPoint(BezierCurve &path_, float tNearest, float &tLookahead);

    /**
     * @brief ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
     * @param path_ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ß¶ï¿½ï¿½ï¿½
     * @param robotPos ï¿½ï¿½ï¿½ï¿½ï¿½Ëµï¿½Ç°Î»ï¿½ï¿½
     * @param nearestPt ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
     * @param tLookahead Ç°ï¿½Óµï¿½ï¿½tÖµ
     * @return float ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö? (ï¿½ï¿½ï¿½ï¿½ï¿½Å£ï¿½ï¿½ï¿½Ê¾Æ«ï¿½ï¿½ï¿½Æ?¿½ï¿?)
     */
    float CalculateLateralError(BezierCurve &path_, const Vector2D &robotPos, const Vector2D &nearestPt, float tLookahead);
    
    void KFS_Selection_Planning(void);
    
    void Path_correction(void);
    
    void Clamping_Bar_Selection_Planning(void);
        
    int num = 0;
    float tNearest = 0.0f;                // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ú±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÏµÄ²ï¿½ï¿½ï¿½t (0~1)
    float tLookahead = 0.0f;              // Ç°ï¿½Óµï¿½ï¿½Ú±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÏµÄ²ï¿½ï¿½ï¿½t (0~1)
    float m_lookaheadDist = 0.4f;         // Ç°ï¿½Ó¾ï¿½ï¿½ï¿½ (ï¿½ï¿½Î»: ï¿½ï¿½)
    float lateralError = 0.0f;            // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿? (ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Æ«ï¿½ï¿½Â·ï¿½ï¿½ï¿½Ä¾ï¿½ï¿½ï¿½)
    float correctspeed = 0.0f;            // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Äºï¿½ï¿½ï¿½ï¿½Æ«ï¿½Ù¶È´ï¿½Ð¡
    Vector2D nearestPt;                   // Â·ï¿½ï¿½ï¿½Ï¾ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Äµï¿½
    Vector2D lookaheadPt;                 // Â·ï¿½ï¿½ï¿½Ïµï¿½Ç°ï¿½Óµï¿½
    Vector2D lookaheadTangent;            // Ç°ï¿½Óµã´¦ï¿½ï¿½ï¿½ï¿½ï¿½ß·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    Vector2D pathEnd;                     // Â·ï¿½ï¿½ï¿½Õµï¿½ï¿½ï¿½ï¿½ï¿½
    Vector2D corrVelocity = {0.0f, 0.0f}; // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Äºï¿½ï¿½ï¿½ï¿½Æ«ï¿½Ù¶ï¿½ï¿½ï¿½ï¿½ï¿½
    PID_Position pid_track;               // Ñ­ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½PIDï¿½ï¿½ï¿½ï¿½ï¿½ï¿½

    // ï¿½Ó¾ï¿½PID
    PID_Position pid_vision_x;
    PID_Position pid_vision_y;
    PID_Position pid_vision_yaw;
    Point3D vision_data_ = {0.0f, 0.0f, 0.0f}; // ï¿½ï¿½ï¿½Ú½ï¿½ï¿½Õµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?

public:
    void UpdateVisionData(Point3D data) { vision_data_ = data; } // ï¿½ï¿½ï¿½ï¿½ï¿½Ú»Øµï¿½ï¿½ï¿½ï¿½ï¿½
};
#endif // __cplusplus

#endif // __OMNI_CHASSISSETUP_H