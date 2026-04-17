#ifndef __CAMERA_SETUP_H
#define __CAMERA_SETUP_H

#pragma once

#ifdef __cplusplus

extern "C"
{
#include "stm32h7xx_hal.h"
};

#include "Module_Camera.h"
#include "APP_PID.h"
#include "usart.h"
#include "chassis.h"

class Camera_Setup
{
public:
    Camera_Setup();

    void SetCameraUart(UART_HandleTypeDef *uart);
    void SetCameraLimit(float speed_max, float omega_max);
    void SetCameraScale(float pos_scale, float yaw_scale);
    void SetCameraXYRef(float x_ref, float y_ref);
    void SetCameraYRef(float y_ref);
    float GetCameraYRef() const;

    void SetWeaponDone(bool done);
    void SetZDone(bool done);
    void SetDockDone(bool done);

    bool GetWeaponReq() const;
    bool GetZReq() const;
    float GetZRef() const;

    void SetWeaponStart(bool is_start);

    void ResetOnEnterCameraMode();
    void ResetOnExitCameraMode();

    void Control(Chassis &chassis, float yaw_deg);

private:
    enum Camera_State_E
    {
        CAMERA_WEAPON,
        CAMERA_Z_ROUGH,
        CAMERA_X_ROUGH,
        CAMERA_Z_FINE,
        CAMERA_YAW,
        CAMERA_DOCK,
        CAMERA_DONE,
    };

    float clamp_value(float value, float low, float high);
    float avg_z(float z_now);
    bool check_stable(float error, float limit, uint8_t &count);

    static constexpr float kDegToRad = 0.017453292519943295f;

    Camera_State_E camera_state_ = CAMERA_WEAPON;

    Module_Camera *camera_ = nullptr;
    UART_HandleTypeDef *camera_uart_ = &huart6;

    bool weapon_cameraStart = false;
    bool camera_init_ = false;

    bool weapon_req_ = false;
    bool z_req_ = false;

    bool weapon_done_ = false;
    bool z_done_ = false;
    bool dock_done_ = false;

    float z_ref_ = 0.0f;

    float camera_x_ref_ = 0.0f;
    float camera_y_ref_ = 0.90f;

    float yaw_lock_ = 0.0f;

    float speed_max_ = 0.5f;
    float omega_max_ = 0.25f;

    float pos_scale_ = 1.0f;
    float yaw_scale_ = 1.0f;

    uint8_t z_rough_count_ = 0;
    uint8_t x_count_ = 0;
    uint8_t z_fine_count_ = 0;

    float yaw_sum_ = 0.0f;
    uint8_t yaw_sample_count_ = 0;
    float yaw_avg_err_ = 0.0f;

    uint32_t cam_frame_seq_last_ = 0;
    bool cam_frame_seq_valid_ = false;

    float z_buf_[20] = {0.0f};
    float z_sum_ = 0.0f;
    uint8_t z_idx_ = 0;
    uint8_t z_num_ = 0;

    Camera_Data_t cam_data_dbg_ = {0.0f, 0.0f, 0.0f, 0.0f};

#if CAMERA_FAKE
    float fake_x = 0.0f;
    float fake_y = 0.9f;
    float fake_z = 0.08f;
    float fake_yaw = 0.0f;
#endif

    PID_Position camera_pid_x_;
    PID_Position camera_pid_y_;
    PID_Position camera_pid_yaw_;
};

#endif // __cplusplus

#endif // __CAMERA_SETUP_H
