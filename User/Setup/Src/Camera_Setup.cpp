#include "Camera_Setup.h"

#include <cmath>

#include "usart.h"

#ifndef CAMERA_FAKE
#define CAMERA_FAKE 0
#endif

Camera_Setup::Camera_Setup()
{
    camera_pid_x_.set_params(camera_x_pid_params, 0.0f);
    camera_pid_y_.set_params(camera_y_pid_params, 0.0f);
    camera_pid_yaw_.set_params(camera_yaw_pid_params, 10000.0f);
    camera_pid_yaw_.set_as_circular();
}

void Camera_Setup::SetCameraUart(UART_HandleTypeDef *uart)
{
    camera_uart_ = uart;
    camera_init_ = false;
}

void Camera_Setup::SetCameraLimit(float speed_max, float omega_max)
{
    speed_max_ = speed_max;
    omega_max_ = omega_max;
}

void Camera_Setup::SetCameraScale(float pos_scale, float yaw_scale)
{
    pos_scale_ = pos_scale;
    yaw_scale_ = yaw_scale;
}

void Camera_Setup::SetCameraXYRef(float x_ref, float y_ref)
{
    camera_x_ref_ = x_ref;
    camera_y_ref_ = y_ref;
}

void Camera_Setup::SetCameraYRef(float y_ref)
{
    camera_y_ref_ = y_ref;
}

float Camera_Setup::GetCameraYRef() const
{
    return camera_y_ref_;
}

void Camera_Setup::SetWeaponDone(bool done)
{
    weapon_done_ = done;
}

void Camera_Setup::SetZDone(bool done)
{
    z_done_ = done;
}

void Camera_Setup::SetDockDone(bool done)
{
    dock_done_ = done;
}

bool Camera_Setup::GetWeaponReq() const
{
    return weapon_req_;
}

bool Camera_Setup::GetZReq() const
{
    return z_req_;
}

float Camera_Setup::GetZRef() const
{
    return z_ref_;
}

void Camera_Setup::SetWeaponStart(bool is_start)
{
    weapon_cameraStart = is_start;
}

void Camera_Setup::ResetOnEnterCameraMode()
{
    z_sum_ = 0.0f;
    z_idx_ = 0;
    z_num_ = 0;
    cam_frame_seq_valid_ = false;
    yaw_sum_ = 0.0f;
    yaw_sample_count_ = 0;
    yaw_avg_err_ = 0.0f;
    for (uint8_t i = 0; i < 20; i++)
    {
        z_buf_[i] = 0.0f;
    }

    camera_pid_x_.set_params(camera_x_pid_params, 0.0f);
    camera_pid_y_.set_params(camera_y_pid_params, 0.0f);
    camera_pid_yaw_.set_params(camera_yaw_pid_params, 10000.0f);
    camera_pid_yaw_.set_as_circular();
}

void Camera_Setup::ResetOnExitCameraMode()
{
    weapon_req_ = false;
    z_req_ = false;
    z_ref_ = 0.0f;
    camera_state_ = CAMERA_WEAPON;
    z_rough_count_ = 0;
    x_count_ = 0;
    z_fine_count_ = 0;
    cam_frame_seq_valid_ = false;
    yaw_sum_ = 0.0f;
    yaw_sample_count_ = 0;
    yaw_avg_err_ = 0.0f;
    z_sum_ = 0.0f;
    z_idx_ = 0;
    z_num_ = 0;
    for (uint8_t i = 0; i < 20; i++)
    {
        z_buf_[i] = 0.0f;
    }
}

float Camera_Setup::clamp_value(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

float Camera_Setup::avg_z(float z_now)
{
    return z_now;
}

bool Camera_Setup::check_stable(float error, float limit, uint8_t &count)
{
    if (std::fabs(error) < limit)
    {
        count++;
    }
    else
    {
        count = 0;
    }

    return count >= 50;
}

void Camera_Setup::Control(Chassis &chassis, float yaw_deg)
{
    if (!weapon_cameraStart)
    {
        chassis.setTargetWorldSpeedLockNowRotZMode(0.0f, 0.0f);
        return;
    }

    if (camera_state_ == CAMERA_WEAPON && !weapon_req_ && !z_req_)
    {
        yaw_lock_ = yaw_deg;
    }

    cam_data_dbg_ = {0.0f, 0.0f, 0.0f, 0.0f};

#if CAMERA_FAKE
    cam_data_dbg_.x = fake_x;
    cam_data_dbg_.y = fake_y;
    cam_data_dbg_.z = fake_z;
    cam_data_dbg_.yaw = fake_yaw;
    bool cam_new_frame = true;
#else
    if (!camera_init_)
    {
        camera_ = Module_Camera::GetInstance(camera_uart_);
        camera_->InitUART();
        camera_init_ = true;
    }

    cam_data_dbg_ = camera_->GetCameraData();
    uint32_t cam_seq = camera_->GetFrameSeq();

    bool cam_new_frame = false;
    if (!cam_frame_seq_valid_ || cam_seq != cam_frame_seq_last_)
    {
        cam_new_frame = true;
        cam_frame_seq_last_ = cam_seq;
        cam_frame_seq_valid_ = true;
    }
#endif

    float x_err = cam_data_dbg_.x - camera_x_ref_;
    float y_err = cam_data_dbg_.y - camera_y_ref_;
    float z_err = avg_z(cam_data_dbg_.z);
    float yaw_err = cam_data_dbg_.yaw;

    bool x_new_sample = cam_new_frame;
    bool z_new_sample = cam_new_frame;
    bool yaw_new_sample = cam_new_frame;

    float vel_x = 0.0f;
    float vel_y = 0.0f;

    switch (camera_state_)
    {
    case CAMERA_WEAPON:
    {
        weapon_req_ = true;
        z_req_ = false;

        chassis.setTargetWorldSpeedLockToRotZMode(0.0f, 0.0f, yaw_lock_ * kDegToRad);

        if (weapon_done_)
        {
            weapon_req_ = false;
            camera_state_ = CAMERA_Z_ROUGH;
            z_done_ = false;
            z_rough_count_ = 0;
        }

        break;
    }

    case CAMERA_Z_ROUGH:
    {
        z_ref_ = 0.06f;
        z_req_ = true;

        if (z_done_ && z_new_sample)
        {
            z_rough_count_++;
            if (z_rough_count_ >= 50)
            {
                z_req_ = false;
                camera_state_ = CAMERA_X_ROUGH;
                x_count_ = 0;
            }
        }
        else if (!z_done_)
        {
            z_rough_count_ = 0;
        }

        chassis.setTargetWorldSpeedLockToRotZMode(0.0f, 0.0f, yaw_lock_ * kDegToRad);

        break;
    }

    case CAMERA_X_ROUGH:
    {
        vel_x = camera_pid_x_.pid_calc(0.0f, x_err) * pos_scale_;
        vel_x = clamp_value(vel_x, -speed_max_, speed_max_);
        vel_y = 0.0f;

        chassis.setTargetWorldSpeedLockToRotZMode(vel_x, vel_y, yaw_lock_ * kDegToRad);

        if (x_new_sample && check_stable(x_err, 0.002f, x_count_))
        {
            camera_state_ = CAMERA_Z_FINE;
            z_done_ = false;
            z_fine_count_ = 0;
        }

        break;
    }

    case CAMERA_Z_FINE:
    {
        z_ref_ = z_err;
        z_req_ = true;

        if (z_done_ && z_new_sample)
        {
            z_fine_count_++;
            if (z_fine_count_ >= 50)
            {
                z_req_ = false;
                camera_state_ = CAMERA_YAW;
                yaw_sum_ = 0.0f;
                yaw_sample_count_ = 0;
                yaw_avg_err_ = 0.0f;
            }
        }
        else if (!z_done_)
        {
            z_fine_count_ = 0;
        }

        chassis.setTargetWorldSpeedLockToRotZMode(0.0f, 0.0f, yaw_lock_ * kDegToRad);

        break;
    }

    case CAMERA_YAW:
    {
        if (yaw_sample_count_ < 50)
        {
            chassis.setTargetWorldSpeedLockToRotZMode(0.0f, 0.0f, yaw_lock_ * kDegToRad);

            if (yaw_new_sample)
            {
                yaw_sum_ += yaw_err;
                yaw_sample_count_++;

                if (yaw_sample_count_ >= 50)
                {
                    yaw_avg_err_ = yaw_sum_ / 50.0f;
                    yaw_lock_ = yaw_deg - yaw_avg_err_;
                }
            }
        }
        else
        {
            chassis.setTargetWorldSpeedLockToRotZMode(0.0f, 0.0f, yaw_lock_ * kDegToRad);

            float gyro_err = yaw_lock_ - yaw_deg;
            while (gyro_err > 180.0f)
            {
                gyro_err -= 360.0f;
            }
            while (gyro_err < -180.0f)
            {
                gyro_err += 360.0f;
            }

            if (std::fabs(gyro_err) < 0.5f)
            {
                camera_state_ = CAMERA_DOCK;
            }
        }

        break;
    }

    case CAMERA_DOCK:
    {
        vel_x = camera_pid_x_.pid_calc(0.0f, x_err) * pos_scale_;
        vel_y = camera_pid_y_.pid_calc(0.0f, y_err) * pos_scale_;

        vel_x = clamp_value(vel_x, -speed_max_, speed_max_);
        vel_y = clamp_value(vel_y, -speed_max_, speed_max_);

        chassis.setTargetBodySpeedLockNowRotZMode(vel_x, vel_y);

        if (dock_done_)
        {
            camera_state_ = CAMERA_DONE;
        }

        break;
    }

    case CAMERA_DONE:
    {
        chassis.setTargetBodySpeedLockNowRotZMode(0.0f, 0.0f);
        weapon_cameraStart = false;
        break;
    }

    default:
    {
        camera_state_ = CAMERA_WEAPON;
        weapon_req_ = false;
        z_req_ = false;
        weapon_cameraStart = false;
        chassis.setTargetWorldSpeedLockToRotZMode(0.0f, 0.0f, yaw_lock_ * kDegToRad);
        break;
    }
    }
}
