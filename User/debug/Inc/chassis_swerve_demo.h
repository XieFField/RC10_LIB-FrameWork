#ifndef CHASSIS_SWERVE_DEMO_H
#define CHASSIS_SWERVE_DEMO_H

#pragma once

#ifdef __cplusplus

extern "C" {
    #include "stdint.h"
}

#endif


#ifdef __cplusplus

#include "Motor_DJI.h"
#include "Motor_VESC.h"
#include "APP_Tool.h"
#include "APP_Vector2D.h"
#include "gpio.h"
#include "BSP_TimeStamp.h"

#ifndef JIA_CHASSIS_HOMING_SEARCH_RPM
#define JIA_CHASSIS_HOMING_SEARCH_RPM 50.0f
#endif

namespace chassis_swerve_demo
{

typedef struct {
    float ramp__maxspeed_ = 0.0f;
    float max_accel_ = 0.0f;      // 启动加速度
    float max_decel_ = 0.0f;      // 刹车减速度
    float current_velocity_ = 0.0f;
    float ramp_target_ = 0.0f;
    float filter_k_ = 0.0f;
} Fliter_Ramp_S;


class Chassis_Swerve
{
public:
    Chassis_Swerve() {}
    ~Chassis_Swerve() {}

    void registerSteerMotor(M3508 *motor, uint8_t index)
    {
        if (index < 4) steer_motors[index] = motor;
    }

    void registerDriveMotor(VESC_Motor *motor, uint8_t index)
    {
        if (index < 4) drive_motors[index] = motor;
    }

    void init_swerve();
    void set_homing_start();
    void set_world_speed(float vx, float vy, float omega);
    void set_body_speed(float vx, float vy, float omega);
    void update_yaw(float yaw_rad);

    void set_max_linear_vel(float v)  { max_linear_vel_ = v; }
    void set_max_angular_vel(float w) { max_angular_vel_ = w; }


protected:
    void update();

private:
    M3508 *steer_motors[4] = {nullptr, nullptr, nullptr, nullptr};
    VESC_Motor *drive_motors[4] = {nullptr, nullptr, nullptr, nullptr};

    enum class HomingState : uint8_t {
        kIdle,
        kSearch,
        kEdgeDetected,
        kOffsetApply,
        kAlignToZero,
        kReady,
        kFault,
    };

    struct WheelConfig {
        float pos_x_m = 0.0f;
        float pos_y_m = 0.0f;
        float distance_from_center_m = 0.0f;
        float theta_rad = 0.0f;
        float tangent_theta_rad = 0.0f;
        float theta_oa_to_owi_rad = 0.0f;

        float steer_motor_sign = 1.0f;
        float drive_motor_sign = 1.0f;

        bool homing_enabled = true;
        bool homing_sensor_active_high = true;
        void *homing_gpio_port = nullptr;
        uint16_t homing_gpio_pin = 0;
        float homing_falling_edge_mech_rad = 0.0f;
        float homing_rising_edge_mech_rad = 0.0f;
        float homing_search_rpm = JIA_CHASSIS_HOMING_SEARCH_RPM;
        float homing_zero_offset_rad = 0.0f;
        float homing_timeout_s = 5.0f;

        HomingState homing_state = HomingState::kIdle;
        float homing_elapsed_s = 0.0f;
        bool homing_last_sensor_active = false;
        bool homing_last_edge_is_falling = false;
        float homing_runtime_zero_offset_rad = 0.0f;
        bool homing_zero_valid = false;

        float corrected_local_total_rad = 0.0f;
        float target_steer_oa_rad = 0.0f;
        float target_drive_m_s = 0.0f;
        float align_target_motor_deg_ = 0.0f;

        bool flipped = false;

        Fliter_Ramp_S steer_ramp;
        Fliter_Ramp_S drive_ramp;
    };

    float max_linear_vel_  = 2.0f;   // m/s
    float max_angular_vel_ = 3.14f;  // rad/s

    WheelConfig wheel_configs[4];

    Vector2D tangent_vector[4];
    Vector2D vel_vector[4];

    Vector2D world_speed;
    Vector2D body_speed;
    float omega_z = 0.0f;
    float chassis_yaw_rad_ = 0.0f;
    float chassis_yaw_deg_ = 0.0f;
    float drive_speed[4] = {0};
    float steer_angle[4] = {0};

    int8_t drive_motor_sign[4] = {1, 1, 1, 1};

    bool homing_request_ = true;
    bool all_homed_ = false;

    float zero_lock_delay_s_ = 0.5f;
    float zero_lock_timer_s_ = 0.0f;

    const float wheel_radius_m_ = 0.052f;
    const float vel_to_rpm = (1.0f / 0.052f) * (60.0f / (2.0f * PI));

    float dt_ = 0.0f;
    float last_time_s_ = 0.0f;
    bool time_initialized_ = false;

    static constexpr float kHomingAlignToleranceRad = 0.0349f;

    float get_dt();
    void kinematic_calc();
    void repostioning();
    void apply_ramp(float &current, float target, Fliter_Ramp_S &ramp, float dt);

    float rawTotalToCorrectedLocalRad(const WheelConfig &w, float motor_total_deg) const;
    float correctedLocalToRawTotalDeg(const WheelConfig &w, float corrected_local_rad) const;
    float localToOARad(const WheelConfig &w, float local_rad) const;
    float oaToLocalRad(const WheelConfig &w, float oa_rad) const;
    float normalizeAngle2PI(float angle_rad) const;
    float shortestAngularDistance(float from_rad, float to_rad) const;
    float nearestEquivalentAngle(float current_rad, float target_mod_rad) const;
};

} // namespace chassis_swerve_demo


#endif
#endif
