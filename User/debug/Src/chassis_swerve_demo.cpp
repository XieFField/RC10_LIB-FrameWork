#include "chassis_swerve_demo.h"
#include <cmath>

namespace chassis_swerve_demo
{

    void Chassis_Swerve::init_swerve()
    {
        for (int i = 0; i < 4; i++)
        {
            if (steer_motors[i] == nullptr || drive_motors[i] == nullptr)
            {
                Error_Handler();
            }
        }

        static const struct WheelInit
        {
            float pos_x, pos_y;
            float steer_sign, drive_sign;
            float theta_oa_to_owi_deg;
            void *gpio_port;
            uint16_t gpio_pin;
            float fall_mech_deg, rise_mech_deg;
            float search_rpm, zero_off_deg;
        } kInit[4] = {
            {-0.39f, 0.40f, 1.0f, -1.0f, -90.0f, kPHOTOGATE_1_GPIO_Port, kPHOTOGATE_1_Pin, -30.0f, 150.0f, 10.0f, -30.0f},
            {-0.39f, -0.40f, 1.0f, -1.0f, 0.0f, kPHOTOGATE_2_GPIO_Port, kPHOTOGATE_2_Pin, 60.0f, -120.0f, 10.0f, -30.0f},
            {0.39f, -0.40f, 1.0f, -1.0f, 90.0f, kPHOTOGATE_3_GPIO_Port, kPHOTOGATE_3_Pin, 150.0f, -30.0f, 10.0f, -30.0f},
            {0.39f, 0.40f, 1.0f, 1.0f, 180.0f, kPHOTOGATE_4_GPIO_Port, kPHOTOGATE_4_Pin, -120.0f, 60.0f, 10.0f, -30.0f},
        };

        for (uint8_t i = 0; i < 4; ++i)
        {
            WheelConfig &w = wheel_configs[i];
            const WheelInit &init = kInit[i];

            w.pos_x_m = init.pos_x;
            w.pos_y_m = init.pos_y;
            w.distance_from_center_m = sqrtf(init.pos_x * init.pos_x + init.pos_y * init.pos_y);
            w.theta_rad = atan2f(init.pos_y, init.pos_x);
            if (w.theta_rad < 0.0f)
                w.theta_rad += 2.0f * PI;
            w.tangent_theta_rad = w.theta_rad + PI / 2.0f;
            tangent_vector[i] = Vector2D(cosf(w.tangent_theta_rad), sinf(w.tangent_theta_rad));

            w.theta_oa_to_owi_rad = init.theta_oa_to_owi_deg * PI / 180.0f;
            w.steer_motor_sign = init.steer_sign;
            w.drive_motor_sign = init.drive_sign;

            w.homing_enabled = true;
            w.homing_sensor_active_high = true;
            w.homing_gpio_port = init.gpio_port;
            w.homing_gpio_pin = init.gpio_pin;
            w.homing_falling_edge_mech_rad = init.fall_mech_deg * PI / 180.0f;
            w.homing_rising_edge_mech_rad = init.rise_mech_deg * PI / 180.0f;
            w.homing_search_rpm = init.search_rpm;
            w.homing_zero_offset_rad = init.zero_off_deg * PI / 180.0f;
            w.homing_timeout_s = 5.0f;

            w.homing_state = HomingState::kIdle;
            w.homing_elapsed_s = 0.0f;
            w.homing_last_sensor_active = false;
            w.homing_last_edge_is_falling = false;
            w.homing_runtime_zero_offset_rad = w.homing_zero_offset_rad;
            w.homing_zero_valid = false;

            w.corrected_local_total_rad = 0.0f;
            w.target_steer_oa_rad = 0.0f;
            w.target_drive_m_s = 0.0f;
            w.flipped = false;

            w.steer_ramp.ramp__maxspeed_ = 80000.0f;
            w.steer_ramp.max_accel_ = 160000.0f;
            w.steer_ramp.current_velocity_ = 0.0f;
            w.steer_ramp.filter_k_ = 200.0f;

            w.drive_ramp.ramp__maxspeed_ = 0.1f;
            w.drive_ramp.max_accel_ = 0.1f;
            w.drive_ramp.max_decel_ = 0.1f;
            w.drive_ramp.current_velocity_ = 0.0f;
            w.drive_ramp.filter_k_ = 0.5f;
        }

        all_homed_ = false;
        homing_request_ = true;
        zero_lock_timer_s_ = 0.0f;
        time_initialized_ = false;
    }

    void Chassis_Swerve::set_homing_start()
    {
        homing_request_ = true;
        for (uint8_t i = 0; i < 4; ++i)
        {
            if (wheel_configs[i].homing_enabled)
            {
                wheel_configs[i].homing_state = HomingState::kIdle;
                wheel_configs[i].homing_elapsed_s = 0.0f;
            }
        }
        all_homed_ = false;
    }

    float Chassis_Swerve::get_dt()
    {
        float now = TimeStamp::getInstance().getSeconds();
        if (!time_initialized_)
        {
            last_time_s_ = now;
            time_initialized_ = true;
            return 0.001f;
        }
        float dt = now - last_time_s_;
        last_time_s_ = now;
        if (dt <= 0.0f || dt > 0.1f)
            dt = 0.001f;
        return dt;
    }

    // ============================================================
    // 工具：角度转换
    // ============================================================

    // 电机 raw total (deg) → 带零偏的 LOCAL 连续角度 (rad)，与 chassis 的 corrected_steer_motor_total_angle 一致
    float Chassis_Swerve::rawTotalToCorrectedLocalRad(const WheelConfig &w, float motor_total_deg) const
    {
        float raw_rad = motor_total_deg * PI / 180.0f;
        return raw_rad + w.homing_runtime_zero_offset_rad;
    }

    // 带零偏的 LOCAL 连续角度 (rad) → 电机 raw total (deg)
    float Chassis_Swerve::correctedLocalToRawTotalDeg(const WheelConfig &w, float corrected_local_rad) const
    {
        float raw_rad = corrected_local_rad - w.homing_runtime_zero_offset_rad;
        return raw_rad * 180.0f / PI;
    }

    // LOCAL → OA
    float Chassis_Swerve::localToOARad(const WheelConfig &w, float local_rad) const
    {
        return local_rad + w.theta_oa_to_owi_rad;
    }

    // OA → LOCAL
    float Chassis_Swerve::oaToLocalRad(const WheelConfig &w, float oa_rad) const
    {
        return oa_rad - w.theta_oa_to_owi_rad;
    }

    // 归一化角度到 [0, 2PI)
    float Chassis_Swerve::normalizeAngle2PI(float angle_rad) const
    {
        float a = fmodf(angle_rad, 2.0f * PI);
        if (a < 0.0f)
            a += 2.0f * PI;
        return a;
    }

    float Chassis_Swerve::shortestAngularDistance(float from_rad, float to_rad) const
    {
        float diff = to_rad - from_rad;
        while (diff > PI)
            diff -= 2.0f * PI;
        while (diff <= -PI)
            diff += 2.0f * PI;
        return diff;
    }

    // 找到 target_mod_rad 的等效连续值中与 current_rad 最近的那个
    float Chassis_Swerve::nearestEquivalentAngle(float current_rad, float target_mod_rad) const
    {
        float diff = target_mod_rad - current_rad;
        float k = roundf(diff / (2.0f * PI));
        return target_mod_rad - k * 2.0f * PI;
    }

    // ============================================================
    // S 形 ramp
    // ============================================================
    void Chassis_Swerve::apply_ramp(float &current, float target, Fliter_Ramp_S &ramp, float dt)
    {
        float diff = target - current;
        float target_vel = diff * ramp.filter_k_;

        if (target_vel > ramp.ramp__maxspeed_)
            target_vel = ramp.ramp__maxspeed_;
        if (target_vel < -ramp.ramp__maxspeed_)
            target_vel = -ramp.ramp__maxspeed_;

        float max_dv = ramp.max_accel_ * dt;
        if (target_vel > ramp.current_velocity_ + max_dv)
            ramp.current_velocity_ += max_dv;
        else if (target_vel < ramp.current_velocity_ - max_dv)
            ramp.current_velocity_ -= max_dv;
        else
            ramp.current_velocity_ = target_vel;

        float step = ramp.current_velocity_ * dt;

        if (fabsf(diff) < 0.01f && fabsf(ramp.current_velocity_) < 0.1f)
        {
            ramp.current_velocity_ = 0.0f;
            current = target;
            return;
        }

        current += step;
    }

    // ============================================================
    // 速度设置
    // ============================================================
    void Chassis_Swerve::set_world_speed(float vx, float vy, float omega)
    {
        world_speed = Vector2D(vx, vy);
        omega_z = omega;
        float c = cosf(chassis_yaw_rad_);
        float s = sinf(chassis_yaw_rad_);
        float bx = vx * c + vy * s;
        float by = -vx * s + vy * c;
        body_speed = Vector2D(bx, by);
    }

    void Chassis_Swerve::set_body_speed(float vx, float vy, float omega)
    {
        body_speed = Vector2D(vx, vy);
        omega_z = omega;
        float c = cosf(chassis_yaw_rad_);
        float s = sinf(chassis_yaw_rad_);
        float wx = vx * c - vy * s;
        float wy = vx * s + vy * c;
        world_speed = Vector2D(wx, wy);
    }

    void Chassis_Swerve::update_yaw(float yaw_deg)
    {
        chassis_yaw_deg_ = yaw_deg;
        chassis_yaw_rad_ = yaw_deg * PI / 180.0f;
    }

    // ============================================================
    // 运动学解算
    // ============================================================
    void Chassis_Swerve::kinematic_calc()
    {
        Vector2D v_body(body_speed.x, -body_speed.y);
        float mag = v_body.magnitude();
        if (mag > max_linear_vel_)
            v_body = v_body * (max_linear_vel_ / mag);
        if (fabsf(omega_z) > max_angular_vel_)
            omega_z = copysignf(max_angular_vel_, omega_z);

        bool is_zero_speed = (v_body.magnitude() <= 0.005f && fabsf(omega_z) <= 0.006f);
        if (is_zero_speed)
        {
            zero_lock_timer_s_ += dt_;
        }
        else
        {
            zero_lock_timer_s_ = 0.0f;
        }

        // 零速锁轮：时间门到期后锁到 X 姿态
        if (zero_lock_delay_s_ > 0.0f && zero_lock_timer_s_ >= zero_lock_delay_s_)
        {
            // static const float kXParkOA[4] = { 0.785398f, 2.356194f, 3.926991f, 5.497787f };
            static const float kXParkOA[4] = {5.497787f, 3.926991f, 2.356194f, 0.785398f};
            for (uint8_t i = 0; i < 4; ++i)
            {
                drive_speed[i] = 0.0f;
                steer_angle[i] = kXParkOA[i];
            }
            return;
        }

        if (is_zero_speed)
        {
            for (uint8_t i = 0; i < 4; ++i)
                drive_speed[i] = 0.0f;
            return;
        }

        // 运动学解算：Vector2D无.angle()，用atan2f手算
        for (uint8_t i = 0; i < 4; ++i)
        {
            float L = wheel_configs[i].distance_from_center_m;
            vel_vector[i] = tangent_vector[i] * (omega_z * L) + v_body;
            drive_speed[i] = vel_vector[i].magnitude();
            float ang = atan2f(vel_vector[i].y, vel_vector[i].x);
            if (ang < 0.0f)
                ang += 2.0f * PI;
            steer_angle[i] = ang;
        }

        // 翻转逻辑
        for (uint8_t i = 0; i < 4; ++i)
        {
            WheelConfig &w = wheel_configs[i];

            float cur_local = rawTotalToCorrectedLocalRad(
                w, steer_motors[i] ? steer_motors[i]->getTotalAngle() : 0.0f);
            float cur_oa_rad = normalizeAngle2PI(localToOARad(w, cur_local));

            float target_oa = steer_angle[i];
            float delta = target_oa - cur_oa_rad;
            if (delta > PI)
                delta -= 2.0f * PI;
            else if (delta < -PI)
                delta += 2.0f * PI;

            w.flipped = false;
            drive_motor_sign[i] = 1;

            if (fabsf(delta) > PI / 2.0f)
            {
                target_oa = (target_oa < PI) ? target_oa + PI : target_oa - PI;
                drive_motor_sign[i] = -1;
                w.flipped = true;
            }

            steer_angle[i] = target_oa;

            // 低速闸门：舵没到位不输出驱动
            float final_delta = shortestAngularDistance(cur_oa_rad, target_oa);
            if (drive_speed[i] < 0.15f && fabsf(final_delta) > 0.087266f)
            {
                drive_speed[i] = 0.0f;
            }
        }
    }

    // ============================================================
    // 回零状态机
    // ============================================================
    void Chassis_Swerve::repostioning()
    {
        for (uint8_t i = 0; i < 4; ++i)
        {
            WheelConfig &w = wheel_configs[i];
            if (!w.homing_enabled || w.homing_gpio_port == nullptr)
            {
                w.homing_state = HomingState::kReady;
                w.homing_zero_valid = true;
                continue;
            }

            bool sensor_raw = (HAL_GPIO_ReadPin((GPIO_TypeDef *)w.homing_gpio_port, w.homing_gpio_pin) == GPIO_PIN_SET);
            if (!w.homing_sensor_active_high)
                sensor_raw = !sensor_raw;

            float motor_total_deg = 0.0f;
            if (steer_motors[i] != nullptr)
                motor_total_deg = steer_motors[i]->getTotalAngle();
            float raw_total_rad = motor_total_deg * PI / 180.0f;

            switch (w.homing_state)
            {
            case HomingState::kIdle:
                if (homing_request_)
                {
                    w.homing_state = HomingState::kSearch;
                    w.homing_elapsed_s = 0.0f;
                }
                break;

            case HomingState::kSearch:
                w.homing_elapsed_s += dt_;
                if (sensor_raw != w.homing_last_sensor_active)
                {
                    bool is_falling = w.homing_last_sensor_active && !sensor_raw;
                    float edge_mech_oa_rad = is_falling ? w.homing_falling_edge_mech_rad
                                                        : w.homing_rising_edge_mech_rad;
                    // 对齐 chassis: edge_local = edge_mech_oa - theta_oa_to_owi
                    float edge_local_rad = edge_mech_oa_rad - w.theta_oa_to_owi_rad;
                    w.homing_runtime_zero_offset_rad = edge_local_rad + w.homing_zero_offset_rad - raw_total_rad;
                    w.homing_last_edge_is_falling = is_falling;
                    w.homing_zero_valid = true;
                    w.homing_state = HomingState::kEdgeDetected;
                }
                else if (w.homing_elapsed_s > w.homing_timeout_s)
                    w.homing_state = HomingState::kFault;

                break;

            case HomingState::kEdgeDetected:
                w.homing_state = HomingState::kOffsetApply;
                break;

            case HomingState::kOffsetApply:
                w.homing_state = HomingState::kAlignToZero;
                break;

            case HomingState::kAlignToZero:
            {
                // 对齐 chassis: current_local → current_oa → target_oa(0) → target_local
                float cur_local = rawTotalToCorrectedLocalRad(w, motor_total_deg);
                float cur_oa = localToOARad(w, cur_local);
                float tar_oa_continuous = nearestEquivalentAngle(cur_oa, 0.0f);
                float tar_local_continuous = oaToLocalRad(w, tar_oa_continuous);
                w.align_target_motor_deg_ = correctedLocalToRawTotalDeg(w, tar_local_continuous);

                float cur_oa_mod = normalizeAngle2PI(cur_oa);
                float err = fabsf(shortestAngularDistance(cur_oa_mod, 0.0f));

                if (err <= kHomingAlignToleranceRad)
                    w.homing_state = HomingState::kReady;

                break;
            }

            case HomingState::kReady:

            case HomingState::kFault:
                break;
            }

            w.homing_last_sensor_active = sensor_raw;
        }

        bool all_ready = true;
        for (uint8_t i = 0; i < 4; ++i)
        {
            if (wheel_configs[i].homing_state != HomingState::kReady)
            {
                all_ready = false;
                break;
            }
        }
        if (all_ready)
        {
            all_homed_ = true;
            homing_request_ = false;
        }
    }

    // ============================================================
    // 主更新
    // ============================================================
    void Chassis_Swerve::update()
    {
        dt_ = get_dt();

        // 1. 回零
        repostioning();

        // 2. 回零中各状态的电机关节
        for (uint8_t i = 0; i < 4; ++i)
        {
            WheelConfig &w = wheel_configs[i];
            if (steer_motors[i] == nullptr)
                continue;

            if (w.homing_state == HomingState::kSearch)
            {
                steer_motors[i]->setTargetRPM(w.homing_search_rpm);
                if (drive_motors[i])
                    drive_motors[i]->setTargetRPM(0.0f);
                continue;
            }
            if (w.homing_state == HomingState::kAlignToZero)
            {
                float deg = fmodf(w.align_target_motor_deg_, 360.0f);
                if (deg < 0.0f)
                    deg += 360.0f;
                steer_motors[i]->setTargetAngle(deg);
                if (drive_motors[i])
                    drive_motors[i]->setTargetRPM(0.0f);
                continue;
            }
            if (w.homing_state == HomingState::kFault ||
                w.homing_state == HomingState::kIdle ||
                w.homing_state == HomingState::kEdgeDetected ||
                w.homing_state == HomingState::kOffsetApply)
            {
                steer_motors[i]->setTargetCurrent(0.0f);
                if (drive_motors[i])
                    drive_motors[i]->setTargetRPM(0.0f);
                continue;
            }
        }

        if (!all_homed_)
            return;

        // 3. 运动学
        kinematic_calc();

        // 4. 下发
        for (uint8_t i = 0; i < 4; ++i)
        {
            WheelConfig &w = wheel_configs[i];

            if (steer_motors[i] != nullptr)
            {
                float local_rad = oaToLocalRad(w, steer_angle[i]);
                float motor_deg = correctedLocalToRawTotalDeg(w, local_rad);
                float deg = fmodf(motor_deg, 360.0f);
                if (deg < 0.0f)
                    deg += 360.0f;

                steer_motors[i]->setTargetAngle(deg);
            }

            if (drive_motors[i] != nullptr)
            {
                float ramped = drive_speed[i];
                apply_ramp(ramped, drive_speed[i], w.drive_ramp, dt_);
                float ramped_rpm = ramped * vel_to_rpm * (float)drive_motor_sign[i] * w.drive_motor_sign;
                drive_motors[i]->setTargetRPM(ramped_rpm);
            }
        }
    }

} // namespace chassis_swerve_demo
