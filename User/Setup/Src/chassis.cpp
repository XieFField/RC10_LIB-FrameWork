#include "chassis.h"

#include <cmath>

#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "BSP_TimeStamp.h"
#include "BSP_RtosTimeStampUs64.h"
#include "Module_CrsfReceiver.h"
#include "Module_HWT.h"
#include "APP_PID.h"

#include "APP_Utils.h"
#include "chassis.h"

namespace jia
{
    namespace FourSteerChassis
    {
        void Chassis::init(InitConfig &config)
        {
            runtime_strategy_cfg_ = default_strategy_cfg_;
            deriveNearZeroThresholds();
            refreshActuatorLimitState();

            static const WheelInitConfig kDefaultWheelInit[4] = {
                {.pos_x_m = -0.39f, .pos_y_m = 0.40f, .theta_oa_to_owi_deg = -90.0f, .steer_motor_sign = 1.0f, .drive_motor_sign = 1.0f, .homing_enabled = true, .homing_sensor_active_high = true, .homing_gpio_port = kPHOTOGATE_1_GPIO_Port, .homing_gpio_pin = kPHOTOGATE_1_Pin, .homing_falling_edge_mech_deg = -30.0f, .homing_rising_edge_mech_deg = 150.0f, .homing_search_rpm = 10.0f, .homing_zero_offset_deg = -30.0f, .homing_timeout_s = 5.0f},
                {.pos_x_m = -0.39f, .pos_y_m = -0.40f, .theta_oa_to_owi_deg = 0.0f, .steer_motor_sign = 1.0f, .drive_motor_sign = 1.0f, .homing_enabled = true, .homing_sensor_active_high = true, .homing_gpio_port = kPHOTOGATE_2_GPIO_Port, .homing_gpio_pin = kPHOTOGATE_2_Pin, .homing_falling_edge_mech_deg = 60.0f, .homing_rising_edge_mech_deg = -120.0f, .homing_search_rpm = 10.0f, .homing_zero_offset_deg = -30.0f, .homing_timeout_s = 5.0f},
                {.pos_x_m = 0.39f, .pos_y_m = -0.40f, .theta_oa_to_owi_deg = 90.0f, .steer_motor_sign = 1.0f, .drive_motor_sign = 1.0f, .homing_enabled = true, .homing_sensor_active_high = true, .homing_gpio_port = kPHOTOGATE_3_GPIO_Port, .homing_gpio_pin = kPHOTOGATE_3_Pin, .homing_falling_edge_mech_deg = 150.0f, .homing_rising_edge_mech_deg = -30.0f, .homing_search_rpm = 10.0f, .homing_zero_offset_deg = -30.0f, .homing_timeout_s = 5.0f},
                {.pos_x_m = 0.39f, .pos_y_m = 0.40f, .theta_oa_to_owi_deg = 180.0f, .steer_motor_sign = 1.0f, .drive_motor_sign = -1.0f, .homing_enabled = true, .homing_sensor_active_high = true, .homing_gpio_port = kPHOTOGATE_4_GPIO_Port, .homing_gpio_pin = kPHOTOGATE_4_Pin, .homing_falling_edge_mech_deg = -120.0f, .homing_rising_edge_mech_deg = 60.0f, .homing_search_rpm = 10.0f, .homing_zero_offset_deg = -30.0f, .homing_timeout_s = 5.0f},
            };

            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                const WheelInitConfig &wheel_init = kDefaultWheelInit[i];
                wheel.steer_motor_h = config.steer_motor_h[i];
                wheel.drive_motor_h = config.drive_motor_h[i];
                wheel.pos_x_m = wheel_init.pos_x_m;
                wheel.pos_y_m = wheel_init.pos_y_m;
                wheel.theta_oa_to_owi_rad = degToRadF32(wheel_init.theta_oa_to_owi_deg);
                wheel.steer_motor_sign = (wheel_init.steer_motor_sign == 0.0f) ? 1.0f : wheel_init.steer_motor_sign;
                wheel.drive_motor_sign = (wheel_init.drive_motor_sign == 0.0f) ? 1.0f : wheel_init.drive_motor_sign;
                wheel.homing_enabled = wheel_init.homing_enabled;
                wheel.homing_sensor_active_high = wheel_init.homing_sensor_active_high;
                wheel.homing_gpio_port = wheel_init.homing_gpio_port;
                wheel.homing_gpio_pin = wheel_init.homing_gpio_pin;
                wheel.homing_falling_edge_mech_rad = degToRadF32(wheel_init.homing_falling_edge_mech_deg);
                wheel.homing_rising_edge_mech_rad = degToRadF32(wheel_init.homing_rising_edge_mech_deg);
                wheel.homing_search_rpm = wheel_init.homing_search_rpm;
                wheel.homing_zero_offset_rad = degToRadF32(wheel_init.homing_zero_offset_deg);
                wheel.homing_timeout_s = wheel_init.homing_timeout_s;
                wheel.homing_state = wheel.homing_enabled ? HomingState::kIdle : HomingState::kReady;
                wheel.homing_last_sensor_active = false;
                wheel.homing_last_edge_is_falling = false;
                wheel.homing_zero_valid = !wheel.homing_enabled;
                wheel.homing_elapsed_s = 0.0f;
                wheel.homing_runtime_zero_offset_rad = wheel.homing_zero_offset_rad;
                wheel.corrected_steer_motor_total_angle_rad = 0.0f;
                wheel.corrected_drive_omega_rad_s = 0.0f;
                wheel.target_steer_motor_total_angle_rad = 0.0f;
                wheel.target_drive_omega_rad_s = 0.0f;
                wheel.steer_target_velocity_rad_s = 0.0f;
                wheel.flipped_drive_direction = false;
                selected_flipped_solution_[i] = false;
                drive_gate_scale_[i] = 1.0f;
            }

            adaptive_gate_scale_ = 1.0f;
            adaptive_gate_phase_ = AdaptiveGatePhase::kIdle;
            vector_gate_scale_ = 1.0f;
            vector_gate_active_ = false;
            vector_dir_err_deg_ = 0.0f;
            vector_eta_max_s_ = 0.0f;
            trans_dir_freeze_active_ = false;
            trans_dir_ref_valid_ = false;
            trans_dir_ref_rad_ = 0.0f;
            trans_dir_tar_mag_m_s_ = 0.0f;
            trans_dir_out_mag_m_s_ = 0.0f;
            trans_dir_freeze_reason_ = 0U;
            vector_gate_scale_ = 1.0f;
            vector_gate_active_ = false;
            vector_dir_err_deg_ = 0.0f;
            vector_eta_max_s_ = 0.0f;

            rot_z_pid_.set_params(lock_angle_pid_params, 0.0f);
            rot_z_pid_.set_as_circular();
            clearInputTargetData();
            startHoming();

            const osThreadAttr_t thread_attributes = {
                .name = "chassis_thread",
                .stack_size = 500 * 4,
                .priority = (osPriority_t)(osPriorityAboveNormal7),
            };

            osThreadId_t thread_handle = osThreadNew(this->createThread, this, &thread_attributes);
            if (thread_handle == NULL)
            {
                Error_Handler();
            }
        }

        void Chassis::createThread(void *arg)
        {
            Chassis *chassis = static_cast<Chassis *>(arg);
            chassis->runThread(NULL);
        }

        void Chassis::clearInputTargetData()
        {
            input_target_data_.mode = Mode::kWheelTorqueFreeMode;
            input_target_data_.vel_x = 0.0f;
            input_target_data_.vel_y = 0.0f;
            input_target_data_.omega_z = 0.0f;
            input_target_data_.rot_z = 0.0f;
            input_target_data_.steer_lock_angle_deg = 0.0f;
            input_target_data_.drive_lock_speed_m_s = 0.0f;
            input_target_data_.zero_current_all = false;
            lock_now_rot_z_target_ = 0.0f;
            trans_dir_freeze_active_ = false;
            trans_dir_ref_valid_ = false;
            trans_dir_ref_rad_ = 0.0f;
            trans_dir_tar_mag_m_s_ = 0.0f;
            trans_dir_out_mag_m_s_ = 0.0f;
            trans_dir_freeze_reason_ = 0U;
        }

        Chassis::Result Chassis::setWheelTorqueFreeMode()
        {
            clearInputTargetData();
            input_target_data_.mode = Mode::kWheelTorqueFreeMode;
            input_target_data_.zero_current_all = false;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetBodySpeedMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            input_target_data_.mode = Mode::kBodySpeedMode;
            input_target_data_.zero_current_all = false;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = omega_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetBodySpeedLockNowRotZMode(f32 vel_x, f32 vel_y)
        {
            input_target_data_.mode = Mode::kBodySpeedLockNowRotZMode;
            input_target_data_.zero_current_all = false;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = 0.0f;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetBodySpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            input_target_data_.mode = Mode::kBodySpeedLockNowRotZWithNoOmegaZMode;
            input_target_data_.zero_current_all = false;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = omega_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetBodySpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z)
        {
            input_target_data_.mode = Mode::kBodySpeedLockToRotZMode;
            input_target_data_.zero_current_all = false;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.rot_z = rot_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetWorldSpeedMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            input_target_data_.mode = Mode::kWorldSpeedMode;
            input_target_data_.zero_current_all = false;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = omega_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetWorldSpeedLockNowRotZMode(f32 vel_x, f32 vel_y)
        {
            input_target_data_.mode = Mode::kWorldSpeedLockNowRotZMode;
            input_target_data_.zero_current_all = false;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = 0.0f;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            input_target_data_.mode = Mode::kWorldSpeedLockNowRotZWithNoOmegaZMode;
            input_target_data_.zero_current_all = false;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = omega_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetWorldSpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z)
        {
            input_target_data_.mode = Mode::kWorldSpeedLockToRotZMode;
            input_target_data_.zero_current_all = false;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.rot_z = rot_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setSteerDegAndDriveSpeed(f32 steer_angle_deg, f32 chassis_speed_m_s)
        {
            input_target_data_.mode = Mode::kSteerAngleAndDriveSpeedMode;
            input_target_data_.zero_current_all = false;
            input_target_data_.steer_lock_angle_deg = steer_angle_deg;
            input_target_data_.drive_lock_speed_m_s = chassis_speed_m_s;
            input_target_data_.vel_x = 0.0f;
            input_target_data_.vel_y = 0.0f;
            input_target_data_.omega_z = 0.0f;
            return Result::kOk;
        }

        Chassis::Result Chassis::startHoming()
        {
            // 回零请求只负责“拉起状态机”和清空本轮回零参考，不直接驱动电机；
            // 真正的搜索、沿边沿捕获零位、偏置生效和完成判定都在 runThread 中按周期推进。
            homing_start_request_ = true;
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                wheel.homing_elapsed_s = 0.0f;
                wheel.homing_last_sensor_active = false;
                wheel.homing_runtime_zero_offset_rad = wheel.homing_zero_offset_rad;
                if (wheel.homing_enabled && wheel.homing_gpio_port != nullptr)
                {
                    wheel.homing_state = HomingState::kIdle;
                    wheel.homing_zero_valid = false;
                }
                else
                {
                    wheel.homing_state = HomingState::kReady;
                    wheel.homing_zero_valid = true;
                }
            }
            return Result::kOk;
        }

        bool Chassis::isHomingDone() const
        {
            for (u8 i = 0; i < 4; ++i)
            {
                if (wheel_config_[i].homing_state != HomingState::kReady)
                {
                    return false;
                }
            }
            return true;
        }

        void Chassis::setIdlePostureMode(IdlePostureMode mode)
        {
            idle_posture_mode_ = mode;
        }

        void Chassis::setSteeringStrategyMode(SteeringStrategyMode mode)
        {
            runtime_strategy_cfg_.steering_strategy_mode = mode;
        }

        void Chassis::deriveNearZeroThresholds()
        {
            const f32 enter = (near_zero_cfg_.base_enter_m_s >= 0.0f) ? near_zero_cfg_.base_enter_m_s : 0.0f;
            const f32 exit_raw = (near_zero_cfg_.base_exit_m_s >= 0.0f) ? near_zero_cfg_.base_exit_m_s : 0.0f;
            const f32 exit = (exit_raw > enter) ? exit_raw : (enter + 1.0e-3f);
            const f32 release_scale = (near_zero_cfg_.stop_guard_release_scale >= 0.0f) ? near_zero_cfg_.stop_guard_release_scale : 0.0f;
            const f32 release = enter * release_scale;

            near_zero_derived_.stationary_m_s = enter;
            near_zero_derived_.freeze_enter_m_s = enter;
            near_zero_derived_.freeze_exit_m_s = exit;
            near_zero_derived_.xpark_enter_m_s = enter;
            near_zero_derived_.xpark_exit_m_s = exit;
            near_zero_derived_.stop_guard_release_m_s = release;
        }

        void Chassis::refreshActuatorLimitState()
        {
            // 预留钩子：当前执行器限幅开关直接从 actuator_limit_enable_ 读取，无需额外派生状态。
        }

        f32 Chassis::mapSingleTurnToNearestTotalAngle(const WheelConfig &wheel, f32 target_oa_single_turn_deg) const
        {
            const f32 target_oa_mod_rad = wrapTo2Pi(degToRadF32(target_oa_single_turn_deg));
            const f32 current_oa_total_rad = wheel.corrected_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad;
            const f32 target_oa_total_rad = nearestEquivalentAngle(current_oa_total_rad, target_oa_mod_rad);
            return target_oa_total_rad - wheel.theta_oa_to_owi_rad;
        }

        void Chassis::computeProjectedDriveFromPlannedSteer(const Data &command_data, const f32 planned_oa_total_rad[4], f32 out_drive_omega_rad_s[4]) const
        {
            const f32 safe_wheel_radius = (wheel_radius_m_ > 1.0e-6f) ? wheel_radius_m_ : 1.0e-6f;
            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const f32 wheel_vx = command_data.vel_x - command_data.omega_z * wheel.pos_y_m;
                const f32 wheel_vy = command_data.vel_y + command_data.omega_z * wheel.pos_x_m;
                const f32 unit_x = cosf(planned_oa_total_rad[i]);
                const f32 unit_y = sinf(planned_oa_total_rad[i]);
                const f32 drive_linear = wheel_vx * unit_x + wheel_vy * unit_y;
                out_drive_omega_rad_s[i] = drive_linear / safe_wheel_radius;
            }
        }

        bool Chassis::estimatePlannedBodyTwist(const f32 planned_oa_total_rad[4], const f32 planned_drive_omega_rad_s[4], f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z) const
        {
            f32 normal[3][3] = {};
            f32 rhs[3] = {};

            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const f32 steer_angle_oa_rad = planned_oa_total_rad[i];
                const f32 cos_theta = cosf(steer_angle_oa_rad);
                const f32 sin_theta = sinf(steer_angle_oa_rad);
                const f32 drive_linear_m_s = planned_drive_omega_rad_s[i] * wheel_radius_m_;

                const f32 rows[2][3] = {
                    {cos_theta, sin_theta, -wheel.pos_y_m * cos_theta + wheel.pos_x_m * sin_theta},
                    {-sin_theta, cos_theta, wheel.pos_y_m * sin_theta + wheel.pos_x_m * cos_theta},
                };
                const f32 measurements[2] = {drive_linear_m_s, 0.0f};

                for (u8 row = 0; row < 2; ++row)
                {
                    for (u8 row_i = 0; row_i < 3; ++row_i)
                    {
                        rhs[row_i] += rows[row][row_i] * measurements[row];
                        for (u8 column_i = 0; column_i < 3; ++column_i)
                        {
                            normal[row_i][column_i] += rows[row][row_i] * rows[row][column_i];
                        }
                    }
                }
            }

            f32 augmented[3][4] = {
                {normal[0][0], normal[0][1], normal[0][2], rhs[0]},
                {normal[1][0], normal[1][1], normal[1][2], rhs[1]},
                {normal[2][0], normal[2][1], normal[2][2], rhs[2]},
            };

            if (!solveLinear3x3(augmented, out_vel_x, out_vel_y, out_omega_z))
            {
                out_vel_x = 0.0f;
                out_vel_y = 0.0f;
                out_omega_z = 0.0f;
                return false;
            }
            return true;
        }

        f32 Chassis::updateVectorConsistencyGate(f32 translational_speed_m_s, f32 eta_max_s, f32 dir_err_deg)
        {
            const StrategyConfig::VectorConsistencyConfig &cfg = runtime_strategy_cfg_.vector_consistency;
            if (!cfg.enable || translational_speed_m_s < cfg.min_trans_speed_enable_m_s)
            {
                vector_gate_active_ = false;
                vector_gate_scale_ = 1.0f;
                return vector_gate_scale_;
            }

            const f32 dir_enter = (cfg.dir_err_enter_deg > 0.0f) ? cfg.dir_err_enter_deg : 12.0f;
            const f32 dir_exit = (cfg.dir_err_exit_deg > 0.0f && cfg.dir_err_exit_deg < dir_enter) ? cfg.dir_err_exit_deg : (dir_enter * 0.5f);
            const f32 eta_enter = (cfg.eta_lock_s > 0.0f) ? cfg.eta_lock_s : 0.20f;
            const f32 eta_exit = (cfg.eta_release_s > 0.0f && cfg.eta_release_s < eta_enter) ? cfg.eta_release_s : (eta_enter * 0.3f);

            if (vector_gate_active_)
            {
                vector_gate_active_ = (dir_err_deg >= dir_exit) || (eta_max_s >= eta_exit);
            }
            else
            {
                vector_gate_active_ = (dir_err_deg >= dir_enter) || (eta_max_s >= eta_enter);
            }

            const f32 ramp_up_s = (cfg.gate_ramp_up_s > 1.0e-4f) ? cfg.gate_ramp_up_s : 0.08f;
            const f32 ramp_down_s = (cfg.gate_ramp_down_s > 1.0e-4f) ? cfg.gate_ramp_down_s : 0.03f;
            if (vector_gate_active_)
            {
                vector_gate_scale_ = clampValue(vector_gate_scale_ - period_ / ramp_down_s, 0.0f, 1.0f);
            }
            else
            {
                vector_gate_scale_ = clampValue(vector_gate_scale_ + period_ / ramp_up_s, 0.0f, 1.0f);
            }
            return vector_gate_scale_;
        }

        void Chassis::resetRuntimeStrategyToInitConfig()
        {
            runtime_strategy_cfg_ = default_strategy_cfg_;
            adaptive_gate_scale_ = 1.0f;
            adaptive_gate_phase_ = AdaptiveGatePhase::kIdle;
            vector_gate_scale_ = 1.0f;
            vector_gate_active_ = false;
            vector_dir_err_deg_ = 0.0f;
            vector_eta_max_s_ = 0.0f;
            xpark_gate_active_ = false;
            xpark_stationary_hold_ms_ = 0U;
            trans_dir_freeze_active_ = false;
            trans_dir_ref_valid_ = false;
            trans_dir_ref_rad_ = 0.0f;
            trans_dir_tar_mag_m_s_ = 0.0f;
            trans_dir_out_mag_m_s_ = 0.0f;
            trans_dir_freeze_reason_ = 0U;
            deriveNearZeroThresholds();
            refreshActuatorLimitState();
        }

        f32 Chassis::wrapToPi(f32 angle_rad) const
        {
            return wrapToPiRuntimeF32(angle_rad);
        }

        f32 Chassis::wrapTo2Pi(f32 angle_rad) const
        {
            return wrapTo2PiRuntimeF32(angle_rad);
        }

        f32 Chassis::shortestAngularDistance(f32 from_rad, f32 to_rad) const
        {
            return shortestAngularDistanceRuntimeF32(from_rad, to_rad);
        }

        f32 Chassis::nearestEquivalentAngle(f32 current_rad, f32 target_mod_rad) const
        {
            return nearestEquivalentAngleRuntimeF32(current_rad, target_mod_rad);
        }

        f32 Chassis::magnitude2D(f32 x, f32 y) const
        {
            return magnitude2DRuntimeF32(x, y);
        }

        f32 Chassis::getXParkAngle(const WheelConfig &wheel) const
        {
            return atan2f(wheel.pos_y_m, wheel.pos_x_m);
        }

        f32 Chassis::computeDriveGateScale(f32 abs_error_rad) const
        {
            const f32 close_rad = degToRadF32(runtime_strategy_cfg_.drive_gate_close_angle_deg);
            const f32 min_scale = clampValue(runtime_strategy_cfg_.drive_gate_min_scale, 0.0f, 1.0f);

            switch (runtime_strategy_cfg_.drive_gate_strategy)
            {
            case DriveGateStrategy::kHardGate:
                return (abs_error_rad >= close_rad) ? min_scale : 1.0f;
            case DriveGateStrategy::kSoftGate:
            {
                if (close_rad <= 1.0e-6f)
                {
                    return min_scale;
                }
                const f32 ratio = clampValue(abs_error_rad / close_rad, 0.0f, 1.0f);
                return 1.0f - (1.0f - min_scale) * ratio;
            }
            case DriveGateStrategy::kContinuousCurve:
            {
                const f32 half_rad = degToRadF32(runtime_strategy_cfg_.drive_gate_curve_half_angle_deg);
                const f32 exponent = (runtime_strategy_cfg_.drive_gate_curve_exponent > 0.1f) ? runtime_strategy_cfg_.drive_gate_curve_exponent : 2.0f;
                const f32 curve_min = clampValue(runtime_strategy_cfg_.drive_gate_curve_min_scale, 0.0f, 1.0f);
                if (half_rad <= 1.0e-6f)
                {
                    return curve_min;
                }
                const f32 norm = abs_error_rad / half_rad;
                const f32 scale = 1.0f / (1.0f + powf(norm, exponent));
                return clampValue(curve_min + (1.0f - curve_min) * scale, curve_min, 1.0f);
            }
            case DriveGateStrategy::kAdaptiveGate:
            default:
                return (abs_error_rad >= close_rad) ? min_scale : 1.0f;
            }
        }

        void Chassis::computeDriveGateScales(const f32 steering_errors_rad[4], const Data &command_data, f32 out_scales[4])
        {
            for (u8 i = 0; i < 4; ++i)
            {
                out_scales[i] = 1.0f;
            }

            if (!runtime_strategy_cfg_.enable_drive_gate)
            {
                adaptive_gate_scale_ = 1.0f;
                adaptive_gate_phase_ = AdaptiveGatePhase::kDisabled;
                return;
            }

            if (runtime_strategy_cfg_.drive_gate_strategy == DriveGateStrategy::kAdaptiveGate)
            {
                const f32 linear_speed = magnitude2D(command_data.vel_x, command_data.vel_y);
                const f32 angular_speed = fabsf(command_data.omega_z);
                const bool in_transition = (linear_speed >= runtime_strategy_cfg_.drive_gate_transition_linear_speed_m_s) ||
                                           (angular_speed >= runtime_strategy_cfg_.drive_gate_transition_angular_speed_rad_s);
                const f32 ramp_up = (runtime_strategy_cfg_.drive_gate_scale_ramp_up_s > 1.0e-6f) ? runtime_strategy_cfg_.drive_gate_scale_ramp_up_s : 0.10f;
                const f32 ramp_down = (runtime_strategy_cfg_.drive_gate_scale_ramp_down_s > 1.0e-6f) ? runtime_strategy_cfg_.drive_gate_scale_ramp_down_s : 0.06f;
                const f32 delta = period_ / (in_transition ? ramp_up : ramp_down);
                if (in_transition)
                {
                    adaptive_gate_scale_ = clampValue(adaptive_gate_scale_ + delta, 0.0f, 1.0f);
                    adaptive_gate_phase_ = AdaptiveGatePhase::kTransition;
                }
                else
                {
                    adaptive_gate_scale_ = clampValue(adaptive_gate_scale_ - delta, 0.0f, 1.0f);
                    adaptive_gate_phase_ = AdaptiveGatePhase::kStartHold;
                }
            }
            else
            {
                adaptive_gate_scale_ = 1.0f;
                adaptive_gate_phase_ = AdaptiveGatePhase::kLegacy;
            }

            if (runtime_strategy_cfg_.drive_gate_scope == DriveGateScope::kGlobal)
            {
                f32 max_abs = 0.0f;
                for (u8 i = 0; i < 4; ++i)
                {
                    if (steering_errors_rad[i] > max_abs)
                    {
                        max_abs = steering_errors_rad[i];
                    }
                }
                f32 scale = computeDriveGateScale(max_abs);
                if (runtime_strategy_cfg_.drive_gate_strategy == DriveGateStrategy::kAdaptiveGate)
                {
                    scale *= adaptive_gate_scale_;
                }
                scale = clampValue(scale, 0.0f, 1.0f);
                for (u8 i = 0; i < 4; ++i)
                {
                    out_scales[i] = scale;
                }
                return;
            }

            for (u8 i = 0; i < 4; ++i)
            {
                f32 scale = computeDriveGateScale(steering_errors_rad[i]);
                if (runtime_strategy_cfg_.drive_gate_strategy == DriveGateStrategy::kAdaptiveGate)
                {
                    scale *= adaptive_gate_scale_;
                }
                out_scales[i] = clampValue(scale, 0.0f, 1.0f);
            }
        }

        f32 Chassis::stopSteerGuardBlend(f32 residual_speed_m_s) const
        {
            const f32 release_speed = near_zero_derived_.stop_guard_release_m_s;
            if (residual_speed_m_s <= release_speed)
            {
                return 1.0f;
            }

            switch (runtime_strategy_cfg_.stop_steer_guard_strategy)
            {
            case StopSteerGuardStrategy::kHardHold:
                return 0.0f;
            case StopSteerGuardStrategy::kSoftBlend:
            {
                const f32 start_speed = (runtime_strategy_cfg_.stop_guard_blend_start_speed_m_s > release_speed)
                                            ? runtime_strategy_cfg_.stop_guard_blend_start_speed_m_s
                                            : (release_speed + 1.0e-3f);
                const f32 norm = clampValue((residual_speed_m_s - release_speed) / (start_speed - release_speed), 0.0f, 1.0f);
                return 1.0f - norm;
            }
            case StopSteerGuardStrategy::kContinuousBlend:
            default:
            {
                const f32 half_speed = (runtime_strategy_cfg_.stop_guard_curve_half_speed_m_s > 1.0e-6f)
                                           ? runtime_strategy_cfg_.stop_guard_curve_half_speed_m_s
                                           : 0.08f;
                const f32 exponent = (runtime_strategy_cfg_.stop_guard_curve_exponent > 0.1f)
                                         ? runtime_strategy_cfg_.stop_guard_curve_exponent
                                         : 2.0f;
                const f32 norm = residual_speed_m_s / half_speed;
                return 1.0f / (1.0f + powf(norm, exponent));
            }
            }
        }

        void Chassis::setModeFlag()
        {
            // 将外部模式压缩成线程内使用的少量布尔标志，后续执行顺序只看这些标志，
            // 这样可以把“世界系/车体系”“定向锁角/跟随当前角”“空转模式”解耦开。
            current_mode_flag_.is_world_speed_mode = false;
            current_mode_flag_.is_lock_now_rot_z = false;
            current_mode_flag_.is_lock_to_rot_z = false;
            current_mode_flag_.is_wheel_torque_free = false;

            switch (input_target_data_.mode)
            {
            case Mode::kWheelTorqueFreeMode:
                current_mode_flag_.is_wheel_torque_free = true;
                break;
            case Mode::kBodySpeedMode:
                break;
            case Mode::kBodySpeedLockNowRotZMode:
            case Mode::kBodySpeedLockNowRotZWithNoOmegaZMode:
                current_mode_flag_.is_lock_now_rot_z = true;
                break;
            case Mode::kBodySpeedLockToRotZMode:
                current_mode_flag_.is_lock_to_rot_z = true;
                break;
            case Mode::kWorldSpeedMode:
                current_mode_flag_.is_world_speed_mode = true;
                break;
            case Mode::kWorldSpeedLockNowRotZMode:
            case Mode::kWorldSpeedLockNowRotZWithNoOmegaZMode:
                current_mode_flag_.is_world_speed_mode = true;
                current_mode_flag_.is_lock_now_rot_z = true;
                break;
            case Mode::kWorldSpeedLockToRotZMode:
                current_mode_flag_.is_world_speed_mode = true;
                current_mode_flag_.is_lock_to_rot_z = true;
                break;
            case Mode::kSteerAngleAndDriveSpeedMode:
                break;
            default:
                break;
            }
        }

        Chassis::DebugMode Chassis::resolveDebugMode(u8 raw_mode) const
        {
            switch (raw_mode)
            {
            case 0:
                return DebugMode::kTorqueFree;
            case 1:
                return DebugMode::kBodySpeed;
            case 2:
                return DebugMode::kWorldSpeed;
            case 3:
                return DebugMode::kBodyLockNow;
            case 4:
                return DebugMode::kWorldLockNow;
            case 5:
                return DebugMode::kBodyLockTo;
            case 6:
                return DebugMode::kWorldLockTo;
            case 7:
                return DebugMode::kBodyLockNowWithNoOmegaZ;
            case 8:
                return DebugMode::kWorldLockNowWithNoOmegaZ;
            case 20:
                return DebugMode::kSingleWheel;
            case 21:
                return DebugMode::kAlignForward;
            case 22:
                return DebugMode::kHomingObserve;
            case 30:
                return DebugMode::kDirectActuator;
            default:
                return DebugMode::kTorqueFree;
            }
        }

        void Chassis::applyDebugTargetOverride()
        {
            // 手柄平移坐标 -> 车体坐标约定：前推前进、左推左移
            // 实机上 left_y 前推为负，因此这里对 X 轴取反后再映射到车体前后。
            f32 target_vel_x = -airjoy_data_.left_y * max_vel_x_;
            f32 target_vel_y = airjoy_data_.left_x * max_vel_y_;
            f32 target_omega_z = airjoy_data_.right_x * max_omega_z_;

            if (debug_control_.inject_sine)
            {
                target_omega_z = sineWaveGeneratorF32(time_ms_ / 1000.0f, debug_control_.sine_amplitude, debug_control_.sine_frequency, 0.0f, debug_control_.sine_offset);
            }
            else if (debug_control_.inject_step)
            {
                if (airjoy_data_.right_x > 0.3f)
                {
                    target_omega_z = max_omega_z_;
                }
                else if (airjoy_data_.right_x < -0.3f)
                {
                    target_omega_z = -max_omega_z_;
                }
                else
                {
                    target_omega_z = 0.0f;
                }
            }

            const DebugMode mode = resolveDebugMode(debug_control_.mode_raw);
            debug_control_.mode_resolved_raw = static_cast<u8>(mode);
            switch (mode)
            {
            case DebugMode::kTorqueFree:
                setWheelTorqueFreeMode();
                break;
            case DebugMode::kBodySpeed:
                setTargetBodySpeedMode(target_vel_x, target_vel_y, target_omega_z);
                break;
            case DebugMode::kWorldSpeed:
                setTargetWorldSpeedMode(target_vel_x, target_vel_y, target_omega_z);
                break;
            case DebugMode::kBodyLockNow:
                setTargetBodySpeedLockNowRotZMode(target_vel_x, target_vel_y);
                break;
            case DebugMode::kWorldLockNow:
                setTargetWorldSpeedLockNowRotZMode(target_vel_x, target_vel_y);
                break;
            case DebugMode::kBodyLockTo:
                setTargetBodySpeedLockToRotZMode(target_vel_x, target_vel_y, debug_control_.lock_rot_z);
                break;
            case DebugMode::kWorldLockTo:
                setTargetWorldSpeedLockToRotZMode(target_vel_x, target_vel_y, debug_control_.lock_rot_z);
                break;
            case DebugMode::kBodyLockNowWithNoOmegaZ:
                setTargetBodySpeedLockNowRotZWithNoOmegaZMode(target_vel_x, target_vel_y, target_omega_z);
                break;
            case DebugMode::kWorldLockNowWithNoOmegaZ:
                setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(target_vel_x, target_vel_y, target_omega_z);
                break;
            case DebugMode::kSingleWheel:
            case DebugMode::kAlignForward:
            case DebugMode::kHomingObserve:
            case DebugMode::kDirectActuator:
                setTargetBodySpeedMode(0.0f, 0.0f, 0.0f);
                break;
            default:
                setWheelTorqueFreeMode();
                break;
            }
        }

        void Chassis::isDebugMode()
        {
            syncDebugSteerPidTuneFromRuntimeOnEnableEdge();
            if (!debug_control_.enable)
            {
                debug_control_.mode_resolved_raw = static_cast<u8>(DebugMode::kTorqueFree);
                return;
            }
            applyDebugTargetOverride();
        }

        void Chassis::transSpeedBodyToWorld(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y) const
        {
            f32 cos_theta = cosf(input_hwt_rot_z_);
            f32 sin_theta = sinf(input_hwt_rot_z_);
            out_vel_x = vel_x * cos_theta - vel_y * sin_theta;
            out_vel_y = vel_x * sin_theta + vel_y * cos_theta;
        }

        void Chassis::transSpeedWorldToBody(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y) const
        {
            f32 cos_theta = cosf(input_hwt_rot_z_);
            f32 sin_theta = sinf(input_hwt_rot_z_);
            out_vel_x = vel_x * cos_theta + vel_y * sin_theta;
            out_vel_y = -vel_x * sin_theta + vel_y * cos_theta;
        }

        // “锁当前航向”模式的核心语义是：
        // 只要用户还在主动给 omega_z，就继续按手动旋转执行；一旦用户松开旋转输入，
        // 就把最近一次真实机体朝向当作要维持的 rot_z，再由姿态 PID 生成 out_omega_z 来稳住该朝向。
        // 因此它不是“始终锁某个固定角”，而是“手动旋转”和“松手后自动锁住当前角”之间的平滑切换器。
        void Chassis::isLockNowRotZ(bool is_lock, f32 rot_z, f32 omega_z, f32 &out_rot_z, f32 &out_omega_z)
        {
            // 未启用锁当前航向时，rot_z / omega_z 不做任何二次整形，直接透传给后续统一规划层。
            if (!is_lock)
            {
                out_rot_z = rot_z;
                out_omega_z = omega_z;
                return;
            }

            // “锁当前航向”不是简单地把 rot_z 固定住，而是先在用户开始施加角速度时
            // 抓取当前机体朝向，再在后续由 PID 产生角速度闭环，让机器人保持当下姿态。
            if (omega_z == 0.0f)
            {
                // 这里表示“用户当前没有继续施加旋转输入”。
                // 但在刚松开摇杆的最初一小段时间内，不立即让 PID 介入，而是先进入过渡缓冲：
                // 1. out_rot_z 直接跟随 IMU 当前朝向 input_hwt_rot_z_，把目标角锁在此刻真实姿态上；
                // 2. out_omega_z 先给 0，避免手动旋转刚结束时立即出现一拍突兀的 PID 修正；
                // 3. lock_now_rot_z_shift_count_ 作为缓冲计数器，倒数结束后才真正进入锁角闭环。
                if (lock_now_rot_z_shift_count_ > 0)
                {
                    lock_now_rot_z_shift_count_--;
                    lock_now_rot_z_target_ = input_hwt_rot_z_;
                    out_rot_z = lock_now_rot_z_target_;
                    out_omega_z = 0.0f;
                }
                else
                {
                    // 过渡缓冲结束后，真正用于锁角的目标已经不是外部传入的 rot_z，
                    // 而是前面已经抓取并保存下来的 lock_now_rot_z_target_。
                    // 后续由 rot_z_pid_ 根据“目标朝向 lock_now_rot_z_target_”和“当前真实朝向 input_hwt_rot_z_”
                    // 的误差生成维持姿态所需的 out_omega_z。
                    out_rot_z = lock_now_rot_z_target_;
                    if (rot_z_pid_count_ >= rot_z_pid_period_)
                    {
                        rot_z_pid_count_ = 0;
                        out_omega_z = rot_z_pid_.pid_calc(radToDegF32(lock_now_rot_z_target_), radToDegF32(input_hwt_rot_z_));
                    }
                    else
                    {
                        // PID 不是每个控制周期都重算；在未到刷新周期时，
                        // 暂时沿用上一规划周期的 planned_data_.omega_z，减少输出抖动并维持角速度连续性。
                        out_omega_z = planned_data_.omega_z;
                    }
                    // rot_z_pid_count_ / rot_z_pid_period_ 共同控制姿态 PID 的实际计算节拍。
                    rot_z_pid_count_++;
                }
            }
            else
            {
                // 这里表示“用户仍在主动要求旋转”：
                // 1. 不进入锁角闭环，直接执行当前手动 omega_z；
                // 2. 同时把 out_rot_z 刷新成当前 IMU 朝向 input_hwt_rot_z_，
                //    相当于不断更新“等会儿松手后要锁住的那个角”；
                // 3. 每次有手动旋转输入都重置缓冲计数器，为后续从手动旋转切回自动锁角预留平滑过渡窗口。
                lock_now_rot_z_target_ = input_hwt_rot_z_;
                out_rot_z = lock_now_rot_z_target_;
                out_omega_z = omega_z;
                lock_now_rot_z_shift_count_ = lock_now_rot_z_shift_time_ms_;
            }
        }

        void Chassis::isLockToRotZ(bool is_lock, f32 tar_rot_z, f32 cur_rot_z, f32 &out_rot_z, f32 omega_z, f32 &out_omega_z)
        {
            if (!is_lock)
            {
                out_rot_z = tar_rot_z;
                out_omega_z = omega_z;
                return;
            }

            // “锁到指定航向”会先限制目标角速度变化率，再用姿态 PID 生成维持/逼近该目标角度所需的 omega_z。
            // 这样外层给出的目标角不会瞬间跳变，底盘转向更平滑。
            out_rot_z = limit1DPiAngleRateByTimeF32(tar_rot_z, cur_rot_z, period_, max_lock_to_rot_z_rad_s_);
            if (rot_z_pid_count_ >= rot_z_pid_period_)
            {
                rot_z_pid_count_ = 0;
                out_omega_z = rot_z_pid_.pid_calc(radToDegF32(out_rot_z), radToDegF32(input_hwt_rot_z_));
            }
            else
            {
                out_omega_z = planned_data_.omega_z;
            }
            rot_z_pid_count_++;
        }

        void Chassis::clampTargetSpeedInChassis(f32 vel_x, f32 vel_y, f32 omega_z, f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z) const
        {
            out_vel_x = clampValue(vel_x, -max_vel_x_, max_vel_x_);
            out_vel_y = clampValue(vel_y, -max_vel_y_, max_vel_y_);
            out_omega_z = clampValue(omega_z, -max_omega_z_, max_omega_z_);
        }

        void Chassis::limitPlannedSpeed(f32 tar_vel_x, f32 tar_vel_y, f32 tar_omega_z, f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z)
        {
            // 第一阶段：先按 x/y 分量分别做加减速限幅，保证速度台阶被平滑化。
            out_vel_x = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_vel_x, last_planned_data_.vel_x, period_, max_acc_xy_acc_, max_acc_xy_dec_);
            out_vel_y = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_vel_y, last_planned_data_.vel_y, period_, max_acc_xy_acc_, max_acc_xy_dec_);
            out_omega_z = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_omega_z, last_planned_data_.omega_z, period_, max_alpha_z_acc_, max_alpha_z_dec_);

            // 第二阶段：平移矢量方向限幅（低速滞回冻结 + 方向角速度限幅）。
            const f32 tar_mag = magnitude2D(tar_vel_x, tar_vel_y);
            const f32 out_mag = magnitude2D(out_vel_x, out_vel_y);
            const f32 enter_speed = near_zero_derived_.freeze_enter_m_s;
            const f32 exit_speed = near_zero_derived_.freeze_exit_m_s;
            const f32 dir_rate_limit_rad_s = degToRadF32((trans_dir_rate_limit_deg_s_ >= 0.0f) ? trans_dir_rate_limit_deg_s_ : 0.0f);
            const f32 max_dir_step = dir_rate_limit_rad_s * period_;
            bool entered_freeze_now = false;
            trans_dir_tar_mag_m_s_ = tar_mag;
            trans_dir_out_mag_m_s_ = out_mag;
            trans_dir_freeze_reason_ = 0U;

            if (!trans_dir_ref_valid_ && out_mag > 1.0e-6f)
            {
                trans_dir_ref_rad_ = atan2f(out_vel_y, out_vel_x);
                trans_dir_ref_valid_ = true;
            }

            if (trans_dir_freeze_active_)
            {
                if ((tar_mag >= exit_speed) || (out_mag >= exit_speed))
                {
                    trans_dir_freeze_active_ = false;
                }
            }
            else if ((tar_mag <= enter_speed) && (out_mag <= enter_speed))
            {
                trans_dir_freeze_active_ = true;
                entered_freeze_now = true;
                trans_dir_freeze_reason_ = 1U;
            }

            if (out_mag <= 1.0e-6f)
            {
                out_vel_x = 0.0f;
                out_vel_y = 0.0f;
                trans_dir_ref_valid_ = false;
                trans_dir_ref_rad_ = 0.0f;
                return;
            }

            if (trans_dir_freeze_active_)
            {
                if (!entered_freeze_now)
                {
                    trans_dir_freeze_reason_ = 2U;
                }
                if (trans_dir_ref_valid_)
                {
                    out_vel_x = out_mag * cosf(trans_dir_ref_rad_);
                    out_vel_y = out_mag * sinf(trans_dir_ref_rad_);
                }
                return;
            }

            const f32 target_dir_rad = atan2f(out_vel_y, out_vel_x);
            if (!trans_dir_ref_valid_)
            {
                trans_dir_ref_rad_ = target_dir_rad;
                trans_dir_ref_valid_ = true;
            }

            f32 output_dir_rad = target_dir_rad;
            if (max_dir_step > 1.0e-6f)
            {
                const f32 dir_delta = shortestAngularDistance(trans_dir_ref_rad_, target_dir_rad);
                const f32 clamped_delta = clampValue(dir_delta, -max_dir_step, max_dir_step);
                output_dir_rad = wrapToPi(trans_dir_ref_rad_ + clamped_delta);
            }

            trans_dir_ref_rad_ = output_dir_rad;
            out_vel_x = out_mag * cosf(output_dir_rad);
            out_vel_y = out_mag * sinf(output_dir_rad);
        }

        bool Chassis::readHomingSensor(const WheelConfig &wheel) const
        {
            if (!wheel.homing_enabled || wheel.homing_gpio_port == nullptr)
            {
                return false;
            }
            const bool raw_active = readHomingSensorRawHigh(wheel);
            return wheel.homing_sensor_active_high ? raw_active : !raw_active;
        }

        bool Chassis::readHomingSensorRawHigh(const WheelConfig &wheel) const
        {
            if (!wheel.homing_enabled || wheel.homing_gpio_port == nullptr)
            {
                return false;
            }
            GPIO_TypeDef *port = reinterpret_cast<GPIO_TypeDef *>(wheel.homing_gpio_port);
            return HAL_GPIO_ReadPin(port, wheel.homing_gpio_pin) != GPIO_PIN_RESET;
        }

        f32 Chassis::readSteerMotorRawTotalAngleRad(const WheelConfig &wheel) const
        {
            if (wheel.steer_motor_h == nullptr)
            {
                return 0.0f;
            }
            return wheel.steer_motor_sign * degToRadF32(wheel.steer_motor_h->getTotalAngle());
        }

        f32 Chassis::readDriveMotorOmegaRadS(const WheelConfig &wheel) const
        {
            if (wheel.drive_motor_h == nullptr)
            {
                return 0.0f;
            }
            return wheel.drive_motor_sign * rpmToRadsF32(wheel.drive_motor_h->getRPM());
        }

        f32 Chassis::readCorrectedSteerMotorTotalAngleRad(const WheelConfig &wheel) const
        {
            return readSteerMotorRawTotalAngleRad(wheel) + wheel.homing_runtime_zero_offset_rad;
        }

        void Chassis::updateWheelFeedback()
        {
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                wheel.corrected_steer_motor_total_angle_rad = readCorrectedSteerMotorTotalAngleRad(wheel);
                wheel.corrected_drive_omega_rad_s = readDriveMotorOmegaRadS(wheel);
            }
        }

        bool Chassis::updateHomingState(WheelConfig &wheel)
        {
            // 四舵轮回零状态机的职责是：在每个周期读取限位/零位传感器，
            // 依次完成 Idle -> Search -> EdgeDetected -> OffsetApply -> ContinuousAngleReady -> AlignToZero -> Ready。
            // 这里不直接“判定一次就完成”，而是通过多周期状态推进来吸收传感器抖动和机械延迟。
            if (!wheel.homing_enabled || wheel.homing_gpio_port == nullptr)
            {
                wheel.homing_state = HomingState::kReady;
                wheel.homing_zero_valid = true;
                wheel.homing_runtime_zero_offset_rad = wheel.homing_zero_offset_rad;
                wheel.homing_last_edge_is_falling = false;
                return true;
            }

            const bool sensor_raw_high = readHomingSensorRawHigh(wheel);
            const f32 raw_total_angle_rad = readSteerMotorRawTotalAngleRad(wheel);

            if (wheel.homing_state == HomingState::kIdle)
            {
                if (homing_start_request_)
                {
                    // 收到回零请求后才进入搜索态，避免初始化阶段或无请求时误触发回零动作。
                    wheel.homing_state = HomingState::kSearch;
                    wheel.homing_elapsed_s = 0.0f;
                }
                wheel.homing_last_sensor_active = sensor_raw_high;
                return false;
            }

            if (wheel.homing_state == HomingState::kSearch)
            {
                // 搜索态严格等待“传感器边沿”，不再使用“初始有效电平直接通过”的捷径。
                // 双边沿语义（按你给的实车标定）：
                //   H->L: 触发角是机械 +60°
                //   L->H: 触发角是机械 -120°
                // 两个触发角相差 180°，保证任意起始状态半圈内都能抓到一个有效边沿。
                wheel.homing_elapsed_s += period_;
                const bool is_edge = (sensor_raw_high != wheel.homing_last_sensor_active);
                if (is_edge)
                {
                    const bool is_falling_edge = wheel.homing_last_sensor_active && !sensor_raw_high;
                    const f32 edge_mech_oa_rad = is_falling_edge ? wheel.homing_falling_edge_mech_rad : wheel.homing_rising_edge_mech_rad;
                    const f32 edge_local_corrected_rad = edge_mech_oa_rad - wheel.theta_oa_to_owi_rad;

                    wheel.homing_state = HomingState::kEdgeDetected;
                    wheel.homing_last_edge_is_falling = is_falling_edge;
                    wheel.homing_runtime_zero_offset_rad = edge_local_corrected_rad + wheel.homing_zero_offset_rad - raw_total_angle_rad;
                    wheel.homing_zero_valid = true;
                }
                else if (wheel.homing_elapsed_s > wheel.homing_timeout_s)
                {
                    wheel.homing_state = HomingState::kFault;
                }
                wheel.homing_last_sensor_active = sensor_raw_high;
                return false;
            }

            if (wheel.homing_state == HomingState::kEdgeDetected)
            {
                // 边沿已抓到后，先走一个过渡态，确保零偏已经写入后再进入连续角度就绪态。
                wheel.homing_state = HomingState::kOffsetApply;
                return false;
            }
            if (wheel.homing_state == HomingState::kOffsetApply)
            {
                // 这一拍只做“应用偏置”的状态切换，不再改零偏，保持状态机步骤清晰可追踪。
                wheel.homing_state = HomingState::kContinuousAngleReady;
                return false;
            }
            if (wheel.homing_state == HomingState::kContinuousAngleReady)
            {
                // 连续角度已可用后，先进入“归位到软件零点”阶段：
                // 让 OA 角自动走到 0°（车头前方）再判定该轮回零完成。
                wheel.homing_state = HomingState::kAlignToZero;
                return false;
            }

            if (wheel.homing_state == HomingState::kAlignToZero)
            {
                const f32 current_local_total_rad = wheel.corrected_steer_motor_total_angle_rad;
                const f32 current_oa_total_rad = current_local_total_rad + wheel.theta_oa_to_owi_rad;
                const f32 target_oa_total_rad = nearestEquivalentAngle(current_oa_total_rad, 0.0f);
                const f32 target_local_total_rad = target_oa_total_rad - wheel.theta_oa_to_owi_rad;
                const f32 oa_error_abs_rad = fabsf(shortestAngularDistance(current_oa_total_rad, target_oa_total_rad));

                wheel.target_steer_motor_total_angle_rad = target_local_total_rad;
                if (oa_error_abs_rad <= degToRadF32(homing_align_to_zero_tolerance_deg_))
                {
                    wheel.homing_state = HomingState::kReady;
                    return true;
                }
                return false;
            }

            return wheel.homing_state == HomingState::kReady;
        }

        void Chassis::setSteerMotorTargetCurrent(WheelConfig &wheel, f32 current)
        {
            if (wheel.steer_motor_h != nullptr)
            {
                wheel.steer_motor_h->setTargetCurrent(current);
            }
        }

        void Chassis::setSteerMotorTargetRPM(WheelConfig &wheel, f32 rpm)
        {
            if (wheel.steer_motor_h != nullptr)
            {
                wheel.steer_motor_h->setTargetRPM(rpm / wheel.steer_motor_sign);
            }
        }

        void Chassis::setSteerMotorTargetTotalAngleRad(WheelConfig &wheel, f32 corrected_local_total_angle_rad)
        {
            if (wheel.steer_motor_h == nullptr)
            {
                return;
            }
            f32 raw_motor_total_rad = (corrected_local_total_angle_rad - wheel.homing_runtime_zero_offset_rad) / wheel.steer_motor_sign;
            wheel.steer_motor_h->setTargetTotalAngle(radToDegF32(raw_motor_total_rad));
        }

        void Chassis::setDriveMotorTargetOmegaRadS(WheelConfig &wheel, f32 drive_omega_rad_s)
        {
            if (wheel.drive_motor_h != nullptr)
            {
                wheel.drive_motor_h->setTargetRPM(radsToRpmF32(drive_omega_rad_s / wheel.drive_motor_sign));
            }
        }

        // 这是一个“位置目标 + 速度上限 + 加速度上限”的二阶限幅器。
        // 输入是当前位置 current_value、当前速度 current_rate 和目标位置 target_value，
        // 输出是“下一拍允许走到的位置”，并通过 next_rate 回传这一拍实际采用的速度。
        // 在四舵轮里它主要用于转向角规划：既不允许舵角变化过快，也不允许舵角速度突变过猛。
        f32 Chassis::limitPositionSecondOrder(f32 current_value, f32 current_rate, f32 target_value, f32 max_rate, f32 max_accel, f32 dt_s, f32 &next_rate) const
        {
            // 防止 dt 过小导致除零或数值放大；在异常小周期下退回一个保守的 1ms 步长。
            const f32 safe_dt = (dt_s <= 1.0e-6f) ? 1.0e-3f : dt_s;

            // delta_value 是这一拍距离目标位置还差多少；
            // desired_rate 是“如果想在一拍内尽量逼近目标，希望使用的速度”，
            // 但它先受 max_rate 限制，避免直接给出不可能达到的目标速度。
            const f32 delta_value = target_value - current_value;
            const f32 desired_rate = clampValue(delta_value / safe_dt, -max_rate, max_rate);

            // rate_delta_limit 是“这一拍速度最多允许变化多少”，由最大加速度决定。
            const f32 rate_delta_limit = max_accel * safe_dt;

            next_rate = current_rate;

            // 先做速度变化率限制：如果期望速度离当前速度太远，
            // 这一拍只允许按 max_accel 推进一步，而不是瞬间跳到 desired_rate。
            if (desired_rate > current_rate + rate_delta_limit)
            {
                next_rate = current_rate + rate_delta_limit;
            }
            else if (desired_rate < current_rate - rate_delta_limit)
            {
                next_rate = current_rate - rate_delta_limit;
            }
            else
            {
                next_rate = desired_rate;
            }

            // 再做一次绝对速度限幅，保证最终速度不超过 max_rate。
            next_rate = clampValue(next_rate, -max_rate, max_rate);

            // 按这一拍最终允许的速度积分出位置步进量。
            f32 step_value = next_rate * safe_dt;

            // 如果这一拍已经足够到达目标，则直接截断到目标位置，
            // 避免积分后跨过 target_value 造成过冲。
            if (fabsf(step_value) > fabsf(delta_value))
            {
                step_value = delta_value;
                next_rate = step_value / safe_dt;
            }

            // 返回下一拍允许到达的位置；调用侧会把它当作新的舵角目标。
            return current_value + step_value;
        }

        f32 Chassis::limitValueWithAcceleration(f32 current_value, f32 target_value, f32 max_accel, f32 dt_s) const
        {
            const f32 safe_dt = (dt_s <= 1.0e-6f) ? 1.0e-3f : dt_s;
            const f32 delta_limit = max_accel * safe_dt;
            const f32 delta_value = target_value - current_value;
            if (delta_value > delta_limit)
            {
                return current_value + delta_limit;
            }
            if (delta_value < -delta_limit)
            {
                return current_value - delta_limit;
            }
            return target_value;
        }

        void Chassis::computeModuleCommands(const Data &command_data)
        {
            f32 current_oa_total_rad[4] = {0.0f};
            f32 target_drive_raw_rad_s[4] = {0.0f};
            f32 selected_oa_total_rad[4] = {0.0f};
            f32 steering_errors_rad[4] = {0.0f};
            f32 planned_local_total_rad_arr[4] = {0.0f};
            f32 planned_oa_total_rad_arr[4] = {0.0f};
            f32 next_steer_rate_rad_s_arr[4] = {0.0f};
            f32 wheel_vx_m_s[4] = {0.0f};
            f32 wheel_vy_m_s[4] = {0.0f};
            f32 wheel_speed_m_s_arr[4] = {0.0f};
            f32 residual_speed_m_s_arr[4] = {0.0f};

            f32 max_command_wheel_speed_m_s = 0.0f;
            f32 max_residual_speed_m_s = 0.0f;

            // 第一阶段：采样每轮速度与残速，先建立“驻车进入门控”的判据。
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                const f32 current_local_total = wheel.corrected_steer_motor_total_angle_rad;
                current_oa_total_rad[i] = current_local_total + wheel.theta_oa_to_owi_rad;

                const f32 wheel_vx = command_data.vel_x - command_data.omega_z * wheel.pos_y_m;
                const f32 wheel_vy = command_data.vel_y + command_data.omega_z * wheel.pos_x_m;
                const f32 wheel_speed_m_s = magnitude2D(wheel_vx, wheel_vy);
                wheel_vx_m_s[i] = wheel_vx;
                wheel_vy_m_s[i] = wheel_vy;
                wheel_speed_m_s_arr[i] = wheel_speed_m_s;
                max_command_wheel_speed_m_s = (wheel_speed_m_s > max_command_wheel_speed_m_s) ? wheel_speed_m_s : max_command_wheel_speed_m_s;

                const f32 residual_speed_m_s = fabsf(wheel.corrected_drive_omega_rad_s) * wheel_radius_m_;
                residual_speed_m_s_arr[i] = residual_speed_m_s;
                max_residual_speed_m_s = (residual_speed_m_s > max_residual_speed_m_s) ? residual_speed_m_s : max_residual_speed_m_s;
            }

            const f32 xpark_enter_speed = near_zero_derived_.xpark_enter_m_s;
            const f32 xpark_exit_speed = near_zero_derived_.xpark_exit_m_s;

            const bool command_stationary_intent = xpark_gate_active_
                                                       ? (max_command_wheel_speed_m_s <= xpark_exit_speed)
                                                       : (max_command_wheel_speed_m_s <= xpark_enter_speed);

            if (command_stationary_intent)
            {
                xpark_stationary_hold_ms_ = (xpark_stationary_hold_ms_ > (0xFFFFFFFFU - period_ms_))
                                                ? 0xFFFFFFFFU
                                                : (xpark_stationary_hold_ms_ + period_ms_);
                if (xpark_stationary_hold_ms_ >= near_zero_cfg_.xpark_entry_delay_ms)
                {
                    xpark_gate_active_ = true;
                }
            }
            else
            {
                xpark_stationary_hold_ms_ = 0U;
                xpark_gate_active_ = false;
            }

            const bool allow_xpark_pose = command_stationary_intent && xpark_gate_active_;
            const bool force_uniform_steer_drive = (input_target_data_.mode == Mode::kSteerAngleAndDriveSpeedMode);
            const f32 uniform_steer_oa_mod_rad = wrapTo2Pi(degToRadF32(input_target_data_.steer_lock_angle_deg));
            const f32 uniform_drive_omega_abs = fabsf(input_target_data_.drive_lock_speed_m_s) / wheel_radius_m_;
            const f32 uniform_drive_sign = (input_target_data_.drive_lock_speed_m_s >= 0.0f) ? 1.0f : -1.0f;

            // 第二阶段：计算每轮目标、翻转候选与误差（含 X-Park 延时门控）。
            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const f32 wheel_speed_m_s = wheel_speed_m_s_arr[i];
                const bool is_stationary = wheel_speed_m_s <= near_zero_derived_.stationary_m_s;
                f32 raw_target_oa_mod_rad = 0.0f;
                f32 drive_omega = 0.0f;

                if (is_stationary)
                {
                    raw_target_oa_mod_rad = (allow_xpark_pose && idle_posture_mode_ == IdlePostureMode::kXPark)
                                                ? wrapTo2Pi(getXParkAngle(wheel))
                                                : wrapTo2Pi(current_oa_total_rad[i]);
                    drive_omega = 0.0f;
                }
                else
                {
                    raw_target_oa_mod_rad = wrapTo2Pi(atan2f(wheel_vy_m_s[i], wheel_vx_m_s[i]));
                    drive_omega = wheel_speed_m_s / wheel_radius_m_;
                }

                const f32 alt_target_oa_mod_rad = wrapTo2Pi(raw_target_oa_mod_rad + kPi);
                const f32 candidate_a = nearestEquivalentAngle(current_oa_total_rad[i], raw_target_oa_mod_rad);
                const f32 candidate_b = nearestEquivalentAngle(current_oa_total_rad[i], alt_target_oa_mod_rad);

                f32 selected = candidate_a;
                bool flipped = false;
                if (!is_stationary)
                {
                    if (runtime_strategy_cfg_.steering_strategy_mode == SteeringStrategyMode::kAlwaysForward)
                    {
                        flipped = false;
                    }
                    else
                    {
                        const f32 base_abs_deg = radToDegF32(fabsf(candidate_a - current_oa_total_rad[i]));
                        const f32 flip_abs_deg = radToDegF32(fabsf(candidate_b - current_oa_total_rad[i]));
                        if (selected_flipped_solution_[i])
                        {
                            flipped = flip_abs_deg <= runtime_strategy_cfg_.flip_enter_angle_deg;
                        }
                        else
                        {
                            flipped = (base_abs_deg > runtime_strategy_cfg_.flip_exit_angle_deg) && (flip_abs_deg < base_abs_deg);
                        }
                    }
                }

                if (flipped)
                {
                    selected = candidate_b;
                    drive_omega = -drive_omega;
                }

                if (force_uniform_steer_drive)
                {
                    const f32 fixed_a = nearestEquivalentAngle(current_oa_total_rad[i], uniform_steer_oa_mod_rad);
                    const f32 fixed_b = nearestEquivalentAngle(current_oa_total_rad[i], wrapTo2Pi(uniform_steer_oa_mod_rad + kPi));
                    const bool use_b = fabsf(shortestAngularDistance(current_oa_total_rad[i], fixed_b)) <
                                       fabsf(shortestAngularDistance(current_oa_total_rad[i], fixed_a));
                    selected = use_b ? fixed_b : fixed_a;
                    const f32 selected_sign = use_b ? -1.0f : 1.0f;
                    drive_omega = uniform_drive_sign * selected_sign * uniform_drive_omega_abs;
                    flipped = use_b;
                }

                selected_oa_total_rad[i] = selected;
                selected_flipped_solution_[i] = flipped;
                steering_errors_rad[i] = fabsf(shortestAngularDistance(current_oa_total_rad[i], selected));
                target_drive_raw_rad_s[i] = drive_omega;
            }

            // 第三阶段：停车抑制（指令静止但残速未消失时，先保舵角）。
            const bool command_is_stationary = command_stationary_intent;
            const bool residual_drive_is_moving = max_residual_speed_m_s > near_zero_derived_.stop_guard_release_m_s;
            if (!force_uniform_steer_drive &&
                runtime_strategy_cfg_.enable_stop_steer_guard &&
                command_is_stationary &&
                residual_drive_is_moving)
            {
                for (u8 i = 0; i < 4; ++i)
                {
                    const f32 residual_speed_m_s = residual_speed_m_s_arr[i];
                    const f32 blend = stopSteerGuardBlend(residual_speed_m_s);
                    if (blend >= 1.0f)
                    {
                        continue;
                    }

                    const f32 current = current_oa_total_rad[i];
                    const f32 protected_target = wrapTo2Pi(
                        current + shortestAngularDistance(current, selected_oa_total_rad[i]) * blend);
                    selected_oa_total_rad[i] = protected_target;
                    steering_errors_rad[i] = fabsf(shortestAngularDistance(current, protected_target));
                    selected_flipped_solution_[i] = false;
                }
            }

            // 第四阶段：驱动抑制比例（DriveGate 或余弦补偿）。
            f32 gate_scales[4] = {1.0f, 1.0f, 1.0f, 1.0f};
            computeDriveGateScales(steering_errors_rad, command_data, gate_scales);

            // 第五阶段：先规划舵向目标，并统计全局“最大到角时间”。
            const f32 steer_rate_floor = 1.0e-3f;
            f32 eta_max_s = 0.0f;
            const f32 steer_rate_limit_runtime = actuator_limit_enable_.enable_steer_rate_limit ? max_steer_rate_rad_s_ : 1.0e6f;
            const f32 steer_alpha_limit_runtime = actuator_limit_enable_.enable_steer_alpha_limit ? max_steer_alpha_rad_s2_ : 1.0e8f;
            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const f32 current_local_total = wheel.corrected_steer_motor_total_angle_rad;
                const f32 selected_local_total = selected_oa_total_rad[i] - wheel.theta_oa_to_owi_rad;

                f32 next_steer_rate_rad_s = 0.0f;
                planned_local_total_rad_arr[i] = limitPositionSecondOrder(
                    current_local_total,
                    last_steer_rate_cmd_rad_s_[i],
                    selected_local_total,
                    steer_rate_limit_runtime,
                    steer_alpha_limit_runtime,
                    period_,
                    next_steer_rate_rad_s);
                next_steer_rate_rad_s_arr[i] = next_steer_rate_rad_s;

                const f32 remain_angle_rad = steering_errors_rad[i];
                f32 steer_rate_ref = fabsf(next_steer_rate_rad_s);
                if (steer_rate_ref < steer_rate_floor)
                {
                    steer_rate_ref = (steer_rate_limit_runtime > steer_rate_floor) ? steer_rate_limit_runtime : steer_rate_floor;
                }

                const f32 eta_s = remain_angle_rad / steer_rate_ref;
                eta_max_s = (eta_s > eta_max_s) ? eta_s : eta_max_s;
                planned_oa_total_rad_arr[i] = planned_local_total_rad_arr[i] + wheel.theta_oa_to_owi_rad;
            }

            // 第六阶段：基于“规划舵向角”重算每轮驱动投影，保证过渡期驱动与可实现滚动方向一致。
            if (!force_uniform_steer_drive)
            {
                computeProjectedDriveFromPlannedSteer(command_data, planned_oa_total_rad_arr, target_drive_raw_rad_s);
            }

            // 第七阶段：全局矢量一致性门控（方向优先）——根据合成方向误差与最大到角时间统一压放驱动。
            const f32 translational_speed_m_s = magnitude2D(command_data.vel_x, command_data.vel_y);
            f32 predicted_vel_x = 0.0f;
            f32 predicted_vel_y = 0.0f;
            f32 predicted_omega_z = 0.0f;
            f32 dir_err_deg = 0.0f;
            if (estimatePlannedBodyTwist(planned_oa_total_rad_arr, target_drive_raw_rad_s, predicted_vel_x, predicted_vel_y, predicted_omega_z))
            {
                const f32 predicted_trans_speed_m_s = magnitude2D(predicted_vel_x, predicted_vel_y);
                if ((translational_speed_m_s > 1.0e-6f) && (predicted_trans_speed_m_s > 1.0e-6f))
                {
                    const f32 target_dir_rad = atan2f(command_data.vel_y, command_data.vel_x);
                    const f32 predicted_dir_rad = atan2f(predicted_vel_y, predicted_vel_x);
                    dir_err_deg = radToDegF32(fabsf(shortestAngularDistance(target_dir_rad, predicted_dir_rad)));
                }
            }
            vector_eta_max_s_ = eta_max_s;
            vector_dir_err_deg_ = dir_err_deg;
            const f32 vector_gate_scale = force_uniform_steer_drive
                                              ? 1.0f
                                              : updateVectorConsistencyGate(translational_speed_m_s, eta_max_s, dir_err_deg);
            if (force_uniform_steer_drive)
            {
                vector_gate_scale_ = 1.0f;
                vector_gate_active_ = false;
            }

            // 第八阶段：下发前限幅与缓存（DriveGate/余弦补偿作为终端保护，与全局门控叠乘）。
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                f32 gate_or_cos_scale = 1.0f;
                if (force_uniform_steer_drive)
                {
                    gate_or_cos_scale = 1.0f;
                }
                else if (runtime_strategy_cfg_.enable_drive_gate)
                {
                    gate_or_cos_scale = gate_scales[i];
                }
                else if (enable_cosine_compensation_)
                {
                    gate_or_cos_scale = cosf(steering_errors_rad[i]);
                    if (gate_or_cos_scale < 0.0f)
                    {
                        gate_or_cos_scale = 0.0f;
                    }
                }

                const f32 drive_scale = clampValue(gate_or_cos_scale * vector_gate_scale, 0.0f, 1.0f);
                f32 target_drive_omega = target_drive_raw_rad_s[i] * drive_scale;
                if (actuator_limit_enable_.enable_drive_omega_limit)
                {
                    target_drive_omega = clampValue(target_drive_omega, -max_drive_omega_rad_s_, max_drive_omega_rad_s_);
                }
                drive_gate_scale_[i] = drive_scale;
                wheel.target_steer_motor_total_angle_rad = planned_local_total_rad_arr[i];
                if (actuator_limit_enable_.enable_drive_alpha_limit)
                {
                    wheel.target_drive_omega_rad_s = limitValueWithAcceleration(last_drive_omega_cmd_rad_s_[i], target_drive_omega, max_drive_alpha_rad_s2_, period_);
                }
                else
                {
                    wheel.target_drive_omega_rad_s = target_drive_omega;
                }
                if (actuator_limit_enable_.enable_drive_omega_limit)
                {
                    wheel.target_drive_omega_rad_s = clampValue(wheel.target_drive_omega_rad_s, -max_drive_omega_rad_s_, max_drive_omega_rad_s_);
                }

                wheel.steer_target_velocity_rad_s = next_steer_rate_rad_s_arr[i];
                wheel.flipped_drive_direction = selected_flipped_solution_[i];

                last_steer_rate_cmd_rad_s_[i] = next_steer_rate_rad_s_arr[i];
                last_drive_omega_cmd_rad_s_[i] = wheel.target_drive_omega_rad_s;
                planned_data_.steer_angle_oa_rad[i] = wheel.target_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad;
                planned_data_.drive_omega_rad_s[i] = wheel.target_drive_omega_rad_s;
            }
        }

        void Chassis::applyModuleCommands(bool all_homed)
        {
            // 这里是“四舵轮目标命令”真正落到电机接口前的最后一道门控：
            // computeModuleCommands() 虽然已经为每个轮子算好了目标舵角和驱动速度，
            // 但是否允许按这些目标下发，还要看当前是否全部完成回零，以及是否处于扭矩自由模式。
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];

                if (input_target_data_.zero_current_all)
                {
                    // 硬零电流模式优先级最高：无论回零状态如何，四轮舵向/驱动都直接下发 0 电流。
                    setSteerMotorTargetCurrent(wheel, 0.0f);
                    if (wheel.drive_motor_h != nullptr)
                    {
                        wheel.drive_motor_h->setTargetCurrent(0.0f);
                    }
                    continue;
                }

                if (!all_homed)
                {
                    // 只要还有任意一个轮子没有完成回零，就先禁止所有驱动轮输出，
                    // 避免底盘在零位未建立完成时带着错误朝向强行跑动。
                    setDriveMotorTargetOmegaRadS(wheel, 0.0f);
                    if (wheel.homing_state == HomingState::kSearch)
                    {
                        // 正在搜索零位的轮子，允许转向电机按固定搜索转速慢慢转，
                        // 目的是继续寻找传感器边沿；此时不走位置闭环。
                        setSteerMotorTargetRPM(wheel, wheel.homing_search_rpm);
                    }
                    else if (wheel.homing_state == HomingState::kAlignToZero)
                    {
                        // 零偏建立后允许转向电机继续走位置闭环，把 OA 自动归到软件零点。
                        // 注意：这里每拍都根据当前反馈重算“离 OA=0 最近的等效角”，
                        // 避免被上游常规模块解算写回“保持当前角”后导致归位停滞。
                        const f32 current_local_total_rad = wheel.corrected_steer_motor_total_angle_rad;
                        const f32 current_oa_total_rad = current_local_total_rad + wheel.theta_oa_to_owi_rad;
                        const f32 align_target_oa_total_rad = nearestEquivalentAngle(current_oa_total_rad, 0.0f);
                        wheel.target_steer_motor_total_angle_rad = align_target_oa_total_rad - wheel.theta_oa_to_owi_rad;
                        setSteerMotorTargetTotalAngleRad(wheel, wheel.target_steer_motor_total_angle_rad);
                    }
                    else
                    {
                        // 不在搜索态的轮子，不再给转向动作，直接把转向电机电流打零，
                        // 让状态机以“静止等待”的方式完成后续过渡。
                        setSteerMotorTargetCurrent(wheel, 0.0f);
                    }
                    continue;
                }

                if (current_mode_flag_.is_wheel_torque_free)
                {
                    // 扭矩自由模式下，不执行任何舵角或驱动速度闭环，
                    // 而是把转向和驱动都打成“零电流/零扭矩”状态，方便人工推动或安全释放。
                    setSteerMotorTargetCurrent(wheel, 0.0f);
                    if (wheel.drive_motor_h != nullptr)
                    {
                        wheel.drive_motor_h->setTargetCurrent(0.0f);
                    }
                    continue;
                }

                // 只有“全部回零完成”且“不是扭矩自由模式”时，
                // 才真正把上一阶段规划出的目标舵角和驱动角速度下发给电机闭环。
                setSteerMotorTargetTotalAngleRad(wheel, wheel.target_steer_motor_total_angle_rad);
                setDriveMotorTargetOmegaRadS(wheel, wheel.target_drive_omega_rad_s);
            }
        }

        void Chassis::updateCurrentData(bool all_homed)
        {
            current_data_ = planned_data_;
            if (!all_homed)
            {
                current_data_.vel_x = 0.0f;
                current_data_.vel_y = 0.0f;
                current_data_.omega_z = 0.0f;
            }

            for (u8 i = 0; i < 4; ++i)
            {
                current_data_.steer_angle_oa_rad[i] = wheel_config_[i].corrected_steer_motor_total_angle_rad + wheel_config_[i].theta_oa_to_owi_rad;
                current_data_.drive_omega_rad_s[i] = wheel_config_[i].corrected_drive_omega_rad_s;
            }

            if (all_homed)
            {
                estimateBodySpeedFromModules(current_data_.vel_x, current_data_.vel_y, current_data_.omega_z);
            }
            else
            {
                current_data_.vel_x = 0.0f;
                current_data_.vel_y = 0.0f;
                current_data_.omega_z = 0.0f;
            }
        }

        void Chassis::refreshDebugMirror(bool all_homed)
        {
            debug_mirror_.all_homed = all_homed;
            debug_mirror_.selected_wheel_steer_error_deg = 0.0f;
            debug_mirror_.selected_wheel_drive_released = false;
            debug_mirror_.nz_stationary_m_s = near_zero_derived_.stationary_m_s;
            debug_mirror_.nz_freeze_enter_m_s = near_zero_derived_.freeze_enter_m_s;
            debug_mirror_.nz_freeze_exit_m_s = near_zero_derived_.freeze_exit_m_s;
            debug_mirror_.nz_xpark_enter_m_s = near_zero_derived_.xpark_enter_m_s;
            debug_mirror_.nz_xpark_exit_m_s = near_zero_derived_.xpark_exit_m_s;
            debug_mirror_.nz_stop_guard_release_m_s = near_zero_derived_.stop_guard_release_m_s;
            debug_mirror_.lim_drive_omega = actuator_limit_enable_.enable_drive_omega_limit;
            debug_mirror_.lim_drive_alpha = actuator_limit_enable_.enable_drive_alpha_limit;
            debug_mirror_.lim_steer_rate = actuator_limit_enable_.enable_steer_rate_limit;
            debug_mirror_.lim_steer_alpha = actuator_limit_enable_.enable_steer_alpha_limit;
            debug_mirror_.vec_gate_scale = vector_gate_scale_;
            debug_mirror_.vec_dir_err_deg = vector_dir_err_deg_;
            debug_mirror_.vec_eta_max_s = vector_eta_max_s_;
            debug_mirror_.vec_gate_active = vector_gate_active_;
            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                debug_mirror_.current_oa_deg[i] = radToDegF32(wheel.corrected_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad);
                debug_mirror_.target_oa_deg[i] = radToDegF32(wheel.target_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad);
                debug_mirror_.current_drive_rpm[i] = radsToRpmF32(wheel.corrected_drive_omega_rad_s);
                debug_mirror_.target_drive_rpm[i] = radsToRpmF32(wheel.target_drive_omega_rad_s);
                debug_mirror_.homing_state[i] = static_cast<u8>(wheel.homing_state);
                debug_mirror_.homing_sensor_active[i] = readHomingSensor(wheel);
                debug_mirror_.homing_last_edge_is_falling[i] = wheel.homing_last_edge_is_falling;
                debug_mirror_.homing_runtime_zero_offset_deg[i] = radToDegF32(wheel.homing_runtime_zero_offset_rad);
            }
        }

        void Chassis::syncDebugSteerPidTuneFromRuntimeOnEnableEdge()
        {
            const bool enable_now = debug_control_.enable;
            if (!enable_now)
            {
                debug_pid_tune_.synced_on_enable_edge = false;
                debug_enable_last_cycle_ = false;
                return;
            }

            if (!debug_enable_last_cycle_)
            {
                // 调试使能上升沿：从电机运行态回读 PID 到调参缓存，形成“先读后改”基线。
                syncDebugSteerPidTuneFromRuntime();
                debug_pid_tune_.synced_on_enable_edge = true;
            }
            debug_enable_last_cycle_ = true;
        }

        void Chassis::syncDebugSteerPidTuneFromRuntime()
        {
            for (u8 i = 0; i < 4; ++i)
            {
                const bool speed_dirty = (debug_pid_tune_.steer_speed_pid_applied_stamp[i] != debug_pid_tune_.steer_speed_pid_apply_stamp[i]);
                const bool angle_dirty = (debug_pid_tune_.steer_angle_pid_applied_stamp[i] != debug_pid_tune_.steer_angle_pid_apply_stamp[i]);
                if (speed_dirty || angle_dirty)
                {
                    // 保护未 apply 的手工改动：该轮缓存跳过同步，避免覆盖调试器刚写入的值。
                    continue;
                }

                WheelConfig &wheel = wheel_config_[i];
                M3508 *steer_m3508 = static_cast<M3508 *>(wheel.steer_motor_h);
                if (steer_m3508 == nullptr)
                {
                    continue;
                }

                debug_pid_tune_.steer_speed_pid_cfg[i] = steer_m3508->get_speed_pid_params();
                debug_pid_tune_.steer_angle_pid_cfg[i] = steer_m3508->get_angle_pid_params();
                debug_pid_tune_.steer_speed_pid_td_ratio[i] = steer_m3508->get_speed_pid_td_ratio();
                debug_pid_tune_.steer_angle_pid_i_separa[i] = steer_m3508->get_angle_pid_i_separa_threshold();
            }
        }

        void Chassis::applyDebugSteerPidRuntimeTuning()
        {
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                M3508 *steer_m3508 = static_cast<M3508 *>(wheel.steer_motor_h);
                if (steer_m3508 == nullptr)
                {
                    continue;
                }

                if (debug_pid_tune_.steer_speed_pid_applied_stamp[i] != debug_pid_tune_.steer_speed_pid_apply_stamp[i])
                {
                    steer_m3508->pid_init(debug_pid_tune_.steer_speed_pid_cfg[i], debug_pid_tune_.steer_speed_pid_td_ratio[i],
                                          debug_pid_tune_.steer_angle_pid_cfg[i], debug_pid_tune_.steer_angle_pid_i_separa[i]);
                    debug_pid_tune_.steer_speed_pid_applied_stamp[i] = debug_pid_tune_.steer_speed_pid_apply_stamp[i];
                    debug_pid_tune_.steer_angle_pid_applied_stamp[i] = debug_pid_tune_.steer_angle_pid_apply_stamp[i];
                }
                if (debug_pid_tune_.steer_angle_pid_applied_stamp[i] != debug_pid_tune_.steer_angle_pid_apply_stamp[i])
                {
                    steer_m3508->pid_init(debug_pid_tune_.steer_speed_pid_cfg[i], debug_pid_tune_.steer_speed_pid_td_ratio[i],
                                          debug_pid_tune_.steer_angle_pid_cfg[i], debug_pid_tune_.steer_angle_pid_i_separa[i]);
                    debug_pid_tune_.steer_angle_pid_applied_stamp[i] = debug_pid_tune_.steer_angle_pid_apply_stamp[i];
                    debug_pid_tune_.steer_speed_pid_applied_stamp[i] = debug_pid_tune_.steer_speed_pid_apply_stamp[i];
                }
            }
        }

        void Chassis::emitDebugUart8Log(bool all_homed)
        {
            if (!debug_output_.output_enable || debug_output_.output_mode_raw != 1U)
            {
                return;
            }

            const u32 period_ms = (debug_output_.text_period_ms > 0U) ? debug_output_.text_period_ms : 500U;
            if ((time_ms_ - debug_output_.text_last_ms) < period_ms)
            {
                return;
            }

            if (HAL_UART_GetState(&huart8) != HAL_UART_STATE_READY)
            {
                return;
            }

            debug_output_.text_last_ms = time_ms_;
            if (debug_output_.text_log_level == 0U)
            {
                debug_uart_.printf_DMA((char *)"FS t=%lu home=%u mode=%u dbg=%u hs=%u/%u/%u/%u oa0=%.1f->%.1f rpm0=%.1f->%.1f\r\n",
                                       (u32)time_ms_,
                                       all_homed ? 1U : 0U,
                                       (u32)input_target_data_.mode,
                                       debug_control_.enable ? 1U : 0U,
                                       (u32)debug_mirror_.homing_state[0],
                                       (u32)debug_mirror_.homing_state[1],
                                       (u32)debug_mirror_.homing_state[2],
                                       (u32)debug_mirror_.homing_state[3],
                                       debug_mirror_.current_oa_deg[0],
                                       debug_mirror_.target_oa_deg[0],
                                       debug_mirror_.current_drive_rpm[0],
                                       debug_mirror_.target_drive_rpm[0]);
                return;
            }

            const u8 wheel_idx = (debug_control_.wheel_index < 4) ? debug_control_.wheel_index : 0;
            if (debug_output_.text_log_phase == 0U)
            {
                debug_uart_.printf_DMA((char *)"FS t=%lu home=%u mode=%u dbg=%u hs=%u/%u/%u/%u oa0=%.1f->%.1f rpm0=%.1f->%.1f vec=%.2f de=%.1f eta=%.3f va=%u\r\n",
                                       (u32)time_ms_,
                                       all_homed ? 1U : 0U,
                                       (u32)input_target_data_.mode,
                                       debug_control_.enable ? 1U : 0U,
                                       (u32)debug_mirror_.homing_state[0],
                                       (u32)debug_mirror_.homing_state[1],
                                       (u32)debug_mirror_.homing_state[2],
                                       (u32)debug_mirror_.homing_state[3],
                                       debug_mirror_.current_oa_deg[0],
                                       debug_mirror_.target_oa_deg[0],
                                       debug_mirror_.current_drive_rpm[0],
                                       debug_mirror_.target_drive_rpm[0],
                                       debug_mirror_.vec_gate_scale,
                                       debug_mirror_.vec_dir_err_deg,
                                       debug_mirror_.vec_eta_max_s,
                                       debug_mirror_.vec_gate_active ? 1U : 0U);
            }
            else if (debug_output_.text_log_phase == 1U)
            {
                debug_uart_.printf_DMA((char *)"FSW i=%u hs=%u oa=%.1f->%.1f rpm=%.1f->%.1f gate=%.2f flip=%u sensor=%u edge=%u rel=%u err=%.2f\r\n",
                                       (u32)wheel_idx,
                                       (u32)debug_mirror_.homing_state[wheel_idx],
                                       debug_mirror_.current_oa_deg[wheel_idx],
                                       debug_mirror_.target_oa_deg[wheel_idx],
                                       debug_mirror_.current_drive_rpm[wheel_idx],
                                       debug_mirror_.target_drive_rpm[wheel_idx],
                                       drive_gate_scale_[wheel_idx],
                                       wheel_config_[wheel_idx].flipped_drive_direction ? 1U : 0U,
                                       debug_mirror_.homing_sensor_active[wheel_idx] ? 1U : 0U,
                                       debug_mirror_.homing_last_edge_is_falling[wheel_idx] ? 1U : 0U,
                                       debug_mirror_.selected_wheel_drive_released ? 1U : 0U,
                                       debug_mirror_.selected_wheel_steer_error_deg);
            }
            else
            {
                f32 align_err_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                for (u8 i = 0; i < 4; ++i)
                {
                    const WheelConfig &wheel = wheel_config_[i];
                    const f32 current_oa_total_rad = wheel.corrected_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad;
                    const f32 align_target_oa_total_rad = nearestEquivalentAngle(current_oa_total_rad, 0.0f);
                    align_err_deg[i] = radToDegF32(shortestAngularDistance(current_oa_total_rad, align_target_oa_total_rad));
                }

                debug_uart_.printf_DMA((char *)"FSH hs=%u/%u/%u/%u curOA=%.1f/%.1f/%.1f/%.1f tarOA=%.1f/%.1f/%.1f/%.1f err0=%.1f/%.1f/%.1f/%.1f zoff=%.1f/%.1f/%.1f/%.1f\r\n",
                                       (u32)debug_mirror_.homing_state[0],
                                       (u32)debug_mirror_.homing_state[1],
                                       (u32)debug_mirror_.homing_state[2],
                                       (u32)debug_mirror_.homing_state[3],
                                       debug_mirror_.current_oa_deg[0],
                                       debug_mirror_.current_oa_deg[1],
                                       debug_mirror_.current_oa_deg[2],
                                       debug_mirror_.current_oa_deg[3],
                                       debug_mirror_.target_oa_deg[0],
                                       debug_mirror_.target_oa_deg[1],
                                       debug_mirror_.target_oa_deg[2],
                                       debug_mirror_.target_oa_deg[3],
                                       align_err_deg[0],
                                       align_err_deg[1],
                                       align_err_deg[2],
                                       align_err_deg[3],
                                       debug_mirror_.homing_runtime_zero_offset_deg[0],
                                       debug_mirror_.homing_runtime_zero_offset_deg[1],
                                       debug_mirror_.homing_runtime_zero_offset_deg[2],
                                       debug_mirror_.homing_runtime_zero_offset_deg[3]);
            }

            debug_output_.text_log_phase = (u8)((debug_output_.text_log_phase + 1U) % 3U);
        }

        void Chassis::emitUart8VofaJustFloatPidTrace()
        {
            if (!debug_output_.output_enable || debug_output_.output_mode_raw != 2U)
            {
                return;
            }

            const u32 period_ms = (debug_output_.overview_justfloat_period_ms > 0U) ? debug_output_.overview_justfloat_period_ms : 10U;
            if ((time_ms_ - debug_output_.overview_justfloat_last_ms) < period_ms)
            {
                return;
            }

            if (HAL_UART_GetState(&huart8) != HAL_UART_STATE_READY)
            {
                return;
            }

            debug_output_.overview_justfloat_last_ms = time_ms_;
            float payload[33] = {0.0f};
            payload[0] = (f32)time_ms_ * 0.001f;
            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const Motor_Base *steer_motor = wheel.steer_motor_h;
                if (steer_motor == nullptr)
                {
                    continue;
                }

                const f32 target_multi_turn_deg = steer_motor->getTargetTotalAngle();
                const f32 current_multi_turn_deg = steer_motor->getTotalAngle();
                f32 target_single_turn_deg = fmodf(target_multi_turn_deg, 360.0f);
                if (target_single_turn_deg < 0.0f)
                {
                    target_single_turn_deg += 360.0f;
                }

                const u8 base = 1U + i * 8U;
                payload[base + 0U] = steer_motor->getTargetCurrent(); // mA
                payload[base + 1U] = steer_motor->getCurrent();       // mA
                payload[base + 2U] = steer_motor->getTargetRPM();
                payload[base + 3U] = steer_motor->getRPM();
                payload[base + 4U] = target_single_turn_deg;
                payload[base + 5U] = steer_motor->getAngle();
                payload[base + 6U] = target_multi_turn_deg;
                payload[base + 7U] = current_multi_turn_deg;
            }
            debug_uart_.printf_DMA_JustFloat(payload, 33);
        }

        void Chassis::emitUart8VofaPid1kHzTrace()
        {
            if (!debug_output_.output_enable || debug_output_.output_mode_raw != 3U)
            {
                return;
            }

            const u32 period_ms = (debug_output_.single_wheel_1khz_period_ms > 0U) ? debug_output_.single_wheel_1khz_period_ms : 1U;
            if ((time_ms_ - debug_output_.single_wheel_1khz_last_ms) < period_ms)
            {
                return;
            }

            if (HAL_UART_GetState(&huart8) != HAL_UART_STATE_READY)
            {
                return;
            }

            debug_output_.single_wheel_1khz_last_ms = time_ms_;
            const u8 master_wheel_idx = (debug_control_.wheel_index < 4U) ? debug_control_.wheel_index : 0U;
            const u8 override_wheel_idx = (debug_output_.single_wheel_1khz_index < 4U) ? debug_output_.single_wheel_1khz_index : 0U;
            const u8 active_wheel_idx = debug_output_.single_wheel_1khz_use_override_index ? override_wheel_idx : master_wheel_idx;
            const WheelConfig &wheel = wheel_config_[active_wheel_idx];
            const Motor_Base *steer_motor = wheel.steer_motor_h;
            if (steer_motor == nullptr)
            {
                return;
            }

            const f32 target_multi_turn_deg = steer_motor->getTargetTotalAngle();
            f32 target_single_turn_deg = fmodf(target_multi_turn_deg, 360.0f);
            if (target_single_turn_deg < 0.0f)
            {
                target_single_turn_deg += 360.0f;
            }

            float payload[9] = {0.0f};
            payload[0] = (f32)time_ms_ * 0.001f;
            payload[1] = steer_motor->getTargetCurrent(); // mA
            payload[2] = steer_motor->getCurrent();       // mA
            payload[3] = steer_motor->getTargetRPM();
            payload[4] = steer_motor->getRPM();
            payload[5] = target_single_turn_deg;
            payload[6] = steer_motor->getAngle();
            payload[7] = target_multi_turn_deg;
            payload[8] = steer_motor->getTotalAngle();
            debug_uart_.printf_DMA_JustFloat(payload, 9);
        }

        void Chassis::emitUart8VofaDualMotor1kHzTrace()
        {
            if (!debug_output_.output_enable || debug_output_.output_mode_raw != 4U)
            {
                return;
            }

            const u32 period_ms = (debug_output_.single_wheel_dual_motor_period_ms > 0U) ? debug_output_.single_wheel_dual_motor_period_ms : 2U;
            if ((time_ms_ - debug_output_.single_wheel_dual_motor_last_ms) < period_ms)
            {
                return;
            }

            if (HAL_UART_GetState(&huart8) != HAL_UART_STATE_READY)
            {
                return;
            }

            const u8 master_wheel_idx = (debug_control_.wheel_index < 4U) ? debug_control_.wheel_index : 0U;
            const u8 override_wheel_idx = (debug_output_.single_wheel_dual_motor_index < 4U) ? debug_output_.single_wheel_dual_motor_index : 0U;
            const u8 active_wheel_idx = debug_output_.single_wheel_dual_motor_use_override_index ? override_wheel_idx : master_wheel_idx;
            const WheelConfig &wheel = wheel_config_[active_wheel_idx];
            const Motor_Base *steer_motor = wheel.steer_motor_h;
            const Motor_Base *drive_motor = wheel.drive_motor_h;
            if (steer_motor == nullptr || drive_motor == nullptr)
            {
                return;
            }

            debug_output_.single_wheel_dual_motor_last_ms = time_ms_;

            const f32 steer_target_multi_turn_deg = steer_motor->getTargetTotalAngle();
            f32 steer_target_single_turn_deg = fmodf(steer_target_multi_turn_deg, 360.0f);
            if (steer_target_single_turn_deg < 0.0f)
            {
                steer_target_single_turn_deg += 360.0f;
            }

            const f32 drive_target_multi_turn_deg = drive_motor->getTargetTotalAngle();
            f32 drive_target_single_turn_deg = fmodf(drive_target_multi_turn_deg, 360.0f);
            if (drive_target_single_turn_deg < 0.0f)
            {
                drive_target_single_turn_deg += 360.0f;
            }

            float payload[17] = {0.0f};
            payload[0] = (f32)time_ms_ * 0.001f;

            // steer motor: tarI curI tarRPM curRPM tarAng curAng tarTot curTot
            payload[1] = steer_motor->getTargetCurrent();
            payload[2] = steer_motor->getCurrent();
            payload[3] = steer_motor->getTargetRPM();
            payload[4] = steer_motor->getRPM();
            payload[5] = steer_target_single_turn_deg;
            payload[6] = steer_motor->getAngle();
            payload[7] = steer_target_multi_turn_deg;
            payload[8] = steer_motor->getTotalAngle();

            // drive(heading) motor: tarI curI tarRPM curRPM tarAng curAng tarTot curTot
            payload[9] = drive_motor->getTargetCurrent();
            payload[10] = drive_motor->getCurrent();
            payload[11] = drive_motor->getTargetRPM();
            payload[12] = drive_motor->getRPM();
            payload[13] = drive_target_single_turn_deg;
            payload[14] = drive_motor->getAngle();
            payload[15] = drive_target_multi_turn_deg;
            payload[16] = drive_motor->getTotalAngle();

            debug_uart_.printf_DMA_JustFloat(payload, 17);
        }

        void Chassis::emitDebugOutputByMode(bool all_homed)
        {
            if (!debug_output_.output_enable)
            {
                return;
            }
            switch (static_cast<DebugOutputMode>(debug_output_.output_mode_raw))
            {
            case DebugOutputMode::kText:
                emitDebugUart8Log(all_homed);
                break;
            case DebugOutputMode::kOverviewJustFloat:
                emitUart8VofaJustFloatPidTrace();
                break;
            case DebugOutputMode::kSingleWheelJustFloat:
                emitUart8VofaPid1kHzTrace();
                break;
            case DebugOutputMode::kSingleWheelDualMotorJustFloat:
                emitUart8VofaDualMotor1kHzTrace();
                break;
            case DebugOutputMode::kOff:
            default:
                break;
            }
        }

        bool Chassis::applyDebugModuleOverride(bool all_homed)
        {
            if (!debug_control_.enable)
            {
                return false;
            }

            const DebugMode mode = resolveDebugMode(debug_control_.mode_raw);
            if (!(mode == DebugMode::kSingleWheel || mode == DebugMode::kAlignForward || mode == DebugMode::kHomingObserve || mode == DebugMode::kDirectActuator))
            {
                return false;
            }

            const u8 wheel_idx = (debug_control_.wheel_index < 4) ? debug_control_.wheel_index : 0;
            const bool use_soft_steer = (mode == DebugMode::kSingleWheel) && debug_control_.single_wheel_soft_steer_enable;
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                wheel.target_steer_motor_total_angle_rad = wheel.corrected_steer_motor_total_angle_rad;
                wheel.target_drive_omega_rad_s = 0.0f;
                wheel.steer_target_velocity_rad_s = 0.0f;
                wheel.flipped_drive_direction = false;
                planned_data_.steer_angle_oa_rad[i] = wheel.target_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad;
                planned_data_.drive_omega_rad_s[i] = 0.0f;
                if (!(use_soft_steer && i == wheel_idx))
                {
                    last_steer_rate_cmd_rad_s_[i] = 0.0f;
                }
                last_drive_omega_cmd_rad_s_[i] = 0.0f;
            }

            if (mode == DebugMode::kSingleWheel)
            {
                WheelConfig &debug_wheel = wheel_config_[wheel_idx];
                const f32 target_oa_mod_rad = wrapTo2Pi(degToRadF32(debug_control_.single_wheel_target_steer_deg));
                const f32 current_oa_total_rad = debug_wheel.corrected_steer_motor_total_angle_rad + debug_wheel.theta_oa_to_owi_rad;
                const f32 target_oa_total_rad = nearestEquivalentAngle(current_oa_total_rad, target_oa_mod_rad);
                const f32 selected_local_total_rad = target_oa_total_rad - debug_wheel.theta_oa_to_owi_rad;
                const f32 steer_error_deg = radToDegF32(fabsf(shortestAngularDistance(current_oa_total_rad, target_oa_total_rad)));
                const f32 drive_release_error_deg = (debug_control_.single_wheel_drive_release_error_deg >= 0.0f) ? debug_control_.single_wheel_drive_release_error_deg : 0.0f;
                const bool drive_released = !debug_control_.single_wheel_drive_release_gate_enable || (steer_error_deg <= drive_release_error_deg);

                if (debug_control_.single_wheel_soft_steer_enable)
                {
                    const bool enable_rate_limit = actuator_limit_enable_.enable_steer_rate_limit;
                    const bool enable_alpha_limit = actuator_limit_enable_.enable_steer_alpha_limit;
                    f32 steer_limit_rate_rad_s = enable_rate_limit ? max_steer_rate_rad_s_ : 1.0e6f;
                    f32 steer_limit_accel_rad_s2 = enable_alpha_limit ? max_steer_alpha_rad_s2_ : 1.0e8f;
                    if (debug_control_.single_wheel_use_custom_steer_limit)
                    {
                        if (enable_rate_limit)
                        {
                            steer_limit_rate_rad_s = degToRadF32(debug_control_.single_wheel_steer_rate_limit_deg_s);
                        }
                        if (enable_alpha_limit)
                        {
                            steer_limit_accel_rad_s2 = degToRadF32(debug_control_.single_wheel_steer_accel_limit_deg_s2);
                        }
                    }
                    if (enable_rate_limit && steer_limit_rate_rad_s <= 1.0e-6f)
                    {
                        steer_limit_rate_rad_s = max_steer_rate_rad_s_;
                    }
                    if (enable_alpha_limit && steer_limit_accel_rad_s2 <= 1.0e-6f)
                    {
                        steer_limit_accel_rad_s2 = max_steer_alpha_rad_s2_;
                    }

                    f32 next_steer_rate_rad_s = 0.0f;
                    debug_wheel.target_steer_motor_total_angle_rad = limitPositionSecondOrder(
                        debug_wheel.corrected_steer_motor_total_angle_rad,
                        last_steer_rate_cmd_rad_s_[wheel_idx],
                        selected_local_total_rad,
                        steer_limit_rate_rad_s,
                        steer_limit_accel_rad_s2,
                        period_,
                        next_steer_rate_rad_s);
                    debug_wheel.steer_target_velocity_rad_s = next_steer_rate_rad_s;
                    last_steer_rate_cmd_rad_s_[wheel_idx] = next_steer_rate_rad_s;
                }
                else
                {
                    debug_wheel.target_steer_motor_total_angle_rad = selected_local_total_rad;
                    debug_wheel.steer_target_velocity_rad_s = 0.0f;
                    last_steer_rate_cmd_rad_s_[wheel_idx] = 0.0f;
                }

                debug_wheel.target_drive_omega_rad_s = (debug_control_.single_wheel_drive_enable && drive_released) ? rpmToRadsF32(debug_control_.single_wheel_target_drive_rpm) : 0.0f;
                planned_data_.steer_angle_oa_rad[wheel_idx] = debug_wheel.target_steer_motor_total_angle_rad + debug_wheel.theta_oa_to_owi_rad;
                planned_data_.drive_omega_rad_s[wheel_idx] = debug_wheel.target_drive_omega_rad_s;
                last_drive_omega_cmd_rad_s_[wheel_idx] = debug_wheel.target_drive_omega_rad_s;
                debug_mirror_.selected_wheel_steer_error_deg = steer_error_deg;
                debug_mirror_.selected_wheel_drive_released = drive_released;
#if FOURSTEER_SINGLE_WHEEL_TRACE_UART8
                if (debug_output_.output_mode_raw == 1U && debug_output_.text_log_level >= 1U && (time_ms_ - debug_output_.single_wheel_trace_last_ms) >= 50U)
                {
                    debug_output_.single_wheel_trace_last_ms = time_ms_;
                    debug_uart_.printf_DMA((char *)"SW20,%lu,%u,%u,%u,%.3f,%.3f,%.3f,%u,%u\r\n",
                                           (u32)time_ms_,
                                           (u32)wheel_idx,
                                           all_homed ? 1U : 0U,
                                           (u32)input_target_data_.mode,
                                           radToDegF32(target_oa_total_rad),
                                           radToDegF32(current_oa_total_rad),
                                           steer_error_deg,
                                           (u32)debug_wheel.homing_state,
                                           drive_released ? 1U : 0U);
                }
#endif
            }
            else if (mode == DebugMode::kAlignForward)
            {
                for (u8 i = 0; i < 4; ++i)
                {
                    WheelConfig &wheel = wheel_config_[i];
                    wheel.target_steer_motor_total_angle_rad = -wheel.theta_oa_to_owi_rad;
                    planned_data_.steer_angle_oa_rad[i] = 0.0f;
                }
            }
            else if (mode == DebugMode::kDirectActuator)
            {
                const f32 rpm_limit = (debug_control_.direct_drive_rpm_limit > 0.0f) ? debug_control_.direct_drive_rpm_limit : 300.0f;
                const f32 drive_current_limit_mA = (debug_control_.direct_drive_current_limit_mA > 0.0f) ? debug_control_.direct_drive_current_limit_mA : 12000.0f;
                const f32 drive_brake_limit_mA = (debug_control_.direct_drive_brake_limit_mA > 0.0f) ? debug_control_.direct_drive_brake_limit_mA : 12000.0f;
                const f32 steer_rpm_limit = (debug_control_.direct_steer_rpm_limit > 0.0f) ? debug_control_.direct_steer_rpm_limit : 300.0f;
                const f32 steer_current_limit_mA = (debug_control_.direct_steer_current_limit_mA > 0.0f) ? debug_control_.direct_steer_current_limit_mA : 12000.0f;
                const f32 steer_single_turn_limit_deg = (debug_control_.direct_steer_single_turn_limit_deg > 0.0f) ? debug_control_.direct_steer_single_turn_limit_deg : 180.0f;
                const f32 steer_multi_turn_limit_deg = (debug_control_.direct_steer_multi_turn_limit_deg > 0.0f) ? debug_control_.direct_steer_multi_turn_limit_deg : 1080.0f;
                const f32 step_threshold = (debug_control_.direct_step_threshold > 0.01f) ? debug_control_.direct_step_threshold : 0.3f;
                const f32 drive_step_threshold = (debug_control_.direct_step_drive_threshold > 0.01f) ? debug_control_.direct_step_drive_threshold : 0.3f;
                f32 steer_current_cmd_mA = debug_control_.direct_steer_current_mA[wheel_idx];
                f32 steer_rpm_cmd = debug_control_.direct_steer_rpm[wheel_idx];
                f32 steer_single_turn_deg_cmd = debug_control_.direct_steer_single_turn_deg[wheel_idx];
                f32 steer_multi_turn_deg_cmd = debug_control_.direct_steer_multi_turn_deg[wheel_idx];
                f32 drive_rpm_cmd = debug_control_.direct_drive_rpm[wheel_idx];
                f32 drive_current_cmd_mA = debug_control_.direct_drive_current_mA[wheel_idx];
                f32 drive_brake_cmd_mA = debug_control_.direct_drive_brake_mA[wheel_idx];
                f32 applied_steer_cmd = 0.0f;
                f32 applied_drive_cmd = 0.0f;
                const u8 drive_control_type = (debug_control_.direct_drive_control_type <= 2U) ? debug_control_.direct_drive_control_type : 0U;
                f32 steer_axis_value = 0.0f;
                f32 drive_axis_value = 0.0f;
                f32 steer_step_sign = 0.0f;
                f32 drive_step_sign = 0.0f;
                auto clearDriveCommandByType = [&](WheelConfig &wheel_to_clear, u8 wheel_i) {
                    wheel_to_clear.target_drive_omega_rad_s = 0.0f;
                    planned_data_.drive_omega_rad_s[wheel_i] = 0.0f;
                    if (wheel_to_clear.drive_motor_h == nullptr)
                    {
                        return;
                    }
                    if (drive_control_type == 1U)
                    {
                        wheel_to_clear.drive_motor_h->setTargetCurrent(0.0f);
                    }
                    else if (drive_control_type == 2U)
                    {
                        wheel_to_clear.drive_motor_h->setBrake(0.0f);
                    }
                    else
                    {
                        setDriveMotorTargetOmegaRadS(wheel_to_clear, 0.0f);
                    }
                };

                if (debug_control_.direct_input_source == 1U)
                {
                    // 连续遥控输入：左摇杆控制舵向，右摇杆控制航向
                    steer_axis_value = clampValue(airjoy_data_.left_x, -1.0f, 1.0f);
                    drive_axis_value = clampValue(airjoy_data_.right_x, -1.0f, 1.0f);
                    switch (debug_control_.direct_steer_control_type)
                    {
                    case 0U:
                        steer_current_cmd_mA = steer_axis_value * steer_current_limit_mA;
                        break;
                    case 1U:
                        steer_rpm_cmd = steer_axis_value * steer_rpm_limit;
                        break;
                    case 2U:
                        steer_single_turn_deg_cmd = steer_axis_value * steer_single_turn_limit_deg;
                        break;
                    case 3U:
                    default:
                        steer_multi_turn_deg_cmd = steer_axis_value * steer_multi_turn_limit_deg;
                        break;
                    }
                    switch (drive_control_type)
                    {
                    case 1U:
                        drive_current_cmd_mA = drive_axis_value * drive_current_limit_mA;
                        break;
                    case 2U:
                        drive_brake_cmd_mA = drive_axis_value * drive_brake_limit_mA;
                        break;
                    case 0U:
                    default:
                        drive_rpm_cmd = drive_axis_value * rpm_limit;
                        break;
                    }
                }
                else if (debug_control_.direct_input_source == 2U)
                {
                    // 阶跃遥控输入：左摇杆触发舵向阶跃，右摇杆触发航向阶跃
                    if (airjoy_data_.left_x > step_threshold)
                    {
                        steer_step_sign = 1.0f;
                    }
                    else if (airjoy_data_.left_x < -step_threshold)
                    {
                        steer_step_sign = -1.0f;
                    }
                    switch (debug_control_.direct_steer_control_type)
                    {
                    case 0U:
                        steer_current_cmd_mA = steer_step_sign * fabsf(debug_control_.direct_step_steer_current_mA);
                        break;
                    case 1U:
                        steer_rpm_cmd = steer_step_sign * fabsf(debug_control_.direct_step_steer_rpm);
                        break;
                    case 2U:
                        steer_single_turn_deg_cmd = steer_step_sign * fabsf(debug_control_.direct_step_steer_single_turn_deg);
                        break;
                    case 3U:
                    default:
                        steer_multi_turn_deg_cmd = steer_step_sign * fabsf(debug_control_.direct_step_steer_multi_turn_deg);
                        break;
                    }

                    if (airjoy_data_.right_x > drive_step_threshold)
                    {
                        drive_step_sign = 1.0f;
                    }
                    else if (airjoy_data_.right_x < -drive_step_threshold)
                    {
                        drive_step_sign = -1.0f;
                    }
                    switch (drive_control_type)
                    {
                    case 1U:
                        drive_current_cmd_mA = drive_step_sign * fabsf(debug_control_.direct_step_drive_current_mA);
                        break;
                    case 2U:
                        drive_brake_cmd_mA = drive_step_sign * fabsf(debug_control_.direct_step_drive_brake_mA);
                        break;
                    case 0U:
                    default:
                        drive_rpm_cmd = drive_step_sign * fabsf(debug_control_.direct_step_drive_rpm);
                        break;
                    }
                }

                debug_control_.direct_steer_current_mA[wheel_idx] = steer_current_cmd_mA;
                debug_control_.direct_steer_rpm[wheel_idx] = steer_rpm_cmd;
                debug_control_.direct_steer_single_turn_deg[wheel_idx] = steer_single_turn_deg_cmd;
                debug_control_.direct_steer_multi_turn_deg[wheel_idx] = steer_multi_turn_deg_cmd;
                debug_control_.direct_drive_rpm[wheel_idx] = drive_rpm_cmd;
                debug_control_.direct_drive_current_mA[wheel_idx] = drive_current_cmd_mA;
                debug_control_.direct_drive_brake_mA[wheel_idx] = drive_brake_cmd_mA;
                switch (debug_control_.direct_steer_control_type)
                {
                case 0U:
                    applied_steer_cmd = clampValue(steer_current_cmd_mA, -steer_current_limit_mA, steer_current_limit_mA);
                    break;
                case 1U:
                    applied_steer_cmd = clampValue(steer_rpm_cmd, -steer_rpm_limit, steer_rpm_limit);
                    break;
                case 2U:
                    applied_steer_cmd = clampValue(steer_single_turn_deg_cmd, -steer_single_turn_limit_deg, steer_single_turn_limit_deg);
                    break;
                case 3U:
                default:
                    applied_steer_cmd = clampValue(steer_multi_turn_deg_cmd, -steer_multi_turn_limit_deg, steer_multi_turn_limit_deg);
                    break;
                }
                switch (drive_control_type)
                {
                case 1U:
                    applied_drive_cmd = clampValue(drive_current_cmd_mA, -drive_current_limit_mA, drive_current_limit_mA);
                    break;
                case 2U:
                    applied_drive_cmd = clampValue(drive_brake_cmd_mA, -drive_brake_limit_mA, drive_brake_limit_mA);
                    break;
                case 0U:
                default:
                    applied_drive_cmd = clampValue(drive_rpm_cmd, -rpm_limit, rpm_limit);
                    break;
                }

                for (u8 i = 0; i < 4; ++i)
                {
                    WheelConfig &wheel = wheel_config_[i];
                    if (debug_control_.direct_estop)
                    {
                        setSteerMotorTargetCurrent(wheel, 0.0f);
                        clearDriveCommandByType(wheel, i);
                        continue;
                    }

                    if (i != wheel_idx)
                    {
                        setSteerMotorTargetCurrent(wheel, 0.0f);
                        clearDriveCommandByType(wheel, i);
                        continue;
                    }

                    if (debug_control_.direct_enable_steer[i])
                    {
                        const u8 steer_control_type = debug_control_.direct_steer_control_type;
                        if (steer_control_type == 0U)
                        {
                            const f32 target_current_mA = clampValue(steer_current_cmd_mA, -steer_current_limit_mA, steer_current_limit_mA);
                            wheel.target_steer_motor_total_angle_rad = wheel.corrected_steer_motor_total_angle_rad;
                            planned_data_.steer_angle_oa_rad[i] = wheel.corrected_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad;
                            setSteerMotorTargetCurrent(wheel, target_current_mA);
                        }
                        else if (steer_control_type == 1U)
                        {
                            const f32 target_steer_rpm = clampValue(steer_rpm_cmd, -steer_rpm_limit, steer_rpm_limit);
                            wheel.target_steer_motor_total_angle_rad = wheel.corrected_steer_motor_total_angle_rad;
                            planned_data_.steer_angle_oa_rad[i] = wheel.corrected_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad;
                            setSteerMotorTargetRPM(wheel, target_steer_rpm);
                        }
                        else if (steer_control_type == 2U)
                        {
                            const f32 target_single_turn_deg = clampValue(steer_single_turn_deg_cmd, -steer_single_turn_limit_deg, steer_single_turn_limit_deg);
                            const f32 target_local_total_rad = mapSingleTurnToNearestTotalAngle(wheel, target_single_turn_deg);
                            wheel.target_steer_motor_total_angle_rad = target_local_total_rad;
                            planned_data_.steer_angle_oa_rad[i] = target_local_total_rad + wheel.theta_oa_to_owi_rad;
                            setSteerMotorTargetTotalAngleRad(wheel, target_local_total_rad);
                        }
                        else
                        {
                            const f32 target_oa_total_rad = degToRadF32(clampValue(steer_multi_turn_deg_cmd, -steer_multi_turn_limit_deg, steer_multi_turn_limit_deg));
                            const f32 target_local_total_rad = target_oa_total_rad - wheel.theta_oa_to_owi_rad;
                            wheel.target_steer_motor_total_angle_rad = target_local_total_rad;
                            planned_data_.steer_angle_oa_rad[i] = target_oa_total_rad;
                            setSteerMotorTargetTotalAngleRad(wheel, target_local_total_rad);
                        }
                    }
                    else
                    {
                        setSteerMotorTargetCurrent(wheel, 0.0f);
                    }

                    if (debug_control_.direct_enable_drive[i])
                    {
                        if (drive_control_type == 1U)
                        {
                            const f32 target_current_mA = clampValue(drive_current_cmd_mA, -drive_current_limit_mA, drive_current_limit_mA);
                            wheel.target_drive_omega_rad_s = 0.0f;
                            planned_data_.drive_omega_rad_s[i] = 0.0f;
                            if (wheel.drive_motor_h != nullptr)
                            {
                                wheel.drive_motor_h->setTargetCurrent(target_current_mA / wheel.drive_motor_sign);
                            }
                        }
                        else if (drive_control_type == 2U)
                        {
                            const f32 target_brake_mA = clampValue(drive_brake_cmd_mA, -drive_brake_limit_mA, drive_brake_limit_mA);
                            wheel.target_drive_omega_rad_s = 0.0f;
                            planned_data_.drive_omega_rad_s[i] = 0.0f;
                            if (wheel.drive_motor_h != nullptr)
                            {
                                wheel.drive_motor_h->setBrake(target_brake_mA / wheel.drive_motor_sign);
                            }
                        }
                        else
                        {
                            const f32 target_rpm = clampValue(drive_rpm_cmd, -rpm_limit, rpm_limit);
                            const f32 target_omega_rad_s = rpmToRadsF32(target_rpm);
                            wheel.target_drive_omega_rad_s = target_omega_rad_s;
                            planned_data_.drive_omega_rad_s[i] = target_omega_rad_s;
                            setDriveMotorTargetOmegaRadS(wheel, target_omega_rad_s);
                        }
                    }
                    else
                    {
                        clearDriveCommandByType(wheel, i);
                    }
                }

#if FOURSTEER_SINGLE_WHEEL_TRACE_UART8
                if (debug_output_.output_mode_raw == 1U && debug_output_.text_log_level >= 1U && (time_ms_ - debug_output_.direct_trace_last_ms) >= 100U)
                {
                    debug_output_.direct_trace_last_ms = time_ms_;
                    WheelConfig &dbg_wheel = wheel_config_[wheel_idx];
                    const Motor_Base *steer_motor = dbg_wheel.steer_motor_h;
                    const Motor_Base *drive_motor = dbg_wheel.drive_motor_h;
                    if (steer_motor != nullptr)
                    {
                        debug_uart_.printf_DMA((char *)"SW30,t=%lu,w=%u,src=%u,stType=%u,drType=%u,stCmd=%.3f,drCmd=%.3f,stAxis=%.3f,drAxis=%.3f,stStep=%.1f,drStep=%.1f,stTarI=%.1f,stCurI=%.1f,stTarRPM=%.2f,stCurRPM=%.2f,drTarI=%.1f,drCurI=%.1f,drTarRPM=%.2f,drCurRPM=%.2f,enS=%u,enD=%u,estop=%u\r\n",
                                               (u32)time_ms_,
                                               (u32)wheel_idx,
                                               (u32)debug_control_.direct_input_source,
                                               (u32)debug_control_.direct_steer_control_type,
                                               (u32)drive_control_type,
                                               applied_steer_cmd,
                                               applied_drive_cmd,
                                               steer_axis_value,
                                               drive_axis_value,
                                               steer_step_sign,
                                               drive_step_sign,
                                               steer_motor->getTargetCurrent(),
                                               steer_motor->getCurrent(),
                                               steer_motor->getTargetRPM(),
                                               steer_motor->getRPM(),
                                               (drive_motor != nullptr) ? drive_motor->getTargetCurrent() : 0.0f,
                                               (drive_motor != nullptr) ? drive_motor->getCurrent() : 0.0f,
                                               (drive_motor != nullptr) ? drive_motor->getTargetRPM() : 0.0f,
                                               (drive_motor != nullptr) ? drive_motor->getRPM() : 0.0f,
                                               debug_control_.direct_enable_steer[wheel_idx] ? 1U : 0U,
                                               debug_control_.direct_enable_drive[wheel_idx] ? 1U : 0U,
                                               debug_control_.direct_estop ? 1U : 0U);
                    }
                }
#endif
            }

            planned_data_.vel_x = 0.0f;
            planned_data_.vel_y = 0.0f;
            planned_data_.omega_z = 0.0f;
            planned_data_.acc_x = 0.0f;
            planned_data_.acc_y = 0.0f;
            planned_data_.alpha_z = 0.0f;
            planned_data_.rot_z = input_hwt_rot_z_;

            if (mode != DebugMode::kDirectActuator)
            {
                applyModuleCommands(all_homed);
            }

            updateCurrentData(all_homed);
            refreshDebugMirror(all_homed);
            emitDebugOutputByMode(all_homed);
            last_planned_data_ = planned_data_;
            return true;
        }

        bool Chassis::solveLinear3x3(f32 matrix[3][4], f32 &x0, f32 &x1, f32 &x2) const
        {
            // 这里用的是带主元选取的高斯消元，目标是稳定求解 3x3 线性方程组；
            // 输入是增广矩阵，输出是三项未知量，失败通常意味着矩阵接近奇异。
            for (u8 pivot = 0; pivot < 3; ++pivot)
            {
                u8 best_row = pivot;
                f32 best_abs = fabsf(matrix[pivot][pivot]);
                for (u8 row = pivot + 1; row < 3; ++row)
                {
                    const f32 abs_value = fabsf(matrix[row][pivot]);
                    if (abs_value > best_abs)
                    {
                        best_abs = abs_value;
                        best_row = row;
                    }
                }

                if (best_abs <= 1.0e-6f)
                {
                    return false;
                }

                if (best_row != pivot)
                {
                    for (u8 column = pivot; column < 4; ++column)
                    {
                        const f32 temp = matrix[pivot][column];
                        matrix[pivot][column] = matrix[best_row][column];
                        matrix[best_row][column] = temp;
                    }
                }

                const f32 diagonal = matrix[pivot][pivot];
                for (u8 column = pivot; column < 4; ++column)
                {
                    matrix[pivot][column] /= diagonal;
                }

                for (u8 row = 0; row < 3; ++row)
                {
                    if (row == pivot)
                    {
                        continue;
                    }

                    const f32 factor = matrix[row][pivot];
                    if (fabsf(factor) <= 1.0e-8f)
                    {
                        continue;
                    }

                    for (u8 column = pivot; column < 4; ++column)
                    {
                        matrix[row][column] -= factor * matrix[pivot][column];
                    }
                }
            }

            x0 = matrix[0][3];
            x1 = matrix[1][3];
            x2 = matrix[2][3];
            return true;
        }

        bool Chassis::estimateBodySpeedFromModules(f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z) const
        {
            // 由四个模块的“当前舵角 + 当前驱动速度”反推底盘速度。
            // 这里不是直接解单个方程，而是把每个轮子的两个投影约束累积成最小二乘正规方程，
            // 再求解 3 个底盘自由度 [vx, vy, omega_z]。
            f32 normal[3][3] = {};
            f32 rhs[3] = {};

            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const f32 steer_angle_oa_rad = wheel.corrected_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad;
                const f32 cos_theta = cosf(steer_angle_oa_rad);
                const f32 sin_theta = sinf(steer_angle_oa_rad);
                const f32 drive_linear_m_s = wheel.corrected_drive_omega_rad_s * wheel_radius_m_;

                const f32 rows[2][3] = {
                    {cos_theta, sin_theta, -wheel.pos_y_m * cos_theta + wheel.pos_x_m * sin_theta},
                    {-sin_theta, cos_theta, wheel.pos_y_m * sin_theta + wheel.pos_x_m * cos_theta},
                };
                const f32 measurements[2] = {drive_linear_m_s, 0.0f};

                for (u8 row = 0; row < 2; ++row)
                {
                    for (u8 row_i = 0; row_i < 3; ++row_i)
                    {
                        rhs[row_i] += rows[row][row_i] * measurements[row];
                        for (u8 column_i = 0; column_i < 3; ++column_i)
                        {
                            normal[row_i][column_i] += rows[row][row_i] * rows[row][column_i];
                        }
                    }
                }
            }

            f32 augmented[3][4] = {
                {normal[0][0], normal[0][1], normal[0][2], rhs[0]},
                {normal[1][0], normal[1][1], normal[1][2], rhs[1]},
                {normal[2][0], normal[2][1], normal[2][2], rhs[2]},
            };

            if (!solveLinear3x3(augmented, out_vel_x, out_vel_y, out_omega_z))
            {
                out_vel_x = 0.0f;
                out_vel_y = 0.0f;
                out_omega_z = 0.0f;
                return false;
            }
            return true;
        }

        void Chassis::runThread(void *arg)
        {
            (void)arg;
            HWT101CT *hwt = HWT101CT::GetInstance(&huart8);
            time_ms_ = xTaskGetTickCount();

            for (;;)
            {
                const u64 loop_start_us = RtosTimeStampUs64::getTimeUs();

                // 主线程每个周期的执行顺序是固定的：
                // 1) 读取 IMU 航向/角速度
                // 2) 解析模式并做坐标系转换
                // 3) 处理锁航向逻辑与速度限幅
                // 4) 更新轮反馈与回零状态机
                // 5) 生成模块命令、下发电机目标
                // 6) 回写当前估计值并等待下一周期
                input_hwt_rot_z_ = hwt->get_yaw_rad();
                input_hwt_omega_z_ = hwt->get_yaw_speed_rad();

                // 常态同步手柄缓存：即使 debug_control_.enable 关闭，也保持 airjoy_data_ 实时更新，
                // 便于通过调试器直接观察摇杆输入；不改变任何控制模式接管逻辑。
                CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);

                isDebugMode();
                setModeFlag();
                target_data_.rot_z = input_target_data_.rot_z;

                if (current_mode_flag_.is_world_speed_mode)
                {
                    transSpeedWorldToBody(input_target_data_.vel_x, input_target_data_.vel_y, target_data_.vel_x, target_data_.vel_y);
                }
                else
                {
                    target_data_.vel_x = input_target_data_.vel_x;
                    target_data_.vel_y = input_target_data_.vel_y;
                }
                target_data_.omega_z = input_target_data_.omega_z;

                if (input_target_data_.mode == Mode::kSteerAngleAndDriveSpeedMode)
                {
                    target_data_.vel_x = 0.0f;
                    target_data_.vel_y = 0.0f;
                    target_data_.omega_z = 0.0f;
                }

                // 锁当前航向 / 锁到指定航向都在这里对目标 rot_z 和 omega_z 做二次整形，
                // 之后再统一进入速度限幅和规划层限速。
                if (current_mode_flag_.is_lock_now_rot_z)
                {
                    isLockNowRotZ(true, target_data_.rot_z, target_data_.omega_z, target_data_.rot_z, target_data_.omega_z);
                }
                if (current_mode_flag_.is_lock_to_rot_z)
                {
                    isLockToRotZ(true, input_target_data_.rot_z, target_data_.rot_z, target_data_.rot_z, target_data_.omega_z, target_data_.omega_z);
                }

                // 运行时允许调试器直接改基准阈值/限幅开关，这里每周期派生一次，确保全链路口径一致。
                deriveNearZeroThresholds();
                refreshActuatorLimitState();

                clampTargetSpeedInChassis(target_data_.vel_x, target_data_.vel_y, target_data_.omega_z,
                                          target_data_.vel_x, target_data_.vel_y, target_data_.omega_z);

                limitPlannedSpeed(target_data_.vel_x, target_data_.vel_y, target_data_.omega_z,
                                  planned_data_.vel_x, planned_data_.vel_y, planned_data_.omega_z);

                planned_data_.acc_x = (planned_data_.vel_x - last_planned_data_.vel_x) / period_;
                planned_data_.acc_y = (planned_data_.vel_y - last_planned_data_.vel_y) / period_;
                planned_data_.alpha_z = (planned_data_.omega_z - last_planned_data_.omega_z) / period_;
                planned_data_.rot_z = target_data_.rot_z;

                updateWheelFeedback();
                applyDebugSteerPidRuntimeTuning();

                bool all_homed = true;
                for (u8 i = 0; i < 4; ++i)
                {
                    if (!updateHomingState(wheel_config_[i]))
                    {
                        all_homed = false;
                    }
                }
                homing_start_request_ = false;

                if (applyDebugModuleOverride(all_homed))
                {
                    updateTaskPerfStat(loop_start_us, RtosTimeStampUs64::getTimeUs());
                    vTaskDelayUntil(&time_ms_, period_ms_);
                    continue;
                }

                // 回零和正常控制共用同一套命令生成流程，但最终下发前会根据 all_homed 选择：
                // 未回零时只保留安全动作，已回零时才输出完整舵角/驱动目标。
                computeModuleCommands(planned_data_);
                applyModuleCommands(all_homed);
                updateCurrentData(all_homed);
                refreshDebugMirror(all_homed);
                emitDebugOutputByMode(all_homed);

                last_planned_data_ = planned_data_;
                updateTaskPerfStat(loop_start_us, RtosTimeStampUs64::getTimeUs());
                vTaskDelayUntil(&time_ms_, period_ms_);
            }
        }

        void Chassis::updateTaskPerfStat(u64 loop_start_us, u64 loop_end_us)
        {
            if (loop_start_us == 0ULL || loop_end_us == 0ULL || loop_end_us < loop_start_us)
            {
                return;
            }

            TaskPerfStat &perf = task_perf_stat_;
            const u64 exec_cost_us = loop_end_us - loop_start_us;
            perf.budget_us = static_cast<u32>(period_ms_) * 1000U;

            perf.last_start_us = loop_start_us;
            perf.last_end_us = loop_end_us;
            perf.last_exec_us = exec_cost_us;

            if (perf.loop_count == 0ULL)
            {
                perf.min_exec_us = exec_cost_us;
                perf.max_exec_us = exec_cost_us;
                perf.loop_count = 1ULL;
            }
            else
            {
                if (exec_cost_us < perf.min_exec_us)
                {
                    perf.min_exec_us = exec_cost_us;
                }
                if (exec_cost_us > perf.max_exec_us)
                {
                    perf.max_exec_us = exec_cost_us;
                }

                perf.loop_count += 1ULL;
            }

            if (exec_cost_us > static_cast<u64>(perf.budget_us))
            {
                perf.overrun_count += 1ULL;
            }

            u16 sample_us = 0xFFFFU;
            if (exec_cost_us > 0xFFFFULL)
            {
                perf.window.clamp_count += 1ULL;
            }
            else
            {
                sample_us = static_cast<u16>(exec_cost_us);
            }

            if (perf.window.count < 500U)
            {
                perf.window.samples_us[perf.window.index] = sample_us;
                perf.window.sum_us += sample_us;
                perf.window.count = static_cast<u16>(perf.window.count + 1U);
            }
            else
            {
                const u16 old_sample = perf.window.samples_us[perf.window.index];
                perf.window.sum_us -= old_sample;
                perf.window.samples_us[perf.window.index] = sample_us;
                perf.window.sum_us += sample_us;
            }

            perf.window.index = static_cast<u16>((perf.window.index + 1U) % 500U);

            if (perf.window.count > 0U)
            {
                perf.avg_exec_us = static_cast<u64>(perf.window.sum_us / perf.window.count);
            }
            else
            {
                perf.avg_exec_us = 0ULL;
            }

            perf.window_size = 500U;
            perf.window_count = perf.window.count;
            perf.window_clamp_count = perf.window.clamp_count;
        }

        f32 Chassis::getTargetBodyVelX() const
        {
            return target_data_.vel_x;
        }

        f32 Chassis::getTargetBodyVelY() const
        {
            return target_data_.vel_y;
        }

        f32 Chassis::getTargetWorldVelX() const
        {
            f32 world_x = 0.0f;
            f32 world_y = 0.0f;
            transSpeedBodyToWorld(target_data_.vel_x, target_data_.vel_y, world_x, world_y);
            return world_x;
        }

        f32 Chassis::getTargetWorldVelY() const
        {
            f32 world_x = 0.0f;
            f32 world_y = 0.0f;
            transSpeedBodyToWorld(target_data_.vel_x, target_data_.vel_y, world_x, world_y);
            return world_y;
        }

        f32 Chassis::getTargetOmegaZ() const
        {
            return target_data_.omega_z;
        }

        f32 Chassis::getCurrentBodyVelX() const
        {
            return current_data_.vel_x;
        }

        f32 Chassis::getCurrentBodyVelY() const
        {
            return current_data_.vel_y;
        }

        f32 Chassis::getCurrentWorldVelX() const
        {
            f32 world_x = 0.0f;
            f32 world_y = 0.0f;
            transSpeedBodyToWorld(current_data_.vel_x, current_data_.vel_y, world_x, world_y);
            return world_x;
        }

        f32 Chassis::getCurrentWorldVelY() const
        {
            f32 world_x = 0.0f;
            f32 world_y = 0.0f;
            transSpeedBodyToWorld(current_data_.vel_x, current_data_.vel_y, world_x, world_y);
            return world_y;
        }

        f32 Chassis::getCurrentOmegaZ() const
        {
            return current_data_.omega_z;
        }
    }
}
