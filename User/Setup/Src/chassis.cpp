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
        namespace
        {
            constexpr u16 kSwerveTelemetryMagic = 0xA55AU;
            constexpr u8 kSwerveTelemetryVersion = 2U;
            constexpr u8 kSwerveTelemetryFlagsCrcPayloadOnly = 1U << 0;
            constexpr u8 kSwerveTelemetryFlagsAllHomed = 1U << 1;
            constexpr u16 kSwerveTelemetryMsgTypeMode5 = 0x001FU;
            constexpr u16 kSwerveTelemetryFrameHeaderBytes = 18U; // magic(2)+ver(1)+flags(1)+seq(2)+ts(8)+msg_type(2)+payload_len(2)
            constexpr u16 kSwerveTelemetryFrameCrcBytes = 2U;
            constexpr u8 kSwerveTelemetryWheelCount = 4U;
            constexpr u8 kSwerveTelemetryChassisFloatCount = 8U; // chassis target4f + chassis actual4f
            constexpr u8 kSwerveTelemetryWheelFloatCountPerWheel = 8U; // per wheel(M0..M3): t_drive,a_drive,t_steer,a_steer,t_wvx,t_wvy,a_wvx,a_wvy
            constexpr u16 kSwerveTelemetryPayloadFloatCount = static_cast<u16>(kSwerveTelemetryChassisFloatCount +
                                                                                (kSwerveTelemetryWheelFloatCountPerWheel * kSwerveTelemetryWheelCount));
            constexpr u16 kSwerveTelemetryPayloadBytes = static_cast<u16>(kSwerveTelemetryPayloadFloatCount * sizeof(f32));
            static_assert(kSwerveTelemetryPayloadBytes == 160U, "mode5 V2 payload must be 160 bytes");
            constexpr u16 kSwerveTelemetryFrameBytes = static_cast<u16>(kSwerveTelemetryFrameHeaderBytes + kSwerveTelemetryPayloadBytes + kSwerveTelemetryFrameCrcBytes);
            constexpr u16 kSwerveTelemetryTxBufferBytes = 416U;
            static_assert(kSwerveTelemetryFrameBytes <= kSwerveTelemetryTxBufferBytes, "mode5 telemetry frame buffer too small");
            alignas(4) static u8 swerve_telemetry_tx_frame_buf[kSwerveTelemetryTxBufferBytes] = {0U};

            inline void packU16LE(u8 *dst, u16 value)
            {
                dst[0] = static_cast<u8>(value & 0xFFU);
                dst[1] = static_cast<u8>((value >> 8) & 0xFFU);
            }

            inline void packU64LE(u8 *dst, u64 value)
            {
                for (u8 i = 0U; i < 8U; ++i)
                {
                    dst[i] = static_cast<u8>((value >> (8U * i)) & 0xFFU);
                }
            }

            inline void packF32LE(u8 *dst, f32 value)
            {
                union
                {
                    f32 f;
                    u32 u;
                } conv;
                conv.f = value;
                dst[0] = static_cast<u8>(conv.u & 0xFFU);
                dst[1] = static_cast<u8>((conv.u >> 8) & 0xFFU);
                dst[2] = static_cast<u8>((conv.u >> 16) & 0xFFU);
                dst[3] = static_cast<u8>((conv.u >> 24) & 0xFFU);
            }

            u16 crc16CcittFalse(const u8 *data, u16 len)
            {
                u16 crc = 0xFFFFU;
                for (u16 i = 0U; i < len; ++i)
                {
                    crc ^= static_cast<u16>(data[i]) << 8;
                    for (u8 bit = 0U; bit < 8U; ++bit)
                    {
                        if ((crc & 0x8000U) != 0U)
                        {
                            crc = static_cast<u16>((crc << 1) ^ 0x1021U);
                        }
                        else
                        {
                            crc = static_cast<u16>(crc << 1);
                        }
                    }
                }
                return crc;
            }
        } // namespace

        void Chassis::init(InitConfig &config)
        {
            runtime_strategy_cfg_ = default_strategy_cfg_;
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
// 真正的搜索、沿边沿捕获零位、偏置生效和完成判定都在runThread中按周期推进
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
            runtime_strategy_cfg_.idle_posture_mode = mode;
        }

        void Chassis::setSteeringStrategyMode(SteeringStrategyMode mode)
        {
            runtime_strategy_cfg_.steering_strategy_mode = mode;
        }

        f32 Chassis::getNearZeroEnterSpeedMps() const
        {
            return (runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s >= 0.0f) ? runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s : 0.0f;
        }

        f32 Chassis::getNearZeroExitSpeedMps() const
        {
            const f32 enter = getNearZeroEnterSpeedMps();
            const f32 exit_raw = (runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s >= 0.0f) ? runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s : 0.0f;
            return (exit_raw > enter) ? exit_raw : (enter + 1.0e-3f);
        }

        f32 Chassis::getStopGuardReleaseSpeedMps() const
        {
            return getNearZeroExitSpeedMps();
        }

        void Chassis::refreshActuatorLimitState()
        {
// 预留钩子：当前执行器限幅开关直接从就近布置的 enable_* 成员读取，无需额外派生状态
        }

        f32 Chassis::mapSingleTurnToNearestTotalAngle(const WheelConfig &wheel, f32 target_oa_single_turn_deg) const
        {
            const f32 target_oa_mod_rad = wrapTo2Pi(degToRadF32(target_oa_single_turn_deg));
            const SteerCalibration calibration{
                wheel.theta_oa_to_owi_rad,
                wheel.homing_runtime_zero_offset_rad,
                wheel.steer_motor_sign,
                wheel.drive_motor_sign,
            };
            const f32 current_oa_total_rad = mapCorrectedLocalTotalToOaTotal(wheel.corrected_steer_motor_total_angle_rad, calibration);
            const f32 target_oa_total_rad = nearestEquivalentAngle(current_oa_total_rad, target_oa_mod_rad);
            return mapOaTotalToCorrectedLocalTotal(target_oa_total_rad, calibration);
        }

        void Chassis::computeProjectedDriveFromPlannedSteer(const Data &command_data, const f32 planned_oa_total_rad[4], f32 out_drive_omega_rad_s[4]) const
        {
            const f32 safe_wheel_radius = (runtime_strategy_cfg_.wheel_radius_m_ > 1.0e-6f) ? runtime_strategy_cfg_.wheel_radius_m_ : 1.0e-6f;
            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const f32 wheel_vx = command_data.vel_x + command_data.omega_z * wheel.pos_y_m;
                const f32 wheel_vy = command_data.vel_y - command_data.omega_z * wheel.pos_x_m;
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
                const f32 drive_linear_m_s = planned_drive_omega_rad_s[i] * runtime_strategy_cfg_.wheel_radius_m_;

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

        f32 Chassis::computeMaxCommandWheelSpeedMps(const Data &command_data) const
        {
            f32 max_command_wheel_speed_m_s = 0.0f;
            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const f32 wheel_vx = command_data.vel_x + command_data.omega_z * wheel.pos_y_m;
                const f32 wheel_vy = command_data.vel_y - command_data.omega_z * wheel.pos_x_m;
                const f32 wheel_speed_m_s = magnitude2D(wheel_vx, wheel_vy);
                max_command_wheel_speed_m_s =
                    (wheel_speed_m_s > max_command_wheel_speed_m_s) ? wheel_speed_m_s : max_command_wheel_speed_m_s;
            }
            return max_command_wheel_speed_m_s;
        }

        bool Chassis::shouldActivateLaunchHold() const
        {
            if (runtime_strategy_cfg_.drive_attenuation_mode != DriveAttenuationMode::kHardGate)
            {
                return false;
            }

            if (runtime_strategy_cfg_.hard_gate.scope != DriveGateScope::kGlobal)
            {
                return false;
            }

            if (runtime_strategy_cfg_.hard_gate.min_scale > 1.0e-6f)
            {
                return false;
            }

            const f32 command_speed_m_s = computeMaxCommandWheelSpeedMps(target_data_);
            return xpark_gate_active_ && (command_speed_m_s > getNearZeroExitSpeedMps());
        }

        bool Chassis::isLaunchHoldAligned(const SwervePlannerOutput &planner_output) const
        {
            const f32 close_rad = degToRadF32(runtime_strategy_cfg_.hard_gate.close_angle_deg);
            for (u8 i = 0; i < 4; ++i)
            {
                if (planner_output.steering_errors_rad[i] >= close_rad)
                {
                    return false;
                }
            }
            return true;
        }

        Chassis::Data Chassis::makeLaunchHoldPreviewCommand() const
        {
            Data preview = target_data_;
            clampTargetSpeedInChassis(preview.vel_x, preview.vel_y, preview.omega_z,
                                      preview.vel_x, preview.vel_y, preview.omega_z);
            preview.acc_x = 0.0f;
            preview.acc_y = 0.0f;
            preview.alpha_z = 0.0f;
            preview.rot_z = target_data_.rot_z;
            return preview;
        }

        f32 Chassis::computeDriveGateScale(f32 abs_error_rad) const
        {
            switch (runtime_strategy_cfg_.drive_attenuation_mode)
            {
            case DriveAttenuationMode::kHardGate:
            {
                const f32 close_rad = degToRadF32(runtime_strategy_cfg_.hard_gate.close_angle_deg);
                const f32 min_scale = clampValue(runtime_strategy_cfg_.hard_gate.min_scale, 0.0f, 1.0f);
                return (abs_error_rad >= close_rad) ? min_scale : 1.0f;
            }
            case DriveAttenuationMode::kSoftGate:
            {
                const f32 close_rad = degToRadF32(runtime_strategy_cfg_.soft_gate.close_angle_deg);
                const f32 min_scale = clampValue(runtime_strategy_cfg_.soft_gate.min_scale, 0.0f, 1.0f);
                if (close_rad <= 1.0e-6f)
                {
                    return min_scale;
                }
                const f32 ratio = clampValue(abs_error_rad / close_rad, 0.0f, 1.0f);
                return 1.0f - (1.0f - min_scale) * ratio;
            }
            case DriveAttenuationMode::kContinuousCurve:
            {
                const f32 half_rad = degToRadF32(runtime_strategy_cfg_.continuous_curve_gate.half_angle_deg);
                const f32 exponent = (runtime_strategy_cfg_.continuous_curve_gate.exponent > 0.1f) ? runtime_strategy_cfg_.continuous_curve_gate.exponent : 2.0f;
                const f32 curve_min = clampValue(runtime_strategy_cfg_.continuous_curve_gate.min_scale, 0.0f, 1.0f);
                if (half_rad <= 1.0e-6f)
                {
                    return curve_min;
                }
                const f32 norm = abs_error_rad / half_rad;
                const f32 scale = 1.0f / (1.0f + powf(norm, exponent));
                return clampValue(curve_min + (1.0f - curve_min) * scale, curve_min, 1.0f);
            }
            case DriveAttenuationMode::kAdaptiveGate:
            {
                const f32 close_rad = degToRadF32(runtime_strategy_cfg_.adaptive_gate.close_angle_deg);
                const f32 min_scale = clampValue(runtime_strategy_cfg_.adaptive_gate.min_scale, 0.0f, 1.0f);
                return (abs_error_rad >= close_rad) ? min_scale : 1.0f;
            }
            default:
                return 1.0f;
            }
        }

        void Chassis::computeDriveGateScales(const SwervePlannerInput &planner_input, const f32 steering_errors_rad[4], f32 out_scales[4])
        {
            for (u8 i = 0; i < 4; ++i)
            {
                out_scales[i] = 1.0f;
            }

            max_residual_speed_m_s_ = planner_input.max_residual_speed_m_s;
            hard_gate_bypassed_by_residual_speed_ = false;

            const bool uses_gate_shape =
                runtime_strategy_cfg_.drive_attenuation_mode == DriveAttenuationMode::kHardGate ||
                runtime_strategy_cfg_.drive_attenuation_mode == DriveAttenuationMode::kSoftGate ||
                runtime_strategy_cfg_.drive_attenuation_mode == DriveAttenuationMode::kContinuousCurve ||
                runtime_strategy_cfg_.drive_attenuation_mode == DriveAttenuationMode::kAdaptiveGate;

            if (!uses_gate_shape)
            {
                adaptive_gate_scale_ = 1.0f;
                adaptive_gate_phase_ = AdaptiveGatePhase::kDisabled;
                return;
            }

            if (runtime_strategy_cfg_.drive_attenuation_mode == DriveAttenuationMode::kAdaptiveGate)
            {
                const Data &command_data = planner_input.command;
                const f32 linear_speed = magnitude2D(command_data.vel_x, command_data.vel_y);
                const f32 angular_speed = fabsf(command_data.omega_z);
                const bool in_transition = (linear_speed >= runtime_strategy_cfg_.adaptive_gate.transition_linear_speed_m_s) ||
                                           (angular_speed >= runtime_strategy_cfg_.adaptive_gate.transition_angular_speed_rad_s);
                const f32 ramp_up = (runtime_strategy_cfg_.adaptive_gate.scale_ramp_up_s > 1.0e-6f) ? runtime_strategy_cfg_.adaptive_gate.scale_ramp_up_s : 0.10f;
                const f32 ramp_down = (runtime_strategy_cfg_.adaptive_gate.scale_ramp_down_s > 1.0e-6f) ? runtime_strategy_cfg_.adaptive_gate.scale_ramp_down_s : 0.06f;
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

            if (runtime_strategy_cfg_.drive_attenuation_mode == DriveAttenuationMode::kHardGate)
            {
                const f32 hard_gate_disable_residual_speed_m_s =
                    (runtime_strategy_cfg_.hard_gate.disable_residual_speed_m_s >= 0.0f)
                        ? runtime_strategy_cfg_.hard_gate.disable_residual_speed_m_s
                        : 0.0f;
                if (planner_input.max_residual_speed_m_s > hard_gate_disable_residual_speed_m_s)
                {
                    hard_gate_bypassed_by_residual_speed_ = true;
                    return;
                }
            }

            DriveGateScope gate_scope = DriveGateScope::kGlobal;
            switch (runtime_strategy_cfg_.drive_attenuation_mode)
            {
            case DriveAttenuationMode::kHardGate:
                gate_scope = runtime_strategy_cfg_.hard_gate.scope;
                break;
            case DriveAttenuationMode::kSoftGate:
                gate_scope = runtime_strategy_cfg_.soft_gate.scope;
                break;
            case DriveAttenuationMode::kContinuousCurve:
                gate_scope = runtime_strategy_cfg_.continuous_curve_gate.scope;
                break;
            case DriveAttenuationMode::kAdaptiveGate:
                gate_scope = runtime_strategy_cfg_.adaptive_gate.scope;
                break;
            default:
                gate_scope = DriveGateScope::kGlobal;
                break;
            }

            if (gate_scope == DriveGateScope::kGlobal)
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
                if (runtime_strategy_cfg_.drive_attenuation_mode == DriveAttenuationMode::kAdaptiveGate)
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
                if (runtime_strategy_cfg_.drive_attenuation_mode == DriveAttenuationMode::kAdaptiveGate)
                {
                    scale *= adaptive_gate_scale_;
                }
                out_scales[i] = clampValue(scale, 0.0f, 1.0f);
            }
        }

        f32 Chassis::stopSteerGuardBlend(f32 residual_speed_m_s) const
        {
            const f32 release_speed = getStopGuardReleaseSpeedMps();
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

        Chassis::SwervePlannerInput Chassis::makeSwervePlannerInput(const Data &command_data)
        {
            SwervePlannerInput planner_input{};
            planner_input.command = command_data;

            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                const f32 current_corrected_local_total_rad = wheel.corrected_steer_motor_total_angle_rad;
                planner_input.current_oa_total_rad[i] = mapWheelCorrectedLocalToOaTotal(wheel, current_corrected_local_total_rad);

                const f32 wheel_vx = command_data.vel_x + command_data.omega_z * wheel.pos_y_m;
                const f32 wheel_vy = command_data.vel_y - command_data.omega_z * wheel.pos_x_m;
                const f32 wheel_speed_m_s = magnitude2D(wheel_vx, wheel_vy);
                planner_input.wheel_vx_m_s[i] = wheel_vx;
                planner_input.wheel_vy_m_s[i] = wheel_vy;
                planner_input.wheel_speed_m_s[i] = wheel_speed_m_s;
                planner_input.max_command_wheel_speed_m_s =
                    (wheel_speed_m_s > planner_input.max_command_wheel_speed_m_s) ? wheel_speed_m_s : planner_input.max_command_wheel_speed_m_s;

                const f32 residual_speed_m_s = fabsf(wheel.corrected_drive_omega_rad_s) * runtime_strategy_cfg_.wheel_radius_m_;
                planner_input.residual_speed_m_s[i] = residual_speed_m_s;
                planner_input.max_residual_speed_m_s =
                    (residual_speed_m_s > planner_input.max_residual_speed_m_s) ? residual_speed_m_s : planner_input.max_residual_speed_m_s;
            }

            const f32 xpark_enter_speed = getNearZeroEnterSpeedMps();
            const f32 xpark_exit_speed = getNearZeroExitSpeedMps();
            const bool command_stationary_intent = xpark_gate_active_
                                                       ? (planner_input.max_command_wheel_speed_m_s <= xpark_exit_speed)
                                                       : (planner_input.max_command_wheel_speed_m_s <= xpark_enter_speed);
            const bool residual_stationary_intent = xpark_gate_active_
                                                        ? (planner_input.max_residual_speed_m_s <= xpark_exit_speed)
                                                        : (planner_input.max_residual_speed_m_s <= xpark_enter_speed);
            planner_input.command_stationary_intent = command_stationary_intent && residual_stationary_intent;

            if (planner_input.command_stationary_intent)
            {
                xpark_stationary_hold_ms_ = (xpark_stationary_hold_ms_ > (0xFFFFFFFFU - period_ms_))
                                                ? 0xFFFFFFFFU
                                                : (xpark_stationary_hold_ms_ + period_ms_);
                if (xpark_stationary_hold_ms_ >= runtime_strategy_cfg_.xpark_entry_delay_ms)
                {
                    xpark_gate_active_ = true;
                }
            }
            else
            {
                xpark_stationary_hold_ms_ = 0U;
                xpark_gate_active_ = false;
            }

            planner_input.residual_drive_is_moving = planner_input.max_residual_speed_m_s > getStopGuardReleaseSpeedMps();
            planner_input.allow_xpark_pose = planner_input.command_stationary_intent && xpark_gate_active_;
            planner_input.force_uniform_steer_drive = (input_target_data_.mode == Mode::kSteerAngleAndDriveSpeedMode);
            planner_input.uniform_steer_oa_mod_rad = wrapTo2Pi(degToRadF32(input_target_data_.steer_lock_angle_deg));
            planner_input.uniform_drive_omega_abs = fabsf(input_target_data_.drive_lock_speed_m_s) / runtime_strategy_cfg_.wheel_radius_m_;
            planner_input.uniform_drive_sign = (input_target_data_.drive_lock_speed_m_s >= 0.0f) ? 1.0f : -1.0f;
            return planner_input;
        }

        Chassis::SwervePlannerOutput Chassis::planSwerveModules(const SwervePlannerInput &planner_input)
        {
            SwervePlannerOutput planner_output{};

            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const f32 wheel_speed_m_s = planner_input.wheel_speed_m_s[i];
                const bool is_stationary = wheel_speed_m_s <= getNearZeroEnterSpeedMps();

                if (is_stationary)
                {
                    planner_output.ideal_oa_total_rad[i] = (planner_input.allow_xpark_pose && runtime_strategy_cfg_.idle_posture_mode == IdlePostureMode::kXPark)
                                                               ? wrapTo2Pi(getXParkAngle(wheel))
                                                               : wrapTo2Pi(planner_input.current_oa_total_rad[i]);
                    planner_output.ideal_drive_omega_rad_s[i] = 0.0f;
                }
                else
                {
                    planner_output.ideal_oa_total_rad[i] = wrapTo2Pi(atan2f(planner_input.wheel_vy_m_s[i], planner_input.wheel_vx_m_s[i]));
                    planner_output.ideal_drive_omega_rad_s[i] = wheel_speed_m_s / runtime_strategy_cfg_.wheel_radius_m_;
                }

                const f32 alt_target_oa_mod_rad = wrapTo2Pi(planner_output.ideal_oa_total_rad[i] + kPi);
                const f32 candidate_a = nearestEquivalentAngle(planner_input.current_oa_total_rad[i], planner_output.ideal_oa_total_rad[i]);
                const f32 candidate_b = nearestEquivalentAngle(planner_input.current_oa_total_rad[i], alt_target_oa_mod_rad);

                f32 selected_oa_total_rad = candidate_a;
                f32 selected_drive_omega_rad_s = planner_output.ideal_drive_omega_rad_s[i];
                bool flipped = false;

                if (!is_stationary)
                {
                    if (runtime_strategy_cfg_.steering_strategy_mode == SteeringStrategyMode::kAlwaysForward)
                    {
                        flipped = false;
                    }
                    else
                    {
                        const f32 base_abs_deg = radToDegF32(fabsf(candidate_a - planner_input.current_oa_total_rad[i]));
                        const f32 flip_abs_deg = radToDegF32(fabsf(candidate_b - planner_input.current_oa_total_rad[i]));
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
                    selected_oa_total_rad = candidate_b;
                    selected_drive_omega_rad_s = -selected_drive_omega_rad_s;
                }

                if (planner_input.force_uniform_steer_drive)
                {
                    const f32 fixed_a = nearestEquivalentAngle(planner_input.current_oa_total_rad[i], planner_input.uniform_steer_oa_mod_rad);
                    const f32 fixed_b = nearestEquivalentAngle(planner_input.current_oa_total_rad[i], wrapTo2Pi(planner_input.uniform_steer_oa_mod_rad + kPi));
                    const bool use_b = fabsf(shortestAngularDistance(planner_input.current_oa_total_rad[i], fixed_b)) <
                                       fabsf(shortestAngularDistance(planner_input.current_oa_total_rad[i], fixed_a));
                    selected_oa_total_rad = use_b ? fixed_b : fixed_a;
                    selected_drive_omega_rad_s = planner_input.uniform_drive_sign * (use_b ? -1.0f : 1.0f) * planner_input.uniform_drive_omega_abs;
                    flipped = use_b;
                }

                planner_output.selected_oa_total_rad[i] = selected_oa_total_rad;
                planner_output.flipped_drive_direction[i] = flipped;
                planner_output.steering_errors_rad[i] =
                    fabsf(shortestAngularDistance(planner_input.current_oa_total_rad[i], selected_oa_total_rad));
                planner_output.projected_drive_omega_rad_s[i] = selected_drive_omega_rad_s;
            }

            if (!planner_input.force_uniform_steer_drive &&
                runtime_strategy_cfg_.enable_stop_steer_guard &&
                planner_input.command_stationary_intent &&
                planner_input.residual_drive_is_moving)
            {
                for (u8 i = 0; i < 4; ++i)
                {
                    const f32 blend = stopSteerGuardBlend(planner_input.residual_speed_m_s[i]);
                    if (blend >= 1.0f)
                    {
                        continue;
                    }

                    const f32 current_oa_total_rad = planner_input.current_oa_total_rad[i];
                    const f32 protected_target_oa_total_rad =
                        wrapTo2Pi(current_oa_total_rad +
                                  shortestAngularDistance(current_oa_total_rad, planner_output.selected_oa_total_rad[i]) * blend);
                    planner_output.selected_oa_total_rad[i] = protected_target_oa_total_rad;
                    planner_output.steering_errors_rad[i] =
                        fabsf(shortestAngularDistance(current_oa_total_rad, protected_target_oa_total_rad));
                    planner_output.flipped_drive_direction[i] = false;
                }
            }

            const f32 steer_rate_floor = 1.0e-3f;
            const f32 steer_rate_limit_runtime = runtime_strategy_cfg_.enable_steer_rate_limit_ ? runtime_strategy_cfg_.max_steer_rate_rad_s_ : 1.0e6f;
            const f32 steer_alpha_limit_runtime = runtime_strategy_cfg_.enable_steer_alpha_limit_ ? runtime_strategy_cfg_.max_steer_alpha_rad_s2_ : 1.0e8f;
            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const f32 current_corrected_local_total_rad = wheel.corrected_steer_motor_total_angle_rad;
                const f32 selected_corrected_local_total_rad =
                    mapWheelOaTotalToCorrectedLocal(wheel, planner_output.selected_oa_total_rad[i]);

                f32 next_steer_rate_rad_s = 0.0f;
                planner_output.planned_corrected_local_total_rad[i] = limitPositionSecondOrder(
                    current_corrected_local_total_rad,
                    last_steer_rate_cmd_rad_s_[i],
                    selected_corrected_local_total_rad,
                    steer_rate_limit_runtime,
                    steer_alpha_limit_runtime,
                    period_,
                    next_steer_rate_rad_s);
                planner_output.planned_steer_rate_rad_s[i] = next_steer_rate_rad_s;
                planner_output.planned_oa_total_rad[i] =
                    mapWheelCorrectedLocalToOaTotal(wheel, planner_output.planned_corrected_local_total_rad[i]);

                f32 steer_rate_ref = fabsf(next_steer_rate_rad_s);
                if (steer_rate_ref < steer_rate_floor)
                {
                    steer_rate_ref = (steer_rate_limit_runtime > steer_rate_floor) ? steer_rate_limit_runtime : steer_rate_floor;
                }
                const f32 eta_s = planner_output.steering_errors_rad[i] / steer_rate_ref;
                planner_output.vector_eta_max_s = (eta_s > planner_output.vector_eta_max_s) ? eta_s : planner_output.vector_eta_max_s;
            }

            for (u8 i = 0; i < 4; ++i)
            {
                planner_output.projected_drive_omega_rad_s[i] = planner_output.projected_drive_omega_rad_s[i];
            }
            if (!planner_input.force_uniform_steer_drive)
            {
                computeProjectedDriveFromPlannedSteer(planner_input.command,
                                                     planner_output.planned_oa_total_rad,
                                                     planner_output.projected_drive_omega_rad_s);
            }

            f32 gate_scales[4] = {1.0f, 1.0f, 1.0f, 1.0f};
            computeDriveGateScales(planner_input, planner_output.steering_errors_rad, gate_scales);

            const f32 translational_speed_m_s = magnitude2D(planner_input.command.vel_x, planner_input.command.vel_y);
            f32 predicted_vel_x = 0.0f;
            f32 predicted_vel_y = 0.0f;
            f32 predicted_omega_z = 0.0f;
            if (estimatePlannedBodyTwist(planner_output.planned_oa_total_rad,
                                         planner_output.projected_drive_omega_rad_s,
                                         predicted_vel_x,
                                         predicted_vel_y,
                                         predicted_omega_z))
            {
                const f32 predicted_trans_speed_m_s = magnitude2D(predicted_vel_x, predicted_vel_y);
                if ((translational_speed_m_s > 1.0e-6f) && (predicted_trans_speed_m_s > 1.0e-6f))
                {
                    const f32 target_dir_rad = atan2f(planner_input.command.vel_y, planner_input.command.vel_x);
                    const f32 predicted_dir_rad = atan2f(predicted_vel_y, predicted_vel_x);
                    planner_output.vector_dir_err_deg =
                        radToDegF32(fabsf(shortestAngularDistance(target_dir_rad, predicted_dir_rad)));
                }
            }

            planner_output.vector_gate_scale = planner_input.force_uniform_steer_drive
                                                   ? 1.0f
                                                   : updateVectorConsistencyGate(translational_speed_m_s,
                                                                                 planner_output.vector_eta_max_s,
                                                                                 planner_output.vector_dir_err_deg);
            if (planner_input.force_uniform_steer_drive)
            {
                vector_gate_scale_ = 1.0f;
                vector_gate_active_ = false;
            }
            planner_output.vector_gate_active = vector_gate_active_;

            f32 planner_drive_targets_rad_s[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            f32 max_abs_planner_drive_target_rad_s = 0.0f;
            for (u8 i = 0; i < 4; ++i)
            {
                if (planner_input.force_uniform_steer_drive)
                {
                    planner_output.gate_or_cos_scale[i] = 1.0f;
                }
                else if (runtime_strategy_cfg_.drive_attenuation_mode == DriveAttenuationMode::kHardGate ||
                         runtime_strategy_cfg_.drive_attenuation_mode == DriveAttenuationMode::kSoftGate ||
                         runtime_strategy_cfg_.drive_attenuation_mode == DriveAttenuationMode::kContinuousCurve ||
                         runtime_strategy_cfg_.drive_attenuation_mode == DriveAttenuationMode::kAdaptiveGate)
                {
                    planner_output.gate_or_cos_scale[i] = gate_scales[i];
                }
                else if (runtime_strategy_cfg_.drive_attenuation_mode == DriveAttenuationMode::kCosine)
                {
                    f32 cos_scale = cosf(planner_output.steering_errors_rad[i]);
                    if (cos_scale < 0.0f)
                    {
                        cos_scale = 0.0f;
                    }
                    planner_output.gate_or_cos_scale[i] = cos_scale;
                }
                else
                {
                    planner_output.gate_or_cos_scale[i] = 1.0f;
                }

                const f32 drive_scale =
                    clampValue(planner_output.gate_or_cos_scale[i] * planner_output.vector_gate_scale, 0.0f, 1.0f);
                planner_drive_targets_rad_s[i] = planner_output.projected_drive_omega_rad_s[i] * drive_scale;
                const f32 abs_target_drive = fabsf(planner_drive_targets_rad_s[i]);
                max_abs_planner_drive_target_rad_s =
                    (abs_target_drive > max_abs_planner_drive_target_rad_s) ? abs_target_drive : max_abs_planner_drive_target_rad_s;
            }

            f32 planner_drive_uniform_scale = 1.0f;
            if (runtime_strategy_cfg_.enable_drive_omega_limit_)
            {
                const f32 drive_omega_limit_rad_s = fabsf(runtime_strategy_cfg_.max_drive_omega_rad_s_);
                if ((drive_omega_limit_rad_s > 1.0e-6f) && (max_abs_planner_drive_target_rad_s > drive_omega_limit_rad_s))
                {
                    planner_drive_uniform_scale = drive_omega_limit_rad_s / max_abs_planner_drive_target_rad_s;
                }
            }

            for (u8 i = 0; i < 4; ++i)
            {
                planner_output.final_drive_omega_rad_s[i] = planner_drive_targets_rad_s[i] * planner_drive_uniform_scale;
            }

            return planner_output;
        }

        void Chassis::buildActuatorCommandFrame(const SwervePlannerOutput &planner_output, ActuatorCommandFrame &out_frame) const
        {
            for (u8 i = 0; i < 4; ++i)
            {
                out_frame.steer_corrected_local_total_rad[i] = planner_output.planned_corrected_local_total_rad[i];
                out_frame.steer_oa_total_rad[i] = planner_output.planned_oa_total_rad[i];
                out_frame.steer_rate_rad_s[i] = planner_output.planned_steer_rate_rad_s[i];
                out_frame.drive_omega_rad_s[i] = planner_output.final_drive_omega_rad_s[i];
                out_frame.flipped_drive_direction[i] = planner_output.flipped_drive_direction[i];
            }
        }

        void Chassis::storePlannedActuatorFrame(const SwervePlannerOutput &planner_output, const ActuatorCommandFrame &command_frame)
        {
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                drive_gate_scale_[i] =
                    clampValue(planner_output.gate_or_cos_scale[i] * planner_output.vector_gate_scale, 0.0f, 1.0f);
                wheel.target_steer_motor_total_angle_rad = command_frame.steer_corrected_local_total_rad[i];
                wheel.target_drive_omega_rad_s = command_frame.drive_omega_rad_s[i];
                wheel.steer_target_velocity_rad_s = command_frame.steer_rate_rad_s[i];
                wheel.flipped_drive_direction = command_frame.flipped_drive_direction[i];

                last_steer_rate_cmd_rad_s_[i] = command_frame.steer_rate_rad_s[i];
                selected_flipped_solution_[i] = command_frame.flipped_drive_direction[i];
                planned_data_.steer_angle_oa_rad[i] = command_frame.steer_oa_total_rad[i];
                planned_data_.drive_omega_rad_s[i] = command_frame.drive_omega_rad_s[i];
            }

            planner_output_cache_ = planner_output;
            actuator_command_frame_ = command_frame;
            vector_eta_max_s_ = planner_output.vector_eta_max_s;
            vector_dir_err_deg_ = planner_output.vector_dir_err_deg;
            vector_gate_scale_ = planner_output.vector_gate_scale;
            vector_gate_active_ = planner_output.vector_gate_active;
        }

        f32 Chassis::computeHomingAlignTargetCorrectedLocalTotal(const WheelConfig &wheel) const
        {
            const f32 current_corrected_local_total_rad = wheel.corrected_steer_motor_total_angle_rad;
            const f32 current_oa_total_rad = mapWheelCorrectedLocalToOaTotal(wheel, current_corrected_local_total_rad);
            const f32 align_target_oa_total_rad = nearestEquivalentAngle(current_oa_total_rad, 0.0f);
            return mapWheelOaTotalToCorrectedLocal(wheel, align_target_oa_total_rad);
        }

        void Chassis::setModeFlag()
        {
            // 将外部模式压缩成线程内使用的少量布尔标志，后续执行顺序只看这些标志，
// 这样可以把“世界系/车体系”“定向锁跟随当前角”“空转模式”解耦开
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

        void Chassis::applyDebugTargetOverride(DebugMode mode)
        {
            // 手柄平移坐标 -> 车体坐标约定：前推前进、左推左移。
            // 为匹配遥杆实际符号：left_y 正向映射到 +X；left_x 取反后映射到 +Y。
            f32 target_vel_x = airjoy_data_.left_y * runtime_strategy_cfg_.max_vel_x_;
            f32 target_vel_y = -airjoy_data_.left_x * runtime_strategy_cfg_.max_vel_y_;
            const f32 right_x_cmd = -airjoy_data_.right_x;
            f32 target_omega_z = -right_x_cmd * runtime_strategy_cfg_.max_omega_z_;

            if (debug_control_.inject_sine)
            {
                target_omega_z = sineWaveGeneratorF32(time_ms_ / 1000.0f, debug_control_.sine_amplitude, debug_control_.sine_frequency, 0.0f, debug_control_.sine_offset);
            }
            else if (debug_control_.inject_step)
            {
                if (airjoy_data_.right_x > 0.3f)
                {
                    target_omega_z = runtime_strategy_cfg_.max_omega_z_;
                }
                else if (airjoy_data_.right_x < -0.3f)
                {
                    target_omega_z = -runtime_strategy_cfg_.max_omega_z_;
                }
                else
                {
                    target_omega_z = 0.0f;
                }
            }

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

        void Chassis::clearPlannedMotionForModuleOverride()
        {
            target_data_.vel_x = 0.0f;
            target_data_.vel_y = 0.0f;
            target_data_.omega_z = 0.0f;
            planned_data_.vel_x = 0.0f;
            planned_data_.vel_y = 0.0f;
            planned_data_.omega_z = 0.0f;
            planned_data_.acc_x = 0.0f;
            planned_data_.acc_y = 0.0f;
            planned_data_.alpha_z = 0.0f;
        }

        void Chassis::resetDebugModuleOverrideTargets(u8 wheel_idx, bool preserve_soft_wheel_rate)
        {
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                wheel.target_steer_motor_total_angle_rad = wheel.corrected_steer_motor_total_angle_rad;
                wheel.target_drive_omega_rad_s = 0.0f;
                wheel.steer_target_velocity_rad_s = 0.0f;
                wheel.flipped_drive_direction = false;
                planned_data_.steer_angle_oa_rad[i] = mapWheelCorrectedLocalToOaTotal(wheel, wheel.target_steer_motor_total_angle_rad);
                planned_data_.drive_omega_rad_s[i] = 0.0f;
                if (!(preserve_soft_wheel_rate && i == wheel_idx))
                {
                    last_steer_rate_cmd_rad_s_[i] = 0.0f;
                }
                last_drive_omega_cmd_rad_s_[i] = 0.0f;
            }
        }

        void Chassis::applySingleWheelDebugOverride(u8 wheel_idx, bool all_homed)
        {
            WheelConfig &debug_wheel = wheel_config_[wheel_idx];
            const f32 target_oa_mod_rad = wrapTo2Pi(degToRadF32(debug_control_.single_wheel_target_steer_deg));
            const f32 current_oa_total_rad = mapWheelCorrectedLocalToOaTotal(debug_wheel, debug_wheel.corrected_steer_motor_total_angle_rad);
            const f32 target_oa_total_rad = nearestEquivalentAngle(current_oa_total_rad, target_oa_mod_rad);
            const f32 selected_local_total_rad = mapWheelOaTotalToCorrectedLocal(debug_wheel, target_oa_total_rad);
            const f32 steer_error_deg = radToDegF32(fabsf(shortestAngularDistance(current_oa_total_rad, target_oa_total_rad)));
            const f32 drive_release_error_deg = (debug_control_.single_wheel_drive_release_error_deg >= 0.0f) ? debug_control_.single_wheel_drive_release_error_deg : 0.0f;
            const bool drive_released = !debug_control_.single_wheel_drive_release_gate_enable || (steer_error_deg <= drive_release_error_deg);

            if (debug_control_.single_wheel_soft_steer_enable)
            {
                const bool enable_rate_limit = runtime_strategy_cfg_.enable_steer_rate_limit_;
                const bool enable_alpha_limit = runtime_strategy_cfg_.enable_steer_alpha_limit_;
                f32 steer_limit_rate_rad_s = enable_rate_limit ? runtime_strategy_cfg_.max_steer_rate_rad_s_ : 1.0e6f;
                f32 steer_limit_accel_rad_s2 = enable_alpha_limit ? runtime_strategy_cfg_.max_steer_alpha_rad_s2_ : 1.0e8f;
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
                    steer_limit_rate_rad_s = runtime_strategy_cfg_.max_steer_rate_rad_s_;
                }
                if (enable_alpha_limit && steer_limit_accel_rad_s2 <= 1.0e-6f)
                {
                    steer_limit_accel_rad_s2 = runtime_strategy_cfg_.max_steer_alpha_rad_s2_;
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
            planned_data_.steer_angle_oa_rad[wheel_idx] = mapWheelCorrectedLocalToOaTotal(debug_wheel, debug_wheel.target_steer_motor_total_angle_rad);
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

        void Chassis::applyAlignForwardDebugOverride()
        {
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                wheel.target_steer_motor_total_angle_rad = mapWheelOaTotalToCorrectedLocal(wheel, 0.0f);
                planned_data_.steer_angle_oa_rad[i] = 0.0f;
            }
        }

        void Chassis::applyHomingObserveDebugOverride()
        {
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                wheel.target_steer_motor_total_angle_rad = wheel.corrected_steer_motor_total_angle_rad;
                wheel.target_drive_omega_rad_s = 0.0f;
                wheel.steer_target_velocity_rad_s = 0.0f;
                planned_data_.steer_angle_oa_rad[i] = mapWheelCorrectedLocalToOaTotal(wheel, wheel.corrected_steer_motor_total_angle_rad);
                planned_data_.drive_omega_rad_s[i] = 0.0f;
                last_steer_rate_cmd_rad_s_[i] = 0.0f;
                last_drive_omega_cmd_rad_s_[i] = 0.0f;
            }
        }

        Chassis::DirectActuatorCommandSnapshot Chassis::resolveDirectActuatorCommand(u8 wheel_idx)
        {
            DirectActuatorCommandSnapshot command{};
            command.wheel_idx = wheel_idx;
            command.steer_control_type = debug_control_.direct_steer_control_type;
            command.drive_control_type = (debug_control_.direct_drive_control_type <= 2U) ? debug_control_.direct_drive_control_type : 0U;

            const f32 rpm_limit = (debug_control_.direct_drive_rpm_limit > 0.0f) ? debug_control_.direct_drive_rpm_limit : 300.0f;
            const f32 drive_current_limit_mA = (debug_control_.direct_drive_current_limit_mA > 0.0f) ? debug_control_.direct_drive_current_limit_mA : 12000.0f;
            const f32 drive_brake_limit_mA = (debug_control_.direct_drive_brake_limit_mA > 0.0f) ? debug_control_.direct_drive_brake_limit_mA : 12000.0f;
            const f32 steer_rpm_limit = (debug_control_.direct_steer_rpm_limit > 0.0f) ? debug_control_.direct_steer_rpm_limit : 300.0f;
            const f32 steer_current_limit_mA = (debug_control_.direct_steer_current_limit_mA > 0.0f) ? debug_control_.direct_steer_current_limit_mA : 12000.0f;
            const f32 steer_single_turn_limit_deg = (debug_control_.direct_steer_single_turn_limit_deg > 0.0f) ? debug_control_.direct_steer_single_turn_limit_deg : 180.0f;
            const f32 steer_multi_turn_limit_deg = (debug_control_.direct_steer_multi_turn_limit_deg > 0.0f) ? debug_control_.direct_steer_multi_turn_limit_deg : 1080.0f;
            const f32 step_threshold = (debug_control_.direct_step_threshold > 0.01f) ? debug_control_.direct_step_threshold : 0.3f;
            const f32 drive_step_threshold = (debug_control_.direct_step_drive_threshold > 0.01f) ? debug_control_.direct_step_drive_threshold : 0.3f;

            command.steer_current_cmd_mA = debug_control_.direct_steer_current_mA[wheel_idx];
            command.steer_rpm_cmd = debug_control_.direct_steer_rpm[wheel_idx];
            command.steer_single_turn_deg_cmd = debug_control_.direct_steer_single_turn_deg[wheel_idx];
            command.steer_multi_turn_deg_cmd = debug_control_.direct_steer_multi_turn_deg[wheel_idx];
            command.drive_rpm_cmd = debug_control_.direct_drive_rpm[wheel_idx];
            command.drive_current_cmd_mA = debug_control_.direct_drive_current_mA[wheel_idx];
            command.drive_brake_cmd_mA = debug_control_.direct_drive_brake_mA[wheel_idx];

            if (debug_control_.direct_input_source == 1U)
            {
                command.steer_axis_value = clampValue(airjoy_data_.left_x, -1.0f, 1.0f);
                command.drive_axis_value = clampValue(airjoy_data_.right_x, -1.0f, 1.0f);
                switch (command.steer_control_type)
                {
                case 0U:
                    command.steer_current_cmd_mA = command.steer_axis_value * steer_current_limit_mA;
                    break;
                case 1U:
                    command.steer_rpm_cmd = command.steer_axis_value * steer_rpm_limit;
                    break;
                case 2U:
                    command.steer_single_turn_deg_cmd = command.steer_axis_value * steer_single_turn_limit_deg;
                    break;
                case 3U:
                default:
                    command.steer_multi_turn_deg_cmd = command.steer_axis_value * steer_multi_turn_limit_deg;
                    break;
                }
                switch (command.drive_control_type)
                {
                case 1U:
                    command.drive_current_cmd_mA = command.drive_axis_value * drive_current_limit_mA;
                    break;
                case 2U:
                    command.drive_brake_cmd_mA = command.drive_axis_value * drive_brake_limit_mA;
                    break;
                case 0U:
                default:
                    command.drive_rpm_cmd = command.drive_axis_value * rpm_limit;
                    break;
                }
            }
            else if (debug_control_.direct_input_source == 2U)
            {
                if (airjoy_data_.left_x > step_threshold)
                {
                    command.steer_step_sign = 1.0f;
                }
                else if (airjoy_data_.left_x < -step_threshold)
                {
                    command.steer_step_sign = -1.0f;
                }
                switch (command.steer_control_type)
                {
                case 0U:
                    command.steer_current_cmd_mA = command.steer_step_sign * fabsf(debug_control_.direct_step_steer_current_mA);
                    break;
                case 1U:
                    command.steer_rpm_cmd = command.steer_step_sign * fabsf(debug_control_.direct_step_steer_rpm);
                    break;
                case 2U:
                    command.steer_single_turn_deg_cmd = command.steer_step_sign * fabsf(debug_control_.direct_step_steer_single_turn_deg);
                    break;
                case 3U:
                default:
                    command.steer_multi_turn_deg_cmd = command.steer_step_sign * fabsf(debug_control_.direct_step_steer_multi_turn_deg);
                    break;
                }

                if (airjoy_data_.right_x > drive_step_threshold)
                {
                    command.drive_step_sign = 1.0f;
                }
                else if (airjoy_data_.right_x < -drive_step_threshold)
                {
                    command.drive_step_sign = -1.0f;
                }
                switch (command.drive_control_type)
                {
                case 1U:
                    command.drive_current_cmd_mA = command.drive_step_sign * fabsf(debug_control_.direct_step_drive_current_mA);
                    break;
                case 2U:
                    command.drive_brake_cmd_mA = command.drive_step_sign * fabsf(debug_control_.direct_step_drive_brake_mA);
                    break;
                case 0U:
                default:
                    command.drive_rpm_cmd = command.drive_step_sign * fabsf(debug_control_.direct_step_drive_rpm);
                    break;
                }
            }

            debug_control_.direct_steer_current_mA[wheel_idx] = command.steer_current_cmd_mA;
            debug_control_.direct_steer_rpm[wheel_idx] = command.steer_rpm_cmd;
            debug_control_.direct_steer_single_turn_deg[wheel_idx] = command.steer_single_turn_deg_cmd;
            debug_control_.direct_steer_multi_turn_deg[wheel_idx] = command.steer_multi_turn_deg_cmd;
            debug_control_.direct_drive_rpm[wheel_idx] = command.drive_rpm_cmd;
            debug_control_.direct_drive_current_mA[wheel_idx] = command.drive_current_cmd_mA;
            debug_control_.direct_drive_brake_mA[wheel_idx] = command.drive_brake_cmd_mA;

            switch (command.steer_control_type)
            {
            case 0U:
                command.applied_steer_cmd = clampValue(command.steer_current_cmd_mA, -steer_current_limit_mA, steer_current_limit_mA);
                break;
            case 1U:
                command.applied_steer_cmd = clampValue(command.steer_rpm_cmd, -steer_rpm_limit, steer_rpm_limit);
                break;
            case 2U:
                command.applied_steer_cmd = clampValue(command.steer_single_turn_deg_cmd, -steer_single_turn_limit_deg, steer_single_turn_limit_deg);
                break;
            case 3U:
            default:
                command.applied_steer_cmd = clampValue(command.steer_multi_turn_deg_cmd, -steer_multi_turn_limit_deg, steer_multi_turn_limit_deg);
                break;
            }

            switch (command.drive_control_type)
            {
            case 1U:
                command.applied_drive_cmd = clampValue(command.drive_current_cmd_mA, -drive_current_limit_mA, drive_current_limit_mA);
                break;
            case 2U:
                command.applied_drive_cmd = clampValue(command.drive_brake_cmd_mA, -drive_brake_limit_mA, drive_brake_limit_mA);
                break;
            case 0U:
            default:
                command.applied_drive_cmd = clampValue(command.drive_rpm_cmd, -rpm_limit, rpm_limit);
                break;
            }

            return command;
        }

        void Chassis::clearDirectDriveCommandByType(WheelConfig &wheel, u8 wheel_idx, u8 drive_control_type)
        {
            wheel.target_drive_omega_rad_s = 0.0f;
            planned_data_.drive_omega_rad_s[wheel_idx] = 0.0f;
            if (wheel.drive_motor_h == nullptr)
            {
                return;
            }
            if (drive_control_type == 1U)
            {
                wheel.drive_motor_h->setTargetCurrent(0.0f);
            }
            else if (drive_control_type == 2U)
            {
                wheel.drive_motor_h->setBrake(0.0f);
            }
            else
            {
                setDriveMotorTargetOmegaRadS(wheel, 0.0f);
            }
        }

        void Chassis::applyDirectActuatorSteerCommand(WheelConfig &wheel, u8 wheel_idx, const DirectActuatorCommandSnapshot &command)
        {
            const f32 steer_current_limit_mA = (debug_control_.direct_steer_current_limit_mA > 0.0f) ? debug_control_.direct_steer_current_limit_mA : 12000.0f;
            const f32 steer_rpm_limit = (debug_control_.direct_steer_rpm_limit > 0.0f) ? debug_control_.direct_steer_rpm_limit : 300.0f;
            const f32 steer_single_turn_limit_deg = (debug_control_.direct_steer_single_turn_limit_deg > 0.0f) ? debug_control_.direct_steer_single_turn_limit_deg : 180.0f;
            const f32 steer_multi_turn_limit_deg = (debug_control_.direct_steer_multi_turn_limit_deg > 0.0f) ? debug_control_.direct_steer_multi_turn_limit_deg : 1080.0f;

            if (!debug_control_.direct_enable_steer[wheel_idx])
            {
                setSteerMotorTargetCurrent(wheel, 0.0f);
                return;
            }

            if (command.steer_control_type == 0U)
            {
                const f32 target_current_mA = clampValue(command.steer_current_cmd_mA, -steer_current_limit_mA, steer_current_limit_mA);
                wheel.target_steer_motor_total_angle_rad = wheel.corrected_steer_motor_total_angle_rad;
                planned_data_.steer_angle_oa_rad[wheel_idx] = mapWheelCorrectedLocalToOaTotal(wheel, wheel.corrected_steer_motor_total_angle_rad);
                setSteerMotorTargetCurrent(wheel, target_current_mA);
            }
            else if (command.steer_control_type == 1U)
            {
                const f32 target_steer_rpm = clampValue(command.steer_rpm_cmd, -steer_rpm_limit, steer_rpm_limit);
                wheel.target_steer_motor_total_angle_rad = wheel.corrected_steer_motor_total_angle_rad;
                planned_data_.steer_angle_oa_rad[wheel_idx] = mapWheelCorrectedLocalToOaTotal(wheel, wheel.corrected_steer_motor_total_angle_rad);
                setSteerMotorTargetRPM(wheel, target_steer_rpm);
            }
            else if (command.steer_control_type == 2U)
            {
                const f32 target_single_turn_deg = clampValue(command.steer_single_turn_deg_cmd, -steer_single_turn_limit_deg, steer_single_turn_limit_deg);
                const f32 target_local_total_rad = mapSingleTurnToNearestTotalAngle(wheel, target_single_turn_deg);
                wheel.target_steer_motor_total_angle_rad = target_local_total_rad;
                planned_data_.steer_angle_oa_rad[wheel_idx] = mapWheelCorrectedLocalToOaTotal(wheel, target_local_total_rad);
                setSteerMotorTargetTotalAngleRad(wheel, target_local_total_rad);
            }
            else
            {
                const f32 target_oa_total_rad = degToRadF32(clampValue(command.steer_multi_turn_deg_cmd, -steer_multi_turn_limit_deg, steer_multi_turn_limit_deg));
                const f32 target_local_total_rad = mapWheelOaTotalToCorrectedLocal(wheel, target_oa_total_rad);
                wheel.target_steer_motor_total_angle_rad = target_local_total_rad;
                planned_data_.steer_angle_oa_rad[wheel_idx] = target_oa_total_rad;
                setSteerMotorTargetTotalAngleRad(wheel, target_local_total_rad);
            }
        }

        void Chassis::applyDirectActuatorDriveCommand(WheelConfig &wheel, u8 wheel_idx, const DirectActuatorCommandSnapshot &command)
        {
            const f32 rpm_limit = (debug_control_.direct_drive_rpm_limit > 0.0f) ? debug_control_.direct_drive_rpm_limit : 300.0f;
            const f32 drive_current_limit_mA = (debug_control_.direct_drive_current_limit_mA > 0.0f) ? debug_control_.direct_drive_current_limit_mA : 12000.0f;
            const f32 drive_brake_limit_mA = (debug_control_.direct_drive_brake_limit_mA > 0.0f) ? debug_control_.direct_drive_brake_limit_mA : 12000.0f;

            if (!debug_control_.direct_enable_drive[wheel_idx])
            {
                clearDirectDriveCommandByType(wheel, wheel_idx, command.drive_control_type);
                return;
            }

            if (command.drive_control_type == 1U)
            {
                const f32 target_current_mA = clampValue(command.drive_current_cmd_mA, -drive_current_limit_mA, drive_current_limit_mA);
                wheel.target_drive_omega_rad_s = 0.0f;
                planned_data_.drive_omega_rad_s[wheel_idx] = 0.0f;
                if (wheel.drive_motor_h != nullptr)
                {
                    wheel.drive_motor_h->setTargetCurrent(mapWheelCurrentToDriveMotorCurrent(target_current_mA, makeSteerCalibration(wheel)));
                }
            }
            else if (command.drive_control_type == 2U)
            {
                const f32 target_brake_mA = clampValue(command.drive_brake_cmd_mA, -drive_brake_limit_mA, drive_brake_limit_mA);
                wheel.target_drive_omega_rad_s = 0.0f;
                planned_data_.drive_omega_rad_s[wheel_idx] = 0.0f;
                if (wheel.drive_motor_h != nullptr)
                {
                    wheel.drive_motor_h->setBrake(mapWheelCurrentToDriveMotorCurrent(target_brake_mA, makeSteerCalibration(wheel)));
                }
            }
            else
            {
                const f32 target_rpm = clampValue(command.drive_rpm_cmd, -rpm_limit, rpm_limit);
                const f32 target_omega_rad_s = rpmToRadsF32(target_rpm);
                wheel.target_drive_omega_rad_s = target_omega_rad_s;
                planned_data_.drive_omega_rad_s[wheel_idx] = target_omega_rad_s;
                setDriveMotorTargetOmegaRadS(wheel, target_omega_rad_s);
            }
        }

        void Chassis::applyDirectActuatorDebugOverride(u8 wheel_idx)
        {
            const DirectActuatorCommandSnapshot command = resolveDirectActuatorCommand(wheel_idx);

            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                if (debug_control_.direct_estop || i != wheel_idx)
                {
                    setSteerMotorTargetCurrent(wheel, 0.0f);
                    clearDirectDriveCommandByType(wheel, i, command.drive_control_type);
                    continue;
                }

                applyDirectActuatorSteerCommand(wheel, i, command);
                applyDirectActuatorDriveCommand(wheel, i, command);
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
                                           (u32)command.steer_control_type,
                                           (u32)command.drive_control_type,
                                           command.applied_steer_cmd,
                                           command.applied_drive_cmd,
                                           command.steer_axis_value,
                                           command.drive_axis_value,
                                           command.steer_step_sign,
                                           command.drive_step_sign,
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

        void Chassis::finalizeDebugModuleOverride(bool all_homed, DebugModuleOverrideRoute route)
        {
            planned_data_.vel_x = 0.0f;
            planned_data_.vel_y = 0.0f;
            planned_data_.omega_z = 0.0f;
            planned_data_.acc_x = 0.0f;
            planned_data_.acc_y = 0.0f;
            planned_data_.alpha_z = 0.0f;
            planned_data_.rot_z = input_hwt_rot_z_;

            if (route != DebugModuleOverrideRoute::kDirectActuator)
            {
                applyModuleCommands(all_homed);
            }

            updateCurrentData(all_homed);
            refreshDebugMirror(all_homed);
            emitDebugOutputByMode(all_homed);
            last_planned_data_ = planned_data_;
        }

        void Chassis::isDebugMode()
        {
            syncDebugSteerPidTuneFromRuntimeOnEnableEdge();
            const DebugControlRoute route = classifyDebugControlRoute(debug_control_.enable, debug_control_.mode_raw);
            if (route == DebugControlRoute::kDisabled)
            {
                debug_control_.mode_resolved_raw = static_cast<u8>(DebugMode::kTorqueFree);
                return;
            }

            const DebugMode mode = resolveDebugMode(debug_control_.mode_raw);
            debug_control_.mode_resolved_raw = static_cast<u8>(mode);

            if (route == DebugControlRoute::kTargetInjection)
            {
                applyDebugTargetOverride(mode);
                return;
            }

            setTargetBodySpeedMode(0.0f, 0.0f, 0.0f);
            clearPlannedMotionForModuleOverride();
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
            // 抓取当前机体朝向，再在后续由 PID 产生角速度闭环，让机器人保持当下姿态
// 就把最近一次真实机体朝向当作要维持rot_z，再由姿PID生成out_omega_z来稳住该朝向
// 因此它不是“始终锁某个固定角”，而是“手动旋转”和“松手后自动锁住当前角”之间的平滑切换器
        void Chassis::isLockNowRotZ(bool is_lock, f32 rot_z, f32 omega_z, f32 &out_rot_z, f32 &out_omega_z)
        {
                // 1. out_rot_z 直接跟随 IMU 当前朝向 input_hwt_rot_z_，把目标角锁在此刻真实姿态上；
            if (!is_lock)
            {
                out_rot_z = rot_z;
                out_omega_z = omega_z;
                return;
            }

// “锁当前航向”不是简单地rot_z固定住，而是先在用户开始施加角速度
// 抓取当前机体朝向，再在后续由PID产生角速度闭环，让机器人保持当下姿态
            if (omega_z == 0.0f)
            {
                    // 过渡缓冲结束后，真正用于锁角的目标已经不是外部传入的 rot_z，
                // 但在刚松开摇杆的最初一小段时间内，不立即让 PID 介入，而是先进入过渡缓冲：
                    // 后续由 rot_z_pid_ 根据“目标朝向 lock_now_rot_z_target_”和“当前真实朝向 input_hwt_rot_z_”
                    // 的误差生成维持姿态所需的 out_omega_z
// 3.lock_now_rot_z_shift_count_作为缓冲计数器，倒数结束后才真正进入锁角闭环
                if (lock_now_rot_z_shift_count_ > 0)
                {
                    lock_now_rot_z_shift_count_--;
                    lock_now_rot_z_target_ = input_hwt_rot_z_;
                    out_rot_z = lock_now_rot_z_target_;
                    out_omega_z = 0.0f;
                }
                else
                {
// 过渡缓冲结束后，真正用于锁角的目标已经不是外部传入的rot_z
// 而是前面已经抓取并保存下来的lock_now_rot_z_target_
                    // rot_z_pid_count_ / rot_z_pid_period_ 共同控制姿态 PID 的实际计算节拍
// 的误差生成维持姿态所需out_omega_z
                    out_rot_z = lock_now_rot_z_target_;
                    if (rot_z_pid_count_ >= rot_z_pid_period_)
                    {
                        rot_z_pid_count_ = 0;
                        out_omega_z = rot_z_pid_.pid_calc(radToDegF32(lock_now_rot_z_target_), radToDegF32(input_hwt_rot_z_));
                    }
                    else
                    {
                // 3. 每次有手动旋转输入都重置缓冲计数器，为后续从手动旋转切回自动锁角预留平滑过渡窗口
// 暂时沿用上一规划周期planned_data_.omega_z，减少输出抖动并维持角速度连续性
                        out_omega_z = planned_data_.omega_z;
                    }
                    // rot_z_pid_count_ / rot_z_pid_period_ 共同控制姿态 PID 的实际计算节拍。
                    rot_z_pid_count_++;
                }
            }
            else
            {
                // 这里表示“用户仍在主动要求旋转”：
// 1.不进入锁角闭环，直接执行当前手动omega_z
// 2.同时out_rot_z刷新成当IMU朝向input_hwt_rot_z_
                //    相当于不断更新“等会儿松手后要锁住的那个角”；
// 3.每次有手动旋转输入都重置缓冲计数器，为后续从手动旋转切回自动锁角预留平滑过渡窗口
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

// “锁到指定航向”会先限制目标角速度变化率，再用姿PID生成维持/逼近该目标角度所需omega_z
// 这样外层给出的目标角不会瞬间跳变，底盘转向更平滑
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
            out_vel_x = clampValue(vel_x, -runtime_strategy_cfg_.max_vel_x_, runtime_strategy_cfg_.max_vel_x_);
            out_vel_y = clampValue(vel_y, -runtime_strategy_cfg_.max_vel_y_, runtime_strategy_cfg_.max_vel_y_);
            out_omega_z = clampValue(omega_z, -runtime_strategy_cfg_.max_omega_z_, runtime_strategy_cfg_.max_omega_z_);
        }

        void Chassis::resolvePlannerTargetData()
        {
            const PlannerInputCommand planner_command{
                input_target_data_.vel_x,
                input_target_data_.vel_y,
                input_target_data_.omega_z,
                input_target_data_.rot_z,
                current_mode_flag_.is_world_speed_mode,
                input_target_data_.mode == Mode::kSteerAngleAndDriveSpeedMode,
            };
            const CommandInputSource source =
                (classifyDebugControlRoute(debug_control_.enable, debug_control_.mode_raw) == DebugControlRoute::kTargetInjection)
                    ? CommandInputSource::kDebugTarget
                    : CommandInputSource::kApi;
            normalized_body_command_ = makeNormalizedBodyCommand(planner_command, input_hwt_rot_z_, source);
            target_data_.vel_x = normalized_body_command_.body.vel_x;
            target_data_.vel_y = normalized_body_command_.body.vel_y;
            target_data_.omega_z = normalized_body_command_.body.omega_z;
            target_data_.rot_z = normalized_body_command_.rot_z;

            if (current_mode_flag_.is_lock_now_rot_z)
            {
                isLockNowRotZ(true, target_data_.rot_z, target_data_.omega_z, target_data_.rot_z, target_data_.omega_z);
            }
            if (current_mode_flag_.is_lock_to_rot_z)
            {
                isLockToRotZ(true, input_target_data_.rot_z, target_data_.rot_z, target_data_.rot_z, target_data_.omega_z, target_data_.omega_z);
            }
        }

        void Chassis::updatePlannedMotionData()
        {
            if (!launch_hold_active_ && shouldActivateLaunchHold())
            {
                launch_hold_active_ = true;
                planned_data_ = Data{};
                last_planned_data_ = Data{};
                planned_data_.rot_z = target_data_.rot_z;
                last_planned_data_.rot_z = target_data_.rot_z;
                for (u8 i = 0; i < 4; ++i)
                {
                    last_drive_omega_cmd_rad_s_[i] = 0.0f;
                }
            }

            if (launch_hold_active_)
            {
                const SwervePlannerOutput launch_preview_output =
                    planSwerveModules(makeSwervePlannerInput(makeLaunchHoldPreviewCommand()));
                if (isLaunchHoldAligned(launch_preview_output))
                {
                    launch_hold_active_ = false;
                }
            }

            if (launch_hold_active_)
            {
                planned_data_ = Data{};
                planned_data_.rot_z = target_data_.rot_z;
                return;
            }

            clampTargetSpeedInChassis(target_data_.vel_x, target_data_.vel_y, target_data_.omega_z,
                                      target_data_.vel_x, target_data_.vel_y, target_data_.omega_z);

            limitPlannedSpeed(target_data_.vel_x, target_data_.vel_y, target_data_.omega_z,
                              planned_data_.vel_x, planned_data_.vel_y, planned_data_.omega_z);

            planned_data_.acc_x = (planned_data_.vel_x - last_planned_data_.vel_x) / period_;
            planned_data_.acc_y = (planned_data_.vel_y - last_planned_data_.vel_y) / period_;
            planned_data_.alpha_z = (planned_data_.omega_z - last_planned_data_.omega_z) / period_;
            planned_data_.rot_z = target_data_.rot_z;
        }

        void Chassis::limitPlannedSpeed(f32 tar_vel_x, f32 tar_vel_y, f32 tar_omega_z, f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z)
        {
// 第一阶段：先x/y分量分别做加减速限幅，保证速度台阶被平滑化
            out_vel_x = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_vel_x, last_planned_data_.vel_x, period_, runtime_strategy_cfg_.max_acc_xy_acc_, runtime_strategy_cfg_.max_acc_xy_dec_);
            out_vel_y = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_vel_y, last_planned_data_.vel_y, period_, runtime_strategy_cfg_.max_acc_xy_acc_, runtime_strategy_cfg_.max_acc_xy_dec_);
            out_omega_z = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_omega_z, last_planned_data_.omega_z, period_, runtime_strategy_cfg_.max_alpha_z_acc_, runtime_strategy_cfg_.max_alpha_z_dec_);

// 第二阶段：平移矢量方向限幅（低速滞回冻+方向角速度限幅）
            const f32 tar_mag = magnitude2D(tar_vel_x, tar_vel_y);
            const f32 out_mag = magnitude2D(out_vel_x, out_vel_y);
            const f32 enter_speed = getNearZeroEnterSpeedMps();
            const f32 exit_speed = getNearZeroExitSpeedMps();
            const f32 dir_rate_limit_rad_s = degToRadF32((runtime_strategy_cfg_.trans_dir_rate_limit_deg_s_ >= 0.0f) ? runtime_strategy_cfg_.trans_dir_rate_limit_deg_s_ : 0.0f);
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
            const f32 steer_sign = (wheel.steer_motor_sign == 0.0f) ? 1.0f : wheel.steer_motor_sign;
            return steer_sign * degToRadF32(wheel.steer_motor_h->getTotalAngle());
        }

        f32 Chassis::readDriveMotorOmegaRadS(const WheelConfig &wheel) const
        {
            if (wheel.drive_motor_h == nullptr)
            {
                return 0.0f;
            }
            const SteerCalibration calibration{
                wheel.theta_oa_to_owi_rad,
                wheel.homing_runtime_zero_offset_rad,
                wheel.steer_motor_sign,
                wheel.drive_motor_sign,
            };
            return mapDriveMotorRpmToWheelOmega(wheel.drive_motor_h->getRPM(), calibration);
        }

        f32 Chassis::readCorrectedSteerMotorTotalAngleRad(const WheelConfig &wheel) const
        {
            const SteerCalibration calibration{
                wheel.theta_oa_to_owi_rad,
                wheel.homing_runtime_zero_offset_rad,
                wheel.steer_motor_sign,
                wheel.drive_motor_sign,
            };
            return mapRawSteerMotorTotalToCorrectedLocalTotal((wheel.steer_motor_h == nullptr) ? 0.0f : degToRadF32(wheel.steer_motor_h->getTotalAngle()), calibration);
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
                // 两个触发角相差 180°，保证任意起始状态半圈内都能抓到一个有效边沿
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
                    const SteerCalibration calibration = makeSteerCalibration(wheel);

                    wheel.homing_state = HomingState::kEdgeDetected;
                    wheel.homing_last_edge_is_falling = is_falling_edge;
                    wheel.homing_runtime_zero_offset_rad = computeHomingRuntimeZeroOffset(edge_mech_oa_rad,
                                                                                          raw_total_angle_rad,
                                                                                          wheel.homing_zero_offset_rad,
                                                                                          calibration);
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
// 边沿已抓到后，先走一个过渡态，确保零偏已经写入后再进入连续角度就绪态
                wheel.homing_state = HomingState::kOffsetApply;
                return false;
            }
            if (wheel.homing_state == HomingState::kOffsetApply)
            {
// 这一拍只做“应用偏置”的状态切换，不再改零偏，保持状态机步骤清晰可追踪
                wheel.homing_state = HomingState::kContinuousAngleReady;
                return false;
            }
            if (wheel.homing_state == HomingState::kContinuousAngleReady)
            {
                // 连续角度已可用后，先进入“归位到软件零点”阶段：
// OA角自动走0°（车头前方）再判定该轮回零完成
                wheel.homing_state = HomingState::kAlignToZero;
                return false;
            }

            if (wheel.homing_state == HomingState::kAlignToZero)
            {
                const f32 current_local_total_rad = wheel.corrected_steer_motor_total_angle_rad;
                const f32 current_oa_total_rad = mapWheelCorrectedLocalToOaTotal(wheel, current_local_total_rad);
                const f32 target_oa_total_rad = nearestEquivalentAngle(current_oa_total_rad, 0.0f);
                const f32 target_local_total_rad = mapWheelOaTotalToCorrectedLocal(wheel, target_oa_total_rad);
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
                const f32 steer_sign = (wheel.steer_motor_sign == 0.0f) ? 1.0f : wheel.steer_motor_sign;
                wheel.steer_motor_h->setTargetRPM(rpm / steer_sign);
            }
        }

        void Chassis::setSteerMotorTargetTotalAngleRad(WheelConfig &wheel, f32 corrected_local_total_angle_rad)
        {
            if (wheel.steer_motor_h == nullptr)
            {
                return;
            }
            const SteerCalibration calibration{
                wheel.theta_oa_to_owi_rad,
                wheel.homing_runtime_zero_offset_rad,
                wheel.steer_motor_sign,
                wheel.drive_motor_sign,
            };
            f32 raw_motor_total_rad = mapCorrectedLocalTotalToRawSteerMotorTotal(corrected_local_total_angle_rad, calibration);
            wheel.steer_motor_h->setTargetTotalAngle(radToDegF32(raw_motor_total_rad));
        }

        void Chassis::setDriveMotorTargetOmegaRadS(WheelConfig &wheel, f32 drive_omega_rad_s)
        {
            if (wheel.drive_motor_h != nullptr)
            {
                const SteerCalibration calibration{
                    wheel.theta_oa_to_owi_rad,
                    wheel.homing_runtime_zero_offset_rad,
                    wheel.steer_motor_sign,
                    wheel.drive_motor_sign,
                };
                wheel.drive_motor_h->setTargetRPM(mapWheelOmegaToDriveMotorRpm(drive_omega_rad_s, calibration));
            }
        }

// 这是一个“位置目+速度上限+加速度上限”的二阶限幅器
            // rate_delta_limit 是“这一拍速度最多允许变化多少”，由最大加速度决定
// 输出是“下一拍允许走到的位置”，并通过next_rate回传这一拍实际采用的速度
// 在四舵轮里它主要用于转向角规划：既不允许舵角变化过快，也不允许舵角速度突变过猛
        f32 Chassis::limitPositionSecondOrder(f32 current_value, f32 current_rate, f32 target_value, f32 max_rate, f32 max_accel, f32 dt_s, f32 &next_rate) const
        {
            // 先做速度变化率限制：如果期望速度离当前速度太远，
            const f32 safe_dt = (dt_s <= 1.0e-6f) ? 1.0e-3f : dt_s;

            // delta_value 是这一拍距离目标位置还差多少；
            // desired_rate 是“如果想在一拍内尽量逼近目标，希望使用的速度”，
// 但它先受max_rate限制，避免直接给出不可能达到的目标速度
            const f32 delta_value = target_value - current_value;
            const f32 desired_rate = clampValue(delta_value / safe_dt, -max_rate, max_rate);

// rate_delta_limit是“这一拍速度最多允许变化多少”，由最大加速度决定
            const f32 rate_delta_limit = max_accel * safe_dt;

            next_rate = current_rate;

// 先做速度变化率限制：如果期望速度离当前速度太远
            // 再做一次绝对速度限幅，保证最终速度不超过 max_rate
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

            // 返回下一拍允许到达的位置；调用侧会把它当作新的舵角目标
            next_rate = clampValue(next_rate, -max_rate, max_rate);

// 按这一拍最终允许的速度积分出位置步进量
            f32 step_value = next_rate * safe_dt;

// 如果这一拍已经足够到达目标，则直接截断到目标位置
// 避免积分后跨target_value造成过冲
            if (fabsf(step_value) > fabsf(delta_value))
            {
                step_value = delta_value;
                next_rate = step_value / safe_dt;
            }

// 返回下一拍允许到达的位置；调用侧会把它当作新的舵角目标
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
            const Data planner_command = launch_hold_active_ ? makeLaunchHoldPreviewCommand() : command_data;
            const SwervePlannerInput planner_input = makeSwervePlannerInput(planner_command);
            SwervePlannerOutput planner_output = planSwerveModules(planner_input);
            if (launch_hold_active_)
            {
                for (u8 i = 0; i < 4; ++i)
                {
                    planner_output.gate_or_cos_scale[i] = 0.0f;
                    planner_output.final_drive_omega_rad_s[i] = 0.0f;
                }
            }
            ActuatorCommandFrame command_frame{};
            buildActuatorCommandFrame(planner_output, command_frame);
            storePlannedActuatorFrame(planner_output, command_frame);
        }

        void Chassis::applyModuleCommands(bool all_homed)
        {
            // 这里是“四舵轮目标命令”真正落到电机接口前的最后一道门控：
// computeModuleCommands()虽然已经为每个轮子算好了目标舵角和驱动速度
// 但是否允许按这些目标下发，还要看当前是否全部完成回零，以及是否处于扭矩自由模式
            f32 execution_allowed_drive_targets_rad_s[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            bool execution_allow_drive_position_loop[4] = {true, true, true, true};
            bool execution_apply_shared_alpha = !input_target_data_.zero_current_all && !current_mode_flag_.is_wheel_torque_free;
            f32 shared_drive_alpha_scale = 1.0f;

            if (execution_apply_shared_alpha && runtime_strategy_cfg_.enable_drive_alpha_limit_)
            {
                const f32 drive_alpha_step_limit_rad_s = fabsf(runtime_strategy_cfg_.max_drive_alpha_rad_s2_) * period_;
                for (u8 i = 0; i < 4; ++i)
                {
                    execution_allow_drive_position_loop[i] = all_homed;
                    execution_allowed_drive_targets_rad_s[i] = all_homed ? actuator_command_frame_.drive_omega_rad_s[i] : 0.0f;

                    const f32 delta_drive_target_rad_s = execution_allowed_drive_targets_rad_s[i] - last_drive_omega_cmd_rad_s_[i];
                    const f32 abs_delta_drive_target_rad_s = fabsf(delta_drive_target_rad_s);
                    if (abs_delta_drive_target_rad_s <= drive_alpha_step_limit_rad_s || abs_delta_drive_target_rad_s <= 1.0e-6f)
                    {
                        continue;
                    }

                    const f32 wheel_alpha_scale = drive_alpha_step_limit_rad_s / abs_delta_drive_target_rad_s;
                    shared_drive_alpha_scale = (wheel_alpha_scale < shared_drive_alpha_scale) ? wheel_alpha_scale : shared_drive_alpha_scale;
                }
            }

            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                const f32 planned_drive_target_rad_s = actuator_command_frame_.drive_omega_rad_s[i];
                f32 allowed_drive_target_rad_s = planned_drive_target_rad_s;
                bool allow_drive_position_loop = true;

                if (input_target_data_.zero_current_all)
                {
// 硬零电流模式优先级最高：无论回零状态如何，四轮舵向/驱动都直接下0电流
                    allowed_drive_target_rad_s = 0.0f;
                    wheel.target_drive_omega_rad_s = 0.0f;
                    planned_data_.drive_omega_rad_s[i] = 0.0f;
                    last_drive_omega_cmd_rad_s_[i] = 0.0f;
                    setSteerMotorTargetCurrent(wheel, 0.0f);
                    if (wheel.drive_motor_h != nullptr)
                    {
                        wheel.drive_motor_h->setTargetCurrent(0.0f);
                    }
                    continue;
                }

                if (!all_homed)
                {
// 只要还有任意一个轮子没有完成回零，就先禁止所有驱动轮输出
// 避免底盘在零位未建立完成时带着错误朝向强行跑动
                    allow_drive_position_loop = execution_allow_drive_position_loop[i];
                    allowed_drive_target_rad_s = execution_allowed_drive_targets_rad_s[i];
                    f32 delivered_drive_target_rad_s = allowed_drive_target_rad_s;
                    if (runtime_strategy_cfg_.enable_drive_alpha_limit_)
                    {
                        delivered_drive_target_rad_s =
                            last_drive_omega_cmd_rad_s_[i] +
                            (allowed_drive_target_rad_s - last_drive_omega_cmd_rad_s_[i]) * shared_drive_alpha_scale;
                    }
                    if (runtime_strategy_cfg_.enable_drive_omega_limit_)
                    {
                        delivered_drive_target_rad_s = clampValue(delivered_drive_target_rad_s, -runtime_strategy_cfg_.max_drive_omega_rad_s_, runtime_strategy_cfg_.max_drive_omega_rad_s_);
                    }
                    wheel.target_drive_omega_rad_s = delivered_drive_target_rad_s;
                    planned_data_.drive_omega_rad_s[i] = delivered_drive_target_rad_s;
                    last_drive_omega_cmd_rad_s_[i] = delivered_drive_target_rad_s;
                    setDriveMotorTargetOmegaRadS(wheel, delivered_drive_target_rad_s);
                    if (wheel.homing_state == HomingState::kSearch)
                    {
// 正在搜索零位的轮子，允许转向电机按固定搜索转速慢慢转
// 目的是继续寻找传感器边沿；此时不走位置闭环
                        setSteerMotorTargetRPM(wheel, wheel.homing_search_rpm);
                    }
                    else if (wheel.homing_state == HomingState::kAlignToZero)
                    {
// 零偏建立后允许转向电机继续走位置闭环，把OA自动归到软件零点
                        // 注意：这里每拍都根据当前反馈重算“离 OA=0 最近的等效角”，
// 避免被上游常规模块解算写回“保持当前角”后导致归位停滞
                        wheel.target_steer_motor_total_angle_rad = computeHomingAlignTargetCorrectedLocalTotal(wheel);
                        setSteerMotorTargetTotalAngleRad(wheel, wheel.target_steer_motor_total_angle_rad);
                    }
                    else
                    {
// 不在搜索态的轮子，不再给转向动作，直接把转向电机电流打零
// 让状态机以“静止等待”的方式完成后续过渡
                        setSteerMotorTargetCurrent(wheel, 0.0f);
                    }
                    continue;
                }

                if (current_mode_flag_.is_wheel_torque_free)
                {
// 扭矩自由模式下，不执行任何舵角或驱动速度闭环
// 而是把转向和驱动都打成“零电流/零扭矩”状态，方便人工推动或安全释放
                    allowed_drive_target_rad_s = 0.0f;
                    wheel.target_drive_omega_rad_s = 0.0f;
                    planned_data_.drive_omega_rad_s[i] = 0.0f;
                    last_drive_omega_cmd_rad_s_[i] = 0.0f;
                    setSteerMotorTargetCurrent(wheel, 0.0f);
                    if (wheel.drive_motor_h != nullptr)
                    {
                        wheel.drive_motor_h->setTargetCurrent(0.0f);
                    }
                    continue;
                }

// 只有“全部回零完成”且“不是扭矩自由模式”时
// 才真正把上一阶段规划出的目标舵角和驱动角速度下发给电机闭环
                f32 delivered_drive_target_rad_s = allowed_drive_target_rad_s;
                if (runtime_strategy_cfg_.enable_drive_alpha_limit_)
                {
                    delivered_drive_target_rad_s =
                        last_drive_omega_cmd_rad_s_[i] +
                        (allowed_drive_target_rad_s - last_drive_omega_cmd_rad_s_[i]) * shared_drive_alpha_scale;
                }
                if (runtime_strategy_cfg_.enable_drive_omega_limit_)
                {
                    delivered_drive_target_rad_s = clampValue(delivered_drive_target_rad_s, -runtime_strategy_cfg_.max_drive_omega_rad_s_, runtime_strategy_cfg_.max_drive_omega_rad_s_);
                }

                wheel.target_drive_omega_rad_s = delivered_drive_target_rad_s;
                planned_data_.drive_omega_rad_s[i] = delivered_drive_target_rad_s;
                last_drive_omega_cmd_rad_s_[i] = delivered_drive_target_rad_s;
                setSteerMotorTargetTotalAngleRad(wheel, wheel.target_steer_motor_total_angle_rad);
                if (allow_drive_position_loop)
                {
                    setDriveMotorTargetOmegaRadS(wheel, delivered_drive_target_rad_s);
                }
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
                current_data_.steer_angle_oa_rad[i] = mapWheelCorrectedLocalToOaTotal(wheel_config_[i], wheel_config_[i].corrected_steer_motor_total_angle_rad);
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
            debug_mirror_.nz_stationary_m_s = getNearZeroEnterSpeedMps();
            debug_mirror_.nz_freeze_enter_m_s = getNearZeroEnterSpeedMps();
            debug_mirror_.nz_freeze_exit_m_s = getNearZeroExitSpeedMps();
            debug_mirror_.nz_xpark_enter_m_s = getNearZeroEnterSpeedMps();
            debug_mirror_.nz_xpark_exit_m_s = getNearZeroExitSpeedMps();
            debug_mirror_.nz_stop_guard_release_m_s = getStopGuardReleaseSpeedMps();
            debug_mirror_.lim_drive_omega = runtime_strategy_cfg_.enable_drive_omega_limit_;
            debug_mirror_.lim_drive_alpha = runtime_strategy_cfg_.enable_drive_alpha_limit_;
            debug_mirror_.lim_steer_rate = runtime_strategy_cfg_.enable_steer_rate_limit_;
            debug_mirror_.lim_steer_alpha = runtime_strategy_cfg_.enable_steer_alpha_limit_;
            debug_mirror_.vec_gate_scale = vector_gate_scale_;
            debug_mirror_.vec_dir_err_deg = vector_dir_err_deg_;
            debug_mirror_.vec_eta_max_s = vector_eta_max_s_;
            debug_mirror_.vec_gate_active = vector_gate_active_;
            debug_mirror_.hard_gate_bypassed_by_residual_speed = hard_gate_bypassed_by_residual_speed_;
            debug_mirror_.max_residual_speed_m_s = max_residual_speed_m_s_;
            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                debug_mirror_.current_oa_deg[i] = radToDegF32(mapWheelCorrectedLocalToOaTotal(wheel, wheel.corrected_steer_motor_total_angle_rad));
                debug_mirror_.target_oa_deg[i] = radToDegF32(mapWheelCorrectedLocalToOaTotal(wheel, wheel.target_steer_motor_total_angle_rad));
                debug_mirror_.current_drive_rpm[i] = radsToRpmF32(wheel.corrected_drive_omega_rad_s);
                debug_mirror_.target_drive_rpm[i] = radsToRpmF32(wheel.target_drive_omega_rad_s);
                debug_mirror_.planned_drive_target_rpm[i] = radsToRpmF32(actuator_command_frame_.drive_omega_rad_s[i]);
                debug_mirror_.delivered_drive_target_rpm[i] = radsToRpmF32(wheel.target_drive_omega_rad_s);
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
// 调试使能上升沿：从电机运行态回PID到调参缓存，形成“先读后改”基线
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
// 保护apply的手工改动：该轮缓存跳过同步，避免覆盖调试器刚写入的值
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
                    const f32 current_oa_total_rad = mapWheelCorrectedLocalToOaTotal(wheel, wheel.corrected_steer_motor_total_angle_rad);
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

        void Chassis::emitUart8SwerveTelemetryV2(bool all_homed)
        {
            if (!debug_output_.output_enable || debug_output_.output_mode_raw != static_cast<u8>(DebugOutputMode::kSwerveTelemetryV2))
            {
                return;
            }

            const u8 divider = (debug_output_.telemetry_sample_divider == 0U) ? 1U : debug_output_.telemetry_sample_divider;
            debug_output_.telemetry_cycle_counter = static_cast<u8>(debug_output_.telemetry_cycle_counter + 1U);
            if (debug_output_.telemetry_cycle_counter < divider)
            {
                return;
            }
            debug_output_.telemetry_cycle_counter = 0U;

            const u32 period_ms = (debug_output_.telemetry_period_ms > 0U) ? debug_output_.telemetry_period_ms : 8U;
            if ((time_ms_ - debug_output_.telemetry_last_ms) < period_ms)
            {
                return;
            }

            if (HAL_UART_GetState(&huart8) != HAL_UART_STATE_READY)
            {
                return;
            }

            u8 *const frame = swerve_telemetry_tx_frame_buf;
            u16 cursor = 0U;

            packU16LE(&frame[cursor], kSwerveTelemetryMagic);
            cursor += 2U;
            frame[cursor++] = kSwerveTelemetryVersion;
            const u8 flags = static_cast<u8>(kSwerveTelemetryFlagsCrcPayloadOnly |
                                             (all_homed ? kSwerveTelemetryFlagsAllHomed : 0U) |
                                             ((debug_output_.telemetry_profile_id & 0x0FU) << 4U));
            frame[cursor++] = flags;
            packU16LE(&frame[cursor], debug_output_.telemetry_seq);
            cursor += 2U;
            packU64LE(&frame[cursor], RtosTimeStampUs64::getTimeUs());
            cursor += 8U;
            packU16LE(&frame[cursor], kSwerveTelemetryMsgTypeMode5);
            cursor += 2U;
            const u16 payload_len_pos = cursor;
            cursor += 2U;

            const u16 payload_start = cursor;
            const u16 payload_capacity = kSwerveTelemetryPayloadBytes;
            const auto canPackF32 = [&]() -> bool {
                return static_cast<u16>(cursor - payload_start) <= static_cast<u16>(payload_capacity - sizeof(f32));
            };
            const auto packPayloadF32 = [&](f32 value) -> bool {
                if (!canPackF32())
                {
                    return false;
                }
                packF32LE(&frame[cursor], value);
                cursor += sizeof(f32);
                return true;
            };

            const auto packChassis4f = [&](f32 vx, f32 vy, f32 wz, f32 yaw) -> bool {
                return packPayloadF32(vx) &&
                       packPayloadF32(vy) &&
                       packPayloadF32(wz) &&
                       packPayloadF32(yaw);
            };

            TelemetryChassisState target_state{};
            target_state.vel_x = planned_data_.vel_x;
            target_state.vel_y = planned_data_.vel_y;
            target_state.omega_z = planned_data_.omega_z;
            target_state.yaw_rad = planned_data_.rot_z;

            TelemetryChassisState actual_state{};
            actual_state.vel_x = current_data_.vel_x;
            actual_state.vel_y = current_data_.vel_y;
            actual_state.omega_z = current_data_.omega_z;
            actual_state.yaw_rad = input_hwt_rot_z_;

            TelemetryWheelPose wheel_pose[kTelemetryWheelCount]{};
            for (u8 i = 0U; i < kTelemetryWheelCount; ++i)
            {
                wheel_pose[i].pos_x_m = wheel_config_[i].pos_x_m;
                wheel_pose[i].pos_y_m = wheel_config_[i].pos_y_m;
            }

            const TelemetrySnapshot snapshot = makeTelemetrySnapshot(all_homed,
                                                                    target_state,
                                                                    actual_state,
                                                                    wheel_pose,
                                                                    planned_data_.drive_omega_rad_s,
                                                                    current_data_.drive_omega_rad_s,
                                                                    planned_data_.steer_angle_oa_rad,
                                                                    current_data_.steer_angle_oa_rad);

            if (!packChassis4f(snapshot.target.vel_x, snapshot.target.vel_y, snapshot.target.omega_z, snapshot.target.yaw_rad) ||
                !packChassis4f(snapshot.actual.vel_x, snapshot.actual.vel_y, snapshot.actual.omega_z, snapshot.actual.yaw_rad))
            {
                return;
            }

            for (u8 i = 0U; i < kSwerveTelemetryWheelCount; ++i)
            {
                const TelemetryWheelState &wheel = snapshot.wheels[i];

                if (!packPayloadF32(wheel.target_drive_omega_rad_s) ||
                    !packPayloadF32(wheel.actual_drive_omega_rad_s) ||
                    !packPayloadF32(wheel.target_steer_oa_rad) ||
                    !packPayloadF32(wheel.actual_steer_oa_rad) ||
                    !packPayloadF32(wheel.target_velocity_x_m_s) ||
                    !packPayloadF32(wheel.target_velocity_y_m_s) ||
                    !packPayloadF32(wheel.actual_velocity_x_m_s) ||
                    !packPayloadF32(wheel.actual_velocity_y_m_s))
                {
                    return;
                }
            }

            const u16 payload_len = static_cast<u16>(cursor - payload_start);
            if (payload_len != kSwerveTelemetryPayloadBytes)
            {
                return;
            }
            packU16LE(&frame[payload_len_pos], payload_len);

            const u16 crc = crc16CcittFalse(&frame[payload_start], payload_len);
            packU16LE(&frame[cursor], crc);
            cursor += 2U;

            if (HAL_UART_Transmit_DMA(&huart8, frame, cursor) != HAL_OK)
            {
                return;
            }

            debug_output_.telemetry_last_ms = time_ms_;
            debug_output_.telemetry_seq = static_cast<u16>(debug_output_.telemetry_seq + 1U);
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
            case DebugOutputMode::kSwerveTelemetryV2:
                emitUart8SwerveTelemetryV2(all_homed);
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

            const DebugModuleOverrideRoute route = classifyDebugModuleOverrideRoute(debug_control_.mode_raw);
            if (route == DebugModuleOverrideRoute::kNone)
            {
                return false;
            }

            const u8 wheel_idx = (debug_control_.wheel_index < 4) ? debug_control_.wheel_index : 0;
            resetDebugModuleOverrideTargets(wheel_idx, route == DebugModuleOverrideRoute::kSingleWheel && debug_control_.single_wheel_soft_steer_enable);

            if (route == DebugModuleOverrideRoute::kSingleWheel)
            {
                applySingleWheelDebugOverride(wheel_idx, all_homed);
            }
            else if (route == DebugModuleOverrideRoute::kAlignForward)
            {
                applyAlignForwardDebugOverride();
            }
            else if (route == DebugModuleOverrideRoute::kHomingObserve)
            {
                applyHomingObserveDebugOverride();
            }
            else if (route == DebugModuleOverrideRoute::kDirectActuator)
            {
                applyDirectActuatorDebugOverride(wheel_idx);
            }

            finalizeDebugModuleOverride(all_homed, route);
            return true;
        }

        bool Chassis::solveLinear3x3(f32 matrix[3][4], f32 &x0, f32 &x1, f32 &x2) const
        {
// 这里用的是带主元选取的高斯消元，目标是稳定求3x3线性方程组
// 输入是增广矩阵，输出是三项未知量，失败通常意味着矩阵接近奇异
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
            // 再求解 3 个底盘自由度 [vx, vy, omega_z]
            // 这里不是直接解单个方程，而是把每个轮子的两个投影约束累积成最小二乘正规方程，
// 再求3个底盘自由度[vx,vy,omega_z]
            f32 normal[3][3] = {};
            f32 rhs[3] = {};

            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const f32 steer_angle_oa_rad = mapWheelCorrectedLocalToOaTotal(wheel, wheel.corrected_steer_motor_total_angle_rad);
                const f32 cos_theta = cosf(steer_angle_oa_rad);
                const f32 sin_theta = sinf(steer_angle_oa_rad);
                const f32 drive_linear_m_s = wheel.corrected_drive_omega_rad_s * runtime_strategy_cfg_.wheel_radius_m_;

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

                // 2) 解析模式并做坐标系转换
                // 1) 读取 IMU 航向/角速度
                // 4) 更新轮反馈与回零状态机
                // 3) 处理锁航向逻辑与速度限幅
                // 4) 更新轮反馈与回零状态机
                // 5) 生成模块命令、下发电机目标
                // 6) 回写当前估计值并等待下一周期
                input_hwt_rot_z_ = hwt->get_yaw_rad();
                input_hwt_omega_z_ = hwt->get_yaw_speed_rad();

                // 常态同步手柄缓存：即使 debug_control_.enable 关闭，也保持 airjoy_data_ 实时更新。
                // 便于通过调试器直接观察摇杆输入；不改变任何控制模式接管逻辑。
                CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);

                isDebugMode();
                setModeFlag();
                resolvePlannerTargetData();

                // 运行时允许调试器直接改基准阈值/限幅开关，这里每周期刷新限幅镜像，近零阈值由 helper 直接读取。
                refreshActuatorLimitState();

                updatePlannedMotionData();

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
                if (!all_homed && input_target_data_.zero_current_all)
                {
                    // Homing has higher priority than external zero-current request.
                    // Drop this request immediately to avoid interrupting homing.
                    input_target_data_.zero_current_all = false;
                }
                homing_start_request_ = false;

                if (applyDebugModuleOverride(all_homed))
                {
                    updateTaskPerfStat(loop_start_us, RtosTimeStampUs64::getTimeUs());
                    vTaskDelayUntil(&time_ms_, period_ms_);
                    continue;
                }

// 回零和正常控制共用同一套命令生成流程，但最终下发前会根all_homed选择
// 未回零时只保留安全动作，已回零时才输出完整舵驱动目标
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






