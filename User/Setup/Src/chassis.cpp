/**
 * @file chassis.cpp
 * @author 桑叁年
 * @brief 底盘控制主实现
 */

#define private public
#define protected public
#include "chassis.h"
#undef private
#undef protected

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

// ---------------------------------------------------------------------
// chassis.cpp 阅读指引
//
// 这个实现文件承载四舵轮底盘控制线程的完整运行链路。主流程可以按下面顺序理解：
// 1. 初始化与线程入口：init() / createThread() / runThread()
// 2. 输入规范化：把 API、坐标系模式、锁角请求和调试接管统一折叠到 target_data_
// 3. 车体级规划：限速、速度曲线、起步等待门、yaw lock / zero-stop 相关预处理
// 4. 模块级求解：把车体目标转换成每个轮子的舵角和驱动目标
// 5. 执行仲裁：结合 homing、steer fault、X-Park、zero-stop 等运行态决定最终是否允许下发
// 6. 观测输出：刷新 current/debug mirror，并在 FULL_DEBUG 档输出 trace / telemetry
//
// 因为这里长期叠加了发布态、调试态、恢复态和安全门控，很多函数不仅要回答“做什么”，
// 还要解释“为什么在这里做”和“它依赖哪个上游状态”。本轮注释优先补这一层语义。
// ---------------------------------------------------------------------
namespace jia
{
    namespace FourSteerChassis
    {
        namespace
        {
            // -----------------------------------------------------------------
            // 二进制 telemetry 打包常量与轻量 helper
            // 这组常量只服务 mode5 二进制遥测输出，故意放在匿名命名空间内，避免泄漏到底盘对外接口。
            // -----------------------------------------------------------------
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

            // -----------------------------------------------------------------
            // 调试面板 / 单轮直控配置清洗
            // 所有来自调试面板的 raw u8 枚举都会先在这里被钳回合法范围，
            // 这样线程主流程只处理“已清洗值”，不用到处散落防御判断。
            // -----------------------------------------------------------------
            inline Chassis::DirectAxisInputMode sanitizeDirectAxisInputMode(u8 raw_mode)
            {
                return (raw_mode <= static_cast<u8>(Chassis::DirectAxisInputMode::kRcStep))
                           ? static_cast<Chassis::DirectAxisInputMode>(raw_mode)
                           : Chassis::DirectAxisInputMode::kCached;
            }

            inline Chassis::DirectSteerCommandType sanitizeDirectSteerCommandType(u8 raw_type)
            {
                return (raw_type <= static_cast<u8>(Chassis::DirectSteerCommandType::kMultiTurnDeg))
                           ? static_cast<Chassis::DirectSteerCommandType>(raw_type)
                           : Chassis::DirectSteerCommandType::kRpm;
            }

            inline Chassis::DirectDriveCommandType sanitizeDirectDriveCommandType(u8 raw_type)
            {
                return (raw_type <= static_cast<u8>(Chassis::DirectDriveCommandType::kBrake))
                           ? static_cast<Chassis::DirectDriveCommandType>(raw_type)
                           : Chassis::DirectDriveCommandType::kRpm;
            }

            inline Chassis::SingleWheelInputAxis sanitizeSingleWheelInputAxis(u8 raw_axis)
            {
                return (raw_axis <= static_cast<u8>(Chassis::SingleWheelInputAxis::kRightY))
                           ? static_cast<Chassis::SingleWheelInputAxis>(raw_axis)
                           : Chassis::SingleWheelInputAxis::kLeftX;
            }

            inline Chassis::SingleWheelPlannerMode sanitizeSingleWheelPlannerMode(u8 raw_mode)
            {
                return (raw_mode <= static_cast<u8>(Chassis::SingleWheelPlannerMode::kTrapezoid))
                           ? static_cast<Chassis::SingleWheelPlannerMode>(raw_mode)
                           : Chassis::SingleWheelPlannerMode::kOff;
            }

            inline Chassis::DebugOutputFamily sanitizeDebugOutputFamily(u8 raw_family)
            {
                return (raw_family <= static_cast<u8>(Chassis::DebugOutputFamily::kBinary))
                           ? static_cast<Chassis::DebugOutputFamily>(raw_family)
                           : Chassis::DebugOutputFamily::kOff;
            }

            inline Chassis::JustFloatProfile sanitizeJustFloatProfile(u8 raw_profile)
            {
                return (raw_profile <= static_cast<u8>(Chassis::JustFloatProfile::kDriveZeroStopBrakeTrace))
                           ? static_cast<Chassis::JustFloatProfile>(raw_profile)
                           : Chassis::JustFloatProfile::kYawPid;
            }

            inline Chassis::SingleWheelTracePayloadKind sanitizeSingleWheelTracePayloadKind(u8 raw_payload)
            {
                return (raw_payload <= static_cast<u8>(Chassis::SingleWheelTracePayloadKind::kDriveOnly))
                           ? static_cast<Chassis::SingleWheelTracePayloadKind>(raw_payload)
                           : Chassis::SingleWheelTracePayloadKind::kSteerOnly;
            }

            inline f32 getDirectSteerDefaultLimit(Chassis::DirectSteerCommandType type)
            {
                switch (type)
                {
                case Chassis::DirectSteerCommandType::kCurrent:
                    return 12000.0f;
                case Chassis::DirectSteerCommandType::kSingleTurnDeg:
                    return 180.0f;
                case Chassis::DirectSteerCommandType::kMultiTurnDeg:
                    return 1080.0f;
                case Chassis::DirectSteerCommandType::kRpm:
                default:
                    return 250.0f;
                }
            }

            inline f32 getDirectSteerDefaultStepValue(Chassis::DirectSteerCommandType type)
            {
                switch (type)
                {
                case Chassis::DirectSteerCommandType::kCurrent:
                    return 2000.0f;
                case Chassis::DirectSteerCommandType::kSingleTurnDeg:
                    return 90.0f;
                case Chassis::DirectSteerCommandType::kMultiTurnDeg:
                    return 180.0f;
                case Chassis::DirectSteerCommandType::kRpm:
                default:
                    return 200.0f;
                }
            }

            inline f32 getDirectDriveDefaultLimit(Chassis::DirectDriveCommandType type)
            {
                switch (type)
                {
                case Chassis::DirectDriveCommandType::kCurrent:
                case Chassis::DirectDriveCommandType::kBrake:
                    return 12000.0f;
                case Chassis::DirectDriveCommandType::kRpm:
                default:
                    return 1000.0f;
                }
            }

            inline f32 getDirectDriveDefaultStepValue(Chassis::DirectDriveCommandType type)
            {
                switch (type)
                {
                case Chassis::DirectDriveCommandType::kCurrent:
                    return 2000.0f;
                case Chassis::DirectDriveCommandType::kBrake:
                    return 1500.0f;
                case Chassis::DirectDriveCommandType::kRpm:
                default:
                    return 200.0f;
                }
            }

            inline bool isSingleWheelSteerPlannerSupported(Chassis::DirectSteerCommandType type)
            {
                return type != Chassis::DirectSteerCommandType::kCurrent;
            }

            inline bool isSingleWheelDrivePlannerSupported(Chassis::DirectDriveCommandType type)
            {
                return type == Chassis::DirectDriveCommandType::kRpm;
            }

            inline bool getDriveSpeedPidDerivativeFirst(const VESC_Motor *drive_motor)
            {
                // drive 共享调参链路需要回读当前运行态的微分先行开关。
                return (drive_motor != nullptr) ? drive_motor->get_speed_pid_derivative_first() : false;
            }

            inline void setDriveSpeedPidDerivativeFirst(VESC_Motor *drive_motor, bool derivative_first)
            {
                if (drive_motor == nullptr)
                {
                    return;
                }

                // 通过 VESC 专门接口同步，避免重新把策略塞回 pid_init 入口。
                drive_motor->set_speed_pid_derivative_first(derivative_first);
            }

        } // namespace

        // -----------------------------------------------------------------
        // 生命周期与线程入口
        // -----------------------------------------------------------------
        void Chassis::init(InitConfig &config)
        {
            // init() 只做两件事：
            // 1. 绑定四个模块的硬件句柄，并把 wheel/runtime/cache 初始化到可预测基线；
            // 2. 启动 1ms 底盘控制线程。
            // 真正的控制闭环、状态机推进和输出仲裁都在线程周期里完成。
            runtime_strategy_cfg_ = default_strategy_cfg_;
            refreshActuatorLimitState();

            static const WheelInitConfig kDefaultWheelInit[4] = {
                {.pos_x_m = -0.39f, .pos_y_m = 0.40f, .theta_oa_to_owi_deg = -90.0f, .steer_motor_sign = 1.0f, .drive_motor_sign = -1.0f, .homing_enabled = true, .homing_sensor_active_high = true, .homing_gpio_port = kPHOTOGATE_1_GPIO_Port, .homing_gpio_pin = kPHOTOGATE_1_Pin, .homing_falling_edge_mech_deg = -30.0f, .homing_rising_edge_mech_deg = 150.0f, .homing_search_rpm = JIA_CHASSIS_HOMING_SEARCH_RPM, .homing_zero_offset_deg = -30.0f, .homing_timeout_s = 5.0f},
                {.pos_x_m = -0.39f, .pos_y_m = -0.40f, .theta_oa_to_owi_deg = 0.0f, .steer_motor_sign = 1.0f, .drive_motor_sign = 1.0f, .homing_enabled = true, .homing_sensor_active_high = true, .homing_gpio_port = kPHOTOGATE_2_GPIO_Port, .homing_gpio_pin = kPHOTOGATE_2_Pin, .homing_falling_edge_mech_deg = 60.0f, .homing_rising_edge_mech_deg = -120.0f, .homing_search_rpm = JIA_CHASSIS_HOMING_SEARCH_RPM, .homing_zero_offset_deg = -30.0f, .homing_timeout_s = 5.0f},
                {.pos_x_m = 0.39f, .pos_y_m = -0.40f, .theta_oa_to_owi_deg = 90.0f, .steer_motor_sign = 1.0f, .drive_motor_sign = -1.0f, .homing_enabled = true, .homing_sensor_active_high = true, .homing_gpio_port = kPHOTOGATE_3_GPIO_Port, .homing_gpio_pin = kPHOTOGATE_3_Pin, .homing_falling_edge_mech_deg = 150.0f, .homing_rising_edge_mech_deg = -30.0f, .homing_search_rpm = JIA_CHASSIS_HOMING_SEARCH_RPM, .homing_zero_offset_deg = -30.0f, .homing_timeout_s = 5.0f},
                {.pos_x_m = 0.39f, .pos_y_m = 0.40f, .theta_oa_to_owi_deg = 180.0f, .steer_motor_sign = 1.0f, .drive_motor_sign = 1.0f, .homing_enabled = true, .homing_sensor_active_high = true, .homing_gpio_port = kPHOTOGATE_4_GPIO_Port, .homing_gpio_pin = kPHOTOGATE_4_Pin, .homing_falling_edge_mech_deg = -120.0f, .homing_rising_edge_mech_deg = 60.0f, .homing_search_rpm = JIA_CHASSIS_HOMING_SEARCH_RPM, .homing_zero_offset_deg = -30.0f, .homing_timeout_s = 5.0f},
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
                wheel.homing_align_command_armed = false;
                wheel.homing_zero_valid = !wheel.homing_enabled;
                wheel.homing_search_timeout_armed = false;
                resetHomingEdgeConfirmState(wheel);
                wheel.homing_elapsed_s = 0.0f;
                wheel.homing_runtime_zero_offset_rad = wheel.homing_zero_offset_rad;
                wheel.homing_hold_corrected_local_total_rad = 0.0f;
                wheel.corrected_steer_motor_total_angle_rad = 0.0f;
                wheel.corrected_drive_omega_rad_s = 0.0f;
                wheel.target_steer_motor_total_angle_rad = 0.0f;
                wheel.target_drive_omega_rad_s = 0.0f;
                wheel.steer_target_velocity_rad_s = 0.0f;
                wheel.flipped_drive_direction = false;
                wheel.steer_fault_state = SteerFaultState::kNone;
                wheel.steer_fault_rehome_request = false;
                wheel.steer_feedback_current_mA = 0.0f;
                wheel.steer_feedback_last_current_mA = 0.0f;
                wheel.steer_feedback_last_raw_total_angle_rad = 0.0f;
                wheel.steer_feedback_freeze_ms = 0U;
                wheel.steer_feedback_recovery_toggle_count = 0U;
                wheel.steer_fault_latched_count = 0U;
                selected_flipped_solution_[i] = false;
                low_speed_drive_suppression_scale_[i] = 1.0f;
            }

            high_speed_drive_suppression_scale_ = 1.0f;
            high_speed_trans_gate_active_ = false;
            high_speed_drive_suppression_active_ = false;
            high_speed_dir_err_deg_ = 0.0f;
            high_speed_eta_max_s_ = 0.0f;
            steer_fault_any_active_ = false;
            low_speed_residual_bypass_active_ = false;
            yaw_lock_zero_stop_decel_context_active_ = false;
            yaw_lock_zero_stop_release_hold_elapsed_ms_ = 0U;
            yaw_lock_zero_stop_preview_command_valid_ = false;
            yaw_lock_zero_stop_preview_command_ = Data{};
            trans_dir_freeze_active_ = false;
            trans_dir_ref_valid_ = false;
            trans_dir_ref_rad_ = 0.0f;
            trans_dir_tar_mag_m_s_ = 0.0f;
            trans_dir_out_mag_m_s_ = 0.0f;
            trans_dir_freeze_reason_ = 0U;
            active_manual_speed_profile_mode_ = runtime_strategy_cfg_.manual_speed_profile_mode;
            manual_vel_x_shape_state_ = {};
            manual_vel_y_shape_state_ = {};
            manual_omega_z_shape_state_ = {};
            high_speed_drive_suppression_scale_ = 1.0f;
            high_speed_drive_suppression_active_ = false;
            high_speed_dir_err_deg_ = 0.0f;
            high_speed_eta_max_s_ = 0.0f;

            rot_z_pid_.set_params(lock_angle_pid_params, 1.0f);
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
            // RTOS 线程入口到成员函数的桥接层。
            Chassis *chassis = static_cast<Chassis *>(arg);
            chassis->runThread(NULL);
        }

        void Chassis::clearInputTargetData()
        {
            // “清空输入”并不只是把速度归零，还要一并复位所有和输入语义绑定的运行态：
            // - yaw lock 目标与滤波状态
            // - zero-stop / release hold 上下文
            // - 平移方向冻结参考
            // - 手动速度曲线内部状态
            // 这样调用者拿到的才是真正干净的输入侧基线。
            input_target_data_.mode = Mode::kWheelTorqueFreeMode;
            input_target_data_.vel_x = 0.0f;
            input_target_data_.vel_y = 0.0f;
            input_target_data_.omega_z = 0.0f;
            input_target_data_.rot_z = 0.0f;
            input_target_data_.steer_lock_angle_deg = 0.0f;
            input_target_data_.drive_lock_speed_m_s = 0.0f;
            input_target_data_.zero_current_all = false;
            lock_now_rot_z_target_ = 0.0f;
            yaw_lock_control_active_last_cycle_ = false;
            yaw_lock_zero_stop_decel_context_active_ = false;
            yaw_lock_zero_stop_release_hold_elapsed_ms_ = 0U;
            resetYawPidTargetRuntime();
            trans_dir_freeze_active_ = false;
            trans_dir_ref_valid_ = false;
            trans_dir_ref_rad_ = 0.0f;
            trans_dir_tar_mag_m_s_ = 0.0f;
            trans_dir_out_mag_m_s_ = 0.0f;
            trans_dir_freeze_reason_ = 0U;
            active_manual_speed_profile_mode_ = runtime_strategy_cfg_.manual_speed_profile_mode;
            manual_vel_x_shape_state_ = {};
            manual_vel_y_shape_state_ = {};
            manual_omega_z_shape_state_ = {};
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
            // 真正的搜索、沿边沿捕获零位、偏置生效和完成判定都在 runThread() 中按周期推进。
            homing_start_request_ = true;
            steer_fault_any_active_ = false;
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                clearSteerFaultState(wheel);
                wheel.homing_elapsed_s = 0.0f;
                wheel.homing_last_sensor_active = false;
                wheel.homing_align_command_armed = false;
                wheel.homing_search_timeout_armed = false;
                resetHomingEdgeConfirmState(wheel);
                wheel.homing_runtime_zero_offset_rad = wheel.homing_zero_offset_rad;
                wheel.homing_hold_corrected_local_total_rad = 0.0f;
                if (wheel.homing_enabled && wheel.homing_gpio_port != nullptr)
                {
                    wheel.homing_state = HomingState::kIdle;
                    wheel.homing_zero_valid = false;
                    wheel.homing_align_command_armed = false;
                }
                else
                {
                    wheel.homing_state = HomingState::kReady;
                    wheel.homing_zero_valid = true;
                    wheel.homing_align_command_armed = false;
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

        f32 Chassis::getXParkCommandEnterSpeedMps() const
        {
            return (runtime_strategy_cfg_.xpark_command_threshold_cfg_.enter_m_s >= 0.0f) ? runtime_strategy_cfg_.xpark_command_threshold_cfg_.enter_m_s : 0.0f;
        }

        f32 Chassis::getXParkCommandExitSpeedMps() const
        {
            const f32 enter = getXParkCommandEnterSpeedMps();
            const f32 exit_raw = (runtime_strategy_cfg_.xpark_command_threshold_cfg_.exit_m_s >= 0.0f) ? runtime_strategy_cfg_.xpark_command_threshold_cfg_.exit_m_s : 0.0f;
            return (exit_raw > enter) ? exit_raw : (enter + 1.0e-3f);
        }

        bool Chassis::shouldActivateReverseIntent(f32 target_vel_x, f32 target_vel_y, f32 reference_dir_rad) const
        {
            const StrategyConfig::ReverseIntentConfig &cfg = runtime_strategy_cfg_.reverse_intent;
            if (!cfg.enable)
            {
                return false;
            }

            const f32 speed_m_s = magnitude2DF32(target_vel_x, target_vel_y);
            const f32 min_speed_m_s = (cfg.min_speed_m_s > 0.0f) ? cfg.min_speed_m_s : getNearZeroExitSpeedMps();
            if (speed_m_s <= min_speed_m_s)
            {
                return false;
            }

            const f32 target_dir_rad = atan2f(target_vel_y, target_vel_x);
            const f32 dir_err_deg = radToDegF32(fabsf(shortestAngularDistanceF32(reference_dir_rad, target_dir_rad)));
            const f32 enter_deg = (cfg.enter_angle_deg >= 0.0f) ? cfg.enter_angle_deg : 135.0f;
            const f32 exit_deg = (cfg.exit_angle_deg >= 0.0f) ? cfg.exit_angle_deg : 105.0f;
            return reverse_intent_active_ ? (dir_err_deg >= exit_deg) : (dir_err_deg >= enter_deg);
        }

        void Chassis::refreshActuatorLimitState()
        {
            // 预留钩子：当前执行器限幅开关直接从就近布置的 enable_* 成员读取，无需额外派生状态。
        }

        f32 Chassis::mapSingleTurnToNearestTotalAngle(const WheelConfig &wheel, f32 target_oa_single_turn_deg) const
        {
            const f32 target_oa_mod_rad = wrapTo2PiF32(degToRadF32(target_oa_single_turn_deg));
            const SteerCalibration calibration{
                wheel.theta_oa_to_owi_rad,
                wheel.homing_runtime_zero_offset_rad,
                wheel.steer_motor_sign,
                wheel.drive_motor_sign,
            };
            const f32 current_oa_total_rad = mapCorrectedLocalTotalToOaTotal(wheel.corrected_steer_motor_total_angle_rad, calibration);
            const f32 target_oa_total_rad = nearestEquivalentAngleF32(current_oa_total_rad, target_oa_mod_rad);
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
                const f32 unit_x = cosRadF32(planned_oa_total_rad[i]);
                const f32 unit_y = sinRadF32(planned_oa_total_rad[i]);
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
                const f32 cos_theta = cosRadF32(steer_angle_oa_rad);
                const f32 sin_theta = sinRadF32(steer_angle_oa_rad);
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
            // Expose planned twist in the public/debug body frame.
            out_vel_x = -out_vel_x;
            out_vel_y = -out_vel_y;
            return true;
        }

        f32 Chassis::updateHighSpeedDriveSuppression(f32 translational_speed_m_s, f32 eta_max_s, f32 dir_err_deg)
        {
            const StrategyConfig::HighSpeedDriveSuppressionConfig &cfg = runtime_strategy_cfg_.high_speed_drive_suppression;
            if (!runtime_strategy_cfg_.enable_high_speed_drive_suppression)
            {
                high_speed_trans_gate_active_ = false;
                high_speed_drive_suppression_active_ = false;
                high_speed_drive_suppression_scale_ = 1.0f;
                return high_speed_drive_suppression_scale_;
            }

            const f32 trans_enter_speed_m_s = getNearZeroEnterSpeedMps();
            const f32 trans_exit_speed_m_s = getNearZeroExitSpeedMps();
            if (high_speed_trans_gate_active_)
            {
                high_speed_trans_gate_active_ = translational_speed_m_s > trans_enter_speed_m_s;
            }
            else
            {
                high_speed_trans_gate_active_ = translational_speed_m_s > trans_exit_speed_m_s;
            }

            if (!high_speed_trans_gate_active_)
            {
                high_speed_drive_suppression_active_ = false;
                high_speed_drive_suppression_scale_ = 1.0f;
                return high_speed_drive_suppression_scale_;
            }

            const f32 dir_enter = (cfg.dir_err_enter_deg > 0.0f) ? cfg.dir_err_enter_deg : 12.0f;
            const f32 dir_exit = (cfg.dir_err_exit_deg > 0.0f && cfg.dir_err_exit_deg < dir_enter) ? cfg.dir_err_exit_deg : (dir_enter * 0.5f);
            const f32 eta_enter = (cfg.eta_lock_s > 0.0f) ? cfg.eta_lock_s : 0.20f;
            const f32 eta_exit = (cfg.eta_release_s > 0.0f && cfg.eta_release_s < eta_enter) ? cfg.eta_release_s : (eta_enter * 0.3f);

            if (high_speed_drive_suppression_active_)
            {
                high_speed_drive_suppression_active_ = (dir_err_deg >= dir_exit) || (eta_max_s >= eta_exit);
            }
            else
            {
                high_speed_drive_suppression_active_ = (dir_err_deg >= dir_enter) || (eta_max_s >= eta_enter);
            }

            const f32 ramp_up_s = (cfg.gate_ramp_up_s > 1.0e-4f) ? cfg.gate_ramp_up_s : 0.08f;
            const f32 ramp_down_s = (cfg.gate_ramp_down_s > 1.0e-4f) ? cfg.gate_ramp_down_s : 0.03f;
            if (high_speed_drive_suppression_active_)
            {
                high_speed_drive_suppression_scale_ = clampValue(high_speed_drive_suppression_scale_ - period_ / ramp_down_s, 0.0f, 1.0f);
            }
            else
            {
                high_speed_drive_suppression_scale_ = clampValue(high_speed_drive_suppression_scale_ + period_ / ramp_up_s, 0.0f, 1.0f);
            }
            return high_speed_drive_suppression_scale_;
        }

        void Chassis::resetRuntimeStrategyToInitConfig()
        {
            runtime_strategy_cfg_ = default_strategy_cfg_;
            high_speed_drive_suppression_scale_ = 1.0f;
            high_speed_trans_gate_active_ = false;
            high_speed_drive_suppression_active_ = false;
            high_speed_dir_err_deg_ = 0.0f;
            high_speed_eta_max_s_ = 0.0f;
            low_speed_residual_bypass_active_ = false;
            yaw_lock_zero_stop_decel_context_active_ = false;
            yaw_lock_zero_stop_release_hold_elapsed_ms_ = 0U;
            xpark_gate_active_ = false;
            xpark_stationary_hold_ms_ = 0U;
            trans_dir_freeze_active_ = false;
            trans_dir_ref_valid_ = false;
            trans_dir_ref_rad_ = 0.0f;
            trans_dir_tar_mag_m_s_ = 0.0f;
            trans_dir_out_mag_m_s_ = 0.0f;
            trans_dir_freeze_reason_ = 0U;
            active_manual_speed_profile_mode_ = runtime_strategy_cfg_.manual_speed_profile_mode;
            manual_vel_x_shape_state_ = {};
            manual_vel_y_shape_state_ = {};
            manual_omega_z_shape_state_ = {};
            reverse_intent_active_ = false;
            reverse_intent_dir_err_deg_ = 0.0f;
            refreshActuatorLimitState();
        }

        Chassis::ManualSpeedProfileMode Chassis::resolveEffectiveManualSpeedProfileMode() const
        {
#if JIA_CHASSIS_ENABLE_SINGLE_WHEEL_DEBUG
            const DebugMode debug_mode = resolveDebugMode(debug_control_.common.mode_raw);
            const bool single_wheel_scurve_debug =
                debug_control_.common.enable && isSingleWheelIsolatedMode(debug_mode);
#else
            const bool single_wheel_scurve_debug = false;
#endif
            if (runtime_strategy_cfg_.manual_speed_profile_manual_only &&
                normalized_body_command_.source != CommandInputSource::kDebugTarget &&
                !single_wheel_scurve_debug)
            {
                return ManualSpeedProfileMode::kLegacy;
            }
            return runtime_strategy_cfg_.manual_speed_profile_mode;
        }

        void Chassis::resetManualSpeedProfileRuntimeState(bool reset_gate_state)
        {
            planned_data_ = Data{};
            last_planned_data_ = Data{};
            planned_data_.rot_z = target_data_.rot_z;
            last_planned_data_.rot_z = target_data_.rot_z;
            for (u8 i = 0; i < 4; ++i)
            {
                last_drive_omega_cmd_rad_s_[i] = 0.0f;
            }

            manual_vel_x_shape_state_ = {};
            manual_vel_y_shape_state_ = {};
            manual_omega_z_shape_state_ = {};

            trans_dir_freeze_active_ = false;
            trans_dir_ref_valid_ = false;
            trans_dir_ref_rad_ = 0.0f;
            trans_dir_tar_mag_m_s_ = 0.0f;
            trans_dir_out_mag_m_s_ = 0.0f;
            trans_dir_freeze_reason_ = 0U;
            reverse_intent_active_ = false;
            reverse_intent_dir_err_deg_ = 0.0f;

            if (reset_gate_state)
            {
                high_speed_trans_gate_active_ = false;
                high_speed_drive_suppression_active_ = false;
                high_speed_drive_suppression_scale_ = 1.0f;
                high_speed_dir_err_deg_ = 0.0f;
                high_speed_eta_max_s_ = 0.0f;
                low_speed_residual_bypass_active_ = false;
                yaw_lock_zero_stop_decel_context_active_ = false;
                yaw_lock_zero_stop_release_hold_elapsed_ms_ = 0U;
            }
        }

        f32 Chassis::limitValueByJerkProfile(f32 target_value,
                                             f32 current_value,
                                             JerkLimitedAxisState &axis_state,
                                             f32 accel_limit,
                                             f32 decel_limit,
                                             f32 jerk_acc_limit,
                                             f32 jerk_dec_limit,
                                             f32 settle_vel_epsilon,
                                             f32 settle_accel_epsilon) const
        {
            const f32 safe_acc_limit = fabsf(accel_limit);
            const f32 safe_dec_limit = fabsf(decel_limit);
            const f32 safe_jerk_acc_limit = fabsf(jerk_acc_limit);
            const f32 safe_jerk_dec_limit = fabsf(jerk_dec_limit);
            const f32 safe_settle_vel_epsilon = fabsf(settle_vel_epsilon);
            const f32 safe_settle_accel_epsilon = fabsf(settle_accel_epsilon);

            if (safe_acc_limit <= 1.0e-6f || safe_dec_limit <= 1.0e-6f ||
                safe_jerk_acc_limit <= 1.0e-6f || safe_jerk_dec_limit <= 1.0e-6f)
            {
                axis_state.shaped_value = target_value;
                axis_state.shaped_accel = 0.0f;
                axis_state.initialized = true;
                return target_value;
            }

            if (!axis_state.initialized)
            {
                axis_state.shaped_value = current_value;
                axis_state.shaped_accel = 0.0f;
                axis_state.initialized = true;
            }

            const f32 error = target_value - current_value;
            const f32 current_accel = axis_state.shaped_accel;
            const f32 error_sign = (error > 1.0e-6f) ? 1.0f : ((error < -1.0e-6f) ? -1.0f : 0.0f);
            const f32 braking_accel_limit = (current_accel * error_sign >= 0.0f) ? safe_dec_limit : safe_acc_limit;
            const f32 braking_jerk_limit = (current_accel * error_sign >= 0.0f) ? safe_jerk_dec_limit : safe_jerk_acc_limit;

            if (fabsf(error) <= safe_settle_vel_epsilon && fabsf(current_accel) <= safe_settle_accel_epsilon)
            {
                axis_state.shaped_value = target_value;
                axis_state.shaped_accel = 0.0f;
                return target_value;
            }

            const f32 abs_current_accel = fabsf(current_accel);
            const f32 time_to_cancel_accel = abs_current_accel / braking_jerk_limit;
            const f32 residual_velocity_from_accel =
                abs_current_accel * time_to_cancel_accel - 0.5f * braking_jerk_limit * time_to_cancel_accel * time_to_cancel_accel;
            const f32 min_reachable_step = fabsf(current_accel) * period_ + braking_jerk_limit * period_ * period_;

            if (fabsf(error) <= min_reachable_step)
            {
                axis_state.shaped_value = target_value;
                axis_state.shaped_accel = 0.0f;
                return target_value;
            }

            f32 target_accel = 0.0f;
            if (error_sign == 0.0f)
            {
                if (fabsf(current_accel) <= safe_settle_accel_epsilon)
                {
                    axis_state.shaped_value = target_value;
                    axis_state.shaped_accel = 0.0f;
                    return target_value;
                }
                target_accel = (current_accel > 0.0f) ? -safe_dec_limit : safe_dec_limit;
            }
            else
            {
                if (current_value * target_value < 0.0f)
                {
                    // 快速反向跨零时不能把目标加速度钉成 0，否则旧趋势会在零点附近被卸空却不再真正过零。
                    target_accel = (current_value > 0.0f) ? -safe_dec_limit : safe_dec_limit;
                }
                else
                {
                const f32 accel_limit_for_error = (error_sign * current_accel >= 0.0f) ? safe_acc_limit : safe_dec_limit;
                const f32 jerk_limit_for_error = (error_sign * current_accel >= 0.0f) ? safe_jerk_acc_limit : safe_jerk_dec_limit;
                const f32 braking_guard = fabsf(current_accel) * period_ + jerk_limit_for_error * period_ * period_;
                const bool should_brake_now =
                    (error_sign * current_accel > 0.0f) &&
                    (fabsf(error) <= residual_velocity_from_accel + braking_guard);

                if (should_brake_now)
                {
                    target_accel = 0.0f;
                }
                else
                {
                    target_accel = error_sign * accel_limit_for_error;
                }
                }
            }

            const f32 accel_delta = target_accel - current_accel;
            const f32 accel_delta_sign = (accel_delta > 0.0f) ? 1.0f : ((accel_delta < 0.0f) ? -1.0f : 0.0f);
            const f32 jerk_limit = (accel_delta_sign * error_sign >= 0.0f) ? safe_jerk_acc_limit : safe_jerk_dec_limit;
            const f32 max_accel_step = jerk_limit * period_;
            f32 next_accel = current_accel;

            if (accel_delta > max_accel_step)
            {
                next_accel += max_accel_step;
            }
            else if (accel_delta < -max_accel_step)
            {
                next_accel -= max_accel_step;
            }
            else
            {
                next_accel = target_accel;
            }

            next_accel = clampValue(next_accel, -braking_accel_limit, braking_accel_limit);

            f32 next_value = current_value + next_accel * period_;
            if ((current_value > 0.0f && target_value < 0.0f && next_value < 0.0f) ||
                (current_value < 0.0f && target_value > 0.0f && next_value > 0.0f))
            {
                next_value = 0.0f;
            }
            const f32 next_error = target_value - next_value;
            if (error_sign != 0.0f && next_error * error_sign <= 0.0f)
            {
                axis_state.shaped_value = target_value;
                axis_state.shaped_accel = 0.0f;
                return target_value;
            }

            axis_state.shaped_value = next_value;
            axis_state.shaped_accel = next_accel;
            return next_value;
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
                const f32 wheel_speed_m_s = magnitude2DF32(wheel_vx, wheel_vy);
                max_command_wheel_speed_m_s =
                    (wheel_speed_m_s > max_command_wheel_speed_m_s) ? wheel_speed_m_s : max_command_wheel_speed_m_s;
            }
            return max_command_wheel_speed_m_s;
        }

        bool Chassis::shouldSuppressYawLockOmegaForZeroStopDecel(const Data &command_data)
        {
            const bool yaw_lock_control_requested =
                current_mode_flag_.is_lock_now_rot_z || current_mode_flag_.is_lock_to_rot_z;
            if (!yaw_lock_control_requested ||
                input_target_data_.zero_current_all ||
                current_mode_flag_.is_wheel_torque_free ||
                (input_target_data_.mode == Mode::kSteerAngleAndDriveSpeedMode))
            {
                yaw_lock_zero_stop_decel_context_active_ = false;
                yaw_lock_zero_stop_release_hold_elapsed_ms_ = 0U;
                return false;
            }

            const f32 target_trans_speed_m_s = magnitude2DF32(target_data_.vel_x, target_data_.vel_y);
            const f32 command_trans_speed_m_s = magnitude2DF32(command_data.vel_x, command_data.vel_y);
            const bool manual_yaw_requested =
                ((input_target_data_.mode == Mode::kBodySpeedLockNowRotZWithNoOmegaZMode) ||
                 (input_target_data_.mode == Mode::kWorldSpeedLockNowRotZWithNoOmegaZMode)) &&
                (fabsf(input_target_data_.omega_z) > 1.0e-6f);
            const bool yaw_lock_zero_stop_hold_active =
                yaw_lock_zero_stop_decel_context_active_ || (yaw_lock_zero_stop_release_hold_elapsed_ms_ > 0U);
            const bool zero_omega_without_existing_hold =
                (fabsf(command_data.omega_z) <= 1.0e-6f) && !yaw_lock_zero_stop_hold_active;
            if ((target_trans_speed_m_s > getNearZeroEnterSpeedMps()) ||
                manual_yaw_requested ||
                zero_omega_without_existing_hold)
            {
                yaw_lock_zero_stop_decel_context_active_ = false;
                yaw_lock_zero_stop_release_hold_elapsed_ms_ = 0U;
                return false;
            }

            const f32 wheel_radius_m = fabsf(runtime_strategy_cfg_.wheel_radius_m_);
            f32 max_residual_speed_m_s = 0.0f;
            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const f32 cached_residual_speed_m_s = fabsf(wheel.corrected_drive_omega_rad_s) * wheel_radius_m;
                const f32 live_residual_speed_m_s = fabsf(readDriveMotorOmegaRadS(wheel)) * wheel_radius_m;
                const f32 residual_speed_m_s =
                    (live_residual_speed_m_s > cached_residual_speed_m_s) ? live_residual_speed_m_s : cached_residual_speed_m_s;
                max_residual_speed_m_s = (residual_speed_m_s > max_residual_speed_m_s) ? residual_speed_m_s : max_residual_speed_m_s;
            }

            const u32 release_hold_ms = runtime_strategy_cfg_.yaw_lock_zero_stop_release_hold_ms;
            if (max_residual_speed_m_s <= getNearZeroEnterSpeedMps())
            {
                if (!yaw_lock_zero_stop_decel_context_active_)
                {
                    yaw_lock_zero_stop_release_hold_elapsed_ms_ = 0U;
                    return false;
                }

                if (release_hold_ms == 0U)
                {
                    yaw_lock_zero_stop_decel_context_active_ = false;
                    yaw_lock_zero_stop_release_hold_elapsed_ms_ = 0U;
                    return false;
                }

                if (yaw_lock_zero_stop_release_hold_elapsed_ms_ < release_hold_ms)
                {
                    const u32 next_elapsed_ms =
                        (yaw_lock_zero_stop_release_hold_elapsed_ms_ > (0xFFFFFFFFU - period_ms_))
                            ? 0xFFFFFFFFU
                            : (yaw_lock_zero_stop_release_hold_elapsed_ms_ + period_ms_);
                    yaw_lock_zero_stop_release_hold_elapsed_ms_ = next_elapsed_ms;
                }

                if (yaw_lock_zero_stop_release_hold_elapsed_ms_ < release_hold_ms)
                {
                    return true;
                }

                yaw_lock_zero_stop_decel_context_active_ = false;
                yaw_lock_zero_stop_release_hold_elapsed_ms_ = 0U;
                return false;
            }

            const f32 last_trans_speed_m_s = magnitude2DF32(last_planned_data_.vel_x, last_planned_data_.vel_y);
            const bool translation_decel_context =
                (command_trans_speed_m_s > getNearZeroEnterSpeedMps()) ||
                (last_trans_speed_m_s > getNearZeroExitSpeedMps());
            if (translation_decel_context)
            {
                yaw_lock_zero_stop_decel_context_active_ = true;
            }

            if (!yaw_lock_zero_stop_decel_context_active_)
            {
                yaw_lock_zero_stop_release_hold_elapsed_ms_ = 0U;
                return false;
            }

            if (max_residual_speed_m_s > getNearZeroExitSpeedMps())
            {
                yaw_lock_zero_stop_release_hold_elapsed_ms_ = 0U;
            }
            return true;
        }

        bool Chassis::shouldPreviewSuppressYawLockOmegaForZeroStopDecel(const Data &command_data) const
        {
            const bool yaw_lock_control_requested =
                current_mode_flag_.is_lock_now_rot_z || current_mode_flag_.is_lock_to_rot_z;
            if (!yaw_lock_control_requested ||
                input_target_data_.zero_current_all ||
                current_mode_flag_.is_wheel_torque_free ||
                (input_target_data_.mode == Mode::kSteerAngleAndDriveSpeedMode))
            {
                return false;
            }

            const f32 target_trans_speed_m_s = magnitude2DF32(target_data_.vel_x, target_data_.vel_y);
            const f32 command_trans_speed_m_s = magnitude2DF32(command_data.vel_x, command_data.vel_y);
            const bool manual_yaw_requested =
                ((input_target_data_.mode == Mode::kBodySpeedLockNowRotZWithNoOmegaZMode) ||
                 (input_target_data_.mode == Mode::kWorldSpeedLockNowRotZWithNoOmegaZMode)) &&
                (fabsf(input_target_data_.omega_z) > 1.0e-6f);
            const bool yaw_lock_zero_stop_hold_active =
                yaw_lock_zero_stop_decel_context_active_ || (yaw_lock_zero_stop_release_hold_elapsed_ms_ > 0U);
            const bool zero_omega_without_existing_hold =
                (fabsf(command_data.omega_z) <= 1.0e-6f) && !yaw_lock_zero_stop_hold_active;
            if ((target_trans_speed_m_s > getNearZeroEnterSpeedMps()) ||
                manual_yaw_requested ||
                zero_omega_without_existing_hold)
            {
                return false;
            }

            const f32 wheel_radius_m = fabsf(runtime_strategy_cfg_.wheel_radius_m_);
            f32 max_residual_speed_m_s = 0.0f;
            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const f32 cached_residual_speed_m_s = fabsf(wheel.corrected_drive_omega_rad_s) * wheel_radius_m;
                const f32 live_residual_speed_m_s = fabsf(readDriveMotorOmegaRadS(wheel)) * wheel_radius_m;
                const f32 residual_speed_m_s =
                    (live_residual_speed_m_s > cached_residual_speed_m_s) ? live_residual_speed_m_s : cached_residual_speed_m_s;
                max_residual_speed_m_s = (residual_speed_m_s > max_residual_speed_m_s) ? residual_speed_m_s : max_residual_speed_m_s;
            }

            if (max_residual_speed_m_s <= getNearZeroEnterSpeedMps())
            {
                if (!yaw_lock_zero_stop_decel_context_active_)
                {
                    return false;
                }

                const u32 release_hold_ms = runtime_strategy_cfg_.yaw_lock_zero_stop_release_hold_ms;
                if (release_hold_ms == 0U)
                {
                    return false;
                }

                const u32 next_elapsed_ms =
                    (yaw_lock_zero_stop_release_hold_elapsed_ms_ > (0xFFFFFFFFU - period_ms_))
                        ? 0xFFFFFFFFU
                        : (yaw_lock_zero_stop_release_hold_elapsed_ms_ + period_ms_);
                return next_elapsed_ms < release_hold_ms;
            }

            const f32 last_trans_speed_m_s = magnitude2DF32(last_planned_data_.vel_x, last_planned_data_.vel_y);
            const bool translation_decel_context =
                (command_trans_speed_m_s > getNearZeroEnterSpeedMps()) ||
                (last_trans_speed_m_s > getNearZeroExitSpeedMps());
            return yaw_lock_zero_stop_decel_context_active_ || translation_decel_context;
        }

        bool Chassis::shouldActivateLaunchHold() const
        {
            if (!runtime_strategy_cfg_.enable_low_speed_drive_suppression)
            {
                return false;
            }

            if (runtime_strategy_cfg_.low_speed_drive_suppression.min_scale > 1.0e-6f)
            {
                return false;
            }

            const f32 command_speed_m_s = computeMaxCommandWheelSpeedMps(target_data_);
            if (command_speed_m_s <= getNearZeroExitSpeedMps())
            {
                return false;
            }

            if (xpark_gate_active_)
            {
                return true;
            }

            const f32 planned_speed_m_s = computeMaxCommandWheelSpeedMps(last_planned_data_);
            // 正常 S 曲线起步只允许在“首拍仍完全静止”时进入一次等待门，
            // 放行出第一拍后就按普通速度规划继续，不再因为近零小速度反复重进。
            if (planned_speed_m_s > 1.0e-6f)
            {
                return false;
            }

            const f32 wheel_radius_m = (runtime_strategy_cfg_.wheel_radius_m_ >= 0.0f)
                                           ? runtime_strategy_cfg_.wheel_radius_m_
                                           : -runtime_strategy_cfg_.wheel_radius_m_;
            f32 max_residual_speed_m_s = 0.0f;
            for (u8 i = 0; i < 4; ++i)
            {
                const f32 residual_speed_m_s = fabsf(wheel_config_[i].corrected_drive_omega_rad_s) * wheel_radius_m;
                max_residual_speed_m_s = (residual_speed_m_s > max_residual_speed_m_s) ? residual_speed_m_s : max_residual_speed_m_s;
            }
            return max_residual_speed_m_s <= getNearZeroEnterSpeedMps();
        }

        bool Chassis::isLaunchHoldAligned(const SwervePlannerOutput &planner_output) const
        {
            const f32 close_rad = degToRadF32(runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg);
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

        f32 Chassis::computeLowSpeedDriveSuppressionScale(f32 abs_error_rad) const
        {
            const f32 close_rad = degToRadF32(runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg);
            const f32 min_scale = clampValue(runtime_strategy_cfg_.low_speed_drive_suppression.min_scale, 0.0f, 1.0f);
            return (abs_error_rad >= close_rad) ? min_scale : 1.0f;
        }

        void Chassis::computeLowSpeedDriveSuppressionScales(const SwervePlannerInput &planner_input, const f32 steering_errors_rad[4], f32 out_scales[4])
        {
            for (u8 i = 0; i < 4; ++i)
            {
                out_scales[i] = 1.0f;
            }

            max_residual_speed_m_s_ = planner_input.max_residual_speed_m_s;
            low_speed_drive_suppression_bypassed_by_residual_speed_ = false;

            if (!runtime_strategy_cfg_.enable_low_speed_drive_suppression)
            {
                return;
            }

            const f32 bypass_enter_speed_m_s = getNearZeroExitSpeedMps();
            const f32 bypass_exit_speed_m_s = getNearZeroEnterSpeedMps();
            if (low_speed_residual_bypass_active_)
            {
                low_speed_residual_bypass_active_ = planner_input.max_residual_speed_m_s > bypass_exit_speed_m_s;
            }
            else
            {
                low_speed_residual_bypass_active_ = planner_input.max_residual_speed_m_s > bypass_enter_speed_m_s;
            }

            if (low_speed_residual_bypass_active_)
            {
                low_speed_drive_suppression_bypassed_by_residual_speed_ = true;
                return;
            }

            f32 max_abs = 0.0f;
            for (u8 i = 0; i < 4; ++i)
            {
                if (steering_errors_rad[i] > max_abs)
                {
                    max_abs = steering_errors_rad[i];
                }
            }
            const f32 scale = clampValue(computeLowSpeedDriveSuppressionScale(max_abs), 0.0f, 1.0f);
            for (u8 i = 0; i < 4; ++i)
            {
                out_scales[i] = scale;
            }
        }

        Chassis::SwervePlannerInput Chassis::makeSwervePlannerInput(const Data &command_data)
        {
            SwervePlannerInput planner_input{};
            planner_input.command = command_data;
            const bool yaw_lock_control_requested =
                current_mode_flag_.is_lock_now_rot_z || current_mode_flag_.is_lock_to_rot_z;
            const f32 command_wheel_speed_for_steer_gate = computeMaxCommandWheelSpeedMps(command_data);
            const bool yaw_lock_steer_intent_preview =
                yaw_lock_control_requested &&
                (magnitude2DF32(command_data.vel_x, command_data.vel_y) <= getNearZeroEnterSpeedMps()) &&
                (command_wheel_speed_for_steer_gate <= getNearZeroEnterSpeedMps()) &&
                (computeMaxCommandWheelSpeedMps(target_data_) > getNearZeroEnterSpeedMps());
            f32 max_residual_speed_for_steer_preview_m_s = 0.0f;
            if (yaw_lock_steer_intent_preview)
            {
                const f32 wheel_radius_m = fabsf(runtime_strategy_cfg_.wheel_radius_m_);
                for (u8 i = 0; i < 4; ++i)
                {
                    const f32 residual_speed_m_s = fabsf(wheel_config_[i].corrected_drive_omega_rad_s) * wheel_radius_m;
                    max_residual_speed_for_steer_preview_m_s =
                        (residual_speed_m_s > max_residual_speed_for_steer_preview_m_s) ? residual_speed_m_s : max_residual_speed_for_steer_preview_m_s;
                }
            }
            const bool allow_yaw_lock_steer_intent_preview =
                yaw_lock_steer_intent_preview &&
                !yaw_lock_zero_stop_decel_context_active_ &&
                (max_residual_speed_for_steer_preview_m_s <= getNearZeroEnterSpeedMps());
            const Data &steer_intent_data = allow_yaw_lock_steer_intent_preview ? target_data_ : command_data;

            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                const f32 current_corrected_local_total_rad = wheel.corrected_steer_motor_total_angle_rad;
                planner_input.current_oa_total_rad[i] = mapWheelCorrectedLocalToOaTotal(wheel, current_corrected_local_total_rad);

                const f32 wheel_vx = command_data.vel_x + command_data.omega_z * wheel.pos_y_m;
                const f32 wheel_vy = command_data.vel_y - command_data.omega_z * wheel.pos_x_m;
                const f32 wheel_speed_m_s = magnitude2DF32(wheel_vx, wheel_vy);
                const f32 steer_intent_vx = steer_intent_data.vel_x + steer_intent_data.omega_z * wheel.pos_y_m;
                const f32 steer_intent_vy = steer_intent_data.vel_y - steer_intent_data.omega_z * wheel.pos_x_m;
                const f32 steer_intent_speed_m_s = magnitude2DF32(steer_intent_vx, steer_intent_vy);
                planner_input.wheel_vx_m_s[i] = wheel_vx;
                planner_input.wheel_vy_m_s[i] = wheel_vy;
                planner_input.wheel_speed_m_s[i] = wheel_speed_m_s;
                planner_input.steer_intent_wheel_vx_m_s[i] = steer_intent_vx;
                planner_input.steer_intent_wheel_vy_m_s[i] = steer_intent_vy;
                planner_input.steer_intent_wheel_speed_m_s[i] = steer_intent_speed_m_s;
                planner_input.max_command_wheel_speed_m_s =
                    (wheel_speed_m_s > planner_input.max_command_wheel_speed_m_s) ? wheel_speed_m_s : planner_input.max_command_wheel_speed_m_s;
                planner_input.max_steer_intent_wheel_speed_m_s =
                    (steer_intent_speed_m_s > planner_input.max_steer_intent_wheel_speed_m_s) ? steer_intent_speed_m_s : planner_input.max_steer_intent_wheel_speed_m_s;

                const f32 residual_speed_m_s = fabsf(wheel.corrected_drive_omega_rad_s) * runtime_strategy_cfg_.wheel_radius_m_;
                planner_input.residual_speed_m_s[i] = residual_speed_m_s;
                planner_input.max_residual_speed_m_s =
                    (residual_speed_m_s > planner_input.max_residual_speed_m_s) ? residual_speed_m_s : planner_input.max_residual_speed_m_s;
            }

            const f32 xpark_command_enter_speed = getXParkCommandEnterSpeedMps();
            const f32 xpark_command_exit_speed = getXParkCommandExitSpeedMps();
            const f32 xpark_residual_enter_speed = getNearZeroEnterSpeedMps();
            const f32 xpark_residual_exit_speed = getNearZeroExitSpeedMps();
            // X-Park 有两层门：
            // 1. 进入门：目标速度已经进入 X-Park command 门，且实际残余速度也进入通用 near-zero 门。
            // 2. 保持/退出门：一旦 xpark_gate_active_ 锁存，只看目标速度是否仍在 X-Park command 退出门内。
            // residual 在进入后不再踢出 X-Park，否则轮子刚被锁到 X 姿态后的反馈扰动会反复打断保持态。
            const bool xpark_target_stationary = xpark_gate_active_
                                                     ? (planner_input.max_steer_intent_wheel_speed_m_s <= xpark_command_exit_speed)
                                                     : (planner_input.max_steer_intent_wheel_speed_m_s <= xpark_command_enter_speed);
            const bool xpark_residual_stationary = xpark_gate_active_
                                                       ? (planner_input.max_residual_speed_m_s <= xpark_residual_exit_speed)
                                                       : (planner_input.max_residual_speed_m_s <= xpark_residual_enter_speed);
            const bool xpark_entry_ready = xpark_target_stationary && xpark_residual_stationary;
            planner_input.command_stationary_intent = xpark_target_stationary;

            if (!xpark_gate_active_)
            {
                if (xpark_entry_ready)
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
                }
            }
            else if (!xpark_target_stationary)
            {
                xpark_stationary_hold_ms_ = 0U;
                xpark_gate_active_ = false;
            }

            const bool force_uniform_steer_drive = (input_target_data_.mode == Mode::kSteerAngleAndDriveSpeedMode);
            planner_input.allow_xpark_pose = (!force_uniform_steer_drive) && xpark_gate_active_ && xpark_target_stationary;
            planner_input.force_uniform_steer_drive = force_uniform_steer_drive;
            planner_input.uniform_steer_oa_mod_rad = wrapTo2PiF32(degToRadF32(input_target_data_.steer_lock_angle_deg));
            planner_input.uniform_drive_omega_abs = fabsf(input_target_data_.drive_lock_speed_m_s) / runtime_strategy_cfg_.wheel_radius_m_;
            planner_input.uniform_drive_sign = (input_target_data_.drive_lock_speed_m_s >= 0.0f) ? 1.0f : -1.0f;
            return planner_input;
        }

        Chassis::SwervePlannerOutput Chassis::planSwerveModules(const SwervePlannerInput &planner_input)
        {
            SwervePlannerOutput planner_output{};
            const f32 planner_command_speed_m_s = magnitude2DF32(planner_input.command.vel_x, planner_input.command.vel_y);
            const bool last_planned_has_translation =
                magnitude2DF32(last_planned_data_.vel_x, last_planned_data_.vel_y) > 1.0e-6f;
            const f32 planner_reference_dir_rad = trans_dir_ref_valid_
                                                     ? trans_dir_ref_rad_
                                                     : (last_planned_has_translation
                                                            ? atan2f(last_planned_data_.vel_y, last_planned_data_.vel_x)
                                                            : 0.0f);
            const bool planner_reverse_intent =
                (planner_command_speed_m_s > 1.0e-6f) && shouldActivateReverseIntent(planner_input.command.vel_x, planner_input.command.vel_y, planner_reference_dir_rad);

            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const f32 wheel_speed_m_s = planner_input.wheel_speed_m_s[i];
                const f32 steer_intent_wheel_speed_m_s = planner_input.steer_intent_wheel_speed_m_s[i];
                const bool steer_intent_stationary = steer_intent_wheel_speed_m_s <= getNearZeroEnterSpeedMps();

                if (steer_intent_stationary)
                {
                    planner_output.ideal_oa_total_rad[i] = (planner_input.allow_xpark_pose && runtime_strategy_cfg_.idle_posture_mode == IdlePostureMode::kXPark)
                                                               ? wrapTo2PiF32(getXParkAngle(wheel))
                                                               : wrapTo2PiF32(planner_input.current_oa_total_rad[i]);
                    planner_output.ideal_drive_omega_rad_s[i] = 0.0f;
                }
                else
                {
                    planner_output.ideal_oa_total_rad[i] = wrapTo2PiF32(atan2f(planner_input.steer_intent_wheel_vy_m_s[i], planner_input.steer_intent_wheel_vx_m_s[i]));
                    planner_output.ideal_drive_omega_rad_s[i] = wheel_speed_m_s / runtime_strategy_cfg_.wheel_radius_m_;
                }

                const f32 alt_target_oa_mod_rad = wrapTo2PiF32(planner_output.ideal_oa_total_rad[i] + kPi);
                const f32 candidate_a = nearestEquivalentAngleF32(planner_input.current_oa_total_rad[i], planner_output.ideal_oa_total_rad[i]);
                const f32 candidate_b = nearestEquivalentAngleF32(planner_input.current_oa_total_rad[i], alt_target_oa_mod_rad);

                f32 selected_oa_total_rad = candidate_a;
                f32 selected_drive_omega_rad_s = planner_output.ideal_drive_omega_rad_s[i];
                bool flipped = false;

                if (!steer_intent_stationary)
                {
                    if (runtime_strategy_cfg_.steering_strategy_mode == SteeringStrategyMode::kAlwaysForward)
                    {
                        flipped = false;
                    }
                    else
                    {
                        const f32 base_abs_deg = radToDegF32(fabsf(candidate_a - planner_input.current_oa_total_rad[i]));
                        const f32 flip_abs_deg = radToDegF32(fabsf(candidate_b - planner_input.current_oa_total_rad[i]));
                        if (reverse_intent_active_ || planner_reverse_intent)
                        {
                            const f32 prefer_margin_deg =
                                clampValue(runtime_strategy_cfg_.reverse_intent.flip_prefer_margin_deg, 0.0f, 180.0f);
                            if (flip_abs_deg + prefer_margin_deg < base_abs_deg)
                            {
                                flipped = true;
                            }
                            else if (base_abs_deg + prefer_margin_deg < flip_abs_deg)
                            {
                                flipped = false;
                            }
                            else
                            {
                                flipped = selected_flipped_solution_[i] || (flip_abs_deg < base_abs_deg);
                            }
                        }
                        else if (selected_flipped_solution_[i])
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
                    const f32 fixed_a = nearestEquivalentAngleF32(planner_input.current_oa_total_rad[i], planner_input.uniform_steer_oa_mod_rad);
                    const f32 fixed_b = nearestEquivalentAngleF32(planner_input.current_oa_total_rad[i], wrapTo2PiF32(planner_input.uniform_steer_oa_mod_rad + kPi));
                    const bool use_b = fabsf(shortestAngularDistanceF32(planner_input.current_oa_total_rad[i], fixed_b)) <
                                       fabsf(shortestAngularDistanceF32(planner_input.current_oa_total_rad[i], fixed_a));
                    selected_oa_total_rad = use_b ? fixed_b : fixed_a;
                    selected_drive_omega_rad_s = planner_input.uniform_drive_sign * (use_b ? -1.0f : 1.0f) * planner_input.uniform_drive_omega_abs;
                    flipped = use_b;
                }

                planner_output.selected_oa_total_rad[i] = selected_oa_total_rad;
                planner_output.flipped_drive_direction[i] = flipped;
                planner_output.steering_errors_rad[i] =
                    fabsf(shortestAngularDistanceF32(planner_input.current_oa_total_rad[i], selected_oa_total_rad));
                planner_output.projected_drive_omega_rad_s[i] = selected_drive_omega_rad_s;
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
                planner_output.high_speed_eta_max_s = (eta_s > planner_output.high_speed_eta_max_s) ? eta_s : planner_output.high_speed_eta_max_s;
            }

            f32 planned_steering_errors_rad[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            for (u8 i = 0; i < 4; ++i)
            {
                planned_steering_errors_rad[i] =
                    fabsf(shortestAngularDistanceF32(planner_output.planned_oa_total_rad[i],
                                                      planner_output.selected_oa_total_rad[i]));
            }

            const bool enable_steer_angle_feedforward =
                runtime_strategy_cfg_.enable_steer_angle_feedforward &&
                !planner_input.force_uniform_steer_drive &&
                !planner_input.allow_xpark_pose;
            const f32 steer_ff_lead_s = (runtime_strategy_cfg_.steer_angle_feedforward_lead_s > 0.0f)
                                            ? runtime_strategy_cfg_.steer_angle_feedforward_lead_s
                                            : 0.0f;
            const f32 steer_ff_max_lead_rad = fabsf(runtime_strategy_cfg_.steer_angle_feedforward_max_lead_rad);
            const f32 steer_ff_settle_error_rad = fabsf(runtime_strategy_cfg_.steer_angle_feedforward_settle_error_rad);
            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                f32 steer_cmd_oa_total_rad = planner_output.planned_oa_total_rad[i];
                if (enable_steer_angle_feedforward && (steer_ff_lead_s > 0.0f) && (steer_ff_max_lead_rad > 0.0f))
                {
                    f32 lead_rad = planner_output.planned_steer_rate_rad_s[i] * steer_ff_lead_s;
                    lead_rad = clampValue(lead_rad, -steer_ff_max_lead_rad, steer_ff_max_lead_rad);
                    if (steer_ff_settle_error_rad > 1.0e-6f)
                    {
                        const f32 settle_scale =
                            clampValue(planner_output.steering_errors_rad[i] / steer_ff_settle_error_rad, 0.0f, 1.0f);
                        lead_rad *= settle_scale;
                    }
                    steer_cmd_oa_total_rad += lead_rad;
                }

                planner_output.steer_cmd_oa_total_rad[i] = steer_cmd_oa_total_rad;
                planner_output.steer_cmd_corrected_local_total_rad[i] =
                    mapWheelOaTotalToCorrectedLocal(wheel, steer_cmd_oa_total_rad);
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

            f32 low_speed_scales[4] = {1.0f, 1.0f, 1.0f, 1.0f};
            const bool pure_yaw_motion_intent =
                (planner_command_speed_m_s <= getNearZeroEnterSpeedMps()) &&
                (planner_input.max_command_wheel_speed_m_s > getNearZeroEnterSpeedMps());
            computeLowSpeedDriveSuppressionScales(planner_input,
                                                  pure_yaw_motion_intent ? planned_steering_errors_rad : planner_output.steering_errors_rad,
                                                  low_speed_scales);

            const f32 translational_speed_m_s = planner_command_speed_m_s;
            f32 predicted_vel_x = 0.0f;
            f32 predicted_vel_y = 0.0f;
            f32 predicted_omega_z = 0.0f;
            if (estimatePlannedBodyTwist(planner_output.planned_oa_total_rad,
                                         planner_output.projected_drive_omega_rad_s,
                                         predicted_vel_x,
                                         predicted_vel_y,
                                         predicted_omega_z))
            {
                const f32 predicted_internal_vel_x = -predicted_vel_x;
                const f32 predicted_internal_vel_y = -predicted_vel_y;
                const f32 predicted_trans_speed_m_s = magnitude2DF32(predicted_internal_vel_x, predicted_internal_vel_y);
                if ((translational_speed_m_s > 1.0e-6f) && (predicted_trans_speed_m_s > 1.0e-6f))
                {
                    const f32 target_dir_rad = atan2f(planner_input.command.vel_y, planner_input.command.vel_x);
                    const f32 predicted_dir_rad = atan2f(predicted_internal_vel_y, predicted_internal_vel_x);
                    planner_output.high_speed_dir_err_deg =
                        radToDegF32(fabsf(shortestAngularDistanceF32(target_dir_rad, predicted_dir_rad)));
                }
            }

            planner_output.high_speed_suppression_scale = planner_input.force_uniform_steer_drive
                                                   ? 1.0f
                                                   : updateHighSpeedDriveSuppression(translational_speed_m_s,
                                                                                    planner_output.high_speed_eta_max_s,
                                                                                    planner_output.high_speed_dir_err_deg);
            if (planner_input.force_uniform_steer_drive)
            {
                high_speed_drive_suppression_scale_ = 1.0f;
                high_speed_drive_suppression_active_ = false;
            }
            planner_output.high_speed_suppression_active = high_speed_drive_suppression_active_;

            f32 planner_drive_targets_rad_s[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            f32 max_abs_planner_drive_target_rad_s = 0.0f;
            for (u8 i = 0; i < 4; ++i)
            {
                if (planner_input.force_uniform_steer_drive)
                {
                    planner_output.low_speed_suppression_scale[i] = 1.0f;
                }
                else
                {
                    planner_output.low_speed_suppression_scale[i] = low_speed_scales[i];
                }

                const f32 drive_scale =
                    clampValue(planner_output.low_speed_suppression_scale[i] * planner_output.high_speed_suppression_scale, 0.0f, 1.0f);
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
                out_frame.steer_cmd_corrected_local_total_rad[i] = planner_output.steer_cmd_corrected_local_total_rad[i];
                out_frame.steer_cmd_oa_total_rad[i] = planner_output.steer_cmd_oa_total_rad[i];
                out_frame.steer_rate_rad_s[i] = planner_output.planned_steer_rate_rad_s[i];
                out_frame.drive_omega_rad_s[i] = planner_output.final_drive_omega_rad_s[i];
                out_frame.flipped_drive_direction[i] = planner_output.flipped_drive_direction[i];
            }
        }

        void Chassis::storePlannedActuatorFrame(const SwervePlannerOutput &planner_output, const ActuatorCommandFrame &command_frame)
        {
            // planner_output 里既保留了理想运动学解，也保留了翻转解选择、低速/高速抑制后的最终执行解。
            // 这里把“下一阶段和调试观测真正要读的版本”统一写回成员缓存，避免后续再各自回头解释 planner_output。
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                low_speed_drive_suppression_scale_[i] =
                    clampValue(planner_output.low_speed_suppression_scale[i] * planner_output.high_speed_suppression_scale, 0.0f, 1.0f);
                wheel.target_steer_motor_total_angle_rad = command_frame.steer_cmd_corrected_local_total_rad[i];
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
            high_speed_eta_max_s_ = planner_output.high_speed_eta_max_s;
            high_speed_dir_err_deg_ = planner_output.high_speed_dir_err_deg;
            high_speed_drive_suppression_scale_ = planner_output.high_speed_suppression_scale;
            high_speed_drive_suppression_active_ = planner_output.high_speed_suppression_active;
        }

        f32 Chassis::computeHomingAlignTargetCorrectedLocalTotal(const WheelConfig &wheel) const
        {
            // 回零完成后的“对正前方”语义最终仍要落实到 corrected local 连续角。
            // 这里先回到 OA 语义寻找离当前最近的 0° 等效角，再映射回 corrected local。
            const f32 current_corrected_local_total_rad = wheel.corrected_steer_motor_total_angle_rad;
            const f32 current_oa_total_rad = mapWheelCorrectedLocalToOaTotal(wheel, current_corrected_local_total_rad);
            const f32 align_target_oa_total_rad = nearestEquivalentAngleF32(current_oa_total_rad, 0.0f);
            return mapWheelOaTotalToCorrectedLocal(wheel, align_target_oa_total_rad);
        }

        void Chassis::setModeFlag()
        {
            // 将公开 Mode 压缩成线程主流程更容易消费的布尔标志。
            // 后续 planner / gate / output 基本只看这些标志，而不用在每个分支里重复解释完整枚举语义。
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
            // 调试面板长期沿用一套 raw mode 编号，这里集中做兼容映射。
            // 线程其余部分统一消费 DebugMode，避免“面板协议值”扩散进控制分支。
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
            case 9:
                return DebugMode::kSteerDegAndDriveSpeed;
            case 20:
                return DebugMode::kTorqueFree;
            case 21:
                return DebugMode::kAlignForward;
            case 22:
                return DebugMode::kHomingObserve;
            case 30:
                return DebugMode::kSingleWheelIsolated;
            default:
                return DebugMode::kTorqueFree;
            }
        }

#if JIA_CHASSIS_ENABLE_DEBUG_OVERRIDE
        void Chassis::applyDebugTargetOverride(DebugMode mode)
        {
            // Debug 左摇杆沿对外平移语义接管：上推朝当前 2/3 面，左推朝当前 3/4 面。
            // 映射到内部 body 命令时使用 -left_x -> vel_x、-left_y -> vel_y；
            // 右摇杆 omega_z 保持现有符号约定不变。
            f32 target_vel_x = -airjoy_data_.left_x * runtime_strategy_cfg_.max_vel_x_;
            f32 target_vel_y = -airjoy_data_.left_y * runtime_strategy_cfg_.max_vel_y_;
            const f32 right_x_cmd = -airjoy_data_.right_x;
            f32 target_omega_z = -right_x_cmd * runtime_strategy_cfg_.max_omega_z_;

            const DebugOmegaZInjectionMode omega_z_injection_mode =
                (debug_control_.injection.omega_z_injection_mode_raw <= static_cast<u8>(DebugOmegaZInjectionMode::kSine))
                    ? static_cast<DebugOmegaZInjectionMode>(debug_control_.injection.omega_z_injection_mode_raw)
                    : DebugOmegaZInjectionMode::kOff;
            switch (omega_z_injection_mode)
            {
            case DebugOmegaZInjectionMode::kSine:
                target_omega_z = sineWaveGeneratorF32(time_ms_ / 1000.0f,
                                                      debug_control_.injection.omega_z_sine_amplitude,
                                                      debug_control_.injection.omega_z_sine_frequency_hz,
                                                      0.0f,
                                                      debug_control_.injection.omega_z_sine_offset);
                break;
            case DebugOmegaZInjectionMode::kStep:
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
                break;
            case DebugOmegaZInjectionMode::kOff:
            default:
                break;
            }

            debug_control_.common.mode_resolved_raw = static_cast<u8>(mode);
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
                setTargetBodySpeedLockToRotZMode(target_vel_x, target_vel_y, debug_control_.injection.lock_rot_z);
                break;
            case DebugMode::kWorldLockTo:
                setTargetWorldSpeedLockToRotZMode(target_vel_x, target_vel_y, debug_control_.injection.lock_rot_z);
                break;
            case DebugMode::kBodyLockNowWithNoOmegaZ:
                setTargetBodySpeedLockNowRotZWithNoOmegaZMode(target_vel_x, target_vel_y, target_omega_z);
                break;
            case DebugMode::kWorldLockNowWithNoOmegaZ:
                setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(target_vel_x, target_vel_y, target_omega_z);
                break;
            case DebugMode::kSteerDegAndDriveSpeed:
            {
                const f32 steer_angle_deg =
                    90.0f + clampValue(airjoy_data_.left_x, -1.0f, 1.0f) * debug_control_.injection.steer_deg_limit;
                const f32 drive_speed_m_s = clampValue(airjoy_data_.right_x, -1.0f, 1.0f) * debug_control_.injection.drive_speed_m_s_limit;
                setSteerDegAndDriveSpeed(steer_angle_deg, drive_speed_m_s);
                break;
            }
            case DebugMode::kAlignForward:
            case DebugMode::kHomingObserve:
            case DebugMode::kSingleWheelIsolated:
                setTargetBodySpeedMode(0.0f, 0.0f, 0.0f);
                break;
            default:
                setWheelTorqueFreeMode();
                break;
            }
        }

        void Chassis::clearPlannedMotionForModuleOverride()
        {
            // 模块级 override 接管时，整车 planner 的速度/加速度历史不应继续残留；
            // 否则退出 override 后，旧 planned_data_ 可能被误当成仍有效的整车运动意图。
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
            // 把四个模块都拉回“保持当前舵角、驱动清零”的安全基线。
            // preserve_soft_wheel_rate 用于保留被观察轮的软规划历史，避免切换调试模式时突然断层。
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

        void Chassis::applyAlignForwardDebugOverride()
        {
            // AlignForward 是最纯粹的舵向语义检查模式：
            // 不关心驱动，只要求所有轮子都面对 OA=0，便于核对零偏和安装方向。
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                wheel.target_steer_motor_total_angle_rad = mapWheelOaTotalToCorrectedLocal(wheel, 0.0f);
                planned_data_.steer_angle_oa_rad[i] = 0.0f;
            }
        }

        void Chassis::applyHomingObserveDebugOverride()
        {
            // HomingObserve 不主动推模块运动，只把目标锁在当前反馈上。
            // 这样可以单独观察边沿、零偏和恢复状态机，而不会被新的控制目标干扰。
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

        void Chassis::syncSingleWheelCommandTemplates()
        {
            // 单轮直控允许在运行中切命令类型。
            // 这里负责在“类型变化的边沿”同步默认限幅、阶跃值和局部 planner 状态，避免不同模式共用脏缓存。
            SingleWheelAxisControl &steer = debug_control_.single_wheel.steer;
            SingleWheelAxisControl &drive = debug_control_.single_wheel.drive;

            const DirectSteerCommandType steer_type = sanitizeDirectSteerCommandType(steer.command_type_raw);
            const DirectDriveCommandType drive_type = sanitizeDirectDriveCommandType(drive.command_type_raw);
            steer.command_type_raw = static_cast<u8>(steer_type);
            drive.command_type_raw = static_cast<u8>(drive_type);
            steer.input_axis_raw = static_cast<u8>(sanitizeSingleWheelInputAxis(steer.input_axis_raw));
            drive.input_axis_raw = static_cast<u8>(sanitizeSingleWheelInputAxis(drive.input_axis_raw));
            steer.planner_mode_raw = static_cast<u8>(sanitizeSingleWheelPlannerMode(steer.planner_mode_raw));
            drive.planner_mode_raw = static_cast<u8>(sanitizeSingleWheelPlannerMode(drive.planner_mode_raw));

            if (steer.command_type_raw != single_wheel_last_steer_command_type_raw_)
            {
                if (single_wheel_last_steer_command_type_raw_ == 0xFFU)
                {
                    if (steer.command_limit <= 0.0f)
                    {
                        steer.command_limit = getDirectSteerDefaultLimit(steer_type);
                    }
                    if (steer.step_value == 0.0f)
                    {
                        steer.step_value = getDirectSteerDefaultStepValue(steer_type);
                    }
                }
                else
                {
                    steer.command_value = 0.0f;
                    steer.command_limit = getDirectSteerDefaultLimit(steer_type);
                    steer.step_value = getDirectSteerDefaultStepValue(steer_type);
                    resetSingleWheelAxisPlannerRuntime(single_wheel_steer_planner_state_);
                }
                single_wheel_last_steer_command_type_raw_ = steer.command_type_raw;
            }

            if (drive.command_type_raw != single_wheel_last_drive_command_type_raw_)
            {
                if (single_wheel_last_drive_command_type_raw_ == 0xFFU)
                {
                    if (drive.command_limit <= 0.0f)
                    {
                        drive.command_limit = getDirectDriveDefaultLimit(drive_type);
                    }
                    if (drive.step_value == 0.0f)
                    {
                        drive.step_value = getDirectDriveDefaultStepValue(drive_type);
                    }
                }
                else
                {
                    drive.command_value = 0.0f;
                    drive.command_limit = getDirectDriveDefaultLimit(drive_type);
                    drive.step_value = getDirectDriveDefaultStepValue(drive_type);
                    resetSingleWheelAxisPlannerRuntime(single_wheel_drive_planner_state_);
                }
                single_wheel_last_drive_command_type_raw_ = drive.command_type_raw;
            }
        }

        f32 Chassis::readSingleWheelInputAxisValue(u8 input_axis_raw) const
        {
            // 单轮直控仍沿用 airjoy 原始摇杆值，但会统一钳到 [-1, 1]，让后续 direct/step/planner 逻辑只处理归一化输入。
            switch (sanitizeSingleWheelInputAxis(input_axis_raw))
            {
            case SingleWheelInputAxis::kLeftY:
                return clampValue(airjoy_data_.left_y, -1.0f, 1.0f);
            case SingleWheelInputAxis::kRightX:
                return clampValue(airjoy_data_.right_x, -1.0f, 1.0f);
            case SingleWheelInputAxis::kRightY:
                return clampValue(airjoy_data_.right_y, -1.0f, 1.0f);
            case SingleWheelInputAxis::kLeftX:
            default:
                return clampValue(airjoy_data_.left_x, -1.0f, 1.0f);
            }
        }

        void Chassis::resetSingleWheelAxisPlannerRuntime(SingleWheelAxisPlannerRuntime &runtime)
        {
            // 局部 planner runtime 只服务当前轮、当前命令类型和当前 planner 模式；
            // 这些维度任一变化，都应从空状态重新建立。
            runtime = {};
        }

        f32 Chassis::shapeSingleWheelSteerCommand(u8 wheel_idx, const SingleWheelAxisControl &axis_cfg, f32 target_value)
        {
            // 单轮 steer 直控既支持“原样直通”，也支持梯形/S 曲线整形。
            // 输出的是调试链路最终允许给这个轮子的 steer 目标，而不是面板直接输入值。
            const SingleWheelPlannerMode planner_mode = sanitizeSingleWheelPlannerMode(axis_cfg.planner_mode_raw);
            const DirectSteerCommandType command_type = sanitizeDirectSteerCommandType(axis_cfg.command_type_raw);
            if ((planner_mode == SingleWheelPlannerMode::kOff) ||
                !isSingleWheelSteerPlannerSupported(command_type))
            {
                resetSingleWheelAxisPlannerRuntime(single_wheel_steer_planner_state_);
                return target_value;
            }

            SingleWheelAxisPlannerRuntime &runtime = single_wheel_steer_planner_state_;
            if ((runtime.last_wheel_idx != wheel_idx) ||
                (runtime.last_command_type_raw != axis_cfg.command_type_raw) ||
                (runtime.last_planner_mode_raw != axis_cfg.planner_mode_raw))
            {
                resetSingleWheelAxisPlannerRuntime(runtime);
            }

            f32 shaped_value = target_value;
            if (planner_mode == SingleWheelPlannerMode::kSCurve)
            {
                shaped_value = limitValueByJerkProfile(target_value,
                                                       runtime.last_output_value,
                                                       runtime.jerk_state,
                                                       axis_cfg.scurve.acc_acc,
                                                       axis_cfg.scurve.acc_dec,
                                                       axis_cfg.scurve.jerk_acc,
                                                       axis_cfg.scurve.jerk_dec,
                                                       axis_cfg.scurve.settle_vel_eps,
                                                       axis_cfg.scurve.settle_acc_eps);
            }
            else
            {
                shaped_value = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(target_value,
                                                                              runtime.last_output_value,
                                                                              period_,
                                                                              fabsf(axis_cfg.trapezoid.acc),
                                                                              fabsf(axis_cfg.trapezoid.dec));
            }

            runtime.last_output_value = shaped_value;
            runtime.last_wheel_idx = wheel_idx;
            runtime.last_command_type_raw = axis_cfg.command_type_raw;
            runtime.last_planner_mode_raw = axis_cfg.planner_mode_raw;
            runtime.initialized = true;
            return shaped_value;
        }

        f32 Chassis::shapeSingleWheelDriveOmegaRadS(u8 wheel_idx, const SingleWheelAxisControl &axis_cfg, f32 target_omega_rad_s)
        {
            // drive 轴整形内部先转成线速度做 planner，是为了让加减速参数保持更接近整车语义的直觉单位。
            // 输出前再折回轮速，保持下游驱动接口不变。
            const SingleWheelPlannerMode planner_mode = sanitizeSingleWheelPlannerMode(axis_cfg.planner_mode_raw);
            const DirectDriveCommandType command_type = sanitizeDirectDriveCommandType(axis_cfg.command_type_raw);
            if ((planner_mode == SingleWheelPlannerMode::kOff) ||
                !isSingleWheelDrivePlannerSupported(command_type))
            {
                resetSingleWheelAxisPlannerRuntime(single_wheel_drive_planner_state_);
                return target_omega_rad_s;
            }

            SingleWheelAxisPlannerRuntime &runtime = single_wheel_drive_planner_state_;
            if ((runtime.last_wheel_idx != wheel_idx) ||
                (runtime.last_command_type_raw != axis_cfg.command_type_raw) ||
                (runtime.last_planner_mode_raw != axis_cfg.planner_mode_raw))
            {
                resetSingleWheelAxisPlannerRuntime(runtime);
            }

            const f32 wheel_radius_m = (fabsf(runtime_strategy_cfg_.wheel_radius_m_) > 1.0e-6f) ? fabsf(runtime_strategy_cfg_.wheel_radius_m_) : 1.0f;
            const f32 target_linear_m_s = target_omega_rad_s * wheel_radius_m;
            f32 shaped_linear_m_s = target_linear_m_s;
            if (planner_mode == SingleWheelPlannerMode::kSCurve)
            {
                shaped_linear_m_s = limitValueByJerkProfile(target_linear_m_s,
                                                            runtime.last_output_value,
                                                            runtime.jerk_state,
                                                            axis_cfg.scurve.acc_acc,
                                                            axis_cfg.scurve.acc_dec,
                                                            axis_cfg.scurve.jerk_acc,
                                                            axis_cfg.scurve.jerk_dec,
                                                            axis_cfg.scurve.settle_vel_eps,
                                                            axis_cfg.scurve.settle_acc_eps);
            }
            else
            {
                shaped_linear_m_s =
                    limit1DSignalRateByTimeSeparateAbsIncAndDecF32(target_linear_m_s,
                                                                   runtime.last_output_value,
                                                                   period_,
                                                                   fabsf(axis_cfg.trapezoid.acc),
                                                                   fabsf(axis_cfg.trapezoid.dec));
            }

            runtime.last_output_value = shaped_linear_m_s;
            runtime.last_wheel_idx = wheel_idx;
            runtime.last_command_type_raw = axis_cfg.command_type_raw;
            runtime.last_planner_mode_raw = axis_cfg.planner_mode_raw;
            runtime.initialized = true;
            return shaped_linear_m_s / wheel_radius_m;
        }

        f32 Chassis::resolveSingleWheelDriveStepTargetRpm(u8 wheel_idx, f32 fallback_target_rpm)
        {
            // 单轮 drive 阶跃发生器允许在观察轮上持续输出“保持/空档/反向”的节拍序列，
            // 方便看 PID、虚拟负载和断电重连后的执行表现。
            if (wheel_idx >= 4U)
            {
                return fallback_target_rpm;
            }

            const DebugDriveStepGeneratorConfig &cfg = debug_drive_step_generator_[wheel_idx];
            DebugDriveStepGeneratorRuntime &runtime = drive_step_generator_runtime_[wheel_idx];
            if (!cfg.enable)
            {
                runtime = {};
                return fallback_target_rpm;
            }

            if (!runtime.initialized)
            {
                runtime.initialized = true;
                runtime.output_positive = cfg.start_positive;
                runtime.phase = (fabsf(cfg.hold_ms) <= 1.0e-6f) ? 0U : 1U;
                runtime.elapsed_ms = 0.0f;
            }

            const f32 hold_ms = (cfg.hold_ms > 0.0f) ? cfg.hold_ms : 0.0f;
            const f32 rest_ms = (cfg.rest_ms > 0.0f) ? cfg.rest_ms : 0.0f;

            if (runtime.phase == 1U)
            {
                runtime.elapsed_ms += static_cast<f32>(period_ms_);
                if (runtime.elapsed_ms >= hold_ms)
                {
                    runtime.elapsed_ms = 0.0f;
                    if (cfg.one_shot)
                    {
                        runtime.phase = 2U;
                    }
                    else if (rest_ms > 0.0f)
                    {
                        runtime.phase = 0U;
                    }
                    else if (cfg.alternate_sign)
                    {
                        runtime.output_positive = !runtime.output_positive;
                    }
                }
            }
            else if (runtime.phase == 0U)
            {
                runtime.elapsed_ms += static_cast<f32>(period_ms_);
                if (runtime.elapsed_ms >= rest_ms)
                {
                    runtime.elapsed_ms = 0.0f;
                    runtime.phase = 1U;
                    if (cfg.alternate_sign)
                    {
                        runtime.output_positive = !runtime.output_positive;
                    }
                }
            }
            else if (runtime.phase == 2U)
            {
                if (cfg.auto_restart)
                {
                    runtime.initialized = false;
                    return resolveSingleWheelDriveStepTargetRpm(wheel_idx, fallback_target_rpm);
                }
                return 0.0f;
            }

            debug_drive_load_trace_.step_phase = static_cast<f32>(runtime.phase);
            debug_drive_load_trace_.stepgen_enable = 1.0f;

            if (runtime.phase != 1U)
            {
                return 0.0f;
            }

            const f32 direction = runtime.output_positive ? 1.0f : -1.0f;
            return direction * fabsf(cfg.step_target_rpm);
        }

        Chassis::DirectActuatorCommandSnapshot Chassis::resolveSingleWheelCommand(u8 wheel_idx)
        {
            syncSingleWheelCommandTemplates();

            DirectActuatorCommandSnapshot command{};
            command.wheel_idx = wheel_idx;
            command.steer_input_mode = static_cast<u8>(sanitizeDirectAxisInputMode(debug_control_.single_wheel.steer.input_mode_raw));
            command.drive_input_mode = static_cast<u8>(sanitizeDirectAxisInputMode(debug_control_.single_wheel.drive.input_mode_raw));
            command.steer_input_axis = static_cast<u8>(sanitizeSingleWheelInputAxis(debug_control_.single_wheel.steer.input_axis_raw));
            command.drive_input_axis = static_cast<u8>(sanitizeSingleWheelInputAxis(debug_control_.single_wheel.drive.input_axis_raw));
            command.steer_command_type = static_cast<u8>(sanitizeDirectSteerCommandType(debug_control_.single_wheel.steer.command_type_raw));
            command.drive_command_type = static_cast<u8>(sanitizeDirectDriveCommandType(debug_control_.single_wheel.drive.command_type_raw));
            command.steer_planner_mode = static_cast<u8>(sanitizeSingleWheelPlannerMode(debug_control_.single_wheel.steer.planner_mode_raw));
            command.drive_planner_mode = static_cast<u8>(sanitizeSingleWheelPlannerMode(debug_control_.single_wheel.drive.planner_mode_raw));
            command.steer_invert_input = debug_control_.single_wheel.steer.invert_input;
            command.drive_invert_input = debug_control_.single_wheel.drive.invert_input;
            command.steer_command_limit = (debug_control_.single_wheel.steer.command_limit > 0.0f)
                                              ? debug_control_.single_wheel.steer.command_limit
                                              : getDirectSteerDefaultLimit(static_cast<DirectSteerCommandType>(command.steer_command_type));
            command.drive_command_limit = (debug_control_.single_wheel.drive.command_limit > 0.0f)
                                              ? debug_control_.single_wheel.drive.command_limit
                                              : getDirectDriveDefaultLimit(static_cast<DirectDriveCommandType>(command.drive_command_type));
            command.steer_step_threshold = (debug_control_.single_wheel.steer.step_threshold > 0.01f) ? debug_control_.single_wheel.steer.step_threshold : 0.3f;
            command.drive_step_threshold = (debug_control_.single_wheel.drive.step_threshold > 0.01f) ? debug_control_.single_wheel.drive.step_threshold : 0.3f;
            command.steer_step_value = fabsf((debug_control_.single_wheel.steer.step_value != 0.0f)
                                                 ? debug_control_.single_wheel.steer.step_value
                                                 : getDirectSteerDefaultStepValue(static_cast<DirectSteerCommandType>(command.steer_command_type)));
            command.drive_step_value = fabsf((debug_control_.single_wheel.drive.step_value != 0.0f)
                                                 ? debug_control_.single_wheel.drive.step_value
                                                 : getDirectDriveDefaultStepValue(static_cast<DirectDriveCommandType>(command.drive_command_type)));
            command.steer_command_value = debug_control_.single_wheel.steer.command_value;
            command.drive_command_value = debug_control_.single_wheel.drive.command_value;

            const f32 deadzone = fabsf(debug_control_.single_wheel.input_deadzone);
            const DirectAxisInputMode steer_input_mode = static_cast<DirectAxisInputMode>(command.steer_input_mode);
            const DirectAxisInputMode drive_input_mode = static_cast<DirectAxisInputMode>(command.drive_input_mode);

            if (steer_input_mode != DirectAxisInputMode::kCached)
            {
                command.steer_axis_value = readSingleWheelInputAxisValue(command.steer_input_axis);
                if (command.steer_invert_input)
                {
                    command.steer_axis_value = -command.steer_axis_value;
                }
                command.steer_axis_value = clampValue(command.steer_axis_value, -1.0f, 1.0f);
                if (fabsf(command.steer_axis_value) < deadzone)
                {
                    command.steer_axis_value = 0.0f;
                    command.steer_deadzone_applied = true;
                }

                if (steer_input_mode == DirectAxisInputMode::kRcContinuous)
                {
                    command.steer_command_value = command.steer_axis_value * command.steer_command_limit;
                }
                else if (steer_input_mode == DirectAxisInputMode::kRcStep)
                {
                    if (command.steer_axis_value > command.steer_step_threshold)
                    {
                        command.steer_step_sign = 1.0f;
                    }
                    else if (command.steer_axis_value < -command.steer_step_threshold)
                    {
                        command.steer_step_sign = -1.0f;
                    }
                    command.steer_command_value = command.steer_step_sign * command.steer_step_value;
                }
            }

            if (drive_input_mode != DirectAxisInputMode::kCached)
            {
                command.drive_axis_value = readSingleWheelInputAxisValue(command.drive_input_axis);
                if (command.drive_invert_input)
                {
                    command.drive_axis_value = -command.drive_axis_value;
                }
                command.drive_axis_value = clampValue(command.drive_axis_value, -1.0f, 1.0f);
                if (fabsf(command.drive_axis_value) < deadzone)
                {
                    command.drive_axis_value = 0.0f;
                    command.drive_deadzone_applied = true;
                }

                if (drive_input_mode == DirectAxisInputMode::kRcContinuous)
                {
                    command.drive_command_value = command.drive_axis_value * command.drive_command_limit;
                }
                else if (drive_input_mode == DirectAxisInputMode::kRcStep)
                {
                    if (command.drive_axis_value > command.drive_step_threshold)
                    {
                        command.drive_step_sign = 1.0f;
                    }
                    else if (command.drive_axis_value < -command.drive_step_threshold)
                    {
                        command.drive_step_sign = -1.0f;
                    }
                    command.drive_command_value = command.drive_step_sign * command.drive_step_value;
                }
            }

            command.steer_command_value = shapeSingleWheelSteerCommand(wheel_idx, debug_control_.single_wheel.steer, command.steer_command_value);
            if (sanitizeDirectDriveCommandType(command.drive_command_type) == DirectDriveCommandType::kRpm)
            {
                command.drive_command_value = resolveSingleWheelDriveStepTargetRpm(wheel_idx, command.drive_command_value);
                const f32 shaped_omega_rad_s =
                    shapeSingleWheelDriveOmegaRadS(wheel_idx,
                                                   debug_control_.single_wheel.drive,
                                                   rpmToRadsF32(command.drive_command_value));
                command.drive_command_value = radsToRpmF32(shaped_omega_rad_s);
            }
            else
            {
                resetSingleWheelAxisPlannerRuntime(single_wheel_drive_planner_state_);
                drive_step_generator_runtime_[wheel_idx] = {};
            }

            debug_control_.single_wheel.steer.command_value = command.steer_command_value;
            debug_control_.single_wheel.drive.command_value = command.drive_command_value;
            command.applied_steer_cmd = clampValue(command.steer_command_value, -command.steer_command_limit, command.steer_command_limit);
            command.applied_drive_cmd = clampValue(command.drive_command_value, -command.drive_command_limit, command.drive_command_limit);
            return command;
        }

        void Chassis::clearDirectDriveCommandByType(WheelConfig &wheel, u8 wheel_idx, u8 drive_command_type)
        {
            wheel.target_drive_omega_rad_s = 0.0f;
            planned_data_.drive_omega_rad_s[wheel_idx] = 0.0f;
            if (wheel.drive_motor_h == nullptr)
            {
                return;
            }
            if (drive_command_type == static_cast<u8>(DirectDriveCommandType::kCurrent))
            {
                wheel.drive_motor_h->setTargetCurrent(0.0f);
            }
            else if (drive_command_type == static_cast<u8>(DirectDriveCommandType::kBrake))
            {
                wheel.drive_motor_h->setBrake(0.0f);
            }
            else
            {
                setDriveMotorTargetOmegaRadS(wheel, 0.0f);
            }
        }

        void Chassis::applyResolvedSteerCommand(WheelConfig &wheel, u8 wheel_idx, const DirectActuatorCommandSnapshot &command, bool enable)
        {
            if (!enable)
            {
                setSteerMotorTargetCurrent(wheel, 0.0f);
                return;
            }

            if (command.steer_command_type == static_cast<u8>(DirectSteerCommandType::kCurrent))
            {
                const f32 target_current_mA = command.applied_steer_cmd;
                wheel.target_steer_motor_total_angle_rad = wheel.corrected_steer_motor_total_angle_rad;
                planned_data_.steer_angle_oa_rad[wheel_idx] = mapWheelCorrectedLocalToOaTotal(wheel, wheel.corrected_steer_motor_total_angle_rad);
                setSteerMotorTargetCurrent(wheel, target_current_mA);
            }
            else if (command.steer_command_type == static_cast<u8>(DirectSteerCommandType::kRpm))
            {
                const f32 target_steer_rpm = command.applied_steer_cmd;
                wheel.target_steer_motor_total_angle_rad = wheel.corrected_steer_motor_total_angle_rad;
                planned_data_.steer_angle_oa_rad[wheel_idx] = mapWheelCorrectedLocalToOaTotal(wheel, wheel.corrected_steer_motor_total_angle_rad);
                setSteerMotorTargetRPM(wheel, target_steer_rpm);
            }
            else if (command.steer_command_type == static_cast<u8>(DirectSteerCommandType::kSingleTurnDeg))
            {
                const f32 target_single_turn_deg = command.applied_steer_cmd;
                const f32 target_local_total_rad = mapSingleTurnToNearestTotalAngle(wheel, target_single_turn_deg);
                wheel.target_steer_motor_total_angle_rad = target_local_total_rad;
                planned_data_.steer_angle_oa_rad[wheel_idx] = mapWheelCorrectedLocalToOaTotal(wheel, target_local_total_rad);
                setSteerMotorTargetTotalAngleRad(wheel, target_local_total_rad);
            }
            else
            {
                const f32 target_oa_total_rad = degToRadF32(command.applied_steer_cmd);
                const f32 target_local_total_rad = mapWheelOaTotalToCorrectedLocal(wheel, target_oa_total_rad);
                wheel.target_steer_motor_total_angle_rad = target_local_total_rad;
                planned_data_.steer_angle_oa_rad[wheel_idx] = target_oa_total_rad;
                setSteerMotorTargetTotalAngleRad(wheel, target_local_total_rad);
            }
        }

        void Chassis::applyResolvedDriveCommand(WheelConfig &wheel, u8 wheel_idx, const DirectActuatorCommandSnapshot &command, bool enable)
        {
            if (!enable)
            {
                clearDirectDriveCommandByType(wheel, wheel_idx, command.drive_command_type);
                return;
            }

            if (command.drive_command_type == static_cast<u8>(DirectDriveCommandType::kCurrent))
            {
                const f32 target_current_mA = command.applied_drive_cmd;
                wheel.target_drive_omega_rad_s = 0.0f;
                planned_data_.drive_omega_rad_s[wheel_idx] = 0.0f;
                if (wheel.drive_motor_h != nullptr)
                {
                    wheel.drive_motor_h->setTargetCurrent(mapWheelCurrentToDriveMotorCurrent(target_current_mA, makeSteerCalibration(wheel)));
                }
            }
            else if (command.drive_command_type == static_cast<u8>(DirectDriveCommandType::kBrake))
            {
                const f32 target_brake_mA = command.applied_drive_cmd;
                wheel.target_drive_omega_rad_s = 0.0f;
                planned_data_.drive_omega_rad_s[wheel_idx] = 0.0f;
                if (wheel.drive_motor_h != nullptr)
                {
                    wheel.drive_motor_h->setBrake(mapWheelCurrentToDriveMotorCurrent(target_brake_mA, makeSteerCalibration(wheel)));
                }
            }
            else
            {
                const f32 target_rpm = command.applied_drive_cmd;
                const f32 target_omega_rad_s = rpmToRadsF32(target_rpm);
                wheel.target_drive_omega_rad_s = target_omega_rad_s;
                planned_data_.drive_omega_rad_s[wheel_idx] = target_omega_rad_s;
                setDriveMotorTargetOmegaRadS(wheel, target_omega_rad_s);
            }
        }
#endif

        void Chassis::applyDriveVirtualLoadAndCommand(WheelConfig &wheel,
                                                      u8 wheel_idx,
                                                      f32 delivered_drive_target_rad_s,
                                                      bool single_wheel_isolation_active,
                                                      u8 single_wheel_idx,
                                                      bool chassis_motion_blocked,
                                                      bool allow_drive_position_loop,
                                                      bool drive_zero_stop_active,
                                                      bool entering_drive_zero_stop,
                                                      bool leaving_drive_zero_stop)
        {
            VESC_Motor *drive_vesc = dynamic_cast<VESC_Motor *>(wheel.drive_motor_h);
            f32 drive_bias_current_mA = 0.0f;
#if JIA_CHASSIS_ENABLE_DEBUG_OUTPUT
            f32 j_term_mA = 0.0f;
            f32 b_term_mA = 0.0f;
            f32 tc_term_mA = 0.0f;
            f32 alpha_est_rad_s2 = 0.0f;
#endif
#if JIA_CHASSIS_ENABLE_DRIVE_VIRTUAL_LOAD
            f32 alpha_dt_s = period_;
#endif
            const bool can_use_vesc_drive_assist =
                allow_drive_position_loop &&
                (drive_vesc != nullptr) &&
                !single_wheel_isolation_active;
            const bool can_reset_local_speed_pid =
                can_use_vesc_drive_assist &&
                (drive_vesc->getRpmControlMode() == VESC_RPM_CONTROL_PID_CURRENT);

#if JIA_CHASSIS_ENABLE_DRIVE_VIRTUAL_LOAD
            const bool can_inject_virtual_load =
                allow_drive_position_loop &&
                (drive_vesc != nullptr) &&
                (wheel_idx == single_wheel_idx) &&
                single_wheel_isolation_active &&
                (debug_control_.single_wheel.drive.command_type_raw == static_cast<u8>(DirectDriveCommandType::kRpm)) &&
                (drive_vesc->getRpmControlMode() == VESC_RPM_CONTROL_PID_CURRENT) &&
                debug_drive_virtual_load_[wheel_idx].enable &&
                !current_mode_flag_.is_wheel_torque_free &&
                !input_target_data_.zero_current_all &&
                !chassis_motion_blocked;
#else
            const bool can_inject_virtual_load = false;
#endif

#if JIA_CHASSIS_ENABLE_DRIVE_VIRTUAL_LOAD
            if (can_reset_local_speed_pid && leaving_drive_zero_stop)
            {
                // 从 zero-stop 恢复正常 RPM 闭环前，再清一次速度环状态，避免停前残留被带进下一次起步。
                drive_vesc->reset_speed_pid_state();
            }
#endif

#if JIA_CHASSIS_ENABLE_DRIVE_VIRTUAL_LOAD
            if (!drive_zero_stop_active && can_inject_virtual_load)
            {
                const DebugDriveVirtualLoadConfig &load_cfg = debug_drive_virtual_load_[wheel_idx];
                const f32 omega_rad_s = wheel.corrected_drive_omega_rad_s;
                const u32 feedback_sample_ms = drive_feedback_sample_ms_[wheel_idx];
                const u32 last_feedback_sample_ms = last_drive_feedback_sample_ms_[wheel_idx];
                if ((feedback_sample_ms > last_feedback_sample_ms) && (last_feedback_sample_ms != 0U))
                {
                    const u32 delta_ms = feedback_sample_ms - last_feedback_sample_ms;
                    const f32 measured_dt_s = static_cast<f32>(delta_ms) * 1.0e-3f;
                    if ((delta_ms >= 1U) && (delta_ms <= 100U))
                    {
                        alpha_dt_s = measured_dt_s;
                    }
                }

                alpha_est_rad_s2 = (omega_rad_s - last_drive_feedback_omega_rad_s_[wheel_idx]) / alpha_dt_s;
                j_term_mA = -load_cfg.delta_j_current_per_rad_s2 * alpha_est_rad_s2;
                b_term_mA = -load_cfg.delta_b_current_per_rad_s * omega_rad_s;

                f32 sign_ref = 0.0f;
                if (fabsf(omega_rad_s) >= fabsf(load_cfg.coulomb_sign_vel_eps_rad_s))
                {
                    sign_ref = (omega_rad_s > 0.0f) ? 1.0f : ((omega_rad_s < 0.0f) ? -1.0f : 0.0f);
                }
                else if (fabsf(delivered_drive_target_rad_s) >= 1.0e-6f)
                {
                    sign_ref = (delivered_drive_target_rad_s > 0.0f) ? 1.0f : -1.0f;
                }
                tc_term_mA = -load_cfg.coulomb_current_mA * sign_ref;
                drive_bias_current_mA = clampValue(j_term_mA + b_term_mA + tc_term_mA,
                                                   -fabsf(load_cfg.bias_current_limit_mA),
                                                   fabsf(load_cfg.bias_current_limit_mA));
            }
#endif

            if (drive_vesc != nullptr)
            {
                drive_vesc->setSpeedPidCurrentBias(drive_bias_current_mA);
            }

            const bool can_apply_zero_stop_assist =
                drive_zero_stop_active &&
                can_use_vesc_drive_assist &&
                runtime_strategy_cfg_.enable_drive_zero_stop_assist;

            if (can_apply_zero_stop_assist)
            {
                // zero-stop 的模式层只看“目标速度”：
                // applyModuleCommands() 已经用目标 near-zero enter/exit 决定 drive_zero_stop_active_。
                // 这里处理的是每个 drive 末端的收尾层：active 期间先 brake，把轮子压到静止；
                // 如果允许 settle 零电流收尾，再用实际 residual 复用 NearZero enter/exit 做滞回。
                // 这样目标门不会被反馈噪声踢出，但轮子确实停稳后也不必一直吃 brake 电流。
                const f32 residual_speed_m_s = fabsf(wheel.corrected_drive_omega_rad_s) * runtime_strategy_cfg_.wheel_radius_m_;
                const bool was_brake_active = drive_zero_stop_brake_active_[wheel_idx];
                const bool yaw_lock_release_hold_active =
                    yaw_lock_zero_stop_decel_context_active_ ||
                    (yaw_lock_zero_stop_release_hold_elapsed_ms_ > 0U);
                if (yaw_lock_zero_stop_decel_context_active_ || yaw_lock_release_hold_active)
                {
                    drive_zero_stop_brake_active_[wheel_idx] = true;
                }
                else if (!runtime_strategy_cfg_.enable_drive_zero_stop_settle_zero_current)
                {
                    drive_zero_stop_brake_active_[wheel_idx] = true;
                }
                else if (entering_drive_zero_stop)
                {
                    // 刚进入 zero-stop 的第一拍先 brake，一方面清掉速度环旧状态，一方面避免反馈刚好贴近 0 时漏掉主动刹停。
                    drive_zero_stop_brake_active_[wheel_idx] = true;
                }
                else if (drive_zero_stop_brake_active_[wheel_idx])
                {
                    // 仍在 brake 时，必须 residual 进入 NearZero enter 才认为“已经刹稳”，切到零电流。
                    drive_zero_stop_brake_active_[wheel_idx] = residual_speed_m_s > getNearZeroEnterSpeedMps();
                }
                else
                {
                    // 已经零电流收尾后，只有 residual 离开 NearZero exit 才重新 brake，避免 enter 附近来回抖动。
                    drive_zero_stop_brake_active_[wheel_idx] = residual_speed_m_s > getNearZeroExitSpeedMps();
                }

                const bool need_reset_speed_pid_state =
                    can_reset_local_speed_pid &&
                    entering_drive_zero_stop;

                if (need_reset_speed_pid_state)
                {
                    // 进入 zero-stop brake 前清一次本地速度环状态，避免旧速度环尾巴叠到刹车收尾里。
                    // 退出时的清理在本函数前部 leaving_drive_zero_stop 分支完成。
                    drive_vesc->reset_speed_pid_state();
                }

                if (drive_zero_stop_brake_active_[wheel_idx])
                {
                    const f32 target_brake_current_mA = fabsf(runtime_strategy_cfg_.drive_zero_stop_brake_current_mA);
                    const u32 ramp_time_ms = runtime_strategy_cfg_.drive_zero_stop_brake_ramp_time_ms;

                    if (!was_brake_active)
                    {
                        drive_zero_stop_brake_ramp_elapsed_ms_[wheel_idx] = 0U;
                    }

                    if (ramp_time_ms > 0U)
                    {
                        const u32 prev_elapsed_ms = drive_zero_stop_brake_ramp_elapsed_ms_[wheel_idx];
                        const u32 next_elapsed_ms =
                            (prev_elapsed_ms > (0xFFFFFFFFU - period_ms_)) ? 0xFFFFFFFFU : (prev_elapsed_ms + period_ms_);
                        drive_zero_stop_brake_ramp_elapsed_ms_[wheel_idx] = next_elapsed_ms;

                        const f32 ramp_ratio =
                            clampValue(static_cast<f32>(next_elapsed_ms) / static_cast<f32>(ramp_time_ms), 0.0f, 1.0f);
                        drive_vesc->setBrake(target_brake_current_mA * ramp_ratio);
                    }
                    else
                    {
                        drive_zero_stop_brake_ramp_elapsed_ms_[wheel_idx] = 0U;
                        drive_vesc->setBrake(target_brake_current_mA);
                    }
                }
                else
                {
                    drive_zero_stop_brake_ramp_elapsed_ms_[wheel_idx] = 0U;
                    drive_vesc->setTargetCurrent(0.0f);
                }
            }
            else
            {
                drive_zero_stop_brake_active_[wheel_idx] = false;
                drive_zero_stop_brake_ramp_elapsed_ms_[wheel_idx] = 0U;
                if (allow_drive_position_loop)
                {
                    setDriveMotorTargetOmegaRadS(wheel, delivered_drive_target_rad_s);
                }
            }

#if JIA_CHASSIS_ENABLE_DEBUG_OUTPUT
            if (wheel_idx == static_cast<u8>(debug_drive_load_trace_.observe_wheel_idx))
            {
                const f32 trace_target_rad_s = can_apply_zero_stop_assist ? 0.0f : delivered_drive_target_rad_s;
                debug_drive_load_trace_.target_rpm = radsToRpmF32(trace_target_rad_s);
                debug_drive_load_trace_.feedback_rpm = radsToRpmF32(wheel.corrected_drive_omega_rad_s);
                debug_drive_load_trace_.omega_rad_s = wheel.corrected_drive_omega_rad_s;
                debug_drive_load_trace_.alpha_est_rad_s2 = alpha_est_rad_s2;
                debug_drive_load_trace_.j_term_mA = j_term_mA;
                debug_drive_load_trace_.b_term_mA = b_term_mA;
                debug_drive_load_trace_.tc_term_mA = tc_term_mA;
                debug_drive_load_trace_.load_bias_current_mA = drive_bias_current_mA;
                debug_drive_load_trace_.virtual_load_enable = (!drive_zero_stop_active && can_inject_virtual_load) ? 1.0f : 0.0f;
                if (
#if JIA_CHASSIS_ENABLE_DRIVE_STEP_GENERATOR
                    debug_drive_step_generator_[wheel_idx].enable
#else
                    false
#endif
                )
                {
                    debug_drive_load_trace_.stepgen_enable = 1.0f;
                }

                if (drive_vesc != nullptr)
                {
                    debug_drive_load_trace_.pid_current_mA = drive_vesc->getSpeedPidRawOutputCurrent();
                    debug_drive_load_trace_.total_current_cmd_mA = drive_vesc->getSpeedPidTotalOutputCurrent();
                }
            }
#endif

            last_drive_feedback_omega_rad_s_[wheel_idx] = wheel.corrected_drive_omega_rad_s;
            last_drive_feedback_sample_ms_[wheel_idx] = drive_feedback_sample_ms_[wheel_idx];
        }

#if JIA_CHASSIS_ENABLE_DEBUG_OVERRIDE
        void Chassis::computeSingleWheelIsolatedCommandsMode30(u8 wheel_idx, bool all_homed)
        {
            high_speed_trans_gate_active_ = false;
            high_speed_drive_suppression_active_ = false;
            high_speed_drive_suppression_scale_ = 1.0f;
            high_speed_dir_err_deg_ = 0.0f;
            high_speed_eta_max_s_ = 0.0f;
            reverse_intent_active_ = false;
            reverse_intent_dir_err_deg_ = 0.0f;
            xpark_gate_active_ = false;
            xpark_stationary_hold_ms_ = 0U;
            launch_hold_active_ = false;
            drive_zero_stop_active_ = false;
            yaw_lock_zero_stop_decel_context_active_ = false;
            yaw_lock_zero_stop_release_hold_elapsed_ms_ = 0U;
            for (u8 i = 0; i < 4; ++i)
            {
                drive_zero_stop_brake_active_[i] = false;
                drive_zero_stop_brake_ramp_elapsed_ms_[i] = 0U;
            }
            low_speed_residual_bypass_active_ = false;
            low_speed_drive_suppression_bypassed_by_residual_speed_ = false;
            max_residual_speed_m_s_ = 0.0f;

            planned_data_.vel_x = 0.0f;
            planned_data_.vel_y = 0.0f;
            planned_data_.omega_z = 0.0f;
            planned_data_.acc_x = 0.0f;
            planned_data_.acc_y = 0.0f;
            planned_data_.alpha_z = 0.0f;
            planned_data_.rot_z = input_hwt_rot_z_;

            planner_output_cache_ = {};
            actuator_command_frame_ = {};
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                planned_data_.steer_angle_oa_rad[i] = mapWheelCorrectedLocalToOaTotal(wheel, wheel.corrected_steer_motor_total_angle_rad);
                planned_data_.drive_omega_rad_s[i] = 0.0f;
                actuator_command_frame_.steer_corrected_local_total_rad[i] = wheel.corrected_steer_motor_total_angle_rad;
                actuator_command_frame_.steer_oa_total_rad[i] = planned_data_.steer_angle_oa_rad[i];
                actuator_command_frame_.steer_cmd_corrected_local_total_rad[i] = actuator_command_frame_.steer_corrected_local_total_rad[i];
                actuator_command_frame_.steer_cmd_oa_total_rad[i] = actuator_command_frame_.steer_oa_total_rad[i];
                actuator_command_frame_.steer_rate_rad_s[i] = 0.0f;
                actuator_command_frame_.drive_omega_rad_s[i] = 0.0f;
                actuator_command_frame_.flipped_drive_direction[i] = false;
                low_speed_drive_suppression_scale_[i] = (i == wheel_idx) ? 1.0f : 0.0f;
            }

            WheelConfig &target_wheel = wheel_config_[wheel_idx];
            const DirectActuatorCommandSnapshot command = resolveSingleWheelCommand(wheel_idx);
            applyResolvedSteerCommand(target_wheel, wheel_idx, command, debug_control_.single_wheel.steer.enable && !debug_control_.single_wheel.estop);
            applyResolvedDriveCommand(target_wheel, wheel_idx, command, debug_control_.single_wheel.drive.enable && !debug_control_.single_wheel.estop);
            const f32 preserved_step_phase = debug_drive_load_trace_.step_phase;
            const f32 preserved_stepgen_enable = debug_drive_load_trace_.stepgen_enable;
            debug_drive_load_trace_ = {};
            debug_drive_load_trace_.observe_wheel_idx = static_cast<f32>((debug_control_.common.observe_wheel_index < 4U) ? debug_control_.common.observe_wheel_index : 0U);
            const bool observe_this_wheel = (wheel_idx == static_cast<u8>(debug_drive_load_trace_.observe_wheel_idx));
            if (observe_this_wheel)
            {
                debug_drive_load_trace_.target_rpm = command.applied_drive_cmd;
                debug_drive_load_trace_.feedback_rpm = radsToRpmF32(target_wheel.corrected_drive_omega_rad_s);
                debug_drive_load_trace_.step_phase = preserved_step_phase;
                debug_drive_load_trace_.stepgen_enable = (preserved_stepgen_enable > 0.5f || debug_drive_step_generator_[wheel_idx].enable) ? 1.0f : 0.0f;
            }

            if (command.drive_command_type == static_cast<u8>(DirectDriveCommandType::kRpm))
            {
                bool steer_fault_any_active = false;
                for (u8 i = 0; i < 4; ++i)
                {
                    if (wheel_config_[i].steer_fault_state != SteerFaultState::kNone)
                    {
                        steer_fault_any_active = true;
                        break;
                    }
                }
                steer_fault_any_active_ = steer_fault_any_active;
                const bool chassis_motion_blocked = !all_homed || steer_fault_any_active;
                const bool drive_enabled = debug_control_.single_wheel.drive.enable && !debug_control_.single_wheel.estop;
                if (drive_enabled && !current_mode_flag_.is_wheel_torque_free && !input_target_data_.zero_current_all)
                {
                    // mode30 目标轮 RPM 直控本身已经绕过全车 homing/fault gate；
                    // 虚拟负载只是叠加在这条目标轮速度环上的偏置电流，不应再被其他轮状态全车级短路。
                    applyDriveVirtualLoadAndCommand(target_wheel,
                                                    wheel_idx,
                                                    target_wheel.target_drive_omega_rad_s,
                                                    true,
                                                    wheel_idx,
                                                    false,
                                                    true,
                                                    false,
                                                    false,
                                                    false);
                }
                else if (VESC_Motor *drive_vesc = dynamic_cast<VESC_Motor *>(target_wheel.drive_motor_h))
                {
                    drive_vesc->setSpeedPidCurrentBias(0.0f);
                }
            }
            else
            {
                if (VESC_Motor *drive_vesc = dynamic_cast<VESC_Motor *>(target_wheel.drive_motor_h))
                {
                    drive_vesc->setSpeedPidCurrentBias(0.0f);
                }
            }
            target_wheel.steer_target_velocity_rad_s = 0.0f;
            target_wheel.flipped_drive_direction = false;

            actuator_command_frame_.steer_corrected_local_total_rad[wheel_idx] = target_wheel.target_steer_motor_total_angle_rad;
            actuator_command_frame_.steer_oa_total_rad[wheel_idx] = planned_data_.steer_angle_oa_rad[wheel_idx];
            actuator_command_frame_.steer_cmd_corrected_local_total_rad[wheel_idx] = actuator_command_frame_.steer_corrected_local_total_rad[wheel_idx];
            actuator_command_frame_.steer_cmd_oa_total_rad[wheel_idx] = actuator_command_frame_.steer_oa_total_rad[wheel_idx];
            actuator_command_frame_.drive_omega_rad_s[wheel_idx] = target_wheel.target_drive_omega_rad_s;

#if JIA_CHASSIS_ENABLE_DEBUG_OUTPUT
            if (debug_output_.output_enable &&
                sanitizeDebugOutputFamily(debug_output_.output_family_raw) == DebugOutputFamily::kText &&
                debug_output_.text.log_level >= 1U &&
                (time_ms_ - debug_output_runtime_.text.direct_trace_last_ms) >= 100U)
            {
                debug_output_runtime_.text.direct_trace_last_ms = time_ms_;
                const Motor_Base *steer_motor = target_wheel.steer_motor_h;
                const VESC_Motor *drive_motor = target_wheel.drive_motor_h;
                if (steer_motor != nullptr)
                {
                    debug_uart_.printf_DMA((char *)"SW30,t=%lu,w=%u,stIn=%u,drIn=%u,stAxisId=%u,drAxisId=%u,stType=%u,drType=%u,stPlan=%u,drPlan=%u,stInv=%u,drInv=%u,stDz=%u,drDz=%u,stRaw=%.3f,drRaw=%.3f,stLim=%.3f,drLim=%.3f,stTh=%.3f,drTh=%.3f,stStep=%.3f,drStep=%.3f,stApplied=%.3f,drApplied=%.3f,stAxis=%.3f,drAxis=%.3f,stStepSign=%.1f,drStepSign=%.1f,stTarI=%.1f,stCurI=%.1f,stTarRPM=%.2f,stCurRPM=%.2f,drTarI=%.1f,drCurI=%.1f,drTarRPM=%.2f,drCurRPM=%.2f,enS=%u,enD=%u,estop=%u\r\n",
                                           (u32)time_ms_,
                                           (u32)wheel_idx,
                                           (u32)command.steer_input_mode,
                                           (u32)command.drive_input_mode,
                                           (u32)command.steer_input_axis,
                                           (u32)command.drive_input_axis,
                                           (u32)command.steer_command_type,
                                           (u32)command.drive_command_type,
                                           (u32)command.steer_planner_mode,
                                           (u32)command.drive_planner_mode,
                                           command.steer_invert_input ? 1U : 0U,
                                           command.drive_invert_input ? 1U : 0U,
                                           command.steer_deadzone_applied ? 1U : 0U,
                                           command.drive_deadzone_applied ? 1U : 0U,
                                           command.steer_command_value,
                                           command.drive_command_value,
                                           command.steer_command_limit,
                                           command.drive_command_limit,
                                           command.steer_step_threshold,
                                           command.drive_step_threshold,
                                           command.steer_step_value,
                                           command.drive_step_value,
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
                                           (debug_control_.single_wheel.steer.enable && !debug_control_.single_wheel.estop) ? 1U : 0U,
                                           (debug_control_.single_wheel.drive.enable && !debug_control_.single_wheel.estop) ? 1U : 0U,
                                           debug_control_.single_wheel.estop ? 1U : 0U);
                }
            }
#endif
        }

        bool Chassis::isSingleWheelIsolatedMode(DebugMode mode) const
        {
            return mode == DebugMode::kSingleWheelIsolated;
        }

        void Chassis::applySingleWheelIsolationFilter(DebugMode mode, u8 wheel_idx, bool all_homed)
        {
            if (!isSingleWheelIsolatedMode(mode))
            {
                debug_mirror_.single_wheel_isolation_active = false;
                for (u8 i = 0; i < 4; ++i)
                {
                    debug_mirror_.single_wheel_non_target_zeroed[i] = false;
                }
                return;
            }

            debug_mirror_.single_wheel_target_index = wheel_idx;
            debug_mirror_.single_wheel_isolation_active = true;
            for (u8 i = 0; i < 4; ++i)
            {
                debug_mirror_.single_wheel_non_target_zeroed[i] = (i != wheel_idx);
                if (i == wheel_idx)
                {
                    continue;
                }

                WheelConfig &wheel = wheel_config_[i];
                wheel.target_steer_motor_total_angle_rad = wheel.corrected_steer_motor_total_angle_rad;
                wheel.steer_target_velocity_rad_s = 0.0f;
                wheel.flipped_drive_direction = false;
                wheel.target_drive_omega_rad_s = 0.0f;
                planned_data_.steer_angle_oa_rad[i] = mapWheelCorrectedLocalToOaTotal(wheel, wheel.corrected_steer_motor_total_angle_rad);
                planned_data_.drive_omega_rad_s[i] = 0.0f;
                last_steer_rate_cmd_rad_s_[i] = 0.0f;
                last_drive_omega_cmd_rad_s_[i] = 0.0f;
                setSteerMotorTargetCurrent(wheel, 0.0f);
                if (wheel.drive_motor_h != nullptr)
                {
                    wheel.drive_motor_h->setTargetCurrent(0.0f);
                }
            }

        }

        void Chassis::finalizeDebugModuleOverride(bool all_homed, DebugModuleOverrideRoute route)
        {
            if (route != DebugModuleOverrideRoute::kSingleWheelIsolated)
            {
                planned_data_.vel_x = 0.0f;
                planned_data_.vel_y = 0.0f;
                planned_data_.omega_z = 0.0f;
                planned_data_.acc_x = 0.0f;
                planned_data_.acc_y = 0.0f;
                planned_data_.alpha_z = 0.0f;
            }
            planned_data_.rot_z = input_hwt_rot_z_;
            if (route != DebugModuleOverrideRoute::kSingleWheelIsolated)
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
            const DebugControlRoute route = classifyDebugControlRoute(debug_control_.common.enable, debug_control_.common.mode_raw);
            if (route == DebugControlRoute::kDisabled)
            {
                debug_control_.common.mode_resolved_raw = static_cast<u8>(DebugMode::kTorqueFree);
                return;
            }

            const DebugMode mode = resolveDebugMode(debug_control_.common.mode_raw);
            debug_control_.common.mode_resolved_raw = static_cast<u8>(mode);

            if (route == DebugControlRoute::kTargetInjection)
            {
                applyDebugTargetOverride(mode);
                return;
            }

            if (isSingleWheelIsolatedMode(mode))
            {
                const u8 wheel_idx = (debug_control_.common.control_wheel_index < 4U)
                                         ? debug_control_.common.control_wheel_index
                                         : 0U;
                const u8 observe_idx = (debug_control_.common.observe_wheel_index < 4U)
                                           ? debug_control_.common.observe_wheel_index
                                           : wheel_idx;
                debug_control_.common.control_wheel_index = wheel_idx;
                debug_control_.common.observe_wheel_index = observe_idx;
                return;
            }

            setTargetBodySpeedMode(0.0f, 0.0f, 0.0f);
            clearPlannedMotionForModuleOverride();
        }
#endif

        void Chassis::transSpeedBodyToWorld(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y) const
        {
            f32 cos_theta = cosRadF32(input_hwt_rot_z_);
            f32 sin_theta = sinRadF32(input_hwt_rot_z_);
            out_vel_x = vel_x * cos_theta - vel_y * sin_theta;
            out_vel_y = vel_x * sin_theta + vel_y * cos_theta;
        }

        void Chassis::transSpeedWorldToBody(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y) const
        {
            f32 cos_theta = cosRadF32(input_hwt_rot_z_);
            f32 sin_theta = sinRadF32(input_hwt_rot_z_);
            out_vel_x = vel_x * cos_theta + vel_y * sin_theta;
            out_vel_y = -vel_x * sin_theta + vel_y * cos_theta;
        }

        // “锁当前航向”模式的核心语义是：
        // 抓取最近一次真实机体朝向，再由 PID 生成维持该姿态所需的角速度闭环；
        // 因此它不是“始终锁某个固定角”，而是“手动旋转”和“松手后自动锁住当前角”之间的平滑切换器。
        void Chassis::resetYawPidTargetRuntime()
        {
            lock_yaw_pid_target_filter_valid_ = false;
            lock_yaw_pid_target_filtered_rad_ = 0.0f;
            lock_yaw_pid_deadband_active_ = false;
        }

        f32 Chassis::filterYawPidTarget(f32 target_yaw_rad)
        {
            const f32 alpha = clampValue(lock_yaw_pid_target_lpf_alpha_, 0.0f, 1.0f);
            if (!lock_yaw_pid_target_filter_valid_)
            {
                lock_yaw_pid_target_filter_valid_ = true;
                lock_yaw_pid_target_filtered_rad_ = target_yaw_rad;
                return lock_yaw_pid_target_filtered_rad_;
            }

            lock_yaw_pid_target_filtered_rad_ +=
                alpha * shortestAngularDistanceF32(lock_yaw_pid_target_filtered_rad_, target_yaw_rad);
            return lock_yaw_pid_target_filtered_rad_;
        }

        bool Chassis::computeYawPidOmega(f32 target_yaw_rad, f32 feedback_yaw_rad, f32 &out_omega_z)
        {
            const f32 error_deg = radToDegF32(shortestAngularDistanceF32(feedback_yaw_rad, target_yaw_rad));
            const f32 enter_deg = fabsf(lock_yaw_pid_deadband_enter_deg_);
            const f32 exit_deg = (fabsf(lock_yaw_pid_deadband_exit_deg_) < enter_deg) ? enter_deg : fabsf(lock_yaw_pid_deadband_exit_deg_);
            const f32 abs_error_deg = fabsf(error_deg);

            if (lock_yaw_pid_deadband_active_)
            {
                if (abs_error_deg >= exit_deg)
                {
                    lock_yaw_pid_deadband_active_ = false;
                }
            }
            else if (abs_error_deg <= enter_deg)
            {
                lock_yaw_pid_deadband_active_ = true;
            }

            if (lock_yaw_pid_deadband_active_)
            {
                out_omega_z = 0.0f;
                return false;
            }

            out_omega_z = rot_z_pid_.pid_calc(radToDegF32(target_yaw_rad), radToDegF32(feedback_yaw_rad));
            return true;
        }

        void Chassis::isLockNowRotZ(bool is_lock, f32 rot_z, f32 omega_z, f32 &out_rot_z, f32 &out_omega_z)
        {
            if (!is_lock)
            {
                out_rot_z = rot_z;
                out_omega_z = omega_z;
                yaw_pid_trace_ = YawPidTraceState{};
                resetYawPidTargetRuntime();
                return;
            }

            yaw_pid_trace_.mode_tag = 0.0f;
            yaw_pid_trace_.target_yaw_rad = lock_now_rot_z_target_;
            yaw_pid_trace_.feedback_yaw_rad = input_hwt_rot_z_;
            yaw_pid_trace_.error_deg = radToDegF32(shortestAngularDistanceF32(input_hwt_rot_z_, lock_now_rot_z_target_));
            yaw_pid_trace_.manual_omega_in_rad_s = omega_z;
            yaw_pid_trace_.pid_output_omega_rad_s = 0.0f;
            yaw_pid_trace_.final_omega_cmd_rad_s = 0.0f;
            yaw_pid_trace_.feedback_yaw_rate_rad_s = input_hwt_omega_z_;
            yaw_pid_trace_.shift_remaining_ms = static_cast<f32>(lock_now_rot_z_shift_count_);
            yaw_pid_trace_.pid_compute_fired = 0.0f;
            yaw_pid_trace_.steer_fault_any_active = steer_fault_any_active_ ? 1.0f : 0.0f;
            yaw_pid_trace_.all_homed = 0.0f;
            yaw_pid_trace_.high_speed_suppression_active = high_speed_drive_suppression_active_ ? 1.0f : 0.0f;
            yaw_pid_trace_.reverse_intent_active = reverse_intent_active_ ? 1.0f : 0.0f;

            if (omega_z == 0.0f)
            {
                // 用户已经松开手动旋转输入，开始进入“保持当前朝向”的阶段。
                // 这里不会立即让 PID 接管，而是先经过一个短暂的 shift 缓冲，避免手感突变。
                // 但在刚松开摇杆的最初一小段时间内，不立即让 PID 介入，而是先进入过渡缓冲：
                if (lock_now_rot_z_shift_count_ > 0)
                {
                    lock_now_rot_z_shift_count_--;
                    lock_now_rot_z_target_ = input_hwt_rot_z_;
                    out_rot_z = filterYawPidTarget(lock_now_rot_z_target_);
                    out_omega_z = 0.0f;
                    yaw_pid_trace_.mode_tag = 2.0f;
                    yaw_pid_trace_.target_yaw_rad = out_rot_z;
                    yaw_pid_trace_.feedback_yaw_rad = input_hwt_rot_z_;
                    yaw_pid_trace_.error_deg = radToDegF32(shortestAngularDistanceF32(input_hwt_rot_z_, out_rot_z));
                    yaw_pid_trace_.final_omega_cmd_rad_s = out_omega_z;
                    yaw_pid_trace_.shift_remaining_ms = static_cast<f32>(lock_now_rot_z_shift_count_);
                }
                else
                {
                    // 缓冲结束后，真正用于锁角的目标已经不是外部传入的 rot_z，
                    // 而是前面抓取并保持下来的 lock_now_rot_z_target_。
                    // rot_z_pid_count_ / rot_z_pid_period_ 共同控制姿态 PID 的实际计算节拍。
                    out_rot_z = lock_now_rot_z_target_;
                    if (rot_z_pid_count_ >= rot_z_pid_period_)
                    {
                        rot_z_pid_count_ = 0;
                        yaw_pid_trace_.pid_compute_fired = computeYawPidOmega(out_rot_z, input_hwt_rot_z_, out_omega_z) ? 1.0f : 0.0f;
                        yaw_pid_trace_.pid_output_omega_rad_s = out_omega_z;
                    }
                    else
                    {
                        // 非 PID 计算拍暂时沿用上一规划周期的 omega_z，减少输出抖动并维持角速度连续性。
                        out_omega_z = planned_data_.omega_z;
                    }
                    // rot_z_pid_count_ / rot_z_pid_period_ 共同控制姿态 PID 的实际计算节拍。
                    rot_z_pid_count_++;
                    yaw_pid_trace_.mode_tag = 3.0f;
                    yaw_pid_trace_.target_yaw_rad = out_rot_z;
                    yaw_pid_trace_.feedback_yaw_rad = input_hwt_rot_z_;
                    yaw_pid_trace_.error_deg = radToDegF32(shortestAngularDistanceF32(input_hwt_rot_z_, out_rot_z));
                    yaw_pid_trace_.final_omega_cmd_rad_s = out_omega_z;
                    yaw_pid_trace_.shift_remaining_ms = 0.0f;
                }
            }
            else
            {
                // 用户仍在主动要求旋转：
                // 1. 不进入锁角闭环，直接执行当前手动 omega_z；
                // 2. 同时把 out_rot_z 刷成当前 IMU 朝向，相当于不断更新“等会儿松手后要锁住的那个角”；
                // 3. 每次有手动旋转输入都重置缓冲计数器，为后续切回自动锁角预留平滑过渡窗口。
                lock_now_rot_z_target_ = input_hwt_rot_z_;
                out_rot_z = lock_now_rot_z_target_;
                out_omega_z = omega_z;
                lock_now_rot_z_shift_count_ = lock_now_rot_z_shift_time_ms_;
                resetYawPidTargetRuntime();
                yaw_pid_trace_.mode_tag = 1.0f;
                yaw_pid_trace_.target_yaw_rad = out_rot_z;
                yaw_pid_trace_.feedback_yaw_rad = input_hwt_rot_z_;
                yaw_pid_trace_.error_deg = radToDegF32(shortestAngularDistanceF32(input_hwt_rot_z_, out_rot_z));
                yaw_pid_trace_.final_omega_cmd_rad_s = out_omega_z;
                yaw_pid_trace_.shift_remaining_ms = static_cast<f32>(lock_now_rot_z_shift_count_);
            }
        }

        void Chassis::isLockToRotZ(bool is_lock, f32 tar_rot_z, f32 cur_rot_z, f32 &out_rot_z, f32 omega_z, f32 &out_omega_z)
        {
            if (!is_lock)
            {
                out_rot_z = tar_rot_z;
                out_omega_z = omega_z;
                yaw_pid_trace_ = YawPidTraceState{};
                resetYawPidTargetRuntime();
                return;
            }

            // “锁到指定航向”会先限制目标角变化率，再用姿态 PID 生成维持/逼近该目标角度所需的 omega_z。
            // 这样外层给出的目标角不会瞬间跳变，底盘转向更平滑。
            const f32 rate_limited_rot_z = limit1DPiAngleRateByTimeF32(tar_rot_z, cur_rot_z, period_, max_lock_to_rot_z_rad_s_);
            out_rot_z = filterYawPidTarget(rate_limited_rot_z);
            // LockTo 生效的锁角会被 LockNow 继承，避免切回时重新抓 IMU。
            lock_now_rot_z_target_ = rate_limited_rot_z;
            lock_now_rot_z_shift_count_ = 0U;
            yaw_pid_trace_.mode_tag = 4.0f;
            yaw_pid_trace_.target_yaw_rad = out_rot_z;
            yaw_pid_trace_.feedback_yaw_rad = input_hwt_rot_z_;
            yaw_pid_trace_.error_deg = radToDegF32(shortestAngularDistanceF32(input_hwt_rot_z_, out_rot_z));
            yaw_pid_trace_.manual_omega_in_rad_s = omega_z;
            yaw_pid_trace_.pid_output_omega_rad_s = 0.0f;
            yaw_pid_trace_.final_omega_cmd_rad_s = 0.0f;
            yaw_pid_trace_.feedback_yaw_rate_rad_s = input_hwt_omega_z_;
            yaw_pid_trace_.shift_remaining_ms = 0.0f;
            yaw_pid_trace_.pid_compute_fired = 0.0f;
            yaw_pid_trace_.steer_fault_any_active = steer_fault_any_active_ ? 1.0f : 0.0f;
            yaw_pid_trace_.all_homed = 0.0f;
            yaw_pid_trace_.high_speed_suppression_active = high_speed_drive_suppression_active_ ? 1.0f : 0.0f;
            yaw_pid_trace_.reverse_intent_active = reverse_intent_active_ ? 1.0f : 0.0f;
            if (rot_z_pid_count_ >= rot_z_pid_period_)
            {
                rot_z_pid_count_ = 0;
                yaw_pid_trace_.pid_compute_fired = computeYawPidOmega(out_rot_z, input_hwt_rot_z_, out_omega_z) ? 1.0f : 0.0f;
                yaw_pid_trace_.pid_output_omega_rad_s = out_omega_z;
            }
            else
            {
                out_omega_z = planned_data_.omega_z;
            }
            yaw_pid_trace_.final_omega_cmd_rad_s = out_omega_z;
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
            // 这一层负责把“外部希望底盘如何运动”统一折叠成 planner 只关心的 target_data_：
            // - API 输入与 debug target injection 统一入口
            // - 世界系/车体系目标归一到车体系
            // - yaw lock 进入、保持、退出时的目标修正
            // resolve 完成后，后续规划层就不再关心目标来源，只看 target_data_。
            yaw_lock_zero_stop_preview_command_valid_ = false;
            yaw_lock_zero_stop_preview_command_ = Data{};
            const PlannerInputCommand planner_command{
                input_target_data_.vel_x,
                input_target_data_.vel_y,
                input_target_data_.omega_z,
                input_target_data_.rot_z,
                current_mode_flag_.is_world_speed_mode,
                input_target_data_.mode == Mode::kSteerAngleAndDriveSpeedMode,
            };
            const CommandInputSource source =
#if JIA_CHASSIS_ENABLE_DEBUG_OVERRIDE
                (classifyDebugControlRoute(debug_control_.common.enable, debug_control_.common.mode_raw) == DebugControlRoute::kTargetInjection)
                    ? CommandInputSource::kDebugTarget
                    : CommandInputSource::kApi;
#else
                CommandInputSource::kApi;
#endif
            normalized_body_command_ = makeNormalizedBodyCommand(planner_command, input_hwt_rot_z_, source);
            target_data_.vel_x = normalized_body_command_.body.vel_x;
            target_data_.vel_y = normalized_body_command_.body.vel_y;
            target_data_.omega_z = normalized_body_command_.body.omega_z;
            target_data_.rot_z = normalized_body_command_.rot_z;

            const bool debug_module_override_active =
#if JIA_CHASSIS_ENABLE_DEBUG_OVERRIDE
                classifyDebugControlRoute(debug_control_.common.enable, debug_control_.common.mode_raw) == DebugControlRoute::kModuleOverride;
#else
                false;
#endif
            const bool yaw_lock_control_requested =
                current_mode_flag_.is_lock_now_rot_z || current_mode_flag_.is_lock_to_rot_z;
            const bool yaw_lock_control_active =
                yaw_lock_control_requested && !input_target_data_.zero_current_all && !debug_module_override_active;
            if (!yaw_lock_control_active || !yaw_lock_control_active_last_cycle_)
            {
                lock_now_rot_z_target_ = input_hwt_rot_z_;
                lock_now_rot_z_shift_count_ = 0U;
                resetYawPidTargetRuntime();
            }

            if (yaw_lock_control_active && current_mode_flag_.is_lock_now_rot_z)
            {
                isLockNowRotZ(true, target_data_.rot_z, target_data_.omega_z, target_data_.rot_z, target_data_.omega_z);
            }
            if (yaw_lock_control_active && current_mode_flag_.is_lock_to_rot_z)
            {
                isLockToRotZ(true, input_target_data_.rot_z, lock_now_rot_z_target_, target_data_.rot_z, target_data_.omega_z, target_data_.omega_z);
            }
            yaw_lock_control_active_last_cycle_ = yaw_lock_control_active;
        }

        void Chassis::updatePlannedMotionData()
        {
            // updatePlannedMotionData() 只处理“车体级”目标，不直接生成单轮命令。
            // 它完成的事情包括：
            // 1. 选择生效的手动速度规划模式
            // 2. 判断是否进入 launch hold
            // 3. 为 zero-stop / yaw lock 生成必要的预览目标
            // 4. 输出本周期 planned_data_（速度、加速度、目标朝向）
            const ManualSpeedProfileMode effective_profile_mode = resolveEffectiveManualSpeedProfileMode();
            if (effective_profile_mode != active_manual_speed_profile_mode_)
            {
                active_manual_speed_profile_mode_ = effective_profile_mode;
                resetManualSpeedProfileRuntimeState(true);
            }

            if (!launch_hold_active_ &&
                (xpark_gate_active_ || (effective_profile_mode == ManualSpeedProfileMode::kSCurve)) &&
                shouldActivateLaunchHold())
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
                launch_hold_preview_cache_ = planSwerveModules(makeSwervePlannerInput(makeLaunchHoldPreviewCommand()));
                launch_hold_preview_cache_.valid = true;
                if (isLaunchHoldAligned(launch_hold_preview_cache_))
                {
                    launch_hold_active_ = false;
                    launch_hold_preview_cache_.valid = false;
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

            const bool suppress_yaw_lock_target_omega_for_zero_stop =
                !launch_hold_active_ && shouldPreviewSuppressYawLockOmegaForZeroStopDecel(target_data_);
            if (suppress_yaw_lock_target_omega_for_zero_stop)
            {
                yaw_lock_zero_stop_preview_command_ = target_data_;
                yaw_lock_zero_stop_preview_command_valid_ = true;
                target_data_.omega_z = 0.0f;
            }

            limitPlannedSpeed(target_data_.vel_x, target_data_.vel_y, target_data_.omega_z,
                              planned_data_.vel_x, planned_data_.vel_y, planned_data_.omega_z);

            planned_data_.acc_x = (planned_data_.vel_x - last_planned_data_.vel_x) / period_;
            planned_data_.acc_y = (planned_data_.vel_y - last_planned_data_.vel_y) / period_;
            planned_data_.alpha_z = (planned_data_.omega_z - last_planned_data_.omega_z) / period_;
            planned_data_.rot_z = target_data_.rot_z;
        }

        void Chassis::limitPlannedSpeed(f32 tar_vel_x, f32 tar_vel_y, f32 tar_omega_z, f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z)
        {
            // 第一阶段：先对 x / y / omega 分量分别做加减速限幅，保证速度台阶被平滑化。
            const ManualSpeedProfileMode effective_profile_mode = resolveEffectiveManualSpeedProfileMode();
            if (effective_profile_mode == ManualSpeedProfileMode::kSCurve)
            {
                out_vel_x = limitValueByJerkProfile(tar_vel_x,
                                                    last_planned_data_.vel_x,
                                                    manual_vel_x_shape_state_,
                                                    runtime_strategy_cfg_.manual_trans_acc_acc_,
                                                    runtime_strategy_cfg_.manual_trans_acc_dec_,
                                                    runtime_strategy_cfg_.manual_trans_jerk_acc_,
                                                    runtime_strategy_cfg_.manual_trans_jerk_dec_,
                                                    runtime_strategy_cfg_.manual_trans_settle_vel_eps_,
                                                    runtime_strategy_cfg_.manual_trans_settle_acc_eps_);
                out_vel_y = limitValueByJerkProfile(tar_vel_y,
                                                    last_planned_data_.vel_y,
                                                    manual_vel_y_shape_state_,
                                                    runtime_strategy_cfg_.manual_trans_acc_acc_,
                                                    runtime_strategy_cfg_.manual_trans_acc_dec_,
                                                    runtime_strategy_cfg_.manual_trans_jerk_acc_,
                                                    runtime_strategy_cfg_.manual_trans_jerk_dec_,
                                                    runtime_strategy_cfg_.manual_trans_settle_vel_eps_,
                                                    runtime_strategy_cfg_.manual_trans_settle_acc_eps_);
                out_omega_z = limitValueByJerkProfile(tar_omega_z,
                                                      last_planned_data_.omega_z,
                                                      manual_omega_z_shape_state_,
                                                      runtime_strategy_cfg_.manual_yaw_alpha_acc_,
                                                      runtime_strategy_cfg_.manual_yaw_alpha_dec_,
                                                      runtime_strategy_cfg_.manual_yaw_jerk_acc_,
                                                      runtime_strategy_cfg_.manual_yaw_jerk_dec_,
                                                      runtime_strategy_cfg_.manual_yaw_settle_vel_eps_,
                                                      runtime_strategy_cfg_.manual_yaw_settle_acc_eps_);
            }
            else
            {
                out_vel_x = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_vel_x, last_planned_data_.vel_x, period_, runtime_strategy_cfg_.max_acc_xy_acc_, runtime_strategy_cfg_.max_acc_xy_dec_);
                out_vel_y = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_vel_y, last_planned_data_.vel_y, period_, runtime_strategy_cfg_.max_acc_xy_acc_, runtime_strategy_cfg_.max_acc_xy_dec_);
                out_omega_z = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_omega_z, last_planned_data_.omega_z, period_, runtime_strategy_cfg_.max_alpha_z_acc_, runtime_strategy_cfg_.max_alpha_z_dec_);
            }

            // 第二阶段：再对平移矢量方向做限幅，处理低速冻结滞回和方向角速率限制。
            const f32 tar_mag = magnitude2DF32(tar_vel_x, tar_vel_y);
            const f32 out_mag = magnitude2DF32(out_vel_x, out_vel_y);
            const f32 enter_speed = getNearZeroEnterSpeedMps();
            const f32 exit_speed = getNearZeroExitSpeedMps();
            const f32 dir_rate_limit_rad_s = degToRadF32((runtime_strategy_cfg_.trans_dir_rate_limit_deg_s_ >= 0.0f) ? runtime_strategy_cfg_.trans_dir_rate_limit_deg_s_ : 0.0f);
            const f32 max_dir_step = dir_rate_limit_rad_s * period_;
            bool entered_freeze_now = false;
            trans_dir_tar_mag_m_s_ = tar_mag;
            trans_dir_out_mag_m_s_ = out_mag;
            trans_dir_freeze_reason_ = 0U;
            reverse_intent_dir_err_deg_ = 0.0f;
            const f32 requested_dir_rad = (tar_mag > 1.0e-6f) ? atan2f(tar_vel_y, tar_vel_x) : 0.0f;

            if (!trans_dir_ref_valid_ && out_mag > 1.0e-6f)
            {
                trans_dir_ref_rad_ = atan2f(out_vel_y, out_vel_x);
                trans_dir_ref_valid_ = true;
            }

            const f32 reverse_reference_dir_rad = trans_dir_ref_valid_ ? trans_dir_ref_rad_ : atan2f(last_planned_data_.vel_y, last_planned_data_.vel_x);
            if (tar_mag > 1.0e-6f)
            {
                reverse_intent_dir_err_deg_ =
                    radToDegF32(fabsf(shortestAngularDistanceF32(reverse_reference_dir_rad, requested_dir_rad)));
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

            const bool allow_immediate_reverse_intent = (effective_profile_mode != ManualSpeedProfileMode::kSCurve);
            reverse_intent_active_ = allow_immediate_reverse_intent &&
                                     shouldActivateReverseIntent(tar_vel_x, tar_vel_y, reverse_reference_dir_rad);
            if (reverse_intent_active_)
            {
                out_vel_x = out_mag * cosRadF32(requested_dir_rad);
                out_vel_y = out_mag * sinRadF32(requested_dir_rad);
                trans_dir_freeze_active_ = false;
                trans_dir_ref_valid_ = true;
                trans_dir_ref_rad_ = requested_dir_rad;
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
                    out_vel_x = out_mag * cosRadF32(trans_dir_ref_rad_);
                    out_vel_y = out_mag * sinRadF32(trans_dir_ref_rad_);
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
                const f32 dir_delta = shortestAngularDistanceF32(trans_dir_ref_rad_, target_dir_rad);
                const f32 clamped_delta = clampValue(dir_delta, -max_dir_step, max_dir_step);
                output_dir_rad = wrapToPiF32(trans_dir_ref_rad_ + clamped_delta);
            }

            trans_dir_ref_rad_ = output_dir_rad;
            out_vel_x = out_mag * cosRadF32(output_dir_rad);
            out_vel_y = out_mag * sinRadF32(output_dir_rad);
        }

        void Chassis::resetHomingEdgeConfirmState(WheelConfig &wheel)
        {
            // 这组缓存用于“多次边沿确认同一零偏”。
            // 每次重新开始 search，或判定边沿序列失真时，都要把累计值和计数清空。
            wheel.homing_edge_confirm_count = 0U;
            wheel.homing_last_confirm_edge_is_falling = false;
            wheel.homing_last_confirm_signed_local_rad = 0.0f;
            wheel.homing_candidate_zero_offset_sum_rad = 0.0f;
        }

        bool Chassis::recordHomingEdgeAndCheckConfirmed(WheelConfig &wheel, bool is_falling_edge, f32 signed_local_total_rad)
        {
            // 单次边沿只能提供一个“候选零偏”；为了吸收抖动和机械误差，这里要连续收集三次，
            // 并要求相邻边沿位置近似相差半圈，才把平均结果认作运行时零偏。
            const f32 edge_mech_oa_rad = is_falling_edge ? wheel.homing_falling_edge_mech_rad : wheel.homing_rising_edge_mech_rad;
            const SteerCalibration calibration = makeSteerCalibration(wheel);
            f32 candidate_zero_offset_rad = computeHomingRuntimeZeroOffset(edge_mech_oa_rad,
                                                                           signed_local_total_rad,
                                                                           wheel.homing_zero_offset_rad,
                                                                           calibration);

            if (wheel.homing_edge_confirm_count == 0U)
            {
                wheel.homing_edge_confirm_count = 1U;
                wheel.homing_last_confirm_edge_is_falling = is_falling_edge;
                wheel.homing_last_confirm_signed_local_rad = signed_local_total_rad;
                wheel.homing_candidate_zero_offset_sum_rad = candidate_zero_offset_rad;
                return false;
            }

            const f32 delta_rad = signed_local_total_rad - wheel.homing_last_confirm_signed_local_rad;
            const f32 tolerance_rad = degToRadF32(JIA_CHASSIS_HOMING_EDGE_DELTA_TOLERANCE_DEG);
            if (fabsf(delta_rad - kPi) > tolerance_rad)
            {
                resetHomingEdgeConfirmState(wheel);
                wheel.homing_state = HomingState::kFault;
                wheel.homing_zero_valid = false;
                return false;
            }

            const f32 previous_average_offset_rad =
                wheel.homing_candidate_zero_offset_sum_rad / static_cast<f32>(wheel.homing_edge_confirm_count);
            candidate_zero_offset_rad = nearestEquivalentAngleF32(previous_average_offset_rad, candidate_zero_offset_rad);
            wheel.homing_candidate_zero_offset_sum_rad += candidate_zero_offset_rad;
            wheel.homing_edge_confirm_count += 1U;
            wheel.homing_last_confirm_edge_is_falling = is_falling_edge;
            wheel.homing_last_confirm_signed_local_rad = signed_local_total_rad;

            if (wheel.homing_edge_confirm_count < 3U)
            {
                return false;
            }

            wheel.homing_runtime_zero_offset_rad = wheel.homing_candidate_zero_offset_sum_rad / 3.0f;
            wheel.homing_hold_corrected_local_total_rad = signed_local_total_rad + wheel.homing_runtime_zero_offset_rad;
            wheel.homing_zero_valid = true;
            return true;
        }

        bool Chassis::readHomingSensor(const WheelConfig &wheel) const
        {
            // readHomingSensor() 返回的是“语义上的传感器激活态”，已经吸收了 active_high / active_low 差异。
            // 只有需要判断物理边沿方向时，才直接读取 raw high。
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
            // 原始总角保留电机安装符号语义，用于故障冻结检测和 homing 边沿位置记录。
            if (wheel.steer_motor_h == nullptr)
            {
                return 0.0f;
            }
            const f32 steer_sign = (wheel.steer_motor_sign == 0.0f) ? 1.0f : wheel.steer_motor_sign;
            return steer_sign * degToRadF32(wheel.steer_motor_h->getTotalAngle());
        }

        f32 Chassis::readDriveMotorOmegaRadS(const WheelConfig &wheel) const
        {
            // 驱动反馈需要把“电机 RPM”重新折回“轮子沿当前 OA 方向的角速度”语义。
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
            // corrected local total 是底盘内部最稳定的舵向连续角语义：
            // 已吸收安装方向、零偏与 OA/OWI 坐标差异，planner / homing / hold 都围绕它工作。
            const SteerCalibration calibration{
                wheel.theta_oa_to_owi_rad,
                wheel.homing_runtime_zero_offset_rad,
                wheel.steer_motor_sign,
                wheel.drive_motor_sign,
            };
            return mapRawSteerMotorTotalToCorrectedLocalTotal((wheel.steer_motor_h == nullptr) ? 0.0f : degToRadF32(wheel.steer_motor_h->getTotalAngle()), calibration);
        }

        f32 Chassis::readSteerMotorCurrentMilliAmp(const WheelConfig &wheel) const
        {
            return (wheel.steer_motor_h == nullptr) ? 0.0f : wheel.steer_motor_h->getCurrent();
        }

        void Chassis::clearSteerFaultState(WheelConfig &wheel)
        {
            // 从 recovering 回到正常控制前，冻结观测历史也必须一起刷新；
            // 否则上一轮故障遗留的电流/角度 delta 可能在下一拍继续触发误判。
            wheel.steer_fault_state = SteerFaultState::kNone;
            wheel.steer_fault_rehome_request = false;
            wheel.steer_feedback_freeze_ms = 0U;
            wheel.steer_feedback_recovery_toggle_count = 0U;
            wheel.steer_feedback_current_mA = readSteerMotorCurrentMilliAmp(wheel);
            wheel.steer_feedback_last_current_mA = wheel.steer_feedback_current_mA;
            wheel.steer_feedback_last_raw_total_angle_rad = readSteerMotorRawTotalAngleRad(wheel);
            wheel.steer_feedback_current_delta_mA = 0.0f;
            wheel.steer_feedback_angle_delta_rad = 0.0f;
            wheel.steer_fault_steer_error_rad = 0.0f;
            wheel.steer_fault_control_intent = false;
            wheel.steer_fault_xpark_stationary_hold = false;
            wheel.steer_fault_freeze_candidate = false;
        }

        void Chassis::latchSteerFault(WheelConfig &wheel)
        {
            // 舵向冻结一旦确认，就立刻把该轮打进“已锁存故障”：
            // 退出当前闭环目标、清空零位有效性，并等待后续恢复抖动触发 re-home。
            wheel.steer_fault_state = SteerFaultState::kLatched;
            wheel.steer_fault_rehome_request = false;
            wheel.steer_feedback_recovery_toggle_count = 0U;
            wheel.steer_fault_latched_count += 1U;
            wheel.homing_state = HomingState::kFault;
            wheel.homing_elapsed_s = 0.0f;
            wheel.homing_zero_valid = false;
            wheel.target_drive_omega_rad_s = 0.0f;
            wheel.target_steer_motor_total_angle_rad = 0.0f;
            wheel.steer_target_velocity_rad_s = 0.0f;
            resetSteerMotorClosedLoopState(wheel);
            steer_fault_any_active_ = true;
        }

        void Chassis::requestSingleWheelHoming(WheelConfig &wheel)
        {
            // recovering 不是直接“清故障继续跑”，而是要求这一个轮子重新走一遍最小回零流程。
            // 这里把 search 相关缓存复位到重新入场的干净状态。
            wheel.steer_fault_rehome_request = true;
            wheel.homing_elapsed_s = 0.0f;
            wheel.homing_last_sensor_active = readHomingSensorRawHigh(wheel);
            wheel.homing_last_edge_is_falling = false;
            wheel.homing_align_command_armed = false;
            wheel.homing_search_timeout_armed = false;
            resetHomingEdgeConfirmState(wheel);
            wheel.homing_runtime_zero_offset_rad = wheel.homing_zero_offset_rad;
            wheel.homing_hold_corrected_local_total_rad = 0.0f;
            wheel.homing_zero_valid = false;
            wheel.homing_state = HomingState::kIdle;
            wheel.target_drive_omega_rad_s = 0.0f;
            wheel.target_steer_motor_total_angle_rad = 0.0f;
            wheel.steer_target_velocity_rad_s = 0.0f;
        }

        void Chassis::resetSteerMotorClosedLoopState(WheelConfig &wheel)
        {
            // 断电重连或冻结锁存后，舵向闭环内部历史很可能已经和电机真实状态脱节。
            // 进入 recover/homing 前先把电流输出和 PID 累积量清零，避免恢复第一拍带着旧积分冲出去。
            if (wheel.steer_motor_h == nullptr)
            {
                return;
            }

            wheel.steer_motor_h->setTargetCurrent(0.0f);

        #ifdef TEST_TDD_MOTOR_DJI_H
            return;
        #else
            // Project wiring binds all four steer motors to M3508 in Setup_ConfigInit.cpp.
            M3508 *steer_m3508 = static_cast<M3508 *>(wheel.steer_motor_h);
            steer_m3508->speed_pid_.reset();
            steer_m3508->angle_pid_.reset();
            steer_m3508->anglePid_timeCnt = 0;
            steer_m3508->setTargetCurrent(0.0f);
        #endif
        }

        void Chassis::updateSteerFaultState(WheelConfig &wheel)
        {
            // 这套故障检测不是简单“有电流但不动就报错”，而是同时检查：
            // 1. 当前是否真的存在 steer 控制意图；
            // 2. 是否处于允许忽略的 X-Park 静止保持窗口；
            // 3. 电流变化和角度变化是否同时冻结到可疑范围。
            const StrategyConfig::SteerFaultConfig &steer_fault_cfg = runtime_strategy_cfg_.steer_fault_cfg;
            const f32 current_mA = readSteerMotorCurrentMilliAmp(wheel);
            const f32 raw_total_angle_rad = readSteerMotorRawTotalAngleRad(wheel);
            const f32 current_delta_mA = fabsf(current_mA - wheel.steer_feedback_last_current_mA);
            const f32 angle_delta_rad = fabsf(raw_total_angle_rad - wheel.steer_feedback_last_raw_total_angle_rad);
            const f32 command_speed_m_s = computeMaxCommandWheelSpeedMps(target_data_);
            const bool xpark_stationary_hold = steer_fault_cfg.ignore_during_xpark_hold &&
                                               xpark_gate_active_ &&
                                               (command_speed_m_s <= getXParkCommandExitSpeedMps());
            const f32 steer_error_rad = fabsf(wrapToPiF32(wheel.target_steer_motor_total_angle_rad -
                                                       wheel.corrected_steer_motor_total_angle_rad));
            const bool steer_control_intent = !input_target_data_.zero_current_all &&
                                              !current_mode_flag_.is_wheel_torque_free &&
                                              ((command_speed_m_s > getXParkCommandExitSpeedMps()) ||
                                               (steer_error_rad > degToRadF32(homing_align_to_zero_tolerance_deg_)));
            const bool fault_detection_enabled_for_wheel = !input_target_data_.zero_current_all &&
                                                           !current_mode_flag_.is_wheel_torque_free;
            const bool feedback_frozen_candidate = (fabsf(current_mA) >= steer_fault_cfg.active_current_min_mA) &&
                                                   (current_delta_mA <= steer_fault_cfg.freeze_current_delta_mA) &&
                                                   (angle_delta_rad <= steer_fault_cfg.freeze_angle_delta_rad);
            const bool freeze_candidate = (wheel.homing_state == HomingState::kReady) &&
                                          wheel.homing_zero_valid &&
                                          fault_detection_enabled_for_wheel &&
                                          feedback_frozen_candidate;

            wheel.steer_feedback_current_mA = current_mA;
            wheel.steer_feedback_current_delta_mA = current_delta_mA;
            wheel.steer_feedback_angle_delta_rad = angle_delta_rad;
            wheel.steer_fault_steer_error_rad = steer_error_rad;
            wheel.steer_fault_control_intent = steer_control_intent;
            wheel.steer_fault_xpark_stationary_hold = xpark_stationary_hold;
            wheel.steer_fault_freeze_candidate = freeze_candidate;

            if (!steer_fault_cfg.enable)
            {
                wheel.steer_fault_rehome_request = false;
                wheel.steer_feedback_freeze_ms = 0U;
                wheel.steer_feedback_recovery_toggle_count = 0U;
                wheel.steer_fault_state = SteerFaultState::kNone;
                wheel.steer_feedback_last_current_mA = current_mA;
                wheel.steer_feedback_last_raw_total_angle_rad = raw_total_angle_rad;
                return;
            }

            if (wheel.steer_fault_state == SteerFaultState::kNone)
            {
                if (freeze_candidate)
                {
                    wheel.steer_feedback_freeze_ms = (wheel.steer_feedback_freeze_ms > (0xFFFFFFFFU - period_ms_))
                                                         ? 0xFFFFFFFFU
                                                         : (wheel.steer_feedback_freeze_ms + period_ms_);
                    if (wheel.steer_feedback_freeze_ms >= steer_fault_cfg.freeze_duration_ms)
                    {
                        latchSteerFault(wheel);
                    }
                }
                else
                {
                    wheel.steer_feedback_freeze_ms = 0U;
                }
                wheel.steer_feedback_recovery_toggle_count = 0U;
            }
            else if (wheel.steer_fault_state == SteerFaultState::kLatched)
            {
                if (current_delta_mA >= steer_fault_cfg.recovery_current_delta_mA)
                {
                    wheel.steer_feedback_recovery_toggle_count += 1U;
                }
                if (wheel.steer_feedback_recovery_toggle_count >= steer_fault_cfg.recovery_toggle_threshold)
                {
                    wheel.steer_fault_state = SteerFaultState::kRecovering;
                    requestSingleWheelHoming(wheel);
                }
            }

            if (wheel.steer_fault_state != SteerFaultState::kNone)
            {
                steer_fault_any_active_ = true;
            }

            wheel.steer_feedback_last_current_mA = current_mA;
            wheel.steer_feedback_last_raw_total_angle_rad = raw_total_angle_rad;
        }

        void Chassis::updateWheelFeedback()
        {
            // 每拍先统一刷新四个模块反馈，再在同一拍上做故障检测。
            // 这样 steer fault 看到的电流/角度变化量都来自同一时间基准。
            steer_fault_any_active_ = false;
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                drive_feedback_sample_ms_[i] = static_cast<u32>(time_ms_);
                wheel.corrected_steer_motor_total_angle_rad = readCorrectedSteerMotorTotalAngleRad(wheel);
                wheel.corrected_drive_omega_rad_s = readDriveMotorOmegaRadS(wheel);
                updateSteerFaultState(wheel);
            }
        }

        bool Chassis::updateHomingState(WheelConfig &wheel)
        {
            // 四舵轮回零状态机的职责是：在每个周期读取限位/零位传感器，
            // 依次完成 Idle -> Search -> EdgeDetected -> OffsetApply -> ContinuousAngleReady -> Ready。
            // 这里不直接“判定一次就完成”，而是通过多周期状态推进来吸收传感器抖动和机械延迟。
            if (!wheel.homing_enabled || wheel.homing_gpio_port == nullptr)
            {
                wheel.homing_state = HomingState::kReady;
                wheel.homing_zero_valid = true;
                wheel.homing_runtime_zero_offset_rad = wheel.homing_zero_offset_rad;
                wheel.homing_last_edge_is_falling = false;
                wheel.homing_align_command_armed = false;
                resetHomingEdgeConfirmState(wheel);
                wheel.homing_hold_corrected_local_total_rad = wheel.corrected_steer_motor_total_angle_rad;
                return true;
            }

            const bool sensor_raw_high = readHomingSensorRawHigh(wheel);
            const f32 raw_total_angle_rad = readSteerMotorRawTotalAngleRad(wheel);

            if (wheel.steer_fault_state == SteerFaultState::kRecovering && wheel.steer_fault_rehome_request)
            {
                wheel.steer_fault_rehome_request = false;
                wheel.homing_state = HomingState::kSearch;
                wheel.homing_elapsed_s = 0.0f;
                wheel.homing_last_sensor_active = sensor_raw_high;
                wheel.homing_align_command_armed = false;
                wheel.homing_search_timeout_armed = false;
                resetHomingEdgeConfirmState(wheel);
                wheel.homing_zero_valid = false;
                steer_fault_any_active_ = true;
                return false;
            }

            if (wheel.homing_state == HomingState::kIdle)
            {
                if (homing_start_request_)
                {
                    // Search 从哪一相位开始并不重要，只要开始转动，半圈内总能遇到一个有效边沿。
                    wheel.homing_state = HomingState::kSearch;
                    wheel.homing_elapsed_s = 0.0f;
                    wheel.homing_search_timeout_armed = false;
                    resetHomingEdgeConfirmState(wheel);
                    wheel.homing_zero_valid = false;
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
                const bool has_first_valid_steer_feedback =
                    (fabsf(wheel.steer_feedback_current_mA) > 1.0f) ||
                    (wheel.steer_feedback_current_delta_mA > 1.0f) ||
                    (wheel.steer_feedback_angle_delta_rad > 1.0e-4f);
                if (!wheel.homing_search_timeout_armed)
                {
                    wheel.homing_elapsed_s = 0.0f;
                    if (has_first_valid_steer_feedback)
                    {
                        wheel.homing_search_timeout_armed = true;
                    }
                }
                else
                {
                    wheel.homing_elapsed_s += period_;
                }
                const bool is_edge = (sensor_raw_high != wheel.homing_last_sensor_active);
                if (is_edge)
                {
                    const bool is_falling_edge = wheel.homing_last_sensor_active && !sensor_raw_high;
                    wheel.homing_last_edge_is_falling = is_falling_edge;
                    if (recordHomingEdgeAndCheckConfirmed(wheel, is_falling_edge, raw_total_angle_rad))
                    {
                        wheel.homing_state = HomingState::kEdgeDetected;
                    }
                }
                else if (wheel.homing_search_timeout_armed && (wheel.homing_elapsed_s > wheel.homing_timeout_s))
                {
                    if (wheel.steer_fault_state == SteerFaultState::kRecovering)
                    {
                        latchSteerFault(wheel);
                    }
                    else
                    {
                        wheel.homing_state = HomingState::kFault;
                    }
                }
                wheel.homing_last_sensor_active = sensor_raw_high;
                return false;
            }

            if (wheel.homing_state == HomingState::kEdgeDetected)
            {
                // 边沿已确认后，故意再走两个显式中间态，
                // 让“抓边沿”“应用零偏”“宣布连续角就绪”在调试观察上更容易区分。
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
                wheel.target_steer_motor_total_angle_rad = wheel.homing_hold_corrected_local_total_rad;
                wheel.homing_state = HomingState::kReady;
                wheel.homing_align_command_armed = false;
                if (wheel.steer_fault_state == SteerFaultState::kRecovering)
                {
                    clearSteerFaultState(wheel);
                }
                return true;
            }

            if (wheel.homing_state == HomingState::kAlignToZero)
            {
                const f32 current_local_total_rad = wheel.corrected_steer_motor_total_angle_rad;
                const f32 current_oa_total_rad = mapWheelCorrectedLocalToOaTotal(wheel, current_local_total_rad);
                const f32 target_oa_total_rad = nearestEquivalentAngleF32(current_oa_total_rad, 0.0f);
                const f32 target_local_total_rad = mapWheelOaTotalToCorrectedLocal(wheel, target_oa_total_rad);
                const f32 oa_error_abs_rad = fabsf(shortestAngularDistanceF32(current_oa_total_rad, target_oa_total_rad));

                wheel.target_steer_motor_total_angle_rad = target_local_total_rad;
                if (oa_error_abs_rad <= degToRadF32(homing_align_to_zero_tolerance_deg_))
                {
                    wheel.homing_state = HomingState::kReady;
                    wheel.homing_align_command_armed = false;
                    if (wheel.steer_fault_state == SteerFaultState::kRecovering)
                    {
                        clearSteerFaultState(wheel);
                    }
                    return true;
                }
                wheel.homing_align_command_armed = true;
                return false;
            }

            return wheel.homing_state == HomingState::kReady;
        }

        void Chassis::setSteerMotorTargetCurrent(WheelConfig &wheel, f32 current)
        {
            // 电流模式只做最薄的一层透传，方便故障态、zero-current 和 torque-free 统一复用。
            if (wheel.steer_motor_h != nullptr)
            {
                wheel.steer_motor_h->setTargetCurrent(current);
            }
        }

        void Chassis::setSteerMotorTargetRPM(WheelConfig &wheel, f32 rpm)
        {
            if (wheel.steer_motor_h != nullptr)
            {
                // 底盘内部的 rpm 语义是“按 corrected local 正方向搜索”；
                // 下发到具体电机前要先除以安装方向符号，恢复成电机对象理解的目标值。
                const f32 steer_sign = (wheel.steer_motor_sign == 0.0f) ? 1.0f : wheel.steer_motor_sign;
                const f32 expected_motor_rpm = rpm / steer_sign;
                wheel.steer_motor_h->setTargetRPM(expected_motor_rpm);
                // 断电重连现场偶发出现“底盘已写 RPM，但电机对象没接住”的情况，
                // 因此这里统一做一次写后核对，必要时同周期立即补发一次。
                if (fabsf(wheel.steer_motor_h->getTargetRPM() - expected_motor_rpm) > 1.0e-6f)
                {
                    wheel.steer_motor_h->setTargetRPM(expected_motor_rpm);
                }
            }
        }

        void Chassis::setSteerMotorTargetTotalAngleRad(WheelConfig &wheel, f32 corrected_local_total_angle_rad)
        {
            // 位置闭环下发前，需要把底盘内部 corrected local 连续角重新映射回电机原始 total angle。
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
            // drive 下发同理：底盘内部一直讲“轮子角速度”，实际接口讲“电机 RPM”，
            // 中间需要结合零偏、OA/OWI 和安装方向做一次完整换算。
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

        f32 Chassis::limitPositionSecondOrder(f32 current_value, f32 current_rate, f32 target_value, f32 max_rate, f32 max_accel, f32 dt_s, f32 &next_rate) const
        {
            // 这是一个“位置目标 + 速度上限 + 加速度上限”的二阶限幅器。
            // 在四舵轮里它主要用于舵角规划：既限制角速度，也限制角速度变化率，避免恢复/翻转时出现跳变。
            const f32 safe_dt = (dt_s <= 1.0e-6f) ? 1.0e-3f : dt_s;

            // desired_rate 表示“若想尽快逼近目标，本拍理论上会想要的速度”，
            // 但它会先受 max_rate 限制，避免直接给出不可能达到的指令。
            const f32 delta_value = target_value - current_value;
            const f32 desired_rate = clampValue(delta_value / safe_dt, -max_rate, max_rate);

            // rate_delta_limit 是“这一拍速度最多允许变化多少”，由最大加速度决定。
            const f32 rate_delta_limit = max_accel * safe_dt;

            next_rate = current_rate;

            // 先做速度变化率限制，再做一次绝对速度限幅，保证最终速度不超过 max_rate。
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

            next_rate = clampValue(next_rate, -max_rate, max_rate);

            // 按这一拍最终允许的速度积分出位置步进量。
            f32 step_value = next_rate * safe_dt;

            // 如果这一拍已经足够到达目标，则直接截断到目标位置，避免积分后跨 target_value 造成过冲。
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
            // 这里把车体级 planned_data_ 转换为每个轮子的执行目标。
            // 如果说 updatePlannedMotionData() 解决“整车应该怎么动”，
            // 那 computeModuleCommands() 解决的就是“每个轮子各自该转到哪里、驱动多快”。
            SwervePlannerOutput planner_output = {};
            if (launch_hold_active_ && launch_hold_preview_cache_.valid)
            {
                planner_output = launch_hold_preview_cache_;
            }
            else
            {
                Data planner_command = launch_hold_active_ ? makeLaunchHoldPreviewCommand() : command_data;
                if (!launch_hold_active_ && yaw_lock_zero_stop_preview_command_valid_)
                {
                    planner_command = yaw_lock_zero_stop_preview_command_;
                }
                if (!launch_hold_active_ && shouldSuppressYawLockOmegaForZeroStopDecel(planner_command))
                {
                    planner_command.omega_z = 0.0f;
                }
                const SwervePlannerInput planner_input = makeSwervePlannerInput(planner_command);
                planner_output = planSwerveModules(planner_input);
            }
            if (launch_hold_active_)
            {
                for (u8 i = 0; i < 4; ++i)
                {
                    planner_output.low_speed_suppression_scale[i] = 0.0f;
                    planner_output.final_drive_omega_rad_s[i] = 0.0f;
                }
            }
            ActuatorCommandFrame command_frame{};
            buildActuatorCommandFrame(planner_output, command_frame);
            storePlannedActuatorFrame(planner_output, command_frame);
        }

        void Chassis::applyModuleCommands(bool all_homed)
        {
            // 这是底盘线程的最终执行仲裁层。
            // planner 已经给出了理想模块目标，但这里还要再结合运行态决定“是否允许原样下发”。
            // 它同时处理：
            // - homing 未完成时的安全降级
            // - steer fault / recovering 期间的恢复控制
            // - zero current / torque free / 单轮隔离等强制模式
            // - drive zero-stop assist 与 X-Park steer hold 的末端执行语义
#if JIA_CHASSIS_ENABLE_SINGLE_WHEEL_DEBUG
            const DebugMode debug_mode = resolveDebugMode(debug_control_.common.mode_raw);
            const bool single_wheel_isolation_active =
                debug_control_.common.enable && isSingleWheelIsolatedMode(debug_mode);
            const u8 single_wheel_idx = (debug_control_.common.control_wheel_index < 4U) ? debug_control_.common.control_wheel_index : 0U;
#else
            const bool single_wheel_isolation_active = false;
            const u8 single_wheel_idx = 0U;
#endif
#if JIA_CHASSIS_ENABLE_DEBUG_OUTPUT
            debug_drive_load_trace_ = {};
            debug_drive_load_trace_.observe_wheel_idx = static_cast<f32>((debug_control_.common.observe_wheel_index < 4U) ? debug_control_.common.observe_wheel_index : 0U);
#endif
            bool steer_fault_any_active = false;
            for (u8 i = 0; i < 4; ++i)
            {
                if (wheel_config_[i].steer_fault_state != SteerFaultState::kNone)
                {
                    steer_fault_any_active = true;
                    break;
                }
            }
            steer_fault_any_active_ = steer_fault_any_active;
            const bool chassis_motion_blocked = !all_homed || steer_fault_any_active;
            const bool prev_drive_zero_stop_active = drive_zero_stop_active_;
            const bool yaw_lock_zero_stop_hold_pre_active =
                yaw_lock_zero_stop_decel_context_active_ || (yaw_lock_zero_stop_release_hold_elapsed_ms_ > 0U);
            if (runtime_strategy_cfg_.enable_drive_zero_stop_assist &&
                !single_wheel_isolation_active &&
                !input_target_data_.zero_current_all &&
                !current_mode_flag_.is_wheel_torque_free &&
                !chassis_motion_blocked)
            {
                // 只有“整车目标意图”和“当前执行帧目标”都已经逼近静止时，才进入 zero-stop assist。
                // 这样既不会误伤正常起步/恢复第一拍，也不会把宿主测试里手工塞入的 drive 命令当成静止收尾。
                const bool use_body_target_for_zero_stop_gate =
                    (input_target_data_.mode == Mode::kBodySpeedMode) ||
                    (input_target_data_.mode == Mode::kBodySpeedLockNowRotZMode) ||
                    (input_target_data_.mode == Mode::kBodySpeedLockNowRotZWithNoOmegaZMode) ||
                    (input_target_data_.mode == Mode::kBodySpeedLockToRotZMode) ||
                    (input_target_data_.mode == Mode::kWorldSpeedMode) ||
                    (input_target_data_.mode == Mode::kWorldSpeedLockNowRotZMode) ||
                    (input_target_data_.mode == Mode::kWorldSpeedLockNowRotZWithNoOmegaZMode) ||
                    (input_target_data_.mode == Mode::kWorldSpeedLockToRotZMode);
                // 正常底盘链路下优先依据整车目标是否已静止来决定是否进入 zero-stop，
                // 避免速度规划尾巴还没完全衰减时，把刹车收尾整体拖后。
                f32 max_frame_command_speed_m_s = 0.0f;
                f32 max_last_delivered_speed_m_s = 0.0f;
                const f32 wheel_radius_m = fabsf(runtime_strategy_cfg_.wheel_radius_m_);
                for (u8 i = 0; i < 4; ++i)
                {
                    const f32 frame_command_speed_m_s = fabsf(actuator_command_frame_.drive_omega_rad_s[i]) * wheel_radius_m;
                    max_frame_command_speed_m_s = (frame_command_speed_m_s > max_frame_command_speed_m_s) ? frame_command_speed_m_s : max_frame_command_speed_m_s;
                    const f32 last_delivered_speed_m_s = fabsf(last_drive_omega_cmd_rad_s_[i]) * wheel_radius_m;
                    max_last_delivered_speed_m_s = (last_delivered_speed_m_s > max_last_delivered_speed_m_s) ? last_delivered_speed_m_s : max_last_delivered_speed_m_s;
                }
                Data zero_stop_gate_target_data = target_data_;
                const bool suppress_yaw_lock_omega_for_zero_stop =
                    yaw_lock_zero_stop_hold_pre_active;
                if (suppress_yaw_lock_omega_for_zero_stop)
                {
                    zero_stop_gate_target_data.omega_z = 0.0f;
                }
                const f32 target_command_speed_m_s = computeMaxCommandWheelSpeedMps(zero_stop_gate_target_data);
                const f32 max_command_speed_m_s = use_body_target_for_zero_stop_gate
                                                      ? target_command_speed_m_s
                                                      : ((target_command_speed_m_s > max_frame_command_speed_m_s)
                                                             ? target_command_speed_m_s
                                                             : max_frame_command_speed_m_s);
                const bool yaw_lock_control_requested =
                    current_mode_flag_.is_lock_now_rot_z || current_mode_flag_.is_lock_to_rot_z;
                const f32 drive_alpha_step_speed_m_s =
                    runtime_strategy_cfg_.enable_drive_alpha_limit_
                        ? (fabsf(runtime_strategy_cfg_.max_drive_alpha_rad_s2_) * period_ * wheel_radius_m)
                        : 0.0f;
                const bool yaw_lock_zero_command_decelerating =
                    yaw_lock_control_requested &&
                    !yaw_lock_zero_stop_decel_context_active_ &&
                    runtime_strategy_cfg_.enable_drive_alpha_limit_ &&
                    (drive_alpha_step_speed_m_s > 1.0e-6f) &&
                    (target_command_speed_m_s <= getNearZeroEnterSpeedMps()) &&
                    (max_frame_command_speed_m_s <= getNearZeroEnterSpeedMps()) &&
                    (max_last_delivered_speed_m_s > (drive_alpha_step_speed_m_s + 1.0e-6f));

                if (yaw_lock_zero_command_decelerating)
                {
                    drive_zero_stop_active_ = false;
                }
                else if (drive_zero_stop_active_)
                {
                    drive_zero_stop_active_ = max_command_speed_m_s <= getNearZeroExitSpeedMps();
                }
                else
                {
                    drive_zero_stop_active_ = max_command_speed_m_s <= getNearZeroEnterSpeedMps();
                }
            }
            else
            {
                drive_zero_stop_active_ = false;
                yaw_lock_zero_stop_decel_context_active_ = false;
                yaw_lock_zero_stop_release_hold_elapsed_ms_ = 0U;
            }
            const bool entering_drive_zero_stop = !prev_drive_zero_stop_active && drive_zero_stop_active_;
            const bool leaving_drive_zero_stop = prev_drive_zero_stop_active && !drive_zero_stop_active_;
            if (!drive_zero_stop_active_)
            {
                yaw_lock_zero_stop_release_hold_elapsed_ms_ = 0U;
                for (u8 i = 0; i < 4; ++i)
                {
                    drive_zero_stop_brake_active_[i] = false;
                    drive_zero_stop_brake_ramp_elapsed_ms_[i] = 0U;
                }
            }
            // 这里是“四舵轮目标命令”真正落到电机接口前的最后一道门控：
            // computeModuleCommands() 虽然已经为每个轮子算好了目标舵角和驱动速度，
            // 但是否允许按这些目标下发，还要看当前是否全部完成回零，以及是否处于扭矩自由模式。
            auto clearXParkSteerHoldState = [](WheelConfig &wheel) {
                wheel.xpark_steer_hold_phase = XParkSteerHoldPhase::kInactive;
                wheel.xpark_steer_hold_locked_target_rad = 0.0f;
                wheel.xpark_steer_hold_error_rad = 0.0f;
                wheel.xpark_steer_hold_target_rate_rad_s = 0.0f;
                wheel.xpark_steer_hold_settle_ms = 0U;
                wheel.xpark_steer_hold_reacquire_ms = 0U;
            };
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
                const bool isolate_this_wheel = single_wheel_isolation_active && (i != single_wheel_idx);

                if (input_target_data_.zero_current_all)
                {
                    // 硬零电流模式优先级最高：无论回零状态如何，四轮舵向/驱动都直接下 0 电流。
                    clearXParkSteerHoldState(wheel);
                    allowed_drive_target_rad_s = 0.0f;
                    wheel.target_drive_omega_rad_s = 0.0f;
                    planned_data_.drive_omega_rad_s[i] = 0.0f;
                    last_drive_omega_cmd_rad_s_[i] = 0.0f;
                    setSteerMotorTargetCurrent(wheel, 0.0f);
                    if (wheel.drive_motor_h != nullptr)
                    {
                        if (VESC_Motor *drive_vesc = dynamic_cast<VESC_Motor *>(wheel.drive_motor_h))
                        {
                            drive_vesc->setSpeedPidCurrentBias(0.0f);
                        }
                        wheel.drive_motor_h->setTargetCurrent(0.0f);
                    }
                    continue;
                }

                if (chassis_motion_blocked)
                {
                    // 只要还有任意一个轮子没有完成回零，或者存在舵向故障/恢复重校准中的轮子，
                    // drive 一律按“电流清零”停机，不走 RPM=0 的速度闭环停机语义，
                    // 避免离线前残留的驱动电流或速度闭环继续推动底盘。
                    clearXParkSteerHoldState(wheel);
                    allow_drive_position_loop = execution_allow_drive_position_loop[i];
                    allowed_drive_target_rad_s = 0.0f;
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
                    if (wheel.drive_motor_h != nullptr)
                    {
                        if (VESC_Motor *drive_vesc = dynamic_cast<VESC_Motor *>(wheel.drive_motor_h))
                        {
                            drive_vesc->setSpeedPidCurrentBias(0.0f);
                        }
                        wheel.drive_motor_h->setTargetCurrent(0.0f);
                    }
                    if (wheel.homing_state == HomingState::kSearch)
                    {
                        // 正在搜索零位的轮子，允许转向电机按固定搜索转速慢慢转；
                        // 但 drive 仍然保持 current=0，全车不允许恢复驱动。
                        setSteerMotorTargetRPM(wheel, wheel.homing_search_rpm);
                    }
                    else if ((wheel.homing_state == HomingState::kReady) &&
                             wheel.homing_zero_valid &&
                             (wheel.steer_fault_state == SteerFaultState::kNone))
                    {
                        wheel.target_steer_motor_total_angle_rad = wheel.homing_hold_corrected_local_total_rad;
                        planned_data_.steer_angle_oa_rad[i] = mapWheelCorrectedLocalToOaTotal(wheel, wheel.homing_hold_corrected_local_total_rad);
                        last_steer_rate_cmd_rad_s_[i] = 0.0f;
                        setSteerMotorTargetTotalAngleRad(wheel, wheel.homing_hold_corrected_local_total_rad);
                    }
                    else if (wheel.homing_state == HomingState::kAlignToZero)
                    {
                        // 零偏建立后允许转向电机继续走位置闭环，把 OA 自动归到软件零点；
                        // 但在整车层面 drive 仍然保持 current=0，直到该轮重新 homing 完成。
                        // 注意：这里每拍都根据当前反馈重算“离 OA=0 最近的等效角”，
                        // 避免被上游常规模块解算写回“保持当前角”后导致归位停滞。
                        if (wheel.homing_align_command_armed)
                        {
                            wheel.target_steer_motor_total_angle_rad = computeHomingAlignTargetCorrectedLocalTotal(wheel);
                            setSteerMotorTargetTotalAngleRad(wheel, wheel.target_steer_motor_total_angle_rad);
                        }
                        else
                        {
                            setSteerMotorTargetCurrent(wheel, 0.0f);
                            if (wheel.steer_motor_h != nullptr)
                            {
                                wheel.steer_motor_h->setTargetRPM(0.0f);
                                wheel.steer_motor_h->setTargetTotalAngle(wheel.steer_motor_h->getTargetTotalAngle());
                            }
                        }
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
                    clearXParkSteerHoldState(wheel);
                    allowed_drive_target_rad_s = 0.0f;
                    wheel.target_drive_omega_rad_s = 0.0f;
                    planned_data_.drive_omega_rad_s[i] = 0.0f;
                    last_drive_omega_cmd_rad_s_[i] = 0.0f;
                    setSteerMotorTargetCurrent(wheel, 0.0f);
                    if (wheel.drive_motor_h != nullptr)
                    {
                        if (VESC_Motor *drive_vesc = dynamic_cast<VESC_Motor *>(wheel.drive_motor_h))
                        {
                            drive_vesc->setSpeedPidCurrentBias(0.0f);
                        }
                        wheel.drive_motor_h->setTargetCurrent(0.0f);
                    }
                    continue;
                }

                // 只有“全部回零完成”且“不是扭矩自由模式”时，
                // 才真正把上一阶段规划出的目标舵角和驱动角速度下发给电机闭环。
                if (isolate_this_wheel)
                {
                    clearXParkSteerHoldState(wheel);
                    allowed_drive_target_rad_s = 0.0f;
                    wheel.target_drive_omega_rad_s = 0.0f;
                    planned_data_.drive_omega_rad_s[i] = 0.0f;
                    last_drive_omega_cmd_rad_s_[i] = 0.0f;
                    wheel.target_steer_motor_total_angle_rad = wheel.corrected_steer_motor_total_angle_rad;
                    planned_data_.steer_angle_oa_rad[i] = mapWheelCorrectedLocalToOaTotal(wheel, wheel.corrected_steer_motor_total_angle_rad);
                    last_steer_rate_cmd_rad_s_[i] = 0.0f;
                    setSteerMotorTargetCurrent(wheel, 0.0f);
                    if (wheel.drive_motor_h != nullptr)
                    {
                        if (VESC_Motor *drive_vesc = dynamic_cast<VESC_Motor *>(wheel.drive_motor_h))
                        {
                            drive_vesc->setSpeedPidCurrentBias(0.0f);
                        }
                        wheel.drive_motor_h->setTargetCurrent(0.0f);
                    }
                    continue;
                }

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
                if (drive_zero_stop_active_)
                {
                    // 整车已经进入静止命令区后，不再让 drive 目标继续沿 RPM 路径慢慢收尾，而是交给后面的 zero-stop assist 收口。
                    delivered_drive_target_rad_s = 0.0f;
                }

                wheel.target_drive_omega_rad_s = delivered_drive_target_rad_s;
                planned_data_.drive_omega_rad_s[i] = delivered_drive_target_rad_s;
                last_drive_omega_cmd_rad_s_[i] = delivered_drive_target_rad_s;
                wheel.target_steer_motor_total_angle_rad = actuator_command_frame_.steer_cmd_corrected_local_total_rad[i];
                wheel.steer_target_velocity_rad_s = actuator_command_frame_.steer_rate_rad_s[i];
                planned_data_.steer_angle_oa_rad[i] = actuator_command_frame_.steer_oa_total_rad[i];
                const StrategyConfig::XParkSteerHoldConfig &xpark_hold_cfg = runtime_strategy_cfg_.xpark_steer_hold_cfg_;
                const f32 xpark_hold_entry_deg = clampValue(xpark_hold_cfg.entry_angle_deg, 0.0f, 180.0f);
                const f32 xpark_hold_exit_deg = clampValue((xpark_hold_cfg.exit_angle_deg > xpark_hold_entry_deg)
                                                               ? xpark_hold_cfg.exit_angle_deg
                                                               : (xpark_hold_entry_deg + 1.0e-3f),
                                                           xpark_hold_entry_deg,
                                                           180.0f);
                const f32 xpark_hold_settle_deg = clampValue(xpark_hold_cfg.settle_angle_deg, 0.0f, 180.0f);
                const f32 xpark_hold_settle_rate_deg_s =
                    clampValue(xpark_hold_cfg.settle_target_rate_deg_s, 0.0f, 360000.0f);
                const f32 xpark_hold_entry_rad = degToRadF32(xpark_hold_entry_deg);
                const f32 xpark_hold_exit_rad = degToRadF32(xpark_hold_exit_deg);
                const f32 xpark_hold_settle_rad = degToRadF32(xpark_hold_settle_deg);
                const f32 xpark_hold_settle_rate_rad_s = degToRadF32(xpark_hold_settle_rate_deg_s);
                const bool force_uniform_steer_drive = (input_target_data_.mode == Mode::kSteerAngleAndDriveSpeedMode);
                const bool xpark_hold_eligible =
                    xpark_hold_cfg.enable &&
                    xpark_gate_active_ &&
                    !force_uniform_steer_drive &&
                    (runtime_strategy_cfg_.idle_posture_mode == IdlePostureMode::kXPark) &&
                    all_homed &&
                    (wheel.homing_state == HomingState::kReady) &&
                    (wheel.steer_fault_state == SteerFaultState::kNone) &&
                    !single_wheel_isolation_active;

                bool command_steer_zero_current = false;
                if (xpark_hold_eligible)
                {
                    const f32 current_corrected_local_total_rad = wheel.corrected_steer_motor_total_angle_rad;
                    const f32 current_oa_total_rad = mapWheelCorrectedLocalToOaTotal(wheel, current_corrected_local_total_rad);
                    const f32 xpark_target_oa_total_rad =
                        nearestEquivalentAngleF32(current_oa_total_rad, wrapTo2PiF32(getXParkAngle(wheel)));
                    const f32 xpark_error_abs_rad =
                        fabsf(shortestAngularDistanceF32(current_oa_total_rad, xpark_target_oa_total_rad));
                    wheel.xpark_steer_hold_error_rad = xpark_error_abs_rad;

                    if ((wheel.xpark_steer_hold_phase == XParkSteerHoldPhase::kInactive) &&
                        (xpark_error_abs_rad <= xpark_hold_entry_rad))
                    {
                        wheel.xpark_steer_hold_phase = XParkSteerHoldPhase::kSettling;
                        wheel.xpark_steer_hold_locked_target_rad =
                            mapWheelOaTotalToCorrectedLocal(wheel, xpark_target_oa_total_rad);
                        wheel.xpark_steer_hold_settle_ms = 0U;
                        wheel.xpark_steer_hold_reacquire_ms = 0U;
                        if (xpark_hold_cfg.entry_reset_enable && (wheel.steer_motor_h != nullptr))
                        {
#ifdef TEST_TDD_MOTOR_DJI_H
                            if (M3508 *steer_m3508 = static_cast<M3508 *>(wheel.steer_motor_h))
                            {
                                steer_m3508->reset_speed_pid_state();
                            }
#else
                            if (M3508 *steer_m3508 = static_cast<M3508 *>(wheel.steer_motor_h))
                            {
                                steer_m3508->speed_pid_.reset();
                            }
#endif
                        }
                    }

                    if (wheel.xpark_steer_hold_phase == XParkSteerHoldPhase::kSettling)
                    {
                        wheel.target_steer_motor_total_angle_rad = wheel.xpark_steer_hold_locked_target_rad;
                        wheel.steer_target_velocity_rad_s = 0.0f;
                        planned_data_.steer_angle_oa_rad[i] =
                            mapWheelCorrectedLocalToOaTotal(wheel, wheel.xpark_steer_hold_locked_target_rad);
                        last_steer_rate_cmd_rad_s_[i] = 0.0f;

                        const f32 settle_error_abs_rad =
                            fabsf(wheel.target_steer_motor_total_angle_rad - wheel.corrected_steer_motor_total_angle_rad);
                        const f32 settle_target_rate_abs_rad_s = fabsf(wheel.steer_target_velocity_rad_s);
                        wheel.xpark_steer_hold_target_rate_rad_s = settle_target_rate_abs_rad_s;

                        const bool settle_ready =
                            (settle_error_abs_rad <= xpark_hold_settle_rad) &&
                            (settle_target_rate_abs_rad_s <= xpark_hold_settle_rate_rad_s);
                        if (settle_ready)
                        {
                            wheel.xpark_steer_hold_settle_ms =
                                (wheel.xpark_steer_hold_settle_ms > (0xFFFFFFFFU - period_ms_))
                                    ? 0xFFFFFFFFU
                                    : (wheel.xpark_steer_hold_settle_ms + period_ms_);
                        }
                        else
                        {
                            wheel.xpark_steer_hold_settle_ms = 0U;
                        }

                        if (settle_ready &&
                            (wheel.xpark_steer_hold_settle_ms >= xpark_hold_cfg.settle_hold_ms))
                        {
                            wheel.xpark_steer_hold_phase = XParkSteerHoldPhase::kLatchedZeroCurrent;
                            wheel.xpark_steer_hold_reacquire_ms = 0U;
                            command_steer_zero_current = true;
                        }
                    }
                    else if (wheel.xpark_steer_hold_phase == XParkSteerHoldPhase::kLatchedZeroCurrent)
                    {
                        command_steer_zero_current = true;
                        wheel.xpark_steer_hold_target_rate_rad_s = 0.0f;
                        const bool reacquire_ready = (xpark_error_abs_rad > xpark_hold_exit_rad);
                        if (reacquire_ready)
                        {
                            wheel.xpark_steer_hold_reacquire_ms =
                                (wheel.xpark_steer_hold_reacquire_ms > (0xFFFFFFFFU - period_ms_))
                                    ? 0xFFFFFFFFU
                                    : (wheel.xpark_steer_hold_reacquire_ms + period_ms_);
                        }
                        else
                        {
                            wheel.xpark_steer_hold_reacquire_ms = 0U;
                        }

                        if (reacquire_ready &&
                            (wheel.xpark_steer_hold_reacquire_ms >= xpark_hold_cfg.reacquire_hold_ms))
                        {
                            wheel.xpark_steer_hold_phase = XParkSteerHoldPhase::kSettling;
                            wheel.xpark_steer_hold_locked_target_rad =
                                mapWheelOaTotalToCorrectedLocal(wheel, xpark_target_oa_total_rad);
                            wheel.xpark_steer_hold_settle_ms = 0U;
                            wheel.xpark_steer_hold_reacquire_ms = 0U;
                            if (xpark_hold_cfg.entry_reset_enable && (wheel.steer_motor_h != nullptr))
                            {
#ifdef TEST_TDD_MOTOR_DJI_H
                                if (M3508 *steer_m3508 = static_cast<M3508 *>(wheel.steer_motor_h))
                                {
                                    steer_m3508->reset_speed_pid_state();
                                }
#else
                                if (M3508 *steer_m3508 = static_cast<M3508 *>(wheel.steer_motor_h))
                                {
                                    steer_m3508->speed_pid_.reset();
                                }
#endif
                            }
                            wheel.target_steer_motor_total_angle_rad = wheel.xpark_steer_hold_locked_target_rad;
                            wheel.steer_target_velocity_rad_s = 0.0f;
                            planned_data_.steer_angle_oa_rad[i] =
                                mapWheelCorrectedLocalToOaTotal(wheel, wheel.xpark_steer_hold_locked_target_rad);
                            last_steer_rate_cmd_rad_s_[i] = 0.0f;
                            wheel.xpark_steer_hold_target_rate_rad_s = 0.0f;
                            command_steer_zero_current = false;
                        }
                    }
                    else
                    {
                        wheel.xpark_steer_hold_locked_target_rad = 0.0f;
                        wheel.xpark_steer_hold_target_rate_rad_s = 0.0f;
                        wheel.xpark_steer_hold_settle_ms = 0U;
                        wheel.xpark_steer_hold_reacquire_ms = 0U;
                    }
                }
                else
                {
                    clearXParkSteerHoldState(wheel);
                }

                if (command_steer_zero_current &&
                    (wheel.xpark_steer_hold_phase == XParkSteerHoldPhase::kLatchedZeroCurrent))
                {
                    setSteerMotorTargetCurrent(wheel, 0.0f);
                }
                else
                {
                    setSteerMotorTargetTotalAngleRad(wheel, wheel.target_steer_motor_total_angle_rad);
                }
                applyDriveVirtualLoadAndCommand(wheel,
                                                i,
                                                delivered_drive_target_rad_s,
                                                single_wheel_isolation_active,
                                                single_wheel_idx,
                                                chassis_motion_blocked,
                                                allow_drive_position_loop,
                                                drive_zero_stop_active_,
                                                entering_drive_zero_stop,
                                                leaving_drive_zero_stop);
            }

            if (single_wheel_isolation_active)
            {
#if JIA_CHASSIS_ENABLE_SINGLE_WHEEL_DEBUG
                applySingleWheelIsolationFilter(debug_mode, single_wheel_idx, all_homed);
#endif
            }
        }

        void Chassis::updateCurrentData(bool all_homed)
        {
            // current_data_ 面向“外部观测与反馈读取”：
            // 它先继承本拍 planned_data_ 的结构，再根据 homing/fault 情况决定哪些车体速度反馈可以被信任。
            bool steer_fault_any_active = false;
            for (u8 i = 0; i < 4; ++i)
            {
                if (wheel_config_[i].steer_fault_state != SteerFaultState::kNone)
                {
                    steer_fault_any_active = true;
                    break;
                }
            }
            current_data_ = planned_data_;
            if (!all_homed || steer_fault_any_active)
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

            if (all_homed && !steer_fault_any_active)
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

#if JIA_CHASSIS_ENABLE_DEBUG_MIRROR
        void Chassis::refreshDebugMirror(bool all_homed)
        {
            // DebugMirror 不保存额外真相，而是把分散在 runtime/cache 里的关键结论平铺成“调试器易读视图”。
            debug_mirror_.all_homed = all_homed;
            debug_mirror_.single_wheel_target_index =
                (debug_control_.common.control_wheel_index < 4U)
                    ? debug_control_.common.control_wheel_index
                    : 0U;
            const DebugMode debug_mode = resolveDebugMode(debug_control_.common.mode_raw);
            const bool single_wheel_isolation_active =
                debug_control_.common.enable && isSingleWheelIsolatedMode(debug_mode);
            debug_mirror_.single_wheel_isolation_active =
                single_wheel_isolation_active;
            debug_mirror_.nz_stationary_m_s = getNearZeroEnterSpeedMps();
            debug_mirror_.nz_freeze_enter_m_s = getNearZeroEnterSpeedMps();
            debug_mirror_.nz_freeze_exit_m_s = getNearZeroExitSpeedMps();
            debug_mirror_.nz_xpark_enter_m_s = getXParkCommandEnterSpeedMps();
            debug_mirror_.nz_xpark_exit_m_s = getXParkCommandExitSpeedMps();
            debug_mirror_.xpark_steer_hold_enable = runtime_strategy_cfg_.xpark_steer_hold_cfg_.enable;
            debug_mirror_.xpark_steer_hold_entry_deg =
                clampValue(runtime_strategy_cfg_.xpark_steer_hold_cfg_.entry_angle_deg, 0.0f, 180.0f);
            debug_mirror_.xpark_steer_hold_exit_deg =
                clampValue((runtime_strategy_cfg_.xpark_steer_hold_cfg_.exit_angle_deg > debug_mirror_.xpark_steer_hold_entry_deg)
                               ? runtime_strategy_cfg_.xpark_steer_hold_cfg_.exit_angle_deg
                               : (debug_mirror_.xpark_steer_hold_entry_deg + 1.0e-3f),
                           debug_mirror_.xpark_steer_hold_entry_deg,
                           180.0f);
            debug_mirror_.xpark_steer_hold_settle_deg =
                clampValue(runtime_strategy_cfg_.xpark_steer_hold_cfg_.settle_angle_deg, 0.0f, 180.0f);
            debug_mirror_.xpark_steer_hold_settle_target_rate_deg_s =
                clampValue(runtime_strategy_cfg_.xpark_steer_hold_cfg_.settle_target_rate_deg_s, 0.0f, 360000.0f);
            debug_mirror_.xpark_steer_hold_settle_hold_ms =
                static_cast<f32>(runtime_strategy_cfg_.xpark_steer_hold_cfg_.settle_hold_ms);
            debug_mirror_.xpark_steer_hold_reacquire_hold_ms =
                static_cast<f32>(runtime_strategy_cfg_.xpark_steer_hold_cfg_.reacquire_hold_ms);
            debug_mirror_.xpark_steer_hold_entry_reset_enable =
                runtime_strategy_cfg_.xpark_steer_hold_cfg_.entry_reset_enable;
            debug_mirror_.lim_drive_omega = runtime_strategy_cfg_.enable_drive_omega_limit_;
            debug_mirror_.lim_drive_alpha = runtime_strategy_cfg_.enable_drive_alpha_limit_;
            debug_mirror_.lim_steer_rate = runtime_strategy_cfg_.enable_steer_rate_limit_;
            debug_mirror_.lim_steer_alpha = runtime_strategy_cfg_.enable_steer_alpha_limit_;
            debug_mirror_.high_speed_drive_suppression_scale = high_speed_drive_suppression_scale_;
            debug_mirror_.high_speed_dir_err_deg = high_speed_dir_err_deg_;
            debug_mirror_.high_speed_eta_max_s = high_speed_eta_max_s_;
            debug_mirror_.high_speed_drive_suppression_active = high_speed_drive_suppression_active_;
            debug_mirror_.low_speed_drive_suppression_bypassed_by_residual_speed = low_speed_drive_suppression_bypassed_by_residual_speed_;
            debug_mirror_.max_residual_speed_m_s = max_residual_speed_m_s_;
            debug_mirror_.reverse_intent_active = reverse_intent_active_;
            debug_mirror_.reverse_intent_dir_err_deg = reverse_intent_dir_err_deg_;
            debug_mirror_.steer_fault_any_active = steer_fault_any_active_;
            for (u8 i = 0; i < 4; ++i)
            {
                debug_mirror_.single_wheel_non_target_zeroed[i] =
                    debug_mirror_.single_wheel_isolation_active && (i != debug_mirror_.single_wheel_target_index);
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
                debug_mirror_.steer_fault_active[i] = (wheel.steer_fault_state != SteerFaultState::kNone);
                debug_mirror_.steer_fault_recovering[i] = (wheel.steer_fault_state == SteerFaultState::kRecovering);
                debug_mirror_.steer_fault_control_intent[i] = wheel.steer_fault_control_intent;
                debug_mirror_.steer_fault_xpark_stationary_hold[i] = wheel.steer_fault_xpark_stationary_hold;
                debug_mirror_.steer_fault_freeze_candidate[i] = wheel.steer_fault_freeze_candidate;
                debug_mirror_.steer_feedback_current_mA[i] = wheel.steer_feedback_current_mA;
                debug_mirror_.steer_feedback_current_delta_mA[i] = wheel.steer_feedback_current_delta_mA;
                debug_mirror_.steer_feedback_angle_delta_rad[i] = wheel.steer_feedback_angle_delta_rad;
                debug_mirror_.xpark_steer_hold_phase[i] = static_cast<u8>(wheel.xpark_steer_hold_phase);
                debug_mirror_.xpark_steer_hold_locked[i] =
                    (wheel.xpark_steer_hold_phase != XParkSteerHoldPhase::kInactive);
                debug_mirror_.xpark_steer_hold_error_deg[i] = radToDegF32(wheel.xpark_steer_hold_error_rad);
                debug_mirror_.xpark_steer_hold_target_rate_deg_s[i] =
                    radToDegF32(wheel.xpark_steer_hold_target_rate_rad_s);
                debug_mirror_.xpark_steer_hold_settle_ms[i] = static_cast<f32>(wheel.xpark_steer_hold_settle_ms);
                debug_mirror_.xpark_steer_hold_reacquire_ms[i] = static_cast<f32>(wheel.xpark_steer_hold_reacquire_ms);
                debug_mirror_.steer_fault_steer_error_deg[i] = radToDegF32(wheel.steer_fault_steer_error_rad);
                debug_mirror_.steer_feedback_current_freeze_ms[i] = static_cast<f32>(wheel.steer_feedback_freeze_ms);
                debug_mirror_.steer_feedback_recovery_toggle_count[i] = static_cast<f32>(wheel.steer_feedback_recovery_toggle_count);
                debug_mirror_.steer_fault_latched_count[i] = static_cast<f32>(wheel.steer_fault_latched_count);
            }
        }
#endif

#if JIA_CHASSIS_ENABLE_PID_TUNE_CACHE
        void Chassis::syncDebugSteerPidTuneFromRuntimeOnEnableEdge()
        {
            const bool enable_now = debug_control_.common.enable;
            if (!enable_now)
            {
                debug_pid_tune_.synced_on_enable_edge = false;
                debug_enable_last_cycle_ = false;
                return;
            }

            if (!debug_enable_last_cycle_)
            {
                // 调试使能上升沿：先把当前电机运行态回读到缓存，形成“先读后改”的调参基线。
                syncDebugSteerPidTuneFromRuntime();
                debug_pid_tune_.synced_on_enable_edge = true;
            }
            debug_enable_last_cycle_ = true;
        }

        void Chassis::syncDebugSteerPidTuneFromRuntime()
        {
            const bool steer_speed_dirty = (debug_pid_tune_.steer_speed_pid_applied_stamp != debug_pid_tune_.steer_speed_pid_apply_stamp);
            const bool steer_angle_dirty = (debug_pid_tune_.steer_angle_pid_applied_stamp != debug_pid_tune_.steer_angle_pid_apply_stamp);
            if (!steer_speed_dirty && !steer_angle_dirty)
            {
                for (u8 i = 0; i < 4; ++i)
                {
                    WheelConfig &wheel = wheel_config_[i];
                    M3508 *steer_m3508 = static_cast<M3508 *>(wheel.steer_motor_h);
                    if (steer_m3508 == nullptr)
                    {
                        continue;
                    }

                    debug_pid_tune_.steer_speed_pid_cfg = steer_m3508->get_speed_pid_params();
                    debug_pid_tune_.steer_angle_pid_cfg = steer_m3508->get_angle_pid_params();
                    debug_pid_tune_.steer_speed_pid_td_ratio = steer_m3508->get_speed_pid_td_ratio();
                    debug_pid_tune_.steer_angle_pid_i_separa = steer_m3508->get_angle_pid_i_separa_threshold();
                    break;
                }
            }

            const bool drive_dirty = (debug_pid_tune_.drive_speed_pid_applied_stamp != debug_pid_tune_.drive_speed_pid_apply_stamp);
            if (drive_dirty)
            {
                return;
            }

            for (u8 i = 0; i < 4; ++i)
            {
                VESC_Motor *drive_motor = wheel_config_[i].drive_motor_h;
                if (drive_motor == nullptr)
                {
                    continue;
                }

                debug_pid_tune_.drive_speed_pid_cfg = drive_motor->get_speed_pid_params();
                debug_pid_tune_.drive_speed_pid_td_ratio = drive_motor->get_speed_pid_td_ratio();
                // dirty cache 不存在时，回读共享 drive 速度环的微分先行状态。
                debug_pid_tune_.drive_speed_pid_derivative_first = getDriveSpeedPidDerivativeFirst(drive_motor);
                break;
            }
        }

        void Chassis::applyDebugSteerPidRuntimeTuning()
        {
            const bool steer_speed_dirty = (debug_pid_tune_.steer_speed_pid_applied_stamp != debug_pid_tune_.steer_speed_pid_apply_stamp);
            const bool steer_angle_dirty = (debug_pid_tune_.steer_angle_pid_applied_stamp != debug_pid_tune_.steer_angle_pid_apply_stamp);
            if (steer_speed_dirty || steer_angle_dirty)
            {
                bool applied_any = false;
                for (u8 i = 0; i < 4; ++i)
                {
                    WheelConfig &wheel = wheel_config_[i];
                    M3508 *steer_m3508 = static_cast<M3508 *>(wheel.steer_motor_h);
                    if (steer_m3508 == nullptr)
                    {
                        continue;
                    }

                    steer_m3508->pid_init(debug_pid_tune_.steer_speed_pid_cfg, debug_pid_tune_.steer_speed_pid_td_ratio,
                                          debug_pid_tune_.steer_angle_pid_cfg, debug_pid_tune_.steer_angle_pid_i_separa);
                    applied_any = true;
                }

                if (applied_any)
                {
                    debug_pid_tune_.steer_speed_pid_applied_stamp = debug_pid_tune_.steer_speed_pid_apply_stamp;
                    debug_pid_tune_.steer_angle_pid_applied_stamp = debug_pid_tune_.steer_angle_pid_apply_stamp;
                }
            }

            if (debug_pid_tune_.drive_speed_pid_applied_stamp != debug_pid_tune_.drive_speed_pid_apply_stamp)
            {
                bool applied_any = false;
                for (u8 i = 0; i < 4; ++i)
                {
                    VESC_Motor *drive_motor = wheel_config_[i].drive_motor_h;
                    if (drive_motor == nullptr)
                    {
                        continue;
                    }

                    drive_motor->pid_init(debug_pid_tune_.drive_speed_pid_cfg,
                                          debug_pid_tune_.drive_speed_pid_td_ratio);
                    // 参数仍从 pid_init 下发，但微分先行改走独立 setter，避免接口再次扩散。
                    setDriveSpeedPidDerivativeFirst(drive_motor, debug_pid_tune_.drive_speed_pid_derivative_first);
                    applied_any = true;
                }

                if (applied_any)
                {
                    debug_pid_tune_.drive_speed_pid_applied_stamp = debug_pid_tune_.drive_speed_pid_apply_stamp;
                }
            }
        }
#endif

#if JIA_CHASSIS_ENABLE_DEBUG_OUTPUT
        void Chassis::emitDebugUart8Log(bool all_homed)
        {
            if (!debug_output_.output_enable || sanitizeDebugOutputFamily(debug_output_.output_family_raw) != DebugOutputFamily::kText)
            {
                return;
            }

            const u32 period_ms = (debug_output_.text.period_ms > 0U) ? debug_output_.text.period_ms : 500U;
            if ((time_ms_ - debug_output_runtime_.text.last_ms) < period_ms)
            {
                return;
            }

            if (HAL_UART_GetState(&huart8) != HAL_UART_STATE_READY)
            {
                return;
            }

            debug_output_runtime_.text.last_ms = time_ms_;
            if (debug_output_.text.log_level == 0U)
            {
                debug_uart_.printf_DMA((char *)"FS t=%lu home=%u mode=%u dbg=%u hs=%u/%u/%u/%u oa0=%.1f->%.1f rpm0=%.1f->%.1f\r\n",
                                       (u32)time_ms_,
                                       all_homed ? 1U : 0U,
                                       (u32)input_target_data_.mode,
                                       debug_control_.common.enable ? 1U : 0U,
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

            const u8 wheel_idx = (debug_control_.common.observe_wheel_index < 4U) ? debug_control_.common.observe_wheel_index : 0U;
            if (debug_output_runtime_.text.log_phase == 0U)
            {
                debug_uart_.printf_DMA((char *)"FS t=%lu home=%u mode=%u dbg=%u hs=%u/%u/%u/%u oa0=%.1f->%.1f rpm0=%.1f->%.1f vec=%.2f de=%.1f eta=%.3f va=%u\r\n",
                                       (u32)time_ms_,
                                       all_homed ? 1U : 0U,
                                       (u32)input_target_data_.mode,
                                       debug_control_.common.enable ? 1U : 0U,
                                       (u32)debug_mirror_.homing_state[0],
                                       (u32)debug_mirror_.homing_state[1],
                                       (u32)debug_mirror_.homing_state[2],
                                       (u32)debug_mirror_.homing_state[3],
                                       debug_mirror_.current_oa_deg[0],
                                       debug_mirror_.target_oa_deg[0],
                                       debug_mirror_.current_drive_rpm[0],
                                       debug_mirror_.target_drive_rpm[0],
                                       debug_mirror_.high_speed_drive_suppression_scale,
                                       debug_mirror_.high_speed_dir_err_deg,
                                       debug_mirror_.high_speed_eta_max_s,
                                       debug_mirror_.high_speed_drive_suppression_active ? 1U : 0U);
            }
            else if (debug_output_runtime_.text.log_phase == 1U)
            {
                debug_uart_.printf_DMA((char *)"FSW i=%u hs=%u oa=%.1f->%.1f rpm=%.1f->%.1f gate=%.2f flip=%u sensor=%u edge=%u\r\n",
                                       (u32)wheel_idx,
                                       (u32)debug_mirror_.homing_state[wheel_idx],
                                       debug_mirror_.current_oa_deg[wheel_idx],
                                       debug_mirror_.target_oa_deg[wheel_idx],
                                       debug_mirror_.current_drive_rpm[wheel_idx],
                                       debug_mirror_.target_drive_rpm[wheel_idx],
                                       low_speed_drive_suppression_scale_[wheel_idx],
                                       wheel_config_[wheel_idx].flipped_drive_direction ? 1U : 0U,
                                       debug_mirror_.homing_sensor_active[wheel_idx] ? 1U : 0U,
                                       debug_mirror_.homing_last_edge_is_falling[wheel_idx] ? 1U : 0U);
            }
            else
            {
                f32 align_err_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                for (u8 i = 0; i < 4; ++i)
                {
                    const WheelConfig &wheel = wheel_config_[i];
                    const f32 current_oa_total_rad = mapWheelCorrectedLocalToOaTotal(wheel, wheel.corrected_steer_motor_total_angle_rad);
                    const f32 align_target_oa_total_rad = nearestEquivalentAngleF32(current_oa_total_rad, 0.0f);
                    align_err_deg[i] = radToDegF32(shortestAngularDistanceF32(current_oa_total_rad, align_target_oa_total_rad));
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

            debug_output_runtime_.text.log_phase = (u8)((debug_output_runtime_.text.log_phase + 1U) % 3U);
        }

        void Chassis::emitUart8VofaJustFloatPidTrace()
        {
            if (!debug_output_.output_enable ||
                sanitizeDebugOutputFamily(debug_output_.output_family_raw) != DebugOutputFamily::kJustFloat ||
                sanitizeJustFloatProfile(debug_output_.justfloat.profile_raw) != JustFloatProfile::kOverview)
            {
                return;
            }

            const u32 period_ms = (debug_output_.justfloat.overview.period_ms > 0U) ? debug_output_.justfloat.overview.period_ms : 10U;
            if ((time_ms_ - debug_output_runtime_.justfloat.overview.last_ms) < period_ms)
            {
                return;
            }

            if (HAL_UART_GetState(&huart8) != HAL_UART_STATE_READY)
            {
                return;
            }

            debug_output_runtime_.justfloat.overview.last_ms = time_ms_;
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
            if (!debug_output_.output_enable ||
                sanitizeDebugOutputFamily(debug_output_.output_family_raw) != DebugOutputFamily::kJustFloat ||
                sanitizeJustFloatProfile(debug_output_.justfloat.profile_raw) != JustFloatProfile::kSingleWheelTrace ||
                sanitizeSingleWheelTracePayloadKind(debug_output_.justfloat.single_wheel_payload_raw) != SingleWheelTracePayloadKind::kSteerOnly)
            {
                return;
            }

            const u32 period_ms = (debug_output_.justfloat.single_wheel.period_ms > 0U) ? debug_output_.justfloat.single_wheel.period_ms : 1U;
            if ((time_ms_ - debug_output_runtime_.justfloat.single_wheel.last_ms) < period_ms)
            {
                return;
            }

            if (HAL_UART_GetState(&huart8) != HAL_UART_STATE_READY)
            {
                return;
            }

            debug_output_runtime_.justfloat.single_wheel.last_ms = time_ms_;
            const u8 observe_wheel_idx = (debug_control_.common.observe_wheel_index < 4U) ? debug_control_.common.observe_wheel_index : 0U;
            const WheelConfig &wheel = wheel_config_[observe_wheel_idx];
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

        void Chassis::emitUart8VofaSingleWheelDriveTrace()
        {
            if (!debug_output_.output_enable ||
                sanitizeDebugOutputFamily(debug_output_.output_family_raw) != DebugOutputFamily::kJustFloat ||
                sanitizeJustFloatProfile(debug_output_.justfloat.profile_raw) != JustFloatProfile::kSingleWheelTrace ||
                sanitizeSingleWheelTracePayloadKind(debug_output_.justfloat.single_wheel_payload_raw) != SingleWheelTracePayloadKind::kDriveOnly)
            {
                return;
            }

            const u32 period_ms = (debug_output_.justfloat.single_wheel.period_ms > 0U) ? debug_output_.justfloat.single_wheel.period_ms : 1U;
            if ((time_ms_ - debug_output_runtime_.justfloat.single_wheel.last_ms) < period_ms)
            {
                return;
            }

            if (HAL_UART_GetState(&huart8) != HAL_UART_STATE_READY)
            {
                return;
            }

            debug_output_runtime_.justfloat.single_wheel.last_ms = time_ms_;
            const u8 observe_wheel_idx = (debug_control_.common.observe_wheel_index < 4U) ? debug_control_.common.observe_wheel_index : 0U;
            const WheelConfig &wheel = wheel_config_[observe_wheel_idx];
            const VESC_Motor *drive_motor = wheel.drive_motor_h;
            if (drive_motor == nullptr)
            {
                return;
            }

            const f32 target_multi_turn_deg = drive_motor->getTargetTotalAngle();
            f32 target_single_turn_deg = fmodf(target_multi_turn_deg, 360.0f);
            if (target_single_turn_deg < 0.0f)
            {
                target_single_turn_deg += 360.0f;
            }

            float payload[9] = {0.0f};
            payload[0] = (f32)time_ms_ * 0.001f;
            payload[1] = drive_motor->getTargetCurrent(); // mA
            payload[2] = drive_motor->getCurrent();       // mA
            payload[3] = drive_motor->getTargetRPM();
            payload[4] = drive_motor->getRPM();
            payload[5] = target_single_turn_deg;
            payload[6] = drive_motor->getAngle();
            payload[7] = target_multi_turn_deg;
            payload[8] = drive_motor->getTotalAngle();
            debug_uart_.printf_DMA_JustFloat(payload, 9);
        }

        void Chassis::emitUart8VofaDualMotor1kHzTrace()
        {
            if (!debug_output_.output_enable ||
                sanitizeDebugOutputFamily(debug_output_.output_family_raw) != DebugOutputFamily::kJustFloat ||
                sanitizeJustFloatProfile(debug_output_.justfloat.profile_raw) != JustFloatProfile::kSingleWheelTrace ||
                sanitizeSingleWheelTracePayloadKind(debug_output_.justfloat.single_wheel_payload_raw) != SingleWheelTracePayloadKind::kSteerAndDrive)
            {
                return;
            }

            const u32 period_ms = (debug_output_.justfloat.single_wheel.period_ms > 0U) ? debug_output_.justfloat.single_wheel.period_ms : 2U;
            if ((time_ms_ - debug_output_runtime_.justfloat.single_wheel.last_ms) < period_ms)
            {
                return;
            }

            if (HAL_UART_GetState(&huart8) != HAL_UART_STATE_READY)
            {
                return;
            }

            const u8 observe_wheel_idx = (debug_control_.common.observe_wheel_index < 4U) ? debug_control_.common.observe_wheel_index : 0U;
            const WheelConfig &wheel = wheel_config_[observe_wheel_idx];
            const Motor_Base *steer_motor = wheel.steer_motor_h;
            const VESC_Motor *drive_motor = wheel.drive_motor_h;
            if (steer_motor == nullptr || drive_motor == nullptr)
            {
                return;
            }

            debug_output_runtime_.justfloat.single_wheel.last_ms = time_ms_;

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

        void Chassis::emitUart8VofaYawPidTrace()
        {
            if (!debug_output_.output_enable ||
                sanitizeDebugOutputFamily(debug_output_.output_family_raw) != DebugOutputFamily::kJustFloat ||
                sanitizeJustFloatProfile(debug_output_.justfloat.profile_raw) != JustFloatProfile::kYawPid)
            {
                return;
            }

            const u32 period_ms = (debug_output_.justfloat.yaw_pid.period_ms > 0U) ? debug_output_.justfloat.yaw_pid.period_ms : 10U;
            if ((time_ms_ - debug_output_runtime_.justfloat.yaw_pid.last_ms) < period_ms)
            {
                return;
            }

            if (HAL_UART_GetState(&huart8) != HAL_UART_STATE_READY)
            {
                return;
            }

            debug_output_runtime_.justfloat.yaw_pid.last_ms = time_ms_;

            float payload[15] = {0.0f};
            payload[0] = static_cast<f32>(time_ms_) * 0.001f;
            payload[1] = yaw_pid_trace_.mode_tag;
            payload[2] = yaw_pid_trace_.target_yaw_rad;
            payload[3] = yaw_pid_trace_.feedback_yaw_rad;
            payload[4] = yaw_pid_trace_.error_deg;
            payload[5] = yaw_pid_trace_.manual_omega_in_rad_s;
            payload[6] = yaw_pid_trace_.pid_output_omega_rad_s;
            payload[7] = yaw_pid_trace_.final_omega_cmd_rad_s;
            payload[8] = yaw_pid_trace_.feedback_yaw_rate_rad_s;
            payload[9] = yaw_pid_trace_.shift_remaining_ms;
            payload[10] = yaw_pid_trace_.pid_compute_fired;
            payload[11] = debug_mirror_.steer_fault_any_active ? 1.0f : 0.0f;
            payload[12] = debug_mirror_.all_homed ? 1.0f : 0.0f;
            payload[13] = debug_mirror_.high_speed_drive_suppression_active ? 1.0f : 0.0f;
            payload[14] = debug_mirror_.reverse_intent_active ? 1.0f : 0.0f;
            debug_uart_.printf_DMA_JustFloat(payload, 15);
        }

        void Chassis::emitUart8VofaDrivePidLoadTrace()
        {
            if (!debug_output_.output_enable ||
                sanitizeDebugOutputFamily(debug_output_.output_family_raw) != DebugOutputFamily::kJustFloat ||
                sanitizeJustFloatProfile(debug_output_.justfloat.profile_raw) != JustFloatProfile::kDrivePidLoadTune)
            {
                return;
            }

            const u32 period_ms = (debug_output_.justfloat.drive_pid_load.period_ms > 0U) ? debug_output_.justfloat.drive_pid_load.period_ms : 1U;
            if ((time_ms_ - debug_output_runtime_.justfloat.drive_pid_load.last_ms) < period_ms)
            {
                return;
            }

            if (HAL_UART_GetState(&huart8) != HAL_UART_STATE_READY)
            {
                return;
            }

            debug_output_runtime_.justfloat.drive_pid_load.last_ms = time_ms_;

            float payload[16] = {0.0f};
            payload[0] = static_cast<f32>(time_ms_) * 0.001f;
            payload[1] = debug_drive_load_trace_.observe_wheel_idx;
            payload[2] = debug_drive_load_trace_.target_rpm;
            payload[3] = debug_drive_load_trace_.feedback_rpm;
            payload[4] = debug_drive_load_trace_.total_current_cmd_mA;
            payload[5] = debug_drive_load_trace_.pid_current_mA;
            payload[6] = debug_drive_load_trace_.load_bias_current_mA;
            payload[7] = debug_drive_load_trace_.j_term_mA;
            payload[8] = debug_drive_load_trace_.b_term_mA;
            payload[9] = debug_drive_load_trace_.tc_term_mA;
            payload[10] = debug_drive_load_trace_.omega_rad_s;
            payload[11] = debug_drive_load_trace_.alpha_est_rad_s2;
            payload[12] = debug_drive_load_trace_.step_phase;
            payload[13] = debug_drive_load_trace_.virtual_load_enable;
            payload[14] = debug_drive_load_trace_.stepgen_enable;
            payload[15] = 0.0f;
            const u8 observe_wheel_idx = (debug_control_.common.observe_wheel_index < 4U) ? debug_control_.common.observe_wheel_index : 0U;
            if (wheel_config_[observe_wheel_idx].drive_motor_h != nullptr)
            {
                payload[15] = wheel_config_[observe_wheel_idx].drive_motor_h->getCurrent();
            }
            debug_uart_.printf_DMA_JustFloat(payload, 16);
        }

        void Chassis::emitUart8VofaDriveZeroStopBrakeTrace()
        {
            if (!debug_output_.output_enable ||
                sanitizeDebugOutputFamily(debug_output_.output_family_raw) != DebugOutputFamily::kJustFloat ||
                sanitizeJustFloatProfile(debug_output_.justfloat.profile_raw) != JustFloatProfile::kDriveZeroStopBrakeTrace)
            {
                return;
            }

            const u32 period_ms = (debug_output_.justfloat.drive_zero_stop_brake.period_ms > 0U) ? debug_output_.justfloat.drive_zero_stop_brake.period_ms : 1U;
            if ((time_ms_ - debug_output_runtime_.justfloat.drive_zero_stop_brake.last_ms) < period_ms)
            {
                return;
            }

            if (HAL_UART_GetState(&huart8) != HAL_UART_STATE_READY)
            {
                return;
            }

            debug_output_runtime_.justfloat.drive_zero_stop_brake.last_ms = time_ms_;

            const u8 observe_wheel_idx = (debug_control_.common.observe_wheel_index < 4U) ? debug_control_.common.observe_wheel_index : 0U;
            const VESC_Motor *drive_motor = wheel_config_[observe_wheel_idx].drive_motor_h;
            const f32 wheel_radius_m = fabsf(runtime_strategy_cfg_.wheel_radius_m_);
            const f32 residual_speed_m_s = fabsf(wheel_config_[observe_wheel_idx].corrected_drive_omega_rad_s) * wheel_radius_m;
            const f32 target_command_speed_m_s = computeMaxCommandWheelSpeedMps(target_data_);

            float payload[12] = {0.0f};
            payload[0] = static_cast<f32>(time_ms_) * 0.001f;
            payload[1] = static_cast<f32>(observe_wheel_idx);
            // 速度口径继续复用现有 drive load trace，避免另起一套调试语义。
            payload[2] = debug_drive_load_trace_.target_rpm;
            payload[2] = debug_drive_load_trace_.target_rpm;
            payload[3] = debug_drive_load_trace_.feedback_rpm;
            payload[4] = drive_zero_stop_brake_active_[observe_wheel_idx] ? 1.0f : 0.0f;
            payload[5] = (drive_motor != nullptr) ? drive_motor->getTargetBrakeCurrent() : 0.0f;
            payload[6] = ((drive_motor != nullptr) && drive_motor->isBrakeCommandActive()) ? 1.0f : 0.0f;
            payload[7] = (drive_motor != nullptr) ? drive_motor->getCurrent() : 0.0f;
            payload[8] = drive_zero_stop_active_ ? 1.0f : 0.0f;
            payload[9] = residual_speed_m_s;
            payload[10] = target_command_speed_m_s;
            payload[11] = target_data_.omega_z;
            debug_uart_.printf_DMA_JustFloat(payload, 12);
        }

#if JIA_CHASSIS_ENABLE_BINARY_TELEMETRY
        void Chassis::emitUart8SwerveTelemetryV2(bool all_homed)
        {
            if (!debug_output_.output_enable || sanitizeDebugOutputFamily(debug_output_.output_family_raw) != DebugOutputFamily::kBinary)
            {
                return;
            }

            const u8 divider = (debug_output_.binary.telemetry.sample_divider == 0U) ? 1U : debug_output_.binary.telemetry.sample_divider;
            debug_output_runtime_.binary.telemetry.cycle_counter = static_cast<u8>(debug_output_runtime_.binary.telemetry.cycle_counter + 1U);
            if (debug_output_runtime_.binary.telemetry.cycle_counter < divider)
            {
                return;
            }
            debug_output_runtime_.binary.telemetry.cycle_counter = 0U;

            const u32 period_ms = (debug_output_.binary.telemetry.period_ms > 0U) ? debug_output_.binary.telemetry.period_ms : 8U;
            if ((time_ms_ - debug_output_runtime_.binary.telemetry.last_ms) < period_ms)
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
                                             ((debug_output_.binary.telemetry.profile_id & 0x0FU) << 4U));
            frame[cursor++] = flags;
            packU16LE(&frame[cursor], debug_output_runtime_.binary.telemetry.seq);
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

            debug_output_runtime_.binary.telemetry.last_ms = time_ms_;
            debug_output_runtime_.binary.telemetry.seq = static_cast<u16>(debug_output_runtime_.binary.telemetry.seq + 1U);
        }
#endif

        void Chassis::emitDebugOutputByMode(bool all_homed)
        {
            if (!debug_output_.output_enable)
            {
                return;
            }
            switch (sanitizeDebugOutputFamily(debug_output_.output_family_raw))
            {
            case DebugOutputFamily::kText:
                emitDebugUart8Log(all_homed);
                break;
            case DebugOutputFamily::kJustFloat:
                switch (sanitizeJustFloatProfile(debug_output_.justfloat.profile_raw))
                {
                case JustFloatProfile::kOverview:
                    emitUart8VofaJustFloatPidTrace();
                    break;
                case JustFloatProfile::kSingleWheelTrace:
                {
                    const SingleWheelTracePayloadKind payload_kind =
                        sanitizeSingleWheelTracePayloadKind(debug_output_.justfloat.single_wheel_payload_raw);
                    if (payload_kind == SingleWheelTracePayloadKind::kSteerAndDrive)
                    {
                        emitUart8VofaDualMotor1kHzTrace();
                    }
                    else if (payload_kind == SingleWheelTracePayloadKind::kDriveOnly)
                    {
                        emitUart8VofaSingleWheelDriveTrace();
                    }
                    else
                    {
                        emitUart8VofaPid1kHzTrace();
                    }
                    break;
                }
                case JustFloatProfile::kDrivePidLoadTune:
                    emitUart8VofaDrivePidLoadTrace();
                    break;
                case JustFloatProfile::kDriveZeroStopBrakeTrace:
                    emitUart8VofaDriveZeroStopBrakeTrace();
                    break;
                case JustFloatProfile::kYawPid:
                default:
                    emitUart8VofaYawPidTrace();
                    break;
                }
                break;
            case DebugOutputFamily::kBinary:
                emitUart8SwerveTelemetryV2(all_homed);
                break;
            case DebugOutputFamily::kOff:
            default:
                break;
            }
        }
#endif

#if JIA_CHASSIS_ENABLE_DEBUG_OVERRIDE
        bool Chassis::applyDebugModuleOverride(bool all_homed)
        {
            if (!debug_control_.common.enable)
            {
                return false;
            }

            const DebugModuleOverrideRoute route = classifyDebugModuleOverrideRoute(debug_control_.common.mode_raw);
            if (route == DebugModuleOverrideRoute::kNone)
            {
                return false;
            }

            const u8 wheel_idx = (debug_control_.common.control_wheel_index < 4U) ? debug_control_.common.control_wheel_index : 0U;
            const DebugMode mode = resolveDebugMode(debug_control_.common.mode_raw);
            resetDebugModuleOverrideTargets(wheel_idx, false);

            if (route == DebugModuleOverrideRoute::kAlignForward)
            {
                applyAlignForwardDebugOverride();
            }
            else if (route == DebugModuleOverrideRoute::kHomingObserve)
            {
                applyHomingObserveDebugOverride();
            }
            else if (route == DebugModuleOverrideRoute::kSingleWheelIsolated)
            {
                computeSingleWheelIsolatedCommandsMode30(wheel_idx, all_homed);
                applySingleWheelIsolationFilter(mode, wheel_idx, all_homed);
            }

            finalizeDebugModuleOverride(all_homed, route);
            return true;
        }
#endif

        bool Chassis::solveLinear3x3(f32 matrix[3][4], f32 &x0, f32 &x1, f32 &x2) const
        {
            // 这里用的是带主元选取的高斯消元，目标是稳定求解 3x3 线性方程组。
            // 输入是增广矩阵，输出是三项未知量；失败通常意味着矩阵接近奇异。
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
            // 从四个模块的当前朝向和驱动反馈反推底盘速度。
            // 这里不是直接解单个方程，而是把每轮的纵向约束与横向“无侧滑”约束累积成最小二乘正规方程，
            // 最后统一求出 3 个底盘自由度 [vx, vy, omega_z]。
            f32 normal[3][3] = {};
            f32 rhs[3] = {};

            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const f32 steer_angle_oa_rad = mapWheelCorrectedLocalToOaTotal(wheel, wheel.corrected_steer_motor_total_angle_rad);
                const f32 cos_theta = cosRadF32(steer_angle_oa_rad);
                const f32 sin_theta = sinRadF32(steer_angle_oa_rad);
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
#if JIA_CHASSIS_ENABLE_TASK_PERF_STAT
                const u64 loop_start_us = RtosTimeStampUs64::getTimeUs();
                u64 plan_us = 0ULL;     // [RO] 本周期规划阶段耗时（微秒）
                u64 feedback_us = 0ULL; // [RO] 本周期反馈刷新阶段耗时（微秒）
                u64 homing_us = 0ULL;   // [RO] 本周期回零状态机阶段耗时（微秒）
                u64 apply_us = 0ULL;    // [RO] 本周期命令生成与下发阶段耗时（微秒）
                u64 debug_us = 0ULL;    // [RO] 本周期调试镜像与输出阶段耗时（微秒）
                u64 stage_start_us = loop_start_us;
#endif

                // 每个 1ms 控制周期都固定经过这六个阶段：
                // 1. 读取 IMU / 遥控器输入
                // 2. 折叠模式、坐标系和调试接管，得到 target_data_
                // 3. 规划车体级目标 planned_data_
                // 4. 刷新轮反馈，并推进 steer fault / homing 状态机
                // 5. 求解并下发模块命令
                // 6. 刷新观测镜像 / trace，然后等待下一拍
                input_hwt_rot_z_ = hwt->get_yaw_rad();
                input_hwt_omega_z_ = hwt->get_yaw_speed_rad();

                // 常态同步手柄缓存：即使 debug_control_.common.enable 关闭，也保持 airjoy_data_ 实时更新。
                // 便于通过调试器直接观察摇杆输入；不改变任何控制模式接管逻辑。
                CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);

#if JIA_CHASSIS_ENABLE_DEBUG_OVERRIDE
                // RUNTIME_MIN 不走调试接管，避免调试面板字段把正常底盘输入链路拉进固件。
                isDebugMode();
#endif
                setModeFlag();
                resolvePlannerTargetData();

                // 运行时允许调试器直接改基准阈值/限幅开关，这里每周期刷新限幅镜像，近零阈值由 helper 直接读取。
                refreshActuatorLimitState();

                updatePlannedMotionData();
#if JIA_CHASSIS_ENABLE_TASK_PERF_STAT
                plan_us = RtosTimeStampUs64::getTimeUs() - stage_start_us;
#endif

#if JIA_CHASSIS_ENABLE_TASK_PERF_STAT
                stage_start_us = RtosTimeStampUs64::getTimeUs();
#endif
                updateWheelFeedback();
#if JIA_CHASSIS_ENABLE_PID_TUNE_CACHE
                applyDebugSteerPidRuntimeTuning();
#endif
#if JIA_CHASSIS_ENABLE_TASK_PERF_STAT
                feedback_us = RtosTimeStampUs64::getTimeUs() - stage_start_us;
#endif

#if JIA_CHASSIS_ENABLE_TASK_PERF_STAT
                stage_start_us = RtosTimeStampUs64::getTimeUs();
#endif
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
                    // Homing 优先级高于外部 zero-current 锁存；
                    // 只要还没回零完成，就立刻丢掉这次 zero-current 请求，避免把校准流程自己打断。
                    input_target_data_.zero_current_all = false;
                }
                homing_start_request_ = false;
#if JIA_CHASSIS_ENABLE_TASK_PERF_STAT
                homing_us = RtosTimeStampUs64::getTimeUs() - stage_start_us;
#endif

#if JIA_CHASSIS_ENABLE_DEBUG_OVERRIDE
                if (applyDebugModuleOverride(all_homed))
                {
#if JIA_CHASSIS_ENABLE_TASK_PERF_STAT
                    updateTaskPerfBreakdown(plan_us, feedback_us, homing_us, 0ULL, 0ULL);
                    updateTaskPerfStat(loop_start_us, RtosTimeStampUs64::getTimeUs());
#endif
                    vTaskDelayUntil(&time_ms_, period_ms_);
                    continue;
                }
#endif

                // 回零和正常控制共用同一套模块求解流程；
                // 差别不在 compute，而在 apply 时依据 all_homed / steer fault 决定哪些目标允许真正落到电机。
#if JIA_CHASSIS_ENABLE_TASK_PERF_STAT
                stage_start_us = RtosTimeStampUs64::getTimeUs();
#endif
                computeModuleCommands(planned_data_);
                applyModuleCommands(all_homed);
                updateCurrentData(all_homed);
#if JIA_CHASSIS_ENABLE_TASK_PERF_STAT
                apply_us = RtosTimeStampUs64::getTimeUs() - stage_start_us;
#endif

#if JIA_CHASSIS_ENABLE_TASK_PERF_STAT
                stage_start_us = RtosTimeStampUs64::getTimeUs();
#endif
#if JIA_CHASSIS_ENABLE_DEBUG_MIRROR
                refreshDebugMirror(all_homed);
#endif
#if JIA_CHASSIS_ENABLE_DEBUG_OUTPUT
                emitDebugOutputByMode(all_homed);
#endif
#if JIA_CHASSIS_ENABLE_TASK_PERF_STAT
                debug_us = RtosTimeStampUs64::getTimeUs() - stage_start_us;
#endif

                last_planned_data_ = planned_data_;
#if JIA_CHASSIS_ENABLE_TASK_PERF_STAT
                updateTaskPerfBreakdown(plan_us, feedback_us, homing_us, apply_us, debug_us);
                updateTaskPerfStat(loop_start_us, RtosTimeStampUs64::getTimeUs());
#endif
                vTaskDelayUntil(&time_ms_, period_ms_);
            }
        }

#if JIA_CHASSIS_ENABLE_TASK_PERF_STAT
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

        void Chassis::updateTaskPerfBreakdown(u64 plan_us, u64 feedback_us, u64 homing_us, u64 apply_us, u64 debug_us)
        {
            task_perf_stat_.plan_us = plan_us;
            task_perf_stat_.feedback_us = feedback_us;
            task_perf_stat_.homing_us = homing_us;
            task_perf_stat_.apply_us = apply_us;
            task_perf_stat_.debug_us = debug_us;
        }
#endif

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




