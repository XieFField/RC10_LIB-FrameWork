#ifndef CHASSIS_H_
#define CHASSIS_H_

#include "APP_Utils.h"

#include "FreeRTOS.h"

#include "Motor_DJI.h"
#include "Module_CrsfReceiver.h"
#include "APP_debugTool.h"
#include "APP_PID.h"

#ifndef FOURSTEER_SINGLE_WHEEL_TRACE_UART8
#define FOURSTEER_SINGLE_WHEEL_TRACE_UART8 0
#endif

namespace jia
{
    namespace FourSteerChassis
    {
        class Chassis
        {
        public:
            /* ----------------------------------------------------------------- */
            // 对外控制接口
            enum class Result
            {
                kOk,
                kError,
            };

            enum class Coordinate
            {
                kBody,
                kWorld,
            };

            struct ExternalCommand
            {
                Coordinate coord = Coordinate::kBody;
                f32 vel_x = 0.0f;
                f32 vel_y = 0.0f;
                f32 omega_z = 0.0f;
            };

            struct BodyCommand
            {
                f32 vel_x = 0.0f;
                f32 vel_y = 0.0f;
                f32 omega_z = 0.0f;
            };

            struct SteerCalibration
            {
                f32 theta_oa_to_owi_rad = 0.0f;
                f32 homing_runtime_zero_offset_rad = 0.0f;
                f32 steer_motor_sign = 1.0f;
                f32 drive_motor_sign = 1.0f;
            };

            // mode30 单轮直控命令快照。
            // 这不是配置面板，而是每个控制周期里“解析后的当前有效命令”镜像，
            // 便于文本日志、调试器观察和问题回放时直接看到两条轴最终走了什么语义。
            struct DirectActuatorCommandSnapshot
            {
                u8 wheel_idx = 0U;               // [RO] 本次快照对应的目标轮号。mode30 始终只会对这一只轮生成直控命令。
                u8 steer_input_mode = 0U;        // [RO] 舵向轴输入模式快照：缓存值 / 遥控连续 / 遥控阶跃。
                u8 drive_input_mode = 0U;        // [RO] 驱动轴输入模式快照：缓存值 / 遥控连续 / 遥控阶跃。
                u8 steer_command_type = 0U;      // [RO] 舵向轴命令类型快照：电流 / 速度 / 单圈角 / 多圈角。
                u8 drive_command_type = 0U;      // [RO] 驱动轴命令类型快照：速度 / 电流 / 刹车。
                f32 steer_axis_value = 0.0f;     // [RO] 舵向轴本周期摇杆输入值，已归一化到 [-1, 1]；仅 RC 模式有意义。
                f32 drive_axis_value = 0.0f;     // [RO] 驱动轴本周期摇杆输入值，已归一化到 [-1, 1]；仅 RC 模式有意义。
                f32 steer_step_sign = 0.0f;      // [RO] 舵向阶跃方向：-1 / 0 / +1。用于观察 RcStep 当前触发了哪一侧。
                f32 drive_step_sign = 0.0f;      // [RO] 驱动阶跃方向：-1 / 0 / +1。用于观察 RcStep 当前触发了哪一侧。
                f32 steer_command_value = 0.0f;  // [RO] 舵向轴当前原始命令值。单位由 steer_command_type 决定，尚未做最终限幅。
                f32 drive_command_value = 0.0f;  // [RO] 驱动轴当前原始命令值。单位由 drive_command_type 决定，尚未做最终限幅。
                f32 steer_command_limit = 0.0f;  // [RO] 舵向轴本周期使用的命令限幅模板。连续输入映射和最终 clamp 都参考它。
                f32 drive_command_limit = 0.0f;  // [RO] 驱动轴本周期使用的命令限幅模板。连续输入映射和最终 clamp 都参考它。
                f32 steer_step_threshold = 0.0f; // [RO] 舵向轴阶跃触发阈值。摇杆绝对值超过它才会输出阶跃。
                f32 drive_step_threshold = 0.0f; // [RO] 驱动轴阶跃触发阈值。摇杆绝对值超过它才会输出阶跃。
                f32 steer_step_value = 0.0f;     // [RO] 舵向轴阶跃幅值模板。RcStep 触发后输出的是 +/- 这个值。
                f32 drive_step_value = 0.0f;     // [RO] 驱动轴阶跃幅值模板。RcStep 触发后输出的是 +/- 这个值。
                f32 applied_steer_cmd = 0.0f;    // [RO] 舵向轴最终下发值。等于原始命令经过本类型限幅后的结果。
                f32 applied_drive_cmd = 0.0f;    // [RO] 驱动轴最终下发值。等于原始命令经过本类型限幅后的结果。
            };

            static constexpr u8 kTelemetryWheelCount = 4U;

            struct TelemetryChassisState
            {
                f32 vel_x = 0.0f;
                f32 vel_y = 0.0f;
                f32 omega_z = 0.0f;
                f32 yaw_rad = 0.0f;
            };

            struct TelemetryWheelPose
            {
                f32 pos_x_m = 0.0f;
                f32 pos_y_m = 0.0f;
            };

            struct TelemetryWheelState
            {
                f32 target_drive_omega_rad_s = 0.0f;
                f32 actual_drive_omega_rad_s = 0.0f;
                f32 target_steer_oa_rad = 0.0f;
                f32 actual_steer_oa_rad = 0.0f;
                f32 target_velocity_x_m_s = 0.0f;
                f32 target_velocity_y_m_s = 0.0f;
                f32 actual_velocity_x_m_s = 0.0f;
                f32 actual_velocity_y_m_s = 0.0f;
            };

            struct TelemetrySnapshot
            {
                bool homing_all_ready = false;
                TelemetryChassisState target{};
                TelemetryChassisState actual{};
                TelemetryWheelState wheels[kTelemetryWheelCount]{};
            };

            struct PlannerInputCommand
            {
                f32 vel_x = 0.0f;
                f32 vel_y = 0.0f;
                f32 omega_z = 0.0f;
                f32 rot_z = 0.0f;
                bool is_world_speed_mode = false;
                bool is_steer_only_mode = false;
            };

            enum class CommandInputSource : u8
            {
                kApi = 0,
                kDebugTarget = 1,
                kDebugModuleOverride = 2,
            };

            struct NormalizedBodyCommand
            {
                CommandInputSource source = CommandInputSource::kApi;
                BodyCommand body{};
                f32 rot_z = 0.0f;
                bool is_world_speed_mode = false;
                bool is_steer_only_mode = false;
            };

            struct PlannerTargetState
            {
                f32 vel_x = 0.0f;
                f32 vel_y = 0.0f;
                f32 omega_z = 0.0f;
                f32 rot_z = 0.0f;
            };

            struct PlannerInputSnapshot
            {
                PlannerTargetState target{};
            };

            enum class DebugControlRoute : u8
            {
                kDisabled = 0,
                kTargetInjection = 1,
                kModuleOverride = 2,
            };

            enum class DebugModuleOverrideRoute : u8
            {
                kNone = 0,
                kAlignForward = 1,
                kHomingObserve = 2,
                kDirectActuator = 3,
            };

            // 空闲姿态：定义底盘失能或无输入时，四个舵轮应保持的姿态策略。
            // kHoldLast 适合保持最后姿态，kXPark 适合进入 X 停靠姿态以减小外力拖拽干涉。
            enum class IdlePostureMode
            {
                kHoldLast,
                kXPark,
            };

            // 转向解选择策略：kAlwaysForward 永远不走 180 度翻转解；
            // kShortestPath 允许翻转并优先最小转角方案。
            enum class SteeringStrategyMode : u8
            {
                kAlwaysForward = 0,
                kShortestPath = 1,
            };

            enum class ManualSpeedProfileMode : u8
            {
                kLegacy = 0,
                kSCurve = 1,
            };

            struct JerkLimitedAxisState
            {
                f32 shaped_value = 0.0f;
                f32 shaped_accel = 0.0f;
                bool initialized = false;
            };

            // 生命周期
            Chassis() = default;
            ~Chassis() = default;

            // 公开接口
            Result setZeroCurrent();
            // External frame convention for setSpeed*/get*:
            // +x = vehicle right, +y = vehicle front, omega_z keeps current sign convention.
            Result setSpeed(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z);
            Result setSpeed_LockNowYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z = 0.0f);
            Result setSpeed_LockToYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 rot_z);
            Robot_Twist getBodySpeed() const;
            Robot_Twist getWorldSpeed() const;
            Result setSteerDegAndDriveSpeed(f32 steer_angle_deg, f32 chassis_speed_m_s);

            // 兼容控制层
            Result setWheelTorqueFreeMode();
            Result setTargetBodySpeedMode(f32 vel_x, f32 vel_y, f32 omega_z);
            Result setTargetBodySpeedLockNowRotZMode(f32 vel_x, f32 vel_y);
            Result setTargetBodySpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z = 0.0f);
            Result setTargetBodySpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z);
            Result setTargetWorldSpeedMode(f32 vel_x, f32 vel_y, f32 omega_z);
            Result setTargetWorldSpeedLockNowRotZMode(f32 vel_x, f32 vel_y);
            Result setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z = 0.0f);
            Result setTargetWorldSpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z);
            f32 getTargetBodyVelX() const;
            f32 getTargetBodyVelY() const;
            f32 getTargetWorldVelX() const;
            f32 getTargetWorldVelY() const;
            f32 getTargetOmegaZ() const;
            f32 getCurrentBodyVelX() const;
            f32 getCurrentBodyVelY() const;
            f32 getCurrentWorldVelX() const;
            f32 getCurrentWorldVelY() const;
            f32 getCurrentOmegaZ() const;
            static BodyCommand mapExternalCommandToBody(const ExternalCommand &command);
            static BodyCommand normalizeBodyCommandForPlanner(const BodyCommand &command);
            static f32 mapRawSteerMotorTotalToSignedLocalTotal(f32 raw_motor_total_rad, f32 steer_motor_sign);
            static f32 mapSignedLocalTotalToRawSteerMotorTotal(f32 signed_local_total_rad, f32 steer_motor_sign);
            static f32 applyHomingRuntimeZeroOffset(f32 signed_local_total_rad, f32 homing_runtime_zero_offset_rad);
            static f32 removeHomingRuntimeZeroOffset(f32 corrected_local_total_rad, f32 homing_runtime_zero_offset_rad);
            static f32 mapOaTotalToCorrectedLocalTotal(f32 oa_total_rad, const SteerCalibration &calibration);
            static f32 mapCorrectedLocalTotalToOaTotal(f32 corrected_local_total_rad, const SteerCalibration &calibration);
            static f32 mapRawSteerMotorTotalToCorrectedLocalTotal(f32 raw_motor_total_rad, const SteerCalibration &calibration);
            static f32 mapCorrectedLocalTotalToRawSteerMotorTotal(f32 corrected_local_total_rad, const SteerCalibration &calibration);
            static f32 mapDriveMotorRpmToWheelOmega(f32 motor_rpm, const SteerCalibration &calibration);
            static f32 mapWheelOmegaToDriveMotorRpm(f32 wheel_omega_rad_s, const SteerCalibration &calibration);
            static f32 mapWheelCurrentToDriveMotorCurrent(f32 wheel_current_mA, const SteerCalibration &calibration);
            static f32 computeHomingRuntimeZeroOffset(f32 edge_mech_oa_rad,
                                                      f32 raw_motor_total_rad,
                                                      f32 homing_zero_offset_rad,
                                                      const SteerCalibration &calibration);
            static NormalizedBodyCommand makeNormalizedBodyCommand(const PlannerInputCommand &command,
                                                                   f32 input_yaw_rad,
                                                                   CommandInputSource source);
            static PlannerInputSnapshot makePlannerInputSnapshot(const PlannerInputCommand &command, f32 input_yaw_rad);
            static DebugControlRoute classifyDebugControlRoute(bool debug_enable, u8 raw_mode);
            static DebugModuleOverrideRoute classifyDebugModuleOverrideRoute(u8 raw_mode);
            static TelemetrySnapshot makeTelemetrySnapshot(bool homing_all_ready,
                                                           const TelemetryChassisState &target,
                                                           const TelemetryChassisState &actual,
                                                           const TelemetryWheelPose wheel_pose[kTelemetryWheelCount],
                                                           const f32 target_drive_omega_rad_s[kTelemetryWheelCount],
                                                           const f32 actual_drive_omega_rad_s[kTelemetryWheelCount],
                                                           const f32 target_steer_oa_rad[kTelemetryWheelCount],
                                                           const f32 actual_steer_oa_rad[kTelemetryWheelCount]);

            // 初始化配置：只做硬件句柄绑定。
            struct InitConfig
            {
                Motor_Base *steer_motor_h[4] = {nullptr}; // 4 个转向电机句柄，顺序需与 wheels[4] 的轮位定义保持一致
                Motor_Base *drive_motor_h[4] = {nullptr}; // 4 个驱动电机句柄，顺序需与对应转向模块一一匹配
            };

            // 初始化与运行时策略接口
            void init(InitConfig &config);
            void setIdlePostureMode(IdlePostureMode mode);
            void setSteeringStrategyMode(SteeringStrategyMode mode);
            

        private:
            Result startHoming();
            bool isHomingDone() const;
            void resetRuntimeStrategyToInitConfig();

            // 内部策略/状态类型
            enum class HomingState : u8
            {
                kIdle,
                kSearch,
                kEdgeDetected,
                kOffsetApply,
                kContinuousAngleReady,
                kReady,
                kAlignToZero,
                kFault,
            };

            enum class HomingFaultReason : u8
            {
                kNone = 0,
                kTimeout = 1,
            };

            enum class SteerFaultState : u8
            {
                kNone = 0,
                kLatched = 1,
                kRecovering = 2,
            };

            struct WheelInitConfig
            {
                f32 pos_x_m = 0.0f;
                f32 pos_y_m = 0.0f;
                f32 theta_oa_to_owi_deg = 0.0f;
                f32 steer_motor_sign = 1.0f;
                f32 drive_motor_sign = 1.0f;
                bool homing_enabled = false;
                bool homing_sensor_active_high = true;
                void *homing_gpio_port = nullptr;
                u16 homing_gpio_pin = 0;
                f32 homing_falling_edge_mech_deg = 60.0f;
                f32 homing_rising_edge_mech_deg = -120.0f;
                f32 homing_search_rpm = 10.0f;
                f32 homing_zero_offset_deg = 0.0f;
                f32 homing_timeout_s = 5.0f;
            };

            // WheelConfig 是运行时轮组状态快照：既保存静态几何和硬件句柄，也保存回零状态、
            // 补偿结果与最近一次规划输出，供控制线程在每个周期更新。
            struct WheelConfig
            {
                f32 pos_x_m = 0.0f;                               // 该舵轮模块在车体坐标系中的 x 安装位置，单位米
                f32 pos_y_m = 0.0f;                               // 该舵轮模块在车体坐标系中的 y 安装位置，单位米
                f32 theta_oa_to_owi_rad = 0.0f;                   // 安装几何偏移：把舵轮在底盘平面内实际指向/滚动的方向（OA 朝向）换算到转向电机本地机械角参考系（OWI）；它用于坐标变换，不是回零补偿
                f32 steer_motor_sign = 1.0f;                      // 转向电机方向符号：1 表示不取反，-1 表示转向反馈和目标指令都按相反方向解释
                f32 drive_motor_sign = 1.0f;                      // 驱动电机方向符号：1 表示不取反，-1 表示驱动反馈和目标指令都按相反方向解释
                Motor_Base *steer_motor_h = nullptr;              // 该模块绑定的转向电机句柄
                Motor_Base *drive_motor_h = nullptr;              // 该模块绑定的驱动电机句柄
                bool homing_enabled = false;                      // 是否对该轮启用回零流程；false 时默认认为零位已可用
                bool homing_sensor_active_high = true;            // 回零传感器逻辑 active 极性：true 表示高电平视为有效，false 表示低电平视为有效；不决定 H/L 边沿的机械角语义
                void *homing_gpio_port = nullptr;                 // 回零传感器 GPIO 端口运行时副本；读取零位输入时直接使用
                u16 homing_gpio_pin = 0;                          // 回零传感器 GPIO 引脚运行时副本；与端口配合读取真实输入
                f32 homing_falling_edge_mech_rad = 0.0f;          // 原始 GPIO H->L 边沿对应的机械 OA 角（rad）
                f32 homing_rising_edge_mech_rad = 0.0f;           // 原始 GPIO L->H 边沿对应的机械 OA 角（rad）
                f32 homing_search_rpm = 10.0f;                    // 回零搜索阶段给转向电机的转速指令，单位 rpm
                f32 homing_zero_offset_rad = 0.0f;                // 标定得到的零位补偿角：传感器触发点到期望机械零位的固定偏差
                f32 homing_timeout_s = 5.0f;                      // 单轮回零允许持续的最长时间，超时后进入故障态，单位秒
                HomingState homing_state = HomingState::kIdle;    // 当前轮回零状态机所处阶段
                bool homing_last_sensor_active = false;           // 上一控制周期的原始 GPIO 高低电平；用于检测 H/L 边沿
                bool homing_last_edge_is_falling = false;         // 最近一次抓到的边沿方向：true=H->L，false=L->H；方便调试极性和触发角
                bool homing_align_command_armed = false;          // 进入 AlignToZero 后是否已允许下发第一次对零位置命令；用于避免边沿抓取后同拍大跳变
                bool homing_zero_valid = false;                   // 当前轮是否已经建立可用于闭环控制的零位
                f32 homing_elapsed_s = 0.0f;                      // 本次回零已运行时间，单位秒；用于超时判定
                f32 homing_runtime_zero_offset_rad = 0.0f;        // 本次上电运行实际采用的零位补偿；回零成功后会把“当前触发位置”修正成运行时零点
                f32 corrected_steer_motor_total_angle_rad = 0.0f; // 已乘方向符号并叠加运行时零位补偿后的转向电机连续总角度反馈
                f32 corrected_drive_omega_rad_s = 0.0f;           // 已乘方向符号后的驱动轮角速度反馈，单位 rad/s
                f32 target_steer_motor_total_angle_rad = 0.0f;    // 当前周期解算后要发给转向电机的本地连续目标角
                f32 target_drive_omega_rad_s = 0.0f;              // 当前周期解算后要发给驱动电机的目标角速度，单位 rad/s
                f32 steer_target_velocity_rad_s = 0.0f;           // 转向二阶限幅后得到的目标角速度，便于平滑舵向变化
                bool flipped_drive_direction = false;             // 本周期是否采用“舵角翻转 180 度、驱动反向”策略来走更短转角路径
                SteerFaultState steer_fault_state = SteerFaultState::kNone;
                bool steer_fault_rehome_request = false;
                f32 steer_feedback_current_mA = 0.0f;
                f32 steer_feedback_last_current_mA = 0.0f;
                f32 steer_feedback_last_raw_total_angle_rad = 0.0f;
                f32 steer_feedback_current_delta_mA = 0.0f;
                f32 steer_feedback_angle_delta_rad = 0.0f;
                f32 steer_fault_steer_error_rad = 0.0f;
                bool steer_fault_control_intent = false;
                bool steer_fault_xpark_stationary_hold = false;
                bool steer_fault_freeze_candidate = false;
                u32 steer_feedback_freeze_ms = 0U;
                u32 steer_feedback_recovery_toggle_count = 0U;
                u32 steer_fault_latched_count = 0U;
            };

            // Mode 表示四舵轮底盘当前采用的控制语义。
            // 可理解为扭矩自由、车体系/世界系速度控制，以及“锁当前 yaw / 锁目标 yaw”
            // 的不同组合展开。
            enum class Mode
            {
                kWheelTorqueFreeMode,
                kBodySpeedMode,
                kBodySpeedLockNowRotZMode,
                kBodySpeedLockToRotZMode,
                kWorldSpeedMode,
                kWorldSpeedLockNowRotZMode,
                kWorldSpeedLockToRotZMode,
                kWorldSpeedLockNowRotZWithNoOmegaZMode,
                kBodySpeedLockNowRotZWithNoOmegaZMode,
                kSteerAngleAndDriveSpeedMode,
            };

            // ModeFlag 是从 Mode 派生出的布尔型分支标记，用来减少线程内重复比对枚举。
            // 它只描述当前控制意图，不表示轮子是否已经回零成功。
            struct ModeFlag
            {
                bool is_wheel_torque_free = false; // 是否为轮子扭矩自由模式
                bool is_world_speed_mode = false;  // 是否为世界坐标系速度模式
                bool is_lock_now_rot_z = false;    // 是否固定当前rot_z
                bool is_lock_to_rot_z = false;     // 是否固定到rot_z
            };

            // InputTargetData 保存上层最近一次输入的目标意图：
            // vel_x / vel_y / omega_z / rot_z 分别对应平移、偏航角速度和目标偏航角，
            // mode 则决定这些输入要走哪条控制路径。
            struct InputTargetData
            {
                f32 vel_x = 0.0f;
                f32 vel_y = 0.0f;
                f32 omega_z = 0.0f;
                f32 rot_z = 0.0f;
                f32 steer_lock_angle_deg = 0.0f;
                f32 drive_lock_speed_m_s = 0.0f;
                bool zero_current_all = false;
                Mode mode = Mode::kWheelTorqueFreeMode;
            };

            struct Data
            {
                f32 vel_x = 0.0f;
                f32 vel_y = 0.0f;
                f32 omega_z = 0.0f;
                f32 acc_x = 0.0f;
                f32 acc_y = 0.0f;
                f32 alpha_z = 0.0f;
                f32 rot_z = 0.0f;
                f32 steer_angle_oa_rad[4] = {0.0f};
                f32 drive_omega_rad_s[4] = {0.0f};
            };

            struct SwervePlannerInput
            {
                Data command{};
                bool command_stationary_intent = false;
                bool allow_xpark_pose = false;
                bool force_uniform_steer_drive = false;
                f32 uniform_steer_oa_mod_rad = 0.0f;
                f32 uniform_drive_omega_abs = 0.0f;
                f32 uniform_drive_sign = 1.0f;
                f32 current_oa_total_rad[4] = {0.0f};
                f32 wheel_vx_m_s[4] = {0.0f};
                f32 wheel_vy_m_s[4] = {0.0f};
                f32 wheel_speed_m_s[4] = {0.0f};
                f32 residual_speed_m_s[4] = {0.0f};
                f32 max_command_wheel_speed_m_s = 0.0f;
                f32 max_residual_speed_m_s = 0.0f;
            };

            struct SwervePlannerOutput
            {
                f32 ideal_oa_total_rad[4] = {0.0f};
                f32 ideal_drive_omega_rad_s[4] = {0.0f};
                f32 selected_oa_total_rad[4] = {0.0f};
                f32 steering_errors_rad[4] = {0.0f};
                f32 planned_corrected_local_total_rad[4] = {0.0f};
                f32 planned_oa_total_rad[4] = {0.0f};
                f32 planned_steer_rate_rad_s[4] = {0.0f};
                f32 projected_drive_omega_rad_s[4] = {0.0f};
                f32 final_drive_omega_rad_s[4] = {0.0f};
                f32 low_speed_suppression_scale[4] = {1.0f, 1.0f, 1.0f, 1.0f};
                bool flipped_drive_direction[4] = {false, false, false, false};
                f32 high_speed_suppression_scale = 1.0f;
                bool high_speed_suppression_active = false;
                f32 high_speed_dir_err_deg = 0.0f;
                f32 high_speed_eta_max_s = 0.0f;
            };

            struct ActuatorCommandFrame
            {
                f32 steer_corrected_local_total_rad[4] = {0.0f};
                f32 steer_oa_total_rad[4] = {0.0f};
                f32 steer_rate_rad_s[4] = {0.0f};
                f32 drive_omega_rad_s[4] = {0.0f};
                bool flipped_drive_direction[4] = {false, false, false, false};
            };

            // 创建线程
            static void createThread(void *arg);
            // 运行线程函数
            void runThread(void *arg);

            // 输入目标数据
            void isDebugMode();
            enum class DebugMode : u8
            {
                kTorqueFree = 0,
                kBodySpeed = 1,
                kWorldSpeed = 2,
                kBodyLockNow = 3,
                kWorldLockNow = 4,
                kBodyLockTo = 5,
                kWorldLockTo = 6,
                kBodyLockNowWithNoOmegaZ = 7,
                kWorldLockNowWithNoOmegaZ = 8,
                kAlignForward = 21,
                kHomingObserve = 22,
                kDirectActuator = 30,
            };
            enum class DebugOmegaZInjectionMode : u8
            {
                kOff = 0,
                kStep = 1,
                kSine = 2,
            };
            // mode30 单轴输入来源。
            // 设计成舵向/驱动各自独立，避免两个轴被同一个输入模式强行绑定。
            enum class DirectAxisInputMode : u8
            {
                kCached = 0,      // 直接使用调试面板里缓存的命令值。
                kRcContinuous = 1, // 使用遥控摇杆连续量，并按当前命令限幅映射。
                kRcStep = 2,      // 使用遥控摇杆阶跃触发，超过阈值后输出固定步进值。
            };
            // mode30 舵向轴命令类型。
            // 当前活跃命令值、限幅和阶跃模板的物理单位都由它决定。
            enum class DirectSteerCommandType : u8
            {
                kCurrent = 0,       // 直接给舵向电流命令（mA）。
                kRpm = 1,           // 直接给舵向速度命令（rpm）。
                kSingleTurnDeg = 2, // 直接给舵向单圈角命令（deg）。
                kMultiTurnDeg = 3,  // 直接给舵向多圈角命令（deg）。
            };
            // mode30 驱动轴命令类型。
            // drive 轴只保留速度/电流/刹车三种互斥语义，避免旧版多槽缓存并存。
            enum class DirectDriveCommandType : u8
            {
                kRpm = 0,     // 直接给驱动速度命令（rpm）。
                kCurrent = 1, // 直接给驱动电流命令（mA）。
                kBrake = 2,   // 直接给驱动刹车命令（mA）。
            };
            enum class DebugOutputMode : u8
            {
                kOff = 0,
                kText = 1,
                kOverviewJustFloat = 2,
                kSingleWheelJustFloat = 3,
                kSingleWheelDualMotorJustFloat = 4,
                kSwerveTelemetryV2 = 5,
                kYawPidJustFloat = 6,
            };
            DebugMode resolveDebugMode(u8 raw_mode) const;
            void applyDebugTargetOverride(DebugMode mode);
            bool applyDebugModuleOverride(bool all_homed);
            void emitDebugOutputByMode(bool all_homed);
            void clearInputTargetData();
            void setModeFlag();
            void resolvePlannerTargetData();
            void updatePlannedMotionData();
            void clearPlannedMotionForModuleOverride();
            void resetDebugModuleOverrideTargets(u8 wheel_idx, bool preserve_soft_wheel_rate);
            void applyAlignForwardDebugOverride();
            void applyHomingObserveDebugOverride();
            void applyDirectActuatorDebugOverride(u8 wheel_idx);
            void finalizeDebugModuleOverride(bool all_homed, DebugModuleOverrideRoute route);
            void syncDirectActuatorCommandTemplates(); // mode30 类型切换同步入口。用于在命令类型改变时刷新单值命令、限幅和阶跃模板。
            DirectActuatorCommandSnapshot resolveDirectActuatorCommand(u8 wheel_idx); // 解析当前 control_wheel_index 对应轮的 mode30 双轴有效命令快照。
            void clearDirectDriveCommandByType(WheelConfig &wheel, u8 wheel_idx, u8 drive_control_type);
            void applyDirectActuatorSteerCommand(WheelConfig &wheel, u8 wheel_idx, const DirectActuatorCommandSnapshot &command);
            void applyDirectActuatorDriveCommand(WheelConfig &wheel, u8 wheel_idx, const DirectActuatorCommandSnapshot &command);
            void transSpeedBodyToWorld(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y) const;
            void transSpeedWorldToBody(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y) const;
            void isLockNowRotZ(bool is_lock, f32 rot_z, f32 omega_z, f32 &out_rot_z, f32 &out_omega_z);
            void isLockToRotZ(bool is_lock, f32 tar_rot_z, f32 cur_rot_z, f32 &out_rot_z, f32 omega_z, f32 &out_omega_z);
            void clampTargetSpeedInChassis(f32 vel_x, f32 vel_y, f32 omega_z, f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z) const;
            void limitPlannedSpeed(f32 tar_vel_x, f32 tar_vel_y, f32 tar_omega_z, f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z);
            ManualSpeedProfileMode resolveEffectiveManualSpeedProfileMode() const;
            void resetManualSpeedProfileRuntimeState(bool reset_gate_state);
            f32 limitValueByJerkProfile(f32 target_value,
                                        f32 current_value,
                                        JerkLimitedAxisState &axis_state,
                                        f32 accel_limit,
                                        f32 decel_limit,
                                        f32 jerk_acc_limit,
                                        f32 jerk_dec_limit,
                                        f32 settle_vel_epsilon,
                                        f32 settle_accel_epsilon) const;
            void updateWheelFeedback();
            void updateSteerFaultState(WheelConfig &wheel);
            void latchSteerFault(WheelConfig &wheel);
            void clearSteerFaultState(WheelConfig &wheel);
            void requestSingleWheelHoming(WheelConfig &wheel);
            void resetSteerMotorClosedLoopState(WheelConfig &wheel);
            bool updateHomingState(WheelConfig &wheel);
            bool readHomingSensor(const WheelConfig &wheel) const;
            bool readHomingSensorRawHigh(const WheelConfig &wheel) const;
            f32 readSteerMotorRawTotalAngleRad(const WheelConfig &wheel) const;
            f32 readDriveMotorOmegaRadS(const WheelConfig &wheel) const;
            f32 readCorrectedSteerMotorTotalAngleRad(const WheelConfig &wheel) const;
            f32 readSteerMotorCurrentMilliAmp(const WheelConfig &wheel) const;
            void setSteerMotorTargetCurrent(WheelConfig &wheel, f32 current);
            void setSteerMotorTargetRPM(WheelConfig &wheel, f32 rpm);
            void setSteerMotorTargetTotalAngleRad(WheelConfig &wheel, f32 corrected_local_total_angle_rad);
            void setDriveMotorTargetOmegaRadS(WheelConfig &wheel, f32 drive_omega_rad_s);
            f32 limitPositionSecondOrder(f32 current_value, f32 current_rate, f32 target_value, f32 max_rate, f32 max_accel, f32 dt_s, f32 &next_rate) const;
            f32 limitValueWithAcceleration(f32 current_value, f32 target_value, f32 max_accel, f32 dt_s) const;
            f32 wrapToPi(f32 angle_rad) const;
            f32 wrapTo2Pi(f32 angle_rad) const;
            f32 shortestAngularDistance(f32 from_rad, f32 to_rad) const;
            f32 nearestEquivalentAngle(f32 current_rad, f32 target_mod_rad) const;
            f32 magnitude2D(f32 x, f32 y) const;
            f32 getXParkAngle(const WheelConfig &wheel) const;
            f32 computeMaxCommandWheelSpeedMps(const Data &command_data) const;
            f32 computeLowSpeedDriveSuppressionScale(f32 abs_error_rad) const;
            void computeLowSpeedDriveSuppressionScales(const SwervePlannerInput &planner_input, const f32 steering_errors_rad[4], f32 out_scales[4]);
            f32 getNearZeroEnterSpeedMps() const;
            f32 getNearZeroExitSpeedMps() const;
            f32 getXParkCommandEnterSpeedMps() const;
            f32 getXParkCommandExitSpeedMps() const;
            bool shouldActivateReverseIntent(f32 target_vel_x, f32 target_vel_y, f32 reference_dir_rad) const;
            bool shouldActivateLaunchHold() const;
            bool isLaunchHoldAligned(const SwervePlannerOutput &planner_output) const;
            Data makeLaunchHoldPreviewCommand() const;
            void refreshActuatorLimitState();
            f32 mapSingleTurnToNearestTotalAngle(const WheelConfig &wheel, f32 target_oa_single_turn_deg) const;
            SwervePlannerInput makeSwervePlannerInput(const Data &command_data);
            SwervePlannerOutput planSwerveModules(const SwervePlannerInput &planner_input);
            void buildActuatorCommandFrame(const SwervePlannerOutput &planner_output, ActuatorCommandFrame &out_frame) const;
            void storePlannedActuatorFrame(const SwervePlannerOutput &planner_output, const ActuatorCommandFrame &command_frame);
            f32 computeHomingAlignTargetCorrectedLocalTotal(const WheelConfig &wheel) const;
            void computeProjectedDriveFromPlannedSteer(const Data &command_data, const f32 planned_oa_total_rad[4], f32 out_drive_omega_rad_s[4]) const;
            bool estimatePlannedBodyTwist(const f32 planned_oa_total_rad[4], const f32 planned_drive_omega_rad_s[4], f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z) const;
            f32 updateHighSpeedDriveSuppression(f32 translational_speed_m_s, f32 eta_max_s, f32 dir_err_deg);
            void computeModuleCommands(const Data &command_data);
            void applyModuleCommands(bool all_homed);
            void updateCurrentData(bool all_homed);
            void refreshDebugMirror(bool all_homed);
            void emitDebugUart8Log(bool all_homed);
            void emitUart8VofaJustFloatPidTrace();
            void syncDebugSteerPidTuneFromRuntimeOnEnableEdge();
            void syncDebugSteerPidTuneFromRuntime();
            void applyDebugSteerPidRuntimeTuning();
            void emitUart8VofaPid1kHzTrace();
            void emitUart8VofaDualMotor1kHzTrace();
            void emitUart8SwerveTelemetryV2(bool all_homed);
            void emitUart8VofaYawPidTrace();
            bool solveLinear3x3(f32 matrix[3][4], f32 &x0, f32 &x1, f32 &x2) const;
            bool estimateBodySpeedFromModules(f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z) const;
            void updateTaskPerfStat(u64 loop_start_us, u64 loop_end_us);
            static SteerCalibration makeSteerCalibration(const WheelConfig &wheel);
            static f32 mapWheelCorrectedLocalToOaTotal(const WheelConfig &wheel, f32 corrected_local_total_rad);
            static f32 mapWheelOaTotalToCorrectedLocal(const WheelConfig &wheel, f32 oa_total_rad);

            // =====================================================================
            // 系统时基 [RO]
            // 说明：底盘控制链路的统一时间基准。看这里时，先把它理解成“每周期多长”和“当前时刻是多少”。
            // 这里通常不需要改；如果控制周期变化，和时间相关的节流、滤波、超时常量也要一起复核。
            // =====================================================================
            constexpr static u8 period_ms_ = 1;                  // [RO] 控制周期步长（ms）。用于把“每周期”换算成真实时间，默认 1ms。
            constexpr static f32 period_ = period_ms_ / 1000.0f; // [RO] 控制周期步长（s）。给需要秒单位的公式使用，和 period_ms_ 始终一致。
            TickType_t time_ms_ = 0;                             // [RO] 当前系统时刻（ms）。随控制线程推进，用于节流、计时和超时判断。

            // =====================================================================
            // 底盘参数与策略（运行时可调）[RW]
            // 说明：FourSteer 初始化后会把这里作为默认基线读取。
            // 这一组决定“车能跑多快、加减速有多柔和、怎么解算模块、什么时候压驱动、停下来时保持什么姿态”。
            // =====================================================================
            struct StrategyConfig
            {
                ManualSpeedProfileMode manual_speed_profile_mode = ManualSpeedProfileMode::kSCurve;
                bool manual_speed_profile_manual_only = true;
                f32 manual_trans_acc_acc_ = 5.0f;
                f32 manual_trans_acc_dec_ = 12.0f;
                f32 manual_trans_jerk_acc_ = 50.0f;
                f32 manual_trans_jerk_dec_ = 50.0f;
                f32 manual_trans_settle_vel_eps_ = 1.0e-4f;
                f32 manual_trans_settle_acc_eps_ = 0.05f;
                f32 manual_yaw_alpha_acc_ = 5.0f;
                f32 manual_yaw_alpha_dec_ = 12.0f;
                f32 manual_yaw_jerk_acc_ = 50.0f;
                f32 manual_yaw_jerk_dec_ = 50.0f;
                f32 manual_yaw_settle_vel_eps_ = 1.0e-4f;
                f32 manual_yaw_settle_acc_eps_ = 0.05f;
                f32 wheel_radius_m_ = 0.052f;                                    // [RW, 慎改] 轮半径。决定线速度与驱动角速度的换算比例，改错会直接导致速度尺度和里程计比例偏差。
                f32 max_vel_x_ = 2.0f;                                           // [RW] 车体 X 方向最大线速度上限（m/s）。用于规划/限幅，不是电机硬件极限。
                f32 max_vel_y_ = 2.0f;                                           // [RW] 车体 Y 方向最大线速度上限（m/s）。同上，约束横移速度。
                f32 max_omega_z_ = 2.0f;                                         // [RW] 车体 Z 轴最大角速度上限（rad/s）。同上，约束原地旋转或航向变化速度。
                f32 max_acc_xy_acc_ = 2.0f;                                      // [RW] 平面加速段最大加速度（m/s^2）。越小起步越柔和，越大响应越猛。
                f32 max_acc_xy_dec_ = 99999999.0f;                                     // [RW] 平面减速段最大减速度（m/s^2）。越小刹车越平滑，越大停车越快但冲击更强。
                f32 max_alpha_z_acc_ = 2.0f;                                     // [RW] 航向加速段最大角加速度（rad/s^2）。影响转向起步的平顺性。
                f32 max_alpha_z_dec_ = 99999999.0f;                                    // [RW] 航向减速段最大角减速度（rad/s^2）。影响转向收尾和停摆冲击。
                f32 trans_dir_rate_limit_deg_s_ = 99999999.0f;                   // [RW] 平移速度矢量方向变化率上限（deg/s）。限制“速度方向”每秒最多转多少度。
                bool enable_drive_omega_limit_ = false;                          // [RW] 是否启用驱动角速度上限。
                f32 max_drive_omega_rad_s_ = 99999999.0f;                        // [RW] 驱动目标角速度上限（rad/s）。仅在 enable_drive_omega_limit_=true 时生效。
                bool enable_drive_alpha_limit_ = false;                          // [RW] 是否启用驱动角加速度上限。
                f32 max_drive_alpha_rad_s2_ = 99999999.0f;                       // [RW] 驱动角速度变化率上限（rad/s^2）。仅在 enable_drive_alpha_limit_=true 时生效。
                bool enable_steer_rate_limit_ = false;                           // [RW] 是否启用舵向角速度上限。
                f32 max_steer_rate_rad_s_ = 200.0f;                         // [RW] 转向目标角速度上限（rad/s）。仅在 enable_steer_rate_limit_=true 时生效。
                bool enable_steer_alpha_limit_ = true;                          // [RW] 是否启用舵向角加速度上限。
                f32 max_steer_alpha_rad_s2_ = 20000.0f;                          // [RW] 转向目标角加速度上限（rad/s^2）。仅在 enable_steer_alpha_limit_=true 时生效。

                // ---- 近零门限统一配置 --------------------------------------------
                // 所有静止/冻结/X-Park/停车保护阈值统一由这组基准参数派生，避免多处手改失配。
                struct NearZeroThresholdConfig
                {
                    f32 base_enter_m_s = 0.10f; // [RW] 近零门限进入基准（m/s）。
                    f32 base_exit_m_s = 0.15f;  // [RW] 近零门限退出基准（m/s）。应大于 enter 形成滞回。
                } near_zero_cfg_;

                struct XParkCommandThresholdConfig
                {
                    f32 enter_m_s = 0.01f; // [RW] 仅用于 X-Park 命令静止意图的进入门限（m/s），不用于残余反馈过滤。
                    f32 exit_m_s = 0.03f;  // [RW] 仅用于 X-Park 命令静止意图的退出门限（m/s）。应大于 enter 形成滞回。
                } xpark_command_threshold_cfg_;

                struct LowSpeedDriveSuppressionConfig
                {
                    f32 close_angle_deg = 10.0f;             // [RW] 低速抑制使用。舵角误差超过该阈值后进入驱动压制区。
                    f32 min_scale = 0.0f;                   // [RW] 低速抑制使用。进入压制区后保留的最小驱动比例。
                };

                struct SteerFaultConfig
                {
                    bool enable = true;                           // [RW] 是否启用舵向断链检测/恢复状态机。关闭后仅保留观测，不再锁故障。
                    bool ignore_during_xpark_hold = false;         // [RW] 是否在 X 驻车静止保持期间屏蔽舵向断链判定，避免静止姿态误判。
                    f32 freeze_current_delta_mA = 2.0f;           // [RW] 电流冻结阈值（mA）。相邻周期变化不超过该值时，认为电流近似不变。
                    f32 active_current_min_mA = 0.0f;            // [RW] 激活检测所需最小电流幅值（mA）。低于该值时即便冻结也不判故障。
                    f32 freeze_angle_delta_rad = 0.0175f;         // [RW] 角度冻结阈值（rad）。相邻周期总角度变化不超过该值时，认为角度近似不变。
                    u32 freeze_duration_ms = 100U;                // [RW] 冻结持续时长（ms）。冻结候选持续达到该时长才锁故障。
                    f32 recovery_current_delta_mA = 2.0f;         // [RW] 恢复电流跳变阈值（mA）。相邻周期电流变化超过该值时记一次恢复跳动。
                    u32 recovery_toggle_threshold = 100U;         // [RW] 恢复跳动计数门槛。达到该次数后切入恢复重校准。
                } steer_fault_cfg{};

                // ---- 舵角解算 ----------------------------------------------------
                // 决定每个模块在“直接转过去”与“翻转 180° 再配合驱动反向”之间如何选择。
                // 这个选择直接影响转向路径长度、驱动方向是否反转，以及模块在大角度切换时是否抖动。
                SteeringStrategyMode steering_strategy_mode = SteeringStrategyMode::kShortestPath; // [RW] 舵角解算策略。不同策略在转向路径和稳定性上有不同权衡。
                f32 flip_enter_angle_deg = 135.0f;                                        // [RW] 翻转保持上阈值（deg）。当前已在翻转解时，只有翻转解角差超过该阈值才退出翻转。
                f32 flip_exit_angle_deg = 80.0f;                                          // [RW] 翻转切入下阈值（deg）。当前未翻转时，直达解角差足够大且翻转解更优才切入翻转。应小于 flip_enter_angle_deg 形成滞回。
                struct ReverseIntentConfig
                {
                    bool enable = true;
                    f32 enter_angle_deg = 105.0f;
                    f32 exit_angle_deg = 75.0f;
                    f32 min_speed_m_s = 0.0f;
                    f32 flip_prefer_margin_deg = 5.0f;
                } reverse_intent{};

                bool enable_low_speed_drive_suppression = true; // [RW] 是否启用低速抑制。仅在近零/低速找向阶段额外压低驱动。
                LowSpeedDriveSuppressionConfig low_speed_drive_suppression{};

                // ---- 静止姿态 ----------------------------------------------------
                IdlePostureMode idle_posture_mode = IdlePostureMode::kXPark; // [RW] 静止姿态策略。决定停住后是维持当前轮姿态，还是自动收拢为 X-Park。
                u32 xpark_entry_delay_ms = 1000U;                            // [RW] X-Park 进入最短静止持续时间（ms）。

                struct HighSpeedDriveSuppressionConfig
                {
                    f32 dir_err_enter_deg = 12.0f;          // [RW] 高速抑制使用。方向误差进入阈值（deg）。
                    f32 dir_err_exit_deg = 6.0f;            // [RW] 高速抑制使用。方向误差退出阈值（deg），应小于 enter 形成滞回。
                    f32 eta_lock_s = 0.20f;                 // [RW] 高速抑制使用。最大到角时间进入阈值（s）。
                    f32 eta_release_s = 0.06f;              // [RW] 高速抑制使用。最大到角时间退出阈值（s），应小于 lock。
                    f32 gate_ramp_up_s = 0.08f;             // [RW] 高速抑制使用。门控放开时间常数（s）。
                    f32 gate_ramp_down_s = 0.03f;           // [RW] 高速抑制使用。门控收紧时间常数（s）。
                };
                bool enable_high_speed_drive_suppression = false; // [RW] 是否启用高速抑制。只在非近零平移一致性变差时收紧驱动。
                HighSpeedDriveSuppressionConfig high_speed_drive_suppression{};
            };
            StrategyConfig default_strategy_cfg_; // [RW, 慎改] 默认策略基线。用于初始化和“恢复默认值”，不要把它当作实时状态。
            StrategyConfig runtime_strategy_cfg_; // [RW] 当前生效的运行时策略。可被外部接口动态切换，控制链路实际读取它。
            
            // =====================================================================
            // 航向控制参数（运行时可调）[RW]
            // 通过全局 chassis 对象在调试器内直接改值。[RW]
            // 说明：这组参数只影响航向锁定/锁角逻辑，不影响平移速度规划。
            // =====================================================================
            PID_Position rot_z_pid_;                  // [RW, 慎改] 航向位置环 PID。用于 LockToYaw / 相关锁角模式的角度误差闭环。
            u8 rot_z_pid_period_ = 1;                 // [RW] PID 更新周期分频。1 表示每个控制周期都更新，数值越大频率越低、负载越小但响应更慢。
            f32 max_lock_to_rot_z_rad_s_ = 2.0f;      // [RW] LockToYaw 模式下的角速度上限（rad/s）。用于限制“往目标角赶”的最快速度。
            u32 lock_now_rot_z_shift_time_ms_ = 1000; // [RW] LockNow 松手缓冲时长（ms）。松开后短时间内继续维持目标，避免姿态突然跳变。

            // =====================================================================
            // 调试参数（通过全局 chassis 对象在调试器内直接改值）[RW]
            // 说明：这组参数只影响调试链路。正常控制不读取它们，只有切到相应 debug mode 时才会生效。
            // 速查：0~8 = 底盘输入接管/信号注入类模式；20 = 已退役（安全回退）；21 = 四轮朝前；22 = 回零观察；30 = 执行层直控。
            // 手柄平移坐标约定（底盘层）：前推前进、左推左移（-left_y -> vel_x，left_x -> vel_y）。
            // =====================================================================
            struct DebugControl
            {
                // ---- 调试总开关与模式入口 ---------------------------------------
                bool enable = false;                                             // [RW] 调试总开关。false 时整个调试接管链路不生效，系统走正常控制。
                u8 mode_raw = 2;                                                // [RW] 调试模式号。决定当前是输入接管、单轮调试、回零观察还是执行层直控。
                u8 mode_resolved_raw = static_cast<u8>(DebugMode::kWorldSpeed); // [RO] 解析后的实际模式号。用于观察 mode_raw 经过归一化后的结果。
                u8 control_wheel_index = 1;                                     // [RW] mode30 执行目标轮号（0~3）。只用于执行层直控链路选择当前被操作的轮。
                u8 observe_wheel_index = 1;                                     // [RW] 单轮观测焦点轮号（0~3）。文本单轮日志与单轮高速输出统一跟随该轮。

                // ---- LockTo / omega_z 注入 ---------------------------------------
                f32 lock_rot_z = 0.0f;     // [RW] LockTo 调试目标角（rad）。只在 LockTo 相关调试模式中使用，不参与 omega_z 注入。
                u8 omega_z_injection_mode_raw = static_cast<u8>(DebugOmegaZInjectionMode::kOff); // [RW] omega_z 注入模式：0=关闭，1=阶跃覆盖，2=正弦覆盖。启用后会覆盖正常 omega_z 输入，不是叠加。
                f32 omega_z_sine_amplitude = 0.0f;    // [RW] omega_z 正弦注入幅值。仅在注入模式为 Sine 时使用。
                f32 omega_z_sine_frequency_hz = 0.1f; // [RW] omega_z 正弦注入频率（Hz）。仅在注入模式为 Sine 时使用。
                f32 omega_z_sine_offset = 0.0f;       // [RW] omega_z 正弦注入偏置。仅在注入模式为 Sine 时使用。

                // ---- mode30：执行层直控 -----------------------------------------
                bool direct_estop = false;                                       // [RW] mode30 急停闸门。true 时禁止所有执行层输出，适合调试前的“安全锁”。
                bool direct_enable_steer = true;                                 // [RW] mode30 当前控制轮的舵向轴执行使能。false 时只保留安全清零，不下发舵向直控。
                bool direct_enable_drive = true;                                 // [RW] mode30 当前控制轮的驱动轴执行使能。false 时只保留安全清零，不下发驱动直控。
                u8 direct_steer_input_mode_raw = static_cast<u8>(DirectAxisInputMode::kCached); // [RW] mode30 舵向轴输入模式：0=缓存值，1=遥控连续，2=遥控阶跃。
                u8 direct_drive_input_mode_raw = static_cast<u8>(DirectAxisInputMode::kCached); // [RW] mode30 驱动轴输入模式：0=缓存值，1=遥控连续，2=遥控阶跃。
                u8 direct_steer_command_type_raw = static_cast<u8>(DirectSteerCommandType::kRpm); // [RW] mode30 舵向轴命令类型：0=电流，1=速度，2=单圈角，3=多圈角。
                u8 direct_drive_command_type_raw = static_cast<u8>(DirectDriveCommandType::kRpm); // [RW] mode30 驱动轴命令类型：0=速度，1=电流，2=刹车。
                f32 direct_steer_command_value = 0.0f;                            // [RW] mode30 舵向轴当前活跃命令值。单位由命令类型决定。
                f32 direct_drive_command_value = 0.0f;                            // [RW] mode30 驱动轴当前活跃命令值。单位由命令类型决定。
                f32 direct_steer_command_limit = 250.0f;                          // [RW] mode30 舵向轴命令限幅。单位跟随当前舵向命令类型；类型切换后回到安全默认值。
                f32 direct_drive_command_limit = 1000.0f;                         // [RW] mode30 驱动轴命令限幅。单位跟随当前驱动命令类型；类型切换后回到安全默认值。
                f32 direct_steer_step_threshold = 0.3f;                           // [RW] mode30 舵向轴阶跃触发阈值。仅在 RcStep 时生效。
                f32 direct_drive_step_threshold = 0.3f;                           // [RW] mode30 驱动轴阶跃触发阈值。仅在 RcStep 时生效。
                f32 direct_steer_step_value = 200.0f;                             // [RW] mode30 舵向轴阶跃幅值。单位跟随当前舵向命令类型；类型切换后回到默认值。
                f32 direct_drive_step_value = 200.0f;                             // [RW] mode30 驱动轴阶跃幅值。单位跟随当前驱动命令类型；类型切换后回到默认值。
            } debug_control_;
            bool debug_enable_last_cycle_ = false; // [RO] 调试使能上周期状态。常用于检测 enable 上升沿，并在那一刻同步调试参数基线。
            u8 direct_last_steer_command_type_raw_ = 0xFFU; // [RO] 上次已同步的 mode30 舵向命令类型。用于识别“真正发生了类型切换”。
            u8 direct_last_drive_command_type_raw_ = 0xFFU; // [RO] 上次已同步的 mode30 驱动命令类型。用于识别“真正发生了类型切换”。

            // =====================================================================
            // 调试输出 [RW]
            // 说明：这里只管“串口往外发什么”，不管底盘怎么跑。
            //       output_enable 是总开关，output_mode_raw 选路径，text.log_level 决定文本模式的细度。
            // =====================================================================
            struct DebugOutputSlotConfig
            {
                u32 period_ms = 10U; // [RW] 当前输出槽位的最小发送周期（ms）。达到周期门限后才允许本模式尝试发送。
            };

            struct DebugOutputTextConfig
            {
                u32 period_ms = 500U; // [RW] mode1 文本日志周期（ms）。控制文本总刷新频率。
                u8 log_level = 1U;    // [RW] mode1 文本日志等级。0 只发基础汇总，>=1 会轮流输出更细的 FS/FSW/FSH 分相信息。
            };

            struct DebugOutputTelemetryConfig
            {
                u8 sample_divider = 1U; // [RW] mode5 分频发送系数。1 表示每个满足周期门限的控制周期都尝试发送。
                u8 profile_id = 0U;     // [RW] mode5 配置档编号。编码到 flags 高 4 位，便于 PC 侧区分不同发送方案。
                u32 period_ms = 10U;    // [RW] mode5 最小发送周期（ms）。
            };

            // 调试输出配置 [RW]
            // 说明：这里只放“外部可调”的输出参数，不再混入节流时间戳、计数器等运行态噪音。
            struct DebugOutputConfig
            {
                bool output_enable = false; // [RW] 串口输出总开关。false 时所有调试串口输出都停止，但控制逻辑仍继续运行。
                u8 output_mode_raw = static_cast<u8>(DebugOutputMode::kYawPidJustFloat); // [RW] 输出模式选择器：0=关，1=文本日志，2=四轮总览 justfloat，3=单轮高速 justfloat，4=单轮双电机 justfloat，5=SwerveTelemetryV2（二进制），6=YawPid justfloat。

                DebugOutputTextConfig text{}; // [RW] mode1 文本日志配置。
                DebugOutputSlotConfig overview = {5U}; // [RW] mode2 四轮总览 justfloat 配置。
                DebugOutputSlotConfig single_wheel = {1U}; // [RW] mode3 单轮高速 justfloat 配置；输出轮固定跟随 observe_wheel_index。
                DebugOutputSlotConfig dual_motor = {2U}; // [RW] mode4 单轮双电机高速 justfloat 配置；输出轮固定跟随 observe_wheel_index。
                DebugOutputSlotConfig yaw_pid = {4U}; // [RW] mode6 Yaw PID justfloat 配置。
                DebugOutputTelemetryConfig telemetry{}; // [RW] mode5 SwerveTelemetryV2 配置。
            } debug_output_;

            struct DebugOutputSlotRuntime
            {
                TickType_t last_ms = 0U; // [RO] 当前输出槽位最近一次成功发送时刻。用于节流判断。
            };

            struct DebugOutputTextRuntime
            {
                TickType_t last_ms = 0U;      // [RO] mode1 文本总日志最近一次发送时刻。
                u8 log_phase = 0U;            // [RO] mode1 文本分相输出索引。0=FS 总览，1=FSW 单轮细节，2=FSH 回零/对位细节。
                TickType_t direct_trace_last_ms = 0U; // [RO] mode1 执行层文本跟踪最近一次发送时刻。用于 SW30 限频。
            };

            struct DebugOutputTelemetryRuntime
            {
                TickType_t last_ms = 0U;   // [RO] mode5 最近一次成功发送时刻（tick/ms 基准），用于周期门限判断。
                u8 cycle_counter = 0U;     // [RO] mode5 分频计数器。与 sample_divider 配合决定本周期是否允许发送。
                u16 seq = 0U;              // [RO] mode5 帧序号。每成功发送一帧递增，便于 PC 侧检测丢帧。
            };

            // 调试输出运行态 [RO]
            // 说明：这里只放节流、相位和发送计数等运行时观察值，不参与外部调参。
            struct DebugOutputRuntime
            {
                DebugOutputTextRuntime text{};
                DebugOutputSlotRuntime overview{};
                DebugOutputSlotRuntime single_wheel{};
                DebugOutputSlotRuntime dual_motor{};
                DebugOutputSlotRuntime yaw_pid{};
                DebugOutputTelemetryRuntime telemetry{};
            } debug_output_runtime_;

            // =====================================================================
            // DebugPidTune [RW]
            // 说明：这里存的是“待同步的 PID 配置缓存”，不是运行态实时对象。
            //       写完后通常还要等调试使能边沿或同步流程消费，运行中的 PID 才会真正换参数。
            // =====================================================================
            struct DebugPidTune
            {
                PID_Param_Config steer_speed_pid_cfg[4] = {
                    // [RW] 四轮舵向速度环参数缓存。这里只是待同步配置，不会立刻改动正在运行的 PID。
                    {.kp = 32.0f, .ki = 0.085f, .kd = 0.0f, .I_Outlimit = 8000.0f, .isIOutlimit = true, .output_limit = 12000.0f, .deadband = 0.5f},
                    {.kp = 32.0f, .ki = 0.085f, .kd = 0.0f, .I_Outlimit = 8000.0f, .isIOutlimit = true, .output_limit = 12000.0f, .deadband = 0.5f},
                    {.kp = 32.0f, .ki = 0.085f, .kd = 0.0f, .I_Outlimit = 8000.0f, .isIOutlimit = true, .output_limit = 12000.0f, .deadband = 0.5f},
                    {.kp = 32.0f, .ki = 0.085f, .kd = 0.0f, .I_Outlimit = 8000.0f, .isIOutlimit = true, .output_limit = 12000.0f, .deadband = 0.5f},
                };
                PID_Param_Config steer_angle_pid_cfg[4] = {
                    // [RW] 四轮舵向角度环参数缓存。修改后同样要经过同步流程才会进入运行态。
                    {.kp = 3.5f, .ki = 0.0f, .kd = 0.05f, .I_Outlimit = 0.0f, .isIOutlimit = true, .output_limit = 500.0f, .deadband = 0.03f},
                    {.kp = 3.5f, .ki = 0.0f, .kd = 0.05f, .I_Outlimit = 0.0f, .isIOutlimit = true, .output_limit = 500.0f, .deadband = 0.03f},
                    {.kp = 3.5f, .ki = 0.0f, .kd = 0.05f, .I_Outlimit = 0.0f, .isIOutlimit = true, .output_limit = 500.0f, .deadband = 0.03f},
                    {.kp = 3.5f, .ki = 0.0f, .kd = 0.05f, .I_Outlimit = 0.0f, .isIOutlimit = true, .output_limit = 500.0f, .deadband = 0.03f},
                };
                f32 steer_speed_pid_td_ratio[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [RW] 速度环 TD 比例参数。属于扩展调参项，通常和速度环整定一起看。
                f32 steer_angle_pid_i_separa[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [RW] 角度环积分分离参数。用于决定误差多大时才允许积分参与。
                u32 steer_speed_pid_apply_stamp[4] = {0U, 0U, 0U, 0U};      // [RW] 速度环参数申请生效戳。外部写入后，通过同步流程消费。
                u32 steer_angle_pid_apply_stamp[4] = {0U, 0U, 0U, 0U};      // [RW] 角度环参数申请生效戳。外部写入后，通过同步流程消费。
                u32 steer_speed_pid_applied_stamp[4] = {0U, 0U, 0U, 0U};    // [RO] 速度环已生效戳。表示运行态已经真正接收到这组参数。
                u32 steer_angle_pid_applied_stamp[4] = {0U, 0U, 0U, 0U};    // [RO] 角度环已生效戳。表示运行态已经真正接收到这组参数。
                bool synced_on_enable_edge = false;                         // [RO] 本次调试使能上升沿是否已完成同步。避免重复把缓存参数刷入运行态。
            } debug_pid_tune_;

            // 回零与模块运行态（主要观察）[RO]
            struct YawPidTraceState
            {
                f32 mode_tag = 0.0f;
                f32 target_yaw_rad = 0.0f;
                f32 feedback_yaw_rad = 0.0f;
                f32 error_deg = 0.0f;
                f32 manual_omega_in_rad_s = 0.0f;
                f32 pid_output_omega_rad_s = 0.0f;
                f32 final_omega_cmd_rad_s = 0.0f;
                f32 feedback_yaw_rate_rad_s = 0.0f;
                f32 shift_remaining_ms = 0.0f;
                f32 pid_compute_fired = 0.0f;
                f32 steer_fault_any_active = 0.0f;
                f32 all_homed = 0.0f;
                f32 high_speed_suppression_active = 0.0f;
                f32 reverse_intent_active = 0.0f;
            } yaw_pid_trace_;

            bool homing_start_request_ = false;                                // [RW] 回零启动请求锁存位（由外部触发，在线程内消费）
            f32 homing_align_to_zero_tolerance_deg_ = 2.0f;                    // [RW] 回零归位判稳阈值（deg）
            WheelConfig wheel_config_[4];                                      // [RO] 四个模块运行态快照
            f32 last_steer_rate_cmd_rad_s_[4] = {0.0f};                        // [RO] 上周期转向速度命令
            f32 last_drive_omega_cmd_rad_s_[4] = {0.0f};                       // [RO] 上周期最终实际下发到驱动闭环的角速度命令
            bool selected_flipped_solution_[4] = {false};                      // [RO] 每个模块是否选中翻转解
            f32 low_speed_drive_suppression_scale_[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // [RO] 每轮低速抑制最终缩放。
            f32 high_speed_drive_suppression_scale_ = 1.0f;                        // [RO] 当前高速抑制缩放。
            bool high_speed_trans_gate_active_ = false;                            // [RO] 当前高速抑制速度门是否打开。复用 near-zero enter/exit 做滞回。
            bool high_speed_drive_suppression_active_ = false;                     // [RO] 当前高速抑制是否激活。
            f32 high_speed_dir_err_deg_ = 0.0f;                                    // [RO] 当前合成平移方向误差（deg）。
            f32 high_speed_eta_max_s_ = 0.0f;                                      // [RO] 当前四轮最大预计到角时间（s）。
            f32 max_residual_speed_m_s_ = 0.0f;                                // [RO] 当前拍四轮中的最大实际残余速度（m/s）。
            bool low_speed_residual_bypass_active_ = false;                        // [RO] 当前低速抑制残余速度旁路门是否打开。复用 near-zero enter/exit 做滞回。
            bool low_speed_drive_suppression_bypassed_by_residual_speed_ = false; // [RO] 当前拍低速抑制是否因残余速度阈值被旁路。
            u8 rot_z_pid_count_ = 0;                                           // [RO] 航向 PID 分频计数器
            f32 lock_now_rot_z_target_ = 0.0f;                                 // [RO] LockNow 真正维持的航向目标
            u32 lock_now_rot_z_shift_count_ = 0;                               // [RO] LockNow 松手缓冲倒计时
            bool xpark_gate_active_ = false;                                   // [RO] X-Park 进入门控当前是否放行。true 时允许静止姿态切到 X-Park。
            u32 xpark_stationary_hold_ms_ = 0U;                                // [RO] 连续静止累计时长（ms）。用于判断是否达到 X-Park 延时门槛。
            bool launch_hold_active_ = false;                                  // [RO] 静止起步整车等待门控是否激活。激活时先只转舵，不放驱动与车体速度规划。
            bool trans_dir_freeze_active_ = false;                              // [RO] 平移方向冻结门控当前状态。true 时方向保持参考角，只放行速度模长变化。
            bool trans_dir_ref_valid_ = false;                                  // [RO] 平移方向参考角是否有效。无效时先用当前指令方向建立参考。
            f32 trans_dir_ref_rad_ = 0.0f;                                      // [RO] 平移方向参考角（rad）。用于冻结保持与方向角速率限幅。
            f32 trans_dir_tar_mag_m_s_ = 0.0f;                                  // [RO] 平移输入目标速度模长缓存（m/s）。
            f32 trans_dir_out_mag_m_s_ = 0.0f;                                  // [RO] 平移规划输出速度模长缓存（m/s）。
            u8 trans_dir_freeze_reason_ = 0U;                                    // [RO] 冻结原因缓存：0=none,1=enter,2=hold。
            bool reverse_intent_active_ = false;                                 // [RO] 当前是否判定为近似反向意图。
            f32 reverse_intent_dir_err_deg_ = 0.0f;                              // [RO] 当前目标方向与参考方向夹角（deg）。
            bool steer_fault_any_active_ = false;

            // 控制链路缓存（观察）[RO]
            ManualSpeedProfileMode active_manual_speed_profile_mode_ = ManualSpeedProfileMode::kLegacy;
            JerkLimitedAxisState manual_vel_x_shape_state_{};
            JerkLimitedAxisState manual_vel_y_shape_state_{};
            JerkLimitedAxisState manual_omega_z_shape_state_{};
            InputTargetData input_target_data_; // [RO] 输入目标快照（模式与期望速度/角度）
            NormalizedBodyCommand normalized_body_command_; // [RO] 输入来源与统一车体系语义
            Data target_data_;                  // [RO] 模式映射后的目标数据
            Data planned_data_;                 // [RO] 经限幅/策略处理后的规划数据
            Data last_planned_data_;            // [RO] 上一周期规划数据（用于加速度约束）
            Data current_data_;                 // [RO] 当前状态估计数据
            SwervePlannerOutput planner_output_cache_; // [RO] 最近一次舵轮规划输出
            ActuatorCommandFrame actuator_command_frame_; // [RO] 最近一次规划出的执行器目标帧（drive 仍是执行门控前目标）
            ModeFlag current_mode_flag_;        // [RO] 当前控制模式标志位

            // 传感器与输入缓存（观察）[RO]
            f32 input_hwt_rot_z_ = 0.0f;   // [RO] IMU yaw
            f32 input_hwt_omega_z_ = 0.0f; // [RO] IMU yaw speed
            RmPocketData_t airjoy_data_{}; // [RO] 遥控器输入快照

            // 调试镜像（只读观察）[RO]
            struct DebugMirror
            {
                bool all_homed = false;                                             // [RO] 四轮是否全部回零完成
                f32 current_oa_deg[4] = {0.0f};                                     // [RO] 各轮当前 OA 角（deg）
                f32 target_oa_deg[4] = {0.0f};                                      // [RO] 各轮目标 OA 角（deg）
                f32 current_drive_rpm[4] = {0.0f};                                  // [RO] 各轮当前驱动速度（rpm）
                f32 target_drive_rpm[4] = {0.0f};                                   // [RO] 各轮目标驱动速度（rpm）
                f32 planned_drive_target_rpm[4] = {0.0f};                           // [RO] planner/gate 阶段计算出的驱动目标（rpm）
                f32 delivered_drive_target_rpm[4] = {0.0f};                         // [RO] 最终执行层限幅并下发的驱动目标（rpm）
                u8 homing_state[4] = {0, 0, 0, 0};                                  // [RO] 各轮回零状态机状态
                bool homing_sensor_active[4] = {false, false, false, false};        // [RO] 各轮光电门有效状态
                bool homing_last_edge_is_falling[4] = {false, false, false, false}; // [RO] 各轮最近边沿是否下降沿
                f32 homing_runtime_zero_offset_deg[4] = {0.0f};                     // [RO] 各轮运行时零偏（deg）
                f32 nz_stationary_m_s = 0.0f;                                       // [RO] 当前有效静止阈值（m/s）。
                f32 nz_freeze_enter_m_s = 0.0f;                                     // [RO] 当前有效冻结进入阈值（m/s）。
                f32 nz_freeze_exit_m_s = 0.0f;                                      // [RO] 当前有效冻结退出阈值（m/s）。
                f32 nz_xpark_enter_m_s = 0.0f;                                      // [RO] 当前有效 X-Park 进入阈值（m/s）。
                f32 nz_xpark_exit_m_s = 0.0f;                                       // [RO] 当前有效 X-Park 退出阈值（m/s）。
                bool lim_drive_omega = true;                                        // [RO] 驱动角速度限幅是否开启。
                bool lim_drive_alpha = true;                                        // [RO] 驱动角加速度限幅是否开启。
                bool lim_steer_rate = true;                                         // [RO] 舵向角速度限幅是否开启。
                bool lim_steer_alpha = true;                                        // [RO] 舵向角加速度限幅是否开启。
                f32 high_speed_drive_suppression_scale = 1.0f;                      // [RO] 当前高速抑制缩放。
                f32 high_speed_dir_err_deg = 0.0f;                                  // [RO] 高速抑制使用的合成平移方向误差（deg）。
                f32 high_speed_eta_max_s = 0.0f;                                    // [RO] 高速抑制使用的四轮最大预计到角时间（s）。
                bool high_speed_drive_suppression_active = false;                   // [RO] 当前高速抑制是否激活。
                bool low_speed_drive_suppression_bypassed_by_residual_speed = false; // [RO] 当前拍是否因为残余速度过高而旁路了低速抑制。
                f32 max_residual_speed_m_s = 0.0f;                                  // [RO] 当前拍整车四轮中的最大实际残余速度（m/s）。
                bool reverse_intent_active = false;
                f32 reverse_intent_dir_err_deg = 0.0f;
                bool steer_fault_active[4] = {false, false, false, false};
                bool steer_fault_recovering[4] = {false, false, false, false};
                bool steer_fault_control_intent[4] = {false, false, false, false};
                bool steer_fault_xpark_stationary_hold[4] = {false, false, false, false};
                bool steer_fault_freeze_candidate[4] = {false, false, false, false};
                f32 steer_feedback_current_mA[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                f32 steer_feedback_current_delta_mA[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                f32 steer_feedback_angle_delta_rad[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                f32 steer_fault_steer_error_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                f32 steer_feedback_current_freeze_ms[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                f32 steer_feedback_recovery_toggle_count[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                f32 steer_fault_latched_count[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                bool steer_fault_any_active = false;
            } debug_mirror_;

            // 线程执行耗时统计（调试器只读观察）[RO]
            struct TaskPerfStat
            {
                struct WindowState
                {
                    u16 samples_us[500] = {0U}; // [RO] 短窗样本环形缓冲（内部状态）
                    u16 index = 0U;             // [RO] 下一次写入位置
                    u16 count = 0U;             // [RO] 当前有效样本数（<=500）
                    u32 sum_us = 0U;            // [RO] 当前窗口样本和（用于 O(1) 平均）
                    u64 clamp_count = 0ULL;     // [RO] 样本被 u16 饱和截断次数（内部累计）
                } window;

                u64 last_exec_us = 0ULL;       // [RO] 最近一次循环执行耗时（不含 delay）
                u64 min_exec_us = 0ULL;        // [RO] 历史最小执行耗时
                u64 max_exec_us = 0ULL;        // [RO] 历史最大执行耗时
                u64 avg_exec_us = 0ULL;        // [RO] 最近窗口平均执行耗时（短窗）
                u64 loop_count = 0ULL;         // [RO] 已统计循环次数
                u64 overrun_count = 0ULL;      // [RO] 超预算次数（exec_us > budget_us）
                u64 last_start_us = 0ULL;      // [RO] 最近一次循环开始时间戳
                u64 last_end_us = 0ULL;        // [RO] 最近一次循环结束时间戳
                u32 budget_us = 1000U;         // [RO] 单周期预算（us，当前 period_ms_=1）
                u16 window_size = 500U;        // [RO] 短窗长度（循环次数）
                u16 window_count = 0U;         // [RO] 当前窗口有效样本数（<=window_size）
                u64 window_clamp_count = 0ULL; // [RO] 样本被 u16 饱和截断次数
            } task_perf_stat_;

            // 调试串口对象（一般不在调试器改动）[RO]
            Debug_Printf debug_uart_ = Debug_Printf(&huart8); // [RO]
        };

        using Result = jia::FourSteerChassis::Chassis::Result;

        inline Result Chassis::setZeroCurrent()
        {
            input_target_data_.zero_current_all = true;
            return Result::kOk;
        }

        inline Result Chassis::setSpeed(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z)
        {
            const BodyCommand body_command = mapExternalCommandToBody({coord, vel_x, vel_y, omega_z});
            return (coord == Coordinate::kBody) ? setTargetBodySpeedMode(body_command.vel_x, body_command.vel_y, body_command.omega_z)
                                                : setTargetWorldSpeedMode(body_command.vel_x, body_command.vel_y, body_command.omega_z);
        }

        inline Result Chassis::setSpeed_LockNowYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z)
        {
            const BodyCommand body_command = mapExternalCommandToBody({coord, vel_x, vel_y, omega_z});
            return (coord == Coordinate::kBody) ? setTargetBodySpeedLockNowRotZWithNoOmegaZMode(body_command.vel_x, body_command.vel_y, body_command.omega_z)
                                                : setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(body_command.vel_x, body_command.vel_y, body_command.omega_z);
        }

        inline Result Chassis::setSpeed_LockToYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 rot_z)
        {
            const BodyCommand body_command = mapExternalCommandToBody({coord, vel_x, vel_y, 0.0f});
            return (coord == Coordinate::kBody) ? setTargetBodySpeedLockToRotZMode(body_command.vel_x, body_command.vel_y, rot_z)
                                                : setTargetWorldSpeedLockToRotZMode(body_command.vel_x, body_command.vel_y, rot_z);
        }

        inline Chassis::BodyCommand Chassis::mapExternalCommandToBody(const ExternalCommand &command)
        {
            BodyCommand body_command;
            body_command.vel_x = command.vel_y;
            body_command.vel_y = -command.vel_x;
            body_command.omega_z = command.omega_z;
            return body_command;
        }

        inline Chassis::BodyCommand Chassis::normalizeBodyCommandForPlanner(const BodyCommand &command)
        {
            BodyCommand planner_command;
            planner_command.vel_x = -command.vel_x;
            planner_command.vel_y = -command.vel_y;
            planner_command.omega_z = command.omega_z;
            return planner_command;
        }

        inline f32 Chassis::mapRawSteerMotorTotalToSignedLocalTotal(f32 raw_motor_total_rad, f32 steer_motor_sign)
        {
            const f32 steer_sign = (steer_motor_sign == 0.0f) ? 1.0f : steer_motor_sign;
            return raw_motor_total_rad * steer_sign;
        }

        inline f32 Chassis::mapSignedLocalTotalToRawSteerMotorTotal(f32 signed_local_total_rad, f32 steer_motor_sign)
        {
            const f32 steer_sign = (steer_motor_sign == 0.0f) ? 1.0f : steer_motor_sign;
            return signed_local_total_rad / steer_sign;
        }

        inline f32 Chassis::applyHomingRuntimeZeroOffset(f32 signed_local_total_rad, f32 homing_runtime_zero_offset_rad)
        {
            return signed_local_total_rad + homing_runtime_zero_offset_rad;
        }

        inline f32 Chassis::removeHomingRuntimeZeroOffset(f32 corrected_local_total_rad, f32 homing_runtime_zero_offset_rad)
        {
            return corrected_local_total_rad - homing_runtime_zero_offset_rad;
        }

        inline f32 Chassis::mapOaTotalToCorrectedLocalTotal(f32 oa_total_rad, const SteerCalibration &calibration)
        {
            return oa_total_rad - calibration.theta_oa_to_owi_rad;
        }

        inline f32 Chassis::mapCorrectedLocalTotalToOaTotal(f32 corrected_local_total_rad, const SteerCalibration &calibration)
        {
            return corrected_local_total_rad + calibration.theta_oa_to_owi_rad;
        }

        inline f32 Chassis::mapRawSteerMotorTotalToCorrectedLocalTotal(f32 raw_motor_total_rad, const SteerCalibration &calibration)
        {
            const f32 signed_local_total_rad = mapRawSteerMotorTotalToSignedLocalTotal(raw_motor_total_rad, calibration.steer_motor_sign);
            return applyHomingRuntimeZeroOffset(signed_local_total_rad, calibration.homing_runtime_zero_offset_rad);
        }

        inline f32 Chassis::mapCorrectedLocalTotalToRawSteerMotorTotal(f32 corrected_local_total_rad, const SteerCalibration &calibration)
        {
            const f32 signed_local_total_rad = removeHomingRuntimeZeroOffset(corrected_local_total_rad, calibration.homing_runtime_zero_offset_rad);
            return mapSignedLocalTotalToRawSteerMotorTotal(signed_local_total_rad, calibration.steer_motor_sign);
        }

        inline f32 Chassis::mapDriveMotorRpmToWheelOmega(f32 motor_rpm, const SteerCalibration &calibration)
        {
            const f32 drive_sign = (calibration.drive_motor_sign == 0.0f) ? 1.0f : calibration.drive_motor_sign;
            return drive_sign * rpmToRadsF32(motor_rpm);
        }

        inline f32 Chassis::mapWheelOmegaToDriveMotorRpm(f32 wheel_omega_rad_s, const SteerCalibration &calibration)
        {
            const f32 drive_sign = (calibration.drive_motor_sign == 0.0f) ? 1.0f : calibration.drive_motor_sign;
            return radsToRpmF32(wheel_omega_rad_s / drive_sign);
        }

        inline f32 Chassis::mapWheelCurrentToDriveMotorCurrent(f32 wheel_current_mA, const SteerCalibration &calibration)
        {
            const f32 drive_sign = (calibration.drive_motor_sign == 0.0f) ? 1.0f : calibration.drive_motor_sign;
            return wheel_current_mA / drive_sign;
        }

        inline f32 Chassis::computeHomingRuntimeZeroOffset(f32 edge_mech_oa_rad,
                                                           f32 raw_motor_total_rad,
                                                           f32 homing_zero_offset_rad,
                                                           const SteerCalibration &calibration)
        {
            const f32 edge_local_corrected_rad = mapOaTotalToCorrectedLocalTotal(edge_mech_oa_rad, calibration);
            const f32 edge_local_signed_rad = edge_local_corrected_rad + homing_zero_offset_rad;
            return edge_local_signed_rad - mapRawSteerMotorTotalToSignedLocalTotal(raw_motor_total_rad, calibration.steer_motor_sign);
        }

        inline Chassis::NormalizedBodyCommand Chassis::makeNormalizedBodyCommand(const PlannerInputCommand &command,
                                                                                 f32 input_yaw_rad,
                                                                                 CommandInputSource source)
        {
            NormalizedBodyCommand normalized{};
            normalized.source = source;
            normalized.rot_z = command.rot_z;
            normalized.is_world_speed_mode = command.is_world_speed_mode;
            normalized.is_steer_only_mode = command.is_steer_only_mode;

            f32 body_vel_x = command.vel_x;
            f32 body_vel_y = command.vel_y;
            if (command.is_world_speed_mode)
            {
                const f32 cos_theta = cosf(input_yaw_rad);
                const f32 sin_theta = sinf(input_yaw_rad);
                body_vel_x = command.vel_x * cos_theta + command.vel_y * sin_theta;
                body_vel_y = -command.vel_x * sin_theta + command.vel_y * cos_theta;
            }

            normalized.body = normalizeBodyCommandForPlanner({body_vel_x, body_vel_y, command.omega_z});
            if (command.is_steer_only_mode)
            {
                normalized.body.vel_x = 0.0f;
                normalized.body.vel_y = 0.0f;
                normalized.body.omega_z = 0.0f;
            }
            return normalized;
        }

        inline Chassis::DebugControlRoute Chassis::classifyDebugControlRoute(bool debug_enable, u8 raw_mode)
        {
            if (!debug_enable)
            {
                return DebugControlRoute::kDisabled;
            }

            switch (raw_mode)
            {
            case 21:
            case 22:
            case 30:
                return DebugControlRoute::kModuleOverride;
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            default:
                return DebugControlRoute::kTargetInjection;
            }
        }

        inline Chassis::DebugModuleOverrideRoute Chassis::classifyDebugModuleOverrideRoute(u8 raw_mode)
        {
            switch (raw_mode)
            {
            case 21:
                return DebugModuleOverrideRoute::kAlignForward;
            case 22:
                return DebugModuleOverrideRoute::kHomingObserve;
            case 30:
                return DebugModuleOverrideRoute::kDirectActuator;
            default:
                return DebugModuleOverrideRoute::kNone;
            }
        }

        inline Chassis::PlannerInputSnapshot Chassis::makePlannerInputSnapshot(const PlannerInputCommand &command, f32 input_yaw_rad)
        {
            PlannerInputSnapshot snapshot{};
            const NormalizedBodyCommand normalized = makeNormalizedBodyCommand(command, input_yaw_rad, CommandInputSource::kApi);
            snapshot.target.vel_x = normalized.body.vel_x;
            snapshot.target.vel_y = normalized.body.vel_y;
            snapshot.target.omega_z = normalized.body.omega_z;
            snapshot.target.rot_z = normalized.rot_z;

            return snapshot;
        }

        inline Chassis::TelemetrySnapshot Chassis::makeTelemetrySnapshot(bool homing_all_ready,
                                                                         const TelemetryChassisState &target,
                                                                         const TelemetryChassisState &actual,
                                                                         const TelemetryWheelPose wheel_pose[kTelemetryWheelCount],
                                                                         const f32 target_drive_omega_rad_s[kTelemetryWheelCount],
                                                                         const f32 actual_drive_omega_rad_s[kTelemetryWheelCount],
                                                                         const f32 target_steer_oa_rad[kTelemetryWheelCount],
                                                                         const f32 actual_steer_oa_rad[kTelemetryWheelCount])
        {
            TelemetrySnapshot snapshot{};
            snapshot.homing_all_ready = homing_all_ready;
            snapshot.target = target;
            snapshot.actual = actual;

            for (u8 i = 0U; i < kTelemetryWheelCount; ++i)
            {
                TelemetryWheelState &wheel = snapshot.wheels[i];
                wheel.target_drive_omega_rad_s = target_drive_omega_rad_s[i];
                wheel.actual_drive_omega_rad_s = actual_drive_omega_rad_s[i];
                wheel.target_steer_oa_rad = target_steer_oa_rad[i];
                wheel.actual_steer_oa_rad = actual_steer_oa_rad[i];
                wheel.target_velocity_x_m_s = target.vel_x + target.omega_z * wheel_pose[i].pos_y_m;
                wheel.target_velocity_y_m_s = target.vel_y - target.omega_z * wheel_pose[i].pos_x_m;
                wheel.actual_velocity_x_m_s = actual.vel_x + actual.omega_z * wheel_pose[i].pos_y_m;
                wheel.actual_velocity_y_m_s = actual.vel_y - actual.omega_z * wheel_pose[i].pos_x_m;
            }

            return snapshot;
        }

        inline Chassis::SteerCalibration Chassis::makeSteerCalibration(const WheelConfig &wheel)
        {
            SteerCalibration calibration;
            calibration.theta_oa_to_owi_rad = wheel.theta_oa_to_owi_rad;
            calibration.homing_runtime_zero_offset_rad = wheel.homing_runtime_zero_offset_rad;
            calibration.steer_motor_sign = wheel.steer_motor_sign;
            calibration.drive_motor_sign = wheel.drive_motor_sign;
            return calibration;
        }

        inline f32 Chassis::mapWheelCorrectedLocalToOaTotal(const WheelConfig &wheel, f32 corrected_local_total_rad)
        {
            return mapCorrectedLocalTotalToOaTotal(corrected_local_total_rad, makeSteerCalibration(wheel));
        }

        inline f32 Chassis::mapWheelOaTotalToCorrectedLocal(const WheelConfig &wheel, f32 oa_total_rad)
        {
            return mapOaTotalToCorrectedLocalTotal(oa_total_rad, makeSteerCalibration(wheel));
        }

        inline Robot_Twist Chassis::getBodySpeed() const
        {
            Robot_Twist body_speed;
            const f32 internal_vx = getTargetBodyVelX();
            const f32 internal_vy = getTargetBodyVelY();
            body_speed.vx = -internal_vy;
            body_speed.vy = internal_vx;
            body_speed.vz = getTargetOmegaZ();
            return body_speed;
        }

        inline Robot_Twist Chassis::getWorldSpeed() const
        {
            Robot_Twist world_speed;
            const f32 internal_vx = getTargetWorldVelX();
            const f32 internal_vy = getTargetWorldVelY();
            world_speed.vx = -internal_vy;
            world_speed.vy = internal_vx;
            world_speed.vz = getTargetOmegaZ();
            return world_speed;
        }
    }
}

#define JIA_USE_FOUR_STEER_CHASSIS 1

#if JIA_USE_FOUR_STEER_CHASSIS
using jia::FourSteerChassis::Chassis;
#endif

#endif // CHASSIS_H_
