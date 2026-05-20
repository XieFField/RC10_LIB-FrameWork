#ifndef CHASSIS_H_
#define CHASSIS_H_

#include "APP_Utils.h"

#include "FreeRTOS.h"

#include "Motor_DJI.h"
#include "Module_CrsfReceiver.h"
#include "APP_debugTool.h"
#include "APP_PID.h"

#ifndef FOURSTEER_SINGLE_WHEEL_TRACE_UART8
#define FOURSTEER_SINGLE_WHEEL_TRACE_UART8 1
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
                kSingleWheel = 1,
                kAlignForward = 2,
                kHomingObserve = 3,
                kDirectActuator = 4,
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

            // 内部策略/状态类型：不再作为正式外部 API 暴露
            enum class DriveGateStrategy : u8
            {
                kHardGate = 0,
                kSoftGate = 1,
                kContinuousCurve = 2,
                kAdaptiveGate = 3,
            };
            enum class DriveGateScope : u8
            {
                kGlobal = 0,
                kPerWheel = 1,
            };
            enum class StopSteerGuardStrategy : u8
            {
                kHardHold = 0,
                kSoftBlend = 1,
                kContinuousBlend = 2,
            };
            enum class AdaptiveGatePhase : u8
            {
                kIdle = 0,
                kStartHold = 1,
                kTransition = 2,
                kContinuous = 3,
                kLegacy = 4,
                kDisabled = 5,
            };
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
                bool homing_zero_valid = false;                   // 当前轮是否已经建立可用于闭环控制的零位
                f32 homing_elapsed_s = 0.0f;                      // 本次回零已运行时间，单位秒；用于超时判定
                f32 homing_runtime_zero_offset_rad = 0.0f;        // 本次上电运行实际采用的零位补偿；回零成功后会把“当前触发位置”修正成运行时零点
                f32 corrected_steer_motor_total_angle_rad = 0.0f; // 已乘方向符号并叠加运行时零位补偿后的转向电机连续总角度反馈
                f32 corrected_drive_omega_rad_s = 0.0f;           // 已乘方向符号后的驱动轮角速度反馈，单位 rad/s
                f32 target_steer_motor_total_angle_rad = 0.0f;    // 当前周期解算后要发给转向电机的本地连续目标角
                f32 target_drive_omega_rad_s = 0.0f;              // 当前周期解算后要发给驱动电机的目标角速度，单位 rad/s
                f32 steer_target_velocity_rad_s = 0.0f;           // 转向二阶限幅后得到的目标角速度，便于平滑舵向变化
                bool flipped_drive_direction = false;             // 本周期是否采用“舵角翻转 180 度、驱动反向”策略来走更短转角路径
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
                kSingleWheel = 20,
                kAlignForward = 21,
                kHomingObserve = 22,
                kDirectActuator = 30,
            };
            enum class DebugOutputMode : u8
            {
                kOff = 0,
                kText = 1,
                kOverviewJustFloat = 2,
                kSingleWheelJustFloat = 3,
                kSingleWheelDualMotorJustFloat = 4,
                kSwerveTelemetryV2 = 5,
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
            void applySingleWheelDebugOverride(u8 wheel_idx, bool all_homed);
            void applyAlignForwardDebugOverride();
            void applyHomingObserveDebugOverride();
            void applyDirectActuatorDebugOverride(u8 wheel_idx);
            void finalizeDebugModuleOverride(bool all_homed, DebugModuleOverrideRoute route);
            void transSpeedBodyToWorld(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y) const;
            void transSpeedWorldToBody(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y) const;
            void isLockNowRotZ(bool is_lock, f32 rot_z, f32 omega_z, f32 &out_rot_z, f32 &out_omega_z);
            void isLockToRotZ(bool is_lock, f32 tar_rot_z, f32 cur_rot_z, f32 &out_rot_z, f32 omega_z, f32 &out_omega_z);
            void clampTargetSpeedInChassis(f32 vel_x, f32 vel_y, f32 omega_z, f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z) const;
            void limitPlannedSpeed(f32 tar_vel_x, f32 tar_vel_y, f32 tar_omega_z, f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z);
            void updateWheelFeedback();
            bool updateHomingState(WheelConfig &wheel);
            bool readHomingSensor(const WheelConfig &wheel) const;
            bool readHomingSensorRawHigh(const WheelConfig &wheel) const;
            f32 readSteerMotorRawTotalAngleRad(const WheelConfig &wheel) const;
            f32 readDriveMotorOmegaRadS(const WheelConfig &wheel) const;
            f32 readCorrectedSteerMotorTotalAngleRad(const WheelConfig &wheel) const;
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
            f32 computeDriveGateScale(f32 abs_error_rad) const;
            void computeDriveGateScales(const f32 steering_errors_rad[4], const Data &command_data, f32 out_scales[4]);
            f32 stopSteerGuardBlend(f32 residual_speed_m_s) const;
            void deriveNearZeroThresholds();
            void refreshActuatorLimitState();
            f32 mapSingleTurnToNearestTotalAngle(const WheelConfig &wheel, f32 target_oa_single_turn_deg) const;
            void computeProjectedDriveFromPlannedSteer(const Data &command_data, const f32 planned_oa_total_rad[4], f32 out_drive_omega_rad_s[4]) const;
            bool estimatePlannedBodyTwist(const f32 planned_oa_total_rad[4], const f32 planned_drive_omega_rad_s[4], f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z) const;
            f32 updateVectorConsistencyGate(f32 translational_speed_m_s, f32 eta_max_s, f32 dir_err_deg);
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
            // 底盘参数（运行时可调）[RW]
            // 说明：FourSteer 初始化后会把这里作为默认基线读取。
            // 这一组决定“车能跑多快、加减速有多柔和、停下来时保持什么姿态”。
            // =====================================================================
            f32 wheel_radius_m_ = 0.052f;                                    // [RW, 慎改] 轮半径。决定线速度与驱动角速度的换算比例，改错会直接导致速度尺度和里程计比例偏差。
            f32 max_vel_x_ = 4.0f;                                         // [RW] 车体 X 方向最大线速度上限（m/s）。用于规划/限幅，不是电机硬件极限。
            f32 max_vel_y_ = 4.0f;                                         // [RW] 车体 Y 方向最大线速度上限（m/s）。同上，约束横移速度。
            f32 max_omega_z_ = 4.0f;                                       // [RW] 车体 Z 轴最大角速度上限（rad/s）。同上，约束原地旋转或航向变化速度。
            f32 max_acc_xy_acc_ = 5.0f;                                   // [RW] 平面加速段最大加速度（m/s^2）。越小起步越柔和，越大响应越猛。
            f32 max_acc_xy_dec_ = 10.0f;                                   // [RW] 平面减速段最大减速度（m/s^2）。越小刹车越平滑，越大停车越快但冲击更强。
            f32 max_alpha_z_acc_ = 5.0f;                                  // [RW] 航向加速段最大角加速度（rad/s^2）。影响转向起步的平顺性。
            f32 max_alpha_z_dec_ = 10.0f;                                  // [RW] 航向减速段最大角减速度（rad/s^2）。影响转向收尾和停摆冲击。
            f32 max_drive_omega_rad_s_ = 60.0f;                              // [RW] 驱动目标角速度上限（rad/s）。仅在 enable_drive_omega_limit=true 时生效。
            f32 max_drive_alpha_rad_s2_ = 1000.0f;                           // [RW] 驱动角速度变化率上限（rad/s^2）。仅在 enable_drive_alpha_limit=true 时生效。
            f32 max_steer_rate_rad_s_ = 10.0f;                               // [RW] 转向目标角速度上限（rad/s）。仅在 enable_steer_rate_limit=true 时生效。
            f32 max_steer_alpha_rad_s2_ = 10000.0f;                            // [RW] 转向目标角加速度上限（rad/s^2）。仅在 enable_steer_alpha_limit=true 时生效。
            f32 trans_dir_rate_limit_deg_s_ = 540.0f;                        // [RW] 平移速度矢量方向变化率上限（deg/s）。限制“速度方向”每秒最多转多少度。
            bool enable_cosine_compensation_ = false;                        // [RW] 是否启用舵角余弦补偿。开启后，舵角偏离时会折算驱动分量，减小横滑和无效驱动。
            IdlePostureMode idle_posture_mode_ = IdlePostureMode::kXPark; // [RW] 静止姿态策略。决定停住后是维持当前轮姿态，还是自动收拢为 X-Park。

            // =====================================================================
            // 近零门限统一配置（运行时可调）[RW]
            // 说明：所有静止/冻结/X-Park/停车保护阈值统一由基准参数派生，避免多处手改失配。
            // =====================================================================
            struct NearZeroThresholdConfig
            {
                f32 base_enter_m_s = 0.10f;       // [RW] 近零门限进入基准（m/s）。
                f32 base_exit_m_s = 0.14f;        // [RW] 近零门限退出基准（m/s）。应大于 enter 形成滞回。
                u32 xpark_entry_delay_ms = 500U;  // [RW] X-Park 进入最短静止持续时间（ms）。
                f32 stop_guard_release_scale = 1.0f; // [RW] 停车保护释放阈值缩放系数。stop_guard_release = base_enter * scale。
            } near_zero_cfg_;

            struct DerivedNearZeroThresholds
            {
                f32 stationary_m_s = 0.10f;          // [RO] 有效静止阈值（m/s）。
                f32 freeze_enter_m_s = 0.10f;        // [RO] 平移方向冻结进入阈值（m/s）。
                f32 freeze_exit_m_s = 0.14f;         // [RO] 平移方向冻结退出阈值（m/s）。
                f32 xpark_enter_m_s = 0.10f;         // [RO] X-Park 静止判定进入阈值（m/s）。
                f32 xpark_exit_m_s = 0.14f;          // [RO] X-Park 静止判定退出阈值（m/s）。
                f32 stop_guard_release_m_s = 0.10f; // [RO] 停车保护释放阈值（m/s）。
            } near_zero_derived_;

            // =====================================================================
            // 执行器限幅开关（运行时可调）[RW]
            // 说明：false 表示该层限幅被显式旁路，不再需要用超大数值“近似禁用”。
            // =====================================================================
            struct ActuatorLimitEnable
            {
                bool enable_drive_omega_limit = false; // [RW] 是否启用驱动角速度上限。
                bool enable_drive_alpha_limit = false; // [RW] 是否启用驱动角加速度上限。
                bool enable_steer_rate_limit = false;  // [RW] 是否启用舵向角速度上限。
                bool enable_steer_alpha_limit = true; // [RW] 是否启用舵向角加速度上限。
            } actuator_limit_enable_;

            // =====================================================================
            // 策略参数（运行时可调）[RW]
            // 说明：这里控制“怎么解算”“什么时候翻转”“什么时候压驱动”“停车时怎么稳住”。
            // =====================================================================
            struct StrategyConfig
            {
                // ---- 舵角解算 ----------------------------------------------------
                // 决定每个模块在“直接转过去”与“翻转 180° 再配合驱动反向”之间如何选择。
                // 这个选择直接影响转向路径长度、驱动方向是否反转，以及模块在大角度切换时是否抖动。
                SteeringStrategyMode steering_strategy_mode = SteeringStrategyMode::kShortestPath; // [RW] 舵角解算策略。不同策略在转向路径和稳定性上有不同权衡。
                f32 flip_enter_angle_deg = 135.0f; // [RW] 翻转保持上阈值（deg）。当前已在翻转解时，只有翻转解角差超过该阈值才退出翻转。
                f32 flip_exit_angle_deg = 80.0f;   // [RW] 翻转切入下阈值（deg）。当前未翻转时，直达解角差足够大且翻转解更优才切入翻转。应小于 flip_enter_angle_deg 形成滞回。

                // ---- 驱动抑制（Drive Gate） -------------------------------------
                // 当舵角还没对准时，按策略压低驱动输出，减少横滑、打滑和轮子“边转边拖”的冲击。
                // 适合大角度转向、起步前对轮、低速细调等场景。
                bool enable_drive_gate = false;                                       // [RW] 是否启用驱动门控。关闭后驱动不再因舵角误差被额外压低。
                DriveGateStrategy drive_gate_strategy = DriveGateStrategy::kHardGate; // [RW] 门控形状：硬门控更直接，曲线门控更平滑。
                DriveGateScope drive_gate_scope = DriveGateScope::kGlobal;            // [RW] 门控作用范围：全局统一收紧，或按单轮分别计算。
                f32 drive_gate_close_angle_deg = 1.0f;                                // [RW] 关闭区角差阈值（deg）。超过该误差后会进入更强的驱动抑制。
                f32 drive_gate_min_scale = 0.5f;                                      // [RW] 硬门控最小缩放。0 表示可完全关断驱动，1 表示完全不压。
                f32 drive_gate_curve_exponent = 3.0f;                                 // [RW] 曲线门控指数。越大曲线越“硬”，越接近临界开关。
                f32 drive_gate_curve_half_angle_deg = 3.0f;                           // [RW] 曲线门控半效角（deg）。大致决定从“明显抑制”到“基本放开”的过渡宽度。
                f32 drive_gate_curve_min_scale = 0.0f;                                // [RW] 曲线门控保底缩放。即使误差很大，也至少保留多少驱动比例。
                f32 drive_gate_transition_linear_speed_m_s = 0.10f;                   // [RW] 线速度过渡阈值（m/s）。速度越低，门控越容易收紧，减少静止附近的横向冲击。
                f32 drive_gate_transition_angular_speed_rad_s = 0.10f;                // [RW] 角速度过渡阈值（rad/s）。自转越慢，门控越偏向保守。
                f32 drive_gate_scale_ramp_up_s = 0.10f;                               // [RW] 门控放开时间常数（s）。越小表示释放越快，越大表示更平滑。
                f32 drive_gate_scale_ramp_down_s = 0.50f;                             // [RW] 门控收紧时间常数（s）。越大表示收紧更慢，更不容易突然“掐断”驱动。

                // ---- 停车转向保护 ------------------------------------------------
                // 在低速或静止时抑制不必要的舵角摆动，避免轮子在接近停住时反复“找角”。
                // 它主要解决停车抖动、低速微调来回打舵、以及静止姿态不稳定的问题。
                bool enable_stop_steer_guard = false;                                                    // [RW] 是否启用停车转向保护。默认开启以抑制过零与低速区舵角抖动。
                StopSteerGuardStrategy stop_steer_guard_strategy = StopSteerGuardStrategy::kHardHold; // [RW] 停车保护形状：硬保持更稳，曲线混合更柔和。
                f32 stop_guard_blend_start_speed_m_s = 0.20f;                                         // [RW] 混合起点速度（m/s）。从正常舵角控制逐步过渡到停车保护的起始点。
                f32 stop_guard_curve_half_speed_m_s = 0.08f;                                          // [RW] 曲线混合半效速度（m/s）。决定混合函数中“过半”的速度位置。
                f32 stop_guard_curve_exponent = 2.0f;                                                 // [RW] 曲线混合指数。越大过渡越陡，越小过渡越平缓。

                // ---- 全局矢量一致性门控 -------------------------------------------
                struct VectorConsistencyConfig
                {
                    bool enable = true;                     // [RW] 是否启用全局矢量一致性门控。
                    f32 dir_err_enter_deg = 12.0f;         // [RW] 方向误差进入阈值（deg）。
                    f32 dir_err_exit_deg = 6.0f;           // [RW] 方向误差退出阈值（deg），应小于 enter 形成滞回。
                    f32 eta_lock_s = 0.20f;                // [RW] 最大到角时间进入阈值（s）。
                    f32 eta_release_s = 0.06f;             // [RW] 最大到角时间退出阈值（s），应小于 lock。
                    f32 gate_ramp_up_s = 0.08f;            // [RW] 门控放开时间常数（s）。
                    f32 gate_ramp_down_s = 0.03f;          // [RW] 门控收紧时间常数（s）。
                    f32 min_trans_speed_enable_m_s = 0.10f; // [RW] 启动门控的最小平移速度（m/s）。
                } vector_consistency;
            };
            StrategyConfig default_strategy_cfg_; // [RW, 慎改] 默认策略基线。用于初始化和“恢复默认值”，不要把它当作实时状态。
            StrategyConfig runtime_strategy_cfg_; // [RW] 当前生效的运行时策略。可被外部接口动态切换，控制链路实际读取它。

            // =====================================================================
            // 航向控制参数（运行时可调）[RW]
            // 说明：这组参数只影响航向锁定/锁角逻辑，不影响平移速度规划。
            // =====================================================================
            PID_Position rot_z_pid_;                  // [RW, 慎改] 航向位置环 PID。用于 LockToYaw / 相关锁角模式的角度误差闭环。
            u8 rot_z_pid_period_ = 1;                 // [RW] PID 更新周期分频。1 表示每个控制周期都更新，数值越大频率越低、负载越小但响应更慢。
            f32 max_lock_to_rot_z_rad_s_ = 2.0f;      // [RW] LockToYaw 模式下的角速度上限（rad/s）。用于限制“往目标角赶”的最快速度。
            u32 lock_now_rot_z_shift_time_ms_ = 1000; // [RW] LockNow 松手缓冲时长（ms）。松开后短时间内继续维持目标，避免姿态突然跳变。

            // =====================================================================
            // 调试参数（通过全局 chassis 对象在调试器内直接改值）[RW]
            // 说明：这组参数只影响调试链路。正常控制不读取它们，只有切到相应 debug mode 时才会生效。
            // 速查：0~8 = 底盘输入接管/信号注入类模式；20 = 单轮调试；21 = 四轮朝前；22 = 回零观察；30 = 执行层直控。
            // 手柄平移坐标约定（底盘层）：前推前进、左推左移（-left_y -> vel_x，left_x -> vel_y）。
            // =====================================================================
            struct DebugControl
            {
                // ---- 调试总开关与模式入口 ---------------------------------------
                bool enable = false;                                             // [RW] 调试总开关。false 时整个调试接管链路不生效，系统走正常控制。
                u8 mode_raw = 1;                                                // [RW] 调试模式号。决定当前是输入接管、单轮调试、回零观察还是执行层直控。
                u8 mode_resolved_raw = static_cast<u8>(DebugMode::kWorldSpeed); // [RO] 解析后的实际模式号。用于观察 mode_raw 经过归一化后的结果。
                u8 wheel_index = 1;                                             // [RW] 主选中轮号（0~3）。默认作为单轮调试与单轮输出追踪的统一轮号来源。

                // ---- 航向 / 激励注入 ---------------------------------------------
                f32 lock_rot_z = 0.0f;     // [RW] LockTo 调试目标角（rad）。只在航向锁定相关调试中有意义。
                bool inject_step = false;  // [RW] 是否注入阶跃信号。用于调试响应、阶跃跟踪或观察动态性能。
                bool inject_sine = false;  // [RW] 是否注入正弦信号。用于频响、跟踪误差和相位滞后观察。
                f32 sine_amplitude = 0.0f; // [RW] 正弦幅值。与注入开关配合使用，表示激励强度。
                f32 sine_frequency = 0.1f; // [RW] 正弦频率（Hz）。越高越能看出响应速度，越低越适合慢速观察。
                f32 sine_offset = 0.0f;    // [RW] 正弦偏置。把激励整体平移到某个基线附近使用。

                // ---- mode20：单轮调试 -------------------------------------------
                bool single_wheel_drive_enable = true;               // [RW] mode20 下是否允许驱动输出。关闭时只看舵向，不下发驱动。
                bool single_wheel_soft_steer_enable = false;         // [RW] mode20 下是否启用舵向软限幅轨迹。开启后不会一步跳到目标，而是按限速/限加速度渐进到位。
                bool single_wheel_use_custom_steer_limit = false;    // [RW] mode20 是否使用自定义舵向限速参数。关闭时用默认舵向约束。
                f32 single_wheel_steer_rate_limit_deg_s = 120.0f;    // [RW] mode20 舵向角速度上限（deg/s）。只在软舵向或限速轨迹下发挥作用。
                f32 single_wheel_steer_accel_limit_deg_s2 = 600.0f;  // [RW] mode20 舵向角加速度上限（deg/s^2）。限制舵向变化“猛不猛”。
                bool single_wheel_drive_release_gate_enable = false; // [RW] mode20 驱动释放门控。开启后，舵角没对准前会先压住驱动。
                f32 single_wheel_drive_release_error_deg = 5.0f;     // [RW] mode20 驱动释放角差阈值（deg）。舵角误差小于该值时才允许更积极地释放驱动。
                f32 single_wheel_target_steer_deg = 0.0f;            // [RW] mode20 单轮舵向目标 OA（deg）。单轮调试时直接给舵角目标。
                f32 single_wheel_target_drive_rpm = 0.0f;            // [RW] mode20 单轮驱动目标（rpm）。单轮调试时直接给驱动速度目标。

                // ---- mode30：执行层直控 -----------------------------------------
                bool direct_estop = false;                                       // [RW] mode30 急停闸门。true 时禁止所有执行层输出，适合调试前的“安全锁”。
                bool direct_enable_steer[4] = {true, true, true, true};    // [RW] mode30 每轮舵向执行使能。单独放开某一轮的舵向下发。
                bool direct_enable_drive[4] = {true, true, true, true};    // [RW] mode30 每轮驱动执行使能。单独放开某一轮的驱动下发。
                u8 direct_input_source = 1;                                     // [RW] mode30 输入来源：0=调试器缓存值，1=遥控连续输入（左舵向/右航向），2=遥控阶跃输入（左舵向/右航向）。
                u8 direct_steer_control_type = 1;                               // [RW] mode30 舵向控制方式：0=电流，1=速度，2=单圈角，3=多圈角。
                u8 direct_drive_control_type = 0;                               // [RW] mode30 航向控制方式：0=速度，1=电流，2=刹车。默认 0=RPM。
                f32 direct_steer_current_mA[4] = {0.0f, 0.0f, 0.0f, 0.0f};     // [RW] mode30 舵向电流目标缓存。作为直控输入的“待下发值”。
                f32 direct_steer_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};            // [RW] mode30 舵向速度目标缓存。作为直控输入的“待下发值”。
                f32 direct_steer_single_turn_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [RW] mode30 舵向单圈角目标缓存。作为直控输入的“待下发值”。
                f32 direct_steer_multi_turn_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [RW] mode30 舵向多圈角目标缓存。作为直控输入的“待下发值”。
                f32 direct_drive_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};            // [RW] mode30 航向速度目标缓存。作为直控输入的“待下发值”。
                f32 direct_drive_current_mA[4] = {0.0f, 0.0f, 0.0f, 0.0f};     // [RW] mode30 航向电流目标缓存。作为直控输入的“待下发值”。
                f32 direct_drive_brake_mA[4] = {0.0f, 0.0f, 0.0f, 0.0f};       // [RW] mode30 航向刹车目标缓存。作为直控输入的“待下发值”。
                f32 direct_drive_rpm_limit = 1000.0f;                            // [RW] mode30 航向速度限幅（rpm）。下发前的安全上限。
                f32 direct_drive_current_limit_mA = 12000.0f;                   // [RW] mode30 航向电流限幅（mA）。避免当前环指令过大。
                f32 direct_drive_brake_limit_mA = 12000.0f;                     // [RW] mode30 航向刹车限幅（mA）。避免刹车指令过大。
                f32 direct_steer_rpm_limit = 250.0f;                            // [RW] mode30 舵向速度限幅（rpm）。避免舵向指令过快。
                f32 direct_steer_current_limit_mA = 12000.0f;                   // [RW] mode30 舵向电流限幅（mA）。避免当前环指令过大。
                f32 direct_steer_single_turn_limit_deg = 180.0f;                // [RW] mode30 单圈角限幅（deg）。适合做单圈范围内的安全测试。
                f32 direct_steer_multi_turn_limit_deg = 1080.0f;                // [RW] mode30 多圈角限幅（deg）。限制多圈试验时的总角度范围。
                f32 direct_step_threshold = 0.3f;                               // [RW] 左摇杆舵向阶跃触发阈值。超过该幅值才认为用户想发出舵向阶跃动作。
                f32 direct_step_drive_threshold = 0.3f;                         // [RW] 右摇杆航向阶跃触发阈值。超过该幅值才认为用户想发出航向阶跃动作。
                f32 direct_step_steer_current_mA = 2000.0f;                     // [RW] 舵向阶跃幅值（电流，mA）。左摇杆触发后用于给舵向电流固定跃迁量。
                f32 direct_step_steer_rpm = 200.0f;                             // [RW] 舵向阶跃幅值（速度，rpm）。左摇杆触发后用于给舵向速度固定跃迁量。
                f32 direct_step_steer_single_turn_deg = 90.0f;                  // [RW] 舵向阶跃幅值（单圈角，deg）。左摇杆触发后用于给单圈角固定跃迁量。
                f32 direct_step_steer_multi_turn_deg = 180.0f;                  // [RW] 舵向阶跃幅值（多圈角，deg）。左摇杆触发后用于给多圈角固定跃迁量。
                f32 direct_step_drive_rpm = 200.0f;                             // [RW] 航向阶跃幅值（速度，rpm）。右摇杆触发后用于给航向速度固定跃迁量。
                f32 direct_step_drive_current_mA = 2000.0f;                     // [RW] 航向阶跃幅值（电流，mA）。右摇杆触发后用于给航向电流固定跃迁量。
                f32 direct_step_drive_brake_mA = 1500.0f;                       // [RW] 航向阶跃幅值（刹车，mA）。右摇杆触发后用于给航向刹车固定跃迁量。
            } debug_control_;
            bool debug_enable_last_cycle_ = false; // [RO] 调试使能上周期状态。常用于检测 enable 上升沿，并在那一刻同步调试参数基线。

            // =====================================================================
            // 调试输出 [RW]
            // 说明：这里只管“串口往外发什么”，不管底盘怎么跑。
            //       output_enable 是总开关，output_mode_raw 选路径，text_log_level 决定文本模式的细度。
            // =====================================================================
            struct DebugOutput
            {
                // ---- 输出总开关与模式选择 ---------------------------------------
                bool output_enable = true;                                    // [RW] 串口输出总开关。false 时所有调试串口输出都停止，但控制逻辑仍继续运行。
                u8 output_mode_raw = static_cast<u8>(DebugOutputMode::kSwerveTelemetryV2); // [RW] 输出模式选择器：0=关，1=文本日志，2=四轮总览 justfloat，3=单轮高速 justfloat，4=单轮双电机 justfloat，5=SwerveTelemetryV2（二进制）。
                u32 text_period_ms = 500;                                     // [RW] 文本日志周期（ms）。只在 mode1 下使用，控制文本总刷新频率。
                u8 text_log_level = 1;                                        // [RW] 文本日志等级。0 只发基础汇总，>=1 会轮流输出更细的 FS/FSW/FSH 分相信息。
                TickType_t text_last_ms = 0;                                  // [RO] 文本日志节流时间戳。记录上一次发文本的时间，防止串口刷屏。
                u8 text_log_phase = 0;                                        // [RO] 文本分相输出索引。0=FS 总览，1=FSW 单轮细节，2=FSH 回零/对位细节。

                // ---- mode2：四轮总览 justfloat ---------------------------------
                u32 overview_justfloat_period_ms = 5;      // [RW] mode2 四轮总览 justfloat 周期（ms）。控制每次发送完整四轮电机数据的频率。
                TickType_t overview_justfloat_last_ms = 0; // [RO] mode2 发送节流时间戳。防止总览数据过于频繁。

                // ---- mode3：单轮高速 justfloat ---------------------------------
                bool single_wheel_1khz_use_override_index = false; // [RW] mode3 是否使用私有轮号覆盖。false=跟随 DebugControl::wheel_index，true=使用 single_wheel_1khz_index。
                u8 single_wheel_1khz_index = 0;           // [RW] mode3 私有轮号（覆盖值）。仅当 single_wheel_1khz_use_override_index=true 时生效。
                u32 single_wheel_1khz_period_ms = 1;      // [RW] mode3 目标周期（ms）。一般设为 1ms，表示尽可能按控制周期输出。
                TickType_t single_wheel_1khz_last_ms = 0; // [RO] mode3 发送节流时间戳。记录高速输出最近一次发送时刻。

                // ---- mode4：单轮双电机高速 justfloat ----------------------------
                // 约定：这里把 drive_motor_h 作为“航向电机”源，并和 steer_motor_h 同帧输出。
                bool single_wheel_dual_motor_use_override_index = false; // [RW] mode4 是否使用私有轮号覆盖。false=跟随 DebugControl::wheel_index，true=使用 single_wheel_dual_motor_index。
                u8 single_wheel_dual_motor_index = 0;           // [RW] mode4 私有轮号（覆盖值）。仅当 single_wheel_dual_motor_use_override_index=true 时生效。
                u32 single_wheel_dual_motor_period_ms = 2;      // [RW] mode4 目标周期（ms）。默认 2ms，兼顾分辨率与串口稳定性。
                TickType_t single_wheel_dual_motor_last_ms = 0; // [RO] mode4 发送节流时间戳。记录双电机高速输出最近一次发送时刻。

                // ---- mode5: SwerveTelemetryV2 binary ----------------------------
                u8 telemetry_sample_divider = 1U;    // [RW] mode5 分频发送系数。1 表示每个满足周期门限的控制周期都尝试发送。
                u8 telemetry_profile_id = 0U;        // [RW] mode5 配置档编号。编码到 flags 高 4 位，便于 PC 侧区分不同发送方案。
                u32 telemetry_period_ms = 8U;        // [RW] mode5 最小发送周期（ms）。默认8ms，对应约125fps上限。
                TickType_t telemetry_last_ms = 0;    // [RO] mode5 最近一次发送时刻（tick/ms 基准），用于周期门限判断。
                u8 telemetry_cycle_counter = 0U;     // [RO] mode5 分频计数器。与 sample_divider 配合决定本周期是否允许发送。
                u16 telemetry_seq = 0U;              // [RO] mode5 帧序号。每成功发送一帧递增，便于 PC 侧检测丢帧。
                u8 telemetry_frame_buf[384] = {0U};  // [RO] mode5 DMA 发送缓冲区（header + payload + crc）。

                // ---- mode1：文本追踪节流 ----------------------------------------
                TickType_t single_wheel_trace_last_ms = 0; // [RO] 单轮文本跟踪节流时间戳。用于 mode1 下的单轮细节日志限频。
                TickType_t direct_trace_last_ms = 0;       // [RO] 执行层文本跟踪节流时间戳。用于 mode1 下的直控调试日志限频。
            } debug_output_;

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
            bool homing_start_request_ = false;                                // [RW] 回零启动请求锁存位（由外部触发，在线程内消费）
            f32 homing_align_to_zero_tolerance_deg_ = 2.0f;                    // [RW] 回零归位判稳阈值（deg）
            WheelConfig wheel_config_[4];                                      // [RO] 四个模块运行态快照
            f32 last_steer_rate_cmd_rad_s_[4] = {0.0f};                        // [RO] 上周期转向速度命令
            f32 last_drive_omega_cmd_rad_s_[4] = {0.0f};                       // [RO] 上周期驱动角速度命令
            bool selected_flipped_solution_[4] = {false};                      // [RO] 每个模块是否选中翻转解
            f32 drive_gate_scale_[4] = {1.0f, 1.0f, 1.0f, 1.0f};               // [RO] 每轮驱动门控缩放
            f32 adaptive_gate_scale_ = 1.0f;                                   // [RO] 全局自适应门控缩放
            AdaptiveGatePhase adaptive_gate_phase_ = AdaptiveGatePhase::kIdle; // [RO] 自适应门控阶段
            f32 vector_gate_scale_ = 1.0f;                                     // [RO] 全局矢量一致性门控缩放。
            bool vector_gate_active_ = false;                                  // [RO] 全局矢量一致性门控是否激活。
            f32 vector_dir_err_deg_ = 0.0f;                                    // [RO] 当前合成平移方向误差（deg）。
            f32 vector_eta_max_s_ = 0.0f;                                      // [RO] 当前四轮最大预计到角时间（s）。
            u8 rot_z_pid_count_ = 0;                                           // [RO] 航向 PID 分频计数器
            f32 lock_now_rot_z_target_ = 0.0f;                                 // [RO] LockNow 真正维持的航向目标
            u32 lock_now_rot_z_shift_count_ = 0;                               // [RO] LockNow 松手缓冲倒计时
            bool xpark_gate_active_ = false;                                   // [RO] X-Park 进入门控当前是否放行。true 时允许静止姿态切到 X-Park。
            u32 xpark_stationary_hold_ms_ = 0U;                                // [RO] 连续静止累计时长（ms）。用于判断是否达到 X-Park 延时门槛。
            bool trans_dir_freeze_active_ = false;                              // [RO] 平移方向冻结门控当前状态。true 时方向保持参考角，只放行速度模长变化。
            bool trans_dir_ref_valid_ = false;                                  // [RO] 平移方向参考角是否有效。无效时先用当前指令方向建立参考。
            f32 trans_dir_ref_rad_ = 0.0f;                                      // [RO] 平移方向参考角（rad）。用于冻结保持与方向角速率限幅。
            f32 trans_dir_tar_mag_m_s_ = 0.0f;                                  // [RO] 平移输入目标速度模长缓存（m/s）。
            f32 trans_dir_out_mag_m_s_ = 0.0f;                                  // [RO] 平移规划输出速度模长缓存（m/s）。
            u8 trans_dir_freeze_reason_ = 0U;                                    // [RO] 冻结原因缓存：0=none,1=enter,2=hold。

            // 控制链路缓存（观察）[RO]
            InputTargetData input_target_data_; // [RO] 输入目标快照（模式与期望速度/角度）
            Data target_data_;                  // [RO] 模式映射后的目标数据
            Data planned_data_;                 // [RO] 经限幅/策略处理后的规划数据
            Data last_planned_data_;            // [RO] 上一周期规划数据（用于加速度约束）
            Data current_data_;                 // [RO] 当前状态估计数据
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
                u8 homing_state[4] = {0, 0, 0, 0};                                  // [RO] 各轮回零状态机状态
                bool homing_sensor_active[4] = {false, false, false, false};        // [RO] 各轮光电门有效状态
                bool homing_last_edge_is_falling[4] = {false, false, false, false}; // [RO] 各轮最近边沿是否下降沿
                f32 homing_runtime_zero_offset_deg[4] = {0.0f};                     // [RO] 各轮运行时零偏（deg）
                f32 selected_wheel_steer_error_deg = 0.0f;                          // [RO] 选中轮舵向误差（deg）
                bool selected_wheel_drive_released = false;                         // [RO] 选中轮驱动是否已释放
                f32 nz_stationary_m_s = 0.0f;                                       // [RO] 当前有效静止阈值（m/s）。
                f32 nz_freeze_enter_m_s = 0.0f;                                     // [RO] 当前有效冻结进入阈值（m/s）。
                f32 nz_freeze_exit_m_s = 0.0f;                                      // [RO] 当前有效冻结退出阈值（m/s）。
                f32 nz_xpark_enter_m_s = 0.0f;                                      // [RO] 当前有效 X-Park 进入阈值（m/s）。
                f32 nz_xpark_exit_m_s = 0.0f;                                       // [RO] 当前有效 X-Park 退出阈值（m/s）。
                f32 nz_stop_guard_release_m_s = 0.0f;                               // [RO] 当前有效停车保护释放阈值（m/s）。
                bool lim_drive_omega = true;                                        // [RO] 驱动角速度限幅是否开启。
                bool lim_drive_alpha = true;                                        // [RO] 驱动角加速度限幅是否开启。
                bool lim_steer_rate = true;                                         // [RO] 舵向角速度限幅是否开启。
                bool lim_steer_alpha = true;                                        // [RO] 舵向角加速度限幅是否开启。
                f32 vec_gate_scale = 1.0f;                                          // [RO] 全局矢量一致性门控当前缩放。
                f32 vec_dir_err_deg = 0.0f;                                         // [RO] 合成平移方向误差（deg）。
                f32 vec_eta_max_s = 0.0f;                                           // [RO] 四轮最大预计到角时间（s）。
                bool vec_gate_active = false;                                       // [RO] 全局矢量一致性门控当前是否激活。
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
            const f32 steer_sign = (calibration.steer_motor_sign == 0.0f) ? 1.0f : calibration.steer_motor_sign;
            return raw_motor_total_rad * steer_sign + calibration.homing_runtime_zero_offset_rad;
        }

        inline f32 Chassis::mapCorrectedLocalTotalToRawSteerMotorTotal(f32 corrected_local_total_rad, const SteerCalibration &calibration)
        {
            const f32 steer_sign = (calibration.steer_motor_sign == 0.0f) ? 1.0f : calibration.steer_motor_sign;
            return (corrected_local_total_rad - calibration.homing_runtime_zero_offset_rad) / steer_sign;
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
            return edge_local_corrected_rad + homing_zero_offset_rad - raw_motor_total_rad;
        }

        inline Chassis::DebugControlRoute Chassis::classifyDebugControlRoute(bool debug_enable, u8 raw_mode)
        {
            if (!debug_enable)
            {
                return DebugControlRoute::kDisabled;
            }

            switch (raw_mode)
            {
            case 20:
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
            case 20:
                return DebugModuleOverrideRoute::kSingleWheel;
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

            f32 body_vel_x = command.vel_x;
            f32 body_vel_y = command.vel_y;
            if (command.is_world_speed_mode)
            {
                const f32 cos_theta = cosf(input_yaw_rad);
                const f32 sin_theta = sinf(input_yaw_rad);
                body_vel_x = command.vel_x * cos_theta + command.vel_y * sin_theta;
                body_vel_y = -command.vel_x * sin_theta + command.vel_y * cos_theta;
            }

            const BodyCommand planner_command = normalizeBodyCommandForPlanner({body_vel_x, body_vel_y, command.omega_z});
            snapshot.target.vel_x = planner_command.vel_x;
            snapshot.target.vel_y = planner_command.vel_y;
            snapshot.target.omega_z = planner_command.omega_z;
            snapshot.target.rot_z = command.rot_z;

            if (command.is_steer_only_mode)
            {
                snapshot.target.vel_x = 0.0f;
                snapshot.target.vel_y = 0.0f;
                snapshot.target.omega_z = 0.0f;
            }

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
