/**
 * @file chassis.h
 * @author 桑叁年
 * @brief 四舵轮底盘控制类声明
 *
 * 这份头文件同时承担三层职责：
 * 1. 对外暴露底盘控制接口与常用观测类型，供上层模块按“速度 / 锁航向 / 调试接管”语义使用。
 * 2. 声明底盘内部关键运行时状态，帮助维护者理解 swerve 规划、回零、X-Park、故障门控如何配合工作。
 * 3. 在 FULL_DEBUG / RUNTIME_MIN 两个编译档位之间，为调试缓存和观测镜像提供清晰裁剪边界。
 *
 * 阅读建议：
 * - 先看 public 区的对外命令、telemetry 与规划归一化辅助类型；
 * - 再看 private 区的轮组运行态、策略配置、门控锁存与调试缓存；
 * - inline 小函数主要负责坐标系、符号和零偏映射，它们解释了“外部语义”和“内部执行语义”如何衔接。
 */

#ifndef CHASSIS_H_
#define CHASSIS_H_

// =====================================================================
// 底盘编译档位
// =====================================================================
// 使用说明：
// - RUNTIME_MIN：比赛/发布固件默认档。保留正常底盘控制、安全门控、homing、X-Park、
//   drive zero-stop、steer fault 和核心 swerve planner；编译期去掉串口观测、调试接管、
//   单轮直控、PID 调参缓存、调试镜像和线程耗时大缓存，优先降低固件体积与 Chassis RAM 占用。
// - FULL_DEBUG：调试器/host 语义测试档。保留所有调试字段和输出路径，方便直接观察内部状态。
// 如果工程文件或 host 测试需要指定档位，可在编译参数中定义：
//   -DJIA_CHASSIS_PROFILE=JIA_CHASSIS_PROFILE_FULL_DEBUG
//   -DJIA_CHASSIS_PROFILE=JIA_CHASSIS_PROFILE_RUNTIME_MIN
#define JIA_CHASSIS_PROFILE_RUNTIME_MIN 1
#define JIA_CHASSIS_PROFILE_FULL_DEBUG 2

#ifndef JIA_CHASSIS_PROFILE
#define JIA_CHASSIS_PROFILE JIA_CHASSIS_PROFILE_FULL_DEBUG
#endif

// 功能开关均允许外部 -D 单独覆盖。下面只给 profile 的默认值：
// FULL_DEBUG 全开；RUNTIME_MIN 关闭调试/观测/单轮直控类功能。
#ifndef JIA_CHASSIS_ENABLE_DEBUG_OVERRIDE
#define JIA_CHASSIS_ENABLE_DEBUG_OVERRIDE (JIA_CHASSIS_PROFILE == JIA_CHASSIS_PROFILE_FULL_DEBUG)
#endif

#ifndef JIA_CHASSIS_ENABLE_SINGLE_WHEEL_DEBUG
#define JIA_CHASSIS_ENABLE_SINGLE_WHEEL_DEBUG (JIA_CHASSIS_PROFILE == JIA_CHASSIS_PROFILE_FULL_DEBUG)
#endif

#ifndef JIA_CHASSIS_ENABLE_DEBUG_OUTPUT
#define JIA_CHASSIS_ENABLE_DEBUG_OUTPUT (JIA_CHASSIS_PROFILE == JIA_CHASSIS_PROFILE_FULL_DEBUG)
#endif

#ifndef JIA_CHASSIS_ENABLE_PID_TUNE_CACHE
#define JIA_CHASSIS_ENABLE_PID_TUNE_CACHE (JIA_CHASSIS_PROFILE == JIA_CHASSIS_PROFILE_FULL_DEBUG)
#endif

#ifndef JIA_CHASSIS_ENABLE_DEBUG_MIRROR
#define JIA_CHASSIS_ENABLE_DEBUG_MIRROR (JIA_CHASSIS_PROFILE == JIA_CHASSIS_PROFILE_FULL_DEBUG)
#endif

#ifndef JIA_CHASSIS_ENABLE_TASK_PERF_STAT
#define JIA_CHASSIS_ENABLE_TASK_PERF_STAT (JIA_CHASSIS_PROFILE == JIA_CHASSIS_PROFILE_FULL_DEBUG)
#endif

#ifndef JIA_CHASSIS_HOMING_SEARCH_RPM
#define JIA_CHASSIS_HOMING_SEARCH_RPM 60.0f
#endif

#ifndef JIA_CHASSIS_HOMING_EDGE_DELTA_TOLERANCE_DEG
#define JIA_CHASSIS_HOMING_EDGE_DELTA_TOLERANCE_DEG 2.0f
#endif

// “首次上电回零延时”只作用在本次上电后的第一次整车 homing：
// - 它不是每次 startHoming() 都会等待，首次机会一旦消耗，后续手动再次回零不再等待；
// - ENABLE 只控制这段逻辑是否参与编译，不改变其他 homing / recovery 分支的语义；
// - DELAY_MS 表示四个舵轮共享的统一等待时长，0U 等价于关闭等待、退化为当前立即进入 Search 的行为；
// - 等待窗口 active 期间，四个轮都停留在 kIdle，上游也不会放行到 kSearch，因此不会下发搜索 RPM。
#ifndef JIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_ENABLE
#define JIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_ENABLE 1
#endif

#ifndef JIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_MS
#define JIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_MS 0U
#endif

#include "APP_Utils.h"

#include "FreeRTOS.h"

#include "Motor_DJI.h"
#include "Motor_VESC.h"
#include "Module_lora.h"
#include "APP_debugTool.h"
#include "APP_PID.h"

namespace jia
{
    namespace FourSteerChassis
    {
        class Chassis
        {
        public:
            /* ----------------------------------------------------------------- */
            // 对外基础类型
            // 这一组定义的是调用方与底盘共享的“命令语义词汇”，
            // 外部先用这些类型表达意图，底盘内部再统一折算到 planner 能消费的车体系命令。
            enum class Result
            {
                kOk,
                kError,
            };

            // 对外坐标语义。
            // kBody 表示命令已经在底盘当前车体系下表达；
            // kWorld 表示命令在世界系下表达，底盘需要结合当前 yaw 折算回车体系。
            enum class Coordinate
            {
                kBody,
                kWorld,
            };

            // 对外速度命令格式。
            // 保留坐标系字段，是为了让上层不用关心底盘内部的 yaw 变换细节。
            struct ExternalCommand
            {
                Coordinate coord = Coordinate::kBody;
                f32 vel_x = 0.0f;
                f32 vel_y = 0.0f;
                f32 omega_z = 0.0f;
            };

            // 底盘内部统一使用的车体系速度命令。
            // 一旦转成 BodyCommand，后续限幅、yaw lock 和 swerve planner 都围绕这套语义继续工作。
            struct BodyCommand
            {
                f32 vel_x = 0.0f;
                f32 vel_y = 0.0f;
                f32 omega_z = 0.0f;
            };

            // 单轮几何/零偏/方向换算所需的最小标定集合。
            // 把它抽出来，是为了避免“安装偏置 + 回零零偏 + 电机方向符号”在多个 helper 里散落重复。
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
                u8 steer_input_axis = 0U;        // [RO] 舵向轴输入来源：left_x / left_y / right_x / right_y。
                u8 drive_input_axis = 0U;        // [RO] 驱动轴输入来源：left_x / left_y / right_x / right_y。
                u8 steer_command_type = 0U;      // [RO] 舵向轴命令类型快照：电流 / 速度 / 单圈角 / 多圈角。
                u8 drive_command_type = 0U;      // [RO] 驱动轴命令类型快照：速度 / 电流 / 刹车。
                u8 steer_planner_mode = 0U;      // [RO] 舵向轴规划模式快照：关闭 / S 曲线 / 梯形。
                u8 drive_planner_mode = 0U;      // [RO] 驱动轴规划模式快照：关闭 / S 曲线 / 梯形。
                bool steer_invert_input = false; // [RO] 舵向轴是否对摇杆输入取反。
                bool drive_invert_input = false; // [RO] 驱动轴是否对摇杆输入取反。
                bool steer_deadzone_applied = false; // [RO] 舵向轴本周期是否被共享死区压成 0。
                bool drive_deadzone_applied = false; // [RO] 驱动轴本周期是否被共享死区压成 0。
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

            /* ----------------------------------------------------------------- */
            /* ----------------------------------------------------------------- */
            // 规划归一化辅助类型
            // 这层位于“对外命令”与“内部 planner”之间，用来统一记录命令来源、坐标变换结果与调试接管路由。
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
                CommandInputSource source = CommandInputSource::kApi; // [RO] 命令来源标签。只描述“这条命令从哪来”，不代表后续一定由它主导最终执行。
                BodyCommand body{};                                  // [RO] 统一到车体系/世界系解释后的基础 body 命令。
                f32 rot_z = 0.0f;                                    // [RO] 与 body 命令配套的目标航向语义。
                bool is_world_speed_mode = false;                    // [RO] 该命令是否按世界系解释。
                bool is_steer_only_mode = false;                     // [RO] 该命令是否只驱动舵向而不请求常规 drive 运动。
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
                kSingleWheelIsolated = 3,
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

            /* ----------------------------------------------------------------- */
            // 生命周期与对外控制入口
            Chassis() = default;
            ~Chassis() = default;

            // 主控制接口：面向“整车应该怎么动”。
            // 调用者不需要知道模块解算和门控细节，底盘内部会继续处理这些工作。
            Result setZeroCurrent();
            // External/public frame convention for setSpeed*/get*:
            // +y points to the current 2/3 wheel-face side (forward), -x points to the current 3/4 wheel-face side (left),
            // so +x points to the current 1/2 wheel-face side and omega_z keeps the existing sign convention.
            Result setSpeed(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z);
            Result setSpeed_LockNowYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z = 0.0f);
            Result setSpeed_LockToYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 rot_z);
            // 注意：这两个 readback 返回的是“当前目标语义快照”，不是 current_data_ 的实际反馈速度。
            Robot_Twist getBodySpeed() const;
            Robot_Twist getWorldSpeed() const;
            Result setSteerDegAndDriveSpeed(f32 steer_angle_deg, f32 chassis_speed_m_s);

            // 兼容控制层接口：保留旧命名入口，但会落到同一套输入目标与 planner 链路中。
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
            // 纯函数辅助：集中承载坐标系、方向符号、零偏和 telemetry 相关换算。
            // 它们存在的意义是把“容易混的语义”固定在单处，而不是让各业务流程各自解释一遍。
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
            /* ----------------------------------------------------------------- */
            // 初始化与运行时策略切换
            // InitConfig 只负责把外部硬件句柄接进来，不承载几何、限幅或调试策略；
            // 真正决定“如何控制”的，是类内默认策略和运行时策略快照。
            struct InitConfig
            {
                Motor_Base *steer_motor_h[4] = {nullptr}; // 4 个转向电机句柄，顺序需与 wheels[4] 的轮位定义保持一致
                VESC_Motor *drive_motor_h[4] = {nullptr}; // 4 个驱动电机句柄，顺序需与对应转向模块一一匹配
            };

            // 初始化与运行时策略接口
            void init(InitConfig &config);
            void setIdlePostureMode(IdlePostureMode mode);
            void setSteeringStrategyMode(SteeringStrategyMode mode);



        private:
            /* ----------------------------------------------------------------- */
            // 生命周期与总流程控制
            // 这几项位于线程主循环入口附近，负责启动回零、判断底盘是否可进入正常控制，以及把运行时策略恢复到初始化基线。
            Result startHoming();
            bool isHomingDone() const;
            void resetRuntimeStrategyToInitConfig();

            /* ----------------------------------------------------------------- */
            // 内部关键状态类型
            // 这一组不是给外部调用的接口，而是帮助维护者理解控制线程当前所处阶段、故障状态以及轮组运行时结构。
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

            enum class XParkSteerHoldPhase : u8
            {
                kInactive = 0,
                kSettling = 1,
                kLatchedZeroCurrent = 2,
            };
            // 单轮初始化模板。
            // 它描述“装车与回零应当如何初始化”；真正随线程推进变化的状态保存在后面的 WheelConfig 里。
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
                f32 homing_search_rpm = JIA_CHASSIS_HOMING_SEARCH_RPM;
                f32 homing_zero_offset_deg = 0.0f;
                f32 homing_timeout_s = 5.0f;
            };

            // WheelConfig 是“单个舵轮模块在底盘层的完整运行态容器”：
            // - 它既保存装配常量和硬件句柄，也保存该轮在 homing / fault / hold / execute 链路中的实时状态。
            // - chassis 主循环里凡是只影响某一个轮子的决策，原则上都应该收敛到这里，避免跨数组维护导致语义分裂。
            // - 阅读时建议按“静态装配 -> 回零缓存 -> 本拍反馈/目标 -> X-Park hold -> steer fault 观测”五层去理解。
            struct WheelConfig
            {
                // ---- 静态装配与硬件绑定 -----------------------------------------
                f32 pos_x_m = 0.0f;                               // 该舵轮模块在车体坐标系中的 x 安装位置，单位米
                f32 pos_y_m = 0.0f;                               // 该舵轮模块在车体坐标系中的 y 安装位置，单位米
                f32 theta_oa_to_owi_rad = 0.0f;                   // 安装几何偏移：把舵轮在底盘平面内实际指向/滚动的方向（OA 朝向）换算到转向电机本地机械角参考系（OWI）；它用于坐标变换，不是回零补偿
                f32 steer_motor_sign = 1.0f;                      // 转向电机方向符号：1 表示不取反，-1 表示转向反馈和目标指令都按相反方向解释
                f32 drive_motor_sign = 1.0f;                      // 驱动电机方向符号：1 表示不取反，-1 表示驱动反馈和目标指令都按相反方向解释
                Motor_Base *steer_motor_h = nullptr;              // 该模块绑定的转向电机句柄
                VESC_Motor *drive_motor_h = nullptr;              // 该模块绑定的驱动电机句柄
                bool homing_enabled = false;                      // 是否对该轮启用回零流程；false 时默认认为零位已可用
                bool homing_sensor_active_high = true;            // 回零传感器逻辑 active 极性：true 表示高电平视为有效，false 表示低电平视为有效；不决定 H/L 边沿的机械角语义
                void *homing_gpio_port = nullptr;                 // 回零传感器 GPIO 端口运行时副本；读取零位输入时直接使用
                u16 homing_gpio_pin = 0;                          // 回零传感器 GPIO 引脚运行时副本；与端口配合读取真实输入

                // ---- 回零输入与边沿确认缓存 -------------------------------------
                // 这一组服务“三边沿确认 -> 运行时零偏建立 -> Search/Align 分拍收口”整条链路。
                // 它把一次上电期间看见的原始边沿、候选零偏和 ready 后保持目标都收进单轮局部状态里，
                // 供 resetHomingEdgeConfirmState() / recordHomingEdgeAndCheckConfirmed() / updateHomingState() 协同推进。
                f32 homing_falling_edge_mech_rad = 0.0f;          // 原始 GPIO H->L 边沿对应的机械 OA 角（rad）
                f32 homing_rising_edge_mech_rad = 0.0f;           // 原始 GPIO L->H 边沿对应的机械 OA 角（rad）
                f32 homing_search_rpm = JIA_CHASSIS_HOMING_SEARCH_RPM; // 回零搜索阶段给转向电机的转速指令，单位 rpm
                f32 homing_zero_offset_rad = 0.0f;                // 标定零偏：传感器触发点到期望机械零位的固定偏差。它是静态标定量，不等于本次上电求得的运行时零偏。
                f32 homing_timeout_s = 5.0f;                      // 单轮回零允许持续的最长时间，超时后进入故障态，单位秒
                HomingState homing_state = HomingState::kIdle;    // 当前轮回零状态机所处阶段
                bool homing_last_sensor_active = false;           // 上一控制周期的原始 GPIO 高低电平；用于检测 H/L 边沿
                bool homing_last_edge_is_falling = false;         // 最近一次抓到的边沿方向：true=H->L，false=L->H；方便调试极性和触发角
                bool homing_align_command_armed = false;          // AlignToZero 的节拍门：把“零位建立完成”和“开始下发第一条对零位置命令”拆到不同控制拍，避免抓边沿同拍大跳变。
                bool homing_zero_valid = false;                   // 当前轮是否已经建立可用于闭环控制的零位
                bool homing_search_timeout_armed = false;         // Search 超时是否已武装。只有看到首个有效舵向反馈活动后才开始累计超时。
                u8 homing_edge_confirm_count = 0U;                // 本次 Search 已连续确认的原始光电边沿数量
                bool homing_last_confirm_edge_is_falling = false; // 上一次确认边沿方向：true=H->L，false=L->H
                f32 homing_last_confirm_signed_local_rad = 0.0f;  // 上一次确认边沿对应的带方向本地连续角
                f32 homing_candidate_zero_offset_sum_rad = 0.0f;  // 三边沿确认时，已 unwrap 到同一分支的候选零偏累加
                f32 homing_hold_corrected_local_total_rad = 0.0f; // 单轮 ready 后维持的 corrected-local 连续角。它是本拍 hold 目标，不是零偏本身。
                f32 homing_elapsed_s = 0.0f;                      // 本次回零已运行时间，单位秒；用于超时判定
                f32 homing_runtime_zero_offset_rad = 0.0f;        // 本次上电运行实际采用的零位补偿。它由“当前 raw 触发位置 + 标定零偏”折算得出，和静态 homing_zero_offset_rad 不同。

                // ---- 实时反馈与本周期规划输出 -----------------------------------
                // 这里不是单纯“反馈区”，而是该轮本拍执行上下文：
                // corrected_* 由 updateWheelFeedback() 刷新，target_* / steer_target_velocity_rad_s 则先承接 planner 结果，
                // 再在 applyModuleCommands() / homing / zero-stop / fault 恢复等执行分支里被改写成最终执行镜像。
                f32 corrected_steer_motor_total_angle_rad = 0.0f; // 已乘方向符号并叠加运行时零位补偿后的转向电机连续总角度反馈
                f32 corrected_drive_omega_rad_s = 0.0f;           // 已乘方向符号后的驱动轮角速度反馈，单位 rad/s
                f32 target_steer_motor_total_angle_rad = 0.0f;    // 当前周期准备采用的转向本地连续目标角。storePlannedActuatorFrame() 先写入规划 steer command，applyModuleCommands() 后才稳定代表最终执行值。
                f32 target_drive_omega_rad_s = 0.0f;              // 当前周期准备采用的 drive 目标角速度，单位 rad/s。会先承载 planner 值，再在 zero-stop/homing/隔离等执行门控后改写为最终值。
                f32 steer_target_velocity_rad_s = 0.0f;           // 当前周期准备采用的转向目标角速度。既是 planner 二阶限幅输出，也是 hold/homing 等执行分支会覆写的目标速率容器。
                bool flipped_drive_direction = false;             // 当前周期采用的“舵角翻转 180 度、驱动反向”结果。它是本拍工作值，不等价于 selected_flipped_solution_ 那种跨拍锁存记忆。

                // ---- X-Park 舵向 hold 局部状态 ----------------------------------
                // 这组不是普通调试镜像，而是“单轮局部 hold 子状态机”的真实运行态。
                // 整车级 xpark_gate_active_ / xpark_stationary_hold_ms_ 决定何时允许进入这条链，
                // 真正每轮是否已判稳、是否切到零电流、多久后允许重新锁定，则由这里逐轮记录。
                XParkSteerHoldPhase xpark_steer_hold_phase = XParkSteerHoldPhase::kInactive; // [RO] 当前轮 X-Park 舵向 hold 状态机阶段。
                f32 xpark_steer_hold_locked_target_rad = 0.0f;      // [RO] 当前轮进入 hold 后冻结的 corrected-local 总角目标（rad）。
                f32 xpark_steer_hold_error_rad = 0.0f;              // [RO] 当前轮相对 X-Park 理想目标角的绝对角误差（rad）。
                f32 xpark_steer_hold_target_rate_rad_s = 0.0f;      // [RO] 当前轮 hold 判稳使用的目标角速度绝对值（rad/s）。
                u32 xpark_steer_hold_settle_ms = 0U;                // [RO] 当前轮满足 hold 判稳条件后已累计的保持时长（ms）。
                u32 xpark_steer_hold_reacquire_ms = 0U;             // [RO] 当前轮从零电流锁定退出后，重新允许锁定前还需等待的时长（ms）。

                // ---- 舵向故障观测与恢复计数 ------------------------------------
                // 该组实现“冻结观测 -> 锁故障 -> 等恢复跳动 -> 请求单轮 re-home”的完整局部链路。
                // 判据依赖单轮电流/角度微分，恢复动作也按轮推进，因此这些量收敛在 WheelConfig 内，
                // 并由 updateSteerFaultState() / latchSteerFault() / requestSingleWheelHoming() / clearSteerFaultState() 协同维护。
                SteerFaultState steer_fault_state = SteerFaultState::kNone; // [RO] steer freeze fault 恢复状态机阶段：None=正常检测，Latched=故障锁存等待恢复跳动，Recovering=单轮 re-home 中。
                bool steer_fault_rehome_request = false;           // [RO] recover 流程是否已请求这一个轮子重新回零。它会被 homing 状态机延后一拍消费，不代表本拍已进入 Search。
                f32 steer_feedback_current_mA = 0.0f;              // [RO] 当前周期读取到的舵向电流反馈（mA）。
                f32 steer_feedback_last_current_mA = 0.0f;         // [RO] 上一周期舵向电流反馈（mA）。用于计算冻结判据。
                f32 steer_feedback_last_raw_total_angle_rad = 0.0f; // [RO] 上一周期原始舵向连续角反馈（rad）。用于检测“有电流但不动”。
                f32 steer_feedback_current_delta_mA = 0.0f;        // [RO] 本周期与上一周期电流变化量绝对值（mA）。
                f32 steer_feedback_angle_delta_rad = 0.0f;         // [RO] 本周期与上一周期原始连续角变化量绝对值（rad）。
                bool steer_speed_pid_settled_active = false;       // [RO] 当前轮是否已进入“舵向到位判稳”状态；仅首次进入边沿触发一次速度环历史清理。
                f32 steer_speed_pid_settle_error_rad = 0.0f;       // [RO] 当前轮“最终下发舵向目标”与“当前反馈角”之间的绝对误差（rad）。
                f32 steer_speed_pid_settle_target_rate_rad_s = 0.0f; // [RO] 当前轮最终下发舵向目标角速度的绝对值（rad/s），用于判定是否已稳定收尾。
                f32 steer_fault_steer_error_rad = 0.0f;            // [RO] 当前目标舵角与反馈舵角的绝对误差（rad）。
                bool steer_fault_control_intent = false;           // [RO] 当前周期是否存在真正的 steer 控制意图。
                bool steer_fault_xpark_stationary_hold = false;    // [RO] 当前是否落在允许忽略故障判定的 X-Park 静止保持窗口。
                bool steer_fault_freeze_candidate = false;         // [RO] 本周期是否满足“有电流且角度/电流变化同时冻结”的可疑条件。
                u32 steer_feedback_freeze_ms = 0U;                 // [RO] 冻结候选已连续保持的时长（ms）。
                u32 steer_feedback_recovery_toggle_count = 0U;     // [RO] Latched 态下检测到的恢复跳动累计次数。达到阈值后才允许从 Latched 切入 Recovering。
                u32 steer_fault_latched_count = 0U;                // [RO] 该轮历史上累计锁故障次数。
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

            // Data 是“整车级状态/目标”的最小公共载体。
            // 输入解析、规划、当前估计都复用它，只是所在阶段不同。
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

            // SwervePlannerInput 是真正喂给模块解算器的展开版输入：
            // 除了整车命令，还会带上当前模块朝向、残余速度、统一朝向请求等上下文。
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
                f32 steer_intent_wheel_vx_m_s[4] = {0.0f};
                f32 steer_intent_wheel_vy_m_s[4] = {0.0f};
                f32 steer_intent_wheel_speed_m_s[4] = {0.0f};
                f32 residual_speed_m_s[4] = {0.0f};
                f32 max_command_wheel_speed_m_s = 0.0f;
                f32 max_steer_intent_wheel_speed_m_s = 0.0f;
                f32 max_residual_speed_m_s = 0.0f;
            };

            // SwervePlannerOutput 汇总的是“planner 这一拍想出来的所有关键中间量”：
            // 理想解、选中的翻转解、转向速率、抑制比例和最终 drive 目标都会放在这里。
            struct SwervePlannerOutput
            {
                f32 ideal_oa_total_rad[4] = {0.0f};                 // [RO] 纯运动学理想 OA 连续角，不含翻转保持、限速或前馈修正。
                f32 ideal_drive_omega_rad_s[4] = {0.0f};            // [RO] 与 ideal_oa_total_rad 配套的理想 drive 轮角速度。
                f32 selected_oa_total_rad[4] = {0.0f};              // [RO] 在直达解/翻转解之间做完策略选择后的 OA 连续角。
                f32 steering_errors_rad[4] = {0.0f};                // [RO] 当前反馈角到 selected_oa_total_rad 的误差；低速抑制与 launch-hold 主要看它。
                f32 planned_corrected_local_total_rad[4] = {0.0f};  // [RO] 二阶限速/限加速度后的“规划舵向轨迹点”，坐标域为 corrected local 连续角。
                f32 planned_oa_total_rad[4] = {0.0f};               // [RO] planned_corrected_local_total_rad 对应的 OA 连续角视图，便于调试与后续投影。
                f32 planned_steer_rate_rad_s[4] = {0.0f};           // [RO] 规划轨迹点对应的舵向目标角速度。
                f32 steer_cmd_corrected_local_total_rad[4] = {0.0f}; // [RO] 真正准备喂给舵向执行器的连续角命令，可能已叠加 feedforward lead。
                f32 steer_cmd_oa_total_rad[4] = {0.0f};             // [RO] steer_cmd_corrected_local_total_rad 的 OA 视图，帮助区分“规划点”和“最终舵向命令”。
                f32 projected_drive_omega_rad_s[4] = {0.0f};        // [RO] 沿 planned steer 投影后的 drive 预览值，还没经过高/低速抑制与最终执行裁决。
                f32 final_drive_omega_rad_s[4] = {0.0f};            // [RO] planner 视角的最终 drive 目标；执行层仍可因 zero-stop/homing 再次改写。
                f32 low_speed_suppression_scale[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // [RO] 每轮低速舵角未对齐时的 drive 压制比例。
                bool flipped_drive_direction[4] = {false, false, false, false}; // [RO] 当前拍是否选择了翻转解。它是“本拍决策结果”，不是跨拍保持锁存本身。
                f32 high_speed_suppression_scale = 1.0f;            // [RO] 当前拍全局高速抑制比例。会与低速抑制合并后再镜像给外部调试。
                bool high_speed_suppression_active = false;         // [RO] 当前拍是否触发高速抑制。
                f32 high_speed_dir_err_deg = 0.0f;                  // [RO] 当前合成平移方向误差（deg）。
                f32 high_speed_eta_max_s = 0.0f;                    // [RO] 当前四轮最大预计到角时间（s）。
                bool valid = false;                                 // [RO] 该拍 planner 输出是否有效，可否被 launch-hold 等缓存直接复用。
            };

            // ActuatorCommandFrame 是“已经准备好交给执行仲裁层”的一帧模块命令。
            // 它仍然是理想执行目标，真正能否下发以及是否被 zero-stop / homing 改写，要到 applyModuleCommands() 决定。
            struct ActuatorCommandFrame
            {
                f32 steer_corrected_local_total_rad[4] = {0.0f};     // [RO] 规划轨迹点对应的 corrected local 舵角。
                f32 steer_oa_total_rad[4] = {0.0f};                  // [RO] 规划轨迹点的 OA 视图，便于输出与调试核对。
                f32 steer_cmd_corrected_local_total_rad[4] = {0.0f}; // [RO] 执行层准备采用的最终舵向连续角命令。
                f32 steer_cmd_oa_total_rad[4] = {0.0f};              // [RO] 最终舵向命令的 OA 视图。
                f32 steer_rate_rad_s[4] = {0.0f};                    // [RO] 与最终舵向命令配套的目标转向速率。
                f32 drive_omega_rad_s[4] = {0.0f};                   // [RO] planner 视角的最终 drive 目标；尚未经过 homing/zero-stop/单轮隔离等执行门控。
                bool flipped_drive_direction[4] = {false, false, false, false}; // [RO] 当前帧的翻转解选择结果。
            };

            /* ----------------------------------------------------------------- */
            // 线程与输入解析辅助
            // 这一组负责把外部命令、调试接管和单轮直控统一折算成 planner 可消费的目标数据。
            static void createThread(void *arg);
            void runThread(void *arg);

            // 输入目标数据
#if JIA_CHASSIS_ENABLE_DEBUG_OVERRIDE
            void isDebugMode();
#endif
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
                kSteerDegAndDriveSpeed = 9,
                kAlignForward = 21,
                kHomingObserve = 22,
                kSingleWheelIsolated = 30,
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
            enum class SingleWheelInputAxis : u8
            {
                kLeftX = 0,
                kLeftY = 1,
                kRightX = 2,
                kRightY = 3,
            };
            enum class SingleWheelPlannerMode : u8
            {
                kOff = 0,
                kSCurve = 1,
                kTrapezoid = 2,
            };
            struct SingleWheelPlannerSCurveConfig
            {
                f32 acc_acc = 60.0f;
                f32 acc_dec = 60.0f;
                f32 jerk_acc = 250.0f;
                f32 jerk_dec = 250.0f;
                f32 settle_vel_eps = 1.0e-4f;
                f32 settle_acc_eps = 0.05f;
            };
            struct SingleWheelPlannerTrapezoidConfig
            {
                f32 acc = 60.0f;
                f32 dec = 60.0f;
            };
            struct SingleWheelAxisControl
            {
                bool enable = true;
                u8 input_mode_raw = static_cast<u8>(DirectAxisInputMode::kRcContinuous);
                u8 input_axis_raw = static_cast<u8>(SingleWheelInputAxis::kLeftX);
                bool invert_input = false;
                u8 command_type_raw = 0U;
                f32 command_value = 0.0f;
                f32 command_limit = 0.0f;
                f32 step_threshold = 0.5f;
                f32 step_value = 0.0f;
                u8 planner_mode_raw = static_cast<u8>(SingleWheelPlannerMode::kOff);
                SingleWheelPlannerSCurveConfig scurve{};
                SingleWheelPlannerTrapezoidConfig trapezoid{};
            };
            struct SingleWheelAxisPlannerRuntime
            {
                JerkLimitedAxisState jerk_state{};
                f32 last_output_value = 0.0f;
                u8 last_wheel_idx = 0xFFU;
                u8 last_command_type_raw = 0xFFU;
                u8 last_planner_mode_raw = 0xFFU;
                bool initialized = false;
            };
            enum class DebugOutputFamily : u8
            {
                kOff = 0,
                kText = 1,
                kJustFloat = 2,
            };
            enum class JustFloatProfile : u8
            {
                // kOverview (33ch, emitUart8VofaJustFloatPidTrace)
                // ch0: time_s
                // 轮 i (i=0..3) 的基址 = 1 + i*8:
                // ch(base+0): tar_current_mA
                // ch(base+1): cur_current_mA
                // ch(base+2): tar_rpm
                // ch(base+3): cur_rpm
                // ch(base+4): tar_single_turn_deg
                // ch(base+5): cur_single_turn_deg
                // ch(base+6): tar_total_turn_deg
                // ch(base+7): cur_total_turn_deg
                kOverview = 0,

                // kSingleWheelTrace (由 single_wheel_payload_raw 决定)
                // 1) kSteerOnly (9ch, emitUart8VofaPid1kHzTrace)
                // ch0: time_s
                // ch1: steer_tar_current_mA
                // ch2: steer_cur_current_mA
                // ch3: steer_tar_rpm
                // ch4: steer_cur_rpm
                // ch5: steer_tar_single_turn_deg
                // ch6: steer_cur_single_turn_deg
                // ch7: steer_tar_total_turn_deg
                // ch8: steer_cur_total_turn_deg
                // 2) kDriveOnly (9ch, emitUart8VofaSingleWheelDriveTrace)
                // ch0: time_s
                // ch1: drive_tar_current_mA
                // ch2: drive_cur_current_mA
                // ch3: drive_tar_rpm
                // ch4: drive_cur_rpm
                // ch5: drive_tar_single_turn_deg
                // ch6: drive_cur_single_turn_deg
                // ch7: drive_tar_total_turn_deg
                // ch8: drive_cur_total_turn_deg
                // 3) kSteerAndDrive (17ch, emitUart8VofaDualMotor1kHzTrace)
                // ch0: time_s
                // ch1~ch8:  steer 的 8 通道
                // ch9~ch16: drive 的 8 通道
                kSingleWheelTrace = 1,

                // kYawPid (15ch, emitUart8VofaYawPidTrace)
                // ch0:  time_s
                // ch1:  mode_tag
                // ch2:  target_yaw_rad
                // ch3:  feedback_yaw_rad
                // ch4:  error_deg
                // ch5:  manual_omega_in_rad_s
                // ch6:  pid_output_omega_rad_s
                // ch7:  final_omega_cmd_rad_s
                // ch8:  feedback_yaw_rate_rad_s
                // ch9:  shift_remaining_ms
                // ch10: pid_compute_fired
                // ch11: steer_fault_any_active
                // ch12: all_homed
                // ch13: high_speed_drive_suppression_active
                // ch14: reverse_intent_active
                kYawPid = 2,

                // kDriveZeroStopBrakeTrace (12ch, emitUart8VofaDriveZeroStopBrakeTrace)
                // ch0: time_s
                // ch1: observe_wheel_idx
                // ch2: target_rpm
                // ch3: feedback_rpm
                // ch4: zero_stop_brake_active
                // ch5: target_brake_current_mA
                // ch6: vesc_brake_command_active
                // ch7: feedback_current_mA
                // ch8: drive_zero_stop_active
                // ch9: residual_speed_m_s
                // ch10: target_command_speed_m_s
                // ch11: target_omega_z_rad_s
                kDriveZeroStopBrakeTrace = 4,
            };
            enum class SingleWheelTracePayloadKind : u8
            {
                kSteerOnly = 0,
                kSteerAndDrive = 1,
                kDriveOnly = 2,
            };
            DebugMode resolveDebugMode(u8 raw_mode) const;

            /* ----------------------------------------------------------------- */
            // 调试接管与模块 override
            // 与普通 setTarget* 不同，这里允许调试面板直接接管整车目标，或者绕开整车 planner 只改某些模块命令。
#if JIA_CHASSIS_ENABLE_DEBUG_OVERRIDE
            void applyDebugTargetOverride(DebugMode mode);
            // applyDebugModuleOverride() 是调试侧少数会短路正常 compute/apply 链路的入口。
            // 一旦它返回 true，本拍后续模块命令不再来自常规底盘规划，而由 debug route 直接决定。
            bool applyDebugModuleOverride(bool all_homed);
#endif
            /**
             * @brief 清空上一拍遗留的输入目标意图。
             * @details 把公开 set 接口、调试接管和上层命令留下来的目标缓存恢复为中性态，
             *          让本拍 resolve 阶段只基于最新输入重新解释控制语义。
             */
            void clearInputTargetData();
            // setModeFlag() 只做“公开 Mode -> 主流程布尔标签”的压平，
            // 它不是最终模式决策本身；更细的锁角/坐标/调试接管语义仍由后续 resolve 阶段继续解释。
            /**
             * @brief 将公开 Mode 压平成主流程常用的布尔标记。
             * @details 该步骤只负责把模式族归类为 torque-free / world / yaw-lock 这类粗粒度标签，
             *          真正的目标速度、目标朝向和调试覆盖仍由后续 resolve 阶段继续推导。
             */
            void setModeFlag();
            /**
             * @brief 统一解释外部输入意图，得到 planner 可消费的 target_data_。
             * @details 这里负责模式分流、坐标语义归一化、yaw lock 目标整理和调试接管入口，
             *          产物仍是“整车想怎么动”的目标，不直接生成任何单轮命令。
             */
            void resolvePlannerTargetData();
            /**
             * @brief 对 target_data_ 做跨拍整形，得到更平滑的 planned_data_。
             * @details 这一层主要处理 jerk/acc 限幅、方向冻结、速度门控和若干保持态衔接，
             *          目的是让后续模块解算看到的是时间连续、可执行的车体级目标。
             */
            void updatePlannedMotionData();
            /**
             * @brief 在模块 override 接管时清理常规规划残留。
             * @details 避免上一条常规底盘规划结果继续污染调试模块直控路径，让后续 apply/telemetry
             *          看到的 planned_data_ 与 debug route 保持一致。
             */
            void clearPlannedMotionForModuleOverride();
#if JIA_CHASSIS_ENABLE_DEBUG_OVERRIDE
            void resetDebugModuleOverrideTargets(u8 wheel_idx, bool preserve_soft_wheel_rate);
            // 这几个 helper 分别对应不同的 debug module route：
            // AlignForward / HomingObserve 主要服务校对与观察；
            // SingleWheelIsolated 则会真正接管某一轮的控制输出，并隔离其余轮。
            void applyAlignForwardDebugOverride();
            void applyHomingObserveDebugOverride();
            void finalizeDebugModuleOverride(bool all_homed, DebugModuleOverrideRoute route);
#if JIA_CHASSIS_ENABLE_SINGLE_WHEEL_DEBUG
            // mode30 单轮直控链路会先在局部构造“只属于目标轮”的命令真相，再由 filter/finalize 决定如何影响整车其余轮。
            void computeSingleWheelIsolatedCommandsMode30(u8 wheel_idx, bool all_homed = true);
            bool isSingleWheelIsolatedMode(DebugMode mode) const;
            void applySingleWheelIsolationFilter(DebugMode mode, u8 wheel_idx, bool all_homed);
            void syncSingleWheelCommandTemplates(); // mode30 类型切换同步入口。用于在命令类型改变时刷新单值命令、限幅和阶跃模板。
            DirectActuatorCommandSnapshot resolveSingleWheelCommand(u8 wheel_idx); // 解析当前 control_wheel_index 对应轮的 mode30 双轴有效命令快照。
#endif
#endif
            void clearDirectDriveCommandByType(WheelConfig &wheel, u8 wheel_idx, u8 drive_control_type);
#if JIA_CHASSIS_ENABLE_SINGLE_WHEEL_DEBUG
            void applyResolvedSteerCommand(WheelConfig &wheel, u8 wheel_idx, const DirectActuatorCommandSnapshot &command, bool enable);
            void applyResolvedDriveCommand(WheelConfig &wheel, u8 wheel_idx, const DirectActuatorCommandSnapshot &command, bool enable);
#endif

            /* ----------------------------------------------------------------- */
            // 运动学、航向锁定与限幅整形
            // 这部分把“输入意图”整形成“底盘可稳定执行的规划目标”，包括坐标转换、yaw lock、方向冻结和 jerk/acc 限幅。
            /**
             * @brief 将 drive 目标与调试虚拟负载/zero-stop 末端语义一起下发到单轮驱动电机。
             * @param wheel 目标轮的运行态容器。
             * @param wheel_idx 目标轮索引。
             * @param delivered_drive_target_rad_s 执行层最终允许下发的 drive 角速度目标。
             * @param single_wheel_isolation_active 当前是否处于单轮隔离模式。
             * @param single_wheel_idx 单轮隔离模式下被保留控制权的轮号。
             * @param chassis_motion_blocked 当前是否被 homing/fault 等整车门控阻断。
             * @param allow_drive_position_loop 当前轮是否允许继续走 drive 位置/速度闭环。
             * @param drive_zero_stop_active 整车 zero-stop 目标门是否已激活。
             * @param entering_drive_zero_stop 本拍是否刚进入 zero-stop。
             * @param leaving_drive_zero_stop 本拍是否刚退出 zero-stop。
             * @details 它是 drive 末端下发的唯一汇合点，负责把调试虚拟负载、刹车 ramp 和零电流收尾
             *          收敛成“这一拍最终发给驱动电机的真实接口语义”。
             */
            void applyDriveVirtualLoadAndCommand(WheelConfig &wheel,
                                                 u8 wheel_idx,
                                                 f32 delivered_drive_target_rad_s,
                                                 bool single_wheel_isolation_active,
                                                 u8 single_wheel_idx,
                                                 bool chassis_motion_blocked,
                                                 bool allow_drive_position_loop,
                                                 bool drive_zero_stop_active,
                                                 bool entering_drive_zero_stop,
                                                 bool leaving_drive_zero_stop);
#if JIA_CHASSIS_ENABLE_SINGLE_WHEEL_DEBUG
            f32 readSingleWheelInputAxisValue(u8 input_axis_raw) const;
            void resetSingleWheelAxisPlannerRuntime(SingleWheelAxisPlannerRuntime &runtime);
            f32 shapeSingleWheelSteerCommand(u8 wheel_idx, const SingleWheelAxisControl &axis_cfg, f32 target_value);
            f32 shapeSingleWheelDriveOmegaRadS(u8 wheel_idx, const SingleWheelAxisControl &axis_cfg, f32 target_omega_rad_s);
#endif
            void transSpeedBodyToWorld(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y) const;
            void transSpeedWorldToBody(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y) const;
            // yaw lock 这一组 helper 只解决“目标朝向应如何演化、该不该由 PID 接管 omega_z”：
            // 它们不碰平移规划，也不直接决定电机执行，只为 resolve/updatePlannedMotionData() 产出可继续消费的车体级目标。
            void resetYawPidTargetRuntime();
            f32 filterYawPidTarget(f32 target_yaw_rad);
            bool computeYawPidOmega(f32 target_yaw_rad, f32 feedback_yaw_rad, f32 &out_omega_z);
            void isLockNowRotZ(bool is_lock, f32 rot_z, f32 omega_z, f32 &out_rot_z, f32 &out_omega_z);
            void isLockToRotZ(bool is_lock, f32 tar_rot_z, f32 cur_rot_z, f32 &out_rot_z, f32 omega_z, f32 &out_omega_z);
            // clampTargetSpeedInChassis() 做的是配置层硬限幅，limitPlannedSpeed() 做的是跨拍整形与门控保持。
            // 两者先后配合，保证 target_data_ 不越界、planned_data_ 再具备时间连续性。
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

            /* ----------------------------------------------------------------- */
            // 轮组反馈、回零与执行器下发
            // 如果前面的 planner 解决的是“想去哪”，这一组解决的就是“轮子现在在哪、能不能去、最终给电机发什么”。
            // 其中 updateWheelFeedback() 只负责每轮局部反馈刷新与故障观测，整车 current_data_ 聚合在后面的 updateCurrentData() 完成。
            /**
             * @brief 刷新四个舵轮的局部反馈快照，并推进 steer fault 观测。
             * @details 这里只更新“每轮当前读到了什么”，不负责整车级 current_data_ 聚合；
             *          这样 homing/fault/debug mirror 都能在同一拍共享统一时间基准下的轮侧反馈。
             */
            void updateWheelFeedback();
            // 这组 helper 组成“正常控制 -> Latched 故障 -> Recovering -> 单轮 re-home -> 正常控制”的恢复链。
            // 它们不是纯调试镜像；其中部分函数会直接修改 homing_state、fault_state 和闭环目标。
            /**
             * @brief 更新单轮 steer freeze fault 观测并推进其恢复状态机。
             * @param wheel 目标轮运行态容器。
             * @details 它同时检查控制意图、电流变化、角度变化和 X-Park 忽略窗口，
             *          只在“本来应该动但没动”的条件下推进 None -> Latched -> Recovering。
             */
            void updateSteerFaultState(WheelConfig &wheel);
            /**
             * @brief 将单轮舵向故障锁存为 Latched 态。
             * @param wheel 目标轮运行态容器。
             * @details 进入该状态后，常规底盘链路会挂起该轮的正常执行，等待恢复跳动达到阈值后再重回零。
             */
            void latchSteerFault(WheelConfig &wheel);
            /**
             * @brief 清除单轮 steer fault 相关锁存与恢复计数。
             * @param wheel 目标轮运行态容器。
             * @details 一般在单轮 recovery homing 完成后调用，让该轮重新回到正常控制路径。
             */
            void clearSteerFaultState(WheelConfig &wheel);
            /**
             * @brief 为单轮挂起一次重回零请求。
             * @param wheel 目标轮运行态容器。
             * @details 这里只设置请求位，不直接改 homing_state；真正进入 Search 在 updateHomingState() 入口统一消费。
             */
            void requestSingleWheelHoming(WheelConfig &wheel);
            /**
             * @brief 清理单轮舵向闭环执行器的内部历史状态。
             * @param wheel 目标轮运行态容器。
             * @details 用于从故障恢复、重新回零或特殊保持态退出时，避免旧的 PID/目标历史继续影响下一拍。
             */
            void resetSteerMotorClosedLoopState(WheelConfig &wheel);
            /**
             * @brief 重置单轮 homing 边沿确认链路的累计状态。
             * @param wheel 目标轮运行态容器。
             * @details Search 重新开始、Recovering 重入或零位失效时都需要调用，避免新一轮边沿确认混入旧拍缓存。
             */
            void resetHomingEdgeConfirmState(WheelConfig &wheel);
            /**
             * @brief 推进“本次上电首次整车 homing”统一延时窗口。
             * @details 该窗口只在本次上电后的第一次整车 homing 生效；恢复回零与后续人工重回零不走这里。
             *          真实线程与 host 语义测试夹具都应调用同一 helper，避免时序语义分叉。
             */
            void updateFirstBootHomingDelayState();
            /**
             * @brief 记录一次 homing 原始边沿，并判断是否已满足三边沿确认条件。
             * @param wheel 目标轮运行态容器。
             * @param is_falling_edge 当前边沿是否为 H->L。
             * @param signed_local_total_rad 当前边沿对应的本地连续角。
             * @return `true` 表示本轮已拿到足够可靠的边沿序列，可推进到后续零偏建立阶段。
             */
            bool recordHomingEdgeAndCheckConfirmed(WheelConfig &wheel, bool is_falling_edge, f32 signed_local_total_rad);
            // updateHomingState() 是单轮 homing / recovery 状态机主入口：
            // 它读 GPIO 原始电平和 raw steer total angle，逐拍推进 zero-valid、runtime zero offset 和 ready/align/fault 状态。
            /**
             * @brief 推进单轮 homing / recovery 状态机。
             * @param wheel 目标轮运行态容器。
             * @return `true` 表示该轮本拍结束后已处于 Ready，可参与正常底盘控制。
             * @details 它既服务上电首次回零，也服务 steer fault 后的单轮重回零；
             *          其中 Search、EdgeDetected、OffsetApply、ContinuousAngleReady、AlignToZero 分拍存在，
             *          是为了让边沿确认、零偏建立和闭环归位在时序上可观察且可隔离。
             */
            bool updateHomingState(WheelConfig &wheel);
            bool readHomingSensor(const WheelConfig &wheel) const;
            bool readHomingSensorRawHigh(const WheelConfig &wheel) const;
            // raw / corrected 读接口分别服务两套语义：
            // raw 更接近电机轴自身观测，适合边沿定位与冻结检测；corrected 则是底盘内部统一使用的连续舵向语义。
            f32 readSteerMotorRawTotalAngleRad(const WheelConfig &wheel) const;
            f32 readDriveMotorOmegaRadS(const WheelConfig &wheel) const;
            f32 readCorrectedSteerMotorTotalAngleRad(const WheelConfig &wheel) const;
            f32 readSteerMotorCurrentMilliAmp(const WheelConfig &wheel) const;
            // 这四个 target helper 把“底盘内部统一语义”收敛成“电机对象实际接口语义”：
            // 上层分别按电流、舵向 rpm、舵向连续角、drive wheel omega 调用，换算细节统一留在这一层。
            /**
             * @brief 按底盘统一语义给单轮舵向电机下发电流目标。
             * @param wheel 目标轮运行态容器。
             * @param current 目标电流，单位 mA。
             */
            void setSteerMotorTargetCurrent(WheelConfig &wheel, f32 current);
            /**
             * @brief 按底盘统一语义给单轮舵向电机下发速度目标。
             * @param wheel 目标轮运行态容器。
             * @param rpm 以 corrected-local 正方向解释的目标转速。
             * @details 该层负责完成方向符号换算，并在必要时做一次写后核对/补发，
             *          统一兜住底盘写入成功但电机对象未接住目标的偶发场景。
             */
            void setSteerMotorTargetRPM(WheelConfig &wheel, f32 rpm);
            /**
             * @brief 按底盘统一语义给单轮舵向电机下发连续角目标。
             * @param wheel 目标轮运行态容器。
             * @param corrected_local_total_angle_rad corrected-local 语义下的连续角目标。
             * @details 该层负责把 corrected-local 连续角还原为电机对象理解的原始 total-angle 坐标。
             */
            void setSteerMotorTargetTotalAngleRad(WheelConfig &wheel, f32 corrected_local_total_angle_rad);
            /**
             * @brief 按底盘统一语义给单轮驱动电机下发 wheel omega 目标。
             * @param wheel 目标轮运行态容器。
             * @param drive_omega_rad_s 轮侧角速度目标，单位 rad/s。
             */
            void setDriveMotorTargetOmegaRadS(WheelConfig &wheel, f32 drive_omega_rad_s);
            f32 limitPositionSecondOrder(f32 current_value, f32 current_rate, f32 target_value, f32 max_rate, f32 max_accel, f32 dt_s, f32 &next_rate) const;
            f32 limitValueWithAcceleration(f32 current_value, f32 target_value, f32 max_accel, f32 dt_s) const;
            f32 getXParkAngle(const WheelConfig &wheel) const;
            f32 computeMaxCommandWheelSpeedMps(const Data &command_data) const;
            // 这两个 helper 服务 yaw lock 与 zero-stop 的衔接：
            // - preview 版在 planner 阶段提前判断“是否需要先把 omega_z 压到 0”，让后续模块求解看到的是刹停前预览目标；
            // - 非 preview 版在执行阶段再次按当前 residual / hold 上下文确认是否继续压零。
            // 两者配合的目的是让“先刹平移、再纯旋转”的过渡既可预测又不突兀。
            bool shouldSuppressYawLockOmegaForZeroStopDecel(const Data &command_data);
            bool shouldPreviewSuppressYawLockOmegaForZeroStopDecel(const Data &command_data) const;
            f32 computeLowSpeedDriveSuppressionScale(f32 abs_error_rad) const;
            void computeLowSpeedDriveSuppressionScales(const SwervePlannerInput &planner_input, const f32 steering_errors_rad[4], f32 out_scales[4]);
            f32 getNearZeroEnterSpeedMps() const;
            f32 getNearZeroExitSpeedMps() const;
            f32 getXParkCommandEnterSpeedMps() const;
            f32 getXParkCommandExitSpeedMps() const;
            // reverse intent / launch hold / low-speed suppression 共同服务“低速大转角过渡”：
            // - reverse intent 决定是否允许方向近似反转时直接按新方向放行；
            // - launch hold 决定是否先只转舵再恢复 drive；
            // - suppression 决定低速且误差较大时 drive 该被压到什么程度。
            bool shouldActivateReverseIntent(f32 target_vel_x, f32 target_vel_y, f32 reference_dir_rad) const;
            bool shouldActivateLaunchHold() const;
            // launch hold 相关 helper 只服务“先摆正舵轮、再恢复 drive”的过渡阶段：
            // 一个判断是否已经对齐到可放行动作的程度，另一个构造只用于对齐预览的 planner 命令。
            bool isLaunchHoldAligned(const SwervePlannerOutput &planner_output) const;
            Data makeLaunchHoldPreviewCommand() const;
            // refreshActuatorLimitState() 当前是主循环中的“阶段边界钩子”：
            // 现在限幅开关直接就近读取，但保留这层调用点，便于未来把限幅/阈值镜像收敛成统一快照。
            void refreshActuatorLimitState();
            // mapSingleTurnToNearestTotalAngle() 把单圈 OA 语义目标映射成“离当前姿态最近”的连续总角，
            // 主要服务调试对齐与显式单轮舵角命令，避免同一目标因跨圈解释不同而多转整圈。
            f32 mapSingleTurnToNearestTotalAngle(const WheelConfig &wheel, f32 target_oa_single_turn_deg) const;
            /**
             * @brief 将车体级命令与当前反馈/门控上下文展开成模块 planner 输入。
             * @param command_data 当前准备送入模块解算的整车级命令。
             * @return 包含目标 twist、当前模块朝向、残余速度和统一朝向请求的完整 planner 输入。
             */
            SwervePlannerInput makeSwervePlannerInput(const Data &command_data);
            /**
             * @brief 执行四舵轮模块规划，产出本拍的理想解、选中解和抑制结果。
             * @param planner_input 已展开好的模块规划输入。
             * @return planner 视角下的完整输出；其中仍保留理想值、规划值和最终 planner 值，不等于最终执行值。
             */
            SwervePlannerOutput planSwerveModules(const SwervePlannerInput &planner_input);
            // 这两个 helper 把 planner 输出再收束成“后续执行/调试统一读取”的命令帧与缓存快照。
            // build 负责拍扁字段层次，store 负责写入本拍执行镜像与跨拍历史状态。
            /**
             * @brief 将 planner 输出压平成执行层易消费的命令帧。
             * @param planner_output 最近一次模块规划结果。
             * @param out_frame 输出命令帧。
             * @details 该步骤会同时保留 planned 轨迹点与 steer_cmd 最终舵向命令两套视图，
             *          便于后续执行层区分“规划轨迹”与“真正准备下发的舵向目标”。
             */
            void buildActuatorCommandFrame(const SwervePlannerOutput &planner_output, ActuatorCommandFrame &out_frame) const;
            /**
             * @brief 存储本拍 planner 输出与执行前命令帧镜像。
             * @param planner_output 最近一次模块规划结果。
             * @param command_frame 由 planner 输出压平得到的执行前命令帧。
             * @details 它只写缓存与镜像，不直接驱动电机；真正的执行仲裁在 applyModuleCommands() 内完成。
             */
            void storePlannedActuatorFrame(const SwervePlannerOutput &planner_output, const ActuatorCommandFrame &command_frame);
            f32 computeHomingAlignTargetCorrectedLocalTotal(const WheelConfig &wheel) const;
            // 这两个 helper 组成“规划舵向 -> 规划驱动 -> 规划车体 twist readback”的观测链：
            // 一个沿计划舵向投影 drive 目标，另一个再从计划 steer/drive 反解回底盘 twist，供调试/可视化核对规划结果。
            void computeProjectedDriveFromPlannedSteer(const Data &command_data, const f32 planned_oa_total_rad[4], f32 out_drive_omega_rad_s[4]) const;
            // estimatePlannedBodyTwist() 的输出沿用公开/debug body frame 语义，和 setSpeed*/get* 的坐标约定保持一致。
            bool estimatePlannedBodyTwist(const f32 planned_oa_total_rad[4], const f32 planned_drive_omega_rad_s[4], f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z) const;
            f32 updateHighSpeedDriveSuppression(f32 translational_speed_m_s, f32 eta_max_s, f32 dir_err_deg);
            // computeModuleCommands() 负责“从车体命令 -> 模块规划命令帧”，只产出理想执行目标，不直接写电机。
            /**
             * @brief 把车体级命令解算为本拍的模块规划命令帧。
             * @param command_data 已完成车体级整形的整车目标。
             * @details 它只负责生成 planner 理想值、规划值和执行前命令帧，不直接写电机；
             *          真正是否允许原样下发，要交给 applyModuleCommands() 再结合运行态仲裁。
             */
            void computeModuleCommands(const Data &command_data);
            // applyModuleCommands() 是最终执行仲裁层：会结合 homing/fault/zero-current/torque-free/zero-stop 等运行态决定哪些目标真的下发。
            /**
             * @brief 把模块规划命令与运行态门控融合后真正下发到电机层。
             * @param all_homed 当前四轮是否都已达到 Ready。
             * @details 这是“planner 理想目标”与“本拍最终执行语义”的分界点；
             *          它会更新 wheel.target_* / planned_data_ 中对外最常观察的执行镜像。
             */
            void applyModuleCommands(bool all_homed);
            /**
             * @brief 基于本拍最终执行结果与反馈快照更新 current_data_。
             * @param all_homed 当前四轮是否都已达到 Ready。
             * @details 这里负责把轮侧反馈回填到对外可读的整车级 current_data_，
             *          同时决定何时允许整车 twist 估计被视为可信。
             */
            void updateCurrentData(bool all_homed);

            /* ----------------------------------------------------------------- */
            // 调试输出与运行态镜像
            // 这些声明主要服务于观察和调参，不改变主控制决策本身。
#if JIA_CHASSIS_ENABLE_DEBUG_MIRROR
            // refreshDebugMirror() 在 compute/apply/current 全部完成后刷新，只做只读观测镜像整理，不反向驱动控制。
            /**
             * @brief 刷新面向调试器与 host 语义测试的聚合镜像。
             * @param all_homed 当前四轮是否全部回零完成。
             * @details 它把分散在 runtime/cache 中的关键结论整理成易读视图，
             *          但不生成任何新的控制命令，也不回写主控制状态机。
             */
            void refreshDebugMirror(bool all_homed);
#endif
#if JIA_CHASSIS_ENABLE_DEBUG_OUTPUT
            // 这组 emitter 只负责把当前观测快照发出去；节流、profile 与 payload 口径都在各自函数内部定义，不改变控制链。
            /**
             * @brief 输出文本调试摘要。
             * @param all_homed 当前四轮是否全部回零完成。
             * @details 该接口服务“给人眼快速扫状态”的文本通道，主要复用 debug_mirror_ 等聚合视图。
             */
            void emitDebugUart8Log(bool all_homed);
            void emitUart8VofaJustFloatPidTrace();
            void emitUart8VofaPid1kHzTrace();
            void emitUart8VofaSingleWheelDriveTrace();
            void emitUart8VofaDualMotor1kHzTrace();
            // VOFA trace 负责把某一条调试观察链路发给 JustFloat 后端：
            // 它们都是只读 emitter，不生成新控制命令；具体节流周期和 profile 选择由 debug_output_ 与 runtime 配合完成。
            void emitUart8VofaYawPidTrace();
            void emitUart8VofaDriveZeroStopBrakeTrace();
            // emitDebugOutputByMode() 只是输出分发层：
            // family/profile 的路由选择在这里完成，但具体 payload 定义、节流、sample_divider 和 seq 维护都在各自 emitter 内。
            /**
             * @brief 按当前调试输出 family/profile 分发本拍输出。
             * @param all_homed 当前四轮是否全部回零完成。
             * @details 它只做路由选择，不负责生成控制命令；真正的 payload 组织与节流逻辑在各自 emitter 内部。
             */
            void emitDebugOutputByMode(bool all_homed);
#endif
#if JIA_CHASSIS_ENABLE_PID_TUNE_CACHE
            /**
             * @brief 在调试使能上升沿时，把 PID 调参缓存同步到运行态。
             * @details 该入口只在 enable edge 触发一次，避免缓存参数在调试打开期间被重复整包刷入。
             */
            void syncDebugSteerPidTuneFromRuntimeOnEnableEdge();
            /**
             * @brief 将 PID 调参缓存同步到当前电机运行态。
             * @details 该过程消费 debug_pid_tune_ 中的待生效参数戳，把共享配置下发到舵向/驱动执行器。
             */
            void syncDebugSteerPidTuneFromRuntime();
            /**
             * @brief 根据调试缓存对运行中的 PID 进行在线参数更新。
             * @details 它负责把调试面板中的 PID 配置真正写入运行对象，是调参与控制主链之间的桥接层。
             */
            void applyDebugSteerPidRuntimeTuning();
#endif

            /* ----------------------------------------------------------------- */
            // 通用数学与轻量只读映射
            // 放在末尾是为了把主控制流程声明按职责集中阅读。
            bool solveLinear3x3(f32 matrix[3][4], f32 &x0, f32 &x1, f32 &x2) const;
            /**
             * @brief 由四个模块当前反馈反解整车 body twist。
             * @param out_vel_x 输出的车体系 x 线速度。
             * @param out_vel_y 输出的车体系 y 线速度。
             * @param out_omega_z 输出的车体系偏航角速度。
             * @return `true` 表示本次反解成功且结果可用。
             * @details 这是反馈侧估计接口，语义上不同于 planner 侧“按命令预估会怎么动”的反解 helper。
             */
            bool estimateBodySpeedFromModules(f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z) const;
#if JIA_CHASSIS_ENABLE_TASK_PERF_STAT
            /**
             * @brief 更新主循环总执行耗时统计窗口。
             * @param loop_start_us 本拍主循环开始时间戳。
             * @param loop_end_us 本拍主循环结束时间戳。
             * @details 该函数维护最近窗口平均值、历史极值、超预算计数与短窗环形缓存，
             *          主要服务 FULL_DEBUG 下的线程预算诊断。
             */
            void updateTaskPerfStat(u64 loop_start_us, u64 loop_end_us);
            /**
             * @brief 记录主循环各阶段耗时拆分。
             * @param plan_us 规划阶段耗时。
             * @param feedback_us 反馈刷新阶段耗时。
             * @param homing_us homing / fault 状态机阶段耗时。
             * @param apply_us 模块命令生成与执行阶段耗时。
             * @param debug_us 调试镜像与输出阶段耗时。
             */
            void updateTaskPerfBreakdown(u64 plan_us, u64 feedback_us, u64 homing_us, u64 apply_us, u64 debug_us);
#endif
            static SteerCalibration makeSteerCalibration(const WheelConfig &wheel);
            static f32 mapWheelCorrectedLocalToOaTotal(const WheelConfig &wheel, f32 corrected_local_total_rad);
            static f32 mapWheelOaTotalToCorrectedLocal(const WheelConfig &wheel, f32 oa_total_rad);
            // =====================================================================
            // 系统时基与线程时钟 [RO]
            // 说明：这是所有“每周期推进一次”的逻辑共用的统一时间基准。
            // 如果控制周期变化，后面的超时、hold、节流、滤波和采样窗口都要一起重新复核。
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
                // ---- 整车级速度曲线与基础限幅 -----------------------------------
                // 这一组决定 target_data_ 进入 planned_data_ 时如何变得“可执行且平顺”。
                ManualSpeedProfileMode manual_speed_profile_mode = ManualSpeedProfileMode::kLegacy;
                bool manual_speed_profile_manual_only = true;
                f32 manual_trans_acc_acc_ = 5.0f;
                f32 manual_trans_acc_dec_ = 12.0f;
                f32 manual_trans_jerk_acc_ = 130.0f;
                f32 manual_trans_jerk_dec_ = 150.0f;
                f32 manual_trans_settle_vel_eps_ = 1.0e-4f;
                f32 manual_trans_settle_acc_eps_ = 0.05f;
                f32 manual_yaw_alpha_acc_ = 5.0f;
                f32 manual_yaw_alpha_dec_ = 12.0f;
                f32 manual_yaw_jerk_acc_ = 130.0f;
                f32 manual_yaw_jerk_dec_ = 150.0f;
                f32 manual_yaw_settle_vel_eps_ = 1.0e-4f;
                f32 manual_yaw_settle_acc_eps_ = 0.05f;
                f32 wheel_radius_m_ = 0.052f;                                    // [RW, 慎改] 轮半径。决定线速度与驱动角速度的换算比例，改错会直接导致速度尺度和里程计比例偏差。
                f32 max_vel_x_ = 5.0f;                                           // [RW] 车体 X 方向最大线速度上限（m/s）。用于规划/限幅，不是电机硬件极限。
                f32 max_vel_y_ = 5.0f;                                           // [RW] 车体 Y 方向最大线速度上限（m/s）。同上，约束横移速度。
                f32 max_omega_z_ = 5.0f;                                         // [RW] 车体 Z 轴最大角速度上限（rad/s）。同上，约束原地旋转或航向变化速度。
                f32 max_acc_xy_acc_ = 99999999.0f;                                      // [RW] 平面加速段最大加速度（m/s^2）。越小起步越柔和，越大响应越猛。
                f32 max_acc_xy_dec_ = 99999999.0f;                                     // [RW] 平面减速段最大减速度（m/s^2）。越小刹车越平滑，越大停车越快但冲击更强。
                f32 max_alpha_z_acc_ = 99999999.0f;                                     // [RW] 航向加速段最大角加速度（rad/s^2）。影响转向起步的平顺性。
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
                bool enable_steer_angle_feedforward = true;                      // [RW] 是否启用底盘层舵角超前前馈。只影响正常 swerve 规划下发角，不改变物理预计角。
                f32 steer_angle_feedforward_lead_s = 0.3f;                      // [RW] 舵角超前时间（s）。用于补偿舵向电机响应滞后。
                f32 steer_angle_feedforward_max_lead_rad = 0.3f;          // [RW] 舵角超前最大幅度（rad）
                f32 steer_angle_feedforward_settle_error_rad = 0.05235988f;      // [RW] 收尾线性衰减误差窗口（rad），默认约 3°。

                // ---- 通用 near-zero 门限 -----------------------------------------
                // 这组阈值描述“速度已经接近 0”的通用口径。使用者看这里时要先分清对象：
                // - actual/residual：反馈或残余速度是否已经足够小，X-Park 进入门会用它做一次安全确认。
                // - target/command：drive zero-stop 直接复用这组阈值判断目标是否已经进入刹车模式。
                // 它不是 X-Park 目标静止意图的专用门；X-Park command 门在下面单独配置。
                struct NearZeroThresholdConfig
                {
                    f32 base_enter_m_s = 0.005f; // [RW] 通用 near-zero 进入阈值（m/s）。未激活的门控用它判断“可以进入”。
                    f32 base_exit_m_s = 0.015f;  // [RW] 通用 near-zero 退出阈值（m/s）。已激活的门控用它保持滞回，应大于 enter。
                } near_zero_cfg_;

                struct XParkCommandThresholdConfig
                {
                    f32 enter_m_s = 0.005f; // [RW] X-Park 目标静止进入阈值（m/s）。只看 target/command，不看 actual residual。
                    f32 exit_m_s = 0.015f;  // [RW] X-Park 目标静止退出阈值（m/s）。X-Park 已锁存后只用它决定是否退出。
                } xpark_command_threshold_cfg_;

                struct XParkSteerHoldConfig
                {
                    bool enable = true;                    // [RW] 是否启用统一的 X-Park 舵向 hold 状态机。
                    f32 entry_angle_deg = 1.0f;           // [RW] X-Park 舵向误差进入 hold 的角误差阈值（deg）。
                    f32 exit_angle_deg = 10.0f;            // [RW] X-Park 舵向误差退出 hold 的角误差阈值（deg）。应大于 entry 形成滞回。
                    f32 settle_angle_deg = 2.0f;          // [RW] X-Park 舵向 hold 判稳角误差阈值（deg）。
                    f32 settle_target_rate_deg_s = 2.0f;  // [RW] X-Park 舵向 hold 判稳目标角速度阈值（deg/s）。
                    u32 settle_hold_ms = 1000;              // [RW] 满足判稳条件后，进入零电流锁定前需持续保持的时长（ms）。
                    u32 reacquire_hold_ms = 500U;           // [RW] 零电流锁定退出后，重新允许锁定前的等待时长（ms）。
                    bool entry_reset_enable = true;       // [RW] 进入 hold Settling 阶段时是否执行一次舵向速度环历史清理。
                } xpark_steer_hold_cfg_;

                // ---- drive 零速止停辅助 -----------------------------------------
                // 这一组位于 planner 与最终 drive 输出之间：
                // 先决定“目标是否足够接近静止，需要切入 zero-stop 模式”，再决定“每个轮子当前是继续 brake 还是切零电流收尾”。
                // 仅在整车正常 drive 闭环链路里使用。这里分成两层，调参时不要混在一起看：
                // - 模式层：目标速度进入 near_zero_cfg_ 的 enter 门后，zero-stop active；目标速度离开 exit 门后恢复 RPM 闭环。
                // - 末端层：zero-stop 已 active 后，实际 residual 只决定当前轮继续 brake，还是已经停稳可切到零电流。
                // residual 不负责进入/退出 zero-stop 模式；它只负责 active 期间的“刹住以后是否安静收尾”。
                bool enable_drive_zero_stop_assist = true;          // [RW] 是否启用 drive 零速止停辅助。
                bool enable_drive_zero_stop_settle_zero_current = true; // [RW] 是否允许 drive zero-stop 在 residual 进入 near-zero enter 后切到零电流收尾。关闭后 active 期间始终 brake。
                f32 drive_zero_stop_brake_current_mA = 25000.0f;     // [RW] 零速止停进入 brake 分支时下发的刹车电流。
                u32 drive_zero_stop_brake_ramp_time_ms = 0U;         // [RW] zero-stop 目标门进入后，从 0 线性爬升到 brake 电流的时长（ms）。0 表示阶跃下发。
                u32 yaw_lock_zero_stop_release_hold_ms = 20U;       // [RW] yaw lock 从平移减速切到纯旋转前，residual 进入 near-zero 后额外保持 brake 的时长（ms）。

                struct LowSpeedDriveSuppressionConfig
                {
                    f32 close_angle_deg = 10.0f;             // [RW] 低速抑制使用。舵角误差超过该阈值后进入驱动压制区。
                    f32 min_scale = 0.0f;                   // [RW] 低速抑制使用。进入压制区后保留的最小驱动比例。
                };

                // ---- 舵向冻结故障检测与恢复 ------------------------------------
                // 这一组只负责“是否怀疑舵向失联/断电”和“何时允许自动重回零”，不负责普通 homing 参数。
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
                    bool enable = true;                 // [RW] 是否启用“近似反向意图”识别。启用后可更积极地偏向翻转解。
                    f32 enter_angle_deg = 105.0f;      // [RW] 目标方向与参考方向夹角进入阈值（deg）。超过它开始视作“想反着走”。
                    f32 exit_angle_deg = 75.0f;        // [RW] 退出阈值（deg）。小于 enter 形成滞回，避免方向抖动时反复切换。
                    f32 min_speed_m_s = 0.0f;          // [RW] 最小速度门槛。低于它时即便方向反向也不激活 reverse intent。
                    f32 flip_prefer_margin_deg = 5.0f; // [RW] 当翻转解更优时要求额外领先的角差裕量（deg），避免和普通最短路来回打架。
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
            // 配置基线 vs 运行时快照：
            // - default_strategy_cfg_ 是默认基线，回答“系统初始化后原则上应该怎么跑”；
            // - runtime_strategy_cfg_ 是当前生效配置，回答“这一拍控制线程实际按什么规则在跑”。
            StrategyConfig default_strategy_cfg_; // [RW, 慎改] 默认策略基线。用于初始化和“恢复默认值”，不要把它当作实时状态。
            StrategyConfig runtime_strategy_cfg_; // [RW] 当前生效的运行时策略。可被外部接口动态切换，控制链路实际读取它。


            // =====================================================================
            // 航向控制参数（运行时可调）[RW]
            // 通过全局 chassis 对象在调试器内直接改值。[RW]
            // 说明：这组参数只影响航向锁定/锁角逻辑，不影响平移速度规划。
            // =====================================================================
            PID_Position rot_z_pid_;                  // [RW, 慎改] 航向位置环 PID。用于 LockToYaw / 相关锁角模式的角度误差闭环。
            u8 rot_z_pid_period_ = 1;                 // [RW] PID 更新周期分频。1 表示每个控制周期都更新，数值越大频率越低、负载越小但响应更慢。
            f32 max_lock_to_rot_z_rad_s_ = 99999999.0f;      // [RW] LockToYaw 模式下的角速度上限（rad/s）。用于限制“往目标角赶”的最快速度。
            u32 lock_now_rot_z_shift_time_ms_ = 1000; // [RW] LockNow 松手缓冲时长（ms）。松开后短时间内继续维持目标，避免姿态突然跳变。
            f32 lock_yaw_pid_target_lpf_alpha_ = 1.0f; // [RW] 航向 PID 目标低通系数，1=关闭滤波，0=保持上一滤波目标。
            f32 lock_yaw_pid_deadband_enter_deg_ = 0.05f; // [RW] 航向 PID 死区进入阈值（deg）。
            f32 lock_yaw_pid_deadband_exit_deg_ = 0.20f;  // [RW] 航向 PID 死区退出阈值（deg）。

            // =====================================================================
            // 调试参数（通过全局 chassis 对象在调试器内直接改值）[RW]
            // 说明：这组参数只影响调试链路。正常控制不读取它们，只有切到相应 debug mode 时才会生效。
            // 速查：0~9 = 底盘输入接管/信号注入类模式（9 = 定角驱动）；20 = 已退役（安全回退）；21 = 四轮朝前；22 = 回零观察；30 = 单轮独立直控。
            // 手柄平移坐标约定（对外/调试接管语义）：前推朝当前 2/3 面，左推朝当前 3/4 面；
            // 映射到内部 body 命令时使用 -left_x -> vel_x、-left_y -> vel_y。
            // RUNTIME_MIN 会移除整组调试接管字段，避免比赛固件为面板模式、单轮直控和观测轮号长期占 RAM。
            // =====================================================================
#if JIA_CHASSIS_ENABLE_DEBUG_OVERRIDE
            struct DebugControl
            {
                struct Common
                {
                    bool enable = true;                                            // [RW] 调试总开关。
                    u8 mode_raw = 2;                                               // [RW] 调试模式号。
                    u8 mode_resolved_raw = static_cast<u8>(DebugMode::kWorldSpeed); // [RO] 解析后的实际模式号。
                    u8 control_wheel_index = 0U;                                    // [RW] 当前执行目标轮号。单轮模式运行时只认这一处。
                    u8 observe_wheel_index = 0U;                                    // [RW] 当前输出观察轮号。单轮模式运行时只认这一处。
                } common{};

                struct Injection
                {
                    f32 lock_rot_z = 0.0f;     // [RW] LockTo 调试目标角（rad）。
                    u8 omega_z_injection_mode_raw = static_cast<u8>(DebugOmegaZInjectionMode::kOff); // [RW] omega_z 注入模式。
                    f32 omega_z_sine_amplitude = 0.0f;    // [RW] omega_z 正弦注入幅值。
                    f32 omega_z_sine_frequency_hz = 0.1f; // [RW] omega_z 正弦注入频率（Hz）。
                    f32 omega_z_sine_offset = 0.0f;       // [RW] omega_z 正弦注入偏置。
                    f32 steer_deg_limit = 180.0f;
                    f32 drive_speed_m_s_limit = 1.0f;
                } injection{};

                struct SingleWheel
                {
                    bool estop = false;         // [RW] 单轮调试急停闸门。
                    f32 input_deadzone = 0.03f; // [RW] 单轮调试共享摇杆死区。落入死区后两轴输入都直接置 0。
                    SingleWheelAxisControl steer{
                        false,
                        static_cast<u8>(DirectAxisInputMode::kRcContinuous),
                        static_cast<u8>(SingleWheelInputAxis::kLeftX),
                        false,
                        static_cast<u8>(DirectSteerCommandType::kSingleTurnDeg),
                        0.0f,
                        200.0f,
                        0.5f,
                        45.0f,
                        static_cast<u8>(SingleWheelPlannerMode::kOff),
                        {},
                        {}};
                    SingleWheelAxisControl drive{
                        true,
                        static_cast<u8>(DirectAxisInputMode::kRcStep),
                        static_cast<u8>(SingleWheelInputAxis::kRightX),
                        false,
                        static_cast<u8>(DirectDriveCommandType::kRpm),
                        0.0f,
                        1000.0f,
                        0.5f,
                        800.0f,
                        static_cast<u8>(SingleWheelPlannerMode::kOff),
                        {},
                        {}};
                } single_wheel{};
            } debug_control_;
#endif
#if JIA_CHASSIS_ENABLE_DEBUG_OVERRIDE || JIA_CHASSIS_ENABLE_PID_TUNE_CACHE
            bool debug_enable_last_cycle_ = false; // [RO] 调试总开关上一周期的状态。主要用于识别 enable 上升沿，并在刚开启调试时做一次基线同步。
#endif
#if JIA_CHASSIS_ENABLE_SINGLE_WHEEL_DEBUG
            u8 single_wheel_last_steer_command_type_raw_ = 0xFFU; // [RO] 上一次已同步的 mode30 舵向命令类型。切换类型后可据此判断是否需要刷新对应模板。
            u8 single_wheel_last_drive_command_type_raw_ = 0xFFU; // [RO] 上一次已同步的 mode30 驱动命令类型。切换类型后可据此判断是否需要刷新对应模板。
#endif

            // =====================================================================
            // 调试输出 [RW]
            // 说明：这里只管“串口往外发什么”，不管底盘怎么跑。
            //       output_enable 是总开关，output_mode_raw 选路径，text_log_level 决定文本模式的细度。
            // RUNTIME_MIN 默认移除整组配置和运行态，避免串口 trace/telemetry 占用固件空间和 Chassis RAM。
            // =====================================================================
#if JIA_CHASSIS_ENABLE_DEBUG_OUTPUT
            struct DebugOutputSlotConfig
            {
                u32 period_ms = 10U;
            };

            struct DebugOutputTextConfig
            {
                u32 period_ms = 500U;
                u8 log_level = 1U;
            };

            struct DebugOutputJustFloatConfig
            {
                u8 profile_raw = static_cast<u8>(JustFloatProfile::kSingleWheelTrace);
                u8 single_wheel_payload_raw = static_cast<u8>(SingleWheelTracePayloadKind::kSteerAndDrive);
                DebugOutputSlotConfig overview = {5U};
                DebugOutputSlotConfig single_wheel = {1U};
                DebugOutputSlotConfig yaw_pid = {4U};
                DebugOutputSlotConfig drive_zero_stop_brake = {2U};
            };

            struct DebugOutputConfig
            {
                // DebugOutputConfig 只回答“想发什么”：
                // family 选关闭/文本/JustFloat，子配置决定各自 profile 和发送周期。
                // 当前 text / justfloat 都只保留“发送节流”这一层语义：
                // DebugOutputSlotConfig.period_ms 就是该路输出最短隔多久允许发一次，不再区分额外采样周期。
                // 旧 binary telemetry 曾有 sample_divider 这类“采样分频/采样周期”概念；
                // 该概念已随 binary 输出删除，当前 justfloat 配置里不存在“先采样、后发送”的第二层节拍。
                bool output_enable = true;
                u8 output_family_raw = static_cast<u8>(DebugOutputFamily::kJustFloat);
                DebugOutputTextConfig text{};
                DebugOutputJustFloatConfig justfloat{};
            } debug_output_;

            struct DebugOutputSlotRuntime
            {
                TickType_t last_ms = 0U;
            };

            struct DebugOutputTextRuntime
            {
                TickType_t last_ms = 0U;
                u8 log_phase = 0U;
                TickType_t direct_trace_last_ms = 0U;
            };

            struct DebugOutputJustFloatRuntime
            {
                DebugOutputSlotRuntime overview{};
                DebugOutputSlotRuntime single_wheel{};
                DebugOutputSlotRuntime yaw_pid{};
                DebugOutputSlotRuntime drive_zero_stop_brake{};
            };

            struct DebugOutputRuntime
            {
                // DebugOutputRuntime 只回答“这一路上次发到哪了”：
                // last_ms 用于发送节流；它记录的是“上一帧何时允许/已经发出”，
                // 不是旧 binary telemetry 那种独立采样分频计数。
                // 这些状态由各 emitter 在准备发送/发送成功后刷新，
                // 外部应把它当成输出链路私有运行态，只读观察即可。
                DebugOutputTextRuntime text{};
                DebugOutputJustFloatRuntime justfloat{};
            } debug_output_runtime_;
#endif

            // =====================================================================
            // DebugPidTune [RW]
            // 说明：这里存的是“待同步的 PID 配置缓存”，不是运行态实时对象。
            //       写完后通常还要等调试使能边沿或同步流程消费，运行中的 PID 才会真正换参数。
            // RUNTIME_MIN 默认不保留这份缓存；比赛固件用初始化层的 PID 参数，避免每个 Chassis 常驻一份调参面板副本。
            // =====================================================================
#if JIA_CHASSIS_ENABLE_PID_TUNE_CACHE
            struct DebugPidTune
            {
                PID_Param_Config steer_speed_pid_cfg = {.kp = 32.0f, .ki = 0.085f, .kd = 0.0f, .I_Outlimit = 8000.0f, .isIOutlimit = true, .output_limit = 12000.0f, .deadband = 0.5f};
                PID_Param_Config steer_angle_pid_cfg = {.kp = 3.5f, .ki = 0.0f, .kd = 0.05f, .I_Outlimit = 0.0f, .isIOutlimit = true, .output_limit = 500.0f, .deadband = 0.03f};
                f32 steer_speed_pid_td_ratio = 0.0f;         // [RW] 共享舵向速度环 TD 比例参数。属于扩展调参项，通常和速度环整定一起看。
                f32 steer_angle_pid_i_separa = 0.0f;         // [RW] 共享舵向角度环积分分离参数。用于决定误差多大时才允许积分参与。
                u32 steer_speed_pid_apply_stamp = 0U;        // [RW] 共享舵向速度环参数申请生效戳。外部写入后，通过同步流程统一下发到 4 个舵向轮。
                u32 steer_angle_pid_apply_stamp = 0U;        // [RW] 共享舵向角度环参数申请生效戳。外部写入后，通过同步流程统一下发到 4 个舵向轮。
                u32 steer_speed_pid_applied_stamp = 0U;      // [RO] 共享舵向速度环已生效戳。表示存在的 steer 轮已经完成这组共享参数同步。
                u32 steer_angle_pid_applied_stamp = 0U;      // [RO] 共享舵向角度环已生效戳。表示存在的 steer 轮已经完成这组共享参数同步。
                bool synced_on_enable_edge = false;                         // [RO] 本次调试使能上升沿是否已完成同步。避免重复把缓存参数刷入运行态。
                PID_Param_Config drive_speed_pid_cfg = {.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .I_Outlimit = 20000.0f, .isIOutlimit = true, .output_limit = 20000.0f, .deadband = 0.0f};
                f32 drive_speed_pid_td_ratio = 0.0f;                    // [RW] 兼容旧调参字段名。drive 轮改成位置式 PID 后，这里实际承载的是积分分离阈值。
                bool drive_speed_pid_derivative_first = false;          // [RW] 兼容旧调参字段。位置式 PID 下该开关不再生效，运行态固定回读 false。
                u32 drive_speed_pid_apply_stamp = 0U;                   // [RW] drive 共享速度环参数申请生效戳。外部写入后，通过同步流程统一下发到 4 个驱动轮。
                u32 drive_speed_pid_applied_stamp = 0U;                 // [RO] drive 共享速度环已生效戳。表示 4 个 drive 轮已经完成这组共享参数的同步。
            } debug_pid_tune_;
#endif

#if JIA_CHASSIS_ENABLE_DEBUG_OUTPUT
            // 这不是旧 drive load trace 的残留，而是 JustFloatProfile::kDriveZeroStopBrakeTrace
            // 专用的最小观察缓存。前面的 drive 执行路径 / mode30 单轮直控路径先把当前观察轮的关键量写进来，
            // emitter 只负责按既定 payload 顺序消费这 3 个字段，不在发送阶段重新回溯整条控制链。
            struct DebugDriveZeroStopBrakeTraceState
            {
                f32 target_rpm = 0.0f;       // 当前观察轮这一拍用于 trace 展示的目标 RPM。
                f32 feedback_rpm = 0.0f;     // 当前观察轮这一拍用于 trace 展示的反馈 RPM。
                f32 observe_wheel_idx = 0.0f; // 当前 zero-stop-brake trace 正在观察哪一轮。
            } debug_drive_zero_stop_brake_trace_;
#endif

            // =====================================================================
            // 回零、模块反馈与门控锁存 [RO]
            // 读这一组时建议按四种角色理解：
            // - 配置副本：静态装配信息与硬件句柄在运行时的落地副本；
            // - 快照：最近一次反馈、规划和执行结果；
            // - 门控：当前是否允许某种动作继续推进；
            // - 锁存：为了形成滞回和跨周期保持而保留的状态。
            // =====================================================================
            // 这份 yaw trace 很小，并且被航向控制函数直接写入。RUNTIME_MIN 保留它能避免把核心锁角逻辑切碎；
            // 真正占空间的串口输出配置、调试镜像和任务耗时窗口仍由 profile 裁剪。
            // mode_tag 约定：
            // 0=刚进入 Lock 流程的默认态，1=手动旋转输入仍在主导，2=LockNow 松手后的 shift 缓冲，3=LockNow PID 保持，4=LockTo PID 跟踪。
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

            // 回零请求与轮组运行态快照：
            // 这一组保存“轮子自身”最核心的跨拍事实，包括装配副本、反馈快照、执行历史和局部状态机阶段。
            // 其中 last_* 不是单纯调试缓存，而是限幅、zero-stop、残余速度判定这类跨拍整形逻辑的直接输入。
            bool homing_start_request_ = false;                                // [RW] 回零启动请求锁存位（由外部触发，在线程内消费）
            struct FirstBootHomingDelayState
            {
                bool pending = true;   // [RW] 本次上电首次整车 homing 的延时机会是否尚未消耗。
                bool active = false;   // [RW] 当前是否正处在首次整车 homing 的统一等待窗口。
                u32 elapsed_ms = 0U;   // [RW] 首次整车 homing 统一等待已累计时长（ms，仅这一轮使用）。
            } first_boot_homing_delay_;
            f32 homing_align_to_zero_tolerance_deg_ = 2.0f;                    // [RW] 回零归位判稳阈值（deg）
            WheelConfig wheel_config_[4];                                      // [RO] 四个模块运行态快照
            f32 last_steer_rate_cmd_rad_s_[4] = {0.0f};                        // [RO] 上周期最终采用的转向速度命令。主要服务舵向二阶限幅与 hold 收尾。
            f32 last_drive_omega_cmd_rad_s_[4] = {0.0f};                       // [RO] 上周期真正送入 drive 执行链路的角速度命令。zero-stop / alpha-limit 都依赖它做跨拍渐变。
            f32 last_drive_feedback_omega_rad_s_[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [RO] 上一拍 drive 实际反馈角速度。主要服务残余速度估计与零速收尾判断。
            u32 drive_feedback_sample_ms_[4] = {0U, 0U, 0U, 0U};               // [RO] 本拍读取 drive 反馈时的采样时间戳。用于判定“当前反馈来自哪一拍”。
            u32 last_drive_feedback_sample_ms_[4] = {0U, 0U, 0U, 0U};          // [RO] 上一拍 drive 反馈采样时间戳。配合当前时间戳可判断反馈是否连续。
            bool selected_flipped_solution_[4] = {false};                      // [RO] 每个模块上一拍保留下来的翻转解锁存。和 planner_output 里的 flipped_drive_direction[本拍结果] 不同。
            f32 low_speed_drive_suppression_scale_[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // [RO] 每轮低速抑制最终缩放。
            f32 high_speed_drive_suppression_scale_ = 1.0f;                        // [RO] 当前高速抑制缩放。
            bool high_speed_trans_gate_active_ = false;                            // [RO] 当前高速抑制速度门是否打开。复用 near-zero enter/exit 做滞回。
            bool high_speed_drive_suppression_active_ = false;                     // [RO] 当前高速抑制是否激活。
            f32 high_speed_dir_err_deg_ = 0.0f;                                    // [RO] 当前合成平移方向误差（deg）。
            f32 high_speed_eta_max_s_ = 0.0f;                                      // [RO] 当前四轮最大预计到角时间（s）。
            f32 max_residual_speed_m_s_ = 0.0f;                                // [RO] 当前拍四轮中的最大实际残余速度（m/s）。
            bool low_speed_residual_bypass_active_ = false;                        // [RO] 当前低速抑制残余速度旁路门是否打开。复用 near-zero enter/exit 做滞回。
            bool low_speed_drive_suppression_bypassed_by_residual_speed_ = false; // [RO] 当前拍低速抑制是否因残余速度阈值被旁路。

            // 航向控制缓存与锁存：
            // 这一组的作用不是“再存一份 yaw 数据”，而是保证 LockNow / LockTo 在跨拍时具有连续语义，
            // 包括目标保持、shift 缓冲、预览命令缓存，以及“先刹平移、再纯旋转”的 zero-stop 过渡锁存。
            // 它们主要由 isLockNowRotZ() / isLockToRotZ() / filterYawPidTarget() / computeYawPidOmega() 读写。
            u8 rot_z_pid_count_ = 0;                                           // [RO] 航向 PID 分频计数器
            f32 lock_now_rot_z_target_ = 0.0f;                                 // [RO] LockNow 真正维持的航向目标
            u32 lock_now_rot_z_shift_count_ = 0;                               // [RO] LockNow 松手缓冲倒计时
            bool yaw_lock_control_active_last_cycle_ = false;                  // [RO] 上一规划周期是否处于 LockNow/LockTo yaw 锁控制族
            bool yaw_lock_zero_stop_decel_context_active_ = false;              // [RO] yaw lock 从平移减速进入纯旋转前，等待 drive residual 先刹停的锁存门
            u32 yaw_lock_zero_stop_release_hold_elapsed_ms_ = 0U;               // [RO] yaw lock zero-stop 释放保持已累计时长（ms）。达到配置门限后才允许退出 brake latch。
            bool yaw_lock_zero_stop_preview_command_valid_ = false;             // [RO] 当前控制周期是否缓存了用于 compute 阶段推进 hold 的未压零 yaw command。
            Data yaw_lock_zero_stop_preview_command_{};                         // [RO] 当前控制周期在 planner 前保存的未压零 yaw command。
            bool lock_yaw_pid_target_filter_valid_ = false;                    // [RO] 航向 PID 目标低通状态是否已初始化
            f32 lock_yaw_pid_target_filtered_rad_ = 0.0f;                      // [RO] 航向 PID 目标低通后的角度
            bool lock_yaw_pid_deadband_active_ = false;                        // [RO] 航向 PID 双阈值死区当前是否激活

            // 整车门控与保持态锁存：
            // 这一组负责描述“某种过渡是否已经进入、进入后何时退出”，
            // 包括 X-Park、launch-hold、drive zero-stop、平移方向冻结和 reverse intent。
            // 它们主要与 shouldActivateLaunchHold() / shouldSuppressYawLockOmegaForZeroStopDecel() / applyModuleCommands()
            // 以及 WheelConfig 内部的 X-Park / homing / fault 局部状态机协同，让整车过渡拥有明确滞回与记忆。
            bool xpark_gate_active_ = false;                                   // [RO] X-Park 是否已锁存。未锁存进入看 target+residual；锁存后退出只看 target。
            u32 xpark_stationary_hold_ms_ = 0U;                                // [RO] X-Park 进入条件连续成立时长（ms）。只用于进入延时，不表示保持态 residual 健康。
            bool launch_hold_active_ = false;                                  // [RO] 静止起步整车等待门控是否激活。激活时先只转舵，不放驱动与车体速度规划。
            bool drive_zero_stop_active_ = false;                              // [RO] drive zero-stop 目标门是否已激活。true 时目标速度仍在 near-zero 保持区内。
            bool drive_zero_stop_brake_active_[4] = {false, false, false, false}; // [RO] 各轮 zero-stop 末端是否仍在 brake。active=true 且本值=false 表示该轮 residual 已按 NearZero 判稳并切到零电流。
            u32 drive_zero_stop_brake_ramp_elapsed_ms_[4] = {0U, 0U, 0U, 0U};  // [RO] 各轮 zero-stop brake ramp 已累计时长（ms）。进入 brake 后增长，退出目标门时清零。
            bool trans_dir_freeze_active_ = false;                              // [RO] 平移方向冻结门控当前状态。true 时方向保持参考角，只放行速度模长变化。
            bool trans_dir_ref_valid_ = false;                                  // [RO] 平移方向参考角是否有效。无效时先用当前指令方向建立参考。
            f32 trans_dir_ref_rad_ = 0.0f;                                      // [RO] 平移方向参考角（rad）。用于冻结保持与方向角速率限幅。
            f32 trans_dir_tar_mag_m_s_ = 0.0f;                                  // [RO] 平移输入目标速度模长缓存（m/s）。
            f32 trans_dir_out_mag_m_s_ = 0.0f;                                  // [RO] 平移规划输出速度模长缓存（m/s）。
            u8 trans_dir_freeze_reason_ = 0U;                                    // [RO] 冻结原因缓存：0=none,1=enter,2=hold。
            bool reverse_intent_active_ = false;                                 // [RO] 当前是否判定为近似反向意图。
            f32 reverse_intent_dir_err_deg_ = 0.0f;                              // [RO] 当前目标方向与参考方向夹角（deg）。
            bool steer_fault_any_active_ = false;

            // =====================================================================
            // 控制链路快照与输入缓存 [RO]
            // 快照关注“这一拍最后算出了什么”，缓存关注“下一拍继续算时还需要记住什么”。
            // 这组字段一起构成 chassis 主循环的中间真相：从输入意图、到车体级规划、到模块级 planner 输出、再到执行前命令帧，
            // 都能在这里找到对应镜像，从而把“上游想做什么”和“本拍最终准备怎么做”区分开。
            // =====================================================================
            // 这一小组是整形器自身运行态：保存 jerk profile 和单轮 planner 在跨拍时要延续的内部历史。
            ManualSpeedProfileMode active_manual_speed_profile_mode_ = ManualSpeedProfileMode::kLegacy;
            JerkLimitedAxisState manual_vel_x_shape_state_{};
            JerkLimitedAxisState manual_vel_y_shape_state_{};
            JerkLimitedAxisState manual_omega_z_shape_state_{};
#if JIA_CHASSIS_ENABLE_SINGLE_WHEEL_DEBUG
            SingleWheelAxisPlannerRuntime single_wheel_steer_planner_state_{};
            SingleWheelAxisPlannerRuntime single_wheel_drive_planner_state_{};
#endif
            InputTargetData input_target_data_; // [RO] 输入目标快照（模式与期望速度/角度）
            NormalizedBodyCommand normalized_body_command_; // [RO] 输入来源与统一车体系语义
            Data target_data_;                  // [RO] 模式映射后的目标数据。仍是“上游想让底盘做什么”的统一表达。
            Data planned_data_;                 // [RO] 本拍目标镜像。以 planner 结果为主，但 apply 层可能进一步改写其中 drive/steer 可执行部分。
            Data last_planned_data_;            // [RO] 上一周期规划数据（用于加速度约束）
            Data current_data_;                 // [RO] 面向外部读取的融合视图：轮角/轮速用实时反馈回填，整车 twist 仅在 all_homed 且无 steer fault 时可信。
            SwervePlannerOutput planner_output_cache_; // [RO] 最近一次完整 planner 输出。保留“理想值/规划值/最终 planner 值”全套中间量，便于调试回看。
            SwervePlannerOutput launch_hold_preview_cache_; // [RO] 静止起步门控预演输出。帮助先验证“只转舵不放驱动”时模块会怎样收敛。
            ActuatorCommandFrame actuator_command_frame_; // [RO] 最近一次准备交给执行层仲裁的命令帧。drive 仍是执行门控前目标，steer_cmd 则代表执行层优先采用的舵向命令。
            ModeFlag current_mode_flag_;        // [RO] 当前控制模式标志位

            // 传感器与外部输入快照 [RO]
            // 这组只描述输入侧观测，不直接代表控制决策。
            // 它们服务 resolvePlannerTargetData()、遥控映射和 yaw lock 输入解释，不应与 current_data_ 这类控制结论混淆。
            f32 input_hwt_rot_z_ = 0.0f;   // [RO] IMU yaw
            f32 input_hwt_omega_z_ = 0.0f; // [RO] IMU yaw speed
            communication::RC10_AirJoy_Data_S airjoy_data_{}; // [RO] 遥控器输入快照

            // 调试镜像（只读观察）[RO]
            // DebugMirror 是给调试器和 host FULL_DEBUG 语义测试读的“聚合视图”。
            // RUNTIME_MIN 下不再维护这份镜像，运行代码直接读取真实控制状态即可。
#if JIA_CHASSIS_ENABLE_DEBUG_MIRROR
            // DebugMirror 是给调试器和 host FULL_DEBUG 语义测试读的“聚合视图”。
            // 它和上面的真实运行态不同，目标是把“分散在多个字段里的关键结论”整理成更容易读的镜像。
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
                bool xpark_steer_hold_enable = false;                  // [RO] 当前是否启用统一 X-Park 舵向 hold。
                f32 xpark_steer_hold_entry_deg = 0.0f;                 // [RO] 当前生效的 X-Park 舵向 hold 进入阈值（deg）。
                f32 xpark_steer_hold_exit_deg = 0.0f;                  // [RO] 当前生效的 X-Park 舵向 hold 退出阈值（deg）。
                f32 xpark_steer_hold_settle_deg = 0.0f;                // [RO] 当前生效的 X-Park 舵向 hold 判稳角误差阈值（deg）。
                f32 xpark_steer_hold_settle_target_rate_deg_s = 0.0f;  // [RO] 当前生效的 X-Park 舵向 hold 判稳目标角速度阈值（deg/s）。
                f32 xpark_steer_hold_settle_hold_ms = 0.0f;            // [RO] 当前生效的 X-Park 舵向 hold 判稳保持时长（ms）。
                f32 xpark_steer_hold_reacquire_hold_ms = 0.0f;         // [RO] 当前生效的 X-Park 舵向 hold 重新锁定等待时长（ms）。
                bool xpark_steer_hold_entry_reset_enable = false;      // [RO] 当前生效的 X-Park 舵向 hold 进入时是否清理速度环历史。
                bool reverse_intent_active = false;
                f32 reverse_intent_dir_err_deg = 0.0f;
                u8 single_wheel_target_index = 0U;
                bool single_wheel_isolation_active = false;
                bool single_wheel_non_target_zeroed[4] = {false, false, false, false};
                bool steer_fault_active[4] = {false, false, false, false};
                bool steer_fault_recovering[4] = {false, false, false, false};
                bool steer_fault_control_intent[4] = {false, false, false, false};
                bool steer_fault_xpark_stationary_hold[4] = {false, false, false, false};
                bool steer_fault_freeze_candidate[4] = {false, false, false, false};
                f32 steer_feedback_current_mA[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                f32 steer_feedback_current_delta_mA[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                f32 steer_feedback_angle_delta_rad[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                u8 xpark_steer_hold_phase[4] = {0U, 0U, 0U, 0U};                       // [RO] 四个舵轮当前 X-Park 舵向 hold 状态机阶段。
                bool xpark_steer_hold_locked[4] = {false, false, false, false};       // [RO] 四个舵轮当前是否已经进入 X-Park hold 锁定阶段。
                f32 xpark_steer_hold_error_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f};         // [RO] 四个舵轮相对 X-Park 理想目标角的绝对误差（deg）。
                f32 xpark_steer_hold_target_rate_deg_s[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [RO] 四个舵轮 hold 判稳使用的目标角速度绝对值（deg/s）。
                f32 xpark_steer_hold_settle_ms[4] = {0.0f, 0.0f, 0.0f, 0.0f};         // [RO] 四个舵轮满足 hold 判稳条件后的累计保持时长（ms）。
                f32 xpark_steer_hold_reacquire_ms[4] = {0.0f, 0.0f, 0.0f, 0.0f};      // [RO] 四个舵轮重新允许零电流锁定前的剩余等待时长（ms）。
                f32 steer_fault_steer_error_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                f32 steer_feedback_current_freeze_ms[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                f32 steer_feedback_recovery_toggle_count[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                f32 steer_fault_latched_count[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                bool steer_fault_any_active = false;
            } debug_mirror_;
#endif

            // 线程执行耗时统计（调试器只读观察）[RO]
            // 这块包含 500 点短窗采样，是 Chassis 对象里最大的调试缓存之一。
            // RUNTIME_MIN 下默认不编译它；需要在调试器里看 1ms 线程预算/分段耗时时，切到 FULL_DEBUG。
#if JIA_CHASSIS_ENABLE_TASK_PERF_STAT
            struct TaskPerfStat
            {
                // WindowState 是线程耗时短窗统计的内部 O(1) 运行态：
                // 它维护一个固定长度的环形缓冲和窗口和，用于在 1ms 主循环里低成本得到最近均值。
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
                u64 plan_us = 0ULL;          // [RO] 最近一次规划阶段耗时。
                u64 feedback_us = 0ULL;      // [RO] 最近一次反馈刷新阶段耗时。
                u64 homing_us = 0ULL;        // [RO] 最近一次 homing / fault 状态机阶段耗时。
                u64 apply_us = 0ULL;         // [RO] 最近一次模块执行仲裁与下发阶段耗时。
                u64 debug_us = 0ULL;         // [RO] 最近一次调试镜像/输出阶段耗时。
            } task_perf_stat_;
#endif

            // 调试串口对象（一般不在调试器改动）[RO]
#if JIA_CHASSIS_ENABLE_DEBUG_OUTPUT
            Debug_Printf debug_uart_ = Debug_Printf(&huart8); // [RO]
#endif
        };

        using Result = jia::FourSteerChassis::Chassis::Result;

        inline Result Chassis::setZeroCurrent()
        {
            input_target_data_.zero_current_all = true;
            return Result::kOk;
        }

        inline Result Chassis::setSpeed(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z)
        {
            // 第一层语义转换发生在 API 边界：
            // 对外/public 坐标约定与底盘内部 body 语义的 x/y 正方向不同，因此先经 mapExternalCommandToBody() 做一次翻转。
            const BodyCommand body_command = mapExternalCommandToBody({coord, vel_x, vel_y, omega_z});
            return (coord == Coordinate::kBody) ? setTargetBodySpeedMode(body_command.vel_x, body_command.vel_y, body_command.omega_z)
                                                : setTargetWorldSpeedMode(body_command.vel_x, body_command.vel_y, body_command.omega_z);
        }

        inline Result Chassis::setSpeed_LockNowYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z)
        {
            // LockNowYaw 的公开接口仍允许带入手动 omega_z：
            // 当用户仍在主动旋转时，omega_z 会直接参与；松手后才切到“锁住当前 yaw”的自动保持语义。
            const BodyCommand body_command = mapExternalCommandToBody({coord, vel_x, vel_y, omega_z});
            return (coord == Coordinate::kBody) ? setTargetBodySpeedLockNowRotZWithNoOmegaZMode(body_command.vel_x, body_command.vel_y, body_command.omega_z)
                                                : setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(body_command.vel_x, body_command.vel_y, body_command.omega_z);
        }

        inline Result Chassis::setSpeed_LockToYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 rot_z)
        {
            // LockToYaw 的 yaw 目标单独传入 rot_z，平移部分仍沿公开坐标约定映射到内部 body 语义。
            const BodyCommand body_command = mapExternalCommandToBody({coord, vel_x, vel_y, 0.0f});
            return (coord == Coordinate::kBody) ? setTargetBodySpeedLockToRotZMode(body_command.vel_x, body_command.vel_y, rot_z)
                                                : setTargetWorldSpeedLockToRotZMode(body_command.vel_x, body_command.vel_y, rot_z);
        }

        inline Chassis::BodyCommand Chassis::mapExternalCommandToBody(const ExternalCommand &command)
        {
            // 这里解决的是“公开 API 说的前/左”和“底盘内部 body 命令正方向”之间的约定差异。
            // 它不关心 planner 习惯，只负责把外部调用方的语义先翻译成底盘控制层读得懂的 body 命令。
            BodyCommand body_command;
            body_command.vel_x = -command.vel_x;
            body_command.vel_y = -command.vel_y;
            body_command.omega_z = command.omega_z;
            return body_command;
        }

        inline Chassis::BodyCommand Chassis::normalizeBodyCommandForPlanner(const BodyCommand &command)
        {
            // 第二层语义转换发生在 planner 边界：
            // planner 沿用的是更早的一套内部轴向约定，因此在 body 命令进入 planner 前还要再做一次 x/y 翻转。
            // 看起来像重复取反，但目的不同：前者面向外部 API，后者面向内部规划器。
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
                const f32 cos_theta = cosRadF32(input_yaw_rad);
                const f32 sin_theta = sinRadF32(input_yaw_rad);
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
            case 9:
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
                return DebugModuleOverrideRoute::kSingleWheelIsolated;
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
            // getter 返回的是“公开 API 语义下的目标速度快照”。
            // 因为内部 target 仍保留底盘/规划层使用的方向约定，所以这里再翻回外部 readback 语义。
            Robot_Twist body_speed;
            const f32 internal_vx = getTargetBodyVelX();
            const f32 internal_vy = getTargetBodyVelY();
            body_speed.vx = -internal_vx;
            body_speed.vy = -internal_vy;
            body_speed.vz = getTargetOmegaZ();
            return body_speed;
        }

        inline Robot_Twist Chassis::getWorldSpeed() const
        {
            // 世界系 getter 同样返回目标快照而非实际反馈；
            // world/body 的区别只体现在目标已经过坐标变换，而这里仍要把内部方向约定翻回公开 readback 语义。
            Robot_Twist world_speed;
            const f32 internal_vx = getTargetWorldVelX();
            const f32 internal_vy = getTargetWorldVelY();
            world_speed.vx = -internal_vx;
            world_speed.vy = -internal_vy;
            world_speed.vz = getTargetOmegaZ();
            return world_speed;
        }
    }
}

using jia::FourSteerChassis::Chassis;

#endif // CHASSIS_H_
