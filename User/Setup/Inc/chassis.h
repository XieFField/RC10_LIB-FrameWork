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
    namespace ThreeOmniChassis
    {
        class Chassis
        {
        public:
            /* ----------------------------------------------------------------- */
            // 对外控制接口
            // 注意：当前四舵轮 chassis 在本文件内已经有独立初始化/控制分支；
            // 但工程上层 FSM 仍然绑定 ChassisOmni。这里的接口只是“头文件已支持四舵轮”，
            // 不代表整条接线或状态机切换已经完成。
            //  // 枚举类型定义
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
            //  // 设置电流为0
            Result setZeroCurrent();
            //  // 设置速度
            Result setSpeed(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z);
            Result setSpeed_LockNowYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z = 0.0f);
            Result setSpeed_LockToYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 rot_z);
            //  // 读取速度
            Robot_Twist getBodySpeed() const;
            Robot_Twist getWorldSpeed() const;
            /* ----------------------------------------------------------------- */

            struct InitConfig
            {
                M3508 *motor_handle[3];
            };

            // 默认构造和析构函数
            Chassis() = default;
            ~Chassis() = default;

            // 初始化
            void init(InitConfig &config);

            // 设置轮子扭矩自由模式
            Result setWheelTorqueFreeMode();
            // 设置目标速度模式
            //  // 自身坐标系
            //  //  // 速度
            Result setTargetBodySpeedMode(f32 vel_x, f32 vel_y, f32 omega_z);
            //  //  // 固定当前rot_z
            Result setTargetBodySpeedLockNowRotZMode(f32 vel_x, f32 vel_y);
            //  //  // 无输入omega_z时固定当前rot_z
            Result setTargetBodySpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z = 0.0f);
            //  //  // 固定到rot_z
            Result setTargetBodySpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z);
            //  // 世界坐标系
            //  //  // 速度
            Result setTargetWorldSpeedMode(f32 vel_x, f32 vel_y, f32 omega_z);
            //  //  // 固定当前rot_z
            Result setTargetWorldSpeedLockNowRotZMode(f32 vel_x, f32 vel_y);
            //  //  // 无输入omega_z时固定当前rot_z
            Result setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z = 0.0f);
            //  //  // 固定到rot_z
            Result setTargetWorldSpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z);
            // 读取目标速度
            f32 getTargetBodyVelX() const;
            f32 getTargetBodyVelY() const;
            f32 getTargetWorldVelX() const;
            f32 getTargetWorldVelY() const;
            f32 getTargetOmegaZ() const;
            // 读取当前速度
            f32 getCurrentBodyVelX() const;
            f32 getCurrentBodyVelY() const;
            f32 getCurrentWorldVelX() const;
            f32 getCurrentWorldVelY() const;
            f32 getCurrentOmegaZ() const;

        private:
            struct WheelConfig
            {
                f32 pos_x;                     // 单位：米
                f32 pos_y;                     // 单位：米
                f32 rot_z_deg;                 // 单位：度
                M3508 *motor_handle = nullptr; // 电机句柄
                f32 sin_rot_z;
                f32 cos_rot_z;
                f32 eq_radius;     // 等效半径，equivalent radius，可以是负值，单位：米
                f32 abs_sin_rot_z; // 正弦值的绝对值
                f32 abs_cos_rot_z; // 余弦值的绝对值
                f32 abs_eq_radius; // 等效半径的绝对值，单位：米

                M3508 *&h = motor_handle;
                f32 &s = sin_rot_z;
                f32 &c = cos_rot_z;
                f32 &eqr = eq_radius;
                f32 &as = abs_sin_rot_z;
                f32 &ac = abs_cos_rot_z;
                f32 &aeqr = abs_eq_radius;
            };

            enum class Mode
            {
                kWheelTorqueFreeMode,
                kBodySpeedMode,
                kWorldSpeedMode,
                kBodySpeedLockNowRotZMode,
                kWorldSpeedLockNowRotZMode,
                kBodySpeedLockToRotZMode,
                kWorldSpeedLockToRotZMode,
                kBodySpeedLockNowRotZWithNoOmegaZMode,
                kWorldSpeedLockNowRotZWithNoOmegaZMode,
            };

            struct ModeFlag
            {
                bool is_wheel_torque_free; // 是否为轮子扭矩自由模式
                // Coordinate coord = Coordinate::kBody; // 速度坐标系
                bool is_world_speed_mode; // 是否为世界坐标系速度模式
                bool is_lock_now_rot_z;   // 是否固定当前rot_z
                bool is_lock_to_rot_z;    // 是否固定到rot_z
            };

            struct InputTargetData
            {
                f32 vel_x;
                f32 vel_y;
                f32 omega_z;
                f32 rot_z;

                Mode mode;
            };

            struct Data
            {
                f32 vel_x;   // x轴速度，单位：米/秒
                f32 vel_y;   // y轴速度，单位：米/秒
                f32 omega_z; // z轴角速度，单位：rad/s

                f32 acc_x;   // x轴加速度，单位：米/秒^2
                f32 acc_y;   // y轴加速度，单位：米/秒^2
                f32 alpha_z; // z轴角加速度，单位：rad/s^2

                f32 rot_z;

                f32 w1_omega; // 轮子1的角速度，单位：rad/s
                f32 w2_omega; // 轮子2的角速度，单位：rad/s
                f32 w3_omega; // 轮子3的角速度，单位：rad/s

                f32 w1_alpha; // 轮子1的角加速度，单位：rad/s^2
                f32 w2_alpha; // 轮子2的角加速度，单位：rad/s^2
                f32 w3_alpha; // 轮子3的角加速度，单位：rad/s^2
            };

            // 创建线程
            static void createThread(void *arg);
            // 运行线程函数
            void runThread(void *arg);

            // 输入目标数据
            InputTargetData input_target_data_;
            // rot_z速度/位置环pid输出的omega_z
            f32 target_pid_omega_z;
            // 目标数据
            Data target_data_;
            // 规划数据
            Data planned_data_;      // 规划数据
            Data last_planned_data_; // 上一次规划数据
            // 当前数据
            Data current_data_;

            // IMU数据
            f32 input_hwt_rot_z_;
            f32 input_hwt_omega_z_;

            // 模式标志位
            //  // 当前模式标志位
            ModeFlag current_mode_flag_;
            ModeFlag &cmf_ = current_mode_flag_;
            //  // 上一时刻模式标志位
            // ModeFlag last_mode_flag_;
            // ModeFlag &lmf_ = last_mode_flag_;

            // 系统参数
            constexpr static u8 period_ms_ = 1;                  // 控制周期，单位：毫秒
            TickType_t time_ms_;                                 // 当前时间，单位：毫秒
            constexpr static f32 period_ = period_ms_ / 1000.0f; // 控制周期，单位：秒

            // 底盘参数
            //  // 轮子半径
            constexpr static f32 wheel_radius_ = 0.075f; // 轮子半径（单位：米）
            const f32 &wr_ = wheel_radius_;
            //  // 轮子配置
            WheelConfig wheel_config_[3];
            WheelConfig &w1_ = wheel_config_[0];
            WheelConfig &w2_ = wheel_config_[1];
            WheelConfig &w3_ = wheel_config_[2];

            // 速度限制参数
            //  // 轮端速度
            bool is_wheel_omega_limit_ = true;           // 是否进行轮端角速度限制
            f32 max_wheel_omega_ = rpmToRadsF32(400.0f); // 最大轮子角速度，单位：rad/s
            f32 max_wheel_vel_ = 0.0f;                   // 最大轮子线速度，单位：米/秒

            //  // 车端速度
            //  //  // 速度比例系数
            f32 max_vel_x_radio_ = 1.0f;   // x轴速度比例系数
            f32 max_vel_y_radio_ = 1.0f;   // y轴速度比例系数
            f32 max_omega_z_radio_ = 1.0f; // z轴角速度比例系数
            //  //  // 最大速度
            f32 max_vel_x_ = 0.0f;   // 最大x轴速度，单位：米/秒
            f32 max_vel_y_ = 0.0f;   // 最大y轴速度，单位：米/秒
            f32 max_omega_z_ = 0.0f; // 最大z轴角速度，单位：rad/s

            // 加速度限制参数
            //  // 车端加速度
            bool is_chassis_acc_limit_ = true; // 是否进行车端加速度限制
            f32 max_acc_xy_acc_ = 2.0f;        // 最大XY轴线加速度，单位：m/s^2
            f32 max_acc_xy_dec_ = 10.0f;       // 最大XY轴线减速度，单位：m/s^2
            f32 max_alpha_z_acc_ = 4.0f;       // 最大z轴角加速度，单位：rad/s^2
            f32 max_alpha_z_dec_ = 6.0f;       // 最大z轴角减速度，单位：rad/s^2
            //  // 轮端角加速度
            bool is_wheel_alpha_limit_ = false; // 是否进行轮端角加速度限制
            f32 max_wheel_alpha_ = 2.0f * kPi;  // 最大轮子角加速度，单位：rad/s^2

            // rot_z轴PID参数
            //   // 速度环PID
            bool is_omega_z_close_loop_ = false;
            PID_Incremental omega_z_pid_;
            u8 omega_z_pid_period_ = 1;
            u8 omega_z_pid_count_ = 0;
            //   // 位置环PID
            PID_Position rot_z_pid_;
            u8 rot_z_pid_period_ = 1;
            u8 rot_z_pid_count_ = 0;
            f32 max_lock_to_rot_z_radio_ = 1.0f;      // 最大固定到rot_z转动系数，单位：rad/s
            u32 lock_now_rot_z_shift_count_ = 0.;     // 固定当前rot_z计数器
            u32 lock_now_rot_z_shift_time_ms_ = 1000; // 固定当前rot_z缓冲时间，单位：毫秒

            // 调试参数
            bool is_debug_ = false; // 是否开启调试模式
            u8 debug_mode_ = 0;     // 调试模式

            u8 debug_wheel_index_ = 2;    // 调试轮子索引
            f32 debug_input_ = 90.0f;     // 调试输入
            f32 debug_lock_rot_z_ = 0.0f; // 调试固定rot_z

            bool is_step_signal_ = false; // 是否使用阶跃信号

            bool is_sine_ = false; // 是否使用正弦信号
            f32 sine_amplitude_ = 0.0f;
            f32 sine_frequency_ = 0.1f;
            f32 sine_offset_ = 0.0f;

            bool is_wheel_speed_mode_ = false;   // 是否为轮子速度模式
            bool is_wheel_current_mode_ = false; // 是否为轮子电流模式

            Debug_Printf debug_uart_ = Debug_Printf(&huart8); // 调试串口
            u8 printf_period_ms_ = 5;                         // 串口调试打印周期，单位：毫秒
            u8 printf_period_count_ = 0;                      // 串口调试打印周期计数器

            RmPocketData_t airjoy_data_;

        private:
            void isDebugMode();
            void setModeFlag();

            void clearInputTargetData();

        private:
            void inverseKinematics(f32 in_x, f32 in_y, f32 in_z, f32 &out_w1, f32 &out_w2, f32 &out_w3);

            void transSpeedBodyToWorld(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y);
            void transSpeedWorldToBody(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y);

            void isLockNowRotZ(bool is_lock, f32 rot_z, f32 omega_z, f32 &out_rot_z, f32 &out_omega_z);
            void isLockToRotZ(bool is_lock, f32 tar_rot_z, f32 pla_rot_z, f32 &out_rot_z, f32 omega_z, f32 &out_omega_z);

            void isTransSpeedBodyToWorld(bool is_trans, f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y);
            void isTransSpeedWorldToBody(bool is_trans, f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y);

            void calculatePid(PID_Incremental &pid, u8 &count, u8 period, f32 target, f32 feedback, f32 &output);
            void calculatePid(PID_Position &pid, u8 &count, u8 period, f32 target, f32 feedback, f32 &output);

            void initWheelConfig(WheelConfig &wheel, f32 pos_x, f32 pos_y, f32 rot_z_deg, M3508 *motor_handle = nullptr);

            void setWheelTargetCurrent(WheelConfig &wheel, f32 current);
            void setWheelTargetOmega(WheelConfig &wheel, f32 omega);
            f32 getWheelCurrentOmega(const WheelConfig &wheel) const;
            f32 getWheelTargetCurrent(const WheelConfig &wheel) const;
            f32 getWheeCurrentCurrent(const WheelConfig &wheel) const;
            f32 getWheelCurrentRpm(const WheelConfig &wheel) const;

            void clampTargetSpeedInChassis(f32 vel_x, f32 vel_y, f32 omega_z, f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z);

            void isLimitAccInChassis(bool is_limit,
                                     f32 tar_vel_x, f32 tar_vel_y, f32 tar_omega_z,
                                     f32 cur_vel_x, f32 cur_vel_y, f32 cur_omega_z,
                                     f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z);

            void clearData(Data &data);
        };

        inline Chassis::Result Chassis::setZeroCurrent()
        {
            return setWheelTorqueFreeMode();
        }

        inline Chassis::Result Chassis::setSpeed(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z)
        {
            if (coord == Coordinate::kBody)
            {
                return setTargetBodySpeedMode(vel_x, vel_y, omega_z);
            }
            else
            {
                return setTargetWorldSpeedMode(vel_x, vel_y, omega_z);
            }
        }

        inline Chassis::Result Chassis::setSpeed_LockNowYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z)
        {
            if (coord == Coordinate::kBody)
            {
                return setTargetBodySpeedLockNowRotZWithNoOmegaZMode(vel_x, vel_y, omega_z);
            }
            else
            {
                return setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(vel_x, vel_y, omega_z);
            }
        }

        inline Chassis::Result Chassis::setSpeed_LockToYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 rot_z)
        {
            if (coord == Coordinate::kBody)
            {
                return setTargetBodySpeedLockToRotZMode(vel_x, vel_y, rot_z);
            }
            else
            {
                return setTargetWorldSpeedLockToRotZMode(vel_x, vel_y, rot_z);
            }
        }

        inline Robot_Twist Chassis::getBodySpeed() const
        {
            Robot_Twist body_speed;
            body_speed.vx = getTargetBodyVelX();
            body_speed.vy = getTargetBodyVelY();
            body_speed.vz = getTargetOmegaZ();
            return body_speed;
        }

        inline Robot_Twist Chassis::getWorldSpeed() const
        {
            Robot_Twist world_speed;
            world_speed.vx = getTargetWorldVelX();
            world_speed.vy = getTargetWorldVelY();
            world_speed.vz = getTargetOmegaZ();
            return world_speed;
        }
    }

    namespace FourSteerChassis
    {
        class Chassis
        {
        public:
            /* ----------------------------------------------------------------- */
            // 对外控制接口
            //  // 枚举类型定义
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

            // 驱动抑制策略：用于在舵角误差较大时降低驱动轮放行比例。
            enum class DriveGateStrategy : u8
            {
                kHardGate = 0,
                kSoftGate = 1,
                kContinuousCurve = 2,
                kAdaptiveGate = 3,
            };

            // 驱动抑制作用域：整车统一缩放或按轮单独缩放。
            enum class DriveGateScope : u8
            {
                kGlobal = 0,
                kPerWheel = 1,
            };

            // 停车转向保护策略：用于“指令静止但驱动残速未消失”时抑制舵角突变。
            enum class StopSteerGuardStrategy : u8
            {
                kHardHold = 0,
                kSoftBlend = 1,
                kContinuousBlend = 2,
            };

            // AdaptiveGate 运行相位状态，主要用于调试与运行态观测。
            enum class AdaptiveGatePhase : u8
            {
                kIdle = 0,
                kStartHold = 1,
                kTransition = 2,
                kContinuous = 3,
                kLegacy = 4,
                kDisabled = 5,
            };

            // 回零状态机：描述单个舵轮执行 homing 的内部过程。
            // 这些状态只服务于四舵轮底盘内部寻零流程，不代表上层 FSM 状态。
            enum class HomingState : u8
            {
                kIdle,                 // 空闲态：等待 startHoming() 发起回零请求
                kSearch,               // 搜索态：驱动舵轮寻找零位传感器边沿
                kEdgeDetected,         // 已检测到零位边沿，准备锁存当前位置
                kOffsetApply,          // 应用零位偏移，将原始角度对齐到运行时零点
                kContinuousAngleReady, // 连续角度已可用，准备切入正常闭环控制
                kReady,                // 回零完成，允许该轮参与正常控制
                kAlignToZero,          // 归位态：按已建立零偏转到软件零点（OA=0）后再标记完成
                kFault,                // 回零失败/超时，当前轮未完成零位建立
            };

            //  // 设置电流为0
            Result setZeroCurrent();
            //  // 设置速度
            // 这些接口是上层最常用的调用入口：先按坐标系分流，再进入对应的控制模式。
            Result setSpeed(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z);
            // LockNowYaw：锁住“当前 yaw/rot_z”，只改变平移目标；omega_z 参数保留给调用侧兼容使用。
            Result setSpeed_LockNowYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z = 0.0f);
            // LockToYaw：锁到显式给定的目标 yaw/rot_z，常用于对准固定航向。
            Result setSpeed_LockToYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 rot_z);
            //  // 读取速度
            // 返回的是“当前目标速度视图”，不是电机实时反馈；适合上层查看最近一次下发意图。
            Robot_Twist getBodySpeed() const;
            Robot_Twist getWorldSpeed() const;
            /* ----------------------------------------------------------------- */

            // 默认构造和析构函数
            Chassis() = default;
            ~Chassis() = default;

            // 设置轮子扭矩自由模式
            Result setWheelTorqueFreeMode();
            // 设置目标速度模式
            //  // 自身坐标系
            //  //  // 速度
            Result setTargetBodySpeedMode(f32 vel_x, f32 vel_y, f32 omega_z);
            //  //  // 固定当前rot_z
            Result setTargetBodySpeedLockNowRotZMode(f32 vel_x, f32 vel_y);
            //  //  // 无输入omega_z时固定当前rot_z
            Result setTargetBodySpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z = 0.0f);
            //  //  // 固定到rot_z
            Result setTargetBodySpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z);
            //  // 世界坐标系
            //  //  // 速度
            Result setTargetWorldSpeedMode(f32 vel_x, f32 vel_y, f32 omega_z);
            //  //  // 固定当前rot_z
            Result setTargetWorldSpeedLockNowRotZMode(f32 vel_x, f32 vel_y);
            //  //  // 无输入omega_z时固定当前rot_z
            Result setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z = 0.0f);
            //  //  // 固定到rot_z
            Result setTargetWorldSpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z);
            // 读取目标速度
            f32 getTargetBodyVelX() const;
            f32 getTargetBodyVelY() const;
            f32 getTargetWorldVelX() const;
            f32 getTargetWorldVelY() const;
            f32 getTargetOmegaZ() const;
            // 读取当前速度
            f32 getCurrentBodyVelX() const;
            f32 getCurrentBodyVelY() const;
            f32 getCurrentWorldVelX() const;
            f32 getCurrentWorldVelY() const;
            f32 getCurrentOmegaZ() const;

        public:
            // WheelInitConfig 是“单轮初始化描述”，用于填写每个轮组的几何位置、安装朝向、
            // 电机极性，以及是否启用和如何执行回零。
            // 典型回零配置示例：
            // .homing_enabled = true,
            // .homing_sensor_active_high = true,   // 传感器触发时如果输出高电平，就填 true；触发时低电平就填 false
            // .homing_gpio_port = GPIOB,           // 或某个 *_GPIO_Port 宏
            // .homing_gpio_pin = GPIO_PIN_12,      // 或某个 *_Pin 宏
            // .homing_zero_offset_deg = 0.0f,      // 先填 0 跑通，再按实物零位偏差回填补偿角
            struct WheelInitConfig
            {
                f32 pos_x_m = 0.0f;                 // 轮模块相对底盘中心的 X 坐标，单位米
                f32 pos_y_m = 0.0f;                 // 轮模块相对底盘中心的 Y 坐标，单位米
                f32 theta_oa_to_owi_deg = 0.0f;     // 安装几何偏移：把 OA 朝向换算到转向电机机械零位；它用于坐标变换，不是回零补偿
                f32 steer_motor_sign = 1.0f;        // 转向电机方向符号：1.0f 表示不取反，-1.0f 表示取反，0.0f 在初始化时会被当作 1.0f
                f32 drive_motor_sign = 1.0f;        // 驱动电机方向符号：1.0f 表示不取反，-1.0f 表示取反，0.0f 在初始化时会被当作 1.0f
                bool homing_enabled = false;        // 是否启用该轮回零流程；没有装光电/霍尔零位开关时保持 false
                bool homing_sensor_active_high = true; // 传感器原始 GPIO 输入高电平时若视为“触发有效”就填 true，否则填 false；它只影响“逻辑 active”理解，不决定 H/L 边沿对应的机械角
                void *homing_gpio_port = nullptr;   // 真实光电接入后填 STM32 HAL 的 GPIOA/GPIOB 等端口，或 CubeMX 生成的 *_GPIO_Port 宏
                u16 homing_gpio_pin = 0;            // 真实光电接入后填 GPIO_PIN_x，或 CubeMX 生成的 *_Pin 宏；未接时保持 0
                f32 homing_falling_edge_mech_deg = 60.0f; // 原始 GPIO 从高到低（H->L）那个边沿对应的机械 OA 角，单位 deg；默认是 +60°
                f32 homing_rising_edge_mech_deg = -120.0f; // 原始 GPIO 从低到高（L->H）那个边沿对应的机械 OA 角，单位 deg；默认是 -120°
                f32 homing_search_rpm = 10.0f;      // 回零搜索时给转向电机的转速指令，单位 rpm
                f32 homing_zero_offset_deg = 0.0f;  // 回零补偿角：传感器触发点到期望机械零位的偏差；它在建立零点时生效，不是安装角偏移
                f32 homing_timeout_s = 5.0f;        // 单轮回零超时时间，超时后进入故障态，单位秒
            };

            // InitConfig 只负责硬件句柄绑定，运行参数/标定参数使用 FourSteer 类内默认值。
            struct InitConfig
            {
                Motor_Base *steer_motor_h[4] = {nullptr}; // 4 个转向电机句柄，顺序需与 wheels[4] 的轮位定义保持一致
                Motor_Base *drive_motor_h[4] = {nullptr}; // 4 个驱动电机句柄，顺序需与对应转向模块一一匹配
            };

            // 初始化
            // 这里只负责四舵轮 chassis 对象内部参数装配与状态准备，不意味着上层 FSM 已切到四舵轮链路。
            void init(InitConfig &config);
            Result startHoming();
            bool isHomingDone() const;
            // 运行时切换空闲/失能时的舵轮停靠姿态。
            void setIdlePostureMode(IdlePostureMode mode);
            // 运行时策略切换：优先级高于 InitConfig 默认值，可在比赛中动态调整。
            void setSteeringStrategyMode(SteeringStrategyMode mode);
            void setSteeringFlipHysteresisDeg(f32 enter_angle_deg, f32 exit_angle_deg);
            void setDriveGateEnabled(bool enabled);
            void setDriveGateConfig(DriveGateStrategy strategy, DriveGateScope scope, f32 close_angle_deg, f32 min_scale);
            void setDriveGateCurveParams(f32 curve_half_angle_deg, f32 curve_exponent, f32 curve_min_scale);
            void setDriveGateAdaptiveParams(f32 transition_linear_speed_m_s, f32 transition_angular_speed_rad_s, f32 ramp_up_s, f32 ramp_down_s);
            void setStopSteerGuardEnabled(bool enabled);
            void setStopSteerGuardConfig(StopSteerGuardStrategy strategy, f32 release_speed_m_s, f32 blend_start_speed_m_s, f32 curve_half_speed_m_s, f32 curve_exponent);
            void resetRuntimeStrategyToInitConfig();

        private:
            // WheelConfig 是运行时轮组状态快照：既保存静态几何和硬件句柄，也保存回零状态、
            // 补偿结果与最近一次规划输出，供控制线程在每个周期更新。
            struct WheelConfig
            {
                f32 pos_x_m = 0.0f;                         // 该舵轮模块在车体坐标系中的 x 安装位置，单位米
                f32 pos_y_m = 0.0f;                         // 该舵轮模块在车体坐标系中的 y 安装位置，单位米
                f32 theta_oa_to_owi_rad = 0.0f;             // 安装几何偏移：把舵轮在底盘平面内实际指向/滚动的方向（OA 朝向）换算到转向电机本地机械角参考系（OWI）；它用于坐标变换，不是回零补偿
                f32 steer_motor_sign = 1.0f;                // 转向电机方向符号：1 表示不取反，-1 表示转向反馈和目标指令都按相反方向解释
                f32 drive_motor_sign = 1.0f;                // 驱动电机方向符号：1 表示不取反，-1 表示驱动反馈和目标指令都按相反方向解释
                Motor_Base *steer_motor_h = nullptr;        // 该模块绑定的转向电机句柄
                Motor_Base *drive_motor_h = nullptr;        // 该模块绑定的驱动电机句柄
                bool homing_enabled = false;                // 是否对该轮启用回零流程；false 时默认认为零位已可用
                bool homing_sensor_active_high = true;      // 回零传感器逻辑 active 极性：true 表示高电平视为有效，false 表示低电平视为有效；不决定 H/L 边沿的机械角语义
                void *homing_gpio_port = nullptr;           // 回零传感器 GPIO 端口运行时副本；读取零位输入时直接使用
                u16 homing_gpio_pin = 0;                    // 回零传感器 GPIO 引脚运行时副本；与端口配合读取真实输入
                f32 homing_falling_edge_mech_rad = 0.0f;    // 原始 GPIO H->L 边沿对应的机械 OA 角（rad）
                f32 homing_rising_edge_mech_rad = 0.0f;     // 原始 GPIO L->H 边沿对应的机械 OA 角（rad）
                f32 homing_search_rpm = 10.0f;              // 回零搜索阶段给转向电机的转速指令，单位 rpm
                f32 homing_zero_offset_rad = 0.0f;          // 标定得到的零位补偿角：传感器触发点到期望机械零位的固定偏差
                f32 homing_timeout_s = 5.0f;                // 单轮回零允许持续的最长时间，超时后进入故障态，单位秒
                HomingState homing_state = HomingState::kIdle; // 当前轮回零状态机所处阶段
                bool homing_last_sensor_active = false;     // 上一控制周期的原始 GPIO 高低电平；用于检测 H/L 边沿
                bool homing_last_edge_is_falling = false;   // 最近一次抓到的边沿方向：true=H->L，false=L->H；方便调试极性和触发角
                bool homing_zero_valid = false;             // 当前轮是否已经建立可用于闭环控制的零位
                f32 homing_elapsed_s = 0.0f;                // 本次回零已运行时间，单位秒；用于超时判定
                f32 homing_runtime_zero_offset_rad = 0.0f;  // 本次上电运行实际采用的零位补偿；回零成功后会把“当前触发位置”修正成运行时零点
                f32 corrected_steer_motor_total_angle_rad = 0.0f; // 已乘方向符号并叠加运行时零位补偿后的转向电机连续总角度反馈
                f32 corrected_drive_omega_rad_s = 0.0f;     // 已乘方向符号后的驱动轮角速度反馈，单位 rad/s
                f32 target_steer_motor_total_angle_rad = 0.0f; // 当前周期解算后要发给转向电机的本地连续目标角
                f32 target_drive_omega_rad_s = 0.0f;        // 当前周期解算后要发给驱动电机的目标角速度，单位 rad/s
                f32 steer_target_velocity_rad_s = 0.0f;     // 转向二阶限幅后得到的目标角速度，便于平滑舵向变化
                bool flipped_drive_direction = false;       // 本周期是否采用“舵角翻转 180 度、驱动反向”策略来走更短转角路径
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
            };
            DebugMode resolveDebugMode(u8 raw_mode) const;
            void applyDebugTargetOverride();
            bool applyDebugModuleOverride(bool all_homed);
            void emitDebugOutputByMode(bool all_homed);
            void clearInputTargetData();
            void setModeFlag();
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
            bool solveLinear3x3(f32 matrix[3][4], f32 &x0, f32 &x1, f32 &x2) const;
            bool estimateBodySpeedFromModules(f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z) const;
            void updateTaskPerfStat(u64 loop_start_us, u64 loop_end_us);

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
            f32 wheel_radius_m_ = 0.052f;                     // [RW, 慎改] 轮半径。决定线速度与驱动角速度的换算比例，改错会直接导致速度尺度和里程计比例偏差。
            f32 max_vel_x_ = 999.0f;                          // [RW] 车体 X 方向最大线速度上限（m/s）。用于规划/限幅，不是电机硬件极限。
            f32 max_vel_y_ = 999.0f;                          // [RW] 车体 Y 方向最大线速度上限（m/s）。同上，约束横移速度。
            f32 max_omega_z_ = 999.0f;                        // [RW] 车体 Z 轴最大角速度上限（rad/s）。同上，约束原地旋转或航向变化速度。
            f32 max_acc_xy_acc_ = 9999.0f;                    // [RW] 平面加速段最大加速度（m/s^2）。越小起步越柔和，越大响应越猛。
            f32 max_acc_xy_dec_ = 9999.0f;                    // [RW] 平面减速段最大减速度（m/s^2）。越小刹车越平滑，越大停车越快但冲击更强。
            f32 max_alpha_z_acc_ = 9999.0f;                   // [RW] 航向加速段最大角加速度（rad/s^2）。影响转向起步的平顺性。
            f32 max_alpha_z_dec_ = 9999.0f;                   // [RW] 航向减速段最大角减速度（rad/s^2）。影响转向收尾和停摆冲击。
            f32 max_drive_omega_rad_s_ = 25.0f;               // [RW] 驱动目标角速度上限（rad/s）。最终下发给驱动电机前会做限幅，防止超速命令。
            f32 max_drive_alpha_rad_s2_ = 50.0f;              // [RW] 驱动角速度变化率上限（rad/s^2）。越小越像“缓启动/缓刹车”。
            f32 max_steer_rate_rad_s_ = 40000.0f;             // [RW] 转向目标角速度上限（rad/s）。越大转向越快，但过大容易让舵角命令太激进。
            f32 max_steer_alpha_rad_s2_ = 25000.0f;           // [RW] 转向目标角加速度上限（rad/s^2）。用于限制舵角变化的“突然性”。
            f32 stationary_speed_epsilon_m_s_ = 0.01f;        // [RW] 近似静止阈值（m/s）。低于该速度时可认为底盘处于停车/低速保护区。
            bool enable_cosine_compensation_ = false;          // [RW] 是否启用舵角余弦补偿。开启后，舵角偏离时会折算驱动分量，减小横滑和无效驱动。
            IdlePostureMode idle_posture_mode_ = IdlePostureMode::kHoldLast; // [RW] 静止姿态策略。决定停住后是维持当前轮姿态，还是自动收拢为 X-Park。

            // =====================================================================
            // 策略参数（运行时可调）[RW]
            // 说明：这里控制“怎么解算”“什么时候翻转”“什么时候压驱动”“停车时怎么稳住”。
            // =====================================================================
            struct StrategyConfig
            {
                // ---- 舵角解算 ----------------------------------------------------
                // 决定每个模块在“直接转过去”与“翻转 180° 再配合驱动反向”之间如何选择。
                // 这个选择直接影响转向路径长度、驱动方向是否反转，以及模块在大角度切换时是否抖动。
                SteeringStrategyMode steering_strategy_mode = SteeringStrategyMode::kShortestPath;
                f32 flip_enter_angle_deg = 100.0f; // [RW] 翻转进入阈值（deg）。角差大于该值时，策略才允许“舵角+180°并反转驱动”。
                f32 flip_exit_angle_deg = 80.0f;   // [RW] 翻转退出阈值（deg）。必须小于进入阈值，用于形成滞回，避免临界角附近来回翻转。

                // ---- 驱动抑制（Drive Gate） -------------------------------------
                // 当舵角还没对准时，按策略压低驱动输出，减少横滑、打滑和轮子“边转边拖”的冲击。
                // 适合大角度转向、起步前对轮、低速细调等场景。
                bool enable_drive_gate = true; // [RW] 是否启用驱动门控。关闭后驱动不再因舵角误差被额外压低。
                DriveGateStrategy drive_gate_strategy = DriveGateStrategy::kHardGate; // [RW] 门控形状：硬门控更直接，曲线门控更平滑。
                DriveGateScope drive_gate_scope = DriveGateScope::kGlobal;             // [RW] 门控作用范围：全局统一收紧，或按单轮分别计算。
                f32 drive_gate_close_angle_deg = 1.0f;                                  // [RW] 关闭区角差阈值（deg）。超过该误差后会进入更强的驱动抑制。
                f32 drive_gate_min_scale = 0.5f;                                        // [RW] 硬门控最小缩放。0 表示可完全关断驱动，1 表示完全不压。
                f32 drive_gate_curve_exponent = 3.0f;                                   // [RW] 曲线门控指数。越大曲线越“硬”，越接近临界开关。
                f32 drive_gate_curve_half_angle_deg = 3.0f;                             // [RW] 曲线门控半效角（deg）。大致决定从“明显抑制”到“基本放开”的过渡宽度。
                f32 drive_gate_curve_min_scale = 0.0f;                                  // [RW] 曲线门控保底缩放。即使误差很大，也至少保留多少驱动比例。
                f32 drive_gate_transition_linear_speed_m_s = 0.10f;                     // [RW] 线速度过渡阈值（m/s）。速度越低，门控越容易收紧，减少静止附近的横向冲击。
                f32 drive_gate_transition_angular_speed_rad_s = 0.10f;                  // [RW] 角速度过渡阈值（rad/s）。自转越慢，门控越偏向保守。
                f32 drive_gate_scale_ramp_up_s = 0.10f;                                  // [RW] 门控放开时间常数（s）。越小表示释放越快，越大表示更平滑。
                f32 drive_gate_scale_ramp_down_s = 0.50f;                                // [RW] 门控收紧时间常数（s）。越大表示收紧更慢，更不容易突然“掐断”驱动。

                // ---- 停车转向保护 ------------------------------------------------
                // 在低速或静止时抑制不必要的舵角摆动，避免轮子在接近停住时反复“找角”。
                // 它主要解决停车抖动、低速微调来回打舵、以及静止姿态不稳定的问题。
                bool enable_stop_steer_guard = true; // [RW] 是否启用停车转向保护。关闭后低速区的舵角跟踪会更直接，但也更容易抖动。
                StopSteerGuardStrategy stop_steer_guard_strategy = StopSteerGuardStrategy::kHardHold; // [RW] 停车保护形状：硬保持更稳，曲线混合更柔和。
                f32 stop_guard_release_speed_m_s = 0.01f;      // [RW] 释放阈值（m/s）。低于该速度时，可视为进入停车保护区。
                f32 stop_guard_blend_start_speed_m_s = 0.20f;  // [RW] 混合起点速度（m/s）。从正常舵角控制逐步过渡到停车保护的起始点。
                f32 stop_guard_curve_half_speed_m_s = 0.08f;   // [RW] 曲线混合半效速度（m/s）。决定混合函数中“过半”的速度位置。
                f32 stop_guard_curve_exponent = 2.0f;          // [RW] 曲线混合指数。越大过渡越陡，越小过渡越平缓。
            };
            StrategyConfig default_strategy_cfg_; // [RW, 慎改] 默认策略基线。用于初始化和“恢复默认值”，不要把它当作实时状态。
            StrategyConfig runtime_strategy_cfg_; // [RW] 当前生效的运行时策略。可被外部接口动态切换，控制链路实际读取它。

            // =====================================================================
            // 航向控制参数（运行时可调）[RW]
            // 说明：这组参数只影响航向锁定/锁角逻辑，不影响平移速度规划。
            // =====================================================================
            PID_Position rot_z_pid_;              // [RW, 慎改] 航向位置环 PID。用于 LockToYaw / 相关锁角模式的角度误差闭环。
            u8 rot_z_pid_period_ = 1;             // [RW] PID 更新周期分频。1 表示每个控制周期都更新，数值越大频率越低、负载越小但响应更慢。
            f32 max_lock_to_rot_z_rad_s_ = 4.0f;  // [RW] LockToYaw 模式下的角速度上限（rad/s）。用于限制“往目标角赶”的最快速度。
            u32 lock_now_rot_z_shift_time_ms_ = 1000; // [RW] LockNow 松手缓冲时长（ms）。松开后短时间内继续维持目标，避免姿态突然跳变。

            // =====================================================================
            // 调试参数（通过全局 chassis 对象在调试器内直接改值）[RW]
            // 说明：这组参数只影响调试链路。正常控制不读取它们，只有切到相应 debug mode 时才会生效。
            // 速查：0~8 = 底盘输入接管/信号注入类模式；20 = 单轮调试；21 = 四轮朝前；22 = 回零观察；30 = 执行层直控。
            // =====================================================================
            struct DebugControl
            {
                // ---- 调试总开关与模式入口 ---------------------------------------
                bool enable = true; // [RW] 调试总开关。false 时整个调试接管链路不生效，系统走正常控制。
                u8 mode_raw = 0;    // [RW] 调试模式号。决定当前是输入接管、单轮调试、回零观察还是执行层直控。
                u8 mode_resolved_raw = static_cast<u8>(DebugMode::kWorldSpeed); // [RO] 解析后的实际模式号。用于观察 mode_raw 经过归一化后的结果。
                u8 wheel_index = 0; // [RW] 选中轮号（0~3）。用于单轮调试、单轮追踪日志和执行层直控的目标轮选择。

                // ---- 航向 / 激励注入 ---------------------------------------------
                f32 lock_rot_z = 0.0f; // [RW] LockTo 调试目标角（rad）。只在航向锁定相关调试中有意义。
                bool inject_step = false; // [RW] 是否注入阶跃信号。用于调试响应、阶跃跟踪或观察动态性能。
                bool inject_sine = false; // [RW] 是否注入正弦信号。用于频响、跟踪误差和相位滞后观察。
                f32 sine_amplitude = 0.0f; // [RW] 正弦幅值。与注入开关配合使用，表示激励强度。
                f32 sine_frequency = 0.1f; // [RW] 正弦频率（Hz）。越高越能看出响应速度，越低越适合慢速观察。
                f32 sine_offset = 0.0f; // [RW] 正弦偏置。把激励整体平移到某个基线附近使用。

                // ---- mode20：单轮调试 -------------------------------------------
                bool single_wheel_drive_enable = true; // [RW] mode20 下是否允许驱动输出。关闭时只看舵向，不下发驱动。
                bool single_wheel_soft_steer_enable = false; // [RW] mode20 下是否启用舵向软限幅轨迹。开启后不会一步跳到目标，而是按限速/限加速度渐进到位。
                bool single_wheel_use_custom_steer_limit = false; // [RW] mode20 是否使用自定义舵向限速参数。关闭时用默认舵向约束。
                f32 single_wheel_steer_rate_limit_deg_s = 120.0f; // [RW] mode20 舵向角速度上限（deg/s）。只在软舵向或限速轨迹下发挥作用。
                f32 single_wheel_steer_accel_limit_deg_s2 = 600.0f; // [RW] mode20 舵向角加速度上限（deg/s^2）。限制舵向变化“猛不猛”。
                bool single_wheel_drive_release_gate_enable = false; // [RW] mode20 驱动释放门控。开启后，舵角没对准前会先压住驱动。
                f32 single_wheel_drive_release_error_deg = 5.0f; // [RW] mode20 驱动释放角差阈值（deg）。舵角误差小于该值时才允许更积极地释放驱动。
                f32 single_wheel_target_steer_deg = 0.0f; // [RW] mode20 单轮舵向目标 OA（deg）。单轮调试时直接给舵角目标。
                f32 single_wheel_target_drive_rpm = 0.0f; // [RW] mode20 单轮驱动目标（rpm）。单轮调试时直接给驱动速度目标。

                // ---- mode30：执行层直控 -----------------------------------------
                bool direct_estop = true; // [RW] mode30 急停闸门。true 时禁止所有执行层输出，适合调试前的“安全锁”。
                bool direct_enable_steer[4] = {false, false, false, false}; // [RW] mode30 每轮舵向执行使能。单独放开某一轮的舵向下发。
                bool direct_enable_drive[4] = {false, false, false, false}; // [RW] mode30 每轮驱动执行使能。单独放开某一轮的驱动下发。
                u8 direct_input_source = 0; // [RW] mode30 输入来源：0=调试器缓存值，1=左摇杆，2=右摇杆阶跃。
                u8 direct_steer_control_type = 1; // [RW] mode30 舵向控制方式：0=电流，1=速度，2=单圈角，3=多圈角。
                f32 direct_steer_current_mA[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [RW] mode30 舵向电流目标缓存。作为直控输入的“待下发值”。
                f32 direct_steer_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [RW] mode30 舵向速度目标缓存。作为直控输入的“待下发值”。
                f32 direct_steer_single_turn_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [RW] mode30 舵向单圈角目标缓存。作为直控输入的“待下发值”。
                f32 direct_steer_multi_turn_deg[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [RW] mode30 舵向多圈角目标缓存。作为直控输入的“待下发值”。
                f32 direct_drive_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [RW] mode30 驱动速度目标缓存。作为直控输入的“待下发值”。
                f32 direct_drive_rpm_limit = 500.0f; // [RW] mode30 驱动目标限幅（rpm）。下发前的安全上限。
                f32 direct_steer_rpm_limit = 300.0f; // [RW] mode30 舵向速度限幅（rpm）。避免舵向指令过快。
                f32 direct_steer_current_limit_mA = 12000.0f; // [RW] mode30 舵向电流限幅（mA）。避免当前环指令过大。
                f32 direct_steer_single_turn_limit_deg = 180.0f; // [RW] mode30 单圈角限幅（deg）。适合做单圈范围内的安全测试。
                f32 direct_steer_multi_turn_limit_deg = 1080.0f; // [RW] mode30 多圈角限幅（deg）。限制多圈试验时的总角度范围。
                f32 direct_step_threshold = 0.3f; // [RW] 右摇杆阶跃触发阈值。超过该幅值才认为用户想发出阶跃动作。
                f32 direct_step_steer_current_mA = 2000.0f; // [RW] 阶跃幅值（电流，mA）。右摇杆触发后用于给舵向电流一个固定跃迁量。
                f32 direct_step_steer_rpm = 100.0f; // [RW] 阶跃幅值（速度，rpm）。右摇杆触发后用于给舵向速度一个固定跃迁量。
                f32 direct_step_steer_single_turn_deg = 90.0f; // [RW] 阶跃幅值（单圈角，deg）。右摇杆触发后用于给单圈角一个固定跃迁量。
                f32 direct_step_steer_multi_turn_deg = 180.0f; // [RW] 阶跃幅值（多圈角，deg）。右摇杆触发后用于给多圈角一个固定跃迁量。
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
                bool output_enable = true; // [RW] 串口输出总开关。false 时所有调试串口输出都停止，但控制逻辑仍继续运行。
                u8 output_mode_raw = static_cast<u8>(DebugOutputMode::kText); // [RW] 输出模式选择器：0=关，1=文本日志，2=四轮总览 justfloat，3=单轮高速 justfloat。
                u32 text_period_ms = 500; // [RW] 文本日志周期（ms）。只在 mode1 下使用，控制文本总刷新频率。
                u8 text_log_level = 1; // [RW] 文本日志等级。0 只发基础汇总，>=1 会轮流输出更细的 FS/FSW/FSH 分相信息。
                TickType_t text_last_ms = 0; // [RO] 文本日志节流时间戳。记录上一次发文本的时间，防止串口刷屏。
                u8 text_log_phase = 0; // [RO] 文本分相输出索引。0=FS 总览，1=FSW 单轮细节，2=FSH 回零/对位细节。

                // ---- mode2：四轮总览 justfloat ---------------------------------
                u32 overview_justfloat_period_ms = 5; // [RW] mode2 四轮总览 justfloat 周期（ms）。控制每次发送完整四轮电机数据的频率。
                TickType_t overview_justfloat_last_ms = 0; // [RO] mode2 发送节流时间戳。防止总览数据过于频繁。

                // ---- mode3：单轮高速 justfloat ---------------------------------
                u8 single_wheel_1khz_index = 0; // [RW] mode3 高速输出轮号。指定哪一轮作为 1kHz 高速追踪对象。
                u32 single_wheel_1khz_period_ms = 1; // [RW] mode3 目标周期（ms）。一般设为 1ms，表示尽可能按控制周期输出。
                TickType_t single_wheel_1khz_last_ms = 0; // [RO] mode3 发送节流时间戳。记录高速输出最近一次发送时刻。

                // ---- mode1：文本追踪节流 ----------------------------------------
                TickType_t single_wheel_trace_last_ms = 0; // [RO] 单轮文本跟踪节流时间戳。用于 mode1 下的单轮细节日志限频。
                TickType_t direct_trace_last_ms = 0; // [RO] 执行层文本跟踪节流时间戳。用于 mode1 下的直控调试日志限频。
            } debug_output_;

            // =====================================================================
            // DebugPidTune [RW]
            // 说明：这里存的是“待同步的 PID 配置缓存”，不是运行态实时对象。
            //       写完后通常还要等调试使能边沿或同步流程消费，运行中的 PID 才会真正换参数。
            // =====================================================================
            struct DebugPidTune
            {
                PID_Param_Config steer_speed_pid_cfg[4] = { // [RW] 四轮舵向速度环参数缓存。这里只是待同步配置，不会立刻改动正在运行的 PID。
                    {.kp = 32.0f, .ki = 0.085f, .kd = 0.0f, .I_Outlimit = 8000.0f, .isIOutlimit = true, .output_limit = 12000.0f, .deadband = 0.5f},
                    {.kp = 32.0f, .ki = 0.085f, .kd = 0.0f, .I_Outlimit = 8000.0f, .isIOutlimit = true, .output_limit = 12000.0f, .deadband = 0.5f},
                    {.kp = 32.0f, .ki = 0.085f, .kd = 0.0f, .I_Outlimit = 8000.0f, .isIOutlimit = true, .output_limit = 12000.0f, .deadband = 0.5f},
                    {.kp = 32.0f, .ki = 0.085f, .kd = 0.0f, .I_Outlimit = 8000.0f, .isIOutlimit = true, .output_limit = 12000.0f, .deadband = 0.5f},
                };
                PID_Param_Config steer_angle_pid_cfg[4] = { // [RW] 四轮舵向角度环参数缓存。修改后同样要经过同步流程才会进入运行态。
                    {.kp = 3.5f, .ki = 0.0f, .kd = 0.05f, .I_Outlimit = 0.0f, .isIOutlimit = true, .output_limit = 500.0f, .deadband = 0.03f},
                    {.kp = 3.5f, .ki = 0.0f, .kd = 0.05f, .I_Outlimit = 0.0f, .isIOutlimit = true, .output_limit = 500.0f, .deadband = 0.03f},
                    {.kp = 3.5f, .ki = 0.0f, .kd = 0.05f, .I_Outlimit = 0.0f, .isIOutlimit = true, .output_limit = 500.0f, .deadband = 0.03f},
                    {.kp = 3.5f, .ki = 0.0f, .kd = 0.05f, .I_Outlimit = 0.0f, .isIOutlimit = true, .output_limit = 500.0f, .deadband = 0.03f},
                };
                f32 steer_speed_pid_td_ratio[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [RW] 速度环 TD 比例参数。属于扩展调参项，通常和速度环整定一起看。
                f32 steer_angle_pid_i_separa[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // [RW] 角度环积分分离参数。用于决定误差多大时才允许积分参与。
                u32 steer_speed_pid_apply_stamp[4] = {0U, 0U, 0U, 0U}; // [RW] 速度环参数申请生效戳。外部写入后，通过同步流程消费。
                u32 steer_angle_pid_apply_stamp[4] = {0U, 0U, 0U, 0U}; // [RW] 角度环参数申请生效戳。外部写入后，通过同步流程消费。
                u32 steer_speed_pid_applied_stamp[4] = {0U, 0U, 0U, 0U}; // [RO] 速度环已生效戳。表示运行态已经真正接收到这组参数。
                u32 steer_angle_pid_applied_stamp[4] = {0U, 0U, 0U, 0U}; // [RO] 角度环已生效戳。表示运行态已经真正接收到这组参数。
                bool synced_on_enable_edge = false; // [RO] 本次调试使能上升沿是否已完成同步。避免重复把缓存参数刷入运行态。
            } debug_pid_tune_;

            // 回零与模块运行态（主要观察）[RO]
            bool homing_start_request_ = false;   // [RW] 回零启动请求锁存位（由外部触发，在线程内消费）
            f32 homing_align_to_zero_tolerance_deg_ = 2.0f; // [RW] 回零归位判稳阈值（deg）
            WheelConfig wheel_config_[4]; // [RO] 四个模块运行态快照
            f32 last_steer_rate_cmd_rad_s_[4] = {0.0f};  // [RO] 上周期转向速度命令
            f32 last_drive_omega_cmd_rad_s_[4] = {0.0f}; // [RO] 上周期驱动角速度命令
            bool selected_flipped_solution_[4] = {false}; // [RO] 每个模块是否选中翻转解
            f32 drive_gate_scale_[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // [RO] 每轮驱动门控缩放
            f32 adaptive_gate_scale_ = 1.0f; // [RO] 全局自适应门控缩放
            AdaptiveGatePhase adaptive_gate_phase_ = AdaptiveGatePhase::kIdle; // [RO] 自适应门控阶段
            u8 rot_z_pid_count_ = 0; // [RO] 航向 PID 分频计数器
            f32 lock_now_rot_z_target_ = 0.0f; // [RO] LockNow 真正维持的航向目标
            u32 lock_now_rot_z_shift_count_ = 0; // [RO] LockNow 松手缓冲倒计时

            // 控制链路缓存（观察）[RO]
            InputTargetData input_target_data_; // [RO] 输入目标快照（模式与期望速度/角度）
            Data target_data_;                  // [RO] 模式映射后的目标数据
            Data planned_data_;                 // [RO] 经限幅/策略处理后的规划数据
            Data last_planned_data_;            // [RO] 上一周期规划数据（用于加速度约束）
            Data current_data_;                 // [RO] 当前状态估计数据
            ModeFlag current_mode_flag_;        // [RO] 当前控制模式标志位

            // 传感器与输入缓存（观察）[RO]
            f32 input_hwt_rot_z_ = 0.0f; // [RO] IMU yaw
            f32 input_hwt_omega_z_ = 0.0f; // [RO] IMU yaw speed
            RmPocketData_t airjoy_data_{}; // [RO] 遥控器输入快照

            // 调试镜像（只读观察）[RO]
            struct DebugMirror
            {
                bool all_homed = false; // [RO] 四轮是否全部回零完成
                f32 current_oa_deg[4] = {0.0f}; // [RO] 各轮当前 OA 角（deg）
                f32 target_oa_deg[4] = {0.0f}; // [RO] 各轮目标 OA 角（deg）
                f32 current_drive_rpm[4] = {0.0f}; // [RO] 各轮当前驱动速度（rpm）
                f32 target_drive_rpm[4] = {0.0f}; // [RO] 各轮目标驱动速度（rpm）
                u8 homing_state[4] = {0, 0, 0, 0}; // [RO] 各轮回零状态机状态
                bool homing_sensor_active[4] = {false, false, false, false}; // [RO] 各轮光电门有效状态
                bool homing_last_edge_is_falling[4] = {false, false, false, false}; // [RO] 各轮最近边沿是否下降沿
                f32 homing_runtime_zero_offset_deg[4] = {0.0f}; // [RO] 各轮运行时零偏（deg）
                bool flipped_drive[4] = {false, false, false, false}; // [RO] 各轮是否采用翻转驱动解
                f32 drive_gate_scale_dbg[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // [RO] 各轮门控缩放系数
                f32 selected_wheel_steer_error_deg = 0.0f; // [RO] 选中轮舵向误差（deg）
                bool selected_wheel_drive_released = false; // [RO] 选中轮驱动是否已释放
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

                u64 last_exec_us = 0ULL;   // [RO] 最近一次循环执行耗时（不含 delay）
                u64 min_exec_us = 0ULL;    // [RO] 历史最小执行耗时
                u64 max_exec_us = 0ULL;    // [RO] 历史最大执行耗时
                u64 avg_exec_us = 0ULL;    // [RO] 最近窗口平均执行耗时（短窗）
                u64 loop_count = 0ULL;     // [RO] 已统计循环次数
                u64 overrun_count = 0ULL;  // [RO] 超预算次数（exec_us > budget_us）
                u64 last_start_us = 0ULL;  // [RO] 最近一次循环开始时间戳
                u64 last_end_us = 0ULL;    // [RO] 最近一次循环结束时间戳
                u32 budget_us = 1000U;     // [RO] 单周期预算（us，当前 period_ms_=1）
                u16 window_size = 500U;    // [RO] 短窗长度（循环次数）
                u16 window_count = 0U;     // [RO] 当前窗口有效样本数（<=window_size）
                u64 window_clamp_count = 0ULL; // [RO] 样本被 u16 饱和截断次数
            } task_perf_stat_;

            // 调试串口对象（一般不在调试器改动）[RO]
            Debug_Printf debug_uart_ = Debug_Printf(&huart8); // [RO]
        };

        using Result = jia::FourSteerChassis::Chassis::Result;

        inline Result Chassis::setZeroCurrent()
        {
            return setWheelTorqueFreeMode();
        }

        inline Result Chassis::setSpeed(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z)
        {
            return (coord == Coordinate::kBody) ? setTargetBodySpeedMode(vel_x, vel_y, omega_z)
                                                : setTargetWorldSpeedMode(vel_x, vel_y, omega_z);
        }

        inline Result Chassis::setSpeed_LockNowYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z)
        {
            return (coord == Coordinate::kBody) ? setTargetBodySpeedLockNowRotZWithNoOmegaZMode(vel_x, vel_y, omega_z)
                                                : setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(vel_x, vel_y, omega_z);
        }

        inline Result Chassis::setSpeed_LockToYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 rot_z)
        {
            return (coord == Coordinate::kBody) ? setTargetBodySpeedLockToRotZMode(vel_x, vel_y, rot_z)
                                                : setTargetWorldSpeedLockToRotZMode(vel_x, vel_y, rot_z);
        }

        inline Robot_Twist Chassis::getBodySpeed() const
        {
            Robot_Twist body_speed;
            body_speed.vx = getTargetBodyVelX();
            body_speed.vy = getTargetBodyVelY();
            body_speed.vz = getTargetOmegaZ();
            return body_speed;
        }

        inline Robot_Twist Chassis::getWorldSpeed() const
        {
            Robot_Twist world_speed;
            world_speed.vx = getTargetWorldVelX();
            world_speed.vy = getTargetWorldVelY();
            world_speed.vz = getTargetOmegaZ();
            return world_speed;
        }
    }
}

#define JIA_USE_THREE_OMNI_CHASSIS 0
#define JIA_USE_FOUR_STEER_CHASSIS 1

#if JIA_USE_THREE_OMNI_CHASSIS
using jia::ThreeOmniChassis::Chassis;
#endif

#if JIA_USE_FOUR_STEER_CHASSIS
using jia::FourSteerChassis::Chassis;
#endif

#endif // CHASSIS_H_
