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

            // InitConfig 是整车级初始化输入：一次性提供 4 个轮子的电机句柄、底盘限幅、
            // 锁 yaw 参数、空闲姿态策略以及每个轮子的初始化配置。
            struct InitConfig
            {
                Motor_Base *steer_motor_h[4] = {nullptr}; // 4 个转向电机句柄，顺序需与 wheels[4] 的轮位定义保持一致
                Motor_Base *drive_motor_h[4] = {nullptr}; // 4 个驱动电机句柄，顺序需与对应转向模块一一匹配
                f32 wheel_radius_m = 0.075f;             // 轮半径，单位米；用于把驱动轮角速度与底盘线速度互相换算
                f32 max_vel_x_m_s = 4.0f;                // 底盘在自身 x 方向允许的最大规划速度，单位 m/s
                f32 max_vel_y_m_s = 4.0f;                // 底盘在自身 y 方向允许的最大规划速度，单位 m/s
                f32 max_omega_z_rad_s = 8.0f;            // 底盘绕 z 轴允许的最大规划角速度，单位 rad/s
                f32 max_acc_xy_acc_m_s2 = 4.0f;          // 平移速度上升时的最大加速度限幅，单位 m/s^2；用于“加速”阶段
                f32 max_acc_xy_dec_m_s2 = 8.0f;          // 平移速度下降时的最大减速度限幅，单位 m/s^2；用于“减速/刹车”阶段
                f32 max_alpha_z_acc_rad_s2 = 6.0f;       // z 轴角速度上升时的最大角加速度限幅，单位 rad/s^2
                f32 max_alpha_z_dec_rad_s2 = 10.0f;      // z 轴角速度下降时的最大角减速度限幅，单位 rad/s^2
                f32 max_drive_omega_rad_s = rpmToRadsF32(800.0f); // 单轮驱动角速度指令上限，单位 rad/s；模块解算后会再被夹紧到这里
                f32 max_drive_alpha_rad_s2 = 120.0f;     // 单轮驱动角加速度限幅，单位 rad/s^2；防止驱动指令突变过猛
                f32 max_steer_rate_rad_s = 8.0f;         // 单轮转向角速度上限，单位 rad/s；用于限制舵向变化速度
                f32 max_steer_alpha_rad_s2 = 60.0f;      // 单轮转向角加速度上限，单位 rad/s^2；用于限制舵向变化陡峭度
                f32 stationary_speed_epsilon_m_s = 0.01f; // 静止判定阈值，单位 m/s；模块目标速度低于它时会走“近静止/停车姿态”逻辑
                bool enable_cosine_compensation = true;   // 是否启用余弦补偿：转向角误差较大时衰减驱动输出，减少舵向未对准时的横向硬推
                f32 max_lock_to_rot_z_rad_s = 4.0f;       // 锁到目标 yaw 时允许的最大角速度，单位 rad/s；限制 rot_z 追踪收敛速度
                u32 lock_now_rot_z_shift_time_ms = 1000;  // 从普通速度模式切到“无 omega_z 时锁当前 yaw”模式后的过渡保持时间，单位 ms
                IdlePostureMode idle_posture_mode = IdlePostureMode::kHoldLast; // 近静止时的模块姿态策略：保持最后朝向，或切到 X 停车姿态
                SteeringStrategyMode steering_strategy_mode = SteeringStrategyMode::kShortestPath; // 转向解策略：默认允许翻转并优先最短转角
                f32 flip_enter_angle_deg = 100.0f;        // 进入/保持 flipped 解的角误差阈值（大于该值更倾向 flipped）
                f32 flip_exit_angle_deg = 80.0f;          // 从非 flipped 切入 flipped 的阈值，配合 enter 阈值构成滞回
                bool enable_drive_gate = false;           // 是否启用独立驱动抑制（DriveGate）
                DriveGateStrategy drive_gate_strategy = DriveGateStrategy::kHardGate; // DriveGate 策略类型
                DriveGateScope drive_gate_scope = DriveGateScope::kGlobal; // DriveGate 作用域：全局/按轮
                f32 drive_gate_close_angle_deg = 30.0f;   // Gate 关闭角阈值（Hard/Soft 模式使用）
                f32 drive_gate_min_scale = 0.0f;          // Gate 最小放行比例
                f32 drive_gate_curve_exponent = 2.0f;     // 连续曲线策略指数
                f32 drive_gate_curve_half_angle_deg = 20.0f; // 连续曲线半幅角阈值
                f32 drive_gate_curve_min_scale = 0.05f;   // 连续曲线最小比例
                f32 drive_gate_transition_linear_speed_m_s = 0.30f; // AdaptiveGate 线速度过渡阈值
                f32 drive_gate_transition_angular_speed_rad_s = 1.00f; // AdaptiveGate 角速度过渡阈值
                f32 drive_gate_scale_ramp_up_s = 0.10f;   // AdaptiveGate 放开斜坡时间
                f32 drive_gate_scale_ramp_down_s = 0.06f; // AdaptiveGate 收紧斜坡时间
                bool enable_stop_steer_guard = true;      // 是否启用停车转向保护
                StopSteerGuardStrategy stop_steer_guard_strategy = StopSteerGuardStrategy::kHardHold; // 停车转向保护策略
                f32 stop_guard_release_speed_m_s = 0.01f; // 残速低于该阈值后解除停车转向保护
                f32 stop_guard_blend_start_speed_m_s = 0.20f; // SoftBlend 策略开始混合阈值
                f32 stop_guard_curve_half_speed_m_s = 0.08f;  // ContinuousBlend 半幅速度阈值
                f32 stop_guard_curve_exponent = 2.0f;     // ContinuousBlend 曲线指数
                WheelInitConfig wheels[4];                // 4 个舵轮模块各自的安装/回零配置，顺序需与电机句柄数组一致
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
            void applyDebugSteerPidRuntimeTuning();
            void emitUart8VofaPid1kHzTrace();
            bool solveLinear3x3(f32 matrix[3][4], f32 &x0, f32 &x1, f32 &x2) const;
            bool estimateBodySpeedFromModules(f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z) const;

            InputTargetData input_target_data_;
            Data target_data_;
            Data planned_data_;
            Data last_planned_data_;
            Data current_data_;

            // 模式标志位
            ModeFlag current_mode_flag_;

            // IMU数据
            f32 input_hwt_rot_z_ = 0.0f;
            f32 input_hwt_omega_z_ = 0.0f;
            // 遥控器数据缓存：由 runThread() 每周期统一采样刷新，调试/非调试共用同一份输入快照
            RmPocketData_t airjoy_data_{};

            // 系统参数
            constexpr static u8 period_ms_ = 1;                  // 控制周期，单位：毫秒
            TickType_t time_ms_ = 0;                             // 当前时间，单位：毫秒
            constexpr static f32 period_ = period_ms_ / 1000.0f; // 控制周期，单位：秒

            // 底盘参数（从 InitConfig 下发到运行态，用于统一限幅与策略判定）
            f32 wheel_radius_m_ = 0.075f;                     // 轮半径，线速度与驱动角速度换算基准
            f32 max_vel_x_ = 4.0f;                            // 车体 X 方向最大线速度（m/s）
            f32 max_vel_y_ = 4.0f;                            // 车体 Y 方向最大线速度（m/s）
            f32 max_omega_z_ = 8.0f;                          // 车体 Z 轴最大角速度（rad/s）
            f32 max_acc_xy_acc_ = 4.0f;                       // 平面线速度“加速段”最大加速度（m/s^2）
            f32 max_acc_xy_dec_ = 8.0f;                       // 平面线速度“减速段”最大减速度（m/s^2）
            f32 max_alpha_z_acc_ = 6.0f;                      // 车体角速度“加速段”最大角加速度（rad/s^2）
            f32 max_alpha_z_dec_ = 10.0f;                     // 车体角速度“减速段”最大角减速度（rad/s^2）
            f32 max_drive_omega_rad_s_ = rpmToRadsF32(800.0f); // 单轮驱动电机目标角速度上限（rad/s）
            f32 max_drive_alpha_rad_s2_ = 120.0f;             // 单轮驱动角速度变化率上限（rad/s^2）
            f32 max_steer_rate_rad_s_ = 8.0f;                 // 单轮转向目标角速度上限（rad/s）
            f32 max_steer_alpha_rad_s2_ = 60.0f;              // 单轮转向目标角加速度上限（rad/s^2）
            f32 stationary_speed_epsilon_m_s_ = 0.01f;        // 近似静止阈值；低于该值可进入保持/驻车姿态逻辑
            bool enable_cosine_compensation_ = true;          // 是否启用舵角偏差余弦补偿（减小偏角期驱动贡献）
            IdlePostureMode idle_posture_mode_ = IdlePostureMode::kHoldLast; // 静止时姿态策略（保持当前或 X-Park）
            struct StrategyConfig
            {
                // 舵角解算策略：最短路径、带滞回翻转等选择入口
                SteeringStrategyMode steering_strategy_mode = SteeringStrategyMode::kShortestPath;
                f32 flip_enter_angle_deg = 100.0f; // 翻转进入阈值：角差大于该值时允许“舵角+180°并反转驱动”
                f32 flip_exit_angle_deg = 80.0f;   // 翻转退出阈值：形成滞回，避免在临界角附近反复抖动

                // 驱动抑制（Drive Gate）：舵角未对准时按策略压低驱动输出，减小横滑/冲击
                bool enable_drive_gate = false;
                DriveGateStrategy drive_gate_strategy = DriveGateStrategy::kHardGate; // 硬门控或曲线门控
                DriveGateScope drive_gate_scope = DriveGateScope::kGlobal;             // 全局门控或按轮门控
                f32 drive_gate_close_angle_deg = 30.0f;                                 // 超过该角差可触发强抑制
                f32 drive_gate_min_scale = 0.0f;                                        // 硬门控最小缩放（0=可完全关断）
                f32 drive_gate_curve_exponent = 2.0f;                                   // 曲线门控指数（越大越“硬”）
                f32 drive_gate_curve_half_angle_deg = 20.0f;                            // 曲线门控半效角
                f32 drive_gate_curve_min_scale = 0.05f;                                 // 曲线门控最小保底缩放
                f32 drive_gate_transition_linear_speed_m_s = 0.30f;                     // 平移速度过渡阈值（低速更易收紧门控）
                f32 drive_gate_transition_angular_speed_rad_s = 1.00f;                  // 自转速度过渡阈值
                f32 drive_gate_scale_ramp_up_s = 0.10f;                                  // 门控放开时间常数（s）
                f32 drive_gate_scale_ramp_down_s = 0.06f;                                // 门控收紧时间常数（s）

                // 停车转向保护：低速/静止时抑制不必要舵角摆动，避免来回找角
                bool enable_stop_steer_guard = true;
                StopSteerGuardStrategy stop_steer_guard_strategy = StopSteerGuardStrategy::kHardHold;
                f32 stop_guard_release_speed_m_s = 0.01f;      // 低于该速度可认为进入“停车保护区”
                f32 stop_guard_blend_start_speed_m_s = 0.20f;  // 从该速度开始由正常控制向停车保护混合
                f32 stop_guard_curve_half_speed_m_s = 0.08f;   // 曲线混合半效速度
                f32 stop_guard_curve_exponent = 2.0f;          // 混合曲线指数
            };
            StrategyConfig default_strategy_cfg_; // 初始化默认策略（可作为“恢复默认”基线）
            StrategyConfig runtime_strategy_cfg_; // 当前运行时策略（可动态切换）
            bool homing_start_request_ = false;   // 回零启动请求锁存位（由外部触发，在线程内消费）
            f32 homing_align_to_zero_tolerance_deg_ = 2.0f; // 回零归位判稳阈值（deg）：误差小于该值后进入 Ready
            WheelConfig wheel_config_[4];         // 四个模块的运行态快照
            f32 last_steer_rate_cmd_rad_s_[4] = {0.0f};  // 上周期转向速度命令（用于二阶限幅）
            f32 last_drive_omega_cmd_rad_s_[4] = {0.0f}; // 上周期驱动角速度命令（用于加速度限幅）
            bool selected_flipped_solution_[4] = {false}; // 每个模块当前是否选中“翻转驱动”解
            f32 drive_gate_scale_[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // 每轮驱动门控缩放系数
            f32 adaptive_gate_scale_ = 1.0f; // 全局自适应门控缩放（用于平滑过渡）
            AdaptiveGatePhase adaptive_gate_phase_ = AdaptiveGatePhase::kIdle; // 自适应门控状态机阶段

            // 航向控制相关（LockNow/LockTo 共享的姿态 PID）
            PID_Position rot_z_pid_;        // 航向位置环 PID（输入/输出按角度语义换算）
            u8 rot_z_pid_period_ = 1;       // PID 更新周期分频：每 N 个控制周期更新一次
            u8 rot_z_pid_count_ = 0;        // PID 分频计数器
            f32 max_lock_to_rot_z_rad_s_ = 4.0f; // LockToYaw 模式下航向环输出角速度上限
            f32 lock_now_rot_z_target_ = 0.0f; // LockNow 模式真正维持的航向目标；在手动旋转和松手缓冲阶段由当前 IMU 朝向刷新
            u32 lock_now_rot_z_shift_count_ = 0; // LockNow 松手缓冲倒计时（防止手动->锁角瞬间突变）
            u32 lock_now_rot_z_shift_time_ms_ = 1000; // LockNow 松手缓冲时长（ms）

            // 调试参数（通过全局 chassis 对象在调试器内直接改值）
            bool is_debug_ = 1;         // 调试总开关：true 时 isDebugMode() 每周期接管目标输入
            u8 debug_mode_ = 2;             // 调试模式号：0~8 对齐 ThreeOmni；20=单轮直控，21=四轮朝前零点检查，22=纯回零观察，30=四轮电机直控(绕过回零门控，支持舵向角度/舵向RPM两种直控)
            u8 debug_wheel_index_ = 0;      // 单轮调试目标索引（0~3）
            f32 debug_input_ = 90.0f;       // 通用调试输入保留位（兼容 ThreeOmni 习惯）
            f32 debug_lock_rot_z_ = 0.0f;   // LockTo 模式调试目标角（rad）
            bool is_step_signal_ = false;   // 是否启用阶跃注入（用于 omega_z 调试）
            bool is_sine_ = false;          // 是否启用正弦注入（用于 omega_z 调试）
            f32 sine_amplitude_ = 0.0f;     // 正弦注入幅值
            f32 sine_frequency_ = 0.1f;     // 正弦注入频率（Hz）
            f32 sine_offset_ = 0.0f;        // 正弦注入偏置
            bool is_wheel_speed_mode_ = true; // 单轮直控时是否下发驱动转速；false 时驱动置零
            bool debug_wheel_soft_steer_enable_ = false; // 单轮直控舵角是否启用软到位；false=硬切目标角，true=走速率/加速度限幅
            bool debug_wheel_use_custom_steer_limit_ = false; // 软到位时是否使用下面这组单独限幅；false 时复用整车 steer 限幅
            f32 debug_wheel_steer_rate_limit_deg_s_ = 120.0f; // 单轮软到位自定义转向角速度上限（deg/s）
            f32 debug_wheel_steer_accel_limit_deg_s2_ = 600.0f; // 单轮软到位自定义转向角加速度上限（deg/s^2）
            bool debug_wheel_drive_release_gate_enable_ = false; // 单轮直控驱动释放门：true 时必须先把舵角误差压到阈值内才允许驱动输出
            f32 debug_wheel_drive_release_error_deg_ = 5.0f; // 单轮直控驱动放行阈值（deg）；仅当目标 OA 误差小于等于该值时允许放驱动
            f32 debug_wheel_target_steer_deg_ = 0.0f; // 单轮直控舵向目标（OA角，deg；0=车头前方）
            f32 debug_wheel_target_drive_rpm_ = 0.0f; // 单轮直控驱动目标（rpm）
            bool debug_direct_estop_ = true; // 30模式总急停：true 时四轮舵向/驱动全部打零
            bool debug_direct_enable_steer_[4] = {false, false, false, false}; // 30模式每轮舵向使能
            bool debug_direct_enable_drive_[4] = {false, false, false, false}; // 30模式每轮驱动使能
            bool debug_direct_steer_use_rpm_mode_[4] = {false, false, false, false}; // 30模式每轮舵向控制方式：false=角度环，true=速度环直控
            f32 debug_direct_steer_oa_deg_[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 30模式每轮OA目标角（deg）
            f32 debug_direct_steer_rpm_[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 30模式每轮舵向速度环直控目标（rpm）
            f32 debug_direct_drive_rpm_[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 30模式每轮驱动目标转速（rpm）
            f32 debug_direct_drive_rpm_limit_ = 500.0f; // 30模式驱动转速限幅（rpm）
            f32 debug_direct_steer_rpm_limit_ = 300.0f; // 30模式舵向速度环直控限幅（rpm）
            // 30模式多输入扩展：输入源与控制量类型由调试器变量切换，默认只作用 debug_wheel_index_ 轮。
            // input_source: 0=调试器直接给值, 1=遥控器左摇杆映射, 2=右摇杆阈值阶跃
            u8 debug_direct_input_source_ = 0;
            // control_type: 0=舵向电流, 1=舵向速度, 2=舵向单圈角, 3=舵向多圈角
            u8 debug_direct_steer_control_type_ = 1;
            f32 debug_direct_steer_current_mA_[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 30模式每轮舵向电流直控目标（mA）
            f32 debug_direct_steer_single_turn_deg_[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 30模式每轮舵向单圈角目标（deg）
            f32 debug_direct_steer_multi_turn_deg_[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 30模式每轮舵向多圈角目标（deg）
            f32 debug_direct_steer_current_limit_mA_ = 12000.0f; // 30模式舵向电流直控限幅（mA）
            f32 debug_direct_steer_single_turn_limit_deg_ = 180.0f; // 30模式单圈角输入幅值限幅（deg）
            f32 debug_direct_steer_multi_turn_limit_deg_ = 1080.0f; // 30模式多圈角输入幅值限幅（deg）
            f32 debug_direct_step_threshold_ = 0.3f; // 30模式阶跃触发阈值（右摇杆绝对值）
            f32 debug_direct_step_steer_current_mA_ = 2000.0f; // 30模式阶跃幅值：舵向电流（mA）
            f32 debug_direct_step_steer_rpm_ = 100.0f; // 30模式阶跃幅值：舵向速度（rpm）
            f32 debug_direct_step_steer_single_turn_deg_ = 90.0f; // 30模式阶跃幅值：舵向单圈角（deg）
            f32 debug_direct_step_steer_multi_turn_deg_ = 180.0f; // 30模式阶跃幅值：舵向多圈角（deg）
            Debug_Printf debug_uart_ = Debug_Printf(&huart8); // FourSteer 调试串口（UART8）
            u8 debug_uart8_output_mode_ = 1; // UART8输出模式：0=全关，1=仅文本日志，2=仅四轮总览justfloat，3=仅单轮1kHz justfloat
            bool debug_uart8_output_enable_ = true; // UART8 输出总开关；具体输出类型由 output_mode_ 唯一裁决
            u32 debug_uart8_log_period_ms_ = 500; // UART8 常驻日志输出周期（ms），默认 500ms 即 2Hz
            u8 debug_uart8_log_level_ = 1; // UART8 日志级别：0=心跳摘要，1=附带单轮细节与 SW20 专项行
            TickType_t debug_uart8_log_last_ms_ = 0; // UART8 常驻日志节流时间戳
            u8 debug_uart8_log_phase_ = 0; // 文本日志分相发送：0=FS, 1=FSW, 2=FSH
            u32 debug_uart8_justfloat_period_ms_ = 5; // mode=2 四轮总览 justfloat 周期（ms），默认 5ms=200Hz
            TickType_t debug_uart8_justfloat_last_ms_ = 0; // UART8 justfloat 节流时间戳
            u8 debug_pid_1khz_wheel_index_ = 0; // 1kHz PID诊断轮索引（0~3）
            u32 debug_pid_1khz_period_ms_ = 1; // 1kHz PID诊断发送周期（ms）
            TickType_t debug_pid_1khz_last_ms_ = 0; // 1kHz PID诊断节流时间戳

            // 在线 PID 调参入口：通过调试器改参数 + 自增 stamp 触发下发，避免每周期重复写 PID。
            PID_Param_Config debug_steer_speed_pid_cfg_[4] = {
                {.kp = 32.0f, .ki = 0.085f, .kd = 0.0f, .I_Outlimit = 8000.0f, .isIOutlimit = true, .output_limit = 12000.0f, .deadband = 0.5f},
                {.kp = 32.0f, .ki = 0.085f, .kd = 0.0f, .I_Outlimit = 8000.0f, .isIOutlimit = true, .output_limit = 12000.0f, .deadband = 0.5f},
                {.kp = 32.0f, .ki = 0.085f, .kd = 0.0f, .I_Outlimit = 8000.0f, .isIOutlimit = true, .output_limit = 12000.0f, .deadband = 0.5f},
                {.kp = 32.0f, .ki = 0.085f, .kd = 0.0f, .I_Outlimit = 8000.0f, .isIOutlimit = true, .output_limit = 12000.0f, .deadband = 0.5f},
            };
            PID_Param_Config debug_steer_angle_pid_cfg_[4] = {
                {.kp = 3.5f, .ki = 0.0f, .kd = 0.05f, .I_Outlimit = 0.0f, .isIOutlimit = true, .output_limit = 500.0f, .deadband = 0.03f},
                {.kp = 3.5f, .ki = 0.0f, .kd = 0.05f, .I_Outlimit = 0.0f, .isIOutlimit = true, .output_limit = 500.0f, .deadband = 0.03f},
                {.kp = 3.5f, .ki = 0.0f, .kd = 0.05f, .I_Outlimit = 0.0f, .isIOutlimit = true, .output_limit = 500.0f, .deadband = 0.03f},
                {.kp = 3.5f, .ki = 0.0f, .kd = 0.05f, .I_Outlimit = 0.0f, .isIOutlimit = true, .output_limit = 500.0f, .deadband = 0.03f},
            };
            f32 debug_steer_speed_pid_td_ratio_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            f32 debug_steer_angle_pid_i_separa_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            u32 debug_steer_speed_pid_apply_stamp_[4] = {0U, 0U, 0U, 0U};
            u32 debug_steer_angle_pid_apply_stamp_[4] = {0U, 0U, 0U, 0U};
            u32 debug_steer_speed_pid_applied_stamp_[4] = {0U, 0U, 0U, 0U};
            u32 debug_steer_angle_pid_applied_stamp_[4] = {0U, 0U, 0U, 0U};

            // 调试镜像量：给调试器直接看，统一换成更直观的单位，避免联调时反复手算弧度。
            bool debug_all_homed_ = false;                   // 当前周期是否全轮已回零完成
            f32 debug_current_oa_deg_[4] = {0.0f};          // 当前 OA 朝向（deg）
            f32 debug_target_oa_deg_[4] = {0.0f};           // 当前目标 OA 朝向（deg）
            f32 debug_current_drive_rpm_[4] = {0.0f};       // 当前驱动反馈（rpm）
            f32 debug_target_drive_rpm_[4] = {0.0f};        // 当前驱动目标（rpm）
            u8 debug_homing_state_[4] = {0, 0, 0, 0};       // 当前回零状态枚举值
            bool debug_homing_sensor_raw_high_[4] = {false, false, false, false}; // 当前原始光电门电平
            bool debug_homing_sensor_active_[4] = {false, false, false, false}; // 当前按 active_high 极性换算后的逻辑触发态
            bool debug_homing_last_edge_is_falling_[4] = {false, false, false, false}; // 最近一次边沿方向
            f32 debug_homing_runtime_zero_offset_deg_[4] = {0.0f}; // 当前运行时零偏（deg）
            bool debug_flipped_drive_[4] = {false, false, false, false}; // 当前是否采用翻转驱动解
            f32 debug_drive_gate_scale_dbg_[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // 当前驱动抑制比例
            f32 debug_selected_wheel_steer_error_deg_ = 0.0f; // 单轮直控当前选中轮的 OA 目标误差（deg）
            bool debug_selected_wheel_drive_released_ = false; // 单轮直控当前选中轮是否已满足驱动放行条件
            TickType_t debug_wheel_uart_log_last_ms_ = 0; // 单轮调试日志节流时间戳（20Hz）
            TickType_t debug_direct_uart_log_last_ms_ = 0; // 30模式诊断日志节流时间戳
            f32 debug_steer_angle_pid_p_term_[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 四轮转向角度环P配置快照（用于1kHz诊断包）
            f32 debug_steer_angle_pid_i_term_[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 四轮转向角度环I配置快照（用于1kHz诊断包）
            f32 debug_steer_angle_pid_d_term_[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 四轮转向角度环D配置快照（用于1kHz诊断包）
            f32 debug_steer_angle_pid_output_rpm_[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 四轮转向角度环输出（期望rpm）
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
