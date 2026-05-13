#ifndef CHASSIS_H_
#define CHASSIS_H_

#include "APP_Utils.h"

#include "FreeRTOS.h"

#include "Motor_DJI.h"
#include "Module_CrsfReceiver.h"
#include "APP_debugTool.h"
#include "APP_PID.h"

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
                bool homing_sensor_active_high = true; // 传感器原始 GPIO 输入高电平时若视为“触发有效”就填 true，否则填 false
                void *homing_gpio_port = nullptr;   // 真实光电接入后填 STM32 HAL 的 GPIOA/GPIOB 等端口，或 CubeMX 生成的 *_GPIO_Port 宏
                u16 homing_gpio_pin = 0;            // 真实光电接入后填 GPIO_PIN_x，或 CubeMX 生成的 *_Pin 宏；未接时保持 0
                f32 homing_search_rpm = 10.0f;      // 回零搜索时给转向电机的转速指令，单位 rpm
                f32 homing_zero_offset_deg = 0.0f;  // 回零补偿角：传感器触发点到期望机械零位的偏差；它在建立零点时生效，不是安装角偏移
                f32 homing_timeout_s = 5.0f;        // 单轮回零超时时间，超时后进入故障态，单位秒
            };

            // InitConfig 是整车级初始化输入：一次性提供 4 个轮子的电机句柄、底盘限幅、
            // 锁 yaw 参数、空闲姿态策略以及每个轮子的初始化配置。
            struct InitConfig
            {
                Motor_Base *steer_motor_h[4] = {nullptr};
                Motor_Base *drive_motor_h[4] = {nullptr};
                f32 wheel_radius_m = 0.075f;
                f32 max_vel_x_m_s = 4.0f;
                f32 max_vel_y_m_s = 4.0f;
                f32 max_omega_z_rad_s = 8.0f;
                f32 max_acc_xy_acc_m_s2 = 4.0f;
                f32 max_acc_xy_dec_m_s2 = 8.0f;
                f32 max_alpha_z_acc_rad_s2 = 6.0f;
                f32 max_alpha_z_dec_rad_s2 = 10.0f;
                f32 max_drive_omega_rad_s = rpmToRadsF32(800.0f);
                f32 max_drive_alpha_rad_s2 = 120.0f;
                f32 max_steer_rate_rad_s = 8.0f;
                f32 max_steer_alpha_rad_s2 = 60.0f;
                f32 stationary_speed_epsilon_m_s = 0.01f;
                bool enable_cosine_compensation = true;
                f32 max_lock_to_rot_z_rad_s = 4.0f;
                u32 lock_now_rot_z_shift_time_ms = 1000;
                IdlePostureMode idle_posture_mode = IdlePostureMode::kHoldLast;
                WheelInitConfig wheels[4];
            };

            // 初始化
            // 这里只负责四舵轮 chassis 对象内部参数装配与状态准备，不意味着上层 FSM 已切到四舵轮链路。
            void init(InitConfig &config);
            Result startHoming();
            bool isHomingDone() const;
            // 运行时切换空闲/失能时的舵轮停靠姿态。
            void setIdlePostureMode(IdlePostureMode mode);

        private:
            // WheelConfig 是运行时轮组状态快照：既保存静态几何和硬件句柄，也保存回零状态、
            // 补偿结果与最近一次规划输出，供控制线程在每个周期更新。
            struct WheelConfig
            {
                f32 pos_x_m = 0.0f;
                f32 pos_y_m = 0.0f;
                f32 theta_oa_to_owi_rad = 0.0f;
                f32 steer_motor_sign = 1.0f;
                f32 drive_motor_sign = 1.0f;
                Motor_Base *steer_motor_h = nullptr;
                Motor_Base *drive_motor_h = nullptr;
                bool homing_enabled = false;
                bool homing_sensor_active_high = true;
                void *homing_gpio_port = nullptr;
                u16 homing_gpio_pin = 0;
                f32 homing_search_rpm = 10.0f;
                f32 homing_zero_offset_rad = 0.0f;
                f32 homing_timeout_s = 5.0f;
                HomingState homing_state = HomingState::kIdle;
                bool homing_last_sensor_active = false;
                bool homing_zero_valid = false;
                f32 homing_elapsed_s = 0.0f;
                f32 homing_runtime_zero_offset_rad = 0.0f;
                f32 corrected_steer_motor_total_angle_rad = 0.0f;
                f32 corrected_drive_omega_rad_s = 0.0f;
                f32 target_steer_motor_total_angle_rad = 0.0f;
                f32 target_drive_omega_rad_s = 0.0f;
                f32 steer_target_velocity_rad_s = 0.0f;
                bool flipped_drive_direction = false;
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
            void computeModuleCommands(const Data &command_data);
            void applyModuleCommands(bool all_homed);
            void updateCurrentData(bool all_homed);
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

            // 系统参数
            constexpr static u8 period_ms_ = 1;                  // 控制周期，单位：毫秒
            TickType_t time_ms_ = 0;                             // 当前时间，单位：毫秒
            constexpr static f32 period_ = period_ms_ / 1000.0f; // 控制周期，单位：秒

            // 底盘参数
            f32 wheel_radius_m_ = 0.075f;
            f32 max_vel_x_ = 4.0f;
            f32 max_vel_y_ = 4.0f;
            f32 max_omega_z_ = 8.0f;
            f32 max_acc_xy_acc_ = 4.0f;
            f32 max_acc_xy_dec_ = 8.0f;
            f32 max_alpha_z_acc_ = 6.0f;
            f32 max_alpha_z_dec_ = 10.0f;
            f32 max_drive_omega_rad_s_ = rpmToRadsF32(800.0f);
            f32 max_drive_alpha_rad_s2_ = 120.0f;
            f32 max_steer_rate_rad_s_ = 8.0f;
            f32 max_steer_alpha_rad_s2_ = 60.0f;
            f32 stationary_speed_epsilon_m_s_ = 0.01f;
            bool enable_cosine_compensation_ = true;
            IdlePostureMode idle_posture_mode_ = IdlePostureMode::kHoldLast;
            bool homing_start_request_ = false;
            WheelConfig wheel_config_[4];
            f32 last_steer_rate_cmd_rad_s_[4] = {0.0f};
            f32 last_drive_omega_cmd_rad_s_[4] = {0.0f};
            PID_Position rot_z_pid_;
            u8 rot_z_pid_period_ = 1;
            u8 rot_z_pid_count_ = 0;
            f32 max_lock_to_rot_z_rad_s_ = 4.0f;
            u32 lock_now_rot_z_shift_count_ = 0;
            u32 lock_now_rot_z_shift_time_ms_ = 1000;
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
