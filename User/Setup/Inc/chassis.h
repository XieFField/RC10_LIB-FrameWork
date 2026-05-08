#ifndef CHASSIS_H_
#define CHASSIS_H_

#include <stdint.h>

#include "RC10_LIB/APP/Inc/APP_Utils.h"

#include "FreeRTOS.h"

#include "Motor_DJI.h"
#include "Module_CrsfReceiver.h"
#include "APP_debugTool.h"
#include "APP_PID.h"
#include "Module_ChassisSwerve.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct JiaChassisDebugWatch
{
    uint32_t active_chassis_type; // 0=none, 3=tri-omni, 4=four-swerve
    uint32_t loop_count;
    uint32_t last_tick_ms;
    uint32_t dwt_last_cycles;
    uint32_t dwt_last_us;
    uint32_t dwt_min_us;
    uint32_t dwt_max_us;
    uint64_t us64_last_us;
    uint64_t us64_min_us;
    uint64_t us64_max_us;

    uint32_t mode;
    uint32_t debug_mode;
    uint32_t is_debug;
    uint32_t is_world_speed_mode;
    uint32_t is_lock_now_rot_z;
    uint32_t is_lock_to_rot_z;

    float tri_target_vx_m_s;
    float tri_target_vy_m_s;
    float tri_target_wz_rad_s;
    float tri_planned_vx_m_s;
    float tri_planned_vy_m_s;
    float tri_planned_wz_rad_s;
    float tri_wheel_target_omega_rad_s[3];
    float tri_wheel_feedback_omega_rad_s[3];

    uint32_t four_debug_wheel_index;
    uint32_t four_photogate_signal;
    uint32_t four_is_calibrating;
    float four_calibration_angle_deg;
    float four_steer_target_deg;
    float four_steer_feedback_deg;
    float four_steer_target_rpm;
    float four_steer_feedback_rpm;
    float four_drive_command_rpm;
    float four_drive_target_rpm;
    float four_drive_feedback_rpm;
    float four_drive_target_current;
    float four_drive_feedback_current;
    uint32_t four_drive_brake_mode; // 0=rpm, 1=zero-rpm brake, 2=force brake
    uint32_t four_drive_force_brake_enabled;
    uint32_t four_drive_zero_rpm_brake_enabled;
    float four_drive_applied_brake_current;
    float four_drive_zero_rpm_threshold_rpm;
    // 四轮 Swerve 正常模式观测
    uint32_t four_swerve_used_controller_step;
    float four_swerve_body_vx_m_s;
    float four_swerve_body_vy_m_s;
    float four_swerve_body_wz_rad_s;
} JiaChassisDebugWatch;

extern volatile JiaChassisDebugWatch g_jia_chassis_debug_watch;

#ifdef __cplusplus
}
#endif

namespace jia
{
    namespace TriOmniChassis
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
            // 三全向轮逆运动学
            void inverseKinematics(f32 in_x, f32 in_y, f32 in_z, f32 &out_w1, f32 &out_w2, f32 &out_w3);

            // 锁定旋转逻辑（依赖成员变量，暂保留为私有方法）
            void isLockNowRotZ(bool is_lock, f32 rot_z, f32 omega_z, f32 &out_rot_z, f32 &out_omega_z);
            void isLockToRotZ(bool is_lock, f32 tar_rot_z, f32 pla_rot_z, f32 &out_rot_z, f32 omega_z, f32 &out_omega_z);

            // PID 周期计算
            void calculatePid(PID_Incremental &pid, u8 &count, u8 period, f32 target, f32 feedback, f32 &output);
            void calculatePid(PID_Position &pid, u8 &count, u8 period, f32 target, f32 feedback, f32 &output);

            // 轮子配置初始化
            void initWheelConfig(WheelConfig &wheel, f32 pos_x, f32 pos_y, f32 rot_z_deg, M3508 *motor_handle = nullptr);

            // 轮子控制封装（薄封装，直接代理 Motor_Base 接口）
            void setWheelTargetCurrent(WheelConfig &wheel, f32 current);
            void setWheelTargetOmega(WheelConfig &wheel, f32 omega);
            f32 getWheelCurrentOmega(const WheelConfig &wheel) const;
            f32 getWheelTargetCurrent(const WheelConfig &wheel) const;
            f32 getWheeCurrentCurrent(const WheelConfig &wheel) const;
            f32 getWheelCurrentRpm(const WheelConfig &wheel) const;

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
            struct InitConfig
            {
                M3508 *steer_motor_h[4] = {nullptr};
                Motor_Base *drive_motor_h[4] = {nullptr};
            };

            // 初始化
            void init(InitConfig &config);

        private:
            struct WheelConfig
            {
                f32 pos_x;                           // 单位：米
                f32 pos_y;                           // 单位：米
                f32 rot_z_deg;                       // 单位：度
                M3508 *steer_motor_h = nullptr;      // 舵向电机句柄
                Motor_Base *drive_motor_h = nullptr; // 轮向电机句柄

                f32 sin_rot_z;
                f32 cos_rot_z;
                f32 eq_radius; // 等效半径，equivalent radius，可以是负值，单位：米

                f32 abs_sin_rot_z; // 正弦值的绝对值
                f32 abs_cos_rot_z; // 余弦值的绝对值
                f32 abs_eq_radius; // 等效半径的绝对值，单位：米

                M3508 *&smh = steer_motor_h;
                Motor_Base *&dmh = drive_motor_h;
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
                kBodySpeedLockNowRotZMode,
                kBodySpeedLockToRotZMode,
                kWorldSpeedMode,
                kWorldSpeedLockNowRotZMode,
                kWorldSpeedLockToRotZMode,
                kWorldSpeedLockNowRotZWithNoOmegaZMode,
                kBodySpeedLockNowRotZWithNoOmegaZMode,
            };

            struct ModeFlag
            {
                bool is_wheel_torque_free = false;    // 是否为轮子扭矩自由模式
                Coordinate coord = Coordinate::kBody; // 速度坐标系
                bool is_lock_now_rot_z;               // 是否固定当前rot_z
                bool is_lock_to_rot_z;                // 是否固定到rot_z
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

                f32 rot_z; // z轴朝向角度，单位：rad

                f32 w1_steer_angle; // 舵向轮子1角度，单位：rad
                f32 w2_steer_angle; // 舵向轮子2角度，单位：rad
                f32 w3_steer_angle; // 舵向轮子3角度，单位：rad
                f32 w4_steer_angle; // 舵向轮子4角度，单位：rad

                f32 w1_drive_omega; // 航向轮子1角速度，单位：rad/s
                f32 w2_drive_omega; // 航向轮子2角速度，单位：rad/s
                f32 w3_drive_omega; // 航向轮子3角速度，单位：rad/s
                f32 w4_drive_omega; // 航向轮子4角速度，单位：rad/s
            };

            enum class DebugSignalMode : u8
            {
                kJoystick = 0,
                kStep = 1,
                kSine = 2,
                kHandInput = 3,
            };

            struct RuntimeSwerveDebugSnapshot
            {
                bool config_ready = false;
                bool output_valid = false;
                bool used_controller_step = false;
                jia::swerve::ChassisCommand command = {};
                jia::swerve::ModuleFeedback feedback[jia::swerve::kModuleCount] = {};
                jia::swerve::ModuleCommand output[jia::swerve::kModuleCount] = {};
            };

            // 创建线程
            static void createThread(void *arg);
            // 运行线程函数
            void runThread(void *arg);

            // 输入目标数据
            InputTargetData input_target_data_;
            InputTargetData &itd_ = input_target_data_;
            // 目标数据
            Data target_data_;
            Data &td_ = target_data_;
            // 规划数据
            Data planned_data_; // 规划数据
            Data &pd_ = planned_data_;
            Data last_planned_data_; // 上一次规划数据
            Data &lpd_ = last_planned_data_;
            // 当前数据
            Data current_data_;
            Data &cd_ = current_data_;

            // 模式标志位
            ModeFlag current_mode_flag_;
            ModeFlag &cmf_ = current_mode_flag_;

            // IMU数据
            f32 input_hwt_rot_z_;
            f32 &ihrz_ = input_hwt_rot_z_;
            f32 input_hwt_omega_z_;
            f32 &ihoz_ = input_hwt_omega_z_;

            // 系统参数
            constexpr static u8 period_ms_ = 1;                  // 控制周期，单位：毫秒
            TickType_t time_ms_;                                 // 当前时间，单位：毫秒
            constexpr static f32 period_ = period_ms_ / 1000.0f; // 控制周期，单位：秒

            // 底盘参数
            //  // 轮子半径
            constexpr static f32 steer_wheel_radius_ = 0.075f; // 舵向轮子半径（单位：米）
            const f32 &swr_ = steer_wheel_radius_;
            //  // 轮子配置
            WheelConfig wheel_config_[4];
            WheelConfig &w0_ = wheel_config_[0];
            WheelConfig &w1_ = wheel_config_[1];
            WheelConfig &w2_ = wheel_config_[2];
            WheelConfig &w3_ = wheel_config_[3];
            jia::swerve::SwerveConfig swerve_config_ = {};
            jia::swerve::SwerveController swerve_controller_{swerve_config_};
            RuntimeSwerveDebugSnapshot swerve_runtime_debug_ = {};

            // 速度限制参数
            //  // 车端速度限制参数
            f32 max_vel_x_ = 1000.0f;   // 最大x轴速度，单位：米/秒
            f32 max_vel_y_ = 1000.0f;   // 最大y轴速度，单位：米/秒
            f32 max_omega_z_ = 1000.0f; // 最大z轴角速度，单位：rad/s
            //  // 轮端速度限制参数
            bool is_wheel_omega_limit_ = true;            // 是否进行轮端角速度限制
            f32 max_wheel_omega_ = rpmToRadsF32(1000.0f); // 最大轮子角速度，单位：rad/s
            f32 max_wheel_vel_ = 0.0f;                    // 最大轮子线速度，单位：米/秒

            // 加速度限制参数
            //  // 车端加速度限制参数
            bool is_chassis_acc_limit_ = true; // 是否进行车端加速度限制
            f32 max_acc_xy_acc_ = 100.0f;      // 最大XY轴线加速度，单位：m/s^2
            f32 max_acc_xy_dec_ = 100.0f;      // 最大XY轴线减速度，单位：m/s^2
            f32 max_alpha_z_acc_ = 100.0f;     // 最大z轴角加速度，单位：rad/s^2
            f32 max_alpha_z_dec_ = 100.0f;     // 最大z轴角减速度，单位：rad/s^2

            // 调试参数
            bool is_debug_ = true; // 是否开启调试模式
            u8 debug_mode_ = 0;    // 调试模式

            u8 debug_wheel_index_ = 0; // 调试轮子索引

            bool is_step_signal_ = false; // 是否使用阶跃信号

            bool is_sine_ = false; // 是否使用正弦波
            f32 sine_amplitude_ = 0.0f;
            f32 sine_frequency_ = 0.1f;
            f32 sine_offset_ = 0.0f;

            bool is_hand_input_ = false; // 是否使用手动输入信号
            f32 hand_input_ = 0.0f;      // 手动输入信号

            DebugSignalMode steer_signal_mode_ = DebugSignalMode::kJoystick; // 0摇杆 1阶跃 2正弦 3手输
            DebugSignalMode drive_signal_mode_ = DebugSignalMode::kJoystick; // 0摇杆 1阶跃 2正弦 3手输
            f32 steer_hand_input_ = 0.0f; // 舵向手动输入
            f32 drive_hand_input_ = 0.0f; // 轮向手动输入
            f32 debug_step_threshold_ = 0.3f; // 阶跃触发阈值

            f32 debug_input_ = 180.0f;      // 舵向调试输入
            f32 drive_debug_input_ = 1000.0f; // 轮向调试输入，单位：rpm
            f32 debug_lock_rot_z_ = 0.0f; // 调试固定rot_z
            bool is_drive_force_brake_enabled_ = false; // 是否强制轮向刹车
            bool is_drive_zero_rpm_brake_enabled_ = true; // 目标rpm接近0时是否自动刹车
            f32 drive_force_brake_current_ = 70000.0f; // 强制刹车电流，单位：mA
            f32 drive_zero_rpm_brake_current_ = 70000.0f; // 零速自动刹车电流，单位：mA
            f32 drive_zero_rpm_threshold_rpm_ = 30.0f; // 自动刹车阈值，单位：rpm
            bool is_wheel_single_position_mode_ = false; // 是否为轮子单圈位置模式
            bool is_wheel_total_position_mode_ = false; // 是否为轮子多圈位置模式
            bool is_wheel_speed_mode_ = false;           // 是否为轮子速度模式
            bool is_wheel_current_mode_ = false;         // 是否为轮子电流模式

            Debug_Printf debug_uart_ = Debug_Printf(&huart8); // 调试串口
            u8 printf_period_ms_ = 1;                         // 串口调试打印周期，单位：毫秒
            u8 printf_period_count_ = 0;                      // 串口调试打印周期计数器

            RmPocketData_t airjoy_data_; // 从AirJoy接收的数据

        private:
            void isDebugMode();
            f32 buildDebugSetpoint(DebugSignalMode mode, f32 axis, f32 amplitude, f32 hand_input) const;
            void initWheelConfig(WheelConfig &wheel, f32 pos_x, f32 pos_y, f32 rot_z_deg, M3508 *steer_motor_h, Motor_Base *drive_motor_h);
            void configureDefaultSwerve();
            bool buildRuntimeSwerveMotion(jia::swerve::ChassisCommand &out_command);
            void captureRuntimeSwerveFeedback(jia::swerve::ModuleFeedback out_feedback[jia::swerve::kModuleCount]) const;
            void applyRuntimeSwerveCommands(const jia::swerve::ModuleCommand commands[jia::swerve::kModuleCount]);
            void runRuntimeSwerveControl();

            void clearInputTargetData();

            void setSteerWheelTargetRpm(WheelConfig &wheel, f32 rpm);
            void setSteerWheelTargetCurrent(WheelConfig &wheel, f32 current);
            void setSteerWheelTargetAngleDeg(WheelConfig &wheel, f32 angle_deg);
            void setSteerWheelTargetTotalAngleDeg(WheelConfig &wheel, f32 total_angle_deg);
            f32 getSteerWheelTargetAngleDeg(const WheelConfig &wheel) const;
            f32 getSteerWheelTargetCurrent(const WheelConfig &wheel) const;
            f32 getSteerWheelCurrentAngleDeg(const WheelConfig &wheel) const;
            f32 getSteerWheelCurrentAngleDegCalibrated(const WheelConfig &wheel) const;
            f32 getSteerWheelCurrentRPM(const WheelConfig &wheel) const;
            f32 getSteerWheelCurrentCurrent(const WheelConfig &wheel) const;
            f32 getSteerWheelCurrentTotalAngleDegCalibrated(const WheelConfig &wheel) const;
            void setDriveWheelBrake(WheelConfig &wheel, f32 brake_current);
            void setDriveWheelTargetRpm(WheelConfig &wheel, f32 rpm);
            void setDriveWheelTargetCurrent(WheelConfig &wheel, f32 current);
            void applyDriveWheelDebugCommand(WheelConfig &wheel, f32 drive_target_rpm);
            f32 getDriveWheelTargetRPM(const WheelConfig &wheel) const;
            f32 getDriveWheelCurrentRPM(const WheelConfig &wheel) const;
            f32 getDriveWheelCurrentCurrent(const WheelConfig &wheel) const;

        private:
            bool photogate_signal_ = false;
            bool last_photogate_signal_ = false;

            bool is_use_cailbration_angle_ = false; // 是否使用校准角度

            bool is_power_on_cailbration_ = false; // 是否开启校准
            bool is_doing_cailbration_ = false;    // 是否正在校准中
            f32 cailbration_rpm_ = 20.0f;           // 校准rpm，单位：rpm/s
            f32 cailbration_angle_deg_ = 0.0f;     // 校准参考姿态角度，单位：deg
        };

        using Result = jia::FourSteerChassis::Chassis::Result;

        inline Result Chassis::setZeroCurrent()
        {
            return setWheelTorqueFreeMode();
        }

        inline Result Chassis::setSpeed(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z)
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

        inline Result Chassis::setSpeed_LockNowYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 omega_z)
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

        inline Result Chassis::setSpeed_LockToYaw(Coordinate coord, f32 vel_x, f32 vel_y, f32 rot_z)
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

        inline void Chassis::clearInputTargetData()
        {
            itd_.mode = Mode::kWheelTorqueFreeMode;
            itd_.vel_x = 0.0f;
            itd_.vel_y = 0.0f;
            itd_.omega_z = 0.0f;
            itd_.rot_z = 0.0f;
        }

        inline Result Chassis::setWheelTorqueFreeMode()
        {
            clearInputTargetData();
            itd_.mode = Mode::kWheelTorqueFreeMode;
            return Result::kOk;
        }

        inline Result Chassis::setTargetBodySpeedMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            itd_.mode = Mode::kBodySpeedMode;
            itd_.vel_x = vel_x;
            itd_.vel_y = vel_y;
            itd_.omega_z = omega_z;
            return Result::kOk;
        }

        inline Result Chassis::setTargetBodySpeedLockNowRotZMode(f32 vel_x, f32 vel_y)
        {
            itd_.mode = Mode::kBodySpeedLockNowRotZMode;
            itd_.vel_x = vel_x;
            itd_.vel_y = vel_y;
            return Result::kOk;
        }

        inline Result Chassis::setTargetBodySpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            itd_.mode = Mode::kBodySpeedLockNowRotZWithNoOmegaZMode;
            itd_.vel_x = vel_x;
            itd_.vel_y = vel_y;
            itd_.omega_z = omega_z;
            return Result::kOk;
        }

        inline Result Chassis::setTargetBodySpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z)
        {
            itd_.mode = Mode::kBodySpeedLockToRotZMode;
            itd_.vel_x = vel_x;
            itd_.vel_y = vel_y;
            itd_.rot_z = rot_z;
            return Result::kOk;
        }

        inline Result Chassis::setTargetWorldSpeedMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            itd_.mode = Mode::kWorldSpeedMode;
            itd_.vel_x = vel_x;
            itd_.vel_y = vel_y;
            itd_.omega_z = omega_z;
            return Result::kOk;
        }

        inline Result Chassis::setTargetWorldSpeedLockNowRotZMode(f32 vel_x, f32 vel_y)
        {
            itd_.mode = Mode::kWorldSpeedLockNowRotZMode;
            itd_.vel_x = vel_x;
            itd_.vel_y = vel_y;
            return Result::kOk;
        }

        inline Result Chassis::setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            itd_.mode = Mode::kWorldSpeedLockNowRotZWithNoOmegaZMode;
            itd_.vel_x = vel_x;
            itd_.vel_y = vel_y;
            itd_.omega_z = omega_z;
            return Result::kOk;
        }

        inline Result Chassis::setTargetWorldSpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z)
        {
            itd_.mode = Mode::kWorldSpeedLockToRotZMode;
            itd_.vel_x = vel_x;
            itd_.vel_y = vel_y;
            itd_.rot_z = rot_z;
            return Result::kOk;
        }

        inline f32 Chassis::getTargetBodyVelX() const
        {
            return pd_.vel_x;
        }

        inline f32 Chassis::getTargetBodyVelY() const
        {
            return pd_.vel_y;
        }

        inline f32 Chassis::getTargetWorldVelX() const
        {
            return td_.vel_x;
        }

        inline f32 Chassis::getTargetWorldVelY() const
        {
            return td_.vel_y;
        }

        inline f32 Chassis::getTargetOmegaZ() const
        {
            return pd_.omega_z;
        }

        inline f32 Chassis::getCurrentBodyVelX() const
        {
            return cd_.vel_x;
        }

        inline f32 Chassis::getCurrentBodyVelY() const
        {
            return cd_.vel_y;
        }

        inline f32 Chassis::getCurrentWorldVelX() const
        {
            return td_.vel_x;
        }

        inline f32 Chassis::getCurrentWorldVelY() const
        {
            return td_.vel_y;
        }

        inline f32 Chassis::getCurrentOmegaZ() const
        {
            return cd_.omega_z;
        }
    }
}

#define JIA_USE_TRIO_CHASSIS 0
#define JIA_USE_FOUR_STEER_CHASSIS 1

#if JIA_USE_TRIO_CHASSIS
using jia::TriOmniChassis::Chassis;
#endif

#if JIA_USE_FOUR_STEER_CHASSIS
using jia::FourSteerChassis::Chassis;
#endif

#endif // CHASSIS_H_
