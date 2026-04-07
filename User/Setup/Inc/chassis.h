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
            bool is_world_speed_mode_; // 是否为世界速度模式
            bool is_lock_now_rot_z_;   // 是否固定当前rot_z
            bool is_lock_to_rot_z_;    // 是否固定到rot_z

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
            f32 max_wheel_omega_ = rpmToRadsF32(350.0f); // 最大轮子角速度，单位：rad/s
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
            f32 max_lock_to_rot_z_radio_ = 1.79999995; // 最大固定到rot_z转动系数，单位：rad/s

            // 调试参数
            bool is_debug_ = false; // 是否开启调试模式
            u8 debug_mode_ = 0;        // 调试模式

            u8 debug_wheel_index_ = 2; // 调试轮子索引
            f32 debug_input_ = 90.0f; // 调试输入
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
                Motor_Base *steer_motor_h[4] = {nullptr};
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
                Motor_Base *steer_motor_h = nullptr; // 舵向电机句柄
                Motor_Base *drive_motor_h = nullptr; // 轮向电机句柄

                f32 sin_rot_z;
                f32 cos_rot_z;
                f32 eq_radius; // 等效半径，equivalent radius，可以是负值，单位：米

                f32 abs_sin_rot_z; // 正弦值的绝对值
                f32 abs_cos_rot_z; // 余弦值的绝对值
                f32 abs_eq_radius; // 等效半径的绝对值，单位：米

                Motor_Base *&smh = steer_motor_h;
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
                bool is_clear_ = true;                 // 是否清除数据
                bool is_wheel_torque_free_ = false;    // 是否为轮子扭矩自由模式
                Coordinate coord_ = Coordinate::kBody; // 速度坐标系
                bool is_lock_now_rot_z_;               // 是否固定当前rot_z
                bool is_lock_to_rot_z_;                // 是否固定到rot_z
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
            ModeFlag mode_flag;

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
            WheelConfig &w1_ = wheel_config_[0];
            WheelConfig &w2_ = wheel_config_[1];
            WheelConfig &w3_ = wheel_config_[2];
            WheelConfig &w4_ = wheel_config_[3];

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
            bool is_debug_ = false; // 是否开启调试模式
            u8 debug_mode_ = 0;     // 调试模式

            u8 debug_wheel_index_ = 0; // 调试轮子索引

            bool is_step_signal_ = false; // 是否使用阶跃信号

            bool is_sine_ = false; // 是否使用正弦波
            f32 sine_amplitude_ = 0.0f;
            f32 sine_frequency_ = 0.1f;
            f32 sine_offset_ = 0.0f;

            f32 debug_input_ = 90.0f; // 调试输入

            bool is_wheel_speed_mode_ = false;   // 是否为轮子速度模式
            bool is_wheel_current_mode_ = false; // 是否为轮子电流模式

            Debug_Printf debug_uart_ = Debug_Printf(&huart8); // 调试串口
            u8 printf_period_ms_ = 5;                         // 串口调试打印周期，单位：毫秒
            u8 printf_period_count_ = 0;                      // 串口调试打印周期计数器

            RmPocketData_t airjoy_data_; // 从AirJoy接收的数据

        private:
            void clearInputTargetData();
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
            // TODO
            return 0.0f;
        }

        inline f32 Chassis::getTargetBodyVelY() const
        {
            // TODO
            return 0.0f;
        }

        inline f32 Chassis::getTargetWorldVelX() const
        {
            // TODO
            return 0.0f;
        }

        inline f32 Chassis::getTargetWorldVelY() const
        {
            // TODO
            return 0.0f;
        }

        inline f32 Chassis::getTargetOmegaZ() const
        {
            // TODO
            return 0.0f;
        }

        inline f32 Chassis::getCurrentBodyVelX() const
        {
            // TODO
            return 0.0f;
        }

        inline f32 Chassis::getCurrentBodyVelY() const
        {
            // TODO
            return 0.0f;
        }

        inline f32 Chassis::getCurrentWorldVelX() const
        {
            // TODO
            return 0.0f;
        }

        inline f32 Chassis::getCurrentWorldVelY() const
        {
            // TODO
            return 0.0f;
        }

        inline f32 Chassis::getCurrentOmegaZ() const
        {
            // TODO
            return 0.0f;
        }
    }
}

using jia::TriOmniChassis::Chassis;

#endif // CHASSIS_H_
