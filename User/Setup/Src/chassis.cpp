#include "User/Setup/Inc/chassis.h"

#include <cmath>

#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "BSP_TimeDwt.h"
#include "BSP_TimeUs64.h"
#include "BSP_TimeStamp.h"
#include "Module_CrsfReceiver.h"
#include "Module_HWT.h"
#include "APP_PID.h"

#include "RC10_LIB/APP/Inc/APP_Utils.h"
#include "chassis.h"

extern "C" {
volatile JiaChassisDebugWatch g_jia_chassis_debug_watch = {};
}

namespace
{
    void InitChassisPerfCounter()
    {
        jia::TimeDwt::Init(HAL_RCC_GetHCLKFreq());
    }

    void PublishChassisPerfCounter(uint32_t chassis_type,
                                   TickType_t tick_ms,
                                   uint32_t start_cycle,
                                   uint64_t start_us)
    {
        const uint32_t end_cycle = jia::TimeDwt::GetCycle32();
        const uint64_t end_us = jia::TimeStampUs64::GetTimeUs();
        const uint32_t elapsed_cycles = jia::TimeDwt::GetElapsedCycles32(start_cycle, end_cycle);
        const uint32_t elapsed_dwt_us = jia::TimeDwt::CyclesToUs32(elapsed_cycles);
        const uint64_t elapsed_us64 = (end_us >= start_us) ? (end_us - start_us) : 0ULL;

        g_jia_chassis_debug_watch.active_chassis_type = chassis_type;
        ++g_jia_chassis_debug_watch.loop_count;
        g_jia_chassis_debug_watch.last_tick_ms = static_cast<uint32_t>(tick_ms);
        g_jia_chassis_debug_watch.dwt_last_cycles = elapsed_cycles;
        g_jia_chassis_debug_watch.dwt_last_us = elapsed_dwt_us;
        if (g_jia_chassis_debug_watch.dwt_min_us == 0U ||
            elapsed_dwt_us < g_jia_chassis_debug_watch.dwt_min_us)
        {
            g_jia_chassis_debug_watch.dwt_min_us = elapsed_dwt_us;
        }
        if (elapsed_dwt_us > g_jia_chassis_debug_watch.dwt_max_us)
        {
            g_jia_chassis_debug_watch.dwt_max_us = elapsed_dwt_us;
        }

        g_jia_chassis_debug_watch.us64_last_us = elapsed_us64;
        if (g_jia_chassis_debug_watch.us64_min_us == 0ULL ||
            elapsed_us64 < g_jia_chassis_debug_watch.us64_min_us)
        {
            g_jia_chassis_debug_watch.us64_min_us = elapsed_us64;
        }
        if (elapsed_us64 > g_jia_chassis_debug_watch.us64_max_us)
        {
            g_jia_chassis_debug_watch.us64_max_us = elapsed_us64;
        }
    }
} // namespace

namespace jia
{
    namespace TriOmniChassis
    {
        void Chassis::init(InitConfig &config)
        {
            InitChassisPerfCounter();

            const osThreadAttr_t thread_attributes = {
                .name = "chassis_thread",
                .stack_size = 500 * 4,
                .priority = (osPriority_t)(osPriorityAboveNormal7),
            };

            osThreadId_t thread_handle = NULL;
            thread_handle = osThreadNew(this->createThread, this, &thread_attributes);
            if (thread_handle == NULL)
            {
                Error_Handler();
            }

            WheelConfig &wheel_1 = wheel_config_[0];
            WheelConfig &wheel_2 = wheel_config_[1];
            WheelConfig &wheel_3 = wheel_config_[2];

            // 初始化轮子位置映射关系
            //  // 1号轮子
            initWheelConfig(wheel_1, 0.0f, 0.375f, 0.0f, config.motor_handle[0]);
            //  // 2号轮子
            initWheelConfig(wheel_2, -0.37f, -0.375f, -63.741f + 180.0f, config.motor_handle[1]);
            //  // 3号轮子
            initWheelConfig(wheel_3, 0.37f, -0.375f, 63.741f + 180.0f, config.motor_handle[2]);

            // 计算底盘最大速度
            //  // 参数检查
            //        if (max_vel_x_radio_ + max_vel_y_radio_ + max_omega_z_radio_ != 1.0f)
            //        {
            //            Error_Handler();
            //        }
            //  // 参数计算
            max_wheel_vel_ = omegaToVelF32(max_wheel_omega_, wr_);
            //  // 1号轮子
            f32 wheel_1_max_vel_x = max_wheel_vel_ * max_vel_x_radio_ / wheel_1.ac;
            f32 wheel_1_max_vel_y = max_wheel_vel_ * max_vel_y_radio_ / wheel_1.as;
            f32 wheel_1_max_omega_z = max_wheel_vel_ * max_omega_z_radio_ / wheel_1.aeqr;
            //  // 2号轮子
            f32 wheel_2_max_vel_x = max_wheel_vel_ * max_vel_x_radio_ / wheel_2.ac;
            f32 wheel_2_max_vel_y = max_wheel_vel_ * max_vel_y_radio_ / wheel_2.as;
            f32 wheel_2_max_omega_z = max_wheel_vel_ * max_omega_z_radio_ / wheel_2.aeqr;
            //  // 3号轮子
            f32 wheel_3_max_vel_x = max_wheel_vel_ * max_vel_x_radio_ / wheel_3.ac;
            f32 wheel_3_max_vel_y = max_wheel_vel_ * max_vel_y_radio_ / wheel_3.as;
            f32 wheel_3_max_omega_z = max_wheel_vel_ * max_omega_z_radio_ / wheel_3.aeqr;
            //  // 计算底盘最大速度
            max_vel_x_ = minOfThree(wheel_1_max_vel_x, wheel_2_max_vel_x, wheel_3_max_vel_x);
            max_vel_y_ = minOfThree(wheel_1_max_vel_y, wheel_2_max_vel_y, wheel_3_max_vel_y);
            max_omega_z_ = minOfThree(wheel_1_max_omega_z, wheel_2_max_omega_z, wheel_3_max_omega_z);

            // 初始化omega_z_pid
            omega_z_pid_.set_params(omega_z_pid_init_config, 0.0f);

            // 初始化rot_z_pid
            rot_z_pid_.set_params(lock_angle_pid_params, 0.0f);
            rot_z_pid_.set_as_circular();

            // wheel_config_[0].motor_handle->speed_pid_.is_forward_ = true;
            // wheel_config_[1].motor_handle->speed_pid_.is_forward_ = true;
            // wheel_config_[2].motor_handle->speed_pid_.is_forward_ = true;
        }

        void Chassis::createThread(void *arg)
        {
            Chassis *chassis = static_cast<Chassis *>(arg);
            chassis->runThread(NULL);
        }

        void Chassis::runThread(void *arg)
        {
            InputTargetData &it = input_target_data_;
            Data &t = target_data_;
            Data &p = planned_data_;
            Data &lp = last_planned_data_;
            Data &c = current_data_;

            HWT101CT *hwt = HWT101CT::GetInstance(&huart8);
            time_ms_ = xTaskGetTickCount();

            for (;;)
            {
                const uint32_t chassis_task_start_cycle = TimeDwt::GetCycle32();
                const uint64_t chassis_task_start_us = TimeStampUs64::GetTimeUs();

                input_hwt_rot_z_ = hwt->get_yaw_rad();
                input_hwt_omega_z_ = hwt->get_yaw_speed_rad();

                isDebugMode();

                setModeFlag();

                if (cmf_.is_world_speed_mode)
                    {
                        jia::transSpeedWorldToBody(it.vel_x, it.vel_y, input_hwt_rot_z_, t.vel_x, t.vel_y);
                    }
                    else
                    {
                        t.vel_x = it.vel_x;
                        t.vel_y = it.vel_y;
                    }

                if (cmf_.is_lock_now_rot_z)
                    isLockNowRotZ(cmf_.is_lock_now_rot_z, t.rot_z, it.omega_z, t.rot_z, t.omega_z);
                if (cmf_.is_lock_to_rot_z)
                    isLockToRotZ(cmf_.is_lock_to_rot_z, it.rot_z, t.rot_z, t.rot_z, it.omega_z, t.omega_z);

                // 逆运动学解算
                //  // 限制车端的目标速度
                jia::clampTargetSpeedInChassis(t.vel_x, t.vel_y, t.omega_z,
                                               max_vel_x_, max_vel_y_, max_omega_z_,
                                               t.vel_x, t.vel_y, t.omega_z);
                //  // 是否开启车端的omega_z闭环控制
                if (is_omega_z_close_loop_)
                {
                    calculatePid(omega_z_pid_, omega_z_pid_count_, omega_z_pid_period_,
                                 t.omega_z, input_hwt_omega_z_, target_pid_omega_z);
                }
                else
                {
                    target_pid_omega_z = t.omega_z;
                }
                //  // 计算轮端的目标角速度
                inverseKinematics(t.vel_x, t.vel_y, target_pid_omega_z, t.w1_omega, t.w2_omega, t.w3_omega);
                //  // 是否限制轮端的目标角速度
                if (is_wheel_omega_limit_)
                {
                    // 限制轮端的目标角速度
                    f32 vel_scale_ratio = scaleThreeValuesToMaxF32(t.w1_omega, t.w2_omega, t.w3_omega,
                                                                   max_wheel_omega_, max_wheel_omega_, max_wheel_omega_,
                                                                   t.w1_omega, t.w2_omega, t.w3_omega);
                    // 计算车端的目标速度
                    t.vel_x *= vel_scale_ratio;
                    t.vel_y *= vel_scale_ratio;
                    target_pid_omega_z *= vel_scale_ratio;
                }
                //  // 是否限制车端的规划加速度
                jia::limitChassisAcceleration(is_chassis_acc_limit_,
                                              t.vel_x, t.vel_y, target_pid_omega_z,
                                              p.vel_x, p.vel_y, p.omega_z,
                                              period_,
                                              max_acc_xy_acc_, max_acc_xy_dec_,
                                              max_alpha_z_acc_, max_alpha_z_dec_,
                                              p.vel_x, p.vel_y, p.omega_z);
                //  // 计算车端的规划加速度
                p.acc_x = (p.vel_x - lp.vel_x) / period_;
                p.acc_y = (p.vel_y - lp.vel_y) / period_;
                p.alpha_z = (p.omega_z - lp.omega_z) / period_;
                //  // 计算轮端的规划角速度
                inverseKinematics(p.vel_x, p.vel_y, p.omega_z, p.w1_omega, p.w2_omega, p.w3_omega);
                //  // 计算轮端的规划角加速度
                p.w1_alpha = (p.w1_omega - lp.w1_omega) / period_;
                p.w2_alpha = (p.w2_omega - lp.w2_omega) / period_;
                p.w3_alpha = (p.w3_omega - lp.w3_omega) / period_;
                //  // 是否限制轮端的规划角加速度
                if (is_wheel_alpha_limit_)
                {
                    // 限制轮端的规划角速度
                    f32 acc_scale_ratio = scaleThreeValuesToMaxF32(p.w1_alpha, p.w2_alpha, p.w3_alpha,
                                                                   max_wheel_alpha_, max_wheel_alpha_, max_wheel_alpha_,
                                                                   p.w1_alpha, p.w2_alpha, p.w3_alpha);
                    // 计算车端的规划角速度
                    p.acc_x *= acc_scale_ratio;
                    p.acc_y *= acc_scale_ratio;
                    p.alpha_z *= acc_scale_ratio;
                    // 计算轮端的规划角速度
                    p.w1_omega = acc_scale_ratio * (p.w1_omega - lp.w1_omega) + lp.w1_omega;
                    p.w2_omega = acc_scale_ratio * (p.w2_omega - lp.w2_omega) + lp.w2_omega;
                    p.w3_omega = acc_scale_ratio * (p.w3_omega - lp.w3_omega) + lp.w3_omega;
                }
                // 发送转速指令
                if (it.mode == Mode::kWheelTorqueFreeMode)
                {
                    setWheelTargetCurrent(w1_, 0.0f);
                    setWheelTargetCurrent(w2_, 0.0f);
                    setWheelTargetCurrent(w3_, 0.0f);
                }
                else
                {
                    setWheelTargetOmega(w1_, p.w1_omega);
                    setWheelTargetOmega(w2_, p.w2_omega);
                    setWheelTargetOmega(w3_, p.w3_omega);
                }

                // 正运动学解算
                //  // 读取三个电机的反馈
                c.w1_omega = getWheelCurrentOmega(w1_);
                c.w2_omega = getWheelCurrentOmega(w2_);
                c.w3_omega = getWheelCurrentOmega(w3_);

                // 保存当前数据为上一次数据
                lp = p;
                // lmf_ = cmf_;

                // debug_uart_.printf_DMA("%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n", time_ms_, t.w1_omega, t.w2_omega, t.w3_omega, p.w1_omega, p.w2_omega, p.w3_omega, c.w1_omega, c.w2_omega, c.w3_omega);
                // debug_uart_.printf_DMA("%lu\r\n", time_ms_);
                // debug_uart_.printf_DMA("%lu,%f,%f,%f,%f\r\n", time_ms_, t.w1_omega, p.w1_omega, std::abs(c.w1_omega), std::abs(c.w2_omega));
                // debug_uart_.printf_DMA("%f,%f,%f\r\n", input_hwt_omega_z_, input_hwt_rot_z_, tpid.omega_z);

                // f32 t_current = getWheelTargetCurrent(w3_);
                // f32 c_current = getWheelCurrentCurrent(w3_);

                // printf_period_count_++;
                // if (printf_period_count_ >= printf_period_ms_)
                // {
                //     printf_period_count_ = 0;
                // debug_uart_.printf_DMA("%f,%f,%f,%f,%f\r\n", it.omega_z, input_hwt_omega_z_, tpid.omega_z, t.w3_omega, c.w3_omega);
                // debug_uart_.printf_DMA("%f,%f,%f,%f,%f,%f\r\n",
                //                        radsToRpmF32(t.w1_omega), radsToRpmF32(t.w2_omega), radsToRpmF32(t.w3_omega),
                //                        radsToRpmF32(c.w1_omega), radsToRpmF32(c.w2_omega), radsToRpmF32(c.w3_omega));
                // debug_uart_.printf_DMA("%f,%f,%f,%f\r\n", radsToRpmF32(t.w3_omega), radsToRpmF32(c.w3_omega), t_current, c_current);
                // debug_uart_.printf_DMA("%f,%f,%f,%f,%f\r\n", radsToRpmF32(t.w3_omega), radsToRpmF32(c.w3_omega), t.omega_z, tpid.omega_z,input_hwt_omega_z_);
                // debug_uart_.printf_DMA("%f,%f,%f\r\n", it.rot_z, input_hwt_rot_z_, t.omega_z);
                // }

                g_jia_chassis_debug_watch.mode = static_cast<uint32_t>(it.mode);
                g_jia_chassis_debug_watch.debug_mode = debug_mode_;
                g_jia_chassis_debug_watch.is_debug = is_debug_ ? 1U : 0U;
                g_jia_chassis_debug_watch.is_world_speed_mode = cmf_.is_world_speed_mode ? 1U : 0U;
                g_jia_chassis_debug_watch.is_lock_now_rot_z = cmf_.is_lock_now_rot_z ? 1U : 0U;
                g_jia_chassis_debug_watch.is_lock_to_rot_z = cmf_.is_lock_to_rot_z ? 1U : 0U;
                g_jia_chassis_debug_watch.tri_target_vx_m_s = t.vel_x;
                g_jia_chassis_debug_watch.tri_target_vy_m_s = t.vel_y;
                g_jia_chassis_debug_watch.tri_target_wz_rad_s = target_pid_omega_z;
                g_jia_chassis_debug_watch.tri_planned_vx_m_s = p.vel_x;
                g_jia_chassis_debug_watch.tri_planned_vy_m_s = p.vel_y;
                g_jia_chassis_debug_watch.tri_planned_wz_rad_s = p.omega_z;
                g_jia_chassis_debug_watch.tri_wheel_target_omega_rad_s[0] = p.w1_omega;
                g_jia_chassis_debug_watch.tri_wheel_target_omega_rad_s[1] = p.w2_omega;
                g_jia_chassis_debug_watch.tri_wheel_target_omega_rad_s[2] = p.w3_omega;
                g_jia_chassis_debug_watch.tri_wheel_feedback_omega_rad_s[0] = c.w1_omega;
                g_jia_chassis_debug_watch.tri_wheel_feedback_omega_rad_s[1] = c.w2_omega;
                g_jia_chassis_debug_watch.tri_wheel_feedback_omega_rad_s[2] = c.w3_omega;
                PublishChassisPerfCounter(3U, time_ms_, chassis_task_start_cycle, chassis_task_start_us);

                vTaskDelayUntil(&time_ms_, period_ms_);
            }
        }

        void Chassis::calculatePid(PID_Incremental &pid, u8 &count, u8 period, f32 target, f32 feedback, f32 &output)
        {
            if (count >= period)
            {
                count = 0;
                output = pid.pid_calc(target, feedback);
            }
            count++;
        }

        void Chassis::calculatePid(PID_Position &pid, u8 &count, u8 period, f32 target, f32 feedback, f32 &output)
        {
            if (count >= period)
            {
                count = 0;
                output = pid.pid_calc(target, feedback);
            }
            count++;
        }

        void Chassis::setModeFlag()
        {
            switch (input_target_data_.mode)
            {
            case Mode::kWheelTorqueFreeMode:
                cmf_.is_world_speed_mode = false;
                cmf_.is_lock_now_rot_z = false;
                cmf_.is_lock_to_rot_z = false;
                break;
            case Mode::kBodySpeedMode:
                cmf_.is_world_speed_mode = false;
                cmf_.is_lock_now_rot_z = false;
                cmf_.is_lock_to_rot_z = false;
                break;
            case Mode::kBodySpeedLockNowRotZMode:
                cmf_.is_world_speed_mode = false;
                cmf_.is_lock_now_rot_z = true;
                cmf_.is_lock_to_rot_z = false;
                break;
            case Mode::kBodySpeedLockToRotZMode:
                cmf_.is_world_speed_mode = false;
                cmf_.is_lock_now_rot_z = false;
                cmf_.is_lock_to_rot_z = true;
                break;
            case Mode::kWorldSpeedMode:
                cmf_.is_world_speed_mode = true;
                cmf_.is_lock_now_rot_z = false;
                cmf_.is_lock_to_rot_z = false;
                break;
            case Mode::kWorldSpeedLockNowRotZMode:
                cmf_.is_world_speed_mode = true;
                cmf_.is_lock_now_rot_z = true;
                cmf_.is_lock_to_rot_z = false;
                break;
            case Mode::kWorldSpeedLockToRotZMode:
                cmf_.is_world_speed_mode = true;
                cmf_.is_lock_now_rot_z = false;
                cmf_.is_lock_to_rot_z = true;
                break;
            case Mode::kWorldSpeedLockNowRotZWithNoOmegaZMode:
                cmf_.is_world_speed_mode = true;
                cmf_.is_lock_now_rot_z = true;
                cmf_.is_lock_to_rot_z = false;
                break;
            case Mode::kBodySpeedLockNowRotZWithNoOmegaZMode:
                cmf_.is_world_speed_mode = false;
                cmf_.is_lock_now_rot_z = true;
                cmf_.is_lock_to_rot_z = false;
                break;
            default:
                break;
            }
        }

        void Chassis::isDebugMode()
        {
            if (is_debug_)
            {
                CrsfReceiver *receiver = CrsfReceiver::GetInstance(&huart7);

                receiver->getControlData(&airjoy_data_);

                f32 target_vel_x = 0.0f;
                f32 target_vel_y = 0.0f;
                f32 target_omega_z = 0.0f;

                target_vel_x = airjoy_data_.left_x * max_vel_x_;
                target_vel_y = airjoy_data_.left_y * max_vel_y_;

                // if (airjoy_data_.right_x > 0.1f)
                // {
                //     target_omega_z = max_omega_z_;
                // }
                // else if (airjoy_data_.right_x < -0.1f)
                // {
                //     target_omega_z = -max_omega_z_;
                // }
                // else
                // {
                //     target_omega_z = 0.0f;
                // }

                target_omega_z = airjoy_data_.right_x * max_omega_z_;

                if (is_sine_)
                {
                    target_omega_z = sineWaveGeneratorF32(time_ms_ / 1000.0f, sine_amplitude_, sine_frequency_, 0.0f, sine_offset_);
                }
                else if (is_step_signal_)
                {
                    if (airjoy_data_.right_x > 0.3f)
                    {
                        target_omega_z = max_omega_z_;
                    }
                    else if (airjoy_data_.right_x < -0.3f)
                    {
                        target_omega_z = -max_omega_z_;
                    }
                    else
                    {
                        target_omega_z = 0.0f;
                    }
                }

                switch (debug_mode_)
                {
                default:
                case 0:
                {
                    setWheelTorqueFreeMode();
                    break;
                }
                case 1:
                {
                    setTargetBodySpeedMode(target_vel_x, target_vel_y, target_omega_z);
                    break;
                }
                case 2:
                {
                    setTargetWorldSpeedMode(target_vel_x, target_vel_y, target_omega_z);
                    break;
                }
                case 3:
                {
                    setTargetBodySpeedLockNowRotZMode(target_vel_x, target_vel_y);
                    break;
                }
                case 4:
                {
                    setTargetWorldSpeedLockNowRotZMode(target_vel_x, target_vel_y);
                    break;
                }
                case 5:
                {
                    setTargetBodySpeedLockToRotZMode(target_vel_x, target_vel_y, debug_lock_rot_z_);
                    break;
                }
                case 6:
                {
                    setTargetWorldSpeedLockToRotZMode(target_vel_x, target_vel_y, debug_lock_rot_z_);
                    break;
                }
                case 7:
                {
                    setTargetBodySpeedLockNowRotZWithNoOmegaZMode(target_vel_x, target_vel_y, target_omega_z);
                    break;
                }
                case 8:
                {
                    setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(target_vel_x, target_vel_y, target_omega_z);
                    break;
                }
                }

                // // 调试轮子
                // auto &wheel = wheel_config_[debug_wheel_index_];

                // f32 t_rpm = 0.0f;

                // if (is_sine_)
                // {
                //     t_rpm = sineWaveGeneratorF32(time_ms_ / 1000.0f, sine_amplitude_, sine_frequency_, 0.0f, sine_offset_);
                // }
                // else if (is_step_signal_)
                // {
                //     if (airjoy_data_.left_x > 0.3f)
                //     {
                //         t_rpm = debug_input_;
                //     }
                //     else if (airjoy_data_.left_x < -0.3f)
                //     {
                //         t_rpm = -debug_input_;
                //     }
                //     else
                //     {
                //         t_rpm = 0.0f;
                //     }
                // }
                // else
                // {
                //     t_rpm = airjoy_data_.left_x * debug_input_;
                // }

                // if (is_wheel_speed_mode_)
                // {
                //     setWheelTargetOmega(wheel, rpmToRadsF32(t_rpm));
                // }
                // else if (is_wheel_current_mode_)
                // {
                //     setWheelTargetCurrent(wheel, t_rpm);
                // }
                // else
                // {
                //     setWheelTargetCurrent(wheel, 0.0f);
                // }

                // f32 pid_error = wheel_handle->speed_pid_.error_;
                // // f32 pid_error_last = wheel_handle->speed_pid_.error_last_;
                // // f32 pid_error_earlier_ = wheel_handle->speed_pid_.error_earlier_;
                // f32 pid_p = wheel_handle->speed_pid_.P_Term;
                // f32 pid_i = wheel_handle->speed_pid_.I_Term;
                // f32 pid_d = wheel_handle->speed_pid_.D_Term;
                // f32 pid_output = wheel_handle->speed_pid_.output_;

                // f32 t_current = getWheelTargetCurrent(wheel);

                // f32 c_current = getWheeCurrentCurrent(wheel);
                // f32 c_rpm = getWheelCurrentRpm(wheel);

                // printf_period_count_++;
                // if (printf_period_count_ >= printf_period_ms_)
                // {
                //     printf_period_count_ = 0;
                //     debug_uart_.printf_DMA("%f,%f,%f,%f\r\n", t_rpm, c_rpm, t_current, c_current);
                // }
            }
        }

        Chassis::Result Chassis::setWheelTorqueFreeMode()
        {
            clearInputTargetData();
            input_target_data_.mode = Mode::kWheelTorqueFreeMode;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetBodySpeedMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            clearInputTargetData();
            input_target_data_.mode = Mode::kBodySpeedMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = omega_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetBodySpeedLockNowRotZMode(f32 vel_x, f32 vel_y)
        {
            clearInputTargetData();
            input_target_data_.mode = Mode::kBodySpeedLockNowRotZMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetBodySpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z)
        {
            clearInputTargetData();
            input_target_data_.mode = Mode::kBodySpeedLockToRotZMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.rot_z = rot_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetWorldSpeedMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            clearInputTargetData();
            input_target_data_.mode = Mode::kWorldSpeedMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = omega_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetWorldSpeedLockNowRotZMode(f32 vel_x, f32 vel_y)
        {
            clearInputTargetData();
            input_target_data_.mode = Mode::kWorldSpeedLockNowRotZMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetBodySpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            clearInputTargetData();
            input_target_data_.mode = Mode::kBodySpeedLockNowRotZWithNoOmegaZMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = omega_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            clearInputTargetData();
            input_target_data_.mode = Mode::kWorldSpeedLockNowRotZWithNoOmegaZMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = omega_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetWorldSpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z)
        {
            clearInputTargetData();
            input_target_data_.mode = Mode::kWorldSpeedLockToRotZMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.rot_z = rot_z;
            return Result::kOk;
        }

        f32 Chassis::getTargetBodyVelX() const
        {
            return planned_data_.vel_x;
        }

        f32 Chassis::getTargetBodyVelY() const
        {
            return planned_data_.vel_y;
        }

        f32 Chassis::getTargetWorldVelX() const
        {
            return planned_data_.vel_x;
        }

        f32 Chassis::getTargetWorldVelY() const
        {
            return planned_data_.vel_y;
        }

        f32 Chassis::getTargetOmegaZ() const
        {
            return planned_data_.omega_z;
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
            return current_data_.vel_x;
        }

        f32 Chassis::getCurrentWorldVelY() const
        {
            return current_data_.vel_y;
        }

        f32 Chassis::getCurrentOmegaZ() const
        {
            return current_data_.omega_z;
        }

        void Chassis::inverseKinematics(f32 in_x, f32 in_y, f32 in_z, f32 &out_w1, f32 &out_w2, f32 &out_w3)
        {
            out_w1 = ((in_x * w1_.c + in_y * w1_.s + in_z * w1_.eqr) / wr_);
            out_w2 = ((in_x * w2_.c + in_y * w2_.s + in_z * w2_.eqr) / wr_);
            out_w3 = ((in_x * w3_.c + in_y * w3_.s + in_z * w3_.eqr) / wr_);
        }

        void Chassis::isLockNowRotZ(bool is_lock, f32 rot_z, f32 omega_z, f32 &out_rot_z, f32 &out_omega_z)
        {
            if (is_lock)
            {
                if (omega_z == 0.0f)
                {
                    if (lock_now_rot_z_shift_count_ > 0)
                    {
                        lock_now_rot_z_shift_count_--;
                        out_rot_z = input_hwt_rot_z_;
                        out_omega_z = omega_z;
                    }
                    else
                    {
                        calculatePid(rot_z_pid_, rot_z_pid_count_, rot_z_pid_period_,
                                     radToDegF32(rot_z), radToDegF32(input_hwt_rot_z_),
                                     out_omega_z);
                    }
                }
                else
                {
                    out_omega_z = omega_z;
                    out_rot_z = input_hwt_rot_z_;

                    lock_now_rot_z_shift_count_ = lock_now_rot_z_shift_time_ms_;
                }
            }
            else
            {
                out_omega_z = omega_z;
                out_rot_z = rot_z;
            }
        }

        void Chassis::initWheelConfig(WheelConfig &wheel, f32 pos_x, f32 pos_y, f32 rot_z_deg, M3508 *motor_handle)
        {
            wheel.pos_x = pos_x,
            wheel.pos_y = pos_y,
            wheel.rot_z_deg = rot_z_deg,
            wheel.motor_handle = motor_handle,
            wheel.s = sinDegF32(wheel.rot_z_deg);
            wheel.c = cosDegF32(wheel.rot_z_deg);
            wheel.eqr = wheel.pos_x * wheel.s - wheel.pos_y * wheel.c;
            wheel.as = std::abs(wheel.s);
            wheel.ac = std::abs(wheel.c);
            wheel.aeqr = std::abs(wheel.eqr);
        }

        void Chassis::setWheelTargetCurrent(WheelConfig &wheel, f32 current)
        {
            wheel.h->setTargetCurrent(current * 1000.0f);
        }

        void Chassis::setWheelTargetOmega(WheelConfig &wheel, f32 omega)
        {
            wheel.h->setTargetRPM(radsToRpmF32(omega));
        }

        f32 Chassis::getWheelCurrentOmega(const WheelConfig &wheel) const
        {
            return rpmToRadsF32(wheel.h->getRPM());
        }

        f32 Chassis::getWheelTargetCurrent(const WheelConfig &wheel) const
        {
            return wheel.h->getTargetCurrent();
        }

        f32 Chassis::getWheeCurrentCurrent(const WheelConfig &wheel) const
        {
            return wheel.h->getCurrent();
        }

        f32 Chassis::getWheelCurrentRpm(const WheelConfig &wheel) const
        {
            return wheel.h->getRPM();
        }

        void Chassis::clearInputTargetData()
        {
            input_target_data_.mode = Mode::kWheelTorqueFreeMode;
            input_target_data_.vel_x = 0.0f;
            input_target_data_.vel_y = 0.0f;
            input_target_data_.omega_z = 0.0f;
            input_target_data_.rot_z = 0.0f;
        }

        void Chassis::clearData(Data &data)
        {
            memset(&data, 0, sizeof(Data));
        }

        void Chassis::isLockToRotZ(bool is_lock, f32 tar_rot_z, f32 cur_rot_z, f32 &out_rot_z, f32 omega_z, f32 &out_omega_z)
        {
            if (is_lock)
            {
                out_rot_z = limit1DPiAngleRateByTimeF32(tar_rot_z, cur_rot_z, period_, max_lock_to_rot_z_radio_);

                calculatePid(rot_z_pid_, rot_z_pid_count_, rot_z_pid_period_,
                             radToDegF32(out_rot_z), radToDegF32(input_hwt_rot_z_),
                             out_omega_z);
            }
            else
            {
                out_omega_z = omega_z;
                out_rot_z = tar_rot_z;
            }
        }
    }

    namespace FourSteerChassis
    {
        void Chassis::initWheelConfig(WheelConfig &wheel, f32 pos_x, f32 pos_y, f32 rot_z_deg, M3508 *steer_motor_h, Motor_Base *drive_motor_h)
        {
            wheel.pos_x = pos_x;
            wheel.pos_y = pos_y;
            wheel.rot_z_deg = rot_z_deg;
            wheel.steer_motor_h = steer_motor_h;
            wheel.drive_motor_h = drive_motor_h;
            wheel.s = sinDegF32(rot_z_deg);
            wheel.c = cosDegF32(rot_z_deg);
            wheel.eqr = pos_x * wheel.s - pos_y * wheel.c;
            wheel.as = std::abs(wheel.s);
            wheel.ac = std::abs(wheel.c);
            wheel.aeqr = std::abs(wheel.eqr);
        }

        void Chassis::configureDefaultSwerve()
        {
            swerve_config_ = {};
            swerve_config_.shared.wheel_radius_m = swr_;
            swerve_config_.shared.max_drive_omega_rad_s = max_wheel_omega_;
            swerve_config_.shared.max_drive_alpha_rad_s2 = max_acc_xy_acc_ / swr_;
            swerve_config_.shared.max_steer_rate_rad_s = rpmToRadsF32(400.0f);
            swerve_config_.shared.max_steer_alpha_rad_s2 = rpmToRadsF32(800.0f);
            swerve_config_.shared.stationary_speed_epsilon_m_s = 0.001f;
            swerve_config_.shared.enable_cosine_compensation = true;
            swerve_config_.shared.enable_actuator_dynamics = true;

            const f32 half_width_m = 0.37f;
            const f32 half_length_m = 0.375f;
            const f32 module_rot_deg[4] = {45.0f, 135.0f, -135.0f, -45.0f};
            const f32 module_x_m[4] = {half_width_m, -half_width_m, -half_width_m, half_width_m};
            const f32 module_y_m[4] = {half_length_m, half_length_m, -half_length_m, -half_length_m};

            for (std::size_t index = 0; index < jia::swerve::kModuleCount; ++index)
            {
                initWheelConfig(wheel_config_[index],
                                module_x_m[index],
                                module_y_m[index],
                                module_rot_deg[index],
                                wheel_config_[index].steer_motor_h,
                                wheel_config_[index].drive_motor_h);

                auto &module = swerve_config_.modules[index];
                module.geometry.pos_x_m = module_x_m[index];
                module.geometry.pos_y_m = module_y_m[index];
                module.geometry.theta_oa_to_owi_rad = 0.0f;
                module.geometry.steer_motor_sign = 1.0f;
                module.geometry.drive_motor_sign = 1.0f;
                module.homing.enabled = false;
            }

            swerve_controller_.configure(swerve_config_);
            swerve_runtime_debug_ = {};
            swerve_runtime_debug_.config_ready = true;
        }

        bool Chassis::buildRuntimeSwerveMotion(jia::swerve::ChassisCommand &out_command)
        {
            if (itd_.mode == Mode::kWheelTorqueFreeMode)
            {
                out_command = {};
                td_ = {};
                pd_ = {};
                return false;
            }

            f32 body_vel_x = itd_.vel_x;
            f32 body_vel_y = itd_.vel_y;
            if (itd_.mode == Mode::kWorldSpeedMode ||
                itd_.mode == Mode::kWorldSpeedLockNowRotZMode ||
                itd_.mode == Mode::kWorldSpeedLockToRotZMode ||
                itd_.mode == Mode::kWorldSpeedLockNowRotZWithNoOmegaZMode)
            {
                const f32 cos_theta = cosf(input_hwt_rot_z_);
                const f32 sin_theta = sinf(input_hwt_rot_z_);
                body_vel_x = itd_.vel_x * cos_theta + itd_.vel_y * sin_theta;
                body_vel_y = -itd_.vel_x * sin_theta + itd_.vel_y * cos_theta;
            }

            f32 target_rot_z = itd_.rot_z;
            f32 target_omega_z = itd_.omega_z;
            (void)target_rot_z;

            td_.vel_x = clampValue(body_vel_x, -max_vel_x_, max_vel_x_);
            td_.vel_y = clampValue(body_vel_y, -max_vel_y_, max_vel_y_);
            td_.omega_z = clampValue(target_omega_z, -max_omega_z_, max_omega_z_);
            td_.rot_z = input_hwt_rot_z_;
            pd_ = td_;

            out_command.vx_m_s = pd_.vel_x;
            out_command.vy_m_s = pd_.vel_y;
            out_command.wz_rad_s = pd_.omega_z;
            return true;
        }

        void Chassis::captureRuntimeSwerveFeedback(jia::swerve::ModuleFeedback out_feedback[jia::swerve::kModuleCount]) const
        {
            for (std::size_t index = 0; index < jia::swerve::kModuleCount; ++index)
            {
                const WheelConfig &wheel = wheel_config_[index];
                if (wheel.smh == nullptr || wheel.dmh == nullptr)
                {
                    out_feedback[index] = {};
                    continue;
                }
                out_feedback[index].steer_motor_total_angle_rad = degToRadF32(wheel.smh->getTotalAngle());
                out_feedback[index].drive_omega_rad_s = rpmToRadsF32(wheel.dmh->getRPM());
            }
        }

        void Chassis::applyRuntimeSwerveCommands(const jia::swerve::ModuleCommand commands[jia::swerve::kModuleCount])
        {
            for (std::size_t index = 0; index < jia::swerve::kModuleCount; ++index)
            {
                WheelConfig &wheel = wheel_config_[index];
                const jia::swerve::ModuleCommand &command = commands[index];
                if (wheel.smh == nullptr || wheel.dmh == nullptr)
                {
                    continue;
                }
                setSteerWheelTargetTotalAngleDeg(wheel, radToDegF32(command.selected_target_steer_motor_total_angle_rad));
                applyDriveWheelDebugCommand(wheel, radsToRpmF32(command.selected_target_drive_omega_rad_s));
            }
        }

        void Chassis::runRuntimeSwerveControl()
        {
            jia::swerve::ChassisCommand command = {};
            if (!swerve_runtime_debug_.config_ready || !buildRuntimeSwerveMotion(command))
            {
                swerve_runtime_debug_.used_controller_step = false;
                swerve_runtime_debug_.output_valid = false;
                return;
            }

            jia::swerve::ModuleFeedback feedback[jia::swerve::kModuleCount] = {};
            jia::swerve::ModuleCommand output[jia::swerve::kModuleCount] = {};
            captureRuntimeSwerveFeedback(feedback);
            swerve_controller_.step(command, feedback, period_, output);
            applyRuntimeSwerveCommands(output);

            swerve_runtime_debug_.used_controller_step = true;
            swerve_runtime_debug_.output_valid = true;
            swerve_runtime_debug_.command = command;
            for (std::size_t index = 0; index < jia::swerve::kModuleCount; ++index)
            {
                swerve_runtime_debug_.feedback[index] = feedback[index];
                swerve_runtime_debug_.output[index] = output[index];
                (&cd_.w1_steer_angle)[index] = output[index].selected_target_steer_oa_total_angle_rad;
                (&cd_.w1_drive_omega)[index] = feedback[index].drive_omega_rad_s;
                (&pd_.w1_steer_angle)[index] = output[index].selected_target_steer_oa_total_angle_rad;
                (&pd_.w1_drive_omega)[index] = output[index].selected_target_drive_omega_rad_s;
            }
            cd_.vel_x = command.vx_m_s;
            cd_.vel_y = command.vy_m_s;
            cd_.omega_z = command.wz_rad_s;

            // 填充四轮正常模式 debug watch
            g_jia_chassis_debug_watch.four_swerve_used_controller_step = 1U;
            g_jia_chassis_debug_watch.four_swerve_body_vx_m_s = command.vx_m_s;
            g_jia_chassis_debug_watch.four_swerve_body_vy_m_s = command.vy_m_s;
            g_jia_chassis_debug_watch.four_swerve_body_wz_rad_s = command.wz_rad_s;
        }

        void Chassis::init(InitConfig &config)
        {
            InitChassisPerfCounter();

            const osThreadAttr_t thread_attributes = {
                .name = "chassis_thread",
                .stack_size = 500 * 4,
                .priority = (osPriority_t)(osPriorityAboveNormal7),
            };

            osThreadId_t thread_handle = NULL;
            thread_handle = osThreadNew(this->createThread, this, &thread_attributes);
            if (thread_handle == NULL)
            {
                Error_Handler();
            }

            // 初始化轮子配置
            for (int i = 0; i < 4; i++)
            {
                wheel_config_[i].steer_motor_h = config.steer_motor_h[i];
                wheel_config_[i].drive_motor_h = config.drive_motor_h[i];
            }

            if (wheel_config_[0].steer_motor_h != nullptr)
            {
                wheel_config_[0].steer_motor_h->reset_GearRatio(8.0f);
            }

            // 计算底盘最大速度
            max_wheel_vel_ = omegaToVelF32(max_wheel_omega_, swr_);
            configureDefaultSwerve();
        }

        void Chassis::createThread(void *arg)
        {
            Chassis *chassis = static_cast<Chassis *>(arg);
            chassis->runThread(NULL);
        }

        void Chassis::setSteerWheelTargetRpm(WheelConfig &wheel, f32 rpm)
        {
            wheel.smh->setTargetRPM(rpm);
        }

        void Chassis::setSteerWheelTargetCurrent(WheelConfig &wheel, f32 current)
        {
            wheel.smh->setTargetCurrent(current);
        }

        void Chassis::setSteerWheelTargetAngleDeg(WheelConfig &wheel, f32 angle_deg)
        {
            wheel.smh->setTargetAngle(angle_deg);
        }

        void Chassis::setSteerWheelTargetTotalAngleDeg(WheelConfig &wheel, f32 total_angle_deg)
        {
            wheel.smh->setTargetTotalAngle(total_angle_deg);
        }

        f32 Chassis::getSteerWheelCurrentAngleDeg(const WheelConfig &wheel) const
        {
            return wheel.smh->getAngle();
        }

        f32 Chassis::getSteerWheelCurrentAngleDegCalibrated(const WheelConfig &wheel) const
        {
            return normalizeAngleTo180(wheel.smh->getAngle());
        }

        f32 Chassis::getSteerWheelTargetAngleDeg(const WheelConfig &wheel) const
        {
            return wheel.smh->getTargetAngle();
        }

        f32 Chassis::getSteerWheelCurrentRPM(const WheelConfig &wheel) const
        {
            return wheel.smh->getRPM();
        }

        f32 Chassis::getSteerWheelCurrentCurrent(const WheelConfig &wheel) const
        {
            return wheel.smh->getCurrent();
        }

        f32 Chassis::getSteerWheelCurrentTotalAngleDegCalibrated(const WheelConfig &wheel) const
        {
            return wheel.smh->getTotalAngle();
        }

        f32 Chassis::getSteerWheelTargetCurrent(const WheelConfig &wheel) const
        {
            return wheel.smh->getTargetCurrent();
        }

        void Chassis::setDriveWheelTargetRpm(WheelConfig &wheel, f32 rpm)
        {
            wheel.dmh->setTargetRPM(rpm);
        }
        void Chassis::setDriveWheelBrake(WheelConfig &wheel, f32 brake_current)
        {
            wheel.dmh->setBrake(brake_current);
        }
        void Chassis::setDriveWheelTargetCurrent(WheelConfig &wheel, f32 current)
        {
            wheel.dmh->setTargetCurrent(current);
        }
        void Chassis::applyDriveWheelDebugCommand(WheelConfig &wheel, f32 drive_target_rpm)
        {
            g_jia_chassis_debug_watch.four_drive_command_rpm = drive_target_rpm;
            g_jia_chassis_debug_watch.four_drive_brake_mode = 0U;
            g_jia_chassis_debug_watch.four_drive_applied_brake_current = 0.0f;

            if (is_drive_force_brake_enabled_)
            {
                setDriveWheelBrake(wheel, drive_force_brake_current_);
                g_jia_chassis_debug_watch.four_drive_brake_mode = 2U;
                g_jia_chassis_debug_watch.four_drive_applied_brake_current = drive_force_brake_current_;
                return;
            }
            if (is_drive_zero_rpm_brake_enabled_ &&
                std::abs(drive_target_rpm) <= drive_zero_rpm_threshold_rpm_)
            {
                setDriveWheelBrake(wheel, drive_zero_rpm_brake_current_);
                g_jia_chassis_debug_watch.four_drive_brake_mode = 1U;
                g_jia_chassis_debug_watch.four_drive_applied_brake_current = drive_zero_rpm_brake_current_;
                return;
            }
            setDriveWheelTargetRpm(wheel, drive_target_rpm);
        }

        f32 Chassis::getDriveWheelTargetRPM(const WheelConfig &wheel) const
        {
            return wheel.dmh->getTargetRPM();
        }

        f32 Chassis::getDriveWheelCurrentRPM(const WheelConfig &wheel) const
        {
            return wheel.dmh->getRPM();
        }

        f32 Chassis::getDriveWheelCurrentCurrent(const WheelConfig &wheel) const
        {
            return wheel.dmh->getCurrent();
        }

        f32 Chassis::buildDebugSetpoint(DebugSignalMode mode, f32 axis, f32 amplitude, f32 hand_input) const
        {
            switch (mode)
            {
            case DebugSignalMode::kStep:
                if (axis > debug_step_threshold_)
                {
                    return amplitude;
                }
                if (axis < -debug_step_threshold_)
                {
                    return -amplitude;
                }
                return 0.0f;
            case DebugSignalMode::kSine:
                return sineWaveGeneratorF32(time_ms_ / 1000.0f, amplitude, sine_frequency_, 0.0f, sine_offset_);
            case DebugSignalMode::kHandInput:
                return hand_input;
            case DebugSignalMode::kJoystick:
            default:
                return axis * amplitude;
            }
        }

        void Chassis::runThread(void *arg)
        {
            HWT101CT *hwt = HWT101CT::GetInstance(&huart8);
            time_ms_ = xTaskGetTickCount();

            for (;;)
            {
                const uint32_t chassis_task_start_cycle = TimeDwt::GetCycle32();
                const uint64_t chassis_task_start_us = TimeStampUs64::GetTimeUs();

                ihrz_ = hwt->get_yaw_rad();
                ihoz_ = hwt->get_yaw_speed_rad();

                // 仅当非单轮调试模式时运行四轮 Swerve 控制。
                // 单轮调试模式（is_wheel_speed_mode_ / is_wheel_current_mode_ /
                // is_wheel_single_position_mode_ / is_wheel_total_position_mode_）
                // 由 isDebugMode() 直接写电机寄存器，跳过 SwerveController。
                const bool is_single_wheel_debug =
                    (is_wheel_speed_mode_ || is_wheel_current_mode_ ||
                     is_wheel_single_position_mode_ || is_wheel_total_position_mode_);
                if (!is_single_wheel_debug)
                {
                    runRuntimeSwerveControl();
                }
                isDebugMode();

                // printf_period_count_++;
                // if (printf_period_count_ >= printf_period_ms_)
                // {
                //     printf_period_count_ = 0;
                // }

                PublishChassisPerfCounter(4U, time_ms_, chassis_task_start_cycle, chassis_task_start_us);

                vTaskDelayUntil(&time_ms_, period_ms_);
            }
        }

        void Chassis::isDebugMode()
        {
            if (is_debug_)
            {
                CrsfReceiver *receiver = CrsfReceiver::GetInstance(&huart7);

                receiver->getControlData(&airjoy_data_);

                f32 target_vel_x = 0.0f;
                f32 target_vel_y = 0.0f;
                f32 target_omega_z = 0.0f;

                target_vel_x = airjoy_data_.left_x * max_vel_x_;
                target_vel_y = airjoy_data_.left_y * max_vel_y_;

                // if (airjoy_data_.right_x > 0.1f)
                // {
                //     target_omega_z = max_omega_z_;
                // }
                // else if (airjoy_data_.right_x < -0.1f)
                // {
                //     target_omega_z = -max_omega_z_;
                // }
                // else
                // {
                //     target_omega_z = 0.0f;
                // }

                target_omega_z = airjoy_data_.right_x * max_omega_z_;

                if (is_sine_)
                {
                    target_omega_z = sineWaveGeneratorF32(time_ms_ / 1000.0f, sine_amplitude_, sine_frequency_, 0.0f, sine_offset_);
                }
                else if (is_step_signal_)
                {
                    if (airjoy_data_.right_x > 0.3f)
                    {
                        target_omega_z = max_omega_z_;
                    }
                    else if (airjoy_data_.right_x < -0.3f)
                    {
                        target_omega_z = -max_omega_z_;
                    }
                    else
                    {
                        target_omega_z = 0.0f;
                    }
                }

                switch (debug_mode_)
                {
                default:
                case 0:
                {
                    setWheelTorqueFreeMode();
                    break;
                }
                case 1:
                {
                    setTargetBodySpeedMode(target_vel_x, target_vel_y, target_omega_z);
                    break;
                }
                case 2:
                {
                    setTargetWorldSpeedMode(target_vel_x, target_vel_y, target_omega_z);
                    break;
                }
                case 3:
                {
                    setTargetBodySpeedLockNowRotZMode(target_vel_x, target_vel_y);
                    break;
                }
                case 4:
                {
                    setTargetWorldSpeedLockNowRotZMode(target_vel_x, target_vel_y);
                    break;
                }
                case 5:
                {
                    setTargetBodySpeedLockToRotZMode(target_vel_x, target_vel_y, debug_lock_rot_z_);
                    break;
                }
                case 6:
                {
                    setTargetWorldSpeedLockToRotZMode(target_vel_x, target_vel_y, debug_lock_rot_z_);
                    break;
                }
                case 7:
                {
                    setTargetBodySpeedLockNowRotZWithNoOmegaZMode(target_vel_x, target_vel_y, target_omega_z);
                    break;
                }
                case 8:
                {
                    setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(target_vel_x, target_vel_y, target_omega_z);
                    break;
                }
                }

                // 调试单个舵轮
                auto &wheel = wheel_config_[debug_wheel_index_];

                const f32 rc_right_x = airjoy_data_.right_x;
                const f32 rc_left_y = airjoy_data_.left_y;
                f32 steer_target = buildDebugSetpoint(steer_signal_mode_, rc_right_x, debug_input_, steer_hand_input_);
                f32 drive_target_rpm = buildDebugSetpoint(drive_signal_mode_, rc_left_y, drive_debug_input_, drive_hand_input_);

                if (is_wheel_speed_mode_)
                {
                    setSteerWheelTargetRpm(wheel, steer_target);
                }
                else if (is_wheel_current_mode_)
                {
                    setSteerWheelTargetCurrent(wheel, steer_target);
                }
                else if (is_wheel_single_position_mode_)
                {
                    setSteerWheelTargetAngleDeg(wheel, steer_target);
                }
                else if (is_wheel_total_position_mode_)
                {
                    setSteerWheelTargetTotalAngleDeg(wheel, steer_target);
                }
                else
                {
                    setSteerWheelTargetCurrent(wheel, 0.0f);
                }

                photogate_signal_ = HAL_GPIO_ReadPin(kTEST_PHOTOGATE_GPIO_Port, kTEST_PHOTOGATE_Pin);

                if (is_power_on_cailbration_)
                {
                    is_doing_cailbration_ = true;
                    is_power_on_cailbration_ = false;

                    last_photogate_signal_ = photogate_signal_;
                }

                if (is_doing_cailbration_)
                {
                    drive_target_rpm = 0.0f;
                    setSteerWheelTargetRpm(wheel, cailbration_rpm_);

                    if (photogate_signal_ != last_photogate_signal_)
                    {
                        setSteerWheelTargetRpm(wheel, 0.0f);
                        is_doing_cailbration_ = false;

                        const f32 calibrated_reference_deg = photogate_signal_ ? -180.0f : 0.0f;
                        wheel.smh->relocate_totalAngle(calibrated_reference_deg);
                        cailbration_angle_deg_ = calibrated_reference_deg;
                    }
                }

                applyDriveWheelDebugCommand(wheel, drive_target_rpm);
                f32 t_angle_deg = normalizeAngleTo180(getSteerWheelTargetAngleDeg(wheel));
                f32 c_angle_deg = getSteerWheelCurrentAngleDegCalibrated(wheel);

                const bool is_world_mode =
                    itd_.mode == Mode::kWorldSpeedMode ||
                    itd_.mode == Mode::kWorldSpeedLockNowRotZMode ||
                    itd_.mode == Mode::kWorldSpeedLockToRotZMode ||
                    itd_.mode == Mode::kWorldSpeedLockNowRotZWithNoOmegaZMode;
                const bool is_lock_now_mode =
                    itd_.mode == Mode::kBodySpeedLockNowRotZMode ||
                    itd_.mode == Mode::kWorldSpeedLockNowRotZMode ||
                    itd_.mode == Mode::kBodySpeedLockNowRotZWithNoOmegaZMode ||
                    itd_.mode == Mode::kWorldSpeedLockNowRotZWithNoOmegaZMode;
                const bool is_lock_to_mode =
                    itd_.mode == Mode::kBodySpeedLockToRotZMode ||
                    itd_.mode == Mode::kWorldSpeedLockToRotZMode;

                g_jia_chassis_debug_watch.mode = static_cast<uint32_t>(itd_.mode);
                g_jia_chassis_debug_watch.debug_mode = debug_mode_;
                g_jia_chassis_debug_watch.is_debug = is_debug_ ? 1U : 0U;
                g_jia_chassis_debug_watch.is_world_speed_mode = is_world_mode ? 1U : 0U;
                g_jia_chassis_debug_watch.is_lock_now_rot_z = is_lock_now_mode ? 1U : 0U;
                g_jia_chassis_debug_watch.is_lock_to_rot_z = is_lock_to_mode ? 1U : 0U;
                g_jia_chassis_debug_watch.four_debug_wheel_index = debug_wheel_index_;
                g_jia_chassis_debug_watch.four_photogate_signal = photogate_signal_ ? 1U : 0U;
                g_jia_chassis_debug_watch.four_is_calibrating = is_doing_cailbration_ ? 1U : 0U;
                g_jia_chassis_debug_watch.four_calibration_angle_deg = cailbration_angle_deg_;
                g_jia_chassis_debug_watch.four_steer_target_deg = t_angle_deg;
                g_jia_chassis_debug_watch.four_steer_feedback_deg = c_angle_deg;
                g_jia_chassis_debug_watch.four_steer_target_rpm = wheel.smh->getTargetRPM();
                g_jia_chassis_debug_watch.four_steer_feedback_rpm = getSteerWheelCurrentRPM(wheel);
                g_jia_chassis_debug_watch.four_drive_target_rpm = getDriveWheelTargetRPM(wheel);
                g_jia_chassis_debug_watch.four_drive_feedback_rpm = getDriveWheelCurrentRPM(wheel);
                g_jia_chassis_debug_watch.four_drive_target_current = wheel.dmh->getTargetCurrent();
                g_jia_chassis_debug_watch.four_drive_feedback_current = getDriveWheelCurrentCurrent(wheel);
                g_jia_chassis_debug_watch.four_drive_force_brake_enabled = is_drive_force_brake_enabled_ ? 1U : 0U;
                g_jia_chassis_debug_watch.four_drive_zero_rpm_brake_enabled = is_drive_zero_rpm_brake_enabled_ ? 1U : 0U;
                g_jia_chassis_debug_watch.four_drive_zero_rpm_threshold_rpm = drive_zero_rpm_threshold_rpm_;

                printf_period_count_++;
                if (printf_period_count_ >= printf_period_ms_)
                {
                    printf_period_count_ = 0;

                    const f32 steer_target_current = getSteerWheelTargetCurrent(wheel);
                    const f32 steer_feedback_current = getSteerWheelCurrentCurrent(wheel);
                    const f32 steer_target_rpm = wheel.smh->getTargetRPM();
                    const f32 steer_feedback_rpm = getSteerWheelCurrentRPM(wheel);
                    const f32 steer_target_angle_deg = t_angle_deg;
                    const f32 steer_feedback_angle_deg = c_angle_deg;
                    const f32 steer_target_total_angle_deg = wheel.smh->getTargetTotalAngle();
                    const f32 steer_feedback_total_angle_deg = getSteerWheelCurrentTotalAngleDegCalibrated(wheel);

                    const f32 drive_target_current = wheel.dmh->getTargetCurrent();
                    const f32 drive_feedback_current = getDriveWheelCurrentCurrent(wheel);
                    const f32 drive_target_rpm = getDriveWheelTargetRPM(wheel);
                    const f32 drive_feedback_rpm = getDriveWheelCurrentRPM(wheel);
                    const f32 drive_target_angle_deg = wheel.dmh->getTargetAngle();
                    const f32 drive_feedback_angle_deg = wheel.dmh->getAngle();
                    const f32 drive_target_total_angle_deg = wheel.dmh->getTargetTotalAngle();
                    const f32 drive_feedback_total_angle_deg = wheel.dmh->getTotalAngle();

                    // VOFA通道顺序：光电、校准角、舵向电流/速度/单圈角/多圈角、轮向电流/速度/单圈角/多圈角。
                    // 每组内部顺序统一为：目标值在前，反馈值在后，便于直接对照阶跃/跟随情况。
                    f32 debug_frame[18] = {
                        (f32)photogate_signal_,
                        cailbration_angle_deg_,
                        steer_target_current,
                        steer_feedback_current,
                        steer_target_rpm,
                        steer_feedback_rpm,
                        steer_target_angle_deg,
                        steer_feedback_angle_deg,
                        steer_target_total_angle_deg,
                        steer_feedback_total_angle_deg,
                        drive_target_current,
                        drive_feedback_current,
                        drive_target_rpm,
                        drive_feedback_rpm,
                        drive_target_angle_deg,
                        drive_feedback_angle_deg,
                        drive_target_total_angle_deg,
                        drive_feedback_total_angle_deg};
                    debug_uart_.printf_DMA_JustFloat(debug_frame, 18);
                }
            }
        }
    }
}
