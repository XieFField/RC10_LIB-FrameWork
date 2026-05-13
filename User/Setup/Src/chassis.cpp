#include "chassis.h"

#include <cmath>

#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "BSP_TimeStamp.h"
#include "Module_CrsfReceiver.h"
#include "Module_HWT.h"
#include "APP_PID.h"

#include "APP_Utils.h"
#include "chassis.h"

namespace jia
{
    namespace ThreeOmniChassis
    {
        void Chassis::init(InitConfig &config)
        {
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
                input_hwt_rot_z_ = hwt->get_yaw_rad();
                input_hwt_omega_z_ = hwt->get_yaw_speed_rad();

                isDebugMode();

                setModeFlag();

                isTransSpeedWorldToBody(cmf_.is_world_speed_mode, it.vel_x, it.vel_y, t.vel_x, t.vel_y);

                if (cmf_.is_lock_now_rot_z)
                    isLockNowRotZ(cmf_.is_lock_now_rot_z, t.rot_z, it.omega_z, t.rot_z, t.omega_z);
                if (cmf_.is_lock_to_rot_z)
                    isLockToRotZ(cmf_.is_lock_to_rot_z, it.rot_z, t.rot_z, t.rot_z, it.omega_z, t.omega_z);

                // 逆运动学解算
                //  // 限制车端的目标速度
                clampTargetSpeedInChassis(t.vel_x, t.vel_y, t.omega_z, t.vel_x, t.vel_y, t.omega_z);
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
                isLimitAccInChassis(is_chassis_acc_limit_,
                                    t.vel_x, t.vel_y, target_pid_omega_z,
                                    p.vel_x, p.vel_y, p.omega_z,
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

                vTaskDelayUntil(&time_ms_, period_ms_);
            }
        }

        void Chassis::isTransSpeedBodyToWorld(bool is_trans, f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y)
        {
            if (is_trans)
            {
                transSpeedBodyToWorld(vel_x, vel_y, out_vel_x, out_vel_y);
            }
            else
            {
                out_vel_x = vel_x;
                out_vel_y = vel_y;
            }
        }

        void Chassis::isTransSpeedWorldToBody(bool is_trans, f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y)
        {
            if (is_trans)
            {
                transSpeedWorldToBody(vel_x, vel_y, out_vel_x, out_vel_y);
            }
            else
            {
                out_vel_x = vel_x;
                out_vel_y = vel_y;
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
            // TODO
            return 0.0f;
        }

        f32 Chassis::getTargetWorldVelY() const
        {
            // TODO
            return 0.0f;
        }

        f32 Chassis::getTargetOmegaZ() const
        {
            return planned_data_.omega_z;
        }

        f32 Chassis::getCurrentBodyVelX() const
        {
            // TODO
            return 0.0f;
        }

        f32 Chassis::getCurrentBodyVelY() const
        {
            // TODO
            return 0.0f;
        }

        f32 Chassis::getCurrentWorldVelX() const
        {
            // TODO
            return 0.0f;
        }

        f32 Chassis::getCurrentWorldVelY() const
        {
            // TODO
            return 0.0f;
        }

        f32 Chassis::getCurrentOmegaZ() const
        {
            // TODO
            return 0.0f;
        }

        void Chassis::inverseKinematics(f32 in_x, f32 in_y, f32 in_z, f32 &out_w1, f32 &out_w2, f32 &out_w3)
        {
            out_w1 = ((in_x * w1_.c + in_y * w1_.s + in_z * w1_.eqr) / wr_);
            out_w2 = ((in_x * w2_.c + in_y * w2_.s + in_z * w2_.eqr) / wr_);
            out_w3 = ((in_x * w3_.c + in_y * w3_.s + in_z * w3_.eqr) / wr_);
        }

        void Chassis::transSpeedBodyToWorld(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y)
        {
            f32 cos_theta = cosf(input_hwt_rot_z_);
            f32 sin_theta = sinf(input_hwt_rot_z_);

            out_vel_x = vel_x * cos_theta - vel_y * sin_theta;
            out_vel_y = vel_x * sin_theta + vel_y * cos_theta;
        }

        void Chassis::transSpeedWorldToBody(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y)
        {
            f32 cos_theta = cosf(input_hwt_rot_z_);
            f32 sin_theta = sinf(input_hwt_rot_z_);

            out_vel_x = vel_x * cos_theta + vel_y * sin_theta;
            out_vel_y = -vel_x * sin_theta + vel_y * cos_theta;
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

        void Chassis::clampTargetSpeedInChassis(f32 vel_x, f32 vel_y, f32 omega_z, f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z)
        {
            out_vel_x = clampValue(vel_x, -max_vel_x_, max_vel_x_);
            out_vel_y = clampValue(vel_y, -max_vel_y_, max_vel_y_);
            out_omega_z = clampValue(omega_z, -max_omega_z_, max_omega_z_);
        }

        void Chassis::isLimitAccInChassis(bool is_limit,
                                          f32 tar_vel_x, f32 tar_vel_y, f32 tar_omega_z,
                                          f32 cur_vel_x, f32 cur_vel_y, f32 cur_omega_z,
                                          f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z)
        {
            if (is_limit)
            {
                out_vel_x = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_vel_x, cur_vel_x, period_, max_acc_xy_acc_, max_acc_xy_dec_);
                out_vel_y = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_vel_y, cur_vel_y, period_, max_acc_xy_acc_, max_acc_xy_dec_);
                out_omega_z = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_omega_z, cur_omega_z, period_, max_alpha_z_acc_, max_alpha_z_dec_);
            }
            else
            {
                out_vel_x = tar_vel_x;
                out_vel_y = tar_vel_y;
                out_omega_z = tar_omega_z;
            }
        }

        void Chassis::clearInputTargetData()
        {
            input_target_data_.mode = Mode::kWheelTorqueFreeMode;
            input_target_data_.vel_x = 0.0f;
            input_target_data_.vel_y = 0.0f;
            input_target_data_.omega_z = 0.0f;
            input_target_data_.rot_z = 0.0f;
            lock_now_rot_z_target_ = 0.0f;
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
        void Chassis::init(InitConfig &config)
        {
            wheel_radius_m_ = (config.wheel_radius_m > 1.0e-6f) ? config.wheel_radius_m : 0.075f;
            max_vel_x_ = config.max_vel_x_m_s;
            max_vel_y_ = config.max_vel_y_m_s;
            max_omega_z_ = config.max_omega_z_rad_s;
            max_acc_xy_acc_ = config.max_acc_xy_acc_m_s2;
            max_acc_xy_dec_ = config.max_acc_xy_dec_m_s2;
            max_alpha_z_acc_ = config.max_alpha_z_acc_rad_s2;
            max_alpha_z_dec_ = config.max_alpha_z_dec_rad_s2;
            max_drive_omega_rad_s_ = config.max_drive_omega_rad_s;
            max_drive_alpha_rad_s2_ = config.max_drive_alpha_rad_s2;
            max_steer_rate_rad_s_ = config.max_steer_rate_rad_s;
            max_steer_alpha_rad_s2_ = config.max_steer_alpha_rad_s2;
            stationary_speed_epsilon_m_s_ = config.stationary_speed_epsilon_m_s;
            enable_cosine_compensation_ = config.enable_cosine_compensation;
            max_lock_to_rot_z_rad_s_ = config.max_lock_to_rot_z_rad_s;
            lock_now_rot_z_shift_time_ms_ = config.lock_now_rot_z_shift_time_ms;
            idle_posture_mode_ = config.idle_posture_mode;

            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                wheel.steer_motor_h = config.steer_motor_h[i];
                wheel.drive_motor_h = config.drive_motor_h[i];
                wheel.pos_x_m = config.wheels[i].pos_x_m;
                wheel.pos_y_m = config.wheels[i].pos_y_m;
                wheel.theta_oa_to_owi_rad = degToRadF32(config.wheels[i].theta_oa_to_owi_deg);
                wheel.steer_motor_sign = (config.wheels[i].steer_motor_sign == 0.0f) ? 1.0f : config.wheels[i].steer_motor_sign;
                wheel.drive_motor_sign = (config.wheels[i].drive_motor_sign == 0.0f) ? 1.0f : config.wheels[i].drive_motor_sign;
                wheel.homing_enabled = config.wheels[i].homing_enabled;
                wheel.homing_sensor_active_high = config.wheels[i].homing_sensor_active_high;
                wheel.homing_gpio_port = config.wheels[i].homing_gpio_port;
                wheel.homing_gpio_pin = config.wheels[i].homing_gpio_pin;
                wheel.homing_search_rpm = config.wheels[i].homing_search_rpm;
                wheel.homing_zero_offset_rad = degToRadF32(config.wheels[i].homing_zero_offset_deg);
                wheel.homing_timeout_s = config.wheels[i].homing_timeout_s;
                wheel.homing_state = wheel.homing_enabled ? HomingState::kIdle : HomingState::kReady;
                wheel.homing_zero_valid = !wheel.homing_enabled;
                wheel.homing_runtime_zero_offset_rad = wheel.homing_zero_offset_rad;
            }

            rot_z_pid_.set_params(lock_angle_pid_params, 0.0f);
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
            Chassis *chassis = static_cast<Chassis *>(arg);
            chassis->runThread(NULL);
        }

        void Chassis::clearInputTargetData()
        {
            input_target_data_.mode = Mode::kWheelTorqueFreeMode;
            input_target_data_.vel_x = 0.0f;
            input_target_data_.vel_y = 0.0f;
            input_target_data_.omega_z = 0.0f;
            input_target_data_.rot_z = 0.0f;
        }

        Chassis::Result Chassis::setWheelTorqueFreeMode()
        {
            clearInputTargetData();
            input_target_data_.mode = Mode::kWheelTorqueFreeMode;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetBodySpeedMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            input_target_data_.mode = Mode::kBodySpeedMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = omega_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetBodySpeedLockNowRotZMode(f32 vel_x, f32 vel_y)
        {
            input_target_data_.mode = Mode::kBodySpeedLockNowRotZMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = 0.0f;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetBodySpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            input_target_data_.mode = Mode::kBodySpeedLockNowRotZWithNoOmegaZMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = omega_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetBodySpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z)
        {
            input_target_data_.mode = Mode::kBodySpeedLockToRotZMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.rot_z = rot_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetWorldSpeedMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            input_target_data_.mode = Mode::kWorldSpeedMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = omega_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetWorldSpeedLockNowRotZMode(f32 vel_x, f32 vel_y)
        {
            input_target_data_.mode = Mode::kWorldSpeedLockNowRotZMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = 0.0f;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z)
        {
            input_target_data_.mode = Mode::kWorldSpeedLockNowRotZWithNoOmegaZMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.omega_z = omega_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::setTargetWorldSpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z)
        {
            input_target_data_.mode = Mode::kWorldSpeedLockToRotZMode;
            input_target_data_.vel_x = vel_x;
            input_target_data_.vel_y = vel_y;
            input_target_data_.rot_z = rot_z;
            return Result::kOk;
        }

        Chassis::Result Chassis::startHoming()
        {
            // 回零请求只负责“拉起状态机”和清空本轮回零参考，不直接驱动电机；
            // 真正的搜索、沿边沿捕获零位、偏置生效和完成判定都在 runThread 中按周期推进。
            homing_start_request_ = true;
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                wheel.homing_elapsed_s = 0.0f;
                wheel.homing_last_sensor_active = false;
                wheel.homing_runtime_zero_offset_rad = wheel.homing_zero_offset_rad;
                if (wheel.homing_enabled && wheel.homing_gpio_port != nullptr)
                {
                    wheel.homing_state = HomingState::kIdle;
                    wheel.homing_zero_valid = false;
                }
                else
                {
                    wheel.homing_state = HomingState::kReady;
                    wheel.homing_zero_valid = true;
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
            idle_posture_mode_ = mode;
        }

        f32 Chassis::wrapToPi(f32 angle_rad) const
        {
            while (angle_rad >= kPi)
            {
                angle_rad -= 2.0f * kPi;
            }
            while (angle_rad < -kPi)
            {
                angle_rad += 2.0f * kPi;
            }
            return angle_rad;
        }

        f32 Chassis::wrapTo2Pi(f32 angle_rad) const
        {
            while (angle_rad >= 2.0f * kPi)
            {
                angle_rad -= 2.0f * kPi;
            }
            while (angle_rad < 0.0f)
            {
                angle_rad += 2.0f * kPi;
            }
            return angle_rad;
        }

        f32 Chassis::shortestAngularDistance(f32 from_rad, f32 to_rad) const
        {
            return wrapToPi(to_rad - from_rad);
        }

        f32 Chassis::nearestEquivalentAngle(f32 current_rad, f32 target_mod_rad) const
        {
            return current_rad + shortestAngularDistance(current_rad, target_mod_rad);
        }

        f32 Chassis::magnitude2D(f32 x, f32 y) const
        {
            return sqrtf(x * x + y * y);
        }

        f32 Chassis::getXParkAngle(const WheelConfig &wheel) const
        {
            return atan2f(wheel.pos_y_m, wheel.pos_x_m);
        }

        void Chassis::setModeFlag()
        {
            // 将外部模式压缩成线程内使用的少量布尔标志，后续执行顺序只看这些标志，
            // 这样可以把“世界系/车体系”“定向锁角/跟随当前角”“空转模式”解耦开。
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
            default:
                break;
            }
        }

        void Chassis::transSpeedBodyToWorld(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y) const
        {
            f32 cos_theta = cosf(input_hwt_rot_z_);
            f32 sin_theta = sinf(input_hwt_rot_z_);
            out_vel_x = vel_x * cos_theta - vel_y * sin_theta;
            out_vel_y = vel_x * sin_theta + vel_y * cos_theta;
        }

        void Chassis::transSpeedWorldToBody(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y) const
        {
            f32 cos_theta = cosf(input_hwt_rot_z_);
            f32 sin_theta = sinf(input_hwt_rot_z_);
            out_vel_x = vel_x * cos_theta + vel_y * sin_theta;
            out_vel_y = -vel_x * sin_theta + vel_y * cos_theta;
        }

        // “锁当前航向”模式的核心语义是：
        // 只要用户还在主动给 omega_z，就继续按手动旋转执行；一旦用户松开旋转输入，
        // 就把最近一次真实机体朝向当作要维持的 rot_z，再由姿态 PID 生成 out_omega_z 来稳住该朝向。
        // 因此它不是“始终锁某个固定角”，而是“手动旋转”和“松手后自动锁住当前角”之间的平滑切换器。
        void Chassis::isLockNowRotZ(bool is_lock, f32 rot_z, f32 omega_z, f32 &out_rot_z, f32 &out_omega_z)
        {
            // 未启用锁当前航向时，rot_z / omega_z 不做任何二次整形，直接透传给后续统一规划层。
            if (!is_lock)
            {
                out_rot_z = rot_z;
                out_omega_z = omega_z;
                return;
            }

            // “锁当前航向”不是简单地把 rot_z 固定住，而是先在用户开始施加角速度时
            // 抓取当前机体朝向，再在后续由 PID 产生角速度闭环，让机器人保持当下姿态。
            if (omega_z == 0.0f)
            {
                // 这里表示“用户当前没有继续施加旋转输入”。
                // 但在刚松开摇杆的最初一小段时间内，不立即让 PID 介入，而是先进入过渡缓冲：
                // 1. out_rot_z 直接跟随 IMU 当前朝向 input_hwt_rot_z_，把目标角锁在此刻真实姿态上；
                // 2. out_omega_z 先给 0，避免手动旋转刚结束时立即出现一拍突兀的 PID 修正；
                // 3. lock_now_rot_z_shift_count_ 作为缓冲计数器，倒数结束后才真正进入锁角闭环。
                if (lock_now_rot_z_shift_count_ > 0)
                {
                    lock_now_rot_z_shift_count_--;
                    lock_now_rot_z_target_ = input_hwt_rot_z_;
                    out_rot_z = lock_now_rot_z_target_;
                    out_omega_z = 0.0f;
                }
                else
                {
                    // 过渡缓冲结束后，真正用于锁角的目标已经不是外部传入的 rot_z，
                    // 而是前面已经抓取并保存下来的 lock_now_rot_z_target_。
                    // 后续由 rot_z_pid_ 根据“目标朝向 lock_now_rot_z_target_”和“当前真实朝向 input_hwt_rot_z_”
                    // 的误差生成维持姿态所需的 out_omega_z。
                    out_rot_z = lock_now_rot_z_target_;
                    if (rot_z_pid_count_ >= rot_z_pid_period_)
                    {
                        rot_z_pid_count_ = 0;
                        out_omega_z = rot_z_pid_.pid_calc(radToDegF32(lock_now_rot_z_target_), radToDegF32(input_hwt_rot_z_));
                    }
                    else
                    {
                        // PID 不是每个控制周期都重算；在未到刷新周期时，
                        // 暂时沿用上一规划周期的 planned_data_.omega_z，减少输出抖动并维持角速度连续性。
                        out_omega_z = planned_data_.omega_z;
                    }
                    // rot_z_pid_count_ / rot_z_pid_period_ 共同控制姿态 PID 的实际计算节拍。
                    rot_z_pid_count_++;
                }
            }
            else
            {
                // 这里表示“用户仍在主动要求旋转”：
                // 1. 不进入锁角闭环，直接执行当前手动 omega_z；
                // 2. 同时把 out_rot_z 刷新成当前 IMU 朝向 input_hwt_rot_z_，
                //    相当于不断更新“等会儿松手后要锁住的那个角”；
                // 3. 每次有手动旋转输入都重置缓冲计数器，为后续从手动旋转切回自动锁角预留平滑过渡窗口。
                lock_now_rot_z_target_ = input_hwt_rot_z_;
                out_rot_z = lock_now_rot_z_target_;
                out_omega_z = omega_z;
                lock_now_rot_z_shift_count_ = lock_now_rot_z_shift_time_ms_;
            }
        }

        void Chassis::isLockToRotZ(bool is_lock, f32 tar_rot_z, f32 cur_rot_z, f32 &out_rot_z, f32 omega_z, f32 &out_omega_z)
        {
            if (!is_lock)
            {
                out_rot_z = tar_rot_z;
                out_omega_z = omega_z;
                return;
            }

            // “锁到指定航向”会先限制目标角速度变化率，再用姿态 PID 生成维持/逼近该目标角度所需的 omega_z。
            // 这样外层给出的目标角不会瞬间跳变，底盘转向更平滑。
            out_rot_z = limit1DPiAngleRateByTimeF32(tar_rot_z, cur_rot_z, period_, max_lock_to_rot_z_rad_s_);
            if (rot_z_pid_count_ >= rot_z_pid_period_)
            {
                rot_z_pid_count_ = 0;
                out_omega_z = rot_z_pid_.pid_calc(radToDegF32(out_rot_z), radToDegF32(input_hwt_rot_z_));
            }
            else
            {
                out_omega_z = planned_data_.omega_z;
            }
            rot_z_pid_count_++;
        }

        void Chassis::clampTargetSpeedInChassis(f32 vel_x, f32 vel_y, f32 omega_z, f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z) const
        {
            out_vel_x = clampValue(vel_x, -max_vel_x_, max_vel_x_);
            out_vel_y = clampValue(vel_y, -max_vel_y_, max_vel_y_);
            out_omega_z = clampValue(omega_z, -max_omega_z_, max_omega_z_);
        }

        void Chassis::limitPlannedSpeed(f32 tar_vel_x, f32 tar_vel_y, f32 tar_omega_z, f32 &out_vel_x, f32 &out_vel_y, f32 &out_omega_z)
        {
            // 规划层限速使用“对称加减速”约束：目标值不变，但每周期只允许按加/减速度上限逼近，
            // 这是比简单 clamp 更平滑的速度整形，避免规划速度台阶过大。
            out_vel_x = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_vel_x, last_planned_data_.vel_x, period_, max_acc_xy_acc_, max_acc_xy_dec_);
            out_vel_y = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_vel_y, last_planned_data_.vel_y, period_, max_acc_xy_acc_, max_acc_xy_dec_);
            out_omega_z = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tar_omega_z, last_planned_data_.omega_z, period_, max_alpha_z_acc_, max_alpha_z_dec_);
        }

        bool Chassis::readHomingSensor(const WheelConfig &wheel) const
        {
            if (!wheel.homing_enabled || wheel.homing_gpio_port == nullptr)
            {
                return false;
            }
            GPIO_TypeDef *port = reinterpret_cast<GPIO_TypeDef *>(wheel.homing_gpio_port);
            const bool raw_active = HAL_GPIO_ReadPin(port, wheel.homing_gpio_pin) != GPIO_PIN_RESET;
            return wheel.homing_sensor_active_high ? raw_active : !raw_active;
        }

        f32 Chassis::readSteerMotorRawTotalAngleRad(const WheelConfig &wheel) const
        {
            if (wheel.steer_motor_h == nullptr)
            {
                return 0.0f;
            }
            return wheel.steer_motor_sign * degToRadF32(wheel.steer_motor_h->getTotalAngle());
        }

        f32 Chassis::readDriveMotorOmegaRadS(const WheelConfig &wheel) const
        {
            if (wheel.drive_motor_h == nullptr)
            {
                return 0.0f;
            }
            return wheel.drive_motor_sign * rpmToRadsF32(wheel.drive_motor_h->getRPM());
        }

        f32 Chassis::readCorrectedSteerMotorTotalAngleRad(const WheelConfig &wheel) const
        {
            return readSteerMotorRawTotalAngleRad(wheel) + wheel.homing_runtime_zero_offset_rad;
        }

        void Chassis::updateWheelFeedback()
        {
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                wheel.corrected_steer_motor_total_angle_rad = readCorrectedSteerMotorTotalAngleRad(wheel);
                wheel.corrected_drive_omega_rad_s = readDriveMotorOmegaRadS(wheel);
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
                return true;
            }

            const bool sensor_active = readHomingSensor(wheel);
            const f32 raw_total_angle_rad = readSteerMotorRawTotalAngleRad(wheel);

            if (wheel.homing_state == HomingState::kIdle)
            {
                if (homing_start_request_)
                {
                    // 收到回零请求后才进入搜索态，避免初始化阶段或无请求时误触发回零动作。
                    wheel.homing_state = HomingState::kSearch;
                    wheel.homing_elapsed_s = 0.0f;
                }
                wheel.homing_last_sensor_active = sensor_active;
                return false;
            }

            if (wheel.homing_state == HomingState::kSearch)
            {
                // 搜索态只负责等待“传感器边沿”或“启动瞬间已处于有效态”的情况；
                // 一旦捕获到边沿，就把当前原始编码器角度换算成运行时零偏移。
                wheel.homing_elapsed_s += period_;
                const bool is_edge = (sensor_active != wheel.homing_last_sensor_active);
                const bool is_initial_active = sensor_active && (wheel.homing_elapsed_s <= period_ + 1.0e-6f);
                if (is_edge || is_initial_active)
                {
                    wheel.homing_state = HomingState::kEdgeDetected;
                    wheel.homing_runtime_zero_offset_rad = wheel.homing_zero_offset_rad - raw_total_angle_rad;
                    wheel.homing_zero_valid = true;
                }
                else if (wheel.homing_elapsed_s > wheel.homing_timeout_s)
                {
                    wheel.homing_state = HomingState::kFault;
                }
                wheel.homing_last_sensor_active = sensor_active;
                return false;
            }

            if (wheel.homing_state == HomingState::kEdgeDetected)
            {
                // 边沿已抓到后，先走一个过渡态，确保零偏已经写入后再进入连续角度就绪态。
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
                // 连续角度已经可用，后续底盘可以把该轮纳入正常闭环控制。
                wheel.homing_state = HomingState::kReady;
                return true;
            }

            return wheel.homing_state == HomingState::kReady;
        }

        void Chassis::setSteerMotorTargetCurrent(WheelConfig &wheel, f32 current)
        {
            if (wheel.steer_motor_h != nullptr)
            {
                wheel.steer_motor_h->setTargetCurrent(current);
            }
        }

        void Chassis::setSteerMotorTargetRPM(WheelConfig &wheel, f32 rpm)
        {
            if (wheel.steer_motor_h != nullptr)
            {
                wheel.steer_motor_h->setTargetRPM(rpm / wheel.steer_motor_sign);
            }
        }

        void Chassis::setSteerMotorTargetTotalAngleRad(WheelConfig &wheel, f32 corrected_local_total_angle_rad)
        {
            if (wheel.steer_motor_h == nullptr)
            {
                return;
            }
            f32 raw_motor_total_rad = (corrected_local_total_angle_rad - wheel.homing_runtime_zero_offset_rad) / wheel.steer_motor_sign;
            wheel.steer_motor_h->setTargetTotalAngle(radToDegF32(raw_motor_total_rad));
        }

        void Chassis::setDriveMotorTargetOmegaRadS(WheelConfig &wheel, f32 drive_omega_rad_s)
        {
            if (wheel.drive_motor_h != nullptr)
            {
                wheel.drive_motor_h->setTargetRPM(radsToRpmF32(drive_omega_rad_s / wheel.drive_motor_sign));
            }
        }

        // 这是一个“位置目标 + 速度上限 + 加速度上限”的二阶限幅器。
        // 输入是当前位置 current_value、当前速度 current_rate 和目标位置 target_value，
        // 输出是“下一拍允许走到的位置”，并通过 next_rate 回传这一拍实际采用的速度。
        // 在四舵轮里它主要用于转向角规划：既不允许舵角变化过快，也不允许舵角速度突变过猛。
        f32 Chassis::limitPositionSecondOrder(f32 current_value, f32 current_rate, f32 target_value, f32 max_rate, f32 max_accel, f32 dt_s, f32 &next_rate) const
        {
            // 防止 dt 过小导致除零或数值放大；在异常小周期下退回一个保守的 1ms 步长。
            const f32 safe_dt = (dt_s <= 1.0e-6f) ? 1.0e-3f : dt_s;

            // delta_value 是这一拍距离目标位置还差多少；
            // desired_rate 是“如果想在一拍内尽量逼近目标，希望使用的速度”，
            // 但它先受 max_rate 限制，避免直接给出不可能达到的目标速度。
            const f32 delta_value = target_value - current_value;
            const f32 desired_rate = clampValue(delta_value / safe_dt, -max_rate, max_rate);

            // rate_delta_limit 是“这一拍速度最多允许变化多少”，由最大加速度决定。
            const f32 rate_delta_limit = max_accel * safe_dt;

            next_rate = current_rate;

            // 先做速度变化率限制：如果期望速度离当前速度太远，
            // 这一拍只允许按 max_accel 推进一步，而不是瞬间跳到 desired_rate。
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

            // 再做一次绝对速度限幅，保证最终速度不超过 max_rate。
            next_rate = clampValue(next_rate, -max_rate, max_rate);

            // 按这一拍最终允许的速度积分出位置步进量。
            f32 step_value = next_rate * safe_dt;

            // 如果这一拍已经足够到达目标，则直接截断到目标位置，
            // 避免积分后跨过 target_value 造成过冲。
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
            // 这一段是四舵轮模块命令生成的核心：
            // 先把底盘速度分解到每个轮模块的安装坐标系，再决定舵角是“正向到位”还是“反向转 180° 后驱动反转”，
            // 最后分别对舵角和驱动速度做速度/加速度约束，避免命令突变。
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                const f32 current_local_total_rad = wheel.corrected_steer_motor_total_angle_rad;
                const f32 current_oa_total_rad = current_local_total_rad + wheel.theta_oa_to_owi_rad;

                const f32 wheel_velocity_oa_x = command_data.vel_x - command_data.omega_z * wheel.pos_y_m;
                const f32 wheel_velocity_oa_y = command_data.vel_y + command_data.omega_z * wheel.pos_x_m;
                const f32 wheel_speed_m_s = magnitude2D(wheel_velocity_oa_x, wheel_velocity_oa_y);

                f32 raw_target_oa_mod_rad = 0.0f;
                f32 target_drive_omega_rad_s = 0.0f;

                if (wheel_speed_m_s <= stationary_speed_epsilon_m_s_)
                {
                    // 近似静止时不强迫舵轮寻找某个“数学最优朝向”，而是优先保持当前位置或进入 X-Park 姿态，
                    // 这样可以减少原地抖动和不必要的舵角来回搜索。
                    raw_target_oa_mod_rad = (idle_posture_mode_ == IdlePostureMode::kXPark)
                                                ? wrapTo2Pi(getXParkAngle(wheel))
                                                : wrapTo2Pi(current_oa_total_rad);
                    target_drive_omega_rad_s = 0.0f;
                }
                else
                {
                    // 有平面速度时，轮子速度方向由 atan2 决定，驱动轮线速度由合速度大小换算而来。
                    raw_target_oa_mod_rad = wrapTo2Pi(atan2f(wheel_velocity_oa_y, wheel_velocity_oa_x));
                    target_drive_omega_rad_s = wheel_speed_m_s / wheel_radius_m_;
                }

                // 舵轮存在“朝向等价类”：目标角加 π 后，只要驱动轮反向即可得到同样的底盘效果。
                // 因此这里比较两种等价姿态谁更接近当前角度，优先选转角更小的一侧。
                const f32 alt_target_oa_mod_rad = wrapTo2Pi(raw_target_oa_mod_rad + kPi);
                const f32 candidate_a_oa_total_rad = nearestEquivalentAngle(current_oa_total_rad, raw_target_oa_mod_rad);
                const f32 candidate_b_oa_total_rad = nearestEquivalentAngle(current_oa_total_rad, alt_target_oa_mod_rad);

                f32 selected_oa_total_rad = candidate_a_oa_total_rad;
                bool flipped_drive = false;
                if (fabsf(candidate_b_oa_total_rad - current_oa_total_rad) < fabsf(candidate_a_oa_total_rad - current_oa_total_rad))
                {
                    selected_oa_total_rad = candidate_b_oa_total_rad;
                    target_drive_omega_rad_s = -target_drive_omega_rad_s;
                    flipped_drive = true;
                }

                f32 cosine_scale = 1.0f;
                if (enable_cosine_compensation_)
                {
                    // 舵角偏离目标越多，驱动轮对底盘速度的有效贡献越小；
                    // 余弦补偿把这种几何损失显式折算进驱动轮目标速度中。
                    cosine_scale = cosf(fabsf(shortestAngularDistance(current_oa_total_rad, selected_oa_total_rad)));
                    if (cosine_scale < 0.0f)
                    {
                        cosine_scale = 0.0f;
                    }
                }
                target_drive_omega_rad_s *= cosine_scale;
                target_drive_omega_rad_s = clampValue(target_drive_omega_rad_s, -max_drive_omega_rad_s_, max_drive_omega_rad_s_);

                f32 next_steer_rate_rad_s = 0.0f;
                const f32 selected_local_total_rad = selected_oa_total_rad - wheel.theta_oa_to_owi_rad;
                wheel.target_steer_motor_total_angle_rad = limitPositionSecondOrder(
                    current_local_total_rad,
                    last_steer_rate_cmd_rad_s_[i],
                    selected_local_total_rad,
                    max_steer_rate_rad_s_,
                    max_steer_alpha_rad_s2_,
                    period_,
                    next_steer_rate_rad_s);
                wheel.target_drive_omega_rad_s = clampValue(
                    limitValueWithAcceleration(last_drive_omega_cmd_rad_s_[i], target_drive_omega_rad_s, max_drive_alpha_rad_s2_, period_),
                    -max_drive_omega_rad_s_,
                    max_drive_omega_rad_s_);
                wheel.steer_target_velocity_rad_s = next_steer_rate_rad_s;
                wheel.flipped_drive_direction = flipped_drive;

                last_steer_rate_cmd_rad_s_[i] = next_steer_rate_rad_s;
                last_drive_omega_cmd_rad_s_[i] = wheel.target_drive_omega_rad_s;

                planned_data_.steer_angle_oa_rad[i] = wheel.target_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad;
                planned_data_.drive_omega_rad_s[i] = wheel.target_drive_omega_rad_s;
            }
        }

        void Chassis::applyModuleCommands(bool all_homed)
        {
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];

                if (!all_homed)
                {
                    setDriveMotorTargetOmegaRadS(wheel, 0.0f);
                    if (wheel.homing_state == HomingState::kSearch)
                    {
                        setSteerMotorTargetRPM(wheel, wheel.homing_search_rpm);
                    }
                    else
                    {
                        setSteerMotorTargetCurrent(wheel, 0.0f);
                    }
                    continue;
                }

                if (current_mode_flag_.is_wheel_torque_free)
                {
                    setSteerMotorTargetCurrent(wheel, 0.0f);
                    if (wheel.drive_motor_h != nullptr)
                    {
                        wheel.drive_motor_h->setTargetCurrent(0.0f);
                    }
                    continue;
                }

                setSteerMotorTargetTotalAngleRad(wheel, wheel.target_steer_motor_total_angle_rad);
                setDriveMotorTargetOmegaRadS(wheel, wheel.target_drive_omega_rad_s);
            }
        }

        void Chassis::updateCurrentData(bool all_homed)
        {
            current_data_ = planned_data_;
            if (!all_homed)
            {
                current_data_.vel_x = 0.0f;
                current_data_.vel_y = 0.0f;
                current_data_.omega_z = 0.0f;
            }

            for (u8 i = 0; i < 4; ++i)
            {
                current_data_.steer_angle_oa_rad[i] = wheel_config_[i].corrected_steer_motor_total_angle_rad + wheel_config_[i].theta_oa_to_owi_rad;
                current_data_.drive_omega_rad_s[i] = wheel_config_[i].corrected_drive_omega_rad_s;
            }

            if (all_homed)
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

        bool Chassis::solveLinear3x3(f32 matrix[3][4], f32 &x0, f32 &x1, f32 &x2) const
        {
            // 这里用的是带主元选取的高斯消元，目标是稳定求解 3x3 线性方程组；
            // 输入是增广矩阵，输出是三项未知量，失败通常意味着矩阵接近奇异。
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
            // 由四个模块的“当前舵角 + 当前驱动速度”反推底盘速度。
            // 这里不是直接解单个方程，而是把每个轮子的两个投影约束累积成最小二乘正规方程，
            // 再求解 3 个底盘自由度 [vx, vy, omega_z]。
            f32 normal[3][3] = {};
            f32 rhs[3] = {};

            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                const f32 steer_angle_oa_rad = wheel.corrected_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad;
                const f32 cos_theta = cosf(steer_angle_oa_rad);
                const f32 sin_theta = sinf(steer_angle_oa_rad);
                const f32 drive_linear_m_s = wheel.corrected_drive_omega_rad_s * wheel_radius_m_;

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
                // 主线程每个周期的执行顺序是固定的：
                // 1) 读取 IMU 航向/角速度
                // 2) 解析模式并做坐标系转换
                // 3) 处理锁航向逻辑与速度限幅
                // 4) 更新轮反馈与回零状态机
                // 5) 生成模块命令、下发电机目标
                // 6) 回写当前估计值并等待下一周期
                input_hwt_rot_z_ = hwt->get_yaw_rad();
                input_hwt_omega_z_ = hwt->get_yaw_speed_rad();

                setModeFlag();
                target_data_.rot_z = input_target_data_.rot_z;

                if (current_mode_flag_.is_world_speed_mode)
                {
                    transSpeedWorldToBody(input_target_data_.vel_x, input_target_data_.vel_y, target_data_.vel_x, target_data_.vel_y);
                }
                else
                {
                    target_data_.vel_x = input_target_data_.vel_x;
                    target_data_.vel_y = input_target_data_.vel_y;
                }
                target_data_.omega_z = input_target_data_.omega_z;

                // 锁当前航向 / 锁到指定航向都在这里对目标 rot_z 和 omega_z 做二次整形，
                // 之后再统一进入速度限幅和规划层限速。
                if (current_mode_flag_.is_lock_now_rot_z)
                {
                    isLockNowRotZ(true, target_data_.rot_z, target_data_.omega_z, target_data_.rot_z, target_data_.omega_z);
                }
                if (current_mode_flag_.is_lock_to_rot_z)
                {
                    isLockToRotZ(true, input_target_data_.rot_z, target_data_.rot_z, target_data_.rot_z, target_data_.omega_z, target_data_.omega_z);
                }

                clampTargetSpeedInChassis(target_data_.vel_x, target_data_.vel_y, target_data_.omega_z,
                                          target_data_.vel_x, target_data_.vel_y, target_data_.omega_z);

                limitPlannedSpeed(target_data_.vel_x, target_data_.vel_y, target_data_.omega_z,
                                  planned_data_.vel_x, planned_data_.vel_y, planned_data_.omega_z);

                planned_data_.acc_x = (planned_data_.vel_x - last_planned_data_.vel_x) / period_;
                planned_data_.acc_y = (planned_data_.vel_y - last_planned_data_.vel_y) / period_;
                planned_data_.alpha_z = (planned_data_.omega_z - last_planned_data_.omega_z) / period_;
                planned_data_.rot_z = target_data_.rot_z;

                updateWheelFeedback();

                bool all_homed = true;
                for (u8 i = 0; i < 4; ++i)
                {
                    if (!updateHomingState(wheel_config_[i]))
                    {
                        all_homed = false;
                    }
                }
                homing_start_request_ = false;

                // 回零和正常控制共用同一套命令生成流程，但最终下发前会根据 all_homed 选择：
                // 未回零时只保留安全动作，已回零时才输出完整舵角/驱动目标。
                computeModuleCommands(planned_data_);
                applyModuleCommands(all_homed);
                updateCurrentData(all_homed);

                last_planned_data_ = planned_data_;
                vTaskDelayUntil(&time_ms_, period_ms_);
            }
        }

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
