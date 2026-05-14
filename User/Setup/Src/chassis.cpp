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
            default_strategy_cfg_.steering_strategy_mode = config.steering_strategy_mode;
            default_strategy_cfg_.flip_enter_angle_deg = config.flip_enter_angle_deg;
            default_strategy_cfg_.flip_exit_angle_deg = config.flip_exit_angle_deg;
            default_strategy_cfg_.enable_drive_gate = config.enable_drive_gate;
            default_strategy_cfg_.drive_gate_strategy = config.drive_gate_strategy;
            default_strategy_cfg_.drive_gate_scope = config.drive_gate_scope;
            default_strategy_cfg_.drive_gate_close_angle_deg = config.drive_gate_close_angle_deg;
            default_strategy_cfg_.drive_gate_min_scale = config.drive_gate_min_scale;
            default_strategy_cfg_.drive_gate_curve_exponent = config.drive_gate_curve_exponent;
            default_strategy_cfg_.drive_gate_curve_half_angle_deg = config.drive_gate_curve_half_angle_deg;
            default_strategy_cfg_.drive_gate_curve_min_scale = config.drive_gate_curve_min_scale;
            default_strategy_cfg_.drive_gate_transition_linear_speed_m_s = config.drive_gate_transition_linear_speed_m_s;
            default_strategy_cfg_.drive_gate_transition_angular_speed_rad_s = config.drive_gate_transition_angular_speed_rad_s;
            default_strategy_cfg_.drive_gate_scale_ramp_up_s = config.drive_gate_scale_ramp_up_s;
            default_strategy_cfg_.drive_gate_scale_ramp_down_s = config.drive_gate_scale_ramp_down_s;
            default_strategy_cfg_.enable_stop_steer_guard = config.enable_stop_steer_guard;
            default_strategy_cfg_.stop_steer_guard_strategy = config.stop_steer_guard_strategy;
            default_strategy_cfg_.stop_guard_release_speed_m_s = config.stop_guard_release_speed_m_s;
            default_strategy_cfg_.stop_guard_blend_start_speed_m_s = config.stop_guard_blend_start_speed_m_s;
            default_strategy_cfg_.stop_guard_curve_half_speed_m_s = config.stop_guard_curve_half_speed_m_s;
            default_strategy_cfg_.stop_guard_curve_exponent = config.stop_guard_curve_exponent;
            runtime_strategy_cfg_ = default_strategy_cfg_;

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
                wheel.homing_falling_edge_mech_rad = degToRadF32(config.wheels[i].homing_falling_edge_mech_deg);
                wheel.homing_rising_edge_mech_rad = degToRadF32(config.wheels[i].homing_rising_edge_mech_deg);
                wheel.homing_search_rpm = config.wheels[i].homing_search_rpm;
                wheel.homing_zero_offset_rad = degToRadF32(config.wheels[i].homing_zero_offset_deg);
                wheel.homing_timeout_s = config.wheels[i].homing_timeout_s;
                wheel.homing_state = wheel.homing_enabled ? HomingState::kIdle : HomingState::kReady;
                wheel.homing_last_edge_is_falling = false;
                wheel.homing_zero_valid = !wheel.homing_enabled;
                wheel.homing_runtime_zero_offset_rad = wheel.homing_zero_offset_rad;
                selected_flipped_solution_[i] = false;
                drive_gate_scale_[i] = 1.0f;
            }

            adaptive_gate_scale_ = 1.0f;
            adaptive_gate_phase_ = AdaptiveGatePhase::kIdle;

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
            lock_now_rot_z_target_ = 0.0f;
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

        void Chassis::setSteeringStrategyMode(SteeringStrategyMode mode)
        {
            runtime_strategy_cfg_.steering_strategy_mode = mode;
        }

        void Chassis::setSteeringFlipHysteresisDeg(f32 enter_angle_deg, f32 exit_angle_deg)
        {
            runtime_strategy_cfg_.flip_enter_angle_deg = (enter_angle_deg > 0.0f) ? enter_angle_deg : 100.0f;
            runtime_strategy_cfg_.flip_exit_angle_deg = (exit_angle_deg > 0.0f) ? exit_angle_deg : 80.0f;
        }

        void Chassis::setDriveGateEnabled(bool enabled)
        {
            runtime_strategy_cfg_.enable_drive_gate = enabled;
            if (!enabled)
            {
                adaptive_gate_scale_ = 1.0f;
                adaptive_gate_phase_ = AdaptiveGatePhase::kDisabled;
            }
        }

        void Chassis::setDriveGateConfig(DriveGateStrategy strategy, DriveGateScope scope, f32 close_angle_deg, f32 min_scale)
        {
            runtime_strategy_cfg_.drive_gate_strategy = strategy;
            runtime_strategy_cfg_.drive_gate_scope = scope;
            runtime_strategy_cfg_.drive_gate_close_angle_deg = (close_angle_deg > 0.0f) ? close_angle_deg : 30.0f;
            runtime_strategy_cfg_.drive_gate_min_scale = clampValue(min_scale, 0.0f, 1.0f);
        }

        void Chassis::setDriveGateCurveParams(f32 curve_half_angle_deg, f32 curve_exponent, f32 curve_min_scale)
        {
            runtime_strategy_cfg_.drive_gate_curve_half_angle_deg = (curve_half_angle_deg > 0.1f) ? curve_half_angle_deg : 20.0f;
            runtime_strategy_cfg_.drive_gate_curve_exponent = (curve_exponent > 0.1f) ? curve_exponent : 2.0f;
            runtime_strategy_cfg_.drive_gate_curve_min_scale = clampValue(curve_min_scale, 0.0f, 1.0f);
        }

        void Chassis::setDriveGateAdaptiveParams(f32 transition_linear_speed_m_s, f32 transition_angular_speed_rad_s, f32 ramp_up_s, f32 ramp_down_s)
        {
            runtime_strategy_cfg_.drive_gate_transition_linear_speed_m_s = (transition_linear_speed_m_s >= 0.0f) ? transition_linear_speed_m_s : 0.30f;
            runtime_strategy_cfg_.drive_gate_transition_angular_speed_rad_s = (transition_angular_speed_rad_s >= 0.0f) ? transition_angular_speed_rad_s : 1.00f;
            runtime_strategy_cfg_.drive_gate_scale_ramp_up_s = (ramp_up_s > 1.0e-4f) ? ramp_up_s : 0.10f;
            runtime_strategy_cfg_.drive_gate_scale_ramp_down_s = (ramp_down_s > 1.0e-4f) ? ramp_down_s : 0.06f;
        }

        void Chassis::setStopSteerGuardEnabled(bool enabled)
        {
            runtime_strategy_cfg_.enable_stop_steer_guard = enabled;
        }

        void Chassis::setStopSteerGuardConfig(StopSteerGuardStrategy strategy, f32 release_speed_m_s, f32 blend_start_speed_m_s, f32 curve_half_speed_m_s, f32 curve_exponent)
        {
            runtime_strategy_cfg_.stop_steer_guard_strategy = strategy;
            runtime_strategy_cfg_.stop_guard_release_speed_m_s = (release_speed_m_s >= 0.0f) ? release_speed_m_s : 0.01f;
            runtime_strategy_cfg_.stop_guard_blend_start_speed_m_s = (blend_start_speed_m_s >= 0.0f) ? blend_start_speed_m_s : 0.20f;
            runtime_strategy_cfg_.stop_guard_curve_half_speed_m_s = (curve_half_speed_m_s > 1.0e-4f) ? curve_half_speed_m_s : 0.08f;
            runtime_strategy_cfg_.stop_guard_curve_exponent = (curve_exponent > 0.1f) ? curve_exponent : 2.0f;
        }

        void Chassis::resetRuntimeStrategyToInitConfig()
        {
            runtime_strategy_cfg_ = default_strategy_cfg_;
            adaptive_gate_scale_ = 1.0f;
            adaptive_gate_phase_ = AdaptiveGatePhase::kIdle;
        }

        f32 Chassis::wrapToPi(f32 angle_rad) const
        {
            return wrapToPiRuntimeF32(angle_rad);
        }

        f32 Chassis::wrapTo2Pi(f32 angle_rad) const
        {
            return wrapTo2PiRuntimeF32(angle_rad);
        }

        f32 Chassis::shortestAngularDistance(f32 from_rad, f32 to_rad) const
        {
            return shortestAngularDistanceRuntimeF32(from_rad, to_rad);
        }

        f32 Chassis::nearestEquivalentAngle(f32 current_rad, f32 target_mod_rad) const
        {
            return nearestEquivalentAngleRuntimeF32(current_rad, target_mod_rad);
        }

        f32 Chassis::magnitude2D(f32 x, f32 y) const
        {
            return magnitude2DRuntimeF32(x, y);
        }

        f32 Chassis::getXParkAngle(const WheelConfig &wheel) const
        {
            return atan2f(wheel.pos_y_m, wheel.pos_x_m);
        }

        f32 Chassis::computeDriveGateScale(f32 abs_error_rad) const
        {
            const f32 close_rad = degToRadF32(runtime_strategy_cfg_.drive_gate_close_angle_deg);
            const f32 min_scale = clampValue(runtime_strategy_cfg_.drive_gate_min_scale, 0.0f, 1.0f);

            switch (runtime_strategy_cfg_.drive_gate_strategy)
            {
            case DriveGateStrategy::kHardGate:
                return (abs_error_rad >= close_rad) ? min_scale : 1.0f;
            case DriveGateStrategy::kSoftGate:
            {
                if (close_rad <= 1.0e-6f)
                {
                    return min_scale;
                }
                const f32 ratio = clampValue(abs_error_rad / close_rad, 0.0f, 1.0f);
                return 1.0f - (1.0f - min_scale) * ratio;
            }
            case DriveGateStrategy::kContinuousCurve:
            {
                const f32 half_rad = degToRadF32(runtime_strategy_cfg_.drive_gate_curve_half_angle_deg);
                const f32 exponent = (runtime_strategy_cfg_.drive_gate_curve_exponent > 0.1f) ? runtime_strategy_cfg_.drive_gate_curve_exponent : 2.0f;
                const f32 curve_min = clampValue(runtime_strategy_cfg_.drive_gate_curve_min_scale, 0.0f, 1.0f);
                if (half_rad <= 1.0e-6f)
                {
                    return curve_min;
                }
                const f32 norm = abs_error_rad / half_rad;
                const f32 scale = 1.0f / (1.0f + powf(norm, exponent));
                return clampValue(curve_min + (1.0f - curve_min) * scale, curve_min, 1.0f);
            }
            case DriveGateStrategy::kAdaptiveGate:
            default:
                return (abs_error_rad >= close_rad) ? min_scale : 1.0f;
            }
        }

        void Chassis::computeDriveGateScales(const f32 steering_errors_rad[4], const Data &command_data, f32 out_scales[4])
        {
            for (u8 i = 0; i < 4; ++i)
            {
                out_scales[i] = 1.0f;
            }

            if (!runtime_strategy_cfg_.enable_drive_gate)
            {
                adaptive_gate_scale_ = 1.0f;
                adaptive_gate_phase_ = AdaptiveGatePhase::kDisabled;
                return;
            }

            if (runtime_strategy_cfg_.drive_gate_strategy == DriveGateStrategy::kAdaptiveGate)
            {
                const f32 linear_speed = magnitude2D(command_data.vel_x, command_data.vel_y);
                const f32 angular_speed = fabsf(command_data.omega_z);
                const bool in_transition = (linear_speed >= runtime_strategy_cfg_.drive_gate_transition_linear_speed_m_s) ||
                                           (angular_speed >= runtime_strategy_cfg_.drive_gate_transition_angular_speed_rad_s);
                const f32 ramp_up = (runtime_strategy_cfg_.drive_gate_scale_ramp_up_s > 1.0e-6f) ? runtime_strategy_cfg_.drive_gate_scale_ramp_up_s : 0.10f;
                const f32 ramp_down = (runtime_strategy_cfg_.drive_gate_scale_ramp_down_s > 1.0e-6f) ? runtime_strategy_cfg_.drive_gate_scale_ramp_down_s : 0.06f;
                const f32 delta = period_ / (in_transition ? ramp_up : ramp_down);
                if (in_transition)
                {
                    adaptive_gate_scale_ = clampValue(adaptive_gate_scale_ + delta, 0.0f, 1.0f);
                    adaptive_gate_phase_ = AdaptiveGatePhase::kTransition;
                }
                else
                {
                    adaptive_gate_scale_ = clampValue(adaptive_gate_scale_ - delta, 0.0f, 1.0f);
                    adaptive_gate_phase_ = AdaptiveGatePhase::kStartHold;
                }
            }
            else
            {
                adaptive_gate_scale_ = 1.0f;
                adaptive_gate_phase_ = AdaptiveGatePhase::kLegacy;
            }

            if (runtime_strategy_cfg_.drive_gate_scope == DriveGateScope::kGlobal)
            {
                f32 max_abs = 0.0f;
                for (u8 i = 0; i < 4; ++i)
                {
                    if (steering_errors_rad[i] > max_abs)
                    {
                        max_abs = steering_errors_rad[i];
                    }
                }
                f32 scale = computeDriveGateScale(max_abs);
                if (runtime_strategy_cfg_.drive_gate_strategy == DriveGateStrategy::kAdaptiveGate)
                {
                    scale *= adaptive_gate_scale_;
                }
                scale = clampValue(scale, 0.0f, 1.0f);
                for (u8 i = 0; i < 4; ++i)
                {
                    out_scales[i] = scale;
                }
                return;
            }

            for (u8 i = 0; i < 4; ++i)
            {
                f32 scale = computeDriveGateScale(steering_errors_rad[i]);
                if (runtime_strategy_cfg_.drive_gate_strategy == DriveGateStrategy::kAdaptiveGate)
                {
                    scale *= adaptive_gate_scale_;
                }
                out_scales[i] = clampValue(scale, 0.0f, 1.0f);
            }
        }

        f32 Chassis::stopSteerGuardBlend(f32 residual_speed_m_s) const
        {
            const f32 release_speed = (runtime_strategy_cfg_.stop_guard_release_speed_m_s >= 0.0f) ? runtime_strategy_cfg_.stop_guard_release_speed_m_s : 0.01f;
            if (residual_speed_m_s <= release_speed)
            {
                return 1.0f;
            }

            switch (runtime_strategy_cfg_.stop_steer_guard_strategy)
            {
            case StopSteerGuardStrategy::kHardHold:
                return 0.0f;
            case StopSteerGuardStrategy::kSoftBlend:
            {
                const f32 start_speed = (runtime_strategy_cfg_.stop_guard_blend_start_speed_m_s > release_speed)
                                            ? runtime_strategy_cfg_.stop_guard_blend_start_speed_m_s
                                            : (release_speed + 1.0e-3f);
                const f32 norm = clampValue((residual_speed_m_s - release_speed) / (start_speed - release_speed), 0.0f, 1.0f);
                return 1.0f - norm;
            }
            case StopSteerGuardStrategy::kContinuousBlend:
            default:
            {
                const f32 half_speed = (runtime_strategy_cfg_.stop_guard_curve_half_speed_m_s > 1.0e-6f)
                                           ? runtime_strategy_cfg_.stop_guard_curve_half_speed_m_s
                                           : 0.08f;
                const f32 exponent = (runtime_strategy_cfg_.stop_guard_curve_exponent > 0.1f)
                                         ? runtime_strategy_cfg_.stop_guard_curve_exponent
                                         : 2.0f;
                const f32 norm = residual_speed_m_s / half_speed;
                return 1.0f / (1.0f + powf(norm, exponent));
            }
            }
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

        void Chassis::isDebugMode()
        {
            if (!is_debug_)
            {
                return;
            }

            f32 target_vel_x = airjoy_data_.left_x * max_vel_x_;
            f32 target_vel_y = airjoy_data_.left_y * max_vel_y_;
            f32 target_omega_z = airjoy_data_.right_x * max_omega_z_;

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
                setWheelTorqueFreeMode();
                break;
            case 1:
                setTargetBodySpeedMode(target_vel_x, target_vel_y, target_omega_z);
                break;
            case 2:
                setTargetWorldSpeedMode(target_vel_x, target_vel_y, target_omega_z);
                break;
            case 3:
                setTargetBodySpeedLockNowRotZMode(target_vel_x, target_vel_y);
                break;
            case 4:
                setTargetWorldSpeedLockNowRotZMode(target_vel_x, target_vel_y);
                break;
            case 5:
                setTargetBodySpeedLockToRotZMode(target_vel_x, target_vel_y, debug_lock_rot_z_);
                break;
            case 6:
                setTargetWorldSpeedLockToRotZMode(target_vel_x, target_vel_y, debug_lock_rot_z_);
                break;
            case 7:
                setTargetBodySpeedLockNowRotZWithNoOmegaZMode(target_vel_x, target_vel_y, target_omega_z);
                break;
            case 8:
                setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(target_vel_x, target_vel_y, target_omega_z);
                break;
            case 20:
            case 21:
            case 22:
            case 30:
                // 20/21/22 都在 runThread() 内走专门调试分支：
                // 不复用“扭矩自由模式”，避免 applyModuleCommands() 再把目标清零。
                setTargetBodySpeedMode(0.0f, 0.0f, 0.0f);
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
            const bool raw_active = readHomingSensorRawHigh(wheel);
            return wheel.homing_sensor_active_high ? raw_active : !raw_active;
        }

        bool Chassis::readHomingSensorRawHigh(const WheelConfig &wheel) const
        {
            if (!wheel.homing_enabled || wheel.homing_gpio_port == nullptr)
            {
                return false;
            }
            GPIO_TypeDef *port = reinterpret_cast<GPIO_TypeDef *>(wheel.homing_gpio_port);
            return HAL_GPIO_ReadPin(port, wheel.homing_gpio_pin) != GPIO_PIN_RESET;
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
                wheel.homing_last_edge_is_falling = false;
                return true;
            }

            const bool sensor_raw_high = readHomingSensorRawHigh(wheel);
            const f32 raw_total_angle_rad = readSteerMotorRawTotalAngleRad(wheel);

            if (wheel.homing_state == HomingState::kIdle)
            {
                if (homing_start_request_)
                {
                    // 收到回零请求后才进入搜索态，避免初始化阶段或无请求时误触发回零动作。
                    wheel.homing_state = HomingState::kSearch;
                    wheel.homing_elapsed_s = 0.0f;
                }
                wheel.homing_last_sensor_active = sensor_raw_high;
                return false;
            }

            if (wheel.homing_state == HomingState::kSearch)
            {
                // 搜索态严格等待“传感器边沿”，不再使用“初始有效电平直接通过”的捷径。
                // 双边沿语义（按你给的实车标定）：
                //   H->L: 触发角是机械 +60°
                //   L->H: 触发角是机械 -120°
                // 两个触发角相差 180°，保证任意起始状态半圈内都能抓到一个有效边沿。
                wheel.homing_elapsed_s += period_;
                const bool is_edge = (sensor_raw_high != wheel.homing_last_sensor_active);
                if (is_edge)
                {
                    const bool is_falling_edge = wheel.homing_last_sensor_active && !sensor_raw_high;
                    const f32 edge_mech_oa_rad = is_falling_edge ? wheel.homing_falling_edge_mech_rad : wheel.homing_rising_edge_mech_rad;
                    const f32 edge_local_corrected_rad = edge_mech_oa_rad - wheel.theta_oa_to_owi_rad;

                    wheel.homing_state = HomingState::kEdgeDetected;
                    wheel.homing_last_edge_is_falling = is_falling_edge;
                    wheel.homing_runtime_zero_offset_rad = edge_local_corrected_rad + wheel.homing_zero_offset_rad - raw_total_angle_rad;
                    wheel.homing_zero_valid = true;
                }
                else if (wheel.homing_elapsed_s > wheel.homing_timeout_s)
                {
                    wheel.homing_state = HomingState::kFault;
                }
                wheel.homing_last_sensor_active = sensor_raw_high;
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
            f32 current_oa_total_rad[4] = {0.0f};
            f32 target_drive_raw_rad_s[4] = {0.0f};
            f32 selected_oa_total_rad[4] = {0.0f};
            f32 steering_errors_rad[4] = {0.0f};

            f32 max_command_wheel_speed_m_s = 0.0f;
            f32 max_residual_speed_m_s = 0.0f;

            // 第一阶段：计算每轮原始目标、翻转候选与误差。
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                const f32 current_local_total = wheel.corrected_steer_motor_total_angle_rad;
                current_oa_total_rad[i] = current_local_total + wheel.theta_oa_to_owi_rad;

                const f32 wheel_vx = command_data.vel_x - command_data.omega_z * wheel.pos_y_m;
                const f32 wheel_vy = command_data.vel_y + command_data.omega_z * wheel.pos_x_m;
                const f32 wheel_speed_m_s = magnitude2D(wheel_vx, wheel_vy);
                max_command_wheel_speed_m_s = (wheel_speed_m_s > max_command_wheel_speed_m_s) ? wheel_speed_m_s : max_command_wheel_speed_m_s;

                const f32 residual_speed_m_s = fabsf(wheel.corrected_drive_omega_rad_s) * wheel_radius_m_;
                max_residual_speed_m_s = (residual_speed_m_s > max_residual_speed_m_s) ? residual_speed_m_s : max_residual_speed_m_s;

                const bool is_stationary = wheel_speed_m_s <= stationary_speed_epsilon_m_s_;
                f32 raw_target_oa_mod_rad = 0.0f;
                f32 drive_omega = 0.0f;

                if (is_stationary)
                {
                    raw_target_oa_mod_rad = (idle_posture_mode_ == IdlePostureMode::kXPark)
                                                ? wrapTo2Pi(getXParkAngle(wheel))
                                                : wrapTo2Pi(current_oa_total_rad[i]);
                    drive_omega = 0.0f;
                }
                else
                {
                    raw_target_oa_mod_rad = wrapTo2Pi(atan2f(wheel_vy, wheel_vx));
                    drive_omega = wheel_speed_m_s / wheel_radius_m_;
                }

                const f32 alt_target_oa_mod_rad = wrapTo2Pi(raw_target_oa_mod_rad + kPi);
                const f32 candidate_a = nearestEquivalentAngle(current_oa_total_rad[i], raw_target_oa_mod_rad);
                const f32 candidate_b = nearestEquivalentAngle(current_oa_total_rad[i], alt_target_oa_mod_rad);

                f32 selected = candidate_a;
                bool flipped = false;
                if (!is_stationary)
                {
                    if (runtime_strategy_cfg_.steering_strategy_mode == SteeringStrategyMode::kAlwaysForward)
                    {
                        flipped = false;
                    }
                    else
                    {
                        const f32 base_abs_deg = radToDegF32(fabsf(candidate_a - current_oa_total_rad[i]));
                        const f32 flip_abs_deg = radToDegF32(fabsf(candidate_b - current_oa_total_rad[i]));
                        if (selected_flipped_solution_[i])
                        {
                            flipped = flip_abs_deg <= runtime_strategy_cfg_.flip_enter_angle_deg;
                        }
                        else
                        {
                            flipped = (base_abs_deg > runtime_strategy_cfg_.flip_exit_angle_deg) && (flip_abs_deg < base_abs_deg);
                        }
                    }
                }

                if (flipped)
                {
                    selected = candidate_b;
                    drive_omega = -drive_omega;
                }

                selected_oa_total_rad[i] = selected;
                selected_flipped_solution_[i] = flipped;
                steering_errors_rad[i] = fabsf(shortestAngularDistance(current_oa_total_rad[i], selected));
                target_drive_raw_rad_s[i] = drive_omega;
            }

            // 第二阶段：停车抑制（指令静止但残速未消失时，先保舵角）。
            const bool command_is_stationary = max_command_wheel_speed_m_s <= stationary_speed_epsilon_m_s_;
            const bool residual_drive_is_moving = max_residual_speed_m_s > runtime_strategy_cfg_.stop_guard_release_speed_m_s;
            if (runtime_strategy_cfg_.enable_stop_steer_guard && command_is_stationary && residual_drive_is_moving)
            {
                for (u8 i = 0; i < 4; ++i)
                {
                    const f32 residual_speed_m_s = fabsf(wheel_config_[i].corrected_drive_omega_rad_s) * wheel_radius_m_;
                    const f32 blend = stopSteerGuardBlend(residual_speed_m_s);
                    if (blend >= 1.0f)
                    {
                        continue;
                    }

                    const f32 current = current_oa_total_rad[i];
                    const f32 protected_target = wrapTo2Pi(
                        current + shortestAngularDistance(current, selected_oa_total_rad[i]) * blend);
                    selected_oa_total_rad[i] = protected_target;
                    steering_errors_rad[i] = fabsf(shortestAngularDistance(current, protected_target));
                    selected_flipped_solution_[i] = false;
                }
            }

            // 第三阶段：驱动抑制比例（DriveGate 或余弦补偿）。
            f32 gate_scales[4] = {1.0f, 1.0f, 1.0f, 1.0f};
            computeDriveGateScales(steering_errors_rad, command_data, gate_scales);

            // 第四阶段：下发前限幅与缓存。
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];
                const f32 current_local_total = wheel.corrected_steer_motor_total_angle_rad;

                f32 drive_scale = 1.0f;
                if (runtime_strategy_cfg_.enable_drive_gate)
                {
                    drive_scale = gate_scales[i];
                }
                else if (enable_cosine_compensation_)
                {
                    drive_scale = cosf(steering_errors_rad[i]);
                    if (drive_scale < 0.0f)
                    {
                        drive_scale = 0.0f;
                    }
                }

                f32 target_drive_omega = target_drive_raw_rad_s[i] * drive_scale;
                target_drive_omega = clampValue(target_drive_omega, -max_drive_omega_rad_s_, max_drive_omega_rad_s_);
                drive_gate_scale_[i] = clampValue(drive_scale, 0.0f, 1.0f);

                f32 next_steer_rate_rad_s = 0.0f;
                const f32 selected_local_total = selected_oa_total_rad[i] - wheel.theta_oa_to_owi_rad;
                wheel.target_steer_motor_total_angle_rad = limitPositionSecondOrder(
                    current_local_total,
                    last_steer_rate_cmd_rad_s_[i],
                    selected_local_total,
                    max_steer_rate_rad_s_,
                    max_steer_alpha_rad_s2_,
                    period_,
                    next_steer_rate_rad_s);
                wheel.target_drive_omega_rad_s = clampValue(
                    limitValueWithAcceleration(last_drive_omega_cmd_rad_s_[i], target_drive_omega, max_drive_alpha_rad_s2_, period_),
                    -max_drive_omega_rad_s_,
                    max_drive_omega_rad_s_);

                wheel.steer_target_velocity_rad_s = next_steer_rate_rad_s;
                wheel.flipped_drive_direction = selected_flipped_solution_[i];

                last_steer_rate_cmd_rad_s_[i] = next_steer_rate_rad_s;
                last_drive_omega_cmd_rad_s_[i] = wheel.target_drive_omega_rad_s;
                planned_data_.steer_angle_oa_rad[i] = wheel.target_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad;
                planned_data_.drive_omega_rad_s[i] = wheel.target_drive_omega_rad_s;
            }
        }

        void Chassis::applyModuleCommands(bool all_homed)
        {
            // 这里是“四舵轮目标命令”真正落到电机接口前的最后一道门控：
            // computeModuleCommands() 虽然已经为每个轮子算好了目标舵角和驱动速度，
            // 但是否允许按这些目标下发，还要看当前是否全部完成回零，以及是否处于扭矩自由模式。
            for (u8 i = 0; i < 4; ++i)
            {
                WheelConfig &wheel = wheel_config_[i];

                if (!all_homed)
                {
                    // 只要还有任意一个轮子没有完成回零，就先禁止所有驱动轮输出，
                    // 避免底盘在零位未建立完成时带着错误朝向强行跑动。
                    setDriveMotorTargetOmegaRadS(wheel, 0.0f);
                    if (wheel.homing_state == HomingState::kSearch)
                    {
                        // 正在搜索零位的轮子，允许转向电机按固定搜索转速慢慢转，
                        // 目的是继续寻找传感器边沿；此时不走位置闭环。
                        setSteerMotorTargetRPM(wheel, wheel.homing_search_rpm);
                    }
                    else
                    {
                        // 不在搜索态的轮子，不再给转向动作，直接把转向电机电流打零，
                        // 让状态机以“静止等待”的方式完成后续过渡。
                        setSteerMotorTargetCurrent(wheel, 0.0f);
                    }
                    continue;
                }

                if (current_mode_flag_.is_wheel_torque_free)
                {
                    // 扭矩自由模式下，不执行任何舵角或驱动速度闭环，
                    // 而是把转向和驱动都打成“零电流/零扭矩”状态，方便人工推动或安全释放。
                    setSteerMotorTargetCurrent(wheel, 0.0f);
                    if (wheel.drive_motor_h != nullptr)
                    {
                        wheel.drive_motor_h->setTargetCurrent(0.0f);
                    }
                    continue;
                }

                // 只有“全部回零完成”且“不是扭矩自由模式”时，
                // 才真正把上一阶段规划出的目标舵角和驱动角速度下发给电机闭环。
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

        void Chassis::refreshDebugMirror(bool all_homed)
        {
            debug_all_homed_ = all_homed;
            debug_selected_wheel_steer_error_deg_ = 0.0f;
            debug_selected_wheel_drive_released_ = false;
            for (u8 i = 0; i < 4; ++i)
            {
                const WheelConfig &wheel = wheel_config_[i];
                debug_current_oa_deg_[i] = radToDegF32(wheel.corrected_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad);
                debug_target_oa_deg_[i] = radToDegF32(wheel.target_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad);
                debug_current_drive_rpm_[i] = radsToRpmF32(wheel.corrected_drive_omega_rad_s);
                debug_target_drive_rpm_[i] = radsToRpmF32(wheel.target_drive_omega_rad_s);
                debug_homing_state_[i] = static_cast<u8>(wheel.homing_state);
                debug_homing_sensor_raw_high_[i] = readHomingSensorRawHigh(wheel);
                debug_homing_sensor_active_[i] = readHomingSensor(wheel);
                debug_homing_last_edge_is_falling_[i] = wheel.homing_last_edge_is_falling;
                debug_homing_runtime_zero_offset_deg_[i] = radToDegF32(wheel.homing_runtime_zero_offset_rad);
                debug_flipped_drive_[i] = wheel.flipped_drive_direction;
                debug_drive_gate_scale_dbg_[i] = drive_gate_scale_[i];
            }
        }

        void Chassis::emitDebugUart8Log(bool all_homed)
        {
            if (debug_uart8_output_mode_ != 1U || !debug_uart8_log_enable_)
            {
                return;
            }

            const u32 period_ms = (debug_uart8_log_period_ms_ > 0U) ? debug_uart8_log_period_ms_ : 500U;
            if ((time_ms_ - debug_uart8_log_last_ms_) < period_ms)
            {
                return;
            }

            if (HAL_UART_GetState(&huart8) != HAL_UART_STATE_READY)
            {
                return;
            }

            debug_uart8_log_last_ms_ = time_ms_;
            debug_uart_.printf_DMA((char *)"FS t=%lu home=%u mode=%u dbg=%u hs=%u/%u/%u/%u oa0=%.1f->%.1f rpm0=%.1f->%.1f\r\n",
                                   (u32)time_ms_,
                                   all_homed ? 1U : 0U,
                                   (u32)input_target_data_.mode,
                                   is_debug_ ? 1U : 0U,
                                   (u32)debug_homing_state_[0],
                                   (u32)debug_homing_state_[1],
                                   (u32)debug_homing_state_[2],
                                   (u32)debug_homing_state_[3],
                                   debug_current_oa_deg_[0],
                                   debug_target_oa_deg_[0],
                                   debug_current_drive_rpm_[0],
                                   debug_target_drive_rpm_[0]);

            if (debug_uart8_log_level_ >= 1U && HAL_UART_GetState(&huart8) == HAL_UART_STATE_READY)
            {
                const u8 wheel_idx = (debug_wheel_index_ < 4) ? debug_wheel_index_ : 0;
                debug_uart_.printf_DMA((char *)"FSW i=%u hs=%u oa=%.1f->%.1f rpm=%.1f->%.1f gate=%.2f flip=%u sensor=%u edge=%u rel=%u err=%.2f\r\n",
                                       (u32)wheel_idx,
                                       (u32)debug_homing_state_[wheel_idx],
                                       debug_current_oa_deg_[wheel_idx],
                                       debug_target_oa_deg_[wheel_idx],
                                       debug_current_drive_rpm_[wheel_idx],
                                       debug_target_drive_rpm_[wheel_idx],
                                       debug_drive_gate_scale_dbg_[wheel_idx],
                                       debug_flipped_drive_[wheel_idx] ? 1U : 0U,
                                       debug_homing_sensor_active_[wheel_idx] ? 1U : 0U,
                                       debug_homing_last_edge_is_falling_[wheel_idx] ? 1U : 0U,
                                       debug_selected_wheel_drive_released_ ? 1U : 0U,
                                       debug_selected_wheel_steer_error_deg_);
            }
        }

        void Chassis::emitUart8VofaJustFloatPidTrace()
        {
            if (debug_uart8_output_mode_ != 2U || !debug_uart8_justfloat_enable_)
            {
                return;
            }

            const u32 period_ms = (debug_uart8_justfloat_period_ms_ > 0U) ? debug_uart8_justfloat_period_ms_ : 10U;
            if ((time_ms_ - debug_uart8_justfloat_last_ms_) < period_ms)
            {
                return;
            }

            if (HAL_UART_GetState(&huart8) != HAL_UART_STATE_READY)
            {
                return;
            }

            debug_uart8_justfloat_last_ms_ = time_ms_;
            float payload[25] = {0.0f};
            payload[0] = (f32)time_ms_ * 0.001f;
            for (u8 i = 0; i < 4; ++i)
            {
                payload[1 + i] = debug_target_oa_deg_[i];
                payload[5 + i] = debug_current_oa_deg_[i];
                payload[9 + i] = radToDegF32(shortestAngularDistance(degToRadF32(debug_current_oa_deg_[i]), degToRadF32(debug_target_oa_deg_[i])));
                payload[13 + i] = debug_target_drive_rpm_[i];
                payload[17 + i] = debug_current_drive_rpm_[i];
                payload[21 + i] = (f32)debug_homing_state_[i];
            }
            debug_uart_.printf_DMA_JustFloat(payload, 25);
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

                // 常态同步手柄缓存：即使 is_debug_ 关闭，也保持 airjoy_data_ 实时更新，
                // 便于通过调试器直接观察摇杆输入；不改变任何控制模式接管逻辑。
                CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);

                isDebugMode();
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

                if (is_debug_ && (debug_mode_ == 20 || debug_mode_ == 21 || debug_mode_ == 22 || debug_mode_ == 30))
                {
                    // 调试专用分支：不走底盘速度分解，直接改写模块目标缓存。
                    // 20=单轮直控，21=四轮朝前，22=纯回零观察。
                    // 三者都仍然走 applyModuleCommands(all_homed) 的安全门控，因此未回零前驱动不会放开。
                    const u8 wheel_idx = (debug_wheel_index_ < 4) ? debug_wheel_index_ : 0;
                    const bool use_soft_steer = (debug_mode_ == 20) && debug_wheel_soft_steer_enable_;
                    for (u8 i = 0; i < 4; ++i)
                    {
                        WheelConfig &wheel = wheel_config_[i];
                        wheel.target_steer_motor_total_angle_rad = wheel.corrected_steer_motor_total_angle_rad;
                        wheel.target_drive_omega_rad_s = 0.0f;
                        wheel.steer_target_velocity_rad_s = 0.0f;
                        wheel.flipped_drive_direction = false;
                        planned_data_.steer_angle_oa_rad[i] = wheel.target_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad;
                        planned_data_.drive_omega_rad_s[i] = 0.0f;
                        if (!(use_soft_steer && i == wheel_idx))
                        {
                            last_steer_rate_cmd_rad_s_[i] = 0.0f;
                        }
                        last_drive_omega_cmd_rad_s_[i] = 0.0f;
                    }

                    if (debug_mode_ == 20)
                    {
                        WheelConfig &debug_wheel = wheel_config_[wheel_idx];
                        // 单轮直控也复用“最近等效角”思路：同一个 OA 模角可对应多圈连续角，
                        // 这里优先选离当前 OA 最近的那一圈，避免调试器从 350° 到 10° 时绕远路转一整圈。
                        const f32 target_oa_mod_rad = wrapTo2Pi(degToRadF32(debug_wheel_target_steer_deg_));
                        const f32 current_oa_total_rad = debug_wheel.corrected_steer_motor_total_angle_rad + debug_wheel.theta_oa_to_owi_rad;
                        const f32 target_oa_total_rad = nearestEquivalentAngle(current_oa_total_rad, target_oa_mod_rad);
                        const f32 selected_local_total_rad = target_oa_total_rad - debug_wheel.theta_oa_to_owi_rad;
                        const f32 steer_error_deg = radToDegF32(fabsf(shortestAngularDistance(current_oa_total_rad, target_oa_total_rad)));
                        const f32 drive_release_error_deg = (debug_wheel_drive_release_error_deg_ >= 0.0f) ? debug_wheel_drive_release_error_deg_ : 0.0f;
                        const bool drive_released = !debug_wheel_drive_release_gate_enable_ || (steer_error_deg <= drive_release_error_deg);

                        if (debug_wheel_soft_steer_enable_)
                        {
                            // 软到位模式下复用正常控制链的二阶限幅器，让单轮调试也能观察“有限速/有限加速度”的真实到位过程。
                            f32 steer_limit_rate_rad_s = max_steer_rate_rad_s_;
                            f32 steer_limit_accel_rad_s2 = max_steer_alpha_rad_s2_;
                            if (debug_wheel_use_custom_steer_limit_)
                            {
                                steer_limit_rate_rad_s = degToRadF32(debug_wheel_steer_rate_limit_deg_s_);
                                steer_limit_accel_rad_s2 = degToRadF32(debug_wheel_steer_accel_limit_deg_s2_);
                            }
                            if (steer_limit_rate_rad_s <= 1.0e-6f)
                            {
                                steer_limit_rate_rad_s = max_steer_rate_rad_s_;
                            }
                            if (steer_limit_accel_rad_s2 <= 1.0e-6f)
                            {
                                steer_limit_accel_rad_s2 = max_steer_alpha_rad_s2_;
                            }

                            f32 next_steer_rate_rad_s = 0.0f;
                            debug_wheel.target_steer_motor_total_angle_rad = limitPositionSecondOrder(
                                debug_wheel.corrected_steer_motor_total_angle_rad,
                                last_steer_rate_cmd_rad_s_[wheel_idx],
                                selected_local_total_rad,
                                steer_limit_rate_rad_s,
                                steer_limit_accel_rad_s2,
                                period_,
                                next_steer_rate_rad_s);
                            debug_wheel.steer_target_velocity_rad_s = next_steer_rate_rad_s;
                            last_steer_rate_cmd_rad_s_[wheel_idx] = next_steer_rate_rad_s;
                        }
                        else
                        {
                            debug_wheel.target_steer_motor_total_angle_rad = selected_local_total_rad;
                            debug_wheel.steer_target_velocity_rad_s = 0.0f;
                            last_steer_rate_cmd_rad_s_[wheel_idx] = 0.0f;
                        }

                        debug_wheel.target_drive_omega_rad_s = (is_wheel_speed_mode_ && drive_released) ? rpmToRadsF32(debug_wheel_target_drive_rpm_) : 0.0f;
                        planned_data_.steer_angle_oa_rad[wheel_idx] = debug_wheel.target_steer_motor_total_angle_rad + debug_wheel.theta_oa_to_owi_rad;
                        planned_data_.drive_omega_rad_s[wheel_idx] = debug_wheel.target_drive_omega_rad_s;
                        last_drive_omega_cmd_rad_s_[wheel_idx] = debug_wheel.target_drive_omega_rad_s;
                        debug_selected_wheel_steer_error_deg_ = steer_error_deg;
                        debug_selected_wheel_drive_released_ = drive_released;
#if FOURSTEER_SINGLE_WHEEL_TRACE_UART8
                        if (debug_uart8_output_mode_ == 1U && debug_uart8_log_level_ >= 1U && (time_ms_ - debug_wheel_uart_log_last_ms_) >= 50)
                        {
                            debug_wheel_uart_log_last_ms_ = time_ms_;
                            debug_uart_.printf_DMA((char *)"SW20,%lu,%u,%u,%u,%.3f,%.3f,%.3f,%u,%u\r\n",
                                                   (u32)time_ms_,
                                                   (u32)wheel_idx,
                                                   all_homed ? 1U : 0U,
                                                   (u32)input_target_data_.mode,
                                                   radToDegF32(target_oa_total_rad),
                                                   radToDegF32(current_oa_total_rad),
                                                   steer_error_deg,
                                                   (u32)debug_wheel.homing_state,
                                                   drive_released ? 1U : 0U);
                        }
#endif
                    }
                    else if (debug_mode_ == 21)
                    {
                        for (u8 i = 0; i < 4; ++i)
                        {
                            WheelConfig &wheel = wheel_config_[i];
                            wheel.target_steer_motor_total_angle_rad = -wheel.theta_oa_to_owi_rad;
                            planned_data_.steer_angle_oa_rad[i] = 0.0f;
                        }
                    }
                    else if (debug_mode_ == 30)
                    {
                        const f32 rpm_limit = (debug_direct_drive_rpm_limit_ > 0.0f) ? debug_direct_drive_rpm_limit_ : 300.0f;
                        for (u8 i = 0; i < 4; ++i)
                        {
                            WheelConfig &wheel = wheel_config_[i];
                            if (debug_direct_estop_)
                            {
                                setSteerMotorTargetCurrent(wheel, 0.0f);
                                setDriveMotorTargetOmegaRadS(wheel, 0.0f);
                                wheel.target_drive_omega_rad_s = 0.0f;
                                planned_data_.drive_omega_rad_s[i] = 0.0f;
                                continue;
                            }

                            if (debug_direct_enable_steer_[i])
                            {
                                const f32 target_oa_mod_rad = wrapTo2Pi(degToRadF32(debug_direct_steer_oa_deg_[i]));
                                const f32 current_oa_total_rad = wheel.corrected_steer_motor_total_angle_rad + wheel.theta_oa_to_owi_rad;
                                const f32 target_oa_total_rad = nearestEquivalentAngle(current_oa_total_rad, target_oa_mod_rad);
                                const f32 target_local_total_rad = target_oa_total_rad - wheel.theta_oa_to_owi_rad;
                                wheel.target_steer_motor_total_angle_rad = target_local_total_rad;
                                planned_data_.steer_angle_oa_rad[i] = target_oa_total_rad;
                                setSteerMotorTargetTotalAngleRad(wheel, target_local_total_rad);
                            }
                            else
                            {
                                setSteerMotorTargetCurrent(wheel, 0.0f);
                            }

                            if (debug_direct_enable_drive_[i])
                            {
                                const f32 target_rpm = clampValue(debug_direct_drive_rpm_[i], -rpm_limit, rpm_limit);
                                const f32 target_omega_rad_s = rpmToRadsF32(target_rpm);
                                wheel.target_drive_omega_rad_s = target_omega_rad_s;
                                planned_data_.drive_omega_rad_s[i] = target_omega_rad_s;
                                setDriveMotorTargetOmegaRadS(wheel, target_omega_rad_s);
                            }
                            else
                            {
                                wheel.target_drive_omega_rad_s = 0.0f;
                                planned_data_.drive_omega_rad_s[i] = 0.0f;
                                setDriveMotorTargetOmegaRadS(wheel, 0.0f);
                            }
                        }
                    }

                    planned_data_.vel_x = 0.0f;
                    planned_data_.vel_y = 0.0f;
                    planned_data_.omega_z = 0.0f;
                    planned_data_.acc_x = 0.0f;
                    planned_data_.acc_y = 0.0f;
                    planned_data_.alpha_z = 0.0f;
                    planned_data_.rot_z = input_hwt_rot_z_;

                    if (debug_mode_ != 30)
                    {
                        applyModuleCommands(all_homed);
                    }
                    updateCurrentData(all_homed);
                    refreshDebugMirror(all_homed);
                    emitDebugUart8Log(all_homed);
                    emitUart8VofaJustFloatPidTrace();
                    last_planned_data_ = planned_data_;
                    vTaskDelayUntil(&time_ms_, period_ms_);
                    continue;
                }

                // 回零和正常控制共用同一套命令生成流程，但最终下发前会根据 all_homed 选择：
                // 未回零时只保留安全动作，已回零时才输出完整舵角/驱动目标。
                computeModuleCommands(planned_data_);
                applyModuleCommands(all_homed);
                updateCurrentData(all_homed);
                refreshDebugMirror(all_homed);
                emitDebugUart8Log(all_homed);
                emitUart8VofaJustFloatPidTrace();

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
