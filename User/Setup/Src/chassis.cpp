#include "User/Setup/Inc/chassis.h"

#include <cmath>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "BSP_TimeStamp.h"
#include "Module_CrsfReceiver.h"
#include "Module_HWT.h"
#include "APP_PID.h"

#include "RC10_LIB/APP/Inc/APP_Utils.h"
#include "chassis.h"

namespace jia
{
    namespace TriOmniChassis
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

                isTransSpeedBodyToWorld(is_world_speed_mode_, it.vel_x, it.vel_y, t.vel_x, t.vel_y);

                if (is_lock_now_rot_z_)
                    isLockNowRotZ(is_lock_now_rot_z_, t.rot_z, it.omega_z, t.rot_z, t.omega_z);
                if (is_lock_to_rot_z_)
                    isLockToRotZ(is_lock_to_rot_z_, it.rot_z, t.rot_z, t.rot_z, it.omega_z, t.omega_z);

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
                    w1_.h->setTargetCurrent(0.0f);
                    w2_.h->setTargetCurrent(0.0f);
                    w3_.h->setTargetCurrent(0.0f);
                }
                else
                {
                    w1_.h->setTargetRPM(radsToRpmF32(p.w1_omega));
                    w2_.h->setTargetRPM(radsToRpmF32(p.w2_omega));
                    w3_.h->setTargetRPM(radsToRpmF32(p.w3_omega));
                }

                // 正运动学解算
                //  // 读取三个电机的反馈
                c.w1_omega = rpmToRadsF32(w1_.h->getRPM());
                c.w2_omega = rpmToRadsF32(w2_.h->getRPM());
                c.w3_omega = rpmToRadsF32(w3_.h->getRPM());

                // 保存当前数据为上一次数据
                lp = p;

                // debug_uart_.printf_DMA("%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n", time_ms_, t.w1_omega, t.w2_omega, t.w3_omega, p.w1_omega, p.w2_omega, p.w3_omega, c.w1_omega, c.w2_omega, c.w3_omega);
                // debug_uart_.printf_DMA("%lu\r\n", time_ms_);
                // debug_uart_.printf_DMA("%lu,%f,%f,%f,%f\r\n", time_ms_, t.w1_omega, p.w1_omega, std::abs(c.w1_omega), std::abs(c.w2_omega));
                // debug_uart_.printf_DMA("%f,%f,%f\r\n", input_hwt_omega_z_, input_hwt_rot_z_, tpid.omega_z);

                // f32 t_current = wheel_config_[2].motor_handle->getTargetCurrent();
                // f32 c_current = wheel_config_[2].motor_handle->current_;

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
                is_world_speed_mode_ = false;
                is_lock_now_rot_z_ = false;
                is_lock_to_rot_z_ = false;
                break;
            case Mode::kBodySpeedMode:
                is_world_speed_mode_ = false;
                is_lock_now_rot_z_ = false;
                is_lock_to_rot_z_ = false;
                break;
            case Mode::kBodySpeedLockNowRotZMode:
                is_world_speed_mode_ = false;
                is_lock_now_rot_z_ = true;
                is_lock_to_rot_z_ = false;
                break;
            case Mode::kBodySpeedLockToRotZMode:
                is_world_speed_mode_ = false;
                is_lock_now_rot_z_ = false;
                is_lock_to_rot_z_ = true;
                break;
            case Mode::kWorldSpeedMode:
                is_world_speed_mode_ = true;
                is_lock_now_rot_z_ = false;
                is_lock_to_rot_z_ = false;
                break;
            case Mode::kWorldSpeedLockNowRotZMode:
                is_world_speed_mode_ = true;
                is_lock_now_rot_z_ = true;
                is_lock_to_rot_z_ = false;
                break;
            case Mode::kWorldSpeedLockToRotZMode:
                is_world_speed_mode_ = true;
                is_lock_now_rot_z_ = false;
                is_lock_to_rot_z_ = true;
                break;
            case Mode::kWorldSpeedLockNowRotZWithNoOmegaZMode:
                is_world_speed_mode_ = true;
                is_lock_now_rot_z_ = true;
                is_lock_to_rot_z_ = false;
                break;
            case Mode::kBodySpeedLockNowRotZWithNoOmegaZMode:
                is_world_speed_mode_ = false;
                is_lock_now_rot_z_ = true;
                is_lock_to_rot_z_ = false;
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
                // auto &wheel_handle = wheel_config_[debug_wheel_index_].motor_handle;

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
                //     wheel_handle->setTargetRPM(t_rpm);
                // }
                // else if (is_wheel_current_mode_)
                // {
                //     wheel_handle->setTargetCurrent(t_rpm);
                // }
                // else
                // {
                //     wheel_handle->setTargetCurrent(0.0f);
                // }

                // f32 pid_error = wheel_handle->speed_pid_.error_;
                // // f32 pid_error_last = wheel_handle->speed_pid_.error_last_;
                // // f32 pid_error_earlier_ = wheel_handle->speed_pid_.error_earlier_;
                // f32 pid_p = wheel_handle->speed_pid_.P_Term;
                // f32 pid_i = wheel_handle->speed_pid_.I_Term;
                // f32 pid_d = wheel_handle->speed_pid_.D_Term;
                // f32 pid_output = wheel_handle->speed_pid_.output_;

                // f32 t_current = wheel_handle->getTargetCurrent();

                // f32 c_current = wheel_handle->current_;
                // f32 c_rpm = wheel_handle->getRPM();

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

            out_vel_x = vel_x * cos_theta + vel_y * sin_theta;
            out_vel_y = -vel_x * sin_theta + vel_y * cos_theta;
        }

        void Chassis::transSpeedWorldToBody(f32 vel_x, f32 vel_y, f32 &out_vel_x, f32 &out_vel_y)
        {
            f32 cos_theta = cosf(-input_hwt_rot_z_);
            f32 sin_theta = sinf(-input_hwt_rot_z_);

            out_vel_x = vel_x * cos_theta + vel_y * sin_theta;
            out_vel_y = -vel_x * sin_theta + vel_y * cos_theta;
        }

        void Chassis::isLockNowRotZ(bool is_lock, f32 rot_z, f32 omega_z, f32 &out_rot_z, f32 &out_omega_z)
        {
            if (is_lock)
            {
                if (omega_z == 0.0f)
                {
                    calculatePid(rot_z_pid_, rot_z_pid_count_, rot_z_pid_period_,
                                 radToDegF32(rot_z), radToDegF32(input_hwt_rot_z_),
                                 out_omega_z);
                }
                else
                {
                    out_omega_z = omega_z;
                    out_rot_z = input_hwt_rot_z_;
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

        void Chassis::isLockToRotZ(bool is_lock, f32 tar_rot_z, f32 pla_rot_z, f32 &out_rot_z, f32 omega_z, f32 &out_omega_z)
        {
            if (is_lock)
            {
                out_rot_z = limit1DSignalRateByTimeF32(tar_rot_z, pla_rot_z, period_, max_lock_to_rot_z_radio_);

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

            // 计算底盘最大速度
            max_wheel_vel_ = omegaToVelF32(max_wheel_omega_, swr_);
        }

        void Chassis::createThread(void *arg)
        {
            Chassis *chassis = static_cast<Chassis *>(arg);
            chassis->runThread(NULL);
        }

        void Chassis::runThread(void *arg)
        {
            HWT101CT *hwt = HWT101CT::GetInstance(&huart8);
            time_ms_ = xTaskGetTickCount();

            for (;;)
            {
                // printf_period_count_++;
                // if (printf_period_count_ >= printf_period_ms_)
                // {
                //     printf_period_count_ = 0;
                // }

                vTaskDelayUntil(&time_ms_, period_ms_);
            }
        }
    }
}
