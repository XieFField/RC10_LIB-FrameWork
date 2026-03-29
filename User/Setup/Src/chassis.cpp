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

namespace jia
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

        // 初始化轮子位置映射关系
        //  // 1号轮子
        wheel_config &wheel_1 = wheel_config_[0];
        wheel_1.pos_x = 0.0f,
        wheel_1.pos_y = 0.375f,
        wheel_1.rot_z_deg = 0.0f,
        wheel_1.motor_handle = config.motor_handle[0],
        wheel_1.s = sinDegF32(wheel_1.rot_z_deg);
        wheel_1.c = cosDegF32(wheel_1.rot_z_deg);
        wheel_1.eqr = wheel_1.pos_x * wheel_1.s - wheel_1.pos_y * wheel_1.c;
        wheel_1.as = std::abs(wheel_1.s);
        wheel_1.ac = std::abs(wheel_1.c);
        wheel_1.aeqr = std::abs(wheel_1.eqr);
        //  // 2号轮子
        wheel_config &wheel_2 = wheel_config_[1];
        wheel_2.pos_x = -0.37f,
        wheel_2.pos_y = -0.375f,
        wheel_2.rot_z_deg = -63.741f + 180.0f,
        wheel_2.motor_handle = config.motor_handle[1],
        wheel_2.s = sinDegF32(wheel_2.rot_z_deg);
        wheel_2.c = cosDegF32(wheel_2.rot_z_deg);
        wheel_2.eqr = wheel_2.pos_x * wheel_2.s - wheel_2.pos_y * wheel_2.c;
        wheel_2.as = std::abs(wheel_2.s);
        wheel_2.ac = std::abs(wheel_2.c);
        wheel_2.aeqr = std::abs(wheel_2.eqr);
        //  // 3号轮子
        wheel_config &wheel_3 = wheel_config_[2];
        wheel_3.pos_x = 0.37f,
        wheel_3.pos_y = -0.375f,
        wheel_3.rot_z_deg = 63.741f + 180.0f,
        wheel_3.motor_handle = config.motor_handle[2],
        wheel_3.s = sinDegF32(wheel_3.rot_z_deg);
        wheel_3.c = cosDegF32(wheel_3.rot_z_deg);
        wheel_3.eqr = wheel_3.pos_x * wheel_3.s - wheel_3.pos_y * wheel_3.c;
        wheel_3.as = std::abs(wheel_3.s);
        wheel_3.ac = std::abs(wheel_3.c);
        wheel_3.aeqr = std::abs(wheel_3.eqr);

        // 计算底盘最大速度
        //  // 参数检查
        //        if (max_vel_x_radio + max_vel_y_radio + max_omega_z_radio != 1.0f)
        //        {
        //            Error_Handler();
        //        }
        //  // 参数计算
        max_wheel_vel = omegaToVelF32(max_wheel_omega, wr);
        //  // 1号轮子
        f32 wheel_1_max_vel_x = max_wheel_vel * max_vel_x_radio / wheel_1.ac;
        f32 wheel_1_max_vel_y = max_wheel_vel * max_vel_y_radio / wheel_1.as;
        f32 wheel_1_max_omega_z = max_wheel_vel * max_omega_z_radio / wheel_1.aeqr;
        //  // 2号轮子
        f32 wheel_2_max_vel_x = max_wheel_vel * max_vel_x_radio / wheel_2.ac;
        f32 wheel_2_max_vel_y = max_wheel_vel * max_vel_y_radio / wheel_2.as;
        f32 wheel_2_max_omega_z = max_wheel_vel * max_omega_z_radio / wheel_2.aeqr;
        //  // 3号轮子
        f32 wheel_3_max_vel_x = max_wheel_vel * max_vel_x_radio / wheel_3.ac;
        f32 wheel_3_max_vel_y = max_wheel_vel * max_vel_y_radio / wheel_3.as;
        f32 wheel_3_max_omega_z = max_wheel_vel * max_omega_z_radio / wheel_3.aeqr;
        //  // 计算底盘最大速度
        max_vel_x = minOfThree(wheel_1_max_vel_x, wheel_2_max_vel_x, wheel_3_max_vel_x);
        max_vel_y = minOfThree(wheel_1_max_vel_y, wheel_2_max_vel_y, wheel_3_max_vel_y);
        max_omega_z = minOfThree(wheel_1_max_omega_z, wheel_2_max_omega_z, wheel_3_max_omega_z);

        // 初始化omega_z_pid
        omega_z_pid.set_params(omega_z_pid_init_config, 0.0f);

        // 初始化rot_z_pid
        rot_z_pid.set_params(lock_angle_pid_params, 0.0f);
        rot_z_pid.set_as_circular();
    }

    void Chassis::createThread(void *arg)
    {
        Chassis *chassis = static_cast<Chassis *>(arg);
        chassis->runThread(NULL);
    }

    void Chassis::runThread(void *arg)
    {
        // 引用别名
        auto &it = input_target_data;
        auto &t = target_data_;
        auto &tpid = target_pid_data_;
        auto &p = planned_data_;
        auto &lp = last_planned_data_;
        auto &c = current_data_;

        CrsfReceiver *receiver = CrsfReceiver::GetInstance(&huart7);
        HWT101CT *hwt = HWT101CT::GetInstance(&huart8);
        TickType_t time_ms = xTaskGetTickCount();

        for (;;)
        {
            input_hwt_rot_z = hwt->get_yaw_rad();
            input_hwt_omega_z = hwt->get_yaw_speed_rad();

            if (is_debug)
            {
                receiver->getControlData(&airjoy_data);

                f32 target_vel_x = 0.0f;
                f32 target_vel_y = 0.0f;
                f32 target_omega_z = 0.0f;

                target_vel_x = airjoy_data.left_x * max_vel_x;
                target_vel_y = airjoy_data.left_y * max_vel_y;

                // if (airjoy_data.right_x > 0.1f)
                // {
                //     target_omega_z = max_omega_z;
                // }
                // else if (airjoy_data.right_x < -0.1f)
                // {
                //     target_omega_z = -max_omega_z;
                // }
                // else
                // {
                //     target_omega_z = 0.0f;
                // }

                target_omega_z = airjoy_data.right_x * max_omega_z;

                if (is_sine)
                {
                    target_omega_z = sineWaveGeneratorF32(time_ms / 1000.0f, sine_amplitude, sine_frequency, 0.0f, sine_offset);
                }
                else if (is_phase_step)
                {
                    if (airjoy_data.right_x > 0.3f)
                    {
                        target_omega_z = max_omega_z;
                    }
                    else if (airjoy_data.right_x < -0.3f)
                    {
                        target_omega_z = -max_omega_z;
                    }
                    else
                    {
                        target_omega_z = 0.0f;
                    }
                }

                switch (debug_mode)
                {
                case 0:
                {
                    setTargetBodySpeedMode(target_vel_x, target_vel_y, target_omega_z);
                    break;
                }
                case 1:
                {
                    setTargetWorldSpeedMode(target_vel_x, target_vel_y, target_omega_z);
                    break;
                }
                case 2:
                {
                    setTargetBodySpeedLockNowRotZMode(target_vel_x, target_vel_y);
                    break;
                }
                case 3:
                {
                    setTargetWorldSpeedLockNowRotZMode(target_vel_x, target_vel_y);
                    break;
                }
                case 4:
                {
                    setTargetBodySpeedLockToRotZMode(target_vel_x, target_vel_y, debug_lock_rot_z);
                    break;
                }
                case 5:
                {
                    setTargetWorldSpeedLockToRotZMode(target_vel_x, target_vel_y, debug_lock_rot_z);
                    break;
                }
                case 6:
                {
                    setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(target_vel_x, target_vel_y, target_omega_z);
                    break;
                }
                case 7:
                {
                    setTargetBodySpeedLockNowRotZWithNoOmegaZMode(target_vel_x, target_vel_y, target_omega_z);
                    break;
                }
                default:
                {
                    setWheelTorqueFreeMode();
                    break;
                }
                }

                // 调试轮子
                //
                // f32 wheel_speed_input = airjoy_data.left_x * wheel_input_speed_radio;
                // // wheel_speed_input = sineWaveGeneratorF32(time_ms / 1000.0f, sine_amplitude, sine_frequency, 0.0f);
                // auto &wheel_handle = wheel_config_[2].motor_handle;
                // wheel_handle->setTargetRPM(wheel_speed_input);

                // f32 pid_error = wheel_handle->speed_chassis_pid_.error_;
                // // f32 pid_error_last = wheel_handle->speed_chassis_pid_.error_last_;
                // // f32 pid_error_earlier_ = wheel_handle->speed_chassis_pid_.error_earlier_;
                // f32 pid_p = wheel_handle->speed_chassis_pid_.P_Term;
                // f32 pid_i = wheel_handle->speed_chassis_pid_.I_Term;
                // f32 pid_d = wheel_handle->speed_chassis_pid_.D_Term;
                // f32 pid_output = wheel_handle->speed_chassis_pid_.output_;

                // debug_uart.printf_DMA("lu\r\n", time_ms);
                //
            }

            switch (mode_)
            {
            case Mode::kWheelTorqueFreeMode:
                break;
            case Mode::kBodySpeedMode:
                is_world_speed_mode = false;
                is_lock_rot_z = false;
                is_lock_rot_z_with_no_omega_z = false;
                break;
            case Mode::kBodySpeedLockNowRotZMode:
                is_world_speed_mode = false;
                is_lock_rot_z = true;
                is_lock_rot_z_with_no_omega_z = false;
                break;
            case Mode::kBodySpeedLockToRotZMode:
                is_world_speed_mode = false;
                is_lock_rot_z = true;
                is_lock_rot_z_with_no_omega_z = false;
                break;
            case Mode::kWorldSpeedMode:
                is_world_speed_mode = true;
                is_lock_rot_z = false;
                is_lock_rot_z_with_no_omega_z = false;
                break;
            case Mode::kWorldSpeedLockNowRotZMode:
                is_world_speed_mode = true;
                is_lock_rot_z = true;
                is_lock_rot_z_with_no_omega_z = false;
                break;
            case Mode::kWorldSpeedLockToRotZMode:
                is_lock_rot_z = true;
                is_world_speed_mode = true;
                is_lock_rot_z_with_no_omega_z = false;
                break;
            case Mode::kWorldSpeedLockNowRotZWithNoOmegaZMode:
                is_lock_rot_z = true;
                is_world_speed_mode = true;
                is_lock_rot_z_with_no_omega_z = true;
                break;
            case Mode::kBodySpeedLockNowRotZWithNoOmegaZMode:
                is_lock_rot_z = true;
                is_world_speed_mode = false;
                is_lock_rot_z_with_no_omega_z = true;
                break;
            default:

                break;
            }

            if (is_world_speed_mode)
            {
                rotateAroundZAxisF32(it.vel_x, it.vel_y, input_hwt_rot_z, t.vel_x, t.vel_y);
            }
            else
            {
                t.vel_x = it.vel_x;
                t.vel_y = it.vel_y;
            }

            if (is_lock_rot_z_with_no_omega_z)
            {
                if (it.omega_z == 0.0f)
                {
                    rot_z_pid_count++;
                    if (rot_z_pid_count >= rot_z_pid_period)
                    {
                        rot_z_pid_count = 0;
                        t.omega_z = rot_z_pid.pid_calc(radToDegF32(it.rot_z), radToDegF32(input_hwt_rot_z));
                    }
                }
                else
                {
                    t.omega_z = it.omega_z;
                }
            }
            else if (is_lock_rot_z)
            {
                rot_z_pid_count++;
                if (rot_z_pid_count >= rot_z_pid_period)
                {
                    rot_z_pid_count = 0;
                    t.omega_z = rot_z_pid.pid_calc(radToDegF32(it.rot_z), radToDegF32(input_hwt_rot_z));
                }
            }
            else
            {
                t.omega_z = it.omega_z;
            }

            // 逆运动学解算
            //  // 限制车端的目标速度
            t.vel_x = clampValue(t.vel_x, -max_vel_x, max_vel_x);
            t.vel_y = clampValue(t.vel_y, -max_vel_y, max_vel_y);
            t.omega_z = clampValue(t.omega_z, -max_omega_z, max_omega_z);
            //  // 是否开启车端的omega_z闭环控制
            if (is_omega_z_close_loop)
            {
                omega_z_pid_count++;
                if (omega_z_pid_count >= omega_z_pid_period)
                {
                    omega_z_pid_count = 0;
                    tpid.omega_z = omega_z_pid.pid_calc(t.omega_z, input_hwt_omega_z);
                }
            }
            else
            {
                tpid.omega_z = t.omega_z;
            }
            //  // 计算轮端的目标角速度
            inverseKinematics(t.vel_x, t.vel_y, tpid.omega_z, t.w1_omega, t.w2_omega, t.w3_omega);
            //  // 是否限制轮端的目标角速度
            if (is_wheel_omega_limit)
            {
                // 限制轮端的目标角速度
                f32 vel_scale_ratio = scaleThreeValuesToMaxF32(t.w1_omega, t.w2_omega, t.w3_omega,
                                                               max_wheel_omega, max_wheel_omega, max_wheel_omega,
                                                               t.w1_omega, t.w2_omega, t.w3_omega);
                // 计算车端的目标速度
                t.vel_x *= vel_scale_ratio;
                t.vel_y *= vel_scale_ratio;
                tpid.omega_z *= vel_scale_ratio;
            }
            //  // 是否限制车端的规划加速度
            if (is_chassis_acc_limit)
            {
                p.vel_x = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(t.vel_x, p.vel_x, period, max_acc_xy_acc, max_acc_xy_dec);
                p.vel_y = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(t.vel_y, p.vel_y, period, max_acc_xy_acc, max_acc_xy_dec);
                p.omega_z = limit1DSignalRateByTimeSeparateAbsIncAndDecF32(tpid.omega_z, p.omega_z, period, max_alpha_z_acc, max_alpha_z_dec);
            }
            else
            {
                p.vel_x = t.vel_x;
                p.vel_y = t.vel_y;
                p.omega_z = tpid.omega_z;
            }
            //  // 计算车端的规划加速度
            p.acc_x = (p.vel_x - lp.vel_x) / period;
            p.acc_y = (p.vel_y - lp.vel_y) / period;
            p.alpha_z = (p.omega_z - lp.omega_z) / period;
            //  // 计算轮端的规划角速度
            inverseKinematics(p.vel_x, p.vel_y, p.omega_z, p.w1_omega, p.w2_omega, p.w3_omega);
            //  // 计算轮端的规划角加速度
            p.w1_alpha = (p.w1_omega - lp.w1_omega) / period;
            p.w2_alpha = (p.w2_omega - lp.w2_omega) / period;
            p.w3_alpha = (p.w3_omega - lp.w3_omega) / period;
            //  // 是否限制轮端的规划角加速度
            if (is_wheel_alpha_limit)
            {
                // 限制轮端的规划角速度
                f32 acc_scale_ratio = scaleThreeValuesToMaxF32(p.w1_alpha, p.w2_alpha, p.w3_alpha,
                                                               max_wheel_alpha, max_wheel_alpha, max_wheel_alpha,
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
            //  // 发送转速指令
            if (mode_ == Mode::kWheelTorqueFreeMode)
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

            // debug_uart.printf_DMA("%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n", time_ms, t.w1_omega, t.w2_omega, t.w3_omega, p.w1_omega, p.w2_omega, p.w3_omega, c.w1_omega, c.w2_omega, c.w3_omega);
            // debug_uart.printf_DMA("%lu\r\n", time_ms);
            // debug_uart.printf_DMA("%lu,%f,%f,%f,%f\r\n", time_ms, t.w1_omega, p.w1_omega, std::abs(c.w1_omega), std::abs(c.w2_omega));
            // debug_uart.printf_DMA("%f,%f,%f\r\n", input_hwt_omega_z, input_hwt_rot_z, tpid.omega_z);

            printf_period_count++;
            if (printf_period_count >= printf_period_ms)
            {
                printf_period_count = 0;
                // debug_uart.printf_DMA("%f,%f,%f,%f,%f\r\n", it.omega_z, input_hwt_omega_z, tpid.omega_z, t.w3_omega, c.w3_omega);
                debug_uart.printf_DMA("%f,%f,%f,%f\r\n", it.rot_z, input_hwt_rot_z, t.omega_z, input_hwt_omega_z);
            }

            vTaskDelayUntil(&time_ms, period_ms);
        }
    }

    Chassis::Result Chassis::setWheelTorqueFreeMode()
    {
        mode_ = Mode::kWheelTorqueFreeMode;
        return Result::kOk;
    }

    Chassis::Result Chassis::setTargetBodySpeedMode(f32 vel_x, f32 vel_y, f32 omega_z)
    {
        mode_ = Mode::kBodySpeedMode;
        input_target_data.vel_x = vel_x;
        input_target_data.vel_y = vel_y;
        input_target_data.omega_z = omega_z;
        return Result::kOk;
    }

    Chassis::Result Chassis::setTargetBodySpeedLockNowRotZMode(f32 vel_x, f32 vel_y)
    {
        mode_ = Mode::kBodySpeedLockNowRotZMode;
        input_target_data.vel_x = vel_x;
        input_target_data.vel_y = vel_y;
        if (is_lock_rot_z)
        {
        }
        else
        {
            input_target_data.rot_z = input_hwt_rot_z;
        }
        return Result::kOk;
    }

    Chassis::Result Chassis::setTargetBodySpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z)
    {
        mode_ = Mode::kBodySpeedLockToRotZMode;
        input_target_data.vel_x = vel_x;
        input_target_data.vel_y = vel_y;
        input_target_data.rot_z = rot_z;
        return Result::kOk;
    }

    Chassis::Result Chassis::setTargetWorldSpeedMode(f32 vel_x, f32 vel_y, f32 omega_z)
    {
        mode_ = Mode::kWorldSpeedMode;
        input_target_data.vel_x = vel_x;
        input_target_data.vel_y = vel_y;
        input_target_data.omega_z = omega_z;
        return Result::kOk;
    }

    Chassis::Result Chassis::setTargetWorldSpeedLockNowRotZMode(f32 vel_x, f32 vel_y)
    {
        mode_ = Mode::kWorldSpeedLockNowRotZMode;
        input_target_data.vel_x = vel_x;
        input_target_data.vel_y = vel_y;
        if (is_lock_rot_z)
        {
        }
        else
        {
            input_target_data.rot_z = input_hwt_rot_z;
        }
        return Result::kOk;
    }

    Chassis::Result Chassis::setTargetBodySpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z)
    {
        mode_ = Mode::kBodySpeedLockNowRotZWithNoOmegaZMode;
        input_target_data.vel_x = vel_x;
        input_target_data.vel_y = vel_y;
        input_target_data.omega_z = omega_z;

        if (omega_z == 0.0f)
        {
            if (is_lock_rot_z)
            {
            }
            else
            {
                input_target_data.rot_z = input_hwt_rot_z;
            }
        }
        else
        {
            input_target_data.rot_z = input_hwt_rot_z;
        }

        return Result::kOk;
    }

    Chassis::Result Chassis::setTargetWorldSpeedLockNowRotZWithNoOmegaZMode(f32 vel_x, f32 vel_y, f32 omega_z)
    {
        mode_ = Mode::kWorldSpeedLockNowRotZWithNoOmegaZMode;
        input_target_data.vel_x = vel_x;
        input_target_data.vel_y = vel_y;
        input_target_data.omega_z = omega_z;

        if (omega_z == 0.0f)
        {
            if (is_lock_rot_z)
            {
            }
            else
            {
                input_target_data.rot_z = input_hwt_rot_z;
            }
        }
        else
        {
            input_target_data.rot_z = input_hwt_rot_z;
        }

        return Result::kOk;
    }

    Chassis::Result Chassis::setTargetWorldSpeedLockToRotZMode(f32 vel_x, f32 vel_y, f32 rot_z)
    {
        mode_ = Mode::kWorldSpeedLockToRotZMode;
        input_target_data.vel_x = vel_x;
        input_target_data.vel_y = vel_y;
        input_target_data.rot_z = rot_z;
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
        out_w1 = ((in_x * w1_.c + in_y * w1_.s + in_z * w1_.eqr) / wr);
        out_w2 = ((in_x * w2_.c + in_y * w2_.s + in_z * w2_.eqr) / wr);
        out_w3 = ((in_x * w3_.c + in_y * w3_.s + in_z * w3_.eqr) / wr);
    }
}
