#include "User/Setup/Inc/chassis.h"

#include <cmath>

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "BSP_TimeStamp.h"

#include "RC10_LIB/APP/Inc/APP_Utils.h"

namespace jia
{
    void Chassis::init(init_config &config)
    {
        const osThreadAttr_t thread_attributes = {
            .name = "chassis_thread",
            .stack_size = 500 * 4,
            .priority = (osPriority_t)(osPriorityHigh),
        };

        osThreadId_t thread_handle = NULL;
        thread_handle = osThreadNew(this->createThread, this, &thread_attributes);
        if (thread_handle == NULL)
        {
            Error_Handler();
        }

        // 初始化轮子位置映射关系
        // 1号轮子
        wheel_config &wheel_1 = wheel_config_[0];
        wheel_1.pos_x = 0.375f,
        wheel_1.pos_y = -0.37f,
        wheel_1.yaw_deg = 31.87f + 180.0f,
        wheel_1.motor_handle = config.motor_handle[0],
        wheel_1.s = sinDegF32(wheel_1.yaw_deg);
        wheel_1.c = cosDegF32(wheel_1.yaw_deg);
        wheel_1.eqr = wheel_1.pos_x * wheel_1.s - wheel_1.pos_y * wheel_1.c;
        wheel_1.as = std::abs(wheel_1.s);
        wheel_1.ac = std::abs(wheel_1.c);
        wheel_1.aeqr = std::abs(wheel_1.eqr);
        // 2号轮子
        wheel_config &wheel_2 = wheel_config_[1];
        wheel_2.pos_x = 0.375f,
        wheel_2.pos_y = 0.37f,
        wheel_2.yaw_deg = -31.87f + 180.0f + 180.0f,
        wheel_2.motor_handle = config.motor_handle[1],
        wheel_2.s = sinDegF32(wheel_2.yaw_deg);
        wheel_2.c = cosDegF32(wheel_2.yaw_deg);
        wheel_2.eqr = wheel_2.pos_x * wheel_2.s - wheel_2.pos_y * wheel_2.c;
        wheel_2.as = std::abs(wheel_2.s);
        wheel_2.ac = std::abs(wheel_2.c);
        wheel_2.aeqr = std::abs(wheel_2.eqr);
        // 3号轮子
        wheel_config &wheel_3 = wheel_config_[2];
        wheel_3.pos_x = -0.375f,
        wheel_3.pos_y = 0.0f,
        wheel_3.yaw_deg = -90.0f + 180.0f,
        wheel_3.motor_handle = config.motor_handle[2],
        wheel_3.s = sinDegF32(wheel_3.yaw_deg);
        wheel_3.c = cosDegF32(wheel_3.yaw_deg);
        wheel_3.eqr = wheel_3.pos_x * wheel_3.s - wheel_3.pos_y * wheel_3.c;
        wheel_3.as = std::abs(wheel_3.s);
        wheel_3.ac = std::abs(wheel_3.c);
        wheel_3.aeqr = std::abs(wheel_3.eqr);

        // 计算底盘最大速度
        // 参数检查
        // if (vel_x_radio + vel_y_radio + omega_z_radio != 1.0f)
        // {
        //     Error_Handler();
        // }
        // 参数计算
        max_wheel_vel = omegaToVelF32(rpmToRadsF32(max_wheel_omega_rpm), wr);
        // 1号轮子
        f32 wheel_1_max_vel_x = max_wheel_vel * vel_x_radio / wheel_1.ac;
        f32 wheel_1_max_vel_y = max_wheel_vel * vel_y_radio / wheel_1.as;
        f32 wheel_1_max_omega_z = max_wheel_vel * omega_z_radio / wheel_1.aeqr;
        // 2号轮子
        f32 wheel_2_max_vel_x = max_wheel_vel * vel_x_radio / wheel_2.ac;
        f32 wheel_2_max_vel_y = max_wheel_vel * vel_y_radio / wheel_2.as;
        f32 wheel_2_max_omega_z = max_wheel_vel * omega_z_radio / wheel_2.aeqr;
        // 3号轮子
        f32 wheel_3_max_vel_x = max_wheel_vel * vel_x_radio / wheel_3.ac;
        f32 wheel_3_max_vel_y = max_wheel_vel * vel_y_radio / wheel_3.as;
        f32 wheel_3_max_omega_z = max_wheel_vel * omega_z_radio / wheel_3.aeqr;
        // 计算底盘最大速度
        max_vel_x = minOfThree(wheel_1_max_vel_x, wheel_2_max_vel_x, wheel_3_max_vel_x);
        max_vel_y = minOfThree(wheel_1_max_vel_y, wheel_2_max_vel_y, wheel_3_max_vel_y);
        max_omega_z = minOfThree(wheel_1_max_omega_z, wheel_2_max_omega_z, wheel_3_max_omega_z);
        max_omega_deg_z = max_omega_z * 180.0f / kPi;
    }

    void Chassis::createThread(void *arg)
    {
        Chassis *chassis = static_cast<Chassis *>(arg);
        chassis->runThread(NULL);
    }

    void Chassis::runThread(void *arg)
    {
        CrsfReceiver *receiver = CrsfReceiver::GetInstance(&huart7);
        TickType_t time_ms = xTaskGetTickCount();

        for (;;)
        {
            receiver->getControlData(&input_airjoy_data);

            TargetBodySpeedModeData target_data;
            // target_data.vel_x = input_airjoy_data.left_y * max_set_vel_x;
            target_data.vel_y = -input_airjoy_data.left_x * max_set_vel_y;

            if(input_airjoy_data.left_y > 0.1f)
            {
                target_data.vel_x = max_set_vel_x;
            }
            else if(input_airjoy_data.left_y < -0.1f)
            {
                target_data.vel_x = -max_set_vel_x;
            }
            else
            {
                target_data.vel_x = 0.0f;
            }

            target_data.omega_z = input_airjoy_data.right_x * max_set_omega_z_deg * kPi / 180.0f;

            this->setTargetBodySpeedMode(target_data);

            // 调试轮子
            //
            // wheel_speed_input = input_airjoy_data.left_x * wheel_input_speed_radio;
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

            // 处理不同模式的逻辑
            switch (mode_)
            {
            case Mode::kBodySpeedMode:
            {
                // 逆运动学解算
                // 引用别名
                auto &it = input_target_data;
                auto &t = target_data_;
                auto &p = planned_data_;
                auto &lp = last_planned_data_;
                auto &c = current_data_;
                // 计算底盘最大速度
                t.vel_x = clampValue(it.vel_x, -max_vel_x, max_vel_x);
                t.vel_y = clampValue(it.vel_y, -max_vel_y, max_vel_y);
                t.omega_z = clampValue(it.omega_z, -max_omega_z, max_omega_z);
                // 计算三个电机的目标角速度
                inverseKinematicsVel(t);
                // 计算底盘加速度
                p.vel_x = limit1DSignalRateByTimeF32(t.vel_x, p.vel_x, period, max_acc);
                p.vel_y = limit1DSignalRateByTimeF32(t.vel_y, p.vel_y, period, max_acc);
                p.omega_z = limit1DSignalRateByTimeF32(t.omega_z, p.omega_z, period, max_alpha_deg * kPi / 180.0f);
                // 计算三个电机的规划角速度
                inverseKinematicsVel(p);
                // 发送指令到电机
                w1_.h->setTargetRPM(radsToRpmF32(p.w1_omega));
                w2_.h->setTargetRPM(radsToRpmF32(p.w2_omega));
                w3_.h->setTargetRPM(radsToRpmF32(p.w3_omega));
                // 计算三个电机的规划角加速度
                p.w1_alpha = (p.w1_omega - lp.w1_omega) / period;
                p.w2_alpha = (p.w2_omega - lp.w2_omega) / period;
                p.w3_alpha = (p.w3_omega - lp.w3_omega) / period;

                // 正运动学解算
                // 读取电机反馈
                c.w1_omega_rpm = w1_.h->getRPM();
                c.w2_omega_rpm = w2_.h->getRPM();
                c.w3_omega_rpm = w3_.h->getRPM();
                c.w1_omega = rpmToRadsF32(c.w1_omega_rpm);
                c.w2_omega = rpmToRadsF32(c.w2_omega_rpm);
                c.w3_omega = rpmToRadsF32(c.w3_omega_rpm);

                // 保存当前数据为上一次数据
                lp = p;

                // debug_uart.printf_DMA("%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n", time_ms, t.w1_omega, t.w2_omega, t.w3_omega, p.w1_omega, p.w2_omega, p.w3_omega, c.w1_omega, c.w2_omega, c.w3_omega);
                // debug_uart.printf_DMA("%lu\r\n", time_ms);
                debug_uart.printf_DMA("%lu,%f,%f,%f,%f\r\n", time_ms,t.w1_omega,p.w1_omega,std::abs(c.w1_omega_rpm),std::abs(c.w2_omega_rpm));

                break;
            }
            default:
            {
                Error_Handler();
                break;
            }
            }

            // debug_uart.printf_DMA("123");

            vTaskDelayUntil(&time_ms, 1);
        }
    }

    Chassis::Result Chassis::setTargetBodySpeedMode(const TargetBodySpeedModeData &target)
    {
        mode_ = Mode::kBodySpeedMode;

        // 对目标速度进行限幅
        input_target_data.vel_x = target.vel_x;
        input_target_data.vel_y = target.vel_y;
        input_target_data.omega_z = target.omega_z;

        return Result::kOk;
    }

    void Chassis::inverseKinematicsVel(Data &data)
    {
        data.w1_omega = ((data.vel_x * w1_.c + data.vel_y * w1_.s + data.omega_z * w1_.eqr) / wr);
        data.w2_omega = ((data.vel_x * w2_.c + data.vel_y * w2_.s + data.omega_z * w2_.eqr) / wr);
        data.w3_omega = ((data.vel_x * w3_.c + data.vel_y * w3_.s + data.omega_z * w3_.eqr) / wr);
    }
}
