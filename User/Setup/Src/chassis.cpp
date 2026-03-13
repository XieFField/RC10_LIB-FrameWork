#include "User/Setup/Inc/chassis.h"

#include <cmath>

#include "cmsis_os2.h"

#include "Module_CrsfReceiver.h"

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
        if (vel_x_radio + vel_y_radio + omega_z_radio != 1.0f)
        {
            Error_Handler();
        }
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
    }

    void Chassis::createThread(void *arg)
    {
        Chassis *chassis = static_cast<Chassis *>(arg);
        chassis->runThread(NULL);
    }

    void Chassis::runThread(void *arg)
    {
        static CrsfReceiver *receiver = CrsfReceiver::GetInstance(&huart7);
        static RmPocketData_t airjoy_data_;

        for (;;)
        {
            receiver->getControlData(&airjoy_data_);

            TargetBodySpeedModeData target_data;
            target_data.vel_x = airjoy_data_.left_y * max_set_vel_x;
            target_data.vel_y = airjoy_data_.left_x * max_set_vel_y;
            target_data.omega_z = airjoy_data_.right_x * max_set_omega_z_deg * kPi / 180.0f;

            this->setTargetBodySpeedMode(target_data);

            // 处理不同模式的逻辑
            switch (mode_)
            {
            case Mode::kBodySpeedMode:
            {
                // 逆运动学解算
                // 引用别名
                auto &t = target_data_;
                auto &t2 = target_data_2_;
                auto &p = planned_data_;
                // 计算底盘最大速度
                t2.vel_x = clampValue(t.vel_x, -max_vel_x, max_vel_x);
                t2.vel_y = clampValue(t.vel_y, -max_vel_y, max_vel_y);
                t2.omega_z = clampValue(t.omega_z, -max_omega_z, max_omega_z);
                // 计算底盘加速度
                f32 last_p_vel_x = p.vel_x;
                f32 last_p_vel_y = p.vel_y;
                f32 last_p_omega_z = p.omega_z;
                p.vel_x = limit1DSignalRateByTimeF32(t2.vel_x, p.vel_x, period, max_acc);
                p.vel_y = limit1DSignalRateByTimeF32(t2.vel_y, p.vel_y, period, max_acc);
                p.omega_z = limit1DSignalRateByTimeF32(t2.omega_z, p.omega_z, period, max_alpha_deg * kPi / 180.0f);
                p.acc_x = (p.vel_x - last_p_vel_x) / (period);
                p.acc_y = (p.vel_y - last_p_vel_y) / (period);
                p.alpha_z = (p.omega_z - last_p_omega_z) / (period);
                last_p_vel_x = p.vel_x;
                last_p_vel_y = p.vel_y;
                last_p_omega_z = p.omega_z;
                // 计算三个电机的目标角加速度
                p.w1_alpha = (p.acc_x * wheel_config_[0].c + p.acc_y * wheel_config_[0].s + p.alpha_z * wheel_config_[0].eqr) / wr;
                p.w2_alpha = (p.acc_x * wheel_config_[1].c + p.acc_y * wheel_config_[1].s + p.alpha_z * wheel_config_[1].eqr) / wr;
                p.w3_alpha = (p.acc_x * wheel_config_[2].c + p.acc_y * wheel_config_[2].s + p.alpha_z * wheel_config_[2].eqr) / wr;
                // 计算三个电机的目标角速度
                p.w1_omega = ((p.vel_x * wheel_config_[0].c + p.vel_y * wheel_config_[0].s + p.omega_z * wheel_config_[0].eqr) / wr);
                p.w2_omega = ((p.vel_x * wheel_config_[1].c + p.vel_y * wheel_config_[1].s + p.omega_z * wheel_config_[1].eqr) / wr);
                p.w3_omega = ((p.vel_x * wheel_config_[2].c + p.vel_y * wheel_config_[2].s + p.omega_z * wheel_config_[2].eqr) / wr);
                // 发送指令到电机
                wheel_config_[0].motor_handle->setTargetRPM(RadsToRpmF32(p.w1_omega));
                wheel_config_[1].motor_handle->setTargetRPM(RadsToRpmF32(p.w2_omega));
                wheel_config_[2].motor_handle->setTargetRPM(RadsToRpmF32(p.w3_omega));

                break;
            }
            default:
            {
                Error_Handler();
                break;
            }
            }

            osDelay(1);
        }
    }

    Chassis::Result Chassis::setTargetBodySpeedMode(const TargetBodySpeedModeData &target)
    {
        mode_ = Mode::kBodySpeedMode;

        // 对目标速度进行限幅
        target_data_.vel_x = target.vel_x;
        target_data_.vel_y = target.vel_y;
        target_data_.omega_z = target.omega_z;

        return Result::kOk;
    }
}
