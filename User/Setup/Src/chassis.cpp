#include "User/Setup/Inc/chassis.h"

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
        auto &wheel_1_s = wheel_1.sin_yaw;
        auto &wheel_1_c = wheel_1.cos_yaw;
        auto &wheel_1_r = wheel_1.eq_radius;
        wheel_1.pos_x = 0.375f,
        wheel_1.pos_y = -0.37f,
        wheel_1.yaw_deg = 31.87f + 180.0f,
        wheel_1.radius = 0.075f,
        wheel_1.motor_handle = config.motor_handle[0],
        wheel_1_s = sinDegF32(wheel_1.yaw_deg);
        wheel_1_c = cosDegF32(wheel_1.yaw_deg);
        wheel_1_r = wheel_1.pos_x * wheel_1_s - wheel_1.pos_y * wheel_1_c;
        // 2号轮子
        wheel_config &wheel_2 = wheel_config_[1];
        auto &wheel_2_s = wheel_2.sin_yaw;
        auto &wheel_2_c = wheel_2.cos_yaw;
        auto &wheel_2_r = wheel_2.eq_radius;
        wheel_2.pos_x = 0.375f,
        wheel_2.pos_y = 0.37f,
        wheel_2.yaw_deg = -31.87f + 180.0f + 180.0f,
        wheel_2.radius = 0.075f,
        wheel_2.motor_handle = config.motor_handle[1],
        wheel_2_s = sinDegF32(wheel_2.yaw_deg);
        wheel_2_c = cosDegF32(wheel_2.yaw_deg);
        wheel_2_r = wheel_2.pos_x * wheel_2_s - wheel_2.pos_y * wheel_2_c;
        // 3号轮子
        wheel_config &wheel_3 = wheel_config_[2];
        auto &wheel_3_s = wheel_3.sin_yaw;
        auto &wheel_3_c = wheel_3.cos_yaw;
        auto &wheel_3_r = wheel_3.eq_radius;
        wheel_3.pos_x = -0.375f,
        wheel_3.pos_y = 0.0f,
        wheel_3.yaw_deg = -90.0f + 180.0f,
        wheel_3.radius = 0.075f,
        wheel_3.motor_handle = config.motor_handle[2],
        wheel_3_s = sinDegF32(wheel_3.yaw_deg);
        wheel_3_c = cosDegF32(wheel_3.yaw_deg);
        wheel_3_r = wheel_3.pos_x * wheel_3_s - wheel_3.pos_y * wheel_3_c;

        // 计算底盘最大速度
        // 1号轮子
        f32 wheel_1_max_vel_x = max_wheel_rpm * (vel_x_radio / 100.0f) / wheel_1_c;
        f32 wheel_1_max_vel_y = max_wheel_rpm * (vel_y_radio / 100.0f) / wheel_1_s;
        f32 wheel_1_max_omega_z = max_wheel_rpm * (omega_z_radio / 100.0f) / wheel_1_r;
        // 2号轮子
        f32 wheel_2_max_vel_x = max_wheel_rpm * (vel_x_radio / 100.0f) / wheel_2_c;
        f32 wheel_2_max_vel_y = max_wheel_rpm * (vel_y_radio / 100.0f) / wheel_2_s;
        f32 wheel_2_max_omega_z = max_wheel_rpm * (omega_z_radio / 100.0f) / wheel_2_r;
        // 3号轮子
        f32 wheel_3_max_vel_x = max_wheel_rpm * (vel_x_radio / 100.0f) / wheel_3_c;
        f32 wheel_3_max_vel_y = max_wheel_rpm * (vel_y_radio / 100.0f) / wheel_3_s;
        f32 wheel_3_max_omega_z = max_wheel_rpm * (omega_z_radio / 100.0f) / wheel_3_r;
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
                f32 &t_vx = target_data_.vel_x;
                f32 &t_vy = target_data_.vel_y;
                f32 &t_wz = target_data_.omega_z;
                f32 &p_vx = planned_data_.vel_x;
                f32 &p_vy = planned_data_.vel_y;
                f32 &p_wz = planned_data_.omega_z;
                f32 &p_w1_rpm = planned_data_.w1_rpm;
                f32 &p_w2_rpm = planned_data_.w2_rpm;
                f32 &p_w3_rpm = planned_data_.w3_rpm;
                // 限制最大速度
                t_vx = clampValue(t_vx, -max_vel_x, max_vel_x);
                t_vy = clampValue(t_vy, -max_vel_y, max_vel_y);
                t_wz = clampValue(t_wz, -max_omega_z, max_omega_z);
                // 加速度限幅
                p_vx = limit1DSignalRateByTimeF32(t_vx, p_vx, period_ms / 1000.0f, max_acc);
                p_vy = limit1DSignalRateByTimeF32(t_vy, p_vy, period_ms / 1000.0f, max_acc);
                p_wz = limit1DSignalRateByTimeF32(t_wz, p_wz, period_ms / 1000.0f, max_alpha_deg * kPi / 180.0f);
                // 计算四个电机的目标转速
                p_w1_rpm = ((p_vx * wheel_config_[0].cos_yaw + p_vy * wheel_config_[0].sin_yaw + p_wz * wheel_config_[0].eq_radius) / wheel_config_[0].radius) * 60.0f / (2.0f * kPi); // 单位：rpm
                p_w2_rpm = ((p_vx * wheel_config_[1].cos_yaw + p_vy * wheel_config_[1].sin_yaw + p_wz * wheel_config_[1].eq_radius) / wheel_config_[1].radius) * 60.0f / (2.0f * kPi); // 单位：rpm
                p_w3_rpm = ((p_vx * wheel_config_[2].cos_yaw + p_vy * wheel_config_[2].sin_yaw + p_wz * wheel_config_[2].eq_radius) / wheel_config_[2].radius) * 60.0f / (2.0f * kPi); // 单位：rpm
                // 发送指令到电机
                wheel_config_[0].motor_handle->setTargetRPM(p_w1_rpm);
                wheel_config_[1].motor_handle->setTargetRPM(p_w2_rpm);
                wheel_config_[2].motor_handle->setTargetRPM(p_w3_rpm);

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
