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
        auto &wheel_1_s = wheel_1.sin_theta;
        auto &wheel_1_c = wheel_1.cos_theta;
        auto &wheel_1_r = wheel_1.equivalent_radius;
        wheel_1.x = 0.375f,
        wheel_1.y = -0.37f,
        wheel_1.theta_deg = 31.87f + 180.0f,
        wheel_1.radius = 0.075f,
        wheel_1.motor_handle = config.motor_handle[0],
        wheel_1_s = sinDegF32(wheel_1.theta_deg);
        wheel_1_c = cosDegF32(wheel_1.theta_deg);
        wheel_1_r = wheel_1.x * wheel_1_s - wheel_1.y * wheel_1_c;
        // 2号轮子
        wheel_config &wheel_2 = wheel_config_[1];
        auto &wheel_2_s = wheel_2.sin_theta;
        auto &wheel_2_c = wheel_2.cos_theta;
        auto &wheel_2_r = wheel_2.equivalent_radius;
        wheel_2.x = 0.375f,
        wheel_2.y = 0.37f,
        wheel_2.theta_deg = -31.87f + 180.0f + 180.0f,
        wheel_2.radius = 0.075f,
        wheel_2.motor_handle = config.motor_handle[1],
        wheel_2_s = sinDegF32(wheel_2.theta_deg);
        wheel_2_c = cosDegF32(wheel_2.theta_deg);
        wheel_2_r = wheel_2.x * wheel_2_s - wheel_2.y * wheel_2_c;
        // 3号轮子
        wheel_config &wheel_3 = wheel_config_[2];
        auto &wheel_3_s = wheel_3.sin_theta;
        auto &wheel_3_c = wheel_3.cos_theta;
        auto &wheel_3_r = wheel_3.equivalent_radius;
        wheel_3.x = -0.375f,
        wheel_3.y = 0.0f,
        wheel_3.theta_deg = -90.0f + 180.0f,
        wheel_3.radius = 0.075f,
        wheel_3.motor_handle = config.motor_handle[2],
        wheel_3_s = sinDegF32(wheel_3.theta_deg);
        wheel_3_c = cosDegF32(wheel_3.theta_deg);
        wheel_3_r = wheel_3.x * wheel_3_s - wheel_3.y * wheel_3_c;

        // 计算底盘最大速度
        // 1号轮子
        f32 wheel_1_max_vx = max_wheel_rpm * (vx_radio / 100.0f) / wheel_1_c;
        f32 wheel_1_max_vy = max_wheel_rpm * (vy_radio / 100.0f) / wheel_1_s;
        f32 wheel_1_max_wz = max_wheel_rpm * (wz_radio / 100.0f) / wheel_1_r;
        // 2号轮子
        f32 wheel_2_max_vx = max_wheel_rpm * (vx_radio / 100.0f) / wheel_2_c;
        f32 wheel_2_max_vy = max_wheel_rpm * (vy_radio / 100.0f) / wheel_2_s;
        f32 wheel_2_max_wz = max_wheel_rpm * (wz_radio / 100.0f) / wheel_2_r;
        // 3号轮子
        f32 wheel_3_max_vx = max_wheel_rpm * (vx_radio / 100.0f) / wheel_3_c;
        f32 wheel_3_max_vy = max_wheel_rpm * (vy_radio / 100.0f) / wheel_3_s;
        f32 wheel_3_max_wz = max_wheel_rpm * (wz_radio / 100.0f) / wheel_3_r;
        // 计算底盘最大速度
        max_vx = minOfThree(wheel_1_max_vx, wheel_2_max_vx, wheel_3_max_vx);
        max_vy = minOfThree(wheel_1_max_vy, wheel_2_max_vy, wheel_3_max_vy);
        max_wz = minOfThree(wheel_1_max_wz, wheel_2_max_wz, wheel_3_max_wz);
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

            float vx = airjoy_data_.left_y * max_set_vx;
            float vy = airjoy_data_.left_x * max_set_vy;
            float wz = airjoy_data_.right_x * max_set_wz_deg * kPi / 180.0f;

            TargetBodySpeedModeData target_data;
            target_data.vx = vx;
            target_data.vy = vy;
            target_data.wz = wz;

            this->setTargetBodySpeedMode(target_data);

            // 处理不同模式的逻辑
            switch (mode_)
            {
            case Mode::kBodySpeedMode:
            {
                // 逆运动学解算
                // 获取底盘目标速度
                const f32 &t_vx = target_data_.vx; // 线速度，单位：m/s
                const f32 &t_vy = target_data_.vy; // 线速度，单位：m/s
                const f32 &t_wz = target_data_.wz; // 角速度，单位：rad/s

                // 加速度限幅
                planned_data_.vx = limit1DSignalRateByTimeF32(t_vx, planned_data_.vx, period_ms / 1000.0f, max_v_acc);
                planned_data_.vy = limit1DSignalRateByTimeF32(t_vy, planned_data_.vy, period_ms / 1000.0f, max_v_acc);
                planned_data_.wz = limit1DSignalRateByTimeF32(t_wz, planned_data_.wz, period_ms / 1000.0f, max_w_acc_deg * kPi / 180.0f);
                f32 wz_deg = planned_data_.wz * 180.0f / kPi; // 角速度，单位：deg/s

                // 计算四个电机的目标转速
                planned_data_.w1_rpm = ((planned_data_.vx * wheel_config_[0].cos_theta + planned_data_.vy * wheel_config_[0].sin_theta + planned_data_.wz * wheel_config_[0].equivalent_radius) / wheel_config_[0].radius) * 60.0f / (2.0f * kPi); // 单位：rpm
                planned_data_.w2_rpm = ((planned_data_.vx * wheel_config_[1].cos_theta + planned_data_.vy * wheel_config_[1].sin_theta + planned_data_.wz * wheel_config_[1].equivalent_radius) / wheel_config_[1].radius) * 60.0f / (2.0f * kPi); // 单位：rpm
                planned_data_.w3_rpm = ((planned_data_.vx * wheel_config_[2].cos_theta + planned_data_.vy * wheel_config_[2].sin_theta + planned_data_.wz * wheel_config_[2].equivalent_radius) / wheel_config_[2].radius) * 60.0f / (2.0f * kPi); // 单位：rpm

                // 发送指令到电机
                wheel_config_[0].motor_handle->setTargetRPM(planned_data_.w1_rpm);
                wheel_config_[1].motor_handle->setTargetRPM(planned_data_.w2_rpm);
                wheel_config_[2].motor_handle->setTargetRPM(planned_data_.w3_rpm);

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
        target_data_.vx = clampValue(target.vx, -max_vx, max_vx);
        target_data_.vy = clampValue(target.vy, -max_vy, max_vy);
        target_data_.wz = clampValue(target.wz, -max_wz, max_wz);

        return Result::kOk;
    }
}
