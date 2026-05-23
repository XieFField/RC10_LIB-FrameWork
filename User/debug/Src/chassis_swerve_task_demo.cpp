#include "chassis_swerve_task_demo.h"

void Swerve_Task_Demo::loop()
{
    CrsfReceiver::GetInstance(&huart7)->getControlData(&airjoy_data_);
    this->update_yaw(Locate_Setup::getInstance()->get_RobotPos_inWorld().yaw);

    switch (airjoy_data_.SWB)
    {
    case 0x00: // STOP：零速，保持当前舵角
        this->set_body_speed(0.0f, 0.0f, 0.0f);
        break;

    case 0x01: // 世界坐标系驱动
    {
        float vx = airjoy_data_.left_y * 2.0f;
        float vy = airjoy_data_.left_x * 2.0f;
        float omega = airjoy_data_.right_x * 3.14f * 0.5f;

        if (fabsf(vx) < 0.05f) vx = 0.0f;
        if (fabsf(vy) < 0.05f) vy = 0.0f;
        if (fabsf(omega) < 0.01f) omega = 0.0f;

        this->set_world_speed(vx, vy, omega);
        break;
    }

    case 0x02: // 车体坐标系驱动
    {
        float vx = airjoy_data_.left_y * 2.0f;
        float vy = airjoy_data_.left_x * 2.0f;
        float omega = airjoy_data_.right_x * 3.14f * 0.5f;

        if (fabsf(vx) < 0.05f) vx = 0.0f;
        if (fabsf(vy) < 0.05f) vy = 0.0f;
        if (fabsf(omega) < 0.01f) omega = 0.0f;

        this->set_body_speed(vx, vy, omega);
        break;
    }

    default:
        this->set_body_speed(0.0f, 0.0f, 0.0f);
        break;
    }

    this->update();
}
