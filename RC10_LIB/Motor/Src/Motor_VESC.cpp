#include "Motor_VESC.h"

VESC_Motor::VESC_Motor(uint32_t id, fdCANbus* bus, float poles)
    : Motor_Base(id, true, bus), poles_(poles)
{
}

void VESC_Motor::pid_init(const PID_Param_Config& speed_params, float speed_tdRatio)
{
    // 本地 PID 速度环参数初始化。默认不启用，需结合 setRpmControlMode 显式切换。
    speed_pid_.set_params(speed_params, speed_tdRatio);
}

void VESC_Motor::update()
{
    if (mode_ == SET_PID_SPEED_CURRENT)
    {
        // 本地速度环模式：
        // 1. 先计算纯 PID 输出
        // 2. 再叠加上层注入的虚拟负载 bias
        // 3. 最终仍统一下发 CURRENT 命令
        speed_pid_raw_output_current_mA_ = speed_pid_.pid_calc(target_rpm_, rpm_);
        speed_pid_total_output_current_mA_ = speed_pid_raw_output_current_mA_ + speed_pid_current_bias_mA_;
        target_current_ = speed_pid_total_output_current_mA_;
        return;
    }

    // 非本地 PID 速度环模式下，不保留 raw/total 观测值，避免调试时误判当前输出来源。
    speed_pid_raw_output_current_mA_ = 0.0f;
    speed_pid_total_output_current_mA_ = 0.0f;
}

void VESC_Motor::updateFeedback(const CanFrame& cf)
{
    id_check_ = static_cast<uint8_t>(cf.ID & 0xFFU);

    // 根据VESC标准协议解析 CAN_PACKET_STATUS_1
    // Bytes 0-3: eRPM (int32_t)
    // Bytes 4-5: 电流 (int16_t, 实际电流 * 10)
    // Bytes 6-7: 占空比 (int16_t, 实际占空比 * 1000)
    eRPM_ = static_cast<int32_t>((cf.data[0] << 24) |
                                 (cf.data[1] << 16) |
                                 (cf.data[2] << 8) |
                                 cf.data[3]);
    int16_t current_raw = static_cast<int16_t>((cf.data[4] << 8) | cf.data[5]);
    int16_t duty_raw = static_cast<int16_t>((cf.data[6] << 8) | cf.data[7]);

    // 将解析出的数据转换为标准单位并存入成员变量
    rpm_ = eRPM_to_RPM(eRPM_);
    current_ = static_cast<float>(current_raw) * 100.0f; // 转换为 mA
    duty_ = static_cast<float>(duty_raw) * 0.001f;       // 转换为 -1.0 ~ 1.0
}

void VESC_Motor::setTargetCurrent(float current_set)
{
    target_current_ = current_set;
    mode_ = SET_CURRENT;
    reset_otherParam();
}

void VESC_Motor::setTargetRPM(float rpm_set)
{
    target_rpm_ = rpm_set;

    if (rpm_control_mode_ == VESC_RPM_CONTROL_PID_CURRENT)
    {
        // 新增路径：RPM 不再直接发 eRPM，而是保留目标值，等待 update() 计算电流。
        mode_ = SET_PID_SPEED_CURRENT;
        reset_otherParam();
        return;
    }

    // 兼容原始行为：继续让 VESC 内部完成 eRPM 闭环。
    mode_ = SET_eRPM;
    target_eRPM_ = RPM_to_eRPM(rpm_set);
    reset_otherParam();
}

void VESC_Motor::setTargetTotalAngle(float totalAngle_set)
{
    target_totalAngle_ = totalAngle_set;
    mode_ = SET_POS;
    reset_otherParam();
}

void VESC_Motor::setBrake(float brake_current)
{
    target_brake_current_ = brake_current;
    mode_ = SET_BRAKE;
    reset_otherParam();
}

void VESC_Motor::setDuty(float duty)
{
    if (duty > 1.0f)
        duty = 1.0f;
    else if (duty < -1.0f)
        duty = -1.0f;

    target_duty_ = duty;
    mode_ = SET_DUTY;
    reset_otherParam();
}

std::size_t VESC_Motor::packCommand(CanFrame outFrames[], std::size_t maxFrames)
{
    if (maxFrames < 1)
        return 0; // 无法打包

    CanFrame& cf = outFrames[0];
    int32_t sendMsgs = 0;
    cf.ID = 0;
    cf.DLC = 4; // VESC 此处使用 4 字节负载
    cf.isextended = true;
    std::memset(cf.data, 0, sizeof(cf.data));

    switch (mode_)
    {
        case SET_NULL:
            // Do nothing
            break;

        case SET_eRPM:
            cf.ID = (CAN_CMD_SET_ERPM << 8) | (motor_id_ & 0xFFU);
            sendMsgs = static_cast<int32_t>(target_eRPM_);
            break;

        case SET_CURRENT:
        case SET_PID_SPEED_CURRENT:
            // 本地 PID 速度环最终也统一落到 CURRENT 命令下发。
            cf.ID = (CAN_CMD_SET_CURRENT << 8) | (motor_id_ & 0xFFU);
            sendMsgs = static_cast<int32_t>(target_current_); // 单位 mA
            break;

        case SET_DUTY:
            cf.ID = (CAN_CMD_SET_DUTY << 8) | (motor_id_ & 0xFFU);
            sendMsgs = static_cast<int32_t>(target_duty_ * 100000.0f); // 放大 1e5 倍
            break;

        case SET_POS:
        {
            cf.ID = (CAN_CMD_SET_POS << 8) | (motor_id_ & 0xFFU);
            float eAngle = target_totalAngle_ * poles_; // 转换为电机轴角度
            sendMsgs = static_cast<int32_t>(eAngle * 1000000.0f); // 放大 1e6 倍
            break;
        }

        case SET_BRAKE:
            cf.ID = (CAN_CMD_SET_CURRENT_BRAKE << 8) | (motor_id_ & 0xFFU);
            sendMsgs = static_cast<int32_t>(target_brake_current_); // 单位 mA
            break;

        default:
            // Do nothing for unsupported modes
            break;
    }

    cf.data[0] = static_cast<uint8_t>((sendMsgs >> 24) & 0xFF);
    cf.data[1] = static_cast<uint8_t>((sendMsgs >> 16) & 0xFF);
    cf.data[2] = static_cast<uint8_t>((sendMsgs >> 8) & 0xFF);
    cf.data[3] = static_cast<uint8_t>(sendMsgs & 0xFF);

    return 1;
}
