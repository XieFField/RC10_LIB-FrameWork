#include "Motor_VESC.h"


VESC_Motor::VESC_Motor(uint32_t id, fdCANbus* bus, float poles)
    : Motor_Base(id, true, bus), poles_(poles)
{
}

void VESC_Motor::updateFeedback(const CanFrame& cf)
{
    id_check_ = cf.ID & 0xFF;

    // 根据VESC标准协议解析 CAN_PACKET_STATUS_1
    // Bytes 0-3: eRPM (int32_t)
    // Bytes 4-5: 电流 (int16_t, 实际电流 * 10)
    // Bytes 6-7: 占空比 (int16_t, 实际占空比 * 1000)
    eRPM_ = static_cast<int32_t>((cf.data[0] << 24) | (cf.data[1] << 16) | (cf.data[2] << 8) | cf.data[3]);
    int16_t current_raw = static_cast<int16_t>((cf.data[4] << 8) | cf.data[5]);
    int16_t duty_raw = static_cast<int16_t>((cf.data[6] << 8) | cf.data[7]);

    // 将解析出的数据转换为标准单位并存入成员变量
    rpm_ = eRPM_to_RPM(eRPM_);
    current_ = static_cast<float>(current_raw) * 100.0f; // 转换为 mA
    duty_ = static_cast<float>(duty_raw) * 0.001f;     // 转换为 -1.0 ~ 1.0
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
    if(duty > 1.0f) duty = 1.0f;
    else if(duty < -1.0f) duty = -1.0f;

    target_duty_ = duty;
    mode_ = SET_DUTY;
    reset_otherParam();
}

std::size_t VESC_Motor::packCommand(CanFrame outFrames[], std::size_t maxFrames)
{
    if(maxFrames < 1)
        return 0; // 无法打包

    CanFrame &cf = outFrames[0];
    static int32_t sendMsgs = 0;
    cf.DLC = 4; //4
    cf.isextended = true;
    std::memset(cf.data, 0, 8);
    switch (mode_)
    {
        case SET_NULL:
        {
            // Do nothing
            break;
        }

        case SET_eRPM:
        {
            if(target_eRPM_ !=0)
            {
                cf.ID = (CAN_CMD_SET_ERPM << 8) | (motor_id_ & 0xFF);
                sendMsgs = static_cast<int32_t>(target_eRPM_);
                cf.data[0] = (sendMsgs >> 24) & 0xFF;
                cf.data[1] = (sendMsgs >> 16) & 0xFF;
                cf.data[2] = (sendMsgs >> 8) & 0xFF;
                cf.data[3] = sendMsgs & 0xFF;
            }
            else
            {
                cf.ID = (CAN_CMD_SET_CURRENT_BRAKE << 8) | (motor_id_ & 0xFF);
                sendMsgs = 70000; //刹车电流
                cf.data[0] = (sendMsgs >> 24) & 0xFF;
                cf.data[1] = (sendMsgs >> 16) & 0xFF;
                cf.data[2] = (sendMsgs >> 8) & 0xFF;
                cf.data[3] = sendMsgs & 0xFF;
            }
            break;
        }    

        case SET_CURRENT:
        {
            cf.ID = (CAN_CMD_SET_CURRENT << 8) | (motor_id_ & 0xFF);
            sendMsgs = static_cast<int32_t>(target_current_); //单位mA
            cf.data[0] = (sendMsgs >> 24) & 0xFF;
            cf.data[1] = (sendMsgs >> 16) & 0xFF;
            cf.data[2] = (sendMsgs >> 8) & 0xFF;
            cf.data[3] = sendMsgs & 0xFF;
            break;
        }

        case SET_DUTY:
        {
            cf.ID = (CAN_CMD_SET_DUTY << 8) | (motor_id_ & 0xFF);
            sendMsgs = static_cast<int32_t>(target_duty_ * 100000.0f); //放大1e5倍
            cf.data[0] = (sendMsgs >> 24) & 0xFF;
            cf.data[1] = (sendMsgs >> 16) & 0xFF;
            cf.data[2] = (sendMsgs >> 8) & 0xFF;
            cf.data[3] = sendMsgs & 0xFF;
            break;
        }

        case SET_POS:
        {
            cf.ID = (CAN_CMD_SET_POS << 8) | (motor_id_ & 0xFF);
            float eAngle = target_totalAngle_ * poles_; //转换为电机轴角度
            sendMsgs = static_cast<int32_t>(eAngle * 1000000.0f); //放大1e6倍
            cf.data[0] = (sendMsgs >> 24) & 0xFF;
            cf.data[1] = (sendMsgs >> 16) & 0xFF;
            cf.data[2] = (sendMsgs >> 8) & 0xFF;
            cf.data[3] = sendMsgs & 0xFF;
            break;
        }
        
        case SET_BRAKE:
        {
            cf.ID = (CAN_CMD_SET_CURRENT_BRAKE << 8) | (motor_id_ & 0xFF);
            sendMsgs = static_cast<int32_t>(target_brake_current_); //单位mA
            cf.data[0] = (sendMsgs >> 24) & 0xFF;
            cf.data[1] = (sendMsgs >> 16) & 0xFF;
            cf.data[2] = (sendMsgs >> 8) & 0xFF;
            cf.data[3] = sendMsgs & 0xFF;
            break;
        }

        default:
        {
            // Do nothing for unsupported modes
            break;
        }
            
    }

    return 1;
}