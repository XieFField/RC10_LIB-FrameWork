#include "Motor_EL05.h"

#include <cstring>

namespace
{
constexpr float kRadToDeg = 57.2957795f;
constexpr float kRadPerSecToRpm = 9.54929659f;

uint16_t readBeU16(const uint8_t data[8], uint8_t index)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(data[index]) << 8) | data[index + 1U]);
}

void putBeU16(uint8_t data[8], uint8_t index, uint16_t value)
{
    data[index] = static_cast<uint8_t>((value >> 8) & 0xFFU);
    data[index + 1U] = static_cast<uint8_t>(value & 0xFFU);
}

void putLeU16(uint8_t data[8], uint8_t index, uint16_t value)
{
    data[index] = static_cast<uint8_t>(value & 0xFFU);
    data[index + 1U] = static_cast<uint8_t>((value >> 8) & 0xFFU);
}
} // namespace

EL05_Motor::EL05_Motor(uint32_t motor_id, uint32_t master_id, fdCANbus *bus)
    : Motor_Base(motor_id, true, bus),
      master_id_(master_id)
{
}

bool EL05_Motor::matchesFrame(const CanFrame &cf) const
{
    if (!cf.isextended)
    {
        return false;
    }

    const uint8_t comm_type = getCommType(cf.ID);
    if (comm_type == static_cast<uint8_t>(CommType::FEEDBACK))
    {
        return static_cast<uint8_t>(getDataArea2(cf.ID) & 0xFFU) == static_cast<uint8_t>(motor_id_);
    }

    if (comm_type == static_cast<uint8_t>(CommType::ACTIVE_REPORT))
    {
        return getTargetId(cf.ID) == static_cast<uint8_t>(master_id_) &&
               static_cast<uint8_t>(getDataArea2(cf.ID) & 0xFFU) == static_cast<uint8_t>(motor_id_);
    }

    return false;
}

void EL05_Motor::updateFeedback(const CanFrame &cf)
{
    const uint8_t comm_type = getCommType(cf.ID);
    uint8_t feedback_motor_id = 0U;

    if (comm_type == static_cast<uint8_t>(CommType::FEEDBACK))
    {
        feedback_motor_id = static_cast<uint8_t>(getDataArea2(cf.ID) & 0xFFU);
        fault_code_ = static_cast<uint8_t>((cf.ID >> 16) & 0x3FU);
        mode_state_ = static_cast<uint8_t>((cf.ID >> 22) & 0x03U);
    }
    else if (comm_type == static_cast<uint8_t>(CommType::ACTIVE_REPORT))
    {
        if (getTargetId(cf.ID) != static_cast<uint8_t>(master_id_))
        {
            return;
        }
        feedback_motor_id = static_cast<uint8_t>(getDataArea2(cf.ID) & 0xFFU);
    }
    else
    {
        return;
    }

    if (feedback_motor_id != static_cast<uint8_t>(motor_id_))
    {
        return;
    }

    const float position_rad = uint16ToFloat(readBeU16(cf.data, 0U), kPositionMinRad, kPositionMaxRad);
    const float velocity_rad_s = uint16ToFloat(readBeU16(cf.data, 2U), kVelocityMinRadS, kVelocityMaxRadS);
    torque_nm_ = uint16ToFloat(readBeU16(cf.data, 4U), kTorqueMinNm, kTorqueMaxNm);
    temperature_ = static_cast<float>(readBeU16(cf.data, 6U)) / 10.0f;

    angle_ = position_rad * kRadToDeg;
    totalAngle_ = angle_;
    rpm_ = velocity_rad_s * kRadPerSecToRpm;
}

void EL05_Motor::update()
{
}

std::size_t EL05_Motor::packCommand(CanFrame outFrames[], std::size_t maxFrames)
{
    if (maxFrames < 1U)
    {
        return 0U;
    }

    CanFrame &frame = outFrames[0];

    if (pending_disable_)
    {
        packDisableFrame(frame);
        pending_disable_ = false;
        return 1U;
    }

    if (pending_set_zero_)
    {
        packSetZeroFrame(frame);
        pending_set_zero_ = false;
        angle_ = 0.0f;
        totalAngle_ = 0.0f;
        return 1U;
    }

    if (pending_enable_)
    {
        packEnableFrame(frame);
        pending_enable_ = false;
        return 1U;
    }

    switch (init_state_)
    {
    case InitState::NEED_STOP:
        packDisableFrame(frame);
        init_state_ = InitState::NEED_RUN_MODE;
        return 1U;
    case InitState::NEED_RUN_MODE:
        packWriteUint8Frame(frame, kRunModeIndex, kRunModeMit);
        init_state_ = InitState::NEED_FEEDBACK_PERIOD;
        return 1U;
    case InitState::NEED_FEEDBACK_PERIOD:
        packWriteUint8Frame(frame, kFeedbackPeriodIndex, kFeedbackPeriod10ms);
        init_state_ = InitState::NEED_FEEDBACK_ENABLE;
        return 1U;
    case InitState::NEED_FEEDBACK_ENABLE:
        packActiveReportFrame(frame, true);
        init_state_ = InitState::NEED_SET_ZERO;
        return 1U;
    case InitState::NEED_SET_ZERO:
        packSetZeroFrame(frame);
        angle_ = 0.0f;
        totalAngle_ = 0.0f;
        init_state_ = InitState::NEED_ENABLE;
        return 1U;
    case InitState::NEED_ENABLE:
        packEnableFrame(frame);
        init_state_ = InitState::READY;
        return 1U;
    case InitState::READY:
        break;
    }

    if (!desired_enabled_)
    {
        return 0U;
    }

    packMotionControlFrame(frame);
    return 1U;
}

void EL05_Motor::setTargetAngle(float angle_deg)
{
    target_angle_ = angle_deg;
    target_position_rad_ = angle_deg / kRadToDeg;
}

void EL05_Motor::motorEnable()
{
    desired_enabled_ = true;
    pending_disable_ = false;
    pending_enable_ = true;
}

void EL05_Motor::motorDisable()
{
    desired_enabled_ = false;
    pending_enable_ = false;
    pending_disable_ = true;
}

void EL05_Motor::motorSetZero()
{
    pending_set_zero_ = true;
}

float EL05_Motor::getAngle() const
{
    return angle_;
}

float EL05_Motor::getRPM() const
{
    return rpm_;
}

float EL05_Motor::getTorque() const
{
    return torque_nm_;
}

uint16_t EL05_Motor::floatToUint16(float value, float min_value, float max_value)
{
    float clamped = value;
    if (clamped < min_value)
    {
        clamped = min_value;
    }
    if (clamped > max_value)
    {
        clamped = max_value;
    }

    if (max_value <= min_value)
    {
        return 0U;
    }

    return static_cast<uint16_t>((clamped - min_value) * 65535.0f / (max_value - min_value));
}

float EL05_Motor::uint16ToFloat(uint16_t value, float min_value, float max_value)
{
    return (static_cast<float>(value) * (max_value - min_value) / 65535.0f) + min_value;
}

uint32_t EL05_Motor::buildCanId(CommType comm_type, uint16_t data_area2, uint8_t target_id)
{
    return ((static_cast<uint32_t>(comm_type) & 0x1FU) << 24) |
           ((static_cast<uint32_t>(data_area2) & 0xFFFFU) << 8) |
           (static_cast<uint32_t>(target_id) & 0xFFU);
}

uint8_t EL05_Motor::getCommType(uint32_t can_id)
{
    return static_cast<uint8_t>((can_id >> 24) & 0x1FU);
}

uint16_t EL05_Motor::getDataArea2(uint32_t can_id)
{
    return static_cast<uint16_t>((can_id >> 8) & 0xFFFFU);
}

uint8_t EL05_Motor::getTargetId(uint32_t can_id)
{
    return static_cast<uint8_t>(can_id & 0xFFU);
}

void EL05_Motor::packMotionControlFrame(CanFrame &frame) const
{
    std::memset(&frame, 0, sizeof(frame));
    frame.isextended = true;
    frame.DLC = 8;
    frame.ID = buildCanId(CommType::MOTION_CONTROL,
                          floatToUint16(target_torque_nm_, kTorqueMinNm, kTorqueMaxNm),
                          static_cast<uint8_t>(motor_id_));
    putBeU16(frame.data, 0U, floatToUint16(target_position_rad_, kPositionMinRad, kPositionMaxRad));
    putBeU16(frame.data, 2U, floatToUint16(target_velocity_rad_s_, kVelocityMinRadS, kVelocityMaxRadS));
    putBeU16(frame.data, 4U, floatToUint16(target_kp_, kKpMin, kKpMax));
    putBeU16(frame.data, 6U, floatToUint16(target_kd_, kKdMin, kKdMax));
}

void EL05_Motor::packEnableFrame(CanFrame &frame) const
{
    std::memset(&frame, 0, sizeof(frame));
    frame.isextended = true;
    frame.DLC = 8;
    frame.ID = buildCanId(CommType::ENABLE, static_cast<uint16_t>(master_id_), static_cast<uint8_t>(motor_id_));
}

void EL05_Motor::packDisableFrame(CanFrame &frame) const
{
    std::memset(&frame, 0, sizeof(frame));
    frame.isextended = true;
    frame.DLC = 8;
    frame.ID = buildCanId(CommType::DISABLE, static_cast<uint16_t>(master_id_), static_cast<uint8_t>(motor_id_));
}

void EL05_Motor::packSetZeroFrame(CanFrame &frame) const
{
    std::memset(&frame, 0, sizeof(frame));
    frame.isextended = true;
    frame.DLC = 8;
    frame.ID = buildCanId(CommType::SET_ZERO, static_cast<uint16_t>(master_id_), static_cast<uint8_t>(motor_id_));
    frame.data[0] = 1U;
}

void EL05_Motor::packWriteUint8Frame(CanFrame &frame, uint16_t param_index, uint8_t value) const
{
    std::memset(&frame, 0, sizeof(frame));
    frame.isextended = true;
    frame.DLC = 8;
    frame.ID = buildCanId(CommType::WRITE_SINGLE_PARAM, static_cast<uint16_t>(master_id_), static_cast<uint8_t>(motor_id_));
    putLeU16(frame.data, 0U, param_index);
    frame.data[4] = value;
}

void EL05_Motor::packActiveReportFrame(CanFrame &frame, bool enable) const
{
    std::memset(&frame, 0, sizeof(frame));
    frame.isextended = true;
    frame.DLC = 8;
    frame.ID = buildCanId(CommType::ACTIVE_REPORT, static_cast<uint16_t>(master_id_), kBroadcastTargetId);
    frame.data[0] = 0x01U;
    frame.data[1] = 0x02U;
    frame.data[2] = 0x03U;
    frame.data[3] = 0x04U;
    frame.data[4] = 0x05U;
    frame.data[5] = 0x06U;
    frame.data[6] = enable ? 0x01U : 0x00U;
}
