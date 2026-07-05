#ifndef MOTOR_EL05_H
#define MOTOR_EL05_H

#pragma once

#include <cstddef>
#include <cstdint>

#include "Motor_Base.h"

class EL05_Motor : public Motor_Base
{
public:
    EL05_Motor(uint32_t motor_id, uint32_t master_id, fdCANbus *bus);
    ~EL05_Motor() override = default;

    bool matchesFrame(const CanFrame &cf) const override;
    void updateFeedback(const CanFrame &cf) override;
    void update() override;
    std::size_t packCommand(CanFrame outFrames[], std::size_t maxFrames) override;

    void setTargetTotalAngle(float angle_deg) override;

    void motorEnable();
    void motorDisable();
    void motorSetZero();

    float getAngle() const override;
    float getRPM() const override;
    float getTorque() const;
	uint8_t get_MotorState()
	{
		return mode_state_;
	}

private:
    enum class InitState : uint8_t
    {
        NEED_STOP = 0,
        NEED_RUN_MODE,
        NEED_FEEDBACK_PERIOD,
        NEED_FEEDBACK_ENABLE,
        NEED_SET_ZERO,
        NEED_ENABLE,
        READY
    };

    enum class CommType : uint8_t
    {
        MOTION_CONTROL = 1,
        FEEDBACK = 2,
        ENABLE = 3,
        DISABLE = 4,
        SET_ZERO = 6,
        WRITE_SINGLE_PARAM = 18,
        ACTIVE_REPORT = 24
    };

    static constexpr uint8_t kBroadcastTargetId = 0x7F;
    static constexpr uint16_t kRunModeIndex = 0x7005U;
    static constexpr uint16_t kFeedbackPeriodIndex = 0x7026U;
    static constexpr uint8_t kRunModeMit = 0U;
    static constexpr uint8_t kFeedbackPeriod10ms = 1U;
    static constexpr float kPositionMinRad = -12.57f;
    static constexpr float kPositionMaxRad = 12.57f;
    static constexpr float kVelocityMinRadS = -50.0f;
    static constexpr float kVelocityMaxRadS = 50.0f;
    static constexpr float kTorqueMinNm = -6.0f;
    static constexpr float kTorqueMaxNm = 6.0f;
    static constexpr float kKpMin = 0.0f;
    static constexpr float kKpMax = 500.0f;
    static constexpr float kKdMin = 0.0f;
    static constexpr float kKdMax = 5.0f;
    static constexpr float kDefaultKp = 30.0f;
    static constexpr float kDefaultKd = 1.0f;

    static uint16_t floatToUint16(float value, float min_value, float max_value);
    static float uint16ToFloat(uint16_t value, float min_value, float max_value);
    static uint32_t buildCanId(CommType comm_type, uint16_t data_area2, uint8_t target_id);
    static uint8_t getCommType(uint32_t can_id);
    static uint16_t getDataArea2(uint32_t can_id);
    static uint8_t getTargetId(uint32_t can_id);

    void packMotionControlFrame(CanFrame &frame) const;
    void packEnableFrame(CanFrame &frame) const;
    void packDisableFrame(CanFrame &frame) const;
    void packSetZeroFrame(CanFrame &frame) const;
    void packWriteUint8Frame(CanFrame &frame, uint16_t param_index, uint8_t value) const;
    void packActiveReportFrame(CanFrame &frame, bool enable) const;

    uint32_t master_id_;
    float target_position_rad_ = 0.0f;
    float target_velocity_rad_s_ = 0.0f;
    float target_torque_nm_ = 0.0f;
    float target_kp_ = kDefaultKp;
    float target_kd_ = kDefaultKd;
    float torque_nm_ = 0.0f;
    uint8_t mode_state_ = 0U;
    uint8_t fault_code_ = 0U;

    bool desired_enabled_ = true;
    bool pending_enable_ = false;
    bool pending_disable_ = false;
    bool pending_set_zero_ = false;
    InitState init_state_ = InitState::NEED_STOP;
};

#endif
