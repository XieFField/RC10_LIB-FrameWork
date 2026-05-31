#ifndef TEST_TDD_MOTOR_VESC_H
#define TEST_TDD_MOTOR_VESC_H

#include "Motor_DJI.h"

enum VESC_RPM_CONTROL_MODE
{
    VESC_RPM_CONTROL_NATIVE_ERPM = 0,
    VESC_RPM_CONTROL_PID_CURRENT = 1,
};

class VESC_Motor : public Motor_Base
{
public:
    enum class CommandKind : std::uint8_t
    {
        kNone = 0,
        kCurrent,
        kRpm,
        kBrake,
    };

    VESC_Motor() : Motor_Base(0U, false, nullptr) {}
    VESC_Motor(std::uint32_t, fdCANbus *, std::uint32_t) : Motor_Base(0U, false, nullptr) {}

    std::size_t packCommand(CanFrame[], std::size_t) override
    {
        return 0U;
    }

    void updateFeedback(const CanFrame &) override
    {
    }

    void pid_init(const struct PID_Param_Config &speed_params, float speed_td_ratio)
    {
        speed_params_ = speed_params;
        speed_td_ratio_ = speed_td_ratio;
    }

    void set_speed_pid_derivative_first(bool)
    {
    }

    struct PID_Param_Config get_speed_pid_params() const
    {
        return speed_params_;
    }

    float get_speed_pid_td_ratio() const
    {
        return speed_td_ratio_;
    }

    bool get_speed_pid_derivative_first() const
    {
        return false;
    }

    void setRpmControlMode(VESC_RPM_CONTROL_MODE mode)
    {
        rpm_control_mode_ = mode;
    }

    VESC_RPM_CONTROL_MODE getRpmControlMode() const
    {
        return rpm_control_mode_;
    }

    void setSpeedPidCurrentBias(float bias_current_mA)
    {
        speed_pid_current_bias_mA_ = bias_current_mA;
    }

    float getSpeedPidCurrentBias() const
    {
        return speed_pid_current_bias_mA_;
    }

    float getSpeedPidRawOutputCurrent() const
    {
        return speed_pid_raw_output_current_mA_;
    }

    float getSpeedPidTotalOutputCurrent() const
    {
        return speed_pid_total_output_current_mA_;
    }

    void setPidOutputObservation(float raw_current_mA, float total_current_mA)
    {
        speed_pid_raw_output_current_mA_ = raw_current_mA;
        speed_pid_total_output_current_mA_ = total_current_mA;
    }

    void setFeedbackCurrent(float current)
    {
        current_ = current;
    }

    void setFeedbackRpm(float rpm)
    {
        rpm_ = rpm;
    }

    void setFeedbackTotalAngleDeg(float total_angle_deg)
    {
        total_angle_ = total_angle_deg;
    }

    float getFeedbackCurrent() const
    {
        return current_;
    }

    float getFeedbackRpm() const
    {
        return rpm_;
    }

    float getFeedbackTotalAngleDeg() const
    {
        return total_angle_;
    }

    float getTargetBrake() const
    {
        return target_brake_;
    }

    float getTargetBrakeCurrent() const
    {
        return target_brake_;
    }

    bool isBrakeCommandActive() const
    {
        return last_command_kind_ == CommandKind::kBrake;
    }

    CommandKind getLastCommandKind() const
    {
        return last_command_kind_;
    }

    void resetLastCommandObservation()
    {
        last_command_kind_ = CommandKind::kNone;
    }

    void reset_speed_pid_state()
    {
        ++reset_speed_pid_state_call_count_;
    }

    std::uint32_t getResetSpeedPidStateCallCount() const
    {
        return reset_speed_pid_state_call_count_;
    }

    void setTargetCurrent(float current_set) override
    {
        Motor_Base::setTargetCurrent(current_set);
        target_brake_ = 0.0f;
        last_command_kind_ = CommandKind::kCurrent;
    }

    void setTargetRPM(float rpm_set) override
    {
        Motor_Base::setTargetRPM(rpm_set);
        target_brake_ = 0.0f;
        last_command_kind_ = CommandKind::kRpm;
    }

    void setBrake(float brake_current) override
    {
        Motor_Base::setBrake(brake_current);
        last_command_kind_ = CommandKind::kBrake;
    }

    void reset_controlFrequency(std::uint32_t control_frequency_hz)
    {
        control_frequency_hz_ = control_frequency_hz;
    }

    std::uint32_t getControlFrequency() const
    {
        return control_frequency_hz_;
    }

private:
    struct PID_Param_Config speed_params_{};
    float speed_td_ratio_ = 0.0f;
    VESC_RPM_CONTROL_MODE rpm_control_mode_ = VESC_RPM_CONTROL_NATIVE_ERPM;
    std::uint32_t control_frequency_hz_ = 0U;
    float speed_pid_current_bias_mA_ = 0.0f;
    float speed_pid_raw_output_current_mA_ = 0.0f;
    float speed_pid_total_output_current_mA_ = 0.0f;
    CommandKind last_command_kind_ = CommandKind::kNone;
    std::uint32_t reset_speed_pid_state_call_count_ = 0U;
};

#endif
