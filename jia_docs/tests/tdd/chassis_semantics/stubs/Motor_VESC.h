#ifndef TEST_TDD_MOTOR_VESC_H
#define TEST_TDD_MOTOR_VESC_H

#include "Motor_DJI.h"

enum VESC_RPM_CONTROL_MODE
{
    VESC_RPM_CONTROL_NATIVE_ERPM = 0,
    VESC_RPM_CONTROL_PID_CURRENT = 1
};

class VESC_Motor : public Motor_Base
{
public:
    VESC_Motor(std::uint32_t id = 0U, fdCANbus *bus = nullptr, float poles = 21.0f)
        : Motor_Base(id, true, bus), poles_(poles)
    {
    }

    std::size_t packCommand(CanFrame[], std::size_t) override
    {
        return 0U;
    }

    void updateFeedback(const CanFrame &) override
    {
    }

    void pid_init(const PID_Param_Config &speed_params, float speed_td_ratio)
    {
        speed_params_ = speed_params;
        speed_td_ratio_ = speed_td_ratio;
    }

    PID_Param_Config get_speed_pid_params() const
    {
        return speed_params_;
    }

    float get_speed_pid_td_ratio() const
    {
        return speed_td_ratio_;
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

    void setFeedbackRpm(float rpm)
    {
        rpm_ = rpm;
    }

private:
    float poles_ = 21.0f;
    PID_Param_Config speed_params_{};
    float speed_td_ratio_ = 0.0f;
    VESC_RPM_CONTROL_MODE rpm_control_mode_ = VESC_RPM_CONTROL_NATIVE_ERPM;
    float speed_pid_current_bias_mA_ = 0.0f;
    float speed_pid_raw_output_current_mA_ = 0.0f;
    float speed_pid_total_output_current_mA_ = 0.0f;
};

#endif
