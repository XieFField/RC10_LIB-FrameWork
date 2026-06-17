#ifndef TEST_TDD_MOTOR_DJI_H
#define TEST_TDD_MOTOR_DJI_H

#include <cstddef>
#include <cstdint>

#include "APP_PID.h"

struct CanFrame
{
};

class fdCANbus
{
};

class Motor_Base
{
public:
    Motor_Base(std::uint32_t, bool, fdCANbus *, bool = true, bool = true) {}
    virtual ~Motor_Base() = default;

    virtual void setTargetRPM(float rpm_set)
    {
        target_rpm_ = rpm_set;
    }

    virtual void setTargetCurrent(float current_set)
    {
        target_current_ = current_set;
    }

    virtual void setTargetAngle(float angle_set)
    {
        target_angle_ = angle_set;
    }

    virtual void setTargetTotalAngle(float total_angle_set)
    {
        target_total_angle_ = total_angle_set;
    }

    virtual void setBrake(float brake_current)
    {
        target_brake_ = brake_current;
    }

    virtual std::size_t packCommand(CanFrame[], std::size_t) = 0;
    virtual void updateFeedback(const CanFrame &) = 0;

    virtual float getRPM() const
    {
        return rpm_;
    }

    virtual float getCurrent() const
    {
        return current_;
    }

    virtual float getAngle() const
    {
        return angle_;
    }

    virtual float getTotalAngle() const
    {
        return total_angle_;
    }

    float getTargetRPM() const
    {
        return target_rpm_;
    }

    float getTargetCurrent() const
    {
        return target_current_;
    }

    float getTargetAngle() const
    {
        return target_angle_;
    }

    float getTargetTotalAngle() const
    {
        return target_total_angle_;
    }

protected:
    float rpm_ = 0.0f;
    float current_ = 0.0f;
    float angle_ = 0.0f;
    float total_angle_ = 0.0f;
    float target_rpm_ = 0.0f;
    float target_current_ = 0.0f;
    float target_angle_ = 0.0f;
    float target_total_angle_ = 0.0f;
    float target_brake_ = 0.0f;
};

class M3508 : public Motor_Base
{
public:
    enum class CommandKind : std::uint8_t
    {
        kNone = 0,
        kCurrent,
        kTotalAngle,
    };

    M3508() : Motor_Base(0U, false, nullptr) {}

    std::size_t packCommand(CanFrame[], std::size_t) override
    {
        return 0;
    }

    void updateFeedback(const CanFrame &) override
    {
    }

    void pid_init(const struct PID_Param_Config &speed_params,
                  float speed_td_ratio,
                  const struct PID_Param_Config &angle_params,
                  float angle_i_separa)
    {
        speed_params_ = speed_params;
        speed_td_ratio_ = speed_td_ratio;
        angle_params_ = angle_params;
        angle_i_separa_ = angle_i_separa;
    }

    struct PID_Param_Config get_speed_pid_params() const
    {
        return speed_params_;
    }

    struct PID_Param_Config get_angle_pid_params() const
    {
        return angle_params_;
    }

    float get_speed_pid_td_ratio() const
    {
        return speed_td_ratio_;
    }

    float get_angle_pid_i_separa_threshold() const
    {
        return angle_i_separa_;
    }

    void reset_speed_pid_state()
    {
        ++reset_speed_pid_state_call_count_;
    }

    CommandKind getLastCommandKind() const
    {
        return last_command_kind_;
    }

    void resetLastCommandObservation()
    {
        last_command_kind_ = CommandKind::kNone;
    }

    void setTargetCurrent(float current_set) override
    {
        Motor_Base::setTargetCurrent(current_set);
        last_command_kind_ = CommandKind::kCurrent;
    }

    void setTargetTotalAngle(float total_angle_set) override
    {
        Motor_Base::setTargetTotalAngle(total_angle_set);
        last_command_kind_ = CommandKind::kTotalAngle;
    }

    std::uint32_t getResetSpeedPidStateCallCount() const
    {
        return reset_speed_pid_state_call_count_;
    }

private:
    struct PID_Param_Config speed_params_{};
    struct PID_Param_Config angle_params_{};
    float speed_td_ratio_ = 0.0f;
    float angle_i_separa_ = 0.0f;
    CommandKind last_command_kind_ = CommandKind::kNone;
    std::uint32_t reset_speed_pid_state_call_count_ = 0U;
};

#endif
