#pragma once

#include <cmath>
#include <cstddef>
#include <type_traits>

#include "doctest.h"
#include "main.h"

#define private public
#include "chassis.h"
#include "Motor_VESC.h"
#undef private

#define EXPECT_TRUE(expr) CHECK((expr))
#define EXPECT_NEAR(actual, expected, tolerance) CHECK_MESSAGE(std::fabs((actual) - (expected)) <= (tolerance), #actual " actual=", (actual), " expected=", (expected), " tolerance=", (tolerance))

// 这些测试需要观察 Chassis 的内部门控、缓存和调试镜像，所以宿主测试仍通过
// `#define private public` 打开内部状态。这里集中放公共测试桩和 helper，分片文件只写
// 各自行为域的 TEST_CASE；新增跨分片复用逻辑时先放这里，避免在多个分片里复制 setup。
namespace chassis_semantics_test
{
using Chassis = jia::FourSteerChassis::Chassis;

// 最小 Motor_Base 测试桩：只记录反馈和目标命令，不模拟 CAN 打包。
// 语义测试关注 Chassis 对电机对象下发了什么，而不是底层总线格式。
class TestMotor : public Motor_Base
{
public:
    TestMotor() : Motor_Base(0U, false, nullptr) {}

    std::size_t packCommand(CanFrame[], std::size_t) override
    {
        return 0U;
    }

    void updateFeedback(const CanFrame &) override
    {
    }

    void setFeedbackRpm(float rpm)
    {
        rpm_ = rpm;
    }

    void setFeedbackCurrent(float current)
    {
        current_ = current;
    }

    void setFeedbackTotalAngleDeg(float total_angle_deg)
    {
        total_angle_ = total_angle_deg;
    }

    float getTargetBrake() const
    {
        return target_brake_;
    }

    float getFeedbackRpm() const
    {
        return rpm_;
    }

    float getFeedbackCurrent() const
    {
        return current_;
    }

    float getFeedbackTotalAngleDeg() const
    {
        return total_angle_;
    }
};

struct DrivePidTuneHarness
{
    Chassis chassis{};
    VESC_Motor drive_vescs[4]{};
    M3508 steer_motors[4]{};
};

// X-Park steer hold 测试同时需要驱动电机和舵电机句柄，单独成组能让每个
// TEST_CASE 一眼看出自己依赖的最小测试环境。
struct XParkSteerHoldHarness
{
    Chassis chassis{};
    VESC_Motor drive_motors[4]{};
    M3508 steer_motors[4]{};
};

// 下面这些函数按“行为 setup / 单步驱动 / frame 构造”三类排列。
// 它们不是生产代码 API，只服务于 host doctest；分片里调用它们时应保持测试意图清晰。
Chassis::SwervePlannerInput makeGatePlannerInput(float steering_error_deg,
                                                 float command_vel_x,
                                                 float command_omega_z,
                                                 float max_residual_speed_m_s);
void emitDebugOutputForHost(Chassis &chassis, bool all_homed);
void runDebugPlannerCycleForHost(Chassis &chassis);
bool runDebugControlCycleForHost(Chassis &chassis);
void configureDebugOutputFamily(Chassis &chassis, unsigned char family_raw);
void configureJustFloatProfile(Chassis &chassis, unsigned char profile_raw);
void configureSingleWheelPayload(Chassis &chassis, unsigned char payload_raw);
void configureDrivePidTuneHarness(DrivePidTuneHarness &harness);
void setDriveRuntimeDerivativeFirst(VESC_Motor &drive_motor, bool derivative_first);
void configureDriveContinuityHarness(Chassis &chassis, VESC_Motor drive_motors[4]);
void configureXParkWheelGeometry(Chassis &chassis);
void configureXParkSteerHoldHarness(XParkSteerHoldHarness &harness);
void setPhotogateStateForWheel(int wheel_idx, bool active_high);
void configureSingleWheelDebugHarness(Chassis &chassis, TestMotor steer_motors[4], VESC_Motor drive_motors[4]);
void configureSingleWheelIsolatedPlannerHarness(Chassis &chassis, TestMotor steer_motors[4], VESC_Motor drive_motors[4]);
void configureSingleWheelIsolatedDirectHarness(Chassis &chassis, TestMotor steer_motors[4], VESC_Motor drive_motors[4]);
void configureSingleWheelDriveVescHarness(Chassis &chassis, TestMotor steer_motors[4], VESC_Motor drive_motors[4]);
bool runHostDebugControlCycle(Chassis &chassis);
void configureDriveZeroStopHarness(Chassis &chassis, VESC_Motor drive_motors[4]);
void setWheelResidualSpeedMps(Chassis &chassis, int wheel_idx, float residual_speed_m_s);
void setWheelPoseToXPark(Chassis &chassis);
void configureHardGateLaunchHarness(Chassis &chassis, VESC_Motor drive_motors[4]);
void configureXParkTriggerHarness(Chassis &chassis);
void configureSteerFaultRecoveryHarness(Chassis &chassis, TestMotor steer_motors[4], VESC_Motor drive_motors[4]);
bool runHostControlCycle(Chassis &chassis);
void configureYawPidTraceHarness(Chassis &chassis);
void finishWheelHomingByEdgeAndAlign(Chassis &chassis, int wheel_idx, TestMotor steer_motors[4]);
Chassis::ActuatorCommandFrame makeDriveOnlyCommandFrame(float drive_omega_rad_s);
Chassis::ActuatorCommandFrame makePerWheelDriveCommandFrame(float drive0, float drive1, float drive2, float drive3);
Chassis::SwervePlannerOutput makeNeutralPlannerOutput();
void advanceDriveZeroStopCycle(Chassis &chassis,
                               const Chassis::SwervePlannerOutput &planner_output,
                               const Chassis::ActuatorCommandFrame &frame,
                               bool all_homed = true);
void advanceDriveZeroStopCycle(Chassis &chassis,
                               float command_drive_rad_s,
                               float residual_speed_m_s,
                               TickType_t time_ms);
float getWheelXParkTargetOaRad(const Chassis &chassis, int wheel_idx);
void setWheelOaAngleRad(Chassis &chassis, int wheel_idx, float oa_rad);
Chassis::ActuatorCommandFrame makeXParkSteerCommandFrame(Chassis &chassis);

} // namespace chassis_semantics_test
