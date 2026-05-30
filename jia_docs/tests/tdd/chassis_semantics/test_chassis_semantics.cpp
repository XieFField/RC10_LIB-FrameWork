#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <type_traits>

#include "main.h"

#define private public
#include "chassis.h"
#include "Motor_VESC.h"
#undef private

namespace
{
int g_failures = 0;

void expectTrue(bool condition, const char *expression, int line)
{
    if (!condition)
    {
        std::printf("FAIL line %d: %s\n", line, expression);
        ++g_failures;
    }
}

void expectNear(float actual, float expected, float tolerance, const char *expression, int line)
{
    if (std::fabs(actual - expected) > tolerance)
    {
        std::printf("FAIL line %d: %s actual=%f expected=%f tolerance=%f\n",
                    line,
                    expression,
                    static_cast<double>(actual),
                    static_cast<double>(expected),
                    static_cast<double>(tolerance));
        ++g_failures;
    }
}

#define EXPECT_TRUE(expr) expectTrue((expr), #expr, __LINE__)
#define EXPECT_NEAR(actual, expected, tolerance) expectNear((actual), (expected), (tolerance), #actual, __LINE__)

using Chassis = jia::FourSteerChassis::Chassis;

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

struct SteerSettleResetHarness
{
    Chassis chassis{};
    VESC_Motor drive_motors[4]{};
    M3508 steer_motors[4]{};
};

struct XParkSteerDeadbandHarness
{
    Chassis chassis{};
    VESC_Motor drive_motors[4]{};
    M3508 steer_motors[4]{};
};

void configureDrivePidTuneHarness(DrivePidTuneHarness &harness)
{
    for (int i = 0; i < 4; ++i)
    {
        harness.chassis.wheel_config_[i].steer_motor_h = &harness.steer_motors[i];
        harness.chassis.wheel_config_[i].drive_motor_h = &harness.drive_vescs[i];
        harness.chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        harness.drive_vescs[i].pid_init(PID_Param_Config{}, 0.0f);
        harness.drive_vescs[i].setRpmControlMode(VESC_RPM_CONTROL_PID_CURRENT);
    }
}

void setDriveRuntimeDerivativeFirst(VESC_Motor &drive_motor, bool derivative_first)
{
    drive_motor.set_speed_pid_derivative_first(derivative_first);
}

void testDrivePidEnableEdgeReadsBackRuntimeIntoCleanSharedCache()
{
    DrivePidTuneHarness harness;
    configureDrivePidTuneHarness(harness);

    PID_Param_Config runtime_cfg{};
    runtime_cfg.kp = 6.0f;
    runtime_cfg.ki = 0.4f;
    runtime_cfg.kd = 0.1f;
    runtime_cfg.output_limit = 2300.0f;
    harness.drive_vescs[0].pid_init(runtime_cfg, 0.35f);
    setDriveRuntimeDerivativeFirst(harness.drive_vescs[0], true);

    harness.chassis.debug_pid_tune_.drive_speed_pid_cfg = PID_Param_Config{};
    harness.chassis.debug_pid_tune_.drive_speed_pid_td_ratio = 0.0f;
    harness.chassis.debug_pid_tune_.drive_speed_pid_apply_stamp = 0U;
    harness.chassis.debug_pid_tune_.drive_speed_pid_applied_stamp = 0U;
    harness.chassis.debug_control_.common.enable = false;
    harness.chassis.debug_enable_last_cycle_ = false;

    harness.chassis.syncDebugSteerPidTuneFromRuntimeOnEnableEdge();
    harness.chassis.debug_control_.common.enable = true;
    harness.chassis.syncDebugSteerPidTuneFromRuntimeOnEnableEdge();

    EXPECT_NEAR(harness.chassis.debug_pid_tune_.drive_speed_pid_cfg.kp, runtime_cfg.kp, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.drive_speed_pid_cfg.ki, runtime_cfg.ki, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.drive_speed_pid_cfg.kd, runtime_cfg.kd, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.drive_speed_pid_cfg.output_limit, runtime_cfg.output_limit, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.drive_speed_pid_td_ratio, 0.35f, 1.0e-6f);
    EXPECT_TRUE(!harness.chassis.debug_pid_tune_.drive_speed_pid_derivative_first);
}

void testDrivePidDirtyCacheBlocksRuntimeReadbackOverwrite()
{
    DrivePidTuneHarness harness;
    configureDrivePidTuneHarness(harness);

    PID_Param_Config manual_cfg{};
    manual_cfg.kp = 11.0f;
    manual_cfg.ki = 1.2f;
    manual_cfg.output_limit = 4567.0f;
    PID_Param_Config runtime_cfg{};
    runtime_cfg.kp = 2.0f;
    runtime_cfg.ki = 0.1f;
    runtime_cfg.output_limit = 999.0f;
    harness.drive_vescs[0].pid_init(runtime_cfg, 0.12f);
    setDriveRuntimeDerivativeFirst(harness.drive_vescs[0], false);

    harness.chassis.debug_pid_tune_.drive_speed_pid_cfg = manual_cfg;
    harness.chassis.debug_pid_tune_.drive_speed_pid_td_ratio = 0.8f;
    harness.chassis.debug_pid_tune_.drive_speed_pid_derivative_first = true;
    harness.chassis.debug_pid_tune_.drive_speed_pid_apply_stamp = 9U;
    harness.chassis.debug_pid_tune_.drive_speed_pid_applied_stamp = 7U;

    harness.chassis.syncDebugSteerPidTuneFromRuntime();

    EXPECT_NEAR(harness.chassis.debug_pid_tune_.drive_speed_pid_cfg.kp, manual_cfg.kp, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.drive_speed_pid_cfg.ki, manual_cfg.ki, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.drive_speed_pid_cfg.output_limit, manual_cfg.output_limit, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.drive_speed_pid_td_ratio, 0.8f, 1.0e-6f);
    EXPECT_TRUE(harness.chassis.debug_pid_tune_.drive_speed_pid_derivative_first);
}

void testDrivePidSharedApplyPushesSameParamsToAllVescsAndAlignsAppliedStamp()
{
    DrivePidTuneHarness harness;
    configureDrivePidTuneHarness(harness);

    PID_Param_Config shared_cfg{};
    shared_cfg.kp = 8.0f;
    shared_cfg.ki = 0.6f;
    shared_cfg.kd = 0.02f;
    shared_cfg.deadband = 3.0f;
    shared_cfg.output_limit = 3200.0f;

    harness.chassis.debug_pid_tune_.drive_speed_pid_cfg = shared_cfg;
    harness.chassis.debug_pid_tune_.drive_speed_pid_td_ratio = 0.55f;
    harness.chassis.debug_pid_tune_.drive_speed_pid_derivative_first = true;
    harness.chassis.debug_pid_tune_.drive_speed_pid_apply_stamp = 41U;
    harness.chassis.debug_pid_tune_.drive_speed_pid_applied_stamp = 40U;

    harness.chassis.applyDebugSteerPidRuntimeTuning();

    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(harness.drive_vescs[i].get_speed_pid_params().kp, shared_cfg.kp, 1.0e-6f);
        EXPECT_NEAR(harness.drive_vescs[i].get_speed_pid_params().ki, shared_cfg.ki, 1.0e-6f);
        EXPECT_NEAR(harness.drive_vescs[i].get_speed_pid_params().kd, shared_cfg.kd, 1.0e-6f);
        EXPECT_NEAR(harness.drive_vescs[i].get_speed_pid_params().deadband, shared_cfg.deadband, 1.0e-6f);
        EXPECT_NEAR(harness.drive_vescs[i].get_speed_pid_params().output_limit, shared_cfg.output_limit, 1.0e-6f);
        EXPECT_NEAR(harness.drive_vescs[i].get_speed_pid_td_ratio(), 0.55f, 1.0e-6f);
        EXPECT_TRUE(!harness.drive_vescs[i].get_speed_pid_derivative_first());
    }
    EXPECT_TRUE(harness.chassis.debug_pid_tune_.drive_speed_pid_applied_stamp == 41U);
}

void testDrivePidApplySkipsNullHandlesAndStillUpdatesAppliedStamp()
{
    DrivePidTuneHarness harness;
    configureDrivePidTuneHarness(harness);

    harness.chassis.wheel_config_[1].drive_motor_h = nullptr;
    harness.chassis.wheel_config_[2].drive_motor_h = nullptr;

    PID_Param_Config shared_cfg{};
    shared_cfg.kp = 12.0f;

    harness.chassis.debug_pid_tune_.drive_speed_pid_cfg = shared_cfg;
    harness.chassis.debug_pid_tune_.drive_speed_pid_td_ratio = 0.25f;
    harness.chassis.debug_pid_tune_.drive_speed_pid_derivative_first = false;
    harness.chassis.debug_pid_tune_.drive_speed_pid_apply_stamp = 77U;
    harness.chassis.debug_pid_tune_.drive_speed_pid_applied_stamp = 76U;

    harness.chassis.applyDebugSteerPidRuntimeTuning();

    EXPECT_TRUE(harness.chassis.debug_pid_tune_.drive_speed_pid_applied_stamp == 77U);
    EXPECT_NEAR(harness.drive_vescs[0].get_speed_pid_params().kp, shared_cfg.kp, 1.0e-6f);
    EXPECT_NEAR(harness.drive_vescs[3].get_speed_pid_params().kp, shared_cfg.kp, 1.0e-6f);
    EXPECT_TRUE(!harness.drive_vescs[0].get_speed_pid_derivative_first());
    EXPECT_TRUE(!harness.drive_vescs[3].get_speed_pid_derivative_first());
}

void configureDriveContinuityHarness(Chassis &chassis, VESC_Motor drive_motors[4]);
void configureXParkWheelGeometry(Chassis &chassis);
void configureSteerSettleResetHarness(SteerSettleResetHarness &harness);

void setPhotogateStateForWheel(int wheel_idx, bool active_high)
{
    static GPIO_TypeDef *const kPorts[4] = {
        kPHOTOGATE_1_GPIO_Port,
        kPHOTOGATE_2_GPIO_Port,
        kPHOTOGATE_3_GPIO_Port,
        kPHOTOGATE_4_GPIO_Port,
    };
    static const unsigned short kPins[4] = {
        kPHOTOGATE_1_Pin,
        kPHOTOGATE_2_Pin,
        kPHOTOGATE_3_Pin,
        kPHOTOGATE_4_Pin,
    };

    testHostSetPhotogate(kPorts[wheel_idx], kPins[wheel_idx], active_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void testExternalCommandMapsToInternalBodyAxesWithoutChangingOmega()
{
    Chassis::ExternalCommand command{};
    command.coord = Chassis::Coordinate::kBody;
    command.vel_x = 1.25f;
    command.vel_y = -2.50f;
    command.omega_z = 0.75f;

    const Chassis::BodyCommand body = Chassis::mapExternalCommandToBody(command);

    EXPECT_NEAR(body.vel_x, -2.50f, 1.0e-6f);
    EXPECT_NEAR(body.vel_y, -1.25f, 1.0e-6f);
    EXPECT_NEAR(body.omega_z, 0.75f, 1.0e-6f);
}

void testPlannerAxisNormalizationDoesNotDependOnDebugStyleOmegaFlip()
{
    Chassis::BodyCommand command{};
    command.vel_x = 0.80f;
    command.vel_y = -1.10f;
    command.omega_z = -0.45f;

    const Chassis::BodyCommand planner = Chassis::normalizeBodyCommandForPlanner(command);

    EXPECT_NEAR(planner.vel_x, -0.80f, 1.0e-6f);
    EXPECT_NEAR(planner.vel_y, 1.10f, 1.0e-6f);
    EXPECT_NEAR(planner.omega_z, -0.45f, 1.0e-6f);
}

void testSteerGeometryUsesSignedInstallationAngleOnly()
{
    Chassis::SteerCalibration calibration{};
    calibration.theta_oa_to_owi_rad = jia::degToRadF32(-90.0f);
    calibration.homing_runtime_zero_offset_rad = jia::degToRadF32(30.0f);
    calibration.steer_motor_sign = -1.0f;
    calibration.drive_motor_sign = 1.0f;

    const float target_oa_total_rad = jia::degToRadF32(45.0f);
    const float corrected_local_total_rad = Chassis::mapOaTotalToCorrectedLocalTotal(target_oa_total_rad, calibration);

    EXPECT_NEAR(jia::radToDegF32(corrected_local_total_rad), 135.0f, 1.0e-4f);

    const float round_trip_oa_total_rad = Chassis::mapCorrectedLocalTotalToOaTotal(corrected_local_total_rad, calibration);
    EXPECT_NEAR(round_trip_oa_total_rad, target_oa_total_rad, 1.0e-6f);
}

void testRuntimeZeroAndMotorPolarityOnlyAffectMotorLocalConversion()
{
    Chassis::SteerCalibration calibration{};
    calibration.theta_oa_to_owi_rad = jia::degToRadF32(15.0f);
    calibration.homing_runtime_zero_offset_rad = jia::degToRadF32(-20.0f);
    calibration.steer_motor_sign = -1.0f;
    calibration.drive_motor_sign = -1.0f;

    const float corrected_local_total_rad = jia::degToRadF32(100.0f);
    const float raw_motor_total_rad = Chassis::mapCorrectedLocalTotalToRawSteerMotorTotal(corrected_local_total_rad, calibration);
    const float round_trip_corrected_local_rad = Chassis::mapRawSteerMotorTotalToCorrectedLocalTotal(raw_motor_total_rad, calibration);

    EXPECT_NEAR(jia::radToDegF32(raw_motor_total_rad), -120.0f, 1.0e-4f);
    EXPECT_NEAR(round_trip_corrected_local_rad, corrected_local_total_rad, 1.0e-6f);

    const float wheel_omega_rad_s = 6.0f;
    const float drive_rpm = Chassis::mapWheelOmegaToDriveMotorRpm(wheel_omega_rad_s, calibration);
    const float round_trip_wheel_omega_rad_s = Chassis::mapDriveMotorRpmToWheelOmega(drive_rpm, calibration);

    EXPECT_NEAR(drive_rpm, jia::radsToRpmF32(-6.0f), 1.0e-4f);
    EXPECT_NEAR(round_trip_wheel_omega_rad_s, wheel_omega_rad_s, 1.0e-6f);
}

void testSteerMotorSignAndRuntimeZeroOffsetStayAsIndependentMappingStages()
{
    const float raw_motor_total_rad = jia::degToRadF32(75.0f);
    const float signed_local_total_rad = Chassis::mapRawSteerMotorTotalToSignedLocalTotal(raw_motor_total_rad, -1.0f);
    const float corrected_local_total_rad = Chassis::applyHomingRuntimeZeroOffset(signed_local_total_rad, jia::degToRadF32(20.0f));

    EXPECT_NEAR(jia::radToDegF32(signed_local_total_rad), -75.0f, 1.0e-4f);
    EXPECT_NEAR(jia::radToDegF32(corrected_local_total_rad), -55.0f, 1.0e-4f);
    EXPECT_NEAR(Chassis::removeHomingRuntimeZeroOffset(corrected_local_total_rad, jia::degToRadF32(20.0f)), signed_local_total_rad, 1.0e-6f);
    EXPECT_NEAR(Chassis::mapSignedLocalTotalToRawSteerMotorTotal(signed_local_total_rad, -1.0f), raw_motor_total_rad, 1.0e-6f);
}

void testTelemetrySnapshotKeepsTargetAndActualYawSemanticsSeparate()
{
    Chassis::TelemetryChassisState target{};
    target.vel_x = 1.2f;
    target.vel_y = -0.4f;
    target.omega_z = 0.5f;
    target.yaw_rad = 0.25f;

    Chassis::TelemetryChassisState actual{};
    actual.vel_x = 0.8f;
    actual.vel_y = 0.3f;
    actual.omega_z = -0.2f;
    actual.yaw_rad = -0.75f;

    Chassis::TelemetryWheelPose wheel_pose[4]{};
    wheel_pose[0].pos_x_m = 0.39f;
    wheel_pose[0].pos_y_m = 0.40f;

    float target_drive[4] = {5.0f, 0.0f, 0.0f, 0.0f};
    float actual_drive[4] = {4.0f, 0.0f, 0.0f, 0.0f};
    float target_steer[4] = {0.3f, 0.0f, 0.0f, 0.0f};
    float actual_steer[4] = {0.1f, 0.0f, 0.0f, 0.0f};

    const Chassis::TelemetrySnapshot snapshot = Chassis::makeTelemetrySnapshot(
        true,
        target,
        actual,
        wheel_pose,
        target_drive,
        actual_drive,
        target_steer,
        actual_steer);

    EXPECT_TRUE(snapshot.homing_all_ready);
    EXPECT_NEAR(snapshot.target.yaw_rad, 0.25f, 1.0e-6f);
    EXPECT_NEAR(snapshot.actual.yaw_rad, -0.75f, 1.0e-6f);
    EXPECT_NEAR(snapshot.wheels[0].target_velocity_x_m_s, 1.2f + 0.5f * 0.40f, 1.0e-6f);
    EXPECT_NEAR(snapshot.wheels[0].target_velocity_y_m_s, -0.4f - 0.5f * 0.39f, 1.0e-6f);
    EXPECT_NEAR(snapshot.wheels[0].actual_velocity_x_m_s, 0.8f + (-0.2f) * 0.40f, 1.0e-6f);
    EXPECT_NEAR(snapshot.wheels[0].actual_velocity_y_m_s, 0.3f - (-0.2f) * 0.39f, 1.0e-6f);
}

void testTelemetrySnapshotPreservesWheelTargetsWithoutModeDependentReinterpretation()
{
    Chassis::TelemetryChassisState target{};
    Chassis::TelemetryChassisState actual{};
    Chassis::TelemetryWheelPose wheel_pose[4]{};
    float target_drive[4] = {1.0f, -2.0f, 3.0f, -4.0f};
    float actual_drive[4] = {-1.5f, 2.5f, -3.5f, 4.5f};
    float target_steer[4] = {0.1f, 0.2f, 0.3f, 0.4f};
    float actual_steer[4] = {-0.1f, -0.2f, -0.3f, -0.4f};

    const Chassis::TelemetrySnapshot snapshot = Chassis::makeTelemetrySnapshot(
        false,
        target,
        actual,
        wheel_pose,
        target_drive,
        actual_drive,
        target_steer,
        actual_steer);

    EXPECT_TRUE(!snapshot.homing_all_ready);
    EXPECT_NEAR(snapshot.wheels[0].target_drive_omega_rad_s, 1.0f, 1.0e-6f);
    EXPECT_NEAR(snapshot.wheels[1].target_drive_omega_rad_s, -2.0f, 1.0e-6f);
    EXPECT_NEAR(snapshot.wheels[2].actual_drive_omega_rad_s, -3.5f, 1.0e-6f);
    EXPECT_NEAR(snapshot.wheels[3].actual_drive_omega_rad_s, 4.5f, 1.0e-6f);
    EXPECT_NEAR(snapshot.wheels[0].target_steer_oa_rad, 0.1f, 1.0e-6f);
    EXPECT_NEAR(snapshot.wheels[3].actual_steer_oa_rad, -0.4f, 1.0e-6f);
}

void testDriveMotorHardwarePolarityMapsCurrentWithoutLeakingIntoGeometry()
{
    Chassis::SteerCalibration calibration{};
    calibration.drive_motor_sign = -1.0f;

    EXPECT_NEAR(Chassis::mapWheelCurrentToDriveMotorCurrent(3000.0f, calibration), -3000.0f, 1.0e-6f);
    EXPECT_NEAR(Chassis::mapWheelCurrentToDriveMotorCurrent(-1200.0f, calibration), 1200.0f, 1.0e-6f);

    calibration.drive_motor_sign = 1.0f;
    EXPECT_NEAR(Chassis::mapWheelCurrentToDriveMotorCurrent(3000.0f, calibration), 3000.0f, 1.0e-6f);
}

void testDefaultDriveMotorPolarityMatchesPositiveBodyXHardwareLayout()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    Chassis::InitConfig init_cfg{};

    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_omega_limit_ = false;

    for (int i = 0; i < 4; ++i)
    {
        init_cfg.steer_motor_h[i] = &steer_motors[i];
        init_cfg.drive_motor_h[i] = &drive_motors[i];
    }
    chassis.init(init_cfg);

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].homing_enabled = false;
        chassis.wheel_config_[i].homing_state = Chassis::HomingState::kReady;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad =
            chassis.mapWheelOaTotalToCorrectedLocal(chassis.wheel_config_[i], 0.0f);
    }

    Chassis::Data command{};
    command.vel_x = 1.0f;
    command.vel_y = 0.0f;
    command.omega_z = 0.0f;

    chassis.computeModuleCommands(command);
    chassis.applyModuleCommands(true);

    for (int i = 0; i < 4; ++i)
    {
        EXPECT_TRUE(chassis.wheel_config_[i].target_drive_omega_rad_s > 0.0f);
    }
    EXPECT_TRUE(drive_motors[0].getTargetRPM() < 0.0f);
    EXPECT_TRUE(drive_motors[1].getTargetRPM() > 0.0f);
    EXPECT_TRUE(drive_motors[2].getTargetRPM() < 0.0f);
    EXPECT_TRUE(drive_motors[3].getTargetRPM() > 0.0f);
}

void testPlannerInputNormalizationKeepsWorldBodyAndSteerOnlySemanticsExplicit()
{
    Chassis::PlannerInputCommand input{};
    input.vel_x = 1.0f;
    input.vel_y = 0.0f;
    input.omega_z = 0.2f;
    input.rot_z = 1.5f;
    input.is_world_speed_mode = true;

    const Chassis::PlannerInputSnapshot snapshot = Chassis::makePlannerInputSnapshot(input, 0.0f);

    EXPECT_NEAR(snapshot.target.vel_x, -1.0f, 1.0e-6f);
    EXPECT_NEAR(snapshot.target.vel_y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(snapshot.target.omega_z, 0.2f, 1.0e-6f);
    EXPECT_NEAR(snapshot.target.rot_z, 1.5f, 1.0e-6f);

    input.is_steer_only_mode = true;
    const Chassis::PlannerInputSnapshot steer_only_snapshot = Chassis::makePlannerInputSnapshot(input, 0.0f);

    EXPECT_NEAR(steer_only_snapshot.target.vel_x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(steer_only_snapshot.target.vel_y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(steer_only_snapshot.target.omega_z, 0.0f, 1.0e-6f);
    EXPECT_NEAR(steer_only_snapshot.target.rot_z, 1.5f, 1.0e-6f);
}

void testNormalizedBodyCommandKeepsDebugAndApiRoutesSemanticallyAligned()
{
    Chassis::PlannerInputCommand input{};
    input.vel_x = 1.0f;
    input.vel_y = -0.4f;
    input.omega_z = 0.3f;
    input.rot_z = 0.8f;
    input.is_world_speed_mode = true;

    const float yaw_rad = jia::degToRadF32(90.0f);
    const Chassis::NormalizedBodyCommand api_command =
        Chassis::makeNormalizedBodyCommand(input, yaw_rad, Chassis::CommandInputSource::kApi);
    const Chassis::NormalizedBodyCommand debug_command =
        Chassis::makeNormalizedBodyCommand(input, yaw_rad, Chassis::CommandInputSource::kDebugTarget);

    EXPECT_TRUE(api_command.source == Chassis::CommandInputSource::kApi);
    EXPECT_TRUE(debug_command.source == Chassis::CommandInputSource::kDebugTarget);
    EXPECT_NEAR(api_command.body.vel_x, debug_command.body.vel_x, 1.0e-6f);
    EXPECT_NEAR(api_command.body.vel_y, debug_command.body.vel_y, 1.0e-6f);
    EXPECT_NEAR(api_command.body.omega_z, debug_command.body.omega_z, 1.0e-6f);
    EXPECT_NEAR(api_command.body.vel_x, 0.4f, 1.0e-5f);
    EXPECT_NEAR(api_command.body.vel_y, 1.0f, 1.0e-5f);
    EXPECT_NEAR(api_command.body.omega_z, 0.3f, 1.0e-6f);
    EXPECT_NEAR(api_command.rot_z, 0.8f, 1.0e-6f);
}

void testHomingRuntimeZeroOffsetOnlyDependsOnEdgeGeometryAndRawMotorAngle()
{
    Chassis::SteerCalibration calibration{};
    calibration.theta_oa_to_owi_rad = jia::degToRadF32(90.0f);
    calibration.homing_runtime_zero_offset_rad = 0.0f;
    calibration.steer_motor_sign = 1.0f;

    const float edge_mech_oa_rad = jia::degToRadF32(150.0f);
    const float raw_motor_total_rad = jia::degToRadF32(40.0f);
    const float homing_zero_offset_rad = jia::degToRadF32(-30.0f);

    const float runtime_zero_offset_rad = Chassis::computeHomingRuntimeZeroOffset(
        edge_mech_oa_rad,
        raw_motor_total_rad,
        homing_zero_offset_rad,
        calibration);

    EXPECT_NEAR(jia::radToDegF32(runtime_zero_offset_rad), -10.0f, 1.0e-4f);
}

void testDebugRouteClassificationSeparatesInputInjectionFromModuleOverride()
{
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(false, 1) == Chassis::DebugControlRoute::kDisabled);
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(true, 1) == Chassis::DebugControlRoute::kTargetInjection);
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(true, 8) == Chassis::DebugControlRoute::kTargetInjection);
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(true, 20) == Chassis::DebugControlRoute::kTargetInjection);
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(true, 30) == Chassis::DebugControlRoute::kModuleOverride);
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(true, 31) == Chassis::DebugControlRoute::kTargetInjection);
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(true, 32) == Chassis::DebugControlRoute::kTargetInjection);
}

void testDebugModuleOverrideRouteSeparatesRetiredMode20AlignHomingAndSingleWheelModes()
{
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(1) == Chassis::DebugModuleOverrideRoute::kNone);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(20) == Chassis::DebugModuleOverrideRoute::kNone);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(21) == Chassis::DebugModuleOverrideRoute::kAlignForward);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(22) == Chassis::DebugModuleOverrideRoute::kHomingObserve);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(30) == Chassis::DebugModuleOverrideRoute::kSingleWheelIsolated);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(31) == Chassis::DebugModuleOverrideRoute::kNone);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(32) == Chassis::DebugModuleOverrideRoute::kNone);
}

void testResolveDebugModeFallsBackToTorqueFreeWhenMode20IsRetired()
{
    Chassis chassis;
    EXPECT_TRUE(chassis.resolveDebugMode(20) == Chassis::DebugMode::kTorqueFree);
    EXPECT_TRUE(chassis.resolveDebugMode(255) == Chassis::DebugMode::kTorqueFree);
    EXPECT_TRUE(chassis.resolveDebugMode(30) == Chassis::DebugMode::kSingleWheelIsolated);
    EXPECT_TRUE(chassis.resolveDebugMode(31) == Chassis::DebugMode::kTorqueFree);
    EXPECT_TRUE(chassis.resolveDebugMode(32) == Chassis::DebugMode::kTorqueFree);
}

void testLowSpeedDriveSuppressionCanBeDisabled()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
    }

    Chassis::Data command{};
    command.vel_x = 1.0f;

    const Chassis::SwervePlannerOutput planner_output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));

    EXPECT_NEAR(planner_output.low_speed_suppression_scale[0], 1.0f, 1.0e-6f);
    EXPECT_NEAR(planner_output.final_drive_omega_rad_s[0], planner_output.projected_drive_omega_rad_s[0], 1.0e-6f);
}

void testProjectedDriveUsesReachableSteerInsteadOfIdealVector()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = true;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.max_steer_rate_rad_s_ = 1.0f;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = jia::degToRadF32(90.0f);
        chassis.last_steer_rate_cmd_rad_s_[i] = 0.0f;
        chassis.last_drive_omega_cmd_rad_s_[i] = 0.0f;
    }

    Chassis::Data command{};
    command.vel_x = 1.0f;
    command.vel_y = 0.0f;
    command.omega_z = 0.0f;

    const Chassis::SwervePlannerInput planner_input = chassis.makeSwervePlannerInput(command);
    const Chassis::SwervePlannerOutput planner_output = chassis.planSwerveModules(planner_input);

    EXPECT_NEAR(planner_output.ideal_drive_omega_rad_s[0], 20.0f, 1.0e-4f);
    EXPECT_TRUE(std::fabs(planner_output.projected_drive_omega_rad_s[0]) < 1.0f);
    EXPECT_TRUE(std::fabs(planner_output.projected_drive_omega_rad_s[0]) <
                std::fabs(planner_output.ideal_drive_omega_rad_s[0]));
    EXPECT_TRUE(std::fabs(planner_output.final_drive_omega_rad_s[0]) <=
                std::fabs(planner_output.projected_drive_omega_rad_s[0]) + 1.0e-6f);
}

void testHighSpeedDriveSuppressionTightensAndReleasesThroughPlannerOutput()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.high_speed_drive_suppression.dir_err_enter_deg = 5.0f;
    chassis.runtime_strategy_cfg_.high_speed_drive_suppression.dir_err_exit_deg = 2.0f;
    chassis.runtime_strategy_cfg_.high_speed_drive_suppression.eta_lock_s = 0.05f;
    chassis.runtime_strategy_cfg_.high_speed_drive_suppression.eta_release_s = 0.01f;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = true;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.max_steer_rate_rad_s_ = 1.0f;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = jia::degToRadF32(90.0f);
        chassis.last_steer_rate_cmd_rad_s_[i] = 0.0f;
        chassis.last_drive_omega_cmd_rad_s_[i] = 0.0f;
    }

    Chassis::Data command{};
    command.vel_x = 1.0f;
    command.vel_y = 0.0f;
    command.omega_z = 0.0f;

    const Chassis::SwervePlannerOutput gated_output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));
    EXPECT_TRUE(gated_output.high_speed_suppression_active);
    EXPECT_TRUE(gated_output.high_speed_suppression_scale < 1.0f);

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
        chassis.last_steer_rate_cmd_rad_s_[i] = 0.0f;
    }

    const Chassis::SwervePlannerOutput released_output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));
    EXPECT_TRUE(!released_output.high_speed_suppression_active);
    EXPECT_TRUE(released_output.high_speed_suppression_scale > gated_output.high_speed_suppression_scale);
}

void testLowSpeedDriveSuppressionDoesNotReenterInsideNearZeroHysteresisBand()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.min_scale = 0.0f;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg = 1.0f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.10f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.15f;

    const float steering_errors_rad[4] = {
        jia::degToRadF32(10.0f), jia::degToRadF32(10.0f), jia::degToRadF32(10.0f), jia::degToRadF32(10.0f)};
    float gate_scales[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    chassis.computeLowSpeedDriveSuppressionScales(
        makeGatePlannerInput(10.0f, 0.0f, 0.0f, 0.20f), steering_errors_rad, gate_scales);
    chassis.computeLowSpeedDriveSuppressionScales(
        makeGatePlannerInput(10.0f, 0.0f, 0.0f, 0.12f), steering_errors_rad, gate_scales);

    EXPECT_NEAR(gate_scales[0], 1.0f, 1.0e-6f);
    EXPECT_NEAR(gate_scales[3], 1.0f, 1.0e-6f);
}

void testHighSpeedDriveSuppressionWaitsUntilNearZeroExitBeforeEnabling()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.10f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.15f;
    chassis.runtime_strategy_cfg_.high_speed_drive_suppression.dir_err_enter_deg = 5.0f;
    chassis.runtime_strategy_cfg_.high_speed_drive_suppression.dir_err_exit_deg = 2.0f;
    chassis.runtime_strategy_cfg_.high_speed_drive_suppression.eta_lock_s = 0.05f;
    chassis.runtime_strategy_cfg_.high_speed_drive_suppression.eta_release_s = 0.01f;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = true;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.max_steer_rate_rad_s_ = 1.0f;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = jia::degToRadF32(90.0f);
        chassis.last_steer_rate_cmd_rad_s_[i] = 0.0f;
        chassis.last_drive_omega_cmd_rad_s_[i] = 0.0f;
    }

    Chassis::Data slow_command{};
    slow_command.vel_x = 0.12f;
    const Chassis::SwervePlannerOutput slow_output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(slow_command));
    EXPECT_TRUE(!slow_output.high_speed_suppression_active);
    EXPECT_NEAR(slow_output.high_speed_suppression_scale, 1.0f, 1.0e-6f);

    Chassis::Data fast_command{};
    fast_command.vel_x = 0.20f;
    const Chassis::SwervePlannerOutput fast_output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(fast_command));
    EXPECT_TRUE(fast_output.high_speed_suppression_active);
    EXPECT_TRUE(fast_output.high_speed_suppression_scale < 1.0f);
}

void configureSingleWheelDebugHarness(Chassis &chassis, TestMotor steer_motors[4], VESC_Motor drive_motors[4])
{
    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].steer_motor_h = &steer_motors[i];
        chassis.wheel_config_[i].drive_motor_h = &drive_motors[i];
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].target_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].target_steer_motor_total_angle_rad = 0.0f;
    }
}

void configureSingleWheelIsolatedPlannerHarness(Chassis &chassis, TestMotor steer_motors[4], VESC_Motor drive_motors[4])
{
    configureSingleWheelDebugHarness(chassis, steer_motors, drive_motors);
    configureDriveContinuityHarness(chassis, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kSCurve;
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg = 8.0f;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.min_scale = 0.2f;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;

    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 30U;
    chassis.debug_control_.common.control_wheel_index = 1U;
    chassis.debug_control_.common.observe_wheel_index = 1U;
    chassis.input_target_data_.mode = Chassis::Mode::kBodySpeedMode;
    chassis.input_target_data_.vel_x = 0.4f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;
    chassis.input_hwt_rot_z_ = 0.0f;
    chassis.current_mode_flag_.is_world_speed_mode = false;
    chassis.current_mode_flag_.is_lock_now_rot_z = false;
    chassis.current_mode_flag_.is_lock_to_rot_z = false;
    chassis.current_mode_flag_.is_wheel_torque_free = false;
}

void configureSingleWheelIsolatedDirectHarness(Chassis &chassis, TestMotor steer_motors[4], VESC_Motor drive_motors[4])
{
    configureSingleWheelDebugHarness(chassis, steer_motors, drive_motors);
    configureDriveContinuityHarness(chassis, drive_motors);

    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kSCurve;
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;

    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 30U;
    chassis.debug_control_.common.control_wheel_index = 1U;
    chassis.debug_control_.common.observe_wheel_index = 1U;
    chassis.input_target_data_.mode = Chassis::Mode::kBodySpeedMode;
    chassis.debug_control_.single_wheel.estop = false;
    chassis.debug_control_.single_wheel.input_deadzone = 0.0f;

    chassis.debug_control_.single_wheel.steer.enable = true;
    chassis.debug_control_.single_wheel.steer.input_mode_raw = static_cast<unsigned char>(Chassis::DirectAxisInputMode::kRcContinuous);
    chassis.debug_control_.single_wheel.steer.input_axis_raw = static_cast<unsigned char>(Chassis::SingleWheelInputAxis::kLeftX);
    chassis.debug_control_.single_wheel.steer.invert_input = false;
    chassis.debug_control_.single_wheel.steer.command_type_raw = static_cast<unsigned char>(Chassis::DirectSteerCommandType::kSingleTurnDeg);
    chassis.debug_control_.single_wheel.steer.command_value = 0.0f;
    chassis.debug_control_.single_wheel.steer.command_limit = 90.0f;
    chassis.debug_control_.single_wheel.steer.step_threshold = 0.5f;
    chassis.debug_control_.single_wheel.steer.step_value = 45.0f;
    chassis.debug_control_.single_wheel.steer.planner_mode_raw = static_cast<unsigned char>(Chassis::SingleWheelPlannerMode::kOff);
    chassis.debug_control_.single_wheel.steer.scurve.acc_acc = 60.0f;
    chassis.debug_control_.single_wheel.steer.scurve.acc_dec = 60.0f;
    chassis.debug_control_.single_wheel.steer.scurve.jerk_acc = 400.0f;
    chassis.debug_control_.single_wheel.steer.scurve.jerk_dec = 400.0f;
    chassis.debug_control_.single_wheel.steer.scurve.settle_vel_eps = 1.0e-4f;
    chassis.debug_control_.single_wheel.steer.scurve.settle_acc_eps = 0.05f;
    chassis.debug_control_.single_wheel.steer.trapezoid.acc = 120.0f;
    chassis.debug_control_.single_wheel.steer.trapezoid.dec = 120.0f;

    chassis.debug_control_.single_wheel.drive.enable = true;
    chassis.debug_control_.single_wheel.drive.input_mode_raw = static_cast<unsigned char>(Chassis::DirectAxisInputMode::kRcContinuous);
    chassis.debug_control_.single_wheel.drive.input_axis_raw = static_cast<unsigned char>(Chassis::SingleWheelInputAxis::kRightX);
    chassis.debug_control_.single_wheel.drive.invert_input = false;
    chassis.debug_control_.single_wheel.drive.command_type_raw = static_cast<unsigned char>(Chassis::DirectDriveCommandType::kRpm);
    chassis.debug_control_.single_wheel.drive.command_value = 0.0f;
    chassis.debug_control_.single_wheel.drive.command_limit = 1000.0f;
    chassis.debug_control_.single_wheel.drive.step_threshold = 0.5f;
    chassis.debug_control_.single_wheel.drive.step_value = 200.0f;
    chassis.debug_control_.single_wheel.drive.planner_mode_raw = static_cast<unsigned char>(Chassis::SingleWheelPlannerMode::kOff);
    chassis.debug_control_.single_wheel.drive.scurve.acc_acc = 2.0f;
    chassis.debug_control_.single_wheel.drive.scurve.acc_dec = 3.0f;
    chassis.debug_control_.single_wheel.drive.scurve.jerk_acc = 20.0f;
    chassis.debug_control_.single_wheel.drive.scurve.jerk_dec = 30.0f;
    chassis.debug_control_.single_wheel.drive.scurve.settle_vel_eps = 1.0e-4f;
    chassis.debug_control_.single_wheel.drive.scurve.settle_acc_eps = 0.05f;
    chassis.debug_control_.single_wheel.drive.trapezoid.acc = 2.0f;
    chassis.debug_control_.single_wheel.drive.trapezoid.dec = 3.0f;
}

void configureSingleWheelDriveVescHarness(Chassis &chassis, TestMotor steer_motors[4], VESC_Motor drive_motors[4])
{
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = true;
    chassis.debug_control_.common.control_wheel_index = 1U;
    chassis.debug_control_.common.observe_wheel_index = 1U;
    chassis.debug_control_.single_wheel.drive.enable = true;
    chassis.debug_control_.single_wheel.estop = false;
    chassis.debug_control_.single_wheel.drive.command_type_raw = static_cast<unsigned char>(Chassis::DirectDriveCommandType::kRpm);
    chassis.debug_control_.single_wheel.drive.input_mode_raw = static_cast<unsigned char>(Chassis::DirectAxisInputMode::kCached);
    chassis.debug_control_.single_wheel.drive.command_value = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        drive_motors[i].setRpmControlMode(VESC_RPM_CONTROL_PID_CURRENT);
        chassis.wheel_config_[i].homing_state = Chassis::HomingState::kReady;
    }
}

bool runHostDebugControlCycle(Chassis &chassis)
{
    chassis.isDebugMode();
    chassis.setModeFlag();
    chassis.resolvePlannerTargetData();
    chassis.refreshActuatorLimitState();
    chassis.updatePlannedMotionData();
    chassis.updateWheelFeedback();

    bool all_homed = true;
    for (int i = 0; i < 4; ++i)
    {
        if (!chassis.updateHomingState(chassis.wheel_config_[i]))
        {
            all_homed = false;
        }
    }

    if (!all_homed && chassis.input_target_data_.zero_current_all)
    {
        chassis.input_target_data_.zero_current_all = false;
    }
    chassis.homing_start_request_ = false;

    if (chassis.applyDebugModuleOverride(all_homed))
    {
        chassis.updateCurrentData(all_homed);
        chassis.refreshDebugMirror(all_homed);
        chassis.last_planned_data_ = chassis.planned_data_;
        return all_homed;
    }

    chassis.computeModuleCommands(chassis.planned_data_);
    chassis.applyModuleCommands(all_homed);
    chassis.updateCurrentData(all_homed);
    chassis.refreshDebugMirror(all_homed);
    chassis.last_planned_data_ = chassis.planned_data_;
    return all_homed;
}

void testMode30SingleWheelDirectJoystickIsolationOnlyLetsTargetWheelMove()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);

    chassis.airjoy_data_.left_x = 0.5f;
    chassis.airjoy_data_.right_x = -0.25f;
    EXPECT_TRUE(runHostDebugControlCycle(chassis));

    EXPECT_TRUE(chassis.debug_control_.common.mode_raw == 30U);
    EXPECT_NEAR(steer_motors[1].getTargetTotalAngle(), 45.0f, 1.0e-4f);
    EXPECT_NEAR(drive_motors[1].getTargetRPM(), -250.0f, 1.0e-4f);
    EXPECT_NEAR(chassis.wheel_config_[0].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[2].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[3].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
    EXPECT_NEAR(steer_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(steer_motors[2].getTargetCurrent(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(steer_motors[3].getTargetCurrent(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[2].getTargetCurrent(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[3].getTargetCurrent(), 0.0f, 1.0e-6f);
}

void testMode30SingleWheelDirectDriveCanUseSCurveShaping()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);

    chassis.runtime_strategy_cfg_.manual_trans_acc_acc_ = 999.0f;
    chassis.runtime_strategy_cfg_.manual_trans_acc_dec_ = 999.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_ = 999.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_dec_ = 999.0f;
    chassis.debug_control_.single_wheel.drive.command_limit = 600.0f;
    chassis.debug_control_.single_wheel.drive.planner_mode_raw = static_cast<unsigned char>(Chassis::SingleWheelPlannerMode::kSCurve);
    chassis.debug_control_.single_wheel.drive.scurve.acc_acc = 2.0f;
    chassis.debug_control_.single_wheel.drive.scurve.acc_dec = 3.0f;
    chassis.debug_control_.single_wheel.drive.scurve.jerk_acc = 20.0f;
    chassis.debug_control_.single_wheel.drive.scurve.jerk_dec = 30.0f;
    chassis.airjoy_data_.right_x = 1.0f;

    EXPECT_TRUE(runHostDebugControlCycle(chassis));

    EXPECT_TRUE(chassis.debug_control_.common.mode_raw == 30U);
    EXPECT_TRUE(std::fabs(chassis.wheel_config_[1].target_drive_omega_rad_s) > 0.0f);
    EXPECT_TRUE(std::fabs(chassis.wheel_config_[1].target_drive_omega_rad_s) < jia::rpmToRadsF32(600.0f));
    EXPECT_NEAR(std::fabs(chassis.wheel_config_[1].target_drive_omega_rad_s),
                chassis.debug_control_.single_wheel.drive.scurve.jerk_acc * Chassis::period_ * Chassis::period_ / chassis.runtime_strategy_cfg_.wheel_radius_m_,
                1.0e-6f);
}

void testMode30CommonWheelIndexAliasCanDirectlySelectTargetWheel()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.common.control_wheel_index = 3U;
    chassis.debug_control_.common.observe_wheel_index = 2U;
    chassis.airjoy_data_.left_x = -1.0f;
    chassis.airjoy_data_.right_x = 0.2f;

    EXPECT_TRUE(runHostDebugControlCycle(chassis));

    EXPECT_TRUE(chassis.debug_control_.common.control_wheel_index == 3U);
    EXPECT_TRUE(chassis.debug_control_.common.observe_wheel_index == 2U);
    EXPECT_NEAR(steer_motors[3].getTargetTotalAngle(), -90.0f, 1.0e-4f);
    EXPECT_NEAR(drive_motors[3].getTargetRPM(), 200.0f, 1.0e-4f);
    EXPECT_NEAR(steer_motors[1].getTargetTotalAngle(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[1].getTargetRPM(), 0.0f, 1.0e-6f);
}

void testMode30DirectDriveIgnoresAllHomedGateForTargetWheel()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);

    chassis.airjoy_data_.right_x = 0.5f;
    chassis.computeSingleWheelIsolatedCommandsMode30(1U);
    chassis.applySingleWheelIsolationFilter(Chassis::DebugMode::kSingleWheelIsolated, 1U, false);

    EXPECT_NEAR(drive_motors[1].getTargetRPM(), 500.0f, 1.0e-4f);
    EXPECT_TRUE(chassis.wheel_config_[1].target_drive_omega_rad_s > 0.0f);
}

void testMode30SingleWheelRemovedModes31And32DoNotEnterModuleOverride()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.common.mode_raw = 31U;
    EXPECT_TRUE(!chassis.applyDebugModuleOverride(true));
    EXPECT_NEAR(steer_motors[1].getTargetTotalAngle(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[1].getTargetRPM(), 0.0f, 1.0e-6f);

    chassis.debug_control_.common.mode_raw = 32U;
    EXPECT_TRUE(!chassis.applyDebugModuleOverride(true));
    EXPECT_NEAR(steer_motors[1].getTargetTotalAngle(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[1].getTargetRPM(), 0.0f, 1.0e-6f);
}

void testMode30SingleWheelUsesConfiguredAxesAndInversion()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.single_wheel.steer.input_axis_raw = static_cast<unsigned char>(Chassis::SingleWheelInputAxis::kLeftY);
    chassis.debug_control_.single_wheel.steer.invert_input = true;
    chassis.debug_control_.single_wheel.drive.input_axis_raw = static_cast<unsigned char>(Chassis::SingleWheelInputAxis::kRightY);
    chassis.debug_control_.single_wheel.drive.invert_input = true;
    chassis.airjoy_data_.left_y = -0.4f;
    chassis.airjoy_data_.right_y = -0.25f;

    EXPECT_TRUE(runHostDebugControlCycle(chassis));

    EXPECT_NEAR(steer_motors[1].getTargetTotalAngle(), 36.0f, 1.0e-4f);
    EXPECT_NEAR(drive_motors[1].getTargetRPM(), 250.0f, 1.0e-4f);
    EXPECT_TRUE(chassis.debug_control_.single_wheel.steer.command_value > 0.0f);
    EXPECT_TRUE(chassis.debug_control_.single_wheel.drive.command_value > 0.0f);
}

void testMode30SingleWheelSharedDeadzoneSuppressesContinuousAndStepInputs()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.single_wheel.input_deadzone = 0.3f;
    chassis.debug_control_.single_wheel.drive.input_mode_raw = static_cast<unsigned char>(Chassis::DirectAxisInputMode::kRcStep);
    chassis.debug_control_.single_wheel.drive.step_threshold = 0.2f;
    chassis.airjoy_data_.left_x = 0.2f;
    chassis.airjoy_data_.right_x = 0.25f;

    EXPECT_TRUE(runHostDebugControlCycle(chassis));

    EXPECT_NEAR(steer_motors[1].getTargetTotalAngle(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[1].getTargetRPM(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.debug_control_.single_wheel.steer.command_value, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.debug_control_.single_wheel.drive.command_value, 0.0f, 1.0e-6f);
}

void testMode30SingleWheelAxisEnablesAndEstopGateOutputs()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.single_wheel.steer.enable = false;
    chassis.debug_control_.single_wheel.drive.enable = true;
    chassis.debug_control_.single_wheel.steer.input_mode_raw = static_cast<unsigned char>(Chassis::DirectAxisInputMode::kCached);
    chassis.debug_control_.single_wheel.drive.input_mode_raw = static_cast<unsigned char>(Chassis::DirectAxisInputMode::kCached);
    chassis.debug_control_.single_wheel.steer.command_value = 30.0f;
    chassis.debug_control_.single_wheel.drive.command_value = 90.0f;

    EXPECT_TRUE(runHostDebugControlCycle(chassis));
    EXPECT_NEAR(steer_motors[1].getTargetCurrent(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[1].getTargetRPM(), 90.0f, 1.0e-4f);

    chassis.debug_control_.single_wheel.estop = true;
    chassis.debug_control_.single_wheel.steer.enable = true;
    chassis.debug_control_.single_wheel.drive.enable = true;
    chassis.debug_control_.single_wheel.steer.command_value = 60.0f;
    chassis.debug_control_.single_wheel.drive.command_value = 180.0f;

    EXPECT_TRUE(runHostDebugControlCycle(chassis));
    EXPECT_NEAR(steer_motors[1].getTargetCurrent(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[1].getTargetCurrent(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[1].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
}

void testMode30SingleWheelSteerPlannerSupportsSCurveAndTrapezoid()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);

    chassis.runtime_strategy_cfg_.manual_trans_acc_acc_ = 999.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_ = 999.0f;
    chassis.debug_control_.single_wheel.steer.command_type_raw = static_cast<unsigned char>(Chassis::DirectSteerCommandType::kRpm);
    chassis.debug_control_.single_wheel.steer.command_limit = 240.0f;
    chassis.debug_control_.single_wheel.steer.planner_mode_raw = static_cast<unsigned char>(Chassis::SingleWheelPlannerMode::kSCurve);
    chassis.debug_control_.single_wheel.steer.scurve.acc_acc = 80.0f;
    chassis.debug_control_.single_wheel.steer.scurve.acc_dec = 80.0f;
    chassis.debug_control_.single_wheel.steer.scurve.jerk_acc = 500.0f;
    chassis.debug_control_.single_wheel.steer.scurve.jerk_dec = 500.0f;
    chassis.airjoy_data_.left_x = 1.0f;
    EXPECT_TRUE(runHostDebugControlCycle(chassis));
    EXPECT_TRUE(std::fabs(steer_motors[1].getTargetRPM()) > 0.0f);
    EXPECT_TRUE(std::fabs(steer_motors[1].getTargetRPM()) < 240.0f);

    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);
    chassis.runtime_strategy_cfg_.max_acc_xy_acc_ = 999.0f;
    chassis.debug_control_.single_wheel.steer.command_type_raw = static_cast<unsigned char>(Chassis::DirectSteerCommandType::kRpm);
    chassis.debug_control_.single_wheel.steer.command_limit = 240.0f;
    chassis.debug_control_.single_wheel.steer.planner_mode_raw = static_cast<unsigned char>(Chassis::SingleWheelPlannerMode::kTrapezoid);
    chassis.debug_control_.single_wheel.steer.trapezoid.acc = 120.0f;
    chassis.debug_control_.single_wheel.steer.trapezoid.dec = 120.0f;
    chassis.airjoy_data_.left_x = 1.0f;
    EXPECT_TRUE(runHostDebugControlCycle(chassis));
    EXPECT_NEAR(std::fabs(steer_motors[1].getTargetRPM()),
                chassis.debug_control_.single_wheel.steer.trapezoid.acc * Chassis::period_,
                1.0e-6f);
}

void testMode30SingleWheelDrivePlannerSupportsTrapezoidAndIgnoresGlobalManualProfileParams()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);

    chassis.runtime_strategy_cfg_.max_acc_xy_acc_ = 999.0f;
    chassis.runtime_strategy_cfg_.max_acc_xy_dec_ = 999.0f;
    chassis.debug_control_.single_wheel.drive.command_limit = 600.0f;
    chassis.debug_control_.single_wheel.drive.planner_mode_raw = static_cast<unsigned char>(Chassis::SingleWheelPlannerMode::kTrapezoid);
    chassis.debug_control_.single_wheel.drive.trapezoid.acc = 2.0f;
    chassis.debug_control_.single_wheel.drive.trapezoid.dec = 3.0f;
    chassis.airjoy_data_.right_x = 1.0f;

    EXPECT_TRUE(runHostDebugControlCycle(chassis));

    const float expected_drive_step_rad_s =
        chassis.debug_control_.single_wheel.drive.trapezoid.acc * Chassis::period_ / chassis.runtime_strategy_cfg_.wheel_radius_m_;
    EXPECT_NEAR(std::fabs(chassis.wheel_config_[1].target_drive_omega_rad_s), expected_drive_step_rad_s, 1.0e-6f);
    EXPECT_NEAR(std::fabs(drive_motors[1].getTargetRPM()), jia::radsToRpmF32(expected_drive_step_rad_s), 1.0e-4f);
}

void testRefreshDebugMirrorPublishesHomingDiagnosticsForObserveMode()
{
    Chassis chassis;
    chassis.wheel_config_[0].homing_state = Chassis::HomingState::kSearch;
    chassis.wheel_config_[0].homing_elapsed_s = 1.25f;
    chassis.wheel_config_[0].homing_zero_valid = false;
    chassis.wheel_config_[0].homing_last_edge_is_falling = true;
    chassis.wheel_config_[0].homing_runtime_zero_offset_rad = jia::degToRadF32(18.0f);
    chassis.wheel_config_[0].corrected_steer_motor_total_angle_rad = jia::degToRadF32(12.0f);
    chassis.wheel_config_[0].target_steer_motor_total_angle_rad = jia::degToRadF32(12.0f);
    chassis.wheel_config_[0].target_drive_omega_rad_s = 4.0f;

    chassis.debug_control_.common.observe_wheel_index = 0U;
    chassis.applyHomingObserveDebugOverride();
    chassis.refreshDebugMirror(false);

    EXPECT_NEAR(chassis.wheel_config_[0].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[0].target_steer_motor_total_angle_rad,
                chassis.wheel_config_[0].corrected_steer_motor_total_angle_rad,
                1.0e-6f);
    EXPECT_TRUE(chassis.debug_mirror_.homing_state[0] == static_cast<unsigned char>(Chassis::HomingState::kSearch));
    EXPECT_TRUE(chassis.debug_mirror_.homing_last_edge_is_falling[0]);
    EXPECT_NEAR(chassis.debug_mirror_.homing_runtime_zero_offset_deg[0], 18.0f, 1.0e-4f);
}

void testMode30SingleWheelNonTargetCommandTypesBypassPlanner()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.single_wheel.steer.command_type_raw = static_cast<unsigned char>(Chassis::DirectSteerCommandType::kCurrent);
    chassis.debug_control_.single_wheel.steer.command_limit = 1200.0f;
    chassis.debug_control_.single_wheel.steer.planner_mode_raw = static_cast<unsigned char>(Chassis::SingleWheelPlannerMode::kSCurve);
    chassis.debug_control_.single_wheel.drive.command_type_raw = static_cast<unsigned char>(Chassis::DirectDriveCommandType::kBrake);
    chassis.debug_control_.single_wheel.drive.command_limit = 800.0f;
    chassis.debug_control_.single_wheel.drive.planner_mode_raw = static_cast<unsigned char>(Chassis::SingleWheelPlannerMode::kTrapezoid);
    chassis.airjoy_data_.left_x = 1.0f;
    chassis.airjoy_data_.right_x = 1.0f;

    EXPECT_TRUE(runHostDebugControlCycle(chassis));

    EXPECT_NEAR(steer_motors[1].getTargetCurrent(), 1200.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[1].getTargetCurrent(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[1].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
}

void testMode30SingleWheelPlannerStateResetsWhenWheelAndPlannerModeChange()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.single_wheel.drive.command_limit = 600.0f;
    chassis.debug_control_.single_wheel.drive.planner_mode_raw = static_cast<unsigned char>(Chassis::SingleWheelPlannerMode::kTrapezoid);
    chassis.debug_control_.single_wheel.drive.trapezoid.acc = 2.0f;
    chassis.debug_control_.single_wheel.drive.trapezoid.dec = 2.0f;
    chassis.airjoy_data_.right_x = 1.0f;

    EXPECT_TRUE(runHostDebugControlCycle(chassis));
    const float first_wheel_step_rpm = drive_motors[1].getTargetRPM();
    EXPECT_TRUE(first_wheel_step_rpm > 0.0f);

    chassis.debug_control_.common.control_wheel_index = 2U;
    EXPECT_TRUE(runHostDebugControlCycle(chassis));
    EXPECT_NEAR(drive_motors[2].getTargetRPM(), first_wheel_step_rpm, 1.0e-4f);

    chassis.debug_control_.single_wheel.drive.planner_mode_raw = static_cast<unsigned char>(Chassis::SingleWheelPlannerMode::kOff);
    EXPECT_TRUE(runHostDebugControlCycle(chassis));
    EXPECT_NEAR(drive_motors[2].getTargetRPM(), 600.0f, 1.0e-4f);
}

void testJustFloatSingleWheelProfileUsesObserveWheelIndex()
{
    Chassis chassis;
    TestMotor steer_motors[4];

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].steer_motor_h = &steer_motors[i];
    }

    steer_motors[1].setTargetCurrent(111.0f);
    steer_motors[2].setTargetCurrent(222.0f);
    steer_motors[1].setTargetRPM(11.0f);
    steer_motors[2].setTargetRPM(22.0f);

    testHostResetJustFloatCapture();
    configureDebugOutputFamily(chassis, 2U);
    configureJustFloatProfile(chassis, 1U);
    configureSingleWheelPayload(chassis, 0U);
    chassis.debug_output_.justfloat.single_wheel.period_ms = 0U;
    chassis.debug_output_runtime_.justfloat.single_wheel.last_ms = 0U;
    chassis.time_ms_ = 25U;
    chassis.debug_control_.common.control_wheel_index = 2U;
    chassis.debug_control_.common.observe_wheel_index = 1U;

    emitDebugOutputForHost(chassis, true);

    EXPECT_TRUE(g_test_justfloat_capture.called);
    EXPECT_TRUE(g_test_justfloat_capture.size == 9U);
    EXPECT_NEAR(g_test_justfloat_capture.values[1], 111.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[3], 11.0f, 1.0e-6f);
}

void testJustFloatSingleWheelPayloadUsesObserveWheelIndex()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].steer_motor_h = &steer_motors[i];
        chassis.wheel_config_[i].drive_motor_h = &drive_motors[i];
    }

    steer_motors[0].setTargetCurrent(10.0f);
    drive_motors[0].setTargetCurrent(20.0f);
    steer_motors[2].setTargetCurrent(110.0f);
    drive_motors[2].setTargetCurrent(220.0f);
    steer_motors[0].setTargetRPM(30.0f);
    drive_motors[0].setTargetRPM(40.0f);
    steer_motors[2].setTargetRPM(130.0f);
    drive_motors[2].setTargetRPM(240.0f);

    testHostResetJustFloatCapture();
    configureDebugOutputFamily(chassis, 2U);
    configureJustFloatProfile(chassis, 1U);
    configureSingleWheelPayload(chassis, 1U);
    chassis.debug_output_.justfloat.single_wheel.period_ms = 0U;
    chassis.debug_output_runtime_.justfloat.single_wheel.last_ms = 0U;
    chassis.time_ms_ = 40U;
    chassis.debug_control_.common.control_wheel_index = 0U;
    chassis.debug_control_.common.observe_wheel_index = 2U;

    emitDebugOutputForHost(chassis, true);

    EXPECT_TRUE(g_test_justfloat_capture.called);
    EXPECT_TRUE(g_test_justfloat_capture.size == 17U);
    EXPECT_NEAR(g_test_justfloat_capture.values[1], 110.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[3], 130.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[9], 220.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[11], 240.0f, 1.0e-6f);
}

void testJustFloatSingleWheelObserveIndexFallsBackToZeroWhenOutOfRange()
{
    Chassis chassis;
    TestMotor steer_motors[4];

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].steer_motor_h = &steer_motors[i];
    }

    steer_motors[0].setTargetCurrent(321.0f);
    steer_motors[3].setTargetCurrent(999.0f);

    testHostResetJustFloatCapture();
    configureDebugOutputFamily(chassis, 2U);
    configureJustFloatProfile(chassis, 1U);
    configureSingleWheelPayload(chassis, 0U);
    chassis.debug_output_.justfloat.single_wheel.period_ms = 0U;
    chassis.debug_output_runtime_.justfloat.single_wheel.last_ms = 0U;
    chassis.time_ms_ = 60U;
    chassis.debug_control_.common.observe_wheel_index = 9U;

    emitDebugOutputForHost(chassis, true);

    EXPECT_TRUE(g_test_justfloat_capture.called);
    EXPECT_NEAR(g_test_justfloat_capture.values[1], 321.0f, 1.0e-6f);
}

void testJustFloatSingleWheelDriveOnlyPayloadUsesObserveWheelIndex()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].steer_motor_h = &steer_motors[i];
        chassis.wheel_config_[i].drive_motor_h = &drive_motors[i];
    }

    steer_motors[1].setTargetCurrent(111.0f);
    steer_motors[1].setTargetRPM(11.0f);
    drive_motors[1].setTargetCurrent(222.0f);
    drive_motors[1].setTargetRPM(22.0f);
    drive_motors[1].setFeedbackCurrent(333.0f);
    drive_motors[1].setFeedbackRpm(44.0f);
    drive_motors[1].setTargetTotalAngle(555.0f);
    drive_motors[1].setFeedbackTotalAngleDeg(666.0f);

    testHostResetJustFloatCapture();
    configureDebugOutputFamily(chassis, 2U);
    configureJustFloatProfile(chassis, 1U);
    configureSingleWheelPayload(chassis, 2U);
    chassis.debug_output_.justfloat.single_wheel.period_ms = 0U;
    chassis.debug_output_runtime_.justfloat.single_wheel.last_ms = 0U;
    chassis.time_ms_ = 80U;
    chassis.debug_control_.common.control_wheel_index = 0U;
    chassis.debug_control_.common.observe_wheel_index = 1U;

    emitDebugOutputForHost(chassis, true);

    EXPECT_TRUE(g_test_justfloat_capture.called);
    EXPECT_TRUE(g_test_justfloat_capture.size == 9U);
    EXPECT_NEAR(g_test_justfloat_capture.values[1], 222.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[2], 333.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[3], 22.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[4], 44.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[7], 555.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[8], 666.0f, 1.0e-6f);
}

void testJustFloatSingleWheelDriveOnlyObserveIndexFallsBackToZeroWhenOutOfRange()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].drive_motor_h = &drive_motors[i];
    }

    drive_motors[0].setTargetCurrent(432.0f);
    drive_motors[0].setTargetRPM(54.0f);
    drive_motors[3].setTargetCurrent(999.0f);
    drive_motors[3].setTargetRPM(88.0f);

    testHostResetJustFloatCapture();
    configureDebugOutputFamily(chassis, 2U);
    configureJustFloatProfile(chassis, 1U);
    configureSingleWheelPayload(chassis, 2U);
    chassis.debug_output_.justfloat.single_wheel.period_ms = 0U;
    chassis.debug_output_runtime_.justfloat.single_wheel.last_ms = 0U;
    chassis.time_ms_ = 90U;
    chassis.debug_control_.common.observe_wheel_index = 7U;

    emitDebugOutputForHost(chassis, true);

    EXPECT_TRUE(g_test_justfloat_capture.called);
    EXPECT_TRUE(g_test_justfloat_capture.size == 9U);
    EXPECT_NEAR(g_test_justfloat_capture.values[1], 432.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[3], 54.0f, 1.0e-6f);
}

void testMode30SingleWheelDriveStepGeneratorOverridesManualInput()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelDriveVescHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.single_wheel.drive.command_value = 100.0f;
    chassis.debug_drive_step_generator_[1].enable = true;
    chassis.debug_drive_step_generator_[1].step_target_rpm = 420.0f;
    chassis.debug_drive_step_generator_[1].hold_ms = 3.0f;
    chassis.debug_drive_step_generator_[1].rest_ms = 2.0f;
    chassis.debug_drive_step_generator_[1].alternate_sign = true;
    chassis.debug_drive_step_generator_[1].start_positive = true;
    chassis.debug_drive_step_generator_[1].auto_restart = true;

    EXPECT_TRUE(runHostDebugControlCycle(chassis));
    EXPECT_NEAR(drive_motors[1].getTargetRPM(), 420.0f, 1.0e-4f);
    EXPECT_NEAR(chassis.debug_drive_load_trace_.target_rpm, 420.0f, 1.0e-4f);
    EXPECT_NEAR(chassis.debug_drive_load_trace_.step_phase, 1.0f, 1.0e-6f);
    EXPECT_TRUE(chassis.debug_drive_load_trace_.stepgen_enable > 0.5f);
}

void testDriveVirtualLoadInjectsBiasOnlyForSelectedWheelAndPidMode()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelDriveVescHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.single_wheel.drive.command_value = 180.0f;
    chassis.debug_drive_virtual_load_[1].enable = true;
    chassis.debug_drive_virtual_load_[1].delta_j_current_per_rad_s2 = 10.0f;
    chassis.debug_drive_virtual_load_[1].delta_b_current_per_rad_s = 20.0f;
    chassis.debug_drive_virtual_load_[1].coulomb_current_mA = 30.0f;
    chassis.debug_drive_virtual_load_[1].coulomb_sign_vel_eps_rad_s = 0.1f;
    chassis.debug_drive_virtual_load_[1].bias_current_limit_mA = 1000.0f;
    drive_motors[1].setFeedbackRpm(jia::radsToRpmF32(2.0f));
    chassis.last_drive_feedback_omega_rad_s_[1] = 1.0f;
    drive_motors[1].setPidOutputObservation(90.0f, 90.0f);

    EXPECT_TRUE(runHostDebugControlCycle(chassis));
    EXPECT_NEAR(drive_motors[1].getSpeedPidCurrentBias(), -1000.0f, 1.0e-4f);
    EXPECT_NEAR(chassis.debug_drive_load_trace_.omega_rad_s, 2.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.debug_drive_load_trace_.alpha_est_rad_s2, 1000.0f, 1.0e-3f);
    EXPECT_NEAR(chassis.debug_drive_load_trace_.j_term_mA, -10000.0f, 1.0e-3f);
    EXPECT_NEAR(chassis.debug_drive_load_trace_.b_term_mA, -40.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.debug_drive_load_trace_.tc_term_mA, -30.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.debug_drive_load_trace_.load_bias_current_mA, -1000.0f, 1.0e-6f);

    drive_motors[1].setRpmControlMode(VESC_RPM_CONTROL_NATIVE_ERPM);
    EXPECT_TRUE(runHostDebugControlCycle(chassis));
    EXPECT_NEAR(drive_motors[1].getSpeedPidCurrentBias(), 0.0f, 1.0e-6f);
}

void testMode30DriveVirtualLoadIgnoresAllHomedGateForTargetWheel()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelDriveVescHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.single_wheel.drive.command_value = 180.0f;
    chassis.debug_drive_virtual_load_[1].enable = true;
    chassis.debug_drive_virtual_load_[1].delta_j_current_per_rad_s2 = 10.0f;
    chassis.debug_drive_virtual_load_[1].delta_b_current_per_rad_s = 20.0f;
    chassis.debug_drive_virtual_load_[1].coulomb_current_mA = 30.0f;
    chassis.debug_drive_virtual_load_[1].coulomb_sign_vel_eps_rad_s = 0.1f;
    chassis.debug_drive_virtual_load_[1].bias_current_limit_mA = 1000.0f;
    chassis.wheel_config_[1].corrected_drive_omega_rad_s = 2.0f;
    chassis.last_drive_feedback_omega_rad_s_[1] = 1.0f;
    drive_motors[1].setPidOutputObservation(90.0f, 90.0f);

    // mode30 目标轮直控本来就应绕过全车 homing gate，虚拟负载也应该跟着这条语义走。
    chassis.computeSingleWheelIsolatedCommandsMode30(1U, false);

    EXPECT_NEAR(drive_motors[1].getSpeedPidCurrentBias(), -1000.0f, 1.0e-4f);
    EXPECT_TRUE(chassis.debug_drive_load_trace_.virtual_load_enable > 0.5f);
}

void testMode30DriveVirtualLoadIgnoresOtherWheelSteerFaultForTargetWheel()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelDriveVescHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.single_wheel.drive.command_value = 180.0f;
    chassis.debug_drive_virtual_load_[1].enable = true;
    chassis.debug_drive_virtual_load_[1].delta_j_current_per_rad_s2 = 10.0f;
    chassis.debug_drive_virtual_load_[1].delta_b_current_per_rad_s = 20.0f;
    chassis.debug_drive_virtual_load_[1].coulomb_current_mA = 30.0f;
    chassis.debug_drive_virtual_load_[1].coulomb_sign_vel_eps_rad_s = 0.1f;
    chassis.debug_drive_virtual_load_[1].bias_current_limit_mA = 1000.0f;
    chassis.wheel_config_[1].corrected_drive_omega_rad_s = 2.0f;
    chassis.last_drive_feedback_omega_rad_s_[1] = 1.0f;
    drive_motors[1].setPidOutputObservation(90.0f, 90.0f);
    chassis.wheel_config_[0].steer_fault_state = Chassis::SteerFaultState::kRecovering;

    // 单轮虚拟负载只服务目标轮速度环，不该被其他轮的 steer fault 全车级短路。
    chassis.computeSingleWheelIsolatedCommandsMode30(1U, true);

    EXPECT_NEAR(drive_motors[1].getSpeedPidCurrentBias(), -1000.0f, 1.0e-4f);
    EXPECT_TRUE(chassis.debug_drive_load_trace_.virtual_load_enable > 0.5f);
}

void testJustFloatDrivePidLoadProfileEmitsFixed16ChannelPayload()
{
    Chassis chassis;
    testHostResetJustFloatCapture();
    configureDebugOutputFamily(chassis, 2U);
    configureJustFloatProfile(chassis, 3U);
    chassis.debug_output_.justfloat.drive_pid_load.period_ms = 0U;
    chassis.debug_output_runtime_.justfloat.drive_pid_load.last_ms = 0U;
    chassis.time_ms_ = 120U;
    chassis.debug_drive_load_trace_.observe_wheel_idx = 1.0f;
    chassis.debug_drive_load_trace_.target_rpm = 300.0f;
    chassis.debug_drive_load_trace_.feedback_rpm = 250.0f;
    chassis.debug_drive_load_trace_.total_current_cmd_mA = 600.0f;
    chassis.debug_drive_load_trace_.pid_current_mA = 550.0f;
    chassis.debug_drive_load_trace_.load_bias_current_mA = 50.0f;
    chassis.debug_drive_load_trace_.j_term_mA = 10.0f;
    chassis.debug_drive_load_trace_.b_term_mA = 20.0f;
    chassis.debug_drive_load_trace_.tc_term_mA = 20.0f;
    chassis.debug_drive_load_trace_.omega_rad_s = 4.0f;
    chassis.debug_drive_load_trace_.alpha_est_rad_s2 = 6.0f;
    chassis.debug_drive_load_trace_.step_phase = 2.0f;
    chassis.debug_drive_load_trace_.virtual_load_enable = 1.0f;
    chassis.debug_drive_load_trace_.stepgen_enable = 0.0f;

    emitDebugOutputForHost(chassis, true);

    EXPECT_TRUE(g_test_justfloat_capture.called);
    EXPECT_TRUE(g_test_justfloat_capture.size == 16U);
    EXPECT_NEAR(g_test_justfloat_capture.values[0], 0.12f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[1], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[2], 300.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[4], 600.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[12], 2.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[13], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[15], 0.0f, 1.0e-6f);
}

void testDebugOmegaZInjectionModeOffKeepsManualOmegaInput()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.max_vel_x_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_vel_y_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_omega_z_ = 3.0f;
    chassis.airjoy_data_.left_y = 0.0f;
    chassis.airjoy_data_.left_x = 0.0f;
    chassis.airjoy_data_.right_x = 0.5f;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = 0U;

    chassis.applyDebugTargetOverride(Chassis::DebugMode::kBodySpeed);

    EXPECT_TRUE(chassis.input_target_data_.mode == Chassis::Mode::kBodySpeedMode);
    EXPECT_NEAR(chassis.input_target_data_.omega_z, 1.5f, 1.0e-6f);
}

void testDebugOmegaZInjectionModeStepOverridesManualOmegaInput()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.max_vel_x_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_vel_y_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_omega_z_ = 3.0f;
    chassis.airjoy_data_.left_y = 0.0f;
    chassis.airjoy_data_.left_x = 0.0f;
    chassis.airjoy_data_.right_x = 0.5f;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = 1U;

    chassis.applyDebugTargetOverride(Chassis::DebugMode::kBodySpeed);

    EXPECT_NEAR(chassis.input_target_data_.omega_z, 3.0f, 1.0e-6f);
}

void testDebugOmegaZInjectionModeSineOverridesManualOmegaInput()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.max_vel_x_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_vel_y_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_omega_z_ = 3.0f;
    chassis.airjoy_data_.left_y = 0.0f;
    chassis.airjoy_data_.left_x = 0.0f;
    chassis.airjoy_data_.right_x = 0.5f;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = 2U;
    chassis.debug_control_.injection.omega_z_sine_amplitude = 1.0f;
    chassis.debug_control_.injection.omega_z_sine_frequency_hz = 0.0f;
    chassis.debug_control_.injection.omega_z_sine_offset = 0.25f;
    chassis.time_ms_ = 250U;

    chassis.applyDebugTargetOverride(Chassis::DebugMode::kWorldSpeed);

    EXPECT_TRUE(chassis.input_target_data_.mode == Chassis::Mode::kWorldSpeedMode);
    EXPECT_NEAR(chassis.input_target_data_.omega_z, 0.25f, 1.0e-6f);
}

void testDebugOmegaZInjectionDoesNotAffectLockToTarget()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.max_vel_x_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_vel_y_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_omega_z_ = 3.0f;
    chassis.airjoy_data_.left_y = 0.2f;
    chassis.airjoy_data_.left_x = -0.1f;
    chassis.airjoy_data_.right_x = 0.8f;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = 2U;
    chassis.debug_control_.injection.omega_z_sine_amplitude = 2.0f;
    chassis.debug_control_.injection.omega_z_sine_frequency_hz = 0.0f;
    chassis.debug_control_.injection.omega_z_sine_offset = 0.5f;
    chassis.debug_control_.injection.lock_rot_z = 1.2f;

    chassis.applyDebugTargetOverride(Chassis::DebugMode::kBodyLockTo);

    EXPECT_TRUE(chassis.input_target_data_.mode == Chassis::Mode::kBodySpeedLockToRotZMode);
    EXPECT_NEAR(chassis.input_target_data_.rot_z, 1.2f, 1.0e-6f);
}

void configureDriveContinuityHarness(Chassis &chassis, VESC_Motor drive_motors[4])
{
    chassis.runtime_strategy_cfg_.max_drive_alpha_rad_s2_ = 10.0f;
    chassis.runtime_strategy_cfg_.max_drive_omega_rad_s_ = 1000.0f;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = true;
    chassis.runtime_strategy_cfg_.enable_drive_omega_limit_ = false;
    chassis.input_target_data_.zero_current_all = false;
    chassis.current_mode_flag_.is_wheel_torque_free = false;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].drive_motor_h = &drive_motors[i];
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].target_drive_omega_rad_s = 0.0f;
        chassis.last_drive_omega_cmd_rad_s_[i] = 0.0f;
        chassis.planned_data_.drive_omega_rad_s[i] = 0.0f;
        drive_motors[i].pid_init(PID_Param_Config{}, 0.0f);
        drive_motors[i].setRpmControlMode(VESC_RPM_CONTROL_PID_CURRENT);
        drive_motors[i].setTargetRPM(0.0f);
    }
}

void configureDriveZeroStopHarness(Chassis &chassis, VESC_Motor drive_motors[4])
{
    configureDriveContinuityHarness(chassis, drive_motors);

    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_assist = true;
    chassis.runtime_strategy_cfg_.drive_zero_stop_settle_speed_m_s = 0.02f;
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_enter_speed_m_s = 0.08f;
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_exit_speed_m_s = 0.12f;
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_current_mA = 1200.0f;
}

void setWheelResidualSpeedMps(Chassis &chassis, int wheel_idx, float residual_speed_m_s)
{
    const float wheel_radius_m = chassis.runtime_strategy_cfg_.wheel_radius_m_;
    chassis.wheel_config_[wheel_idx].corrected_drive_omega_rad_s =
        (std::fabs(wheel_radius_m) > 1.0e-6f) ? (residual_speed_m_s / wheel_radius_m) : 0.0f;
}

void configureXParkWheelGeometry(Chassis &chassis)
{
    const float half_track_m = 0.20f;
    const float positions[4][2] = {
        {half_track_m, half_track_m},
        {-half_track_m, half_track_m},
        {-half_track_m, -half_track_m},
        {half_track_m, -half_track_m},
    };

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].pos_x_m = positions[i][0];
        chassis.wheel_config_[i].pos_y_m = positions[i][1];
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
    }
}

void setWheelPoseToXPark(Chassis &chassis)
{
    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad =
            std::atan2(chassis.wheel_config_[i].pos_y_m, chassis.wheel_config_[i].pos_x_m);
    }
}

void configureHardGateLaunchHarness(Chassis &chassis, VESC_Motor drive_motors[4])
{
    configureDriveContinuityHarness(chassis, drive_motors);
    configureXParkWheelGeometry(chassis);
    setWheelPoseToXPark(chassis);

    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg = 1.0f;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.min_scale = 0.0f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.max_acc_xy_acc_ = 1.0f;
    chassis.runtime_strategy_cfg_.max_acc_xy_dec_ = 1.0f;
    chassis.runtime_strategy_cfg_.max_alpha_z_acc_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_alpha_z_dec_ = 2.0f;
    chassis.xpark_gate_active_ = true;
}

void configureXParkTriggerHarness(Chassis &chassis)
{
    const float half_track_m = 0.20f;
    const float positions[4][2] = {
        {half_track_m, half_track_m},
        {-half_track_m, half_track_m},
        {-half_track_m, -half_track_m},
        {half_track_m, -half_track_m},
    };

    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.xpark_entry_delay_ms = 10U;
    chassis.xpark_gate_active_ = false;
    chassis.xpark_stationary_hold_ms_ = 0U;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].pos_x_m = positions[i][0];
        chassis.wheel_config_[i].pos_y_m = positions[i][1];
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
    }
}

void configureSteerFaultRecoveryHarness(Chassis &chassis, TestMotor steer_motors[4], VESC_Motor drive_motors[4])
{
    testHostResetPhotogates();

    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_omega_limit_ = false;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = true;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.ignore_during_xpark_hold = true;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.freeze_current_delta_mA = 1.0f;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.active_current_min_mA = 500.0f;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.freeze_angle_delta_rad = 1.0e-4f;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.freeze_duration_ms = 5U;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.recovery_current_delta_mA = 1000.0f;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.recovery_toggle_threshold = 1U;
    chassis.runtime_strategy_cfg_.max_acc_xy_acc_ = 1000.0f;
    chassis.runtime_strategy_cfg_.max_acc_xy_dec_ = 1000.0f;
    chassis.runtime_strategy_cfg_.max_alpha_z_acc_ = 1000.0f;
    chassis.runtime_strategy_cfg_.max_alpha_z_dec_ = 1000.0f;
    chassis.input_target_data_.zero_current_all = false;
    chassis.input_target_data_.mode = Chassis::Mode::kBodySpeedMode;
    chassis.input_target_data_.vel_x = 0.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;
    chassis.current_mode_flag_.is_wheel_torque_free = false;
    chassis.current_mode_flag_.is_world_speed_mode = false;
    chassis.current_mode_flag_.is_lock_now_rot_z = false;
    chassis.current_mode_flag_.is_lock_to_rot_z = false;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].pos_x_m = 0.0f;
        chassis.wheel_config_[i].pos_y_m = 0.0f;
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].steer_motor_h = &steer_motors[i];
        chassis.wheel_config_[i].drive_motor_h = &drive_motors[i];
        chassis.wheel_config_[i].homing_enabled = true;
        chassis.wheel_config_[i].homing_sensor_active_high = true;
        chassis.wheel_config_[i].homing_gpio_port = (i == 0) ? kPHOTOGATE_1_GPIO_Port
                                            : (i == 1) ? kPHOTOGATE_2_GPIO_Port
                                            : (i == 2) ? kPHOTOGATE_3_GPIO_Port
                                                       : kPHOTOGATE_4_GPIO_Port;
        chassis.wheel_config_[i].homing_gpio_pin = static_cast<jia::u16>((i == 0) ? kPHOTOGATE_1_Pin
                                                                    : (i == 1) ? kPHOTOGATE_2_Pin
                                                                    : (i == 2) ? kPHOTOGATE_3_Pin
                                                                               : kPHOTOGATE_4_Pin);
        chassis.wheel_config_[i].homing_falling_edge_mech_rad = 0.0f;
        chassis.wheel_config_[i].homing_rising_edge_mech_rad = 0.0f;
        chassis.wheel_config_[i].homing_search_rpm = 15.0f;
        chassis.wheel_config_[i].homing_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].homing_timeout_s = 0.003f;
        chassis.wheel_config_[i].homing_state = Chassis::HomingState::kReady;
        chassis.wheel_config_[i].homing_last_sensor_active = false;
        chassis.wheel_config_[i].homing_last_edge_is_falling = false;
        chassis.wheel_config_[i].homing_zero_valid = true;
        chassis.wheel_config_[i].homing_elapsed_s = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].target_steer_motor_total_angle_rad = 0.0f;
        chassis.wheel_config_[i].target_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].steer_target_velocity_rad_s = 0.0f;
        chassis.wheel_config_[i].flipped_drive_direction = false;
        chassis.last_drive_omega_cmd_rad_s_[i] = 0.0f;
        chassis.last_steer_rate_cmd_rad_s_[i] = 0.0f;
        chassis.actuator_command_frame_.drive_omega_rad_s[i] = 0.0f;
        chassis.actuator_command_frame_.steer_oa_total_rad[i] = 0.0f;
        chassis.actuator_command_frame_.steer_corrected_local_total_rad[i] = 0.0f;
        chassis.planned_data_.drive_omega_rad_s[i] = 0.0f;
        chassis.planned_data_.steer_angle_oa_rad[i] = 0.0f;
        chassis.current_data_.drive_omega_rad_s[i] = 0.0f;
        chassis.current_data_.steer_angle_oa_rad[i] = 0.0f;
        steer_motors[i].setFeedbackCurrent(0.0f);
        steer_motors[i].setFeedbackRpm(0.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
        steer_motors[i].setTargetCurrent(0.0f);
        steer_motors[i].setTargetRPM(0.0f);
        steer_motors[i].setTargetTotalAngle(0.0f);
        drive_motors[i].setFeedbackCurrent(0.0f);
        drive_motors[i].setFeedbackRpm(0.0f);
        drive_motors[i].setFeedbackTotalAngleDeg(0.0f);
        drive_motors[i].pid_init(PID_Param_Config{}, 0.0f);
        drive_motors[i].setRpmControlMode(VESC_RPM_CONTROL_PID_CURRENT);
        drive_motors[i].setTargetCurrent(0.0f);
        drive_motors[i].setTargetRPM(0.0f);
        drive_motors[i].setTargetTotalAngle(0.0f);
        setPhotogateStateForWheel(i, false);
    }
}

bool runHostControlCycle(Chassis &chassis)
{
    chassis.resolvePlannerTargetData();
    chassis.refreshActuatorLimitState();
    chassis.updatePlannedMotionData();
    chassis.updateWheelFeedback();

    bool all_homed = true;
    for (int i = 0; i < 4; ++i)
    {
        if (!chassis.updateHomingState(chassis.wheel_config_[i]))
        {
            all_homed = false;
        }
    }

    if (!all_homed && chassis.input_target_data_.zero_current_all)
    {
        chassis.input_target_data_.zero_current_all = false;
    }
    chassis.homing_start_request_ = false;

    chassis.computeModuleCommands(chassis.planned_data_);
    chassis.applyModuleCommands(all_homed);
    chassis.updateCurrentData(all_homed);
    chassis.refreshDebugMirror(all_homed);
    chassis.last_planned_data_ = chassis.planned_data_;
    return all_homed;
}

void runDebugPlannerCycleForHost(Chassis &chassis)
{
    chassis.isDebugMode();
    chassis.setModeFlag();
    chassis.resolvePlannerTargetData();
    chassis.refreshActuatorLimitState();
    chassis.updatePlannedMotionData();
    chassis.last_planned_data_ = chassis.planned_data_;
}

bool runDebugControlCycleForHost(Chassis &chassis)
{
    chassis.isDebugMode();
    chassis.setModeFlag();
    return runHostControlCycle(chassis);
}

void emitDebugOutputForHost(Chassis &chassis, bool all_homed)
{
    chassis.refreshDebugMirror(all_homed);
    chassis.emitDebugOutputByMode(all_homed);
}

void configureDebugOutputFamily(Chassis &chassis, unsigned char family_raw)
{
    chassis.debug_output_.output_enable = true;
    chassis.debug_output_.output_family_raw = family_raw;
}

void configureJustFloatProfile(Chassis &chassis, unsigned char profile_raw)
{
    chassis.debug_output_.justfloat.profile_raw = profile_raw;
}

void configureSingleWheelPayload(Chassis &chassis, unsigned char payload_raw)
{
    chassis.debug_output_.justfloat.single_wheel_payload_raw = payload_raw;
}

void configureYawPidTraceHarness(Chassis &chassis)
{
    testHostResetJustFloatCapture();
    configureDebugOutputFamily(chassis, 2U);
    configureJustFloatProfile(chassis, 2U);
    chassis.debug_output_.justfloat.yaw_pid.period_ms = 0U;
    chassis.debug_output_runtime_.justfloat.yaw_pid.last_ms = 0U;
    configureSingleWheelPayload(chassis, 0U);
    chassis.time_ms_ = 100U;
    chassis.input_hwt_omega_z_ = 0.25f;
    chassis.debug_mirror_.all_homed = true;
    chassis.debug_mirror_.steer_fault_any_active = false;
    chassis.debug_mirror_.high_speed_drive_suppression_active = false;
    chassis.debug_mirror_.reverse_intent_active = false;
}

void testJustFloatYawPidProfileDispatchEmitsFixed15ChannelPayload()
{
    Chassis chassis;
    configureYawPidTraceHarness(chassis);

    chassis.yaw_pid_trace_.mode_tag = 4.0f;
    chassis.yaw_pid_trace_.target_yaw_rad = 0.5f;
    chassis.yaw_pid_trace_.feedback_yaw_rad = 0.25f;
    chassis.yaw_pid_trace_.error_deg = jia::radToDegF32(0.25f);
    chassis.yaw_pid_trace_.manual_omega_in_rad_s = 0.0f;
    chassis.yaw_pid_trace_.pid_output_omega_rad_s = 0.8f;
    chassis.yaw_pid_trace_.final_omega_cmd_rad_s = 0.8f;
    chassis.yaw_pid_trace_.feedback_yaw_rate_rad_s = 0.25f;
    chassis.yaw_pid_trace_.shift_remaining_ms = 0.0f;
    chassis.yaw_pid_trace_.pid_compute_fired = 1.0f;

    emitDebugOutputForHost(chassis, true);

    EXPECT_TRUE(g_test_justfloat_capture.called);
    EXPECT_TRUE(g_test_justfloat_capture.size == 15U);
    EXPECT_NEAR(g_test_justfloat_capture.values[0], 0.1f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[1], 4.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[2], 0.5f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[3], 0.25f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[10], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[12], 1.0f, 1.0e-6f);
}

void testLockToYawPidTracePublishesTargetErrorAndPidFireState()
{
    Chassis chassis;
    configureYawPidTraceHarness(chassis);

    chassis.rot_z_pid_period_ = 0U;
    chassis.rot_z_pid_count_ = 0U;
    chassis.input_hwt_rot_z_ = 0.2f;
    chassis.input_hwt_omega_z_ = -0.4f;

    float out_rot_z = 0.0f;
    float out_omega_z = 0.0f;
    chassis.isLockToRotZ(true, 0.6f, 0.2f, out_rot_z, 0.0f, out_omega_z);

    EXPECT_NEAR(chassis.yaw_pid_trace_.mode_tag, 4.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.target_yaw_rad, out_rot_z, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.feedback_yaw_rad, 0.2f, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.feedback_yaw_rate_rad_s, -0.4f, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.final_omega_cmd_rad_s, out_omega_z, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.pid_compute_fired, 1.0f, 1.0e-6f);
    EXPECT_TRUE(chassis.yaw_pid_trace_.error_deg > 0.0f);
}

void testLockToYawThenLockNowKeepsTheEffectiveLockedYaw()
{
    Chassis chassis;
    configureYawPidTraceHarness(chassis);

    chassis.rot_z_pid_period_ = 0U;
    chassis.rot_z_pid_count_ = 0U;
    chassis.lock_now_rot_z_shift_count_ = 5U;
    chassis.input_hwt_rot_z_ = 0.2f;

    float out_rot_z = 0.0f;
    float out_omega_z = 0.0f;

    chassis.isLockToRotZ(true, 0.23f, 0.2f, out_rot_z, 0.0f, out_omega_z);
    const float effective_lock_rot_z = out_rot_z;
    EXPECT_TRUE(std::fabs(effective_lock_rot_z - 0.23f) > 1.0e-6f);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, effective_lock_rot_z, 1.0e-6f);
    EXPECT_TRUE(chassis.lock_now_rot_z_shift_count_ == 0U);

    chassis.input_hwt_rot_z_ = -0.35f;
    chassis.isLockNowRotZ(true, 0.0f, 0.0f, out_rot_z, out_omega_z);

    EXPECT_NEAR(out_rot_z, effective_lock_rot_z, 1.0e-6f);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, effective_lock_rot_z, 1.0e-6f);
    EXPECT_TRUE(std::fabs(out_rot_z - chassis.input_hwt_rot_z_) > 1.0e-6f);
}

void testLockNowYawPidTraceDistinguishesManualShiftAndHoldStates()
{
    Chassis chassis;
    configureYawPidTraceHarness(chassis);
    chassis.rot_z_pid_period_ = 0U;
    chassis.rot_z_pid_count_ = 0U;
    chassis.lock_now_rot_z_shift_time_ms_ = 3U;
    chassis.input_hwt_rot_z_ = 0.3f;

    float out_rot_z = 0.0f;
    float out_omega_z = 0.0f;

    chassis.isLockNowRotZ(true, 0.0f, 1.2f, out_rot_z, out_omega_z);
    EXPECT_NEAR(chassis.yaw_pid_trace_.mode_tag, 1.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.manual_omega_in_rad_s, 1.2f, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.final_omega_cmd_rad_s, 1.2f, 1.0e-6f);

    chassis.isLockNowRotZ(true, 0.0f, 0.0f, out_rot_z, out_omega_z);
    EXPECT_NEAR(chassis.yaw_pid_trace_.mode_tag, 2.0f, 1.0e-6f);
    EXPECT_TRUE(chassis.yaw_pid_trace_.shift_remaining_ms > 0.0f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.final_omega_cmd_rad_s, 0.0f, 1.0e-6f);

    chassis.lock_now_rot_z_shift_count_ = 0U;
    chassis.input_hwt_rot_z_ = 0.28f;
    chassis.isLockNowRotZ(true, 0.0f, 0.0f, out_rot_z, out_omega_z);
    EXPECT_NEAR(chassis.yaw_pid_trace_.mode_tag, 3.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.pid_compute_fired, 1.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.shift_remaining_ms, 0.0f, 1.0e-6f);
}

void finishWheelHomingByEdgeAndAlign(Chassis &chassis, int wheel_idx, TestMotor steer_motors[4])
{
    setPhotogateStateForWheel(wheel_idx, false);
    steer_motors[wheel_idx].setFeedbackTotalAngleDeg(0.0f);
    runHostControlCycle(chassis);

    setPhotogateStateForWheel(wheel_idx, true);
    runHostControlCycle(chassis);
    runHostControlCycle(chassis);
    runHostControlCycle(chassis);

    chassis.wheel_config_[wheel_idx].corrected_steer_motor_total_angle_rad = 0.0f;
    steer_motors[wheel_idx].setFeedbackTotalAngleDeg(0.0f);
    runHostControlCycle(chassis);
}

Chassis::ActuatorCommandFrame makeDriveOnlyCommandFrame(float drive_omega_rad_s)
{
    Chassis::ActuatorCommandFrame frame{};
    for (int i = 0; i < 4; ++i)
    {
        frame.drive_omega_rad_s[i] = drive_omega_rad_s;
    }
    return frame;
}

Chassis::ActuatorCommandFrame makePerWheelDriveCommandFrame(float drive0, float drive1, float drive2, float drive3)
{
    Chassis::ActuatorCommandFrame frame{};
    frame.drive_omega_rad_s[0] = drive0;
    frame.drive_omega_rad_s[1] = drive1;
    frame.drive_omega_rad_s[2] = drive2;
    frame.drive_omega_rad_s[3] = drive3;
    return frame;
}

Chassis::SwervePlannerOutput makeNeutralPlannerOutput()
{
    Chassis::SwervePlannerOutput output{};
    output.high_speed_suppression_scale = 1.0f;
    for (int i = 0; i < 4; ++i)
    {
        output.low_speed_suppression_scale[i] = 1.0f;
    }
    return output;
}

float getWheelXParkTargetOaRad(const Chassis &chassis, int wheel_idx)
{
    return chassis.getXParkAngle(chassis.wheel_config_[wheel_idx]);
}

void setWheelOaAngleRad(Chassis &chassis, int wheel_idx, float oa_rad)
{
    chassis.wheel_config_[wheel_idx].corrected_steer_motor_total_angle_rad =
        chassis.mapWheelOaTotalToCorrectedLocal(chassis.wheel_config_[wheel_idx], oa_rad);
}

Chassis::ActuatorCommandFrame makeXParkSteerCommandFrame(Chassis &chassis)
{
    Chassis::ActuatorCommandFrame frame{};
    for (int i = 0; i < 4; ++i)
    {
        const float xpark_target_oa_rad = getWheelXParkTargetOaRad(chassis, i);
        frame.steer_oa_total_rad[i] = xpark_target_oa_rad;
        frame.steer_corrected_local_total_rad[i] =
            chassis.mapWheelOaTotalToCorrectedLocal(chassis.wheel_config_[i], xpark_target_oa_rad);
    }
    return frame;
}

void configureXParkSteerDeadbandHarness(XParkSteerDeadbandHarness &harness)
{
    Chassis &chassis = harness.chassis;
    configureDriveContinuityHarness(chassis, harness.drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kXPark;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_omega_limit_ = false;
    chassis.runtime_strategy_cfg_.xpark_steer_deadband_cfg_.enable = true;
    chassis.runtime_strategy_cfg_.xpark_steer_deadband_cfg_.enter_angle_deg = 1.0f;
    chassis.runtime_strategy_cfg_.xpark_steer_deadband_cfg_.exit_angle_deg = 3.0f;
    chassis.input_target_data_.zero_current_all = false;
    chassis.current_mode_flag_.is_wheel_torque_free = false;
    chassis.current_mode_flag_.is_world_speed_mode = false;
    chassis.current_mode_flag_.is_lock_now_rot_z = false;
    chassis.current_mode_flag_.is_lock_to_rot_z = false;
    chassis.debug_control_.common.enable = false;
    chassis.xpark_gate_active_ = true;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].steer_motor_h = &harness.steer_motors[i];
        chassis.wheel_config_[i].homing_state = Chassis::HomingState::kReady;
        chassis.wheel_config_[i].steer_fault_state = Chassis::SteerFaultState::kNone;
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].target_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].target_steer_motor_total_angle_rad = 0.0f;
        chassis.wheel_config_[i].steer_target_velocity_rad_s = 0.0f;
        chassis.last_drive_omega_cmd_rad_s_[i] = 0.0f;
        chassis.last_steer_rate_cmd_rad_s_[i] = 0.0f;
        harness.steer_motors[i].setTargetCurrent(0.0f);
        harness.steer_motors[i].setTargetTotalAngle(0.0f);
        harness.steer_motors[i].resetLastCommandObservation();
        setWheelOaAngleRad(chassis, i, getWheelXParkTargetOaRad(chassis, i));
    }
}

void configureSteerSettleResetHarness(SteerSettleResetHarness &harness)
{
    configureDriveContinuityHarness(harness.chassis, harness.drive_motors);
    configureXParkWheelGeometry(harness.chassis);

    harness.chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    harness.chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    harness.chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kHoldLast;
    harness.chassis.runtime_strategy_cfg_.steer_speed_pid_settle_reset_cfg_.enable = true;
    harness.chassis.runtime_strategy_cfg_.steer_speed_pid_settle_reset_cfg_.enter_angle_deg = 0.6f;
    harness.chassis.runtime_strategy_cfg_.steer_speed_pid_settle_reset_cfg_.exit_angle_deg = 3.0f;
    harness.chassis.runtime_strategy_cfg_.steer_speed_pid_settle_reset_cfg_.enter_target_rate_deg_s = 2.0f;
    harness.chassis.runtime_strategy_cfg_.steer_speed_pid_settle_reset_cfg_.exit_target_rate_deg_s = 12.0f;
    harness.chassis.input_target_data_.zero_current_all = false;
    harness.chassis.current_mode_flag_.is_wheel_torque_free = false;
    harness.chassis.debug_control_.common.enable = false;
    harness.chassis.xpark_gate_active_ = false;

    for (int i = 0; i < 4; ++i)
    {
        harness.chassis.wheel_config_[i].steer_motor_h = &harness.steer_motors[i];
        harness.chassis.wheel_config_[i].homing_state = Chassis::HomingState::kReady;
        harness.chassis.wheel_config_[i].steer_fault_state = Chassis::SteerFaultState::kNone;
        harness.chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
        harness.chassis.wheel_config_[i].target_drive_omega_rad_s = 0.0f;
        harness.chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
        harness.chassis.wheel_config_[i].target_steer_motor_total_angle_rad = 0.0f;
        harness.chassis.wheel_config_[i].steer_target_velocity_rad_s = 0.0f;
        harness.chassis.last_drive_omega_cmd_rad_s_[i] = 0.0f;
        harness.chassis.last_steer_rate_cmd_rad_s_[i] = 0.0f;
    }
}

Chassis::ActuatorCommandFrame makeSteerSettleCommandFrame(Chassis &chassis,
                                                          float wheel0_target_deg,
                                                          float wheel0_rate_deg_s)
{
    Chassis::ActuatorCommandFrame frame{};
    for (int i = 0; i < 4; ++i)
    {
        const float current_corrected_local_total_rad = chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad;
        frame.steer_corrected_local_total_rad[i] = current_corrected_local_total_rad;
        frame.steer_oa_total_rad[i] =
            chassis.mapWheelCorrectedLocalToOaTotal(chassis.wheel_config_[i], current_corrected_local_total_rad);
        frame.steer_rate_rad_s[i] = 0.0f;
        frame.drive_omega_rad_s[i] = 0.0f;
    }

    frame.steer_corrected_local_total_rad[0] = jia::degToRadF32(wheel0_target_deg);
    frame.steer_oa_total_rad[0] =
        chassis.mapWheelCorrectedLocalToOaTotal(chassis.wheel_config_[0], frame.steer_corrected_local_total_rad[0]);
    frame.steer_rate_rad_s[0] = jia::degToRadF32(wheel0_rate_deg_s);
    return frame;
}

Chassis::SwervePlannerInput makeGatePlannerInput(float steering_error_deg,
                                                 float command_vel_x,
                                                 float command_omega_z,
                                                 float max_residual_speed_m_s)
{
    Chassis::SwervePlannerInput input{};
    input.command.vel_x = command_vel_x;
    input.command.omega_z = command_omega_z;
    input.max_residual_speed_m_s = max_residual_speed_m_s;
    for (int i = 0; i < 4; ++i)
    {
        input.residual_speed_m_s[i] = max_residual_speed_m_s;
        input.current_oa_total_rad[i] = 0.0f;
        input.wheel_speed_m_s[i] = std::fabs(command_vel_x);
    }
    input.current_oa_total_rad[0] = 0.0f;
    return input;
}

void testSuppressedDriveDoesNotAccumulateHiddenAccelState()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveContinuityHarness(chassis, drive_motors);

    const Chassis::SwervePlannerOutput planner_output = makeNeutralPlannerOutput();
    const Chassis::ActuatorCommandFrame command_frame = makeDriveOnlyCommandFrame(20.0f);

    for (int cycle = 0; cycle < 3; ++cycle)
    {
        chassis.storePlannedActuatorFrame(planner_output, command_frame);
        chassis.applyModuleCommands(false);
    }

    EXPECT_NEAR(chassis.last_drive_omega_cmd_rad_s_[0], 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[0].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[0].getTargetRPM(), 0.0f, 1.0e-6f);
}

void testDriveReleaseResumesFromDeliveredSpeedNotVirtualSpeed()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveContinuityHarness(chassis, drive_motors);
    const float expected_release_step = chassis.runtime_strategy_cfg_.max_drive_alpha_rad_s2_ * Chassis::period_;

    const Chassis::SwervePlannerOutput planner_output = makeNeutralPlannerOutput();
    const Chassis::ActuatorCommandFrame command_frame = makeDriveOnlyCommandFrame(20.0f);

    for (int cycle = 0; cycle < 3; ++cycle)
    {
        chassis.storePlannedActuatorFrame(planner_output, command_frame);
        chassis.applyModuleCommands(false);
    }

    chassis.storePlannedActuatorFrame(planner_output, command_frame);
    chassis.applyModuleCommands(true);

    EXPECT_NEAR(chassis.last_drive_omega_cmd_rad_s_[0], expected_release_step, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[0].target_drive_omega_rad_s, expected_release_step, 1.0e-6f);
    EXPECT_NEAR(drive_motors[0].getTargetRPM(), jia::radsToRpmF32(expected_release_step), 1.0e-4f);
}

void testDriveReleaseHasNoVelocityJumpAfterZeroHold()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveContinuityHarness(chassis, drive_motors);

    const Chassis::SwervePlannerOutput planner_output = makeNeutralPlannerOutput();
    const Chassis::ActuatorCommandFrame command_frame = makeDriveOnlyCommandFrame(20.0f);

    chassis.storePlannedActuatorFrame(planner_output, command_frame);
    chassis.applyModuleCommands(false);
    chassis.storePlannedActuatorFrame(planner_output, command_frame);
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.wheel_config_[0].target_drive_omega_rad_s < 5.0f);
    EXPECT_TRUE(chassis.last_drive_omega_cmd_rad_s_[0] < 5.0f);
}

void testPlannerTargetMayChangeWhileDeliveredStateRemainsContinuous()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveContinuityHarness(chassis, drive_motors);

    const Chassis::SwervePlannerOutput planner_output = makeNeutralPlannerOutput();

    chassis.storePlannedActuatorFrame(planner_output, makeDriveOnlyCommandFrame(10.0f));
    chassis.applyModuleCommands(false);
    EXPECT_NEAR(chassis.actuator_command_frame_.drive_omega_rad_s[0], 10.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.last_drive_omega_cmd_rad_s_[0], 0.0f, 1.0e-6f);

    chassis.storePlannedActuatorFrame(planner_output, makeDriveOnlyCommandFrame(30.0f));
    chassis.applyModuleCommands(false);
    EXPECT_NEAR(chassis.actuator_command_frame_.drive_omega_rad_s[0], 30.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.last_drive_omega_cmd_rad_s_[0], 0.0f, 1.0e-6f);
}

void testTorqueFreeAndNotHomedPathsResetDriveDeliveryStateConsistently()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveContinuityHarness(chassis, drive_motors);

    const Chassis::SwervePlannerOutput planner_output = makeNeutralPlannerOutput();
    const Chassis::ActuatorCommandFrame command_frame = makeDriveOnlyCommandFrame(20.0f);

    chassis.storePlannedActuatorFrame(planner_output, command_frame);
    chassis.current_mode_flag_.is_wheel_torque_free = true;
    chassis.applyModuleCommands(true);
    EXPECT_NEAR(chassis.last_drive_omega_cmd_rad_s_[0], 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[0].target_drive_omega_rad_s, 0.0f, 1.0e-6f);

    chassis.current_mode_flag_.is_wheel_torque_free = false;
    chassis.storePlannedActuatorFrame(planner_output, command_frame);
    chassis.applyModuleCommands(false);
    EXPECT_NEAR(chassis.last_drive_omega_cmd_rad_s_[0], 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[0].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
}

void testDriveZeroStopUsesBrakeWhenResidualSpeedIsStillHigh()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveZeroStopHarness(chassis, drive_motors);

    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(0.0f));
    setWheelResidualSpeedMps(chassis, 0, 0.15f);

    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_NEAR(drive_motors[0].getTargetBrake(), 1200.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[0].getSpeedPidCurrentBias(), 0.0f, 1.0e-6f);
    EXPECT_TRUE(drive_motors[0].getResetSpeedPidStateCallCount() == 1U);
}

void testDriveZeroStopSettlesToZeroCurrentWithoutHoldingBrake()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveZeroStopHarness(chassis, drive_motors);

    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(0.0f));
    setWheelResidualSpeedMps(chassis, 0, 0.01f);

    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(!chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kCurrent);
    EXPECT_NEAR(drive_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
    EXPECT_TRUE(drive_motors[0].getResetSpeedPidStateCallCount() == 1U);
}

void testDriveZeroStopReleaseClearsPidStateBeforeReturningToRpmLoop()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveZeroStopHarness(chassis, drive_motors);

    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(0.0f));
    setWheelResidualSpeedMps(chassis, 0, 0.01f);
    chassis.applyModuleCommands(true);

    setWheelResidualSpeedMps(chassis, 0, 0.00f);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(20.0f));
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(!chassis.drive_zero_stop_active_);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kRpm);
    EXPECT_NEAR(drive_motors[0].getTargetRPM(), jia::radsToRpmF32(20.0f), 1.0e-4f);
    EXPECT_TRUE(drive_motors[0].getResetSpeedPidStateCallCount() == 2U);
}

void testDriveZeroStopBrakeHysteresisDoesNotToggleAroundThreshold()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveZeroStopHarness(chassis, drive_motors);

    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(0.0f));
    setWheelResidualSpeedMps(chassis, 0, 0.13f);
    chassis.applyModuleCommands(true);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);

    setWheelResidualSpeedMps(chassis, 0, 0.09f);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(0.0f));
    chassis.applyModuleCommands(true);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);

    setWheelResidualSpeedMps(chassis, 0, 0.01f);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(0.0f));
    chassis.applyModuleCommands(true);
    EXPECT_TRUE(!chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kCurrent);
}

void testLowSpeedDriveSuppressionBypassesWhenResidualSpeedAboveThreshold()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.min_scale = 0.0f;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg = 1.0f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.3f;

    const Chassis::SwervePlannerInput planner_input = makeGatePlannerInput(10.0f, 0.0f, 0.0f, 0.5f);
    const float steering_errors_rad[4] = {
        jia::degToRadF32(10.0f), jia::degToRadF32(10.0f), jia::degToRadF32(10.0f), jia::degToRadF32(10.0f)};
    float gate_scales[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    chassis.computeLowSpeedDriveSuppressionScales(planner_input, steering_errors_rad, gate_scales);

    EXPECT_NEAR(gate_scales[0], 1.0f, 1.0e-6f);
    EXPECT_NEAR(gate_scales[3], 1.0f, 1.0e-6f);
}

void testLowSpeedDriveSuppressionReenabledWhenResidualSpeedDropsBelowThreshold()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.min_scale = 0.0f;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg = 1.0f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.3f;

    const float steering_errors_rad[4] = {
        jia::degToRadF32(10.0f), jia::degToRadF32(10.0f), jia::degToRadF32(10.0f), jia::degToRadF32(10.0f)};
    float gate_scales[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    chassis.computeLowSpeedDriveSuppressionScales(makeGatePlannerInput(10.0f, 0.0f, 0.0f, 0.5f), steering_errors_rad, gate_scales);
    chassis.computeLowSpeedDriveSuppressionScales(makeGatePlannerInput(10.0f, 0.0f, 0.0f, 0.1f), steering_errors_rad, gate_scales);

    EXPECT_NEAR(gate_scales[0], 0.0f, 1.0e-6f);
    EXPECT_NEAR(gate_scales[2], 0.0f, 1.0e-6f);
}

void testLowSpeedDriveSuppressionUsesGlobalWorstWheelError()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg = 10.0f;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.min_scale = 0.2f;

    const float steering_errors_rad[4] = {
        jia::degToRadF32(0.0f), jia::degToRadF32(10.0f), 0.0f, 0.0f};
    float gate_scales[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    chassis.computeLowSpeedDriveSuppressionScales(makeGatePlannerInput(10.0f, 0.0f, 0.0f, 0.0f), steering_errors_rad, gate_scales);

    EXPECT_NEAR(gate_scales[0], 0.2f, 1.0e-6f);
    EXPECT_NEAR(gate_scales[1], 0.2f, 1.0e-6f);
    EXPECT_NEAR(gate_scales[2], 0.2f, 1.0e-6f);
    EXPECT_NEAR(gate_scales[3], 0.2f, 1.0e-6f);
}

void testGlobalMaxResidualSpeedControlsLowSpeedSuppressionForAllWheels()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.min_scale = 0.0f;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg = 1.0f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.3f;

    Chassis::SwervePlannerInput planner_input = makeGatePlannerInput(10.0f, 0.0f, 0.0f, 0.1f);
    planner_input.residual_speed_m_s[0] = 0.5f;
    planner_input.max_residual_speed_m_s = 0.5f;
    const float steering_errors_rad[4] = {
        jia::degToRadF32(10.0f), jia::degToRadF32(2.0f), jia::degToRadF32(10.0f), jia::degToRadF32(2.0f)};
    float gate_scales[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    chassis.computeLowSpeedDriveSuppressionScales(planner_input, steering_errors_rad, gate_scales);

    EXPECT_NEAR(gate_scales[0], 1.0f, 1.0e-6f);
    EXPECT_NEAR(gate_scales[1], 1.0f, 1.0e-6f);
    EXPECT_NEAR(gate_scales[2], 1.0f, 1.0e-6f);
    EXPECT_NEAR(gate_scales[3], 1.0f, 1.0e-6f);
}

void testRefreshDebugMirrorSeparatesPlannedAndDeliveredDriveDiagnostics()
{
    Chassis chassis;
    chassis.actuator_command_frame_.drive_omega_rad_s[0] = 12.0f;
    chassis.wheel_config_[0].target_drive_omega_rad_s = 3.0f;
    chassis.max_residual_speed_m_s_ = 0.45f;
    chassis.low_speed_drive_suppression_bypassed_by_residual_speed_ = true;

    chassis.refreshDebugMirror(true);

    EXPECT_NEAR(chassis.debug_mirror_.planned_drive_target_rpm[0], jia::radsToRpmF32(12.0f), 1.0e-4f);
    EXPECT_NEAR(chassis.debug_mirror_.delivered_drive_target_rpm[0], jia::radsToRpmF32(3.0f), 1.0e-4f);
    EXPECT_TRUE(chassis.debug_mirror_.low_speed_drive_suppression_bypassed_by_residual_speed);
    EXPECT_NEAR(chassis.debug_mirror_.max_residual_speed_m_s, 0.45f, 1.0e-6f);
}

void testHardGateFromXParkHoldsAllDriveUntilAllWheelsPassCloseAngle()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureHardGateLaunchHarness(chassis, drive_motors);

    Chassis::Data command{};
    command.vel_x = 1.0f;

    Chassis::SwervePlannerOutput planner_output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));

    for (int i = 0; i < 4; ++i)
    {
        EXPECT_TRUE(planner_output.steering_errors_rad[i] > jia::degToRadF32(1.0f));
        EXPECT_NEAR(planner_output.low_speed_suppression_scale[i], 0.0f, 1.0e-6f);
        EXPECT_NEAR(planner_output.final_drive_omega_rad_s[i], 0.0f, 1.0e-6f);
    }

    for (int i = 0; i < 3; ++i)
    {
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
    }

    planner_output = chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(planner_output.low_speed_suppression_scale[i], 0.0f, 1.0e-6f);
        EXPECT_NEAR(planner_output.final_drive_omega_rad_s[i], 0.0f, 1.0e-6f);
    }

    chassis.wheel_config_[3].corrected_steer_motor_total_angle_rad = 0.0f;
    planner_output = chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));

    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(planner_output.low_speed_suppression_scale[i], 1.0f, 1.0e-6f);
        EXPECT_TRUE(std::fabs(planner_output.final_drive_omega_rad_s[i]) > 1.0e-6f);
    }
}

void testLaunchFromXParkHoldsBodyAndDriveAtZeroUntilAllWheelsAligned()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureHardGateLaunchHarness(chassis, drive_motors);

    chassis.target_data_.vel_x = 1.0f;
    chassis.target_data_.vel_y = 0.0f;
    chassis.target_data_.omega_z = 1.0f;

    const float linear_step = chassis.runtime_strategy_cfg_.max_acc_xy_acc_ * Chassis::period_;
    const float angular_step = chassis.runtime_strategy_cfg_.max_alpha_z_acc_ * Chassis::period_;
    const float drive_step = chassis.runtime_strategy_cfg_.max_drive_alpha_rad_s2_ * Chassis::period_;

    for (int cycle = 0; cycle < 3; ++cycle)
    {
        chassis.updatePlannedMotionData();
        chassis.computeModuleCommands(chassis.planned_data_);
        chassis.applyModuleCommands(true);
        chassis.last_planned_data_ = chassis.planned_data_;
    }

    EXPECT_NEAR(chassis.planned_data_.vel_x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.planned_data_.vel_y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.planned_data_.omega_z, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.actuator_command_frame_.drive_omega_rad_s[0], 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.last_drive_omega_cmd_rad_s_[0], 0.0f, 1.0e-6f);

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad =
            chassis.planner_output_cache_.selected_oa_total_rad[i];
    }

    chassis.updatePlannedMotionData();
    chassis.computeModuleCommands(chassis.planned_data_);

    EXPECT_NEAR(chassis.planned_data_.vel_x, linear_step, 1.0e-6f);
    EXPECT_NEAR(chassis.planned_data_.vel_y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.planned_data_.omega_z, angular_step, 1.0e-6f);
    EXPECT_TRUE(std::fabs(chassis.actuator_command_frame_.drive_omega_rad_s[0]) > drive_step);

    chassis.applyModuleCommands(true);

    EXPECT_NEAR(chassis.last_drive_omega_cmd_rad_s_[0], drive_step, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[0].target_drive_omega_rad_s, drive_step, 1.0e-6f);
}

void testNormalLaunchSCurveWaitsForSteerAlignmentBeforeAccumulatingBodyPlan()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveContinuityHarness(chassis, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg = 1.0f;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.min_scale = 0.0f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kSCurve;
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = false;
    chassis.runtime_strategy_cfg_.manual_trans_acc_acc_ = 2.0f;
    chassis.runtime_strategy_cfg_.manual_trans_acc_dec_ = 3.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_ = 20.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_dec_ = 30.0f;

    chassis.target_data_.vel_x = 0.0f;
    chassis.target_data_.vel_y = 1.0f;
    chassis.target_data_.omega_z = 0.0f;

    for (int cycle = 0; cycle < 3; ++cycle)
    {
        chassis.updatePlannedMotionData();
        chassis.computeModuleCommands(chassis.planned_data_);
        chassis.applyModuleCommands(true);
        chassis.last_planned_data_ = chassis.planned_data_;
    }

    EXPECT_TRUE(chassis.launch_hold_active_);
    EXPECT_NEAR(chassis.planned_data_.vel_x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.planned_data_.vel_y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.planned_data_.omega_z, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.actuator_command_frame_.drive_omega_rad_s[0], 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.last_drive_omega_cmd_rad_s_[0], 0.0f, 1.0e-6f);

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad =
            chassis.planner_output_cache_.selected_oa_total_rad[i];
    }

    chassis.updatePlannedMotionData();
    chassis.computeModuleCommands(chassis.planned_data_);

    EXPECT_TRUE(!chassis.launch_hold_active_);
    EXPECT_NEAR(chassis.planned_data_.vel_x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.planned_data_.vel_y,
                chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_ * Chassis::period_ * Chassis::period_,
                1.0e-6f);
    EXPECT_TRUE(std::fabs(chassis.actuator_command_frame_.drive_omega_rad_s[0]) > 1.0e-6f);
}

void testManualSCurveProfileLegacyModeKeepsCurrentAccelerationStepSemantics()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kLegacy;
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = false;
    chassis.runtime_strategy_cfg_.max_acc_xy_acc_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_acc_xy_dec_ = 3.0f;
    chassis.runtime_strategy_cfg_.max_alpha_z_acc_ = 4.0f;
    chassis.runtime_strategy_cfg_.max_alpha_z_dec_ = 5.0f;

    chassis.target_data_.vel_x = 1.0f;
    chassis.target_data_.vel_y = -1.0f;
    chassis.target_data_.omega_z = 1.0f;

    chassis.updatePlannedMotionData();

    EXPECT_NEAR(chassis.planned_data_.vel_x, 2.0f * Chassis::period_, 1.0e-6f);
    EXPECT_NEAR(chassis.planned_data_.vel_y, -2.0f * Chassis::period_, 1.0e-6f);
    EXPECT_NEAR(chassis.planned_data_.omega_z, 4.0f * Chassis::period_, 1.0e-6f);
}

void testManualSCurveProfileProducesSofterFirstStepAndContinuousAcceleration()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kSCurve;
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = false;
    chassis.runtime_strategy_cfg_.manual_trans_acc_acc_ = 2.0f;
    chassis.runtime_strategy_cfg_.manual_trans_acc_dec_ = 3.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_ = 20.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_dec_ = 30.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_alpha_acc_ = 4.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_alpha_dec_ = 5.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_jerk_acc_ = 40.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_jerk_dec_ = 50.0f;

    chassis.target_data_.vel_x = 1.0f;
    chassis.target_data_.vel_y = 0.0f;
    chassis.target_data_.omega_z = 1.0f;

    chassis.updatePlannedMotionData();
    const float first_vel_x = chassis.planned_data_.vel_x;
    const float first_acc_x = chassis.planned_data_.acc_x;
    const float first_omega = chassis.planned_data_.omega_z;
    const float first_alpha = chassis.planned_data_.alpha_z;
    chassis.last_planned_data_ = chassis.planned_data_;

    chassis.updatePlannedMotionData();
    const float second_vel_x = chassis.planned_data_.vel_x;
    const float second_acc_x = chassis.planned_data_.acc_x;
    const float second_omega = chassis.planned_data_.omega_z;
    const float second_alpha = chassis.planned_data_.alpha_z;

    EXPECT_TRUE(first_vel_x > 0.0f);
    EXPECT_TRUE(first_vel_x < chassis.runtime_strategy_cfg_.manual_trans_acc_acc_ * Chassis::period_);
    EXPECT_NEAR(first_vel_x, chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_ * Chassis::period_ * Chassis::period_, 1.0e-6f);
    EXPECT_TRUE(second_vel_x > first_vel_x);
    EXPECT_TRUE(second_acc_x > first_acc_x);
    EXPECT_TRUE(second_acc_x <= chassis.runtime_strategy_cfg_.manual_trans_acc_acc_ + 1.0e-6f);

    EXPECT_TRUE(first_omega > 0.0f);
    EXPECT_TRUE(first_omega < chassis.runtime_strategy_cfg_.manual_yaw_alpha_acc_ * Chassis::period_);
    EXPECT_NEAR(first_omega, chassis.runtime_strategy_cfg_.manual_yaw_jerk_acc_ * Chassis::period_ * Chassis::period_, 1.0e-6f);
    EXPECT_TRUE(second_omega > first_omega);
    EXPECT_TRUE(second_alpha > first_alpha);
    EXPECT_TRUE(second_alpha <= chassis.runtime_strategy_cfg_.manual_yaw_alpha_acc_ + 1.0e-6f);
}

void testManualSCurveProfileToggleResetsShapingHistory()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kSCurve;
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = false;
    chassis.runtime_strategy_cfg_.manual_trans_acc_acc_ = 2.0f;
    chassis.runtime_strategy_cfg_.manual_trans_acc_dec_ = 3.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_ = 20.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_dec_ = 30.0f;

    chassis.target_data_.vel_x = 1.0f;
    chassis.updatePlannedMotionData();
    chassis.last_planned_data_ = chassis.planned_data_;
    chassis.updatePlannedMotionData();

    EXPECT_TRUE(chassis.planned_data_.vel_x > chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_ * Chassis::period_ * Chassis::period_);

    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kLegacy;
    chassis.updatePlannedMotionData();

    EXPECT_NEAR(chassis.planned_data_.vel_x, chassis.runtime_strategy_cfg_.max_acc_xy_acc_ * Chassis::period_, 1.0e-6f);
    EXPECT_NEAR(chassis.last_drive_omega_cmd_rad_s_[0], 0.0f, 1.0e-6f);
    EXPECT_TRUE(!chassis.trans_dir_freeze_active_);
    EXPECT_TRUE(chassis.trans_dir_ref_valid_);
    EXPECT_NEAR(chassis.trans_dir_ref_rad_, 0.0f, 1.0e-6f);
}

void testManualSCurveProfileStartsBrakingBeforeTargetWhenRemainingErrorIsTooSmallForCurrentAcceleration()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.manual_trans_acc_acc_ = 2.0f;
    chassis.runtime_strategy_cfg_.manual_trans_acc_dec_ = 3.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_ = 20.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_dec_ = 30.0f;

    Chassis::JerkLimitedAxisState axis_state{};
    axis_state.initialized = true;
    axis_state.shaped_value = 0.95f;
    axis_state.shaped_accel = 2.0f;

    float next_value = chassis.limitValueByJerkProfile(1.0f,
                                                       0.95f,
                                                       axis_state,
                                                       chassis.runtime_strategy_cfg_.manual_trans_acc_acc_,
                                                       chassis.runtime_strategy_cfg_.manual_trans_acc_dec_,
                                                       chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_,
                                                       chassis.runtime_strategy_cfg_.manual_trans_jerk_dec_,
                                                       chassis.runtime_strategy_cfg_.manual_trans_settle_vel_eps_,
                                                       chassis.runtime_strategy_cfg_.manual_trans_settle_acc_eps_);

    EXPECT_TRUE(next_value > 0.95f);
    EXPECT_TRUE(next_value < 0.952f);
    EXPECT_TRUE(axis_state.shaped_accel < 2.0f);
}

void testManualSCurveProfileSmallResidualClampClearsInternalAcceleration()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.manual_trans_acc_acc_ = 2.0f;
    chassis.runtime_strategy_cfg_.manual_trans_acc_dec_ = 3.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_ = 20.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_dec_ = 30.0f;

    Chassis::JerkLimitedAxisState axis_state{};
    axis_state.initialized = true;
    axis_state.shaped_value = 0.9996f;
    axis_state.shaped_accel = 0.5f;

    float next_value = chassis.limitValueByJerkProfile(1.0f,
                                                       0.9996f,
                                                       axis_state,
                                                       chassis.runtime_strategy_cfg_.manual_trans_acc_acc_,
                                                       chassis.runtime_strategy_cfg_.manual_trans_acc_dec_,
                                                       chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_,
                                                       chassis.runtime_strategy_cfg_.manual_trans_jerk_dec_,
                                                       chassis.runtime_strategy_cfg_.manual_trans_settle_vel_eps_,
                                                       chassis.runtime_strategy_cfg_.manual_trans_settle_acc_eps_);

    EXPECT_NEAR(next_value, 1.0f, 1.0e-6f);
    EXPECT_NEAR(axis_state.shaped_value, 1.0f, 1.0e-6f);
    EXPECT_NEAR(axis_state.shaped_accel, 0.0f, 1.0e-6f);
}

void testManualSCurveProfileManualOnlyModeFallsBackForApiSource()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kSCurve;
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = true;
    chassis.runtime_strategy_cfg_.manual_trans_acc_acc_ = 2.0f;
    chassis.runtime_strategy_cfg_.manual_trans_acc_dec_ = 3.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_ = 20.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_dec_ = 30.0f;
    chassis.runtime_strategy_cfg_.max_acc_xy_acc_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_acc_xy_dec_ = 3.0f;
    chassis.normalized_body_command_.source = Chassis::CommandInputSource::kApi;

    chassis.target_data_.vel_x = 1.0f;
    chassis.updatePlannedMotionData();

    EXPECT_TRUE(chassis.active_manual_speed_profile_mode_ == Chassis::ManualSpeedProfileMode::kLegacy);
    EXPECT_NEAR(chassis.planned_data_.vel_x, 2.0f * Chassis::period_, 1.0e-6f);
}

void testManualSCurveProfileManualOnlyModeUsesSCurveForDebugSource()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kSCurve;
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = true;
    chassis.runtime_strategy_cfg_.manual_trans_acc_acc_ = 2.0f;
    chassis.runtime_strategy_cfg_.manual_trans_acc_dec_ = 3.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_ = 20.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_dec_ = 30.0f;
    chassis.normalized_body_command_.source = Chassis::CommandInputSource::kDebugTarget;

    chassis.target_data_.vel_x = 1.0f;
    chassis.updatePlannedMotionData();

    EXPECT_TRUE(chassis.active_manual_speed_profile_mode_ == Chassis::ManualSpeedProfileMode::kSCurve);
    EXPECT_NEAR(chassis.planned_data_.vel_x, 20.0f * Chassis::period_ * Chassis::period_, 1.0e-6f);
}

void testManualSCurveProfileRapidReverseBleedsPositiveTrendBeforeBuildingNegativeTrend()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kSCurve;
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = false;
    chassis.runtime_strategy_cfg_.manual_trans_acc_acc_ = 2.0f;
    chassis.runtime_strategy_cfg_.manual_trans_acc_dec_ = 3.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_ = 20.0f;
    chassis.runtime_strategy_cfg_.manual_trans_jerk_dec_ = 30.0f;

    chassis.target_data_.vel_x = 1.0f;
    chassis.updatePlannedMotionData();
    chassis.last_planned_data_ = chassis.planned_data_;
    chassis.updatePlannedMotionData();
    const float forward_vel = chassis.planned_data_.vel_x;
    const float forward_acc = chassis.planned_data_.acc_x;

    chassis.last_planned_data_ = chassis.planned_data_;
    chassis.target_data_.vel_x = -1.0f;
    chassis.updatePlannedMotionData();
    const float first_reverse_vel = chassis.planned_data_.vel_x;
    const float first_reverse_acc = chassis.planned_data_.acc_x;

    chassis.last_planned_data_ = chassis.planned_data_;
    chassis.updatePlannedMotionData();

    EXPECT_TRUE(forward_vel > 0.0f);
    EXPECT_TRUE(forward_acc > 0.0f);
    EXPECT_TRUE(first_reverse_vel > 0.0f);
    EXPECT_TRUE(first_reverse_acc < forward_acc);
    EXPECT_TRUE(chassis.planned_data_.acc_x <= first_reverse_acc);
}

void testDebugBodySpeedOmegaTargetFlipsSignImmediatelyWithRightStickDirection()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.max_vel_x_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_vel_y_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_omega_z_ = 3.0f;
    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 1U;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = 0U;

    chassis.airjoy_data_.right_x = -1.0f;
    chassis.applyDebugTargetOverride(Chassis::DebugMode::kBodySpeed);
    EXPECT_NEAR(chassis.input_target_data_.omega_z, -3.0f, 1.0e-6f);

    chassis.airjoy_data_.right_x = 1.0f;
    chassis.applyDebugTargetOverride(Chassis::DebugMode::kBodySpeed);
    EXPECT_NEAR(chassis.input_target_data_.omega_z, 3.0f, 1.0e-6f);
}

void testDebugBodySpeedOmegaRapidReverseUnderSCurveKeepsOldSignForFirstPlannerStep()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kSCurve;
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = true;
    chassis.runtime_strategy_cfg_.manual_yaw_alpha_acc_ = 4.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_alpha_dec_ = 5.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_jerk_acc_ = 40.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_jerk_dec_ = 50.0f;
    chassis.runtime_strategy_cfg_.max_omega_z_ = 3.0f;
    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 1U;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = 0U;

    chassis.airjoy_data_.right_x = 1.0f;
    chassis.applyDebugTargetOverride(Chassis::DebugMode::kBodySpeed);
    chassis.setModeFlag();
    chassis.resolvePlannerTargetData();
    chassis.updatePlannedMotionData();
    chassis.last_planned_data_ = chassis.planned_data_;
    chassis.updatePlannedMotionData();
    const float forward_omega = chassis.planned_data_.omega_z;
    const float forward_alpha = chassis.planned_data_.alpha_z;

    chassis.airjoy_data_.right_x = -1.0f;
    chassis.applyDebugTargetOverride(Chassis::DebugMode::kBodySpeed);
    chassis.setModeFlag();
    chassis.resolvePlannerTargetData();
    EXPECT_NEAR(chassis.target_data_.omega_z, -3.0f, 1.0e-6f);
    chassis.updatePlannedMotionData();
    const float first_reverse_omega = chassis.planned_data_.omega_z;
    const float first_reverse_alpha = chassis.planned_data_.alpha_z;

    EXPECT_TRUE(forward_omega > 0.0f);
    EXPECT_TRUE(forward_alpha > 0.0f);
    EXPECT_TRUE(first_reverse_omega > 0.0f);
    EXPECT_TRUE(first_reverse_alpha < forward_alpha);
}

void testDebugBodySpeedOmegaRapidReverseEventuallyCrossesNegativeAfterEnoughCycles()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kSCurve;
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = true;
    chassis.runtime_strategy_cfg_.manual_yaw_alpha_acc_ = 5.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_alpha_dec_ = 12.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_jerk_acc_ = 50.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_jerk_dec_ = 50.0f;
    chassis.runtime_strategy_cfg_.max_omega_z_ = 3.0f;
    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 1U;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = 0U;

    chassis.airjoy_data_.right_x = 1.0f;
    for (int i = 0; i < 20; ++i)
    {
        runDebugPlannerCycleForHost(chassis);
    }
    EXPECT_TRUE(chassis.planned_data_.omega_z > 0.0f);

    chassis.airjoy_data_.right_x = -1.0f;
    bool crossed_negative = false;
    for (int i = 0; i < 400; ++i)
    {
        runDebugPlannerCycleForHost(chassis);
        if (chassis.planned_data_.omega_z < -1.0e-6f)
        {
            crossed_negative = true;
            break;
        }
    }

    EXPECT_TRUE(crossed_negative);
}

void testJerkProfileRapidReverseEventuallyCrossesZeroAndBuildsOppositeSign()
{
    Chassis chassis;
    Chassis::JerkLimitedAxisState axis_state{};
    float current_value = 0.0f;

    for (int i = 0; i < 20; ++i)
    {
        current_value = chassis.limitValueByJerkProfile(2.0f,
                                                        current_value,
                                                        axis_state,
                                                        5.0f,
                                                        12.0f,
                                                        50.0f,
                                                        50.0f,
                                                        1.0e-4f,
                                                        0.05f);
    }
    EXPECT_TRUE(current_value > 0.0f);

    bool crossed_negative = false;
    for (int i = 0; i < 400; ++i)
    {
        current_value = chassis.limitValueByJerkProfile(-2.0f,
                                                        current_value,
                                                        axis_state,
                                                        5.0f,
                                                        12.0f,
                                                        50.0f,
                                                        50.0f,
                                                        1.0e-4f,
                                                        0.05f);
        if (current_value < -1.0e-6f)
        {
            crossed_negative = true;
            break;
        }
    }

    EXPECT_TRUE(crossed_negative);
}

void testDebugBodySpeedOmegaRapidReverseEventuallyChangesPredictedActuatorOmegaSign()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kSCurve;
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = true;
    chassis.runtime_strategy_cfg_.manual_yaw_alpha_acc_ = 5.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_alpha_dec_ = 12.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_jerk_acc_ = 50.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_jerk_dec_ = 50.0f;
    chassis.runtime_strategy_cfg_.max_omega_z_ = 3.0f;
    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 1U;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = 0U;

    chassis.airjoy_data_.right_x = 1.0f;
    for (int i = 0; i < 20; ++i)
    {
        EXPECT_TRUE(runDebugControlCycleForHost(chassis));
    }
    EXPECT_TRUE(chassis.planned_data_.omega_z > 0.0f);

    float predicted_vel_x = 0.0f;
    float predicted_vel_y = 0.0f;
    float predicted_omega_z = 0.0f;
    EXPECT_TRUE(chassis.estimatePlannedBodyTwist(chassis.actuator_command_frame_.steer_oa_total_rad,
                                                 chassis.actuator_command_frame_.drive_omega_rad_s,
                                                 predicted_vel_x,
                                                 predicted_vel_y,
                                                 predicted_omega_z));
    const float baseline_predicted_omega_z = predicted_omega_z;
    EXPECT_TRUE(std::fabs(baseline_predicted_omega_z) > 1.0e-6f);

    chassis.airjoy_data_.right_x = -1.0f;
    int first_negative_cycle = -1;
    for (int i = 0; i < 1000; ++i)
    {
        EXPECT_TRUE(runDebugControlCycleForHost(chassis));
        if (!chassis.estimatePlannedBodyTwist(chassis.actuator_command_frame_.steer_oa_total_rad,
                                              chassis.actuator_command_frame_.drive_omega_rad_s,
                                              predicted_vel_x,
                                              predicted_vel_y,
                                              predicted_omega_z))
        {
            continue;
        }
        if (predicted_omega_z * baseline_predicted_omega_z < -1.0e-6f)
        {
            first_negative_cycle = i;
            break;
        }
    }

    EXPECT_TRUE(first_negative_cycle >= 0);
}

void testDebugBodySpeedTranslationRapidReverseEventuallyChangesPredictedActuatorDirection()
{
    auto run_axis_case = [](bool test_x_axis) {
        Chassis chassis;
        TestMotor steer_motors[4];
        VESC_Motor drive_motors[4];
        configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
        configureXParkWheelGeometry(chassis);

        chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kSCurve;
        chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = true;
        chassis.runtime_strategy_cfg_.manual_trans_acc_acc_ = 5.0f;
        chassis.runtime_strategy_cfg_.manual_trans_acc_dec_ = 12.0f;
        chassis.runtime_strategy_cfg_.manual_trans_jerk_acc_ = 50.0f;
        chassis.runtime_strategy_cfg_.manual_trans_jerk_dec_ = 50.0f;
        chassis.runtime_strategy_cfg_.max_vel_x_ = 2.0f;
        chassis.runtime_strategy_cfg_.max_vel_y_ = 2.0f;
        chassis.debug_control_.common.enable = true;
        chassis.debug_control_.common.mode_raw = 1U;
        chassis.debug_control_.injection.omega_z_injection_mode_raw = 0U;

        float predicted_vel_x = 0.0f;
        float predicted_vel_y = 0.0f;
        float predicted_omega_z = 0.0f;
        chassis.airjoy_data_.left_y = test_x_axis ? 1.0f : 0.0f;
        chassis.airjoy_data_.left_x = test_x_axis ? 0.0f : -1.0f;
        float baseline_axis_value = 0.0f;
        bool established_baseline = false;
        for (int i = 0; i < 1000; ++i)
        {
            EXPECT_TRUE(runDebugControlCycleForHost(chassis));
            if (!chassis.estimatePlannedBodyTwist(chassis.actuator_command_frame_.steer_oa_total_rad,
                                                  chassis.actuator_command_frame_.drive_omega_rad_s,
                                                  predicted_vel_x,
                                                  predicted_vel_y,
                                                  predicted_omega_z))
            {
                continue;
            }

            baseline_axis_value = test_x_axis ? predicted_vel_x : predicted_vel_y;
            if (std::fabs(baseline_axis_value) > 1.0e-6f)
            {
                established_baseline = true;
                break;
            }
        }
        EXPECT_TRUE(established_baseline);

        chassis.airjoy_data_.left_y = test_x_axis ? -1.0f : 0.0f;
        chassis.airjoy_data_.left_x = test_x_axis ? 0.0f : 1.0f;
        int first_reversed_cycle = -1;
        for (int i = 0; i < 1000; ++i)
        {
            EXPECT_TRUE(runDebugControlCycleForHost(chassis));
            if (!chassis.estimatePlannedBodyTwist(chassis.actuator_command_frame_.steer_oa_total_rad,
                                                  chassis.actuator_command_frame_.drive_omega_rad_s,
                                                  predicted_vel_x,
                                                  predicted_vel_y,
                                                  predicted_omega_z))
            {
                continue;
            }

            const float current_axis_value = test_x_axis ? predicted_vel_x : predicted_vel_y;
            if (current_axis_value * baseline_axis_value < -1.0e-6f)
            {
                first_reversed_cycle = i;
                break;
            }
        }

        EXPECT_TRUE(first_reversed_cycle >= 0);
    };

    run_axis_case(true);
    run_axis_case(false);
}

void testDriveOmegaPlannerLimitUsesUniformScaleAcrossAllWheels()
{
    Chassis chassis;
    configureXParkWheelGeometry(chassis);
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_omega_limit_ = true;
    chassis.runtime_strategy_cfg_.max_drive_omega_rad_s_ = 10.0f;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
    }

    Chassis::Data command{};
    command.vel_x = 1.0f;
    command.omega_z = 1.0f;

    const Chassis::SwervePlannerOutput output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));

    float max_abs_projected = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        max_abs_projected = std::max(max_abs_projected, std::fabs(output.projected_drive_omega_rad_s[i]));
    }

    const float expected_scale = chassis.runtime_strategy_cfg_.max_drive_omega_rad_s_ / max_abs_projected;
    EXPECT_NEAR(std::fabs(output.final_drive_omega_rad_s[0]), std::fabs(output.projected_drive_omega_rad_s[0]) * expected_scale, 1.0e-4f);
    EXPECT_NEAR(std::fabs(output.final_drive_omega_rad_s[1]), std::fabs(output.projected_drive_omega_rad_s[1]) * expected_scale, 1.0e-4f);
    EXPECT_NEAR(std::fabs(output.final_drive_omega_rad_s[2]), std::fabs(output.projected_drive_omega_rad_s[2]) * expected_scale, 1.0e-4f);
    EXPECT_NEAR(std::fabs(output.final_drive_omega_rad_s[3]), std::fabs(output.projected_drive_omega_rad_s[3]) * expected_scale, 1.0e-4f);
}

void testFlipSolutionPrefersSmallSteerDeltaAndInvertsDriveOnQuadrantCrossing()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.steering_strategy_mode = Chassis::SteeringStrategyMode::kShortestPath;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = jia::degToRadF32(10.0f);
    }

    Chassis::Data command{};
    command.vel_x = -1.0f;
    command.vel_y = 0.20f;

    const Chassis::SwervePlannerOutput output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));

    const float direct_target_rad = std::atan2(command.vel_y, command.vel_x);
    const float direct_delta_rad = std::fabs(jia::shortestAngularDistanceF32(jia::degToRadF32(10.0f), direct_target_rad));

    EXPECT_TRUE(output.flipped_drive_direction[0]);
    EXPECT_TRUE(output.steering_errors_rad[0] < direct_delta_rad);
    EXPECT_TRUE(output.projected_drive_omega_rad_s[0] < 0.0f);
    EXPECT_TRUE(output.final_drive_omega_rad_s[0] < 0.0f);
}

void testReverseIntentBypassesTranslationalDirectionSlewOnNearOppositeCommand()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.trans_dir_rate_limit_deg_s_ = 10.0f;
    chassis.runtime_strategy_cfg_.max_acc_xy_acc_ = 1000.0f;
    chassis.runtime_strategy_cfg_.max_acc_xy_dec_ = 1000.0f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.05f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.10f;
    chassis.last_planned_data_.vel_x = 1.0f;
    chassis.last_planned_data_.vel_y = 0.0f;
    chassis.trans_dir_ref_valid_ = true;
    chassis.trans_dir_ref_rad_ = 0.0f;
    chassis.trans_dir_freeze_active_ = false;

    float out_vel_x = 0.0f;
    float out_vel_y = 0.0f;
    float out_omega_z = 0.0f;
    chassis.limitPlannedSpeed(-1.0f, 0.20f, 0.0f, out_vel_x, out_vel_y, out_omega_z);

    EXPECT_TRUE(out_vel_x < 0.0f);
    EXPECT_TRUE(std::atan2(out_vel_y, out_vel_x) > (jia::kPi / 2.0f));
}

void testDirectionSlewCrossesPiBoundaryByShortestPath()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.trans_dir_rate_limit_deg_s_ = 30.0f;
    chassis.runtime_strategy_cfg_.max_acc_xy_acc_ = 1000.0f;
    chassis.runtime_strategy_cfg_.max_acc_xy_dec_ = 1000.0f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.02f;
    chassis.trans_dir_ref_valid_ = true;
    chassis.trans_dir_ref_rad_ = jia::degToRadF32(179.0f);
    chassis.trans_dir_freeze_active_ = false;

    float out_vel_x = -1.0f;
    float out_vel_y = -0.01f;
    float out_omega_z = 0.0f;
    chassis.limitPlannedSpeed(-1.0f, -0.01f, 0.0f, out_vel_x, out_vel_y, out_omega_z);

    const float output_dir_deg = jia::radToDegF32(std::atan2(out_vel_y, out_vel_x));
    EXPECT_TRUE(output_dir_deg > 170.0f || output_dir_deg < -170.0f);
    EXPECT_TRUE(std::fabs(jia::shortestAngularDistanceF32(chassis.trans_dir_ref_rad_, jia::degToRadF32(-179.0f))) < jia::degToRadF32(10.0f));
}

void testReverseIntentDoesNotFallIntoZeroHoldOrSuppressionWhenSteerIsReachable()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg = 25.0f;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.min_scale = 0.0f;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.05f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.10f;
    chassis.last_planned_data_.vel_x = 1.0f;
    chassis.last_planned_data_.vel_y = 0.0f;
    chassis.trans_dir_ref_valid_ = true;
    chassis.trans_dir_ref_rad_ = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = jia::degToRadF32(10.0f);
    }

    Chassis::Data command{};
    command.vel_x = -0.16f;
    command.vel_y = 0.02f;

    const Chassis::SwervePlannerInput input = chassis.makeSwervePlannerInput(command);
    const Chassis::SwervePlannerOutput output = chassis.planSwerveModules(input);

    EXPECT_TRUE(!input.command_stationary_intent);
    EXPECT_TRUE(output.flipped_drive_direction[0]);
    EXPECT_NEAR(output.low_speed_suppression_scale[0], 1.0f, 1.0e-6f);
    EXPECT_TRUE(output.final_drive_omega_rad_s[0] < 0.0f);
}

void testAlwaysForwardModeIgnoresReverseIntentOverride()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.steering_strategy_mode = Chassis::SteeringStrategyMode::kAlwaysForward;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = jia::degToRadF32(10.0f);
    }

    Chassis::Data command{};
    command.vel_x = -1.0f;
    command.vel_y = 0.20f;

    const Chassis::SwervePlannerOutput output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));

    EXPECT_TRUE(!output.flipped_drive_direction[0]);
    EXPECT_TRUE(output.projected_drive_omega_rad_s[0] > 0.0f);
}

void testDriveAlphaDeliveryLimitUsesUniformScaleAcrossAllWheels()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveContinuityHarness(chassis, drive_motors);
    chassis.runtime_strategy_cfg_.max_drive_alpha_rad_s2_ = 1000.0f;

    const Chassis::SwervePlannerOutput planner_output = makeNeutralPlannerOutput();
    const Chassis::ActuatorCommandFrame command_frame = makePerWheelDriveCommandFrame(4.0f, 2.0f, -1.0f, 0.0f);

    chassis.storePlannedActuatorFrame(planner_output, command_frame);
    chassis.applyModuleCommands(true);

    EXPECT_NEAR(chassis.wheel_config_[0].target_drive_omega_rad_s, 1.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[1].target_drive_omega_rad_s, 0.5f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[2].target_drive_omega_rad_s, -0.25f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[3].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
}

void testDriveAlphaDeliveryLimitUsesWorstWheelScaleWhenLastValuesDiffer()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveContinuityHarness(chassis, drive_motors);
    chassis.runtime_strategy_cfg_.max_drive_alpha_rad_s2_ = 1000.0f;

    chassis.last_drive_omega_cmd_rad_s_[0] = 3.0f;
    chassis.last_drive_omega_cmd_rad_s_[1] = 1.0f;
    chassis.last_drive_omega_cmd_rad_s_[2] = -0.5f;
    chassis.last_drive_omega_cmd_rad_s_[3] = 0.0f;

    const Chassis::SwervePlannerOutput planner_output = makeNeutralPlannerOutput();
    const Chassis::ActuatorCommandFrame command_frame = makePerWheelDriveCommandFrame(4.0f, 2.0f, -1.0f, 0.0f);

    chassis.storePlannedActuatorFrame(planner_output, command_frame);
    chassis.applyModuleCommands(true);

    EXPECT_NEAR(chassis.wheel_config_[0].target_drive_omega_rad_s, 4.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[1].target_drive_omega_rad_s, 2.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[2].target_drive_omega_rad_s, -1.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[3].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
}

void testDriveAlphaDeliveryLimitUsesUniformScaleWhileNotHomed()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveContinuityHarness(chassis, drive_motors);
    chassis.runtime_strategy_cfg_.max_drive_alpha_rad_s2_ = 1000.0f;

    chassis.last_drive_omega_cmd_rad_s_[0] = 4.0f;
    chassis.last_drive_omega_cmd_rad_s_[1] = 2.0f;
    chassis.last_drive_omega_cmd_rad_s_[2] = -1.0f;
    chassis.last_drive_omega_cmd_rad_s_[3] = 0.0f;

    const Chassis::SwervePlannerOutput planner_output = makeNeutralPlannerOutput();
    const Chassis::ActuatorCommandFrame command_frame = makePerWheelDriveCommandFrame(4.0f, 2.0f, -1.0f, 0.0f);

    chassis.storePlannedActuatorFrame(planner_output, command_frame);
    chassis.applyModuleCommands(false);

    EXPECT_NEAR(chassis.wheel_config_[0].target_drive_omega_rad_s, 3.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[1].target_drive_omega_rad_s, 1.5f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[2].target_drive_omega_rad_s, -0.75f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[3].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
}

void testDriveDeliveryLimitAlphaThenOmegaStillKeepsSharedProgress()
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveContinuityHarness(chassis, drive_motors);
    chassis.runtime_strategy_cfg_.max_drive_alpha_rad_s2_ = 3000.0f;
    chassis.runtime_strategy_cfg_.enable_drive_omega_limit_ = true;
    chassis.runtime_strategy_cfg_.max_drive_omega_rad_s_ = 0.75f;

    const Chassis::SwervePlannerOutput planner_output = makeNeutralPlannerOutput();
    const Chassis::ActuatorCommandFrame command_frame = makePerWheelDriveCommandFrame(4.0f, 2.0f, -1.0f, 0.0f);

    chassis.storePlannedActuatorFrame(planner_output, command_frame);
    chassis.applyModuleCommands(true);

    EXPECT_NEAR(chassis.wheel_config_[0].target_drive_omega_rad_s, 0.75f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[1].target_drive_omega_rad_s, 0.75f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[2].target_drive_omega_rad_s, -0.75f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[3].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
}

void testXParkActivatesOnlyAfterActualResidualWheelSpeedHoldsForEntryDelay()
{
    Chassis chassis;
    configureXParkTriggerHarness(chassis);

    Chassis::Data command{};
    command.vel_x = 0.005f;
    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 4.0f;
    }

    Chassis::SwervePlannerInput planner_input{};
    for (jia::u32 i = 0; i < chassis.runtime_strategy_cfg_.xpark_entry_delay_ms - 1U; ++i)
    {
        planner_input = chassis.makeSwervePlannerInput(command);
        EXPECT_TRUE(!chassis.xpark_gate_active_);
        EXPECT_TRUE(!planner_input.allow_xpark_pose);
    }

    EXPECT_NEAR(static_cast<float>(chassis.xpark_stationary_hold_ms_), 0.0f, 1.0e-6f);

    planner_input = chassis.makeSwervePlannerInput(command);
    EXPECT_TRUE(!chassis.xpark_gate_active_);
    EXPECT_TRUE(!planner_input.allow_xpark_pose);

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
    }

    for (jia::u32 i = 0; i < chassis.runtime_strategy_cfg_.xpark_entry_delay_ms - 1U; ++i)
    {
        planner_input = chassis.makeSwervePlannerInput(command);
        EXPECT_TRUE(!chassis.xpark_gate_active_);
        EXPECT_TRUE(!planner_input.allow_xpark_pose);
    }

    EXPECT_NEAR(static_cast<float>(chassis.xpark_stationary_hold_ms_),
                static_cast<float>(chassis.runtime_strategy_cfg_.xpark_entry_delay_ms - 1U),
                1.0e-6f);

    planner_input = chassis.makeSwervePlannerInput(command);
    EXPECT_TRUE(chassis.xpark_gate_active_);
    EXPECT_TRUE(planner_input.allow_xpark_pose);
}

void testXParkHoldCounterResetsImmediatelyWhenCommandWheelSpeedExitsThreshold()
{
    Chassis chassis;
    configureXParkTriggerHarness(chassis);

    Chassis::Data slow_command{};
    slow_command.vel_x = 0.005f;
    for (jia::u32 i = 0; i < chassis.runtime_strategy_cfg_.xpark_entry_delay_ms - 1U; ++i)
    {
        chassis.makeSwervePlannerInput(slow_command);
    }

    EXPECT_NEAR(static_cast<float>(chassis.xpark_stationary_hold_ms_),
                static_cast<float>(chassis.runtime_strategy_cfg_.xpark_entry_delay_ms - 1U),
                1.0e-6f);
    EXPECT_TRUE(!chassis.xpark_gate_active_);

    Chassis::Data moving_command{};
    moving_command.vel_x = 0.06f;
    Chassis::SwervePlannerInput planner_input = chassis.makeSwervePlannerInput(moving_command);

    EXPECT_TRUE(!planner_input.command_stationary_intent);
    EXPECT_NEAR(static_cast<float>(chassis.xpark_stationary_hold_ms_), 0.0f, 1.0e-6f);
    EXPECT_TRUE(!chassis.xpark_gate_active_);
    EXPECT_TRUE(!planner_input.allow_xpark_pose);
}

void testXParkResidualWheelSpeedAboveThresholdKeepsGateClosed()
{
    Chassis chassis;
    configureXParkTriggerHarness(chassis);

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 4.0f;
    }

    Chassis::Data command{};
    command.vel_x = 0.005f;

    Chassis::SwervePlannerInput planner_input{};
    for (jia::u32 i = 0; i < chassis.runtime_strategy_cfg_.xpark_entry_delay_ms; ++i)
    {
        planner_input = chassis.makeSwervePlannerInput(command);
    }

    EXPECT_TRUE(!chassis.xpark_gate_active_);
    EXPECT_TRUE(!planner_input.allow_xpark_pose);
    EXPECT_TRUE(planner_input.max_residual_speed_m_s > chassis.getNearZeroExitSpeedMps());
}

void testXParkCommandThresholdTreatsPointZeroFiveAsMovingDespiteLargeResidualFilter()
{
    Chassis chassis;
    configureXParkTriggerHarness(chassis);
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.10f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.15f;

    Chassis::Data command{};
    command.vel_x = 0.05f;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 1.6f;
    }

    const Chassis::SwervePlannerInput planner_input = chassis.makeSwervePlannerInput(command);

    EXPECT_TRUE(!planner_input.command_stationary_intent);
    EXPECT_TRUE(!chassis.xpark_gate_active_);
    EXPECT_TRUE(!planner_input.allow_xpark_pose);
    EXPECT_TRUE(planner_input.max_residual_speed_m_s < chassis.getNearZeroEnterSpeedMps());
}

void testXParkResidualThresholdStillBlocksEntryWhenCommandIsStationary()
{
    Chassis chassis;
    configureXParkTriggerHarness(chassis);
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.10f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.15f;

    Chassis::Data command{};
    command.vel_x = 0.005f;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 2.4f;
    }

    Chassis::SwervePlannerInput planner_input{};
    for (jia::u32 i = 0; i < chassis.runtime_strategy_cfg_.xpark_entry_delay_ms; ++i)
    {
        planner_input = chassis.makeSwervePlannerInput(command);
    }

    EXPECT_TRUE(!planner_input.command_stationary_intent);
    EXPECT_TRUE(!chassis.xpark_gate_active_);
    EXPECT_TRUE(!planner_input.allow_xpark_pose);
    EXPECT_TRUE(planner_input.max_residual_speed_m_s > chassis.getNearZeroEnterSpeedMps());
}

void testXParkUsesIndependentCommandThresholdWhileResidualMayUseLargeNearZeroFilter()
{
    Chassis chassis;
    configureXParkTriggerHarness(chassis);
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.10f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.15f;

    Chassis::Data command{};
    command.vel_x = 0.005f;

    Chassis::SwervePlannerInput planner_input{};
    for (jia::u32 i = 0; i < chassis.runtime_strategy_cfg_.xpark_entry_delay_ms - 1U; ++i)
    {
        planner_input = chassis.makeSwervePlannerInput(command);
        EXPECT_TRUE(!chassis.xpark_gate_active_);
        EXPECT_TRUE(!planner_input.allow_xpark_pose);
    }

    EXPECT_TRUE(planner_input.command_stationary_intent);
    EXPECT_NEAR(static_cast<float>(chassis.xpark_stationary_hold_ms_),
                static_cast<float>(chassis.runtime_strategy_cfg_.xpark_entry_delay_ms - 1U),
                1.0e-6f);

    planner_input = chassis.makeSwervePlannerInput(command);
    EXPECT_TRUE(planner_input.command_stationary_intent);
    EXPECT_TRUE(chassis.xpark_gate_active_);
    EXPECT_TRUE(planner_input.allow_xpark_pose);
}

void testXParkAllowsEntryWhenBothCommandAndResidualAreBelowTheirOwnThresholds()
{
    Chassis chassis;
    configureXParkTriggerHarness(chassis);
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.10f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.15f;

    Chassis::Data command{};
    command.vel_x = 0.005f;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 1.0f;
    }

    Chassis::SwervePlannerInput planner_input{};
    for (jia::u32 i = 0; i < chassis.runtime_strategy_cfg_.xpark_entry_delay_ms - 1U; ++i)
    {
        planner_input = chassis.makeSwervePlannerInput(command);
        EXPECT_TRUE(planner_input.command_stationary_intent);
        EXPECT_TRUE(!chassis.xpark_gate_active_);
        EXPECT_TRUE(!planner_input.allow_xpark_pose);
    }

    planner_input = chassis.makeSwervePlannerInput(command);
    EXPECT_TRUE(planner_input.command_stationary_intent);
    EXPECT_TRUE(chassis.xpark_gate_active_);
    EXPECT_TRUE(planner_input.allow_xpark_pose);
    EXPECT_TRUE(planner_input.max_residual_speed_m_s < chassis.getNearZeroEnterSpeedMps());
}

void testXParkDoesNotEnterWhenCommandExceedsDedicatedCommandThreshold()
{
    Chassis chassis;
    configureXParkTriggerHarness(chassis);
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.10f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.15f;

    Chassis::Data command{};
    command.vel_x = 0.06f;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 1.0f;
    }

    Chassis::SwervePlannerInput planner_input{};
    for (jia::u32 i = 0; i < chassis.runtime_strategy_cfg_.xpark_entry_delay_ms; ++i)
    {
        planner_input = chassis.makeSwervePlannerInput(command);
    }

    EXPECT_TRUE(!planner_input.command_stationary_intent);
    EXPECT_TRUE(!chassis.xpark_gate_active_);
    EXPECT_TRUE(!planner_input.allow_xpark_pose);
    EXPECT_TRUE(planner_input.max_residual_speed_m_s < chassis.getNearZeroEnterSpeedMps());
}

void testXParkSteerDeadbandFreezesWheelWhenErrorEntersOneDegreeBand()
{
    XParkSteerDeadbandHarness harness;
    configureXParkSteerDeadbandHarness(harness);
    Chassis &chassis = harness.chassis;

    const float xpark_target_oa_rad = getWheelXParkTargetOaRad(chassis, 0);
    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.5f));

    const Chassis::SwervePlannerOutput planner_output = makeNeutralPlannerOutput();
    const Chassis::ActuatorCommandFrame command_frame = makeXParkSteerCommandFrame(chassis);
    chassis.storePlannedActuatorFrame(planner_output, command_frame);

    const float current_corrected_local_total_rad = chassis.wheel_config_[0].corrected_steer_motor_total_angle_rad;
    const float planned_xpark_corrected_local_total_rad = command_frame.steer_corrected_local_total_rad[0];

    chassis.applyModuleCommands(true);

    EXPECT_NEAR(chassis.wheel_config_[0].target_steer_motor_total_angle_rad, current_corrected_local_total_rad, 1.0e-6f);
    EXPECT_TRUE(std::fabs(chassis.wheel_config_[0].target_steer_motor_total_angle_rad - planned_xpark_corrected_local_total_rad) > 1.0e-4f);
    EXPECT_TRUE(chassis.wheel_config_[0].steer_speed_pid_settled_active);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kCurrent);
    EXPECT_NEAR(harness.steer_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
}

void testXParkSteerDeadbandStaysLatchedInsideThreeDegreeExitBand()
{
    XParkSteerDeadbandHarness harness;
    configureXParkSteerDeadbandHarness(harness);
    Chassis &chassis = harness.chassis;

    const float xpark_target_oa_rad = getWheelXParkTargetOaRad(chassis, 0);
    const Chassis::SwervePlannerOutput planner_output = makeNeutralPlannerOutput();
    const Chassis::ActuatorCommandFrame command_frame = makeXParkSteerCommandFrame(chassis);
    chassis.storePlannedActuatorFrame(planner_output, command_frame);

    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.5f));
    chassis.applyModuleCommands(true);
    EXPECT_NEAR(chassis.wheel_config_[0].target_steer_motor_total_angle_rad,
                chassis.wheel_config_[0].corrected_steer_motor_total_angle_rad,
                1.0e-6f);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kCurrent);
    EXPECT_NEAR(harness.steer_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);

    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(2.0f));
    chassis.applyModuleCommands(true);

    EXPECT_NEAR(chassis.wheel_config_[0].target_steer_motor_total_angle_rad,
                chassis.wheel_config_[0].corrected_steer_motor_total_angle_rad,
                1.0e-6f);
    EXPECT_TRUE(chassis.wheel_config_[0].steer_speed_pid_settled_active);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kCurrent);
    EXPECT_NEAR(harness.steer_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
}

void testXParkSteerDeadbandReleasesOnceErrorExceedsThreeDegreeExitBand()
{
    XParkSteerDeadbandHarness harness;
    configureXParkSteerDeadbandHarness(harness);
    Chassis &chassis = harness.chassis;

    const float xpark_target_oa_rad = getWheelXParkTargetOaRad(chassis, 0);
    const Chassis::SwervePlannerOutput planner_output = makeNeutralPlannerOutput();
    const Chassis::ActuatorCommandFrame command_frame = makeXParkSteerCommandFrame(chassis);
    chassis.storePlannedActuatorFrame(planner_output, command_frame);

    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.5f));
    chassis.applyModuleCommands(true);
    EXPECT_NEAR(chassis.wheel_config_[0].target_steer_motor_total_angle_rad,
                chassis.wheel_config_[0].corrected_steer_motor_total_angle_rad,
                1.0e-6f);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kCurrent);
    EXPECT_NEAR(harness.steer_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);

    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(4.0f));
    chassis.applyModuleCommands(true);

    EXPECT_NEAR(chassis.wheel_config_[0].target_steer_motor_total_angle_rad,
                command_frame.steer_corrected_local_total_rad[0],
                1.0e-6f);
    EXPECT_TRUE(!chassis.wheel_config_[0].steer_speed_pid_settled_active);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kTotalAngle);
    EXPECT_NEAR(harness.steer_motors[0].getTargetTotalAngle(),
                jia::radToDegF32(command_frame.steer_corrected_local_total_rad[0]),
                1.0e-6f);
}

void testNonXParkSteerCommandDoesNotTriggerSteerDeadbandFreeze()
{
    XParkSteerDeadbandHarness harness;
    configureXParkSteerDeadbandHarness(harness);
    Chassis &chassis = harness.chassis;

    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kHoldLast;
    const float current_oa_rad = getWheelXParkTargetOaRad(chassis, 0) - jia::degToRadF32(0.5f);
    setWheelOaAngleRad(chassis, 0, current_oa_rad);

    Chassis::ActuatorCommandFrame command_frame = makeXParkSteerCommandFrame(chassis);
    command_frame.steer_oa_total_rad[0] = current_oa_rad + jia::degToRadF32(0.5f);
    command_frame.steer_corrected_local_total_rad[0] =
        chassis.mapWheelOaTotalToCorrectedLocal(chassis.wheel_config_[0], command_frame.steer_oa_total_rad[0]);

    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), command_frame);
    chassis.applyModuleCommands(true);

    EXPECT_NEAR(chassis.wheel_config_[0].target_steer_motor_total_angle_rad,
                command_frame.steer_corrected_local_total_rad[0],
                1.0e-6f);
    EXPECT_TRUE(!chassis.wheel_config_[0].xpark_steer_deadband_active);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kTotalAngle);
    EXPECT_TRUE(harness.steer_motors[0].getTargetCurrent() == 0.0f);
}

void testSteerSettleResetTriggersSingleSpeedPidResetOnEnter()
{
    SteerSettleResetHarness harness;
    configureSteerSettleResetHarness(harness);

    harness.chassis.wheel_config_[0].corrected_steer_motor_total_angle_rad = jia::degToRadF32(9.5f);
    const Chassis::ActuatorCommandFrame command_frame = makeSteerSettleCommandFrame(harness.chassis, 10.0f, 2.0f);
    harness.chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), command_frame);

    harness.chassis.applyModuleCommands(true);

    EXPECT_TRUE(harness.chassis.wheel_config_[0].steer_speed_pid_settled_active);
    EXPECT_TRUE(harness.steer_motors[0].getResetSpeedPidStateCallCount() == 1U);
}

void testSteerSettleResetDoesNotRetriggerWhileInsideExitBand()
{
    SteerSettleResetHarness harness;
    configureSteerSettleResetHarness(harness);

    harness.chassis.wheel_config_[0].corrected_steer_motor_total_angle_rad = jia::degToRadF32(9.5f);
    harness.chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(),
                                              makeSteerSettleCommandFrame(harness.chassis, 10.0f, 2.0f));
    harness.chassis.applyModuleCommands(true);

    harness.chassis.wheel_config_[0].corrected_steer_motor_total_angle_rad = jia::degToRadF32(8.0f);
    harness.chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(),
                                              makeSteerSettleCommandFrame(harness.chassis, 10.0f, 10.0f));
    harness.chassis.applyModuleCommands(true);

    EXPECT_TRUE(harness.chassis.wheel_config_[0].steer_speed_pid_settled_active);
    EXPECT_TRUE(harness.steer_motors[0].getResetSpeedPidStateCallCount() == 1U);
}

void testSteerSettleResetRetriggersAfterLeavingAndReEnteringBand()
{
    SteerSettleResetHarness harness;
    configureSteerSettleResetHarness(harness);

    harness.chassis.wheel_config_[0].corrected_steer_motor_total_angle_rad = jia::degToRadF32(9.5f);
    harness.chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(),
                                              makeSteerSettleCommandFrame(harness.chassis, 10.0f, 2.0f));
    harness.chassis.applyModuleCommands(true);

    harness.chassis.wheel_config_[0].corrected_steer_motor_total_angle_rad = jia::degToRadF32(6.0f);
    harness.chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(),
                                              makeSteerSettleCommandFrame(harness.chassis, 10.0f, 2.0f));
    harness.chassis.applyModuleCommands(true);

    harness.chassis.wheel_config_[0].corrected_steer_motor_total_angle_rad = jia::degToRadF32(9.5f);
    harness.chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(),
                                              makeSteerSettleCommandFrame(harness.chassis, 10.0f, 2.0f));
    harness.chassis.applyModuleCommands(true);

    EXPECT_TRUE(harness.chassis.wheel_config_[0].steer_speed_pid_settled_active);
    EXPECT_TRUE(harness.steer_motors[0].getResetSpeedPidStateCallCount() == 2U);
}

void testSteerSettleResetDoesNotTriggerDuringZeroCurrentBranch()
{
    SteerSettleResetHarness harness;
    configureSteerSettleResetHarness(harness);

    harness.chassis.input_target_data_.zero_current_all = true;
    harness.chassis.wheel_config_[0].corrected_steer_motor_total_angle_rad = jia::degToRadF32(9.5f);
    harness.chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(),
                                              makeSteerSettleCommandFrame(harness.chassis, 10.0f, 2.0f));

    harness.chassis.applyModuleCommands(true);

    EXPECT_TRUE(!harness.chassis.wheel_config_[0].steer_speed_pid_settled_active);
    EXPECT_TRUE(harness.steer_motors[0].getResetSpeedPidStateCallCount() == 0U);
}

void testStationaryPhotogateTogglesDoNotSelfLockNormalReadyState()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.target_data_.vel_x = 0.0f;
    chassis.target_data_.vel_y = 0.0f;
    chassis.target_data_.omega_z = 0.0f;

    for (int cycle = 0; cycle < 6; ++cycle)
    {
        setPhotogateStateForWheel(0, (cycle % 2) != 0);
        const bool all_homed = runHostControlCycle(chassis);
        EXPECT_TRUE(all_homed);
        EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kReady);
        EXPECT_NEAR(drive_motors[0].getTargetRPM(), 0.0f, 1.0e-6f);
        EXPECT_NEAR(steer_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
    }
}

void testReadyStationaryWheelsCanStillEnterXParkWithoutTriggeringSteerFault()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.xpark_entry_delay_ms = 5U;
    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kXPark;
    chassis.input_target_data_.vel_x = 0.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
        steer_motors[i].setFeedbackCurrent(120.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
    }

    bool xpark_armed = false;
    for (int cycle = 0; cycle < 12; ++cycle)
    {
        const bool all_homed = runHostControlCycle(chassis);
        EXPECT_TRUE(all_homed);
        for (int i = 0; i < 4; ++i)
        {
            EXPECT_TRUE(chassis.wheel_config_[i].homing_state == Chassis::HomingState::kReady);
            EXPECT_TRUE(!chassis.debug_mirror_.steer_fault_active[i]);
        }
        if (chassis.xpark_gate_active_)
        {
            xpark_armed = true;
            break;
        }
    }

    EXPECT_TRUE(xpark_armed);
    EXPECT_TRUE(chassis.xpark_gate_active_);
    EXPECT_TRUE(std::fabs(chassis.debug_mirror_.target_oa_deg[0]) > 1.0e-3f);
    EXPECT_NEAR(drive_motors[0].getTargetRPM(), 0.0f, 1.0e-6f);
}

void testSingleWheelSteerFreezeFaultStopsVehicleAndFreezesFaultedWheelPath()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 2) ? 6000.0f : 200.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
        drive_motors[i].setTargetCurrent(777.0f);
    }

    bool saw_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        const bool all_homed = runHostControlCycle(chassis);
        if (chassis.wheel_config_[2].homing_state == Chassis::HomingState::kFault)
        {
            saw_fault = true;
            EXPECT_TRUE(!all_homed);
            EXPECT_NEAR(chassis.current_data_.vel_x, 0.0f, 1.0e-6f);
            EXPECT_NEAR(chassis.current_data_.vel_y, 0.0f, 1.0e-6f);
            EXPECT_NEAR(chassis.current_data_.omega_z, 0.0f, 1.0e-6f);
            for (int i = 0; i < 4; ++i)
            {
                EXPECT_NEAR(chassis.wheel_config_[i].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
                EXPECT_NEAR(drive_motors[i].getTargetCurrent(), 0.0f, 1.0e-6f);
            }
            EXPECT_NEAR(steer_motors[2].getTargetCurrent(), 0.0f, 1.0e-6f);
            EXPECT_NEAR(chassis.wheel_config_[2].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
            EXPECT_NEAR(chassis.wheel_config_[2].target_steer_motor_total_angle_rad, 0.0f, 1.0e-6f);
            break;
        }
    }

    EXPECT_TRUE(saw_fault);
}

void testSingleWheelSteerFreezeFaultDoesNotRequireHugeCurrentMagnitude()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 0) ? 600.0f : 20.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
    }

    bool saw_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        const bool all_homed = runHostControlCycle(chassis);
        if (chassis.wheel_config_[0].homing_state == Chassis::HomingState::kFault)
        {
            saw_fault = true;
            EXPECT_TRUE(!all_homed);
            EXPECT_TRUE(chassis.debug_mirror_.steer_fault_active[0]);
            EXPECT_NEAR(chassis.debug_mirror_.steer_feedback_current_mA[0], 600.0f, 1.0e-6f);
            EXPECT_TRUE(chassis.debug_mirror_.steer_feedback_current_freeze_ms[0] >= 5.0f);
            EXPECT_NEAR(drive_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
            break;
        }
    }

    EXPECT_TRUE(saw_fault);
}

void testSteerFaultThresholdsAreDebugTunableAndMirrorPublishesDecisionInputs()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = true;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.active_current_min_mA = 700.0f;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.freeze_duration_ms = 5U;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 0) ? 600.0f : 20.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
    }

    for (int cycle = 0; cycle < 8; ++cycle)
    {
        runHostControlCycle(chassis);
    }

    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kReady);
    EXPECT_TRUE(!chassis.debug_mirror_.steer_fault_active[0]);
    EXPECT_TRUE(chassis.debug_mirror_.steer_fault_control_intent[0]);
    EXPECT_TRUE(!chassis.debug_mirror_.steer_fault_freeze_candidate[0]);
    EXPECT_NEAR(chassis.debug_mirror_.steer_feedback_current_mA[0], 600.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.debug_mirror_.steer_feedback_current_delta_mA[0], 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.debug_mirror_.steer_feedback_angle_delta_rad[0], 0.0f, 1.0e-6f);
    EXPECT_TRUE(chassis.debug_mirror_.steer_feedback_current_freeze_ms[0] < 5.0f);

    chassis.runtime_strategy_cfg_.steer_fault_cfg.active_current_min_mA = 500.0f;

    bool saw_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[0].homing_state == Chassis::HomingState::kFault)
        {
            saw_fault = true;
            break;
        }
    }

    EXPECT_TRUE(saw_fault);
    EXPECT_TRUE(chassis.debug_mirror_.steer_fault_active[0]);
    EXPECT_TRUE(chassis.debug_mirror_.steer_fault_freeze_candidate[0]);
    EXPECT_TRUE(chassis.debug_mirror_.steer_feedback_current_freeze_ms[0] >= 5.0f);
}

void testFaultedWheelOnlyRehomesAfterCurrentTogglesAndMotionResumesAfterRecoveryCompletes()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 1) ? 5000.0f : 100.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
        drive_motors[i].setTargetCurrent(777.0f);
    }

    bool latched_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[1].homing_state == Chassis::HomingState::kFault)
        {
            latched_fault = true;
            break;
        }
    }
    EXPECT_TRUE(latched_fault);

    for (int cycle = 0; cycle < 3; ++cycle)
    {
        runHostControlCycle(chassis);
        EXPECT_TRUE(chassis.wheel_config_[1].homing_state == Chassis::HomingState::kFault);
        EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kReady);
        EXPECT_TRUE(chassis.wheel_config_[2].homing_state == Chassis::HomingState::kReady);
        EXPECT_TRUE(chassis.wheel_config_[3].homing_state == Chassis::HomingState::kReady);
        for (int i = 0; i < 4; ++i)
        {
            EXPECT_NEAR(drive_motors[i].getTargetCurrent(), 0.0f, 1.0e-6f);
        }
    }

    steer_motors[1].setFeedbackCurrent(-5000.0f);
    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[1].homing_state == Chassis::HomingState::kSearch);
    EXPECT_NEAR(steer_motors[1].getTargetRPM(), chassis.wheel_config_[1].homing_search_rpm, 1.0e-6f);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kReady);
    EXPECT_TRUE(chassis.wheel_config_[2].homing_state == Chassis::HomingState::kReady);
    EXPECT_TRUE(chassis.wheel_config_[3].homing_state == Chassis::HomingState::kReady);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(drive_motors[i].getTargetCurrent(), 0.0f, 1.0e-6f);
    }

    for (int cycle = 0; cycle < 2; ++cycle)
    {
        runHostControlCycle(chassis);
        EXPECT_TRUE(chassis.wheel_config_[1].homing_state == Chassis::HomingState::kSearch);
    }

    finishWheelHomingByEdgeAndAlign(chassis, 1, steer_motors);

    EXPECT_TRUE(chassis.wheel_config_[1].homing_state == Chassis::HomingState::kReady);
    EXPECT_TRUE(runHostControlCycle(chassis));
    EXPECT_TRUE(std::fabs(drive_motors[0].getTargetRPM()) > 1.0e-3f);
    EXPECT_TRUE(std::fabs(drive_motors[1].getTargetRPM()) > 1.0e-3f);
}

void testLatchedFaultedWheelKeepsPureZeroCurrentCommandWithoutOverwritingControlMode()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 2) ? 5000.0f : 100.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
    }

    bool latched_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[2].homing_state == Chassis::HomingState::kFault)
        {
            latched_fault = true;
            break;
        }
    }
    EXPECT_TRUE(latched_fault);

    steer_motors[2].setTargetRPM(321.0f);
    steer_motors[2].setTargetTotalAngle(654.0f);

    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[2].homing_state == Chassis::HomingState::kFault);
    EXPECT_NEAR(steer_motors[2].getTargetCurrent(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(steer_motors[2].getTargetRPM(), 321.0f, 1.0e-6f);
    EXPECT_NEAR(steer_motors[2].getTargetTotalAngle(), 654.0f, 1.0e-6f);
}

void testRecoveryImmediatelyReopensSteerSearchAfterFaultLatchSidePidReset()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 0) ? 5000.0f : 100.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
    }

    bool latched_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[0].homing_state == Chassis::HomingState::kFault)
        {
            latched_fault = true;
            break;
        }
    }
    EXPECT_TRUE(latched_fault);

    steer_motors[0].setFeedbackCurrent(-5000.0f);
    runHostControlCycle(chassis);

    EXPECT_TRUE(chassis.wheel_config_[0].steer_fault_state == Chassis::SteerFaultState::kRecovering);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);
    EXPECT_NEAR(steer_motors[0].getTargetRPM(), chassis.wheel_config_[0].homing_search_rpm, 1.0e-6f);
}

void testHomingEdgeCaptureDoesNotImmediatelyJumpToLargeAlignCommand()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.wheel_config_[0].homing_falling_edge_mech_rad = jia::degToRadF32(150.0f);
    chassis.wheel_config_[0].homing_rising_edge_mech_rad = jia::degToRadF32(-30.0f);
    chassis.wheel_config_[0].homing_zero_offset_rad = jia::degToRadF32(-30.0f);
    chassis.wheel_config_[0].homing_runtime_zero_offset_rad = chassis.wheel_config_[0].homing_zero_offset_rad;
    chassis.wheel_config_[0].homing_state = Chassis::HomingState::kSearch;
    chassis.wheel_config_[0].homing_zero_valid = false;
    chassis.wheel_config_[0].homing_last_sensor_active = true;

    steer_motors[0].setFeedbackTotalAngleDeg(40.0f);
    setPhotogateStateForWheel(0, false);

    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kEdgeDetected);

    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kOffsetApply);

    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kContinuousAngleReady);

    const float target_before_align_deg = steer_motors[0].getTargetTotalAngle();
    runHostControlCycle(chassis);

    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kAlignToZero);
    EXPECT_NEAR(steer_motors[0].getTargetTotalAngle(), target_before_align_deg, 1.0e-4f);
}

void testRecoveryRehomeTimeoutRelatchesFault()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 3) ? 5500.0f : 100.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
        drive_motors[i].setTargetCurrent(777.0f);
    }

    bool first_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[3].homing_state == Chassis::HomingState::kFault)
        {
            first_fault = true;
            break;
        }
    }
    EXPECT_TRUE(first_fault);

    steer_motors[3].setFeedbackCurrent(-5500.0f);
    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[3].homing_state == Chassis::HomingState::kSearch);

    setPhotogateStateForWheel(3, false);
    for (int cycle = 0; cycle < 5; ++cycle)
    {
        runHostControlCycle(chassis);
    }

    EXPECT_TRUE(chassis.wheel_config_[3].homing_state == Chassis::HomingState::kFault);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(drive_motors[i].getTargetCurrent(), 0.0f, 1.0e-6f);
    }
}

} // namespace

int main()
{
    testExternalCommandMapsToInternalBodyAxesWithoutChangingOmega();
    testPlannerAxisNormalizationDoesNotDependOnDebugStyleOmegaFlip();
    testSteerGeometryUsesSignedInstallationAngleOnly();
    testRuntimeZeroAndMotorPolarityOnlyAffectMotorLocalConversion();
    testSteerMotorSignAndRuntimeZeroOffsetStayAsIndependentMappingStages();
    testTelemetrySnapshotKeepsTargetAndActualYawSemanticsSeparate();
    testTelemetrySnapshotPreservesWheelTargetsWithoutModeDependentReinterpretation();
    testDriveMotorHardwarePolarityMapsCurrentWithoutLeakingIntoGeometry();
    testDefaultDriveMotorPolarityMatchesPositiveBodyXHardwareLayout();
    testPlannerInputNormalizationKeepsWorldBodyAndSteerOnlySemanticsExplicit();
    testNormalizedBodyCommandKeepsDebugAndApiRoutesSemanticallyAligned();
    testHomingRuntimeZeroOffsetOnlyDependsOnEdgeGeometryAndRawMotorAngle();
    testDebugRouteClassificationSeparatesInputInjectionFromModuleOverride();
    testDebugModuleOverrideRouteSeparatesRetiredMode20AlignHomingAndSingleWheelModes();
    testResolveDebugModeFallsBackToTorqueFreeWhenMode20IsRetired();
    testLowSpeedDriveSuppressionCanBeDisabled();
    testProjectedDriveUsesReachableSteerInsteadOfIdealVector();
    testHighSpeedDriveSuppressionTightensAndReleasesThroughPlannerOutput();
    testLowSpeedDriveSuppressionDoesNotReenterInsideNearZeroHysteresisBand();
    testHighSpeedDriveSuppressionWaitsUntilNearZeroExitBeforeEnabling();
    testMode30SingleWheelDirectJoystickIsolationOnlyLetsTargetWheelMove();
    testMode30SingleWheelDirectDriveCanUseSCurveShaping();
    testMode30CommonWheelIndexAliasCanDirectlySelectTargetWheel();
    testMode30DirectDriveIgnoresAllHomedGateForTargetWheel();
    testMode30SingleWheelRemovedModes31And32DoNotEnterModuleOverride();
    testMode30SingleWheelUsesConfiguredAxesAndInversion();
    testMode30SingleWheelSharedDeadzoneSuppressesContinuousAndStepInputs();
    testMode30SingleWheelAxisEnablesAndEstopGateOutputs();
    testMode30SingleWheelSteerPlannerSupportsSCurveAndTrapezoid();
    testMode30SingleWheelDrivePlannerSupportsTrapezoidAndIgnoresGlobalManualProfileParams();
    testRefreshDebugMirrorPublishesHomingDiagnosticsForObserveMode();
    testMode30SingleWheelNonTargetCommandTypesBypassPlanner();
    testMode30SingleWheelPlannerStateResetsWhenWheelAndPlannerModeChange();
    testJustFloatSingleWheelProfileUsesObserveWheelIndex();
    testJustFloatSingleWheelPayloadUsesObserveWheelIndex();
    testJustFloatSingleWheelObserveIndexFallsBackToZeroWhenOutOfRange();
    testJustFloatSingleWheelDriveOnlyPayloadUsesObserveWheelIndex();
    testJustFloatSingleWheelDriveOnlyObserveIndexFallsBackToZeroWhenOutOfRange();
    testMode30SingleWheelDriveStepGeneratorOverridesManualInput();
    testDriveVirtualLoadInjectsBiasOnlyForSelectedWheelAndPidMode();
    testMode30DriveVirtualLoadIgnoresAllHomedGateForTargetWheel();
    testMode30DriveVirtualLoadIgnoresOtherWheelSteerFaultForTargetWheel();
    testJustFloatDrivePidLoadProfileEmitsFixed16ChannelPayload();
    testDebugOmegaZInjectionModeOffKeepsManualOmegaInput();
    testDebugOmegaZInjectionModeStepOverridesManualOmegaInput();
    testDebugOmegaZInjectionModeSineOverridesManualOmegaInput();
    testDebugOmegaZInjectionDoesNotAffectLockToTarget();
    testSuppressedDriveDoesNotAccumulateHiddenAccelState();
    testDriveReleaseResumesFromDeliveredSpeedNotVirtualSpeed();
    testDriveReleaseHasNoVelocityJumpAfterZeroHold();
    testPlannerTargetMayChangeWhileDeliveredStateRemainsContinuous();
    testTorqueFreeAndNotHomedPathsResetDriveDeliveryStateConsistently();
    testDriveZeroStopUsesBrakeWhenResidualSpeedIsStillHigh();
    testDriveZeroStopSettlesToZeroCurrentWithoutHoldingBrake();
    testDriveZeroStopReleaseClearsPidStateBeforeReturningToRpmLoop();
    testDriveZeroStopBrakeHysteresisDoesNotToggleAroundThreshold();
    testLowSpeedDriveSuppressionBypassesWhenResidualSpeedAboveThreshold();
    testLowSpeedDriveSuppressionReenabledWhenResidualSpeedDropsBelowThreshold();
    testLowSpeedDriveSuppressionUsesGlobalWorstWheelError();
    testGlobalMaxResidualSpeedControlsLowSpeedSuppressionForAllWheels();
    testRefreshDebugMirrorSeparatesPlannedAndDeliveredDriveDiagnostics();
    testJustFloatYawPidProfileDispatchEmitsFixed15ChannelPayload();
    testLockToYawPidTracePublishesTargetErrorAndPidFireState();
    testLockToYawThenLockNowKeepsTheEffectiveLockedYaw();
    testLockNowYawPidTraceDistinguishesManualShiftAndHoldStates();
    testHardGateFromXParkHoldsAllDriveUntilAllWheelsPassCloseAngle();
    testLaunchFromXParkHoldsBodyAndDriveAtZeroUntilAllWheelsAligned();
    testNormalLaunchSCurveWaitsForSteerAlignmentBeforeAccumulatingBodyPlan();
    testManualSCurveProfileLegacyModeKeepsCurrentAccelerationStepSemantics();
    testManualSCurveProfileProducesSofterFirstStepAndContinuousAcceleration();
    testManualSCurveProfileToggleResetsShapingHistory();
    testManualSCurveProfileStartsBrakingBeforeTargetWhenRemainingErrorIsTooSmallForCurrentAcceleration();
    testManualSCurveProfileSmallResidualClampClearsInternalAcceleration();
    testManualSCurveProfileManualOnlyModeFallsBackForApiSource();
    testManualSCurveProfileManualOnlyModeUsesSCurveForDebugSource();
    testManualSCurveProfileRapidReverseBleedsPositiveTrendBeforeBuildingNegativeTrend();
    testDebugBodySpeedOmegaTargetFlipsSignImmediatelyWithRightStickDirection();
    testDebugBodySpeedOmegaRapidReverseUnderSCurveKeepsOldSignForFirstPlannerStep();
    testDebugBodySpeedOmegaRapidReverseEventuallyCrossesNegativeAfterEnoughCycles();
    testJerkProfileRapidReverseEventuallyCrossesZeroAndBuildsOppositeSign();
    testDebugBodySpeedOmegaRapidReverseEventuallyChangesPredictedActuatorOmegaSign();
    testDebugBodySpeedTranslationRapidReverseEventuallyChangesPredictedActuatorDirection();
    testDriveOmegaPlannerLimitUsesUniformScaleAcrossAllWheels();
    testFlipSolutionPrefersSmallSteerDeltaAndInvertsDriveOnQuadrantCrossing();
    testReverseIntentBypassesTranslationalDirectionSlewOnNearOppositeCommand();
    testDirectionSlewCrossesPiBoundaryByShortestPath();
    testReverseIntentDoesNotFallIntoZeroHoldOrSuppressionWhenSteerIsReachable();
    testAlwaysForwardModeIgnoresReverseIntentOverride();
    testDriveAlphaDeliveryLimitUsesUniformScaleAcrossAllWheels();
    testDriveAlphaDeliveryLimitUsesWorstWheelScaleWhenLastValuesDiffer();
    testDriveAlphaDeliveryLimitUsesUniformScaleWhileNotHomed();
    testDriveDeliveryLimitAlphaThenOmegaStillKeepsSharedProgress();
    testXParkActivatesOnlyAfterActualResidualWheelSpeedHoldsForEntryDelay();
    testXParkHoldCounterResetsImmediatelyWhenCommandWheelSpeedExitsThreshold();
    testXParkResidualWheelSpeedAboveThresholdKeepsGateClosed();
    testXParkCommandThresholdTreatsPointZeroFiveAsMovingDespiteLargeResidualFilter();
    testXParkResidualThresholdStillBlocksEntryWhenCommandIsStationary();
    testXParkUsesIndependentCommandThresholdWhileResidualMayUseLargeNearZeroFilter();
    testXParkAllowsEntryWhenBothCommandAndResidualAreBelowTheirOwnThresholds();
    testXParkDoesNotEnterWhenCommandExceedsDedicatedCommandThreshold();
    testXParkSteerDeadbandFreezesWheelWhenErrorEntersOneDegreeBand();
    testXParkSteerDeadbandStaysLatchedInsideThreeDegreeExitBand();
    testXParkSteerDeadbandReleasesOnceErrorExceedsThreeDegreeExitBand();
    testNonXParkSteerCommandDoesNotTriggerSteerDeadbandFreeze();
    testSteerSettleResetTriggersSingleSpeedPidResetOnEnter();
    testSteerSettleResetDoesNotRetriggerWhileInsideExitBand();
    testSteerSettleResetRetriggersAfterLeavingAndReEnteringBand();
    testSteerSettleResetDoesNotTriggerDuringZeroCurrentBranch();
    testStationaryPhotogateTogglesDoNotSelfLockNormalReadyState();
    testReadyStationaryWheelsCanStillEnterXParkWithoutTriggeringSteerFault();
    testSingleWheelSteerFreezeFaultStopsVehicleAndFreezesFaultedWheelPath();
    testSingleWheelSteerFreezeFaultDoesNotRequireHugeCurrentMagnitude();
    testSteerFaultThresholdsAreDebugTunableAndMirrorPublishesDecisionInputs();
    testFaultedWheelOnlyRehomesAfterCurrentTogglesAndMotionResumesAfterRecoveryCompletes();
    testLatchedFaultedWheelKeepsPureZeroCurrentCommandWithoutOverwritingControlMode();
    testRecoveryImmediatelyReopensSteerSearchAfterFaultLatchSidePidReset();
    testHomingEdgeCaptureDoesNotImmediatelyJumpToLargeAlignCommand();
    testRecoveryRehomeTimeoutRelatchesFault();
    testDrivePidEnableEdgeReadsBackRuntimeIntoCleanSharedCache();
    testDrivePidDirtyCacheBlocksRuntimeReadbackOverwrite();
    testDrivePidSharedApplyPushesSameParamsToAllVescsAndAlignsAppliedStamp();
    testDrivePidApplySkipsNullHandlesAndStillUpdatesAppliedStamp();

    if (g_failures != 0)
    {
        std::printf("chassis_semantics test: FAIL failures=%d\n", g_failures);
        return EXIT_FAILURE;
    }

    std::puts("chassis_semantics test: PASS");
    return EXIT_SUCCESS;
}
