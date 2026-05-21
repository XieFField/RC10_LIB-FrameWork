#include <cmath>
#include <cstdio>
#include <cstdlib>

#define private public
#include "chassis.h"
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
};

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
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(true, 20) == Chassis::DebugControlRoute::kModuleOverride);
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(true, 30) == Chassis::DebugControlRoute::kModuleOverride);
}

void testDebugModuleOverrideRouteSeparatesSingleWheelAlignHomingAndDirect()
{
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(1) == Chassis::DebugModuleOverrideRoute::kNone);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(20) == Chassis::DebugModuleOverrideRoute::kSingleWheel);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(21) == Chassis::DebugModuleOverrideRoute::kAlignForward);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(22) == Chassis::DebugModuleOverrideRoute::kHomingObserve);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(30) == Chassis::DebugModuleOverrideRoute::kDirectActuator);
}

void testProjectedDriveUsesReachableSteerInsteadOfIdealVector()
{
    Chassis chassis;
    chassis.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_drive_gate = false;
    chassis.enable_cosine_compensation_ = false;
    chassis.runtime_strategy_cfg_.vector_consistency.enable = false;
    chassis.actuator_limit_enable_.enable_steer_rate_limit = true;
    chassis.actuator_limit_enable_.enable_steer_alpha_limit = false;
    chassis.max_steer_rate_rad_s_ = 1.0f;

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

void testVectorConsistencyGateTightensAndReleasesThroughPlannerOutput()
{
    Chassis chassis;
    chassis.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_drive_gate = false;
    chassis.enable_cosine_compensation_ = false;
    chassis.runtime_strategy_cfg_.vector_consistency.enable = true;
    chassis.runtime_strategy_cfg_.vector_consistency.min_trans_speed_enable_m_s = 0.05f;
    chassis.runtime_strategy_cfg_.vector_consistency.dir_err_enter_deg = 5.0f;
    chassis.runtime_strategy_cfg_.vector_consistency.dir_err_exit_deg = 2.0f;
    chassis.runtime_strategy_cfg_.vector_consistency.eta_lock_s = 0.05f;
    chassis.runtime_strategy_cfg_.vector_consistency.eta_release_s = 0.01f;
    chassis.actuator_limit_enable_.enable_steer_rate_limit = true;
    chassis.actuator_limit_enable_.enable_steer_alpha_limit = false;
    chassis.max_steer_rate_rad_s_ = 1.0f;

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
    EXPECT_TRUE(gated_output.vector_gate_active);
    EXPECT_TRUE(gated_output.vector_gate_scale < 1.0f);

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
        chassis.last_steer_rate_cmd_rad_s_[i] = 0.0f;
    }

    const Chassis::SwervePlannerOutput released_output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));
    EXPECT_TRUE(!released_output.vector_gate_active);
    EXPECT_TRUE(released_output.vector_gate_scale > gated_output.vector_gate_scale);
}

void testDirectActuatorContinuousInputResolvesAxisAndControlTypesConsistently()
{
    Chassis chassis;
    chassis.debug_control_.wheel_index = 2U;
    chassis.debug_control_.direct_input_source = 1U;
    chassis.debug_control_.direct_steer_control_type = 1U;
    chassis.debug_control_.direct_drive_control_type = 1U;
    chassis.debug_control_.direct_steer_rpm_limit = 200.0f;
    chassis.debug_control_.direct_drive_current_limit_mA = 8000.0f;
    chassis.airjoy_data_.left_x = 0.5f;
    chassis.airjoy_data_.right_x = -0.25f;

    const Chassis::DirectActuatorCommandSnapshot resolved = chassis.resolveDirectActuatorCommand(2U);

    EXPECT_TRUE(resolved.drive_control_type == 1U);
    EXPECT_NEAR(resolved.steer_axis_value, 0.5f, 1.0e-6f);
    EXPECT_NEAR(resolved.drive_axis_value, -0.25f, 1.0e-6f);
    EXPECT_NEAR(resolved.steer_rpm_cmd, 100.0f, 1.0e-6f);
    EXPECT_NEAR(resolved.drive_current_cmd_mA, -2000.0f, 1.0e-6f);
    EXPECT_NEAR(resolved.applied_steer_cmd, 100.0f, 1.0e-6f);
    EXPECT_NEAR(resolved.applied_drive_cmd, -2000.0f, 1.0e-6f);
}

void testDirectActuatorOverrideOnlyAppliesToSelectedWheel()
{
    Chassis chassis;
    TestMotor steer_motors[4];
    TestMotor drive_motors[4];

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].steer_motor_h = &steer_motors[i];
        chassis.wheel_config_[i].drive_motor_h = &drive_motors[i];
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = (i == 1) ? -1.0f : 1.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
    }

    chassis.debug_control_.wheel_index = 1U;
    chassis.debug_control_.direct_input_source = 0U;
    chassis.debug_control_.direct_steer_control_type = 2U;
    chassis.debug_control_.direct_drive_control_type = 2U;
    chassis.debug_control_.direct_enable_steer[1] = true;
    chassis.debug_control_.direct_enable_drive[1] = true;
    chassis.debug_control_.direct_steer_single_turn_deg[1] = 45.0f;
    chassis.debug_control_.direct_drive_brake_mA[1] = 1800.0f;

    chassis.applyDirectActuatorDebugOverride(1U);

    EXPECT_NEAR(chassis.wheel_config_[1].target_steer_motor_total_angle_rad, jia::degToRadF32(45.0f), 1.0e-6f);
    EXPECT_NEAR(chassis.planned_data_.steer_angle_oa_rad[1], jia::degToRadF32(45.0f), 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[1].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[1].getTargetBrake(), -1800.0f, 1.0e-6f);
    EXPECT_NEAR(steer_motors[1].getTargetTotalAngle(), 45.0f, 1.0e-4f);
    EXPECT_NEAR(drive_motors[0].getTargetBrake(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[2].getTargetBrake(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[3].getTargetBrake(), 0.0f, 1.0e-6f);
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

    chassis.debug_control_.wheel_index = 0U;
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

void configureDriveContinuityHarness(Chassis &chassis, TestMotor drive_motors[4])
{
    chassis.max_drive_alpha_rad_s2_ = 10.0f;
    chassis.max_drive_omega_rad_s_ = 1000.0f;
    chassis.actuator_limit_enable_.enable_drive_alpha_limit = true;
    chassis.actuator_limit_enable_.enable_drive_omega_limit = false;
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
        drive_motors[i].setTargetRPM(0.0f);
    }
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

Chassis::SwervePlannerOutput makeNeutralPlannerOutput()
{
    Chassis::SwervePlannerOutput output{};
    output.vector_gate_scale = 1.0f;
    for (int i = 0; i < 4; ++i)
    {
        output.gate_or_cos_scale[i] = 1.0f;
    }
    return output;
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
    TestMotor drive_motors[4];
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
    TestMotor drive_motors[4];
    configureDriveContinuityHarness(chassis, drive_motors);
    const float expected_release_step = chassis.max_drive_alpha_rad_s2_ * Chassis::period_;

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
    TestMotor drive_motors[4];
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
    TestMotor drive_motors[4];
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
    TestMotor drive_motors[4];
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

void testHardGateDisabledWhenMaxResidualSpeedAboveThreshold()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_drive_gate = true;
    chassis.runtime_strategy_cfg_.drive_gate_strategy = Chassis::DriveGateStrategy::kHardGate;
    chassis.runtime_strategy_cfg_.drive_gate_scope = Chassis::DriveGateScope::kGlobal;
    chassis.runtime_strategy_cfg_.drive_gate_min_scale = 0.0f;
    chassis.runtime_strategy_cfg_.drive_gate_close_angle_deg = 1.0f;
    chassis.runtime_strategy_cfg_.drive_gate_hard_disable_residual_speed_m_s = 0.3f;

    const Chassis::SwervePlannerInput planner_input = makeGatePlannerInput(10.0f, 0.0f, 0.0f, 0.5f);
    const float steering_errors_rad[4] = {
        jia::degToRadF32(10.0f), jia::degToRadF32(10.0f), jia::degToRadF32(10.0f), jia::degToRadF32(10.0f)};
    float gate_scales[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    chassis.computeDriveGateScales(planner_input, steering_errors_rad, gate_scales);

    EXPECT_NEAR(gate_scales[0], 1.0f, 1.0e-6f);
    EXPECT_NEAR(gate_scales[3], 1.0f, 1.0e-6f);
}

void testHardGateReenabledWhenMaxResidualSpeedDropsBelowThreshold()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_drive_gate = true;
    chassis.runtime_strategy_cfg_.drive_gate_strategy = Chassis::DriveGateStrategy::kHardGate;
    chassis.runtime_strategy_cfg_.drive_gate_scope = Chassis::DriveGateScope::kGlobal;
    chassis.runtime_strategy_cfg_.drive_gate_min_scale = 0.0f;
    chassis.runtime_strategy_cfg_.drive_gate_close_angle_deg = 1.0f;
    chassis.runtime_strategy_cfg_.drive_gate_hard_disable_residual_speed_m_s = 0.3f;

    const float steering_errors_rad[4] = {
        jia::degToRadF32(10.0f), jia::degToRadF32(10.0f), jia::degToRadF32(10.0f), jia::degToRadF32(10.0f)};
    float gate_scales[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    chassis.computeDriveGateScales(makeGatePlannerInput(10.0f, 0.0f, 0.0f, 0.5f), steering_errors_rad, gate_scales);
    chassis.computeDriveGateScales(makeGatePlannerInput(10.0f, 0.0f, 0.0f, 0.1f), steering_errors_rad, gate_scales);

    EXPECT_NEAR(gate_scales[0], 0.0f, 1.0e-6f);
    EXPECT_NEAR(gate_scales[2], 0.0f, 1.0e-6f);
}

void testSoftAndCurveGateIgnoreHardResidualDisableThreshold()
{
    Chassis hard_disabled_chassis;
    hard_disabled_chassis.runtime_strategy_cfg_.enable_drive_gate = true;
    hard_disabled_chassis.runtime_strategy_cfg_.drive_gate_min_scale = 0.2f;
    hard_disabled_chassis.runtime_strategy_cfg_.drive_gate_close_angle_deg = 1.0f;
    hard_disabled_chassis.runtime_strategy_cfg_.drive_gate_hard_disable_residual_speed_m_s = 0.3f;

    const float steering_errors_rad[4] = {
        jia::degToRadF32(10.0f), jia::degToRadF32(10.0f), jia::degToRadF32(10.0f), jia::degToRadF32(10.0f)};
    float soft_gate_scales[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float curve_gate_scales[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    hard_disabled_chassis.runtime_strategy_cfg_.drive_gate_strategy = Chassis::DriveGateStrategy::kSoftGate;
    hard_disabled_chassis.computeDriveGateScales(makeGatePlannerInput(10.0f, 0.0f, 0.0f, 0.5f), steering_errors_rad, soft_gate_scales);

    hard_disabled_chassis.runtime_strategy_cfg_.drive_gate_strategy = Chassis::DriveGateStrategy::kContinuousCurve;
    hard_disabled_chassis.computeDriveGateScales(makeGatePlannerInput(10.0f, 0.0f, 0.0f, 0.5f), steering_errors_rad, curve_gate_scales);

    EXPECT_TRUE(soft_gate_scales[0] < 1.0f);
    EXPECT_TRUE(curve_gate_scales[0] < 1.0f);
}

void testGlobalMaxResidualSpeedControlsHardGateForAllWheels()
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.enable_drive_gate = true;
    chassis.runtime_strategy_cfg_.drive_gate_strategy = Chassis::DriveGateStrategy::kHardGate;
    chassis.runtime_strategy_cfg_.drive_gate_scope = Chassis::DriveGateScope::kGlobal;
    chassis.runtime_strategy_cfg_.drive_gate_min_scale = 0.0f;
    chassis.runtime_strategy_cfg_.drive_gate_close_angle_deg = 1.0f;
    chassis.runtime_strategy_cfg_.drive_gate_hard_disable_residual_speed_m_s = 0.3f;

    Chassis::SwervePlannerInput planner_input = makeGatePlannerInput(10.0f, 0.0f, 0.0f, 0.1f);
    planner_input.residual_speed_m_s[0] = 0.5f;
    planner_input.max_residual_speed_m_s = 0.5f;
    const float steering_errors_rad[4] = {
        jia::degToRadF32(10.0f), jia::degToRadF32(2.0f), jia::degToRadF32(10.0f), jia::degToRadF32(2.0f)};
    float gate_scales[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    chassis.computeDriveGateScales(planner_input, steering_errors_rad, gate_scales);

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
    chassis.hard_gate_bypassed_by_residual_speed_ = true;

    chassis.refreshDebugMirror(true);

    EXPECT_NEAR(chassis.debug_mirror_.planned_drive_target_rpm[0], jia::radsToRpmF32(12.0f), 1.0e-4f);
    EXPECT_NEAR(chassis.debug_mirror_.delivered_drive_target_rpm[0], jia::radsToRpmF32(3.0f), 1.0e-4f);
    EXPECT_TRUE(chassis.debug_mirror_.hard_gate_bypassed_by_residual_speed);
    EXPECT_NEAR(chassis.debug_mirror_.max_residual_speed_m_s, 0.45f, 1.0e-6f);
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
    testPlannerInputNormalizationKeepsWorldBodyAndSteerOnlySemanticsExplicit();
    testNormalizedBodyCommandKeepsDebugAndApiRoutesSemanticallyAligned();
    testHomingRuntimeZeroOffsetOnlyDependsOnEdgeGeometryAndRawMotorAngle();
    testDebugRouteClassificationSeparatesInputInjectionFromModuleOverride();
    testDebugModuleOverrideRouteSeparatesSingleWheelAlignHomingAndDirect();
    testProjectedDriveUsesReachableSteerInsteadOfIdealVector();
    testVectorConsistencyGateTightensAndReleasesThroughPlannerOutput();
    testDirectActuatorContinuousInputResolvesAxisAndControlTypesConsistently();
    testDirectActuatorOverrideOnlyAppliesToSelectedWheel();
    testRefreshDebugMirrorPublishesHomingDiagnosticsForObserveMode();
    testSuppressedDriveDoesNotAccumulateHiddenAccelState();
    testDriveReleaseResumesFromDeliveredSpeedNotVirtualSpeed();
    testDriveReleaseHasNoVelocityJumpAfterZeroHold();
    testPlannerTargetMayChangeWhileDeliveredStateRemainsContinuous();
    testTorqueFreeAndNotHomedPathsResetDriveDeliveryStateConsistently();
    testHardGateDisabledWhenMaxResidualSpeedAboveThreshold();
    testHardGateReenabledWhenMaxResidualSpeedDropsBelowThreshold();
    testSoftAndCurveGateIgnoreHardResidualDisableThreshold();
    testGlobalMaxResidualSpeedControlsHardGateForAllWheels();
    testRefreshDebugMirrorSeparatesPlannedAndDeliveredDriveDiagnostics();

    if (g_failures != 0)
    {
        std::printf("chassis_semantics test: FAIL failures=%d\n", g_failures);
        return EXIT_FAILURE;
    }

    std::puts("chassis_semantics test: PASS");
    return EXIT_SUCCESS;
}
