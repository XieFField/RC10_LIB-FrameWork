#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "chassis.h"

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
} // namespace

int main()
{
    testExternalCommandMapsToInternalBodyAxesWithoutChangingOmega();
    testPlannerAxisNormalizationDoesNotDependOnDebugStyleOmegaFlip();
    testSteerGeometryUsesSignedInstallationAngleOnly();
    testRuntimeZeroAndMotorPolarityOnlyAffectMotorLocalConversion();
    testTelemetrySnapshotKeepsTargetAndActualYawSemanticsSeparate();
    testTelemetrySnapshotPreservesWheelTargetsWithoutModeDependentReinterpretation();
    testDriveMotorHardwarePolarityMapsCurrentWithoutLeakingIntoGeometry();
    testPlannerInputNormalizationKeepsWorldBodyAndSteerOnlySemanticsExplicit();
    testHomingRuntimeZeroOffsetOnlyDependsOnEdgeGeometryAndRawMotorAngle();
    testDebugRouteClassificationSeparatesInputInjectionFromModuleOverride();

    if (g_failures != 0)
    {
        std::printf("chassis_semantics test: FAIL failures=%d\n", g_failures);
        return EXIT_FAILURE;
    }

    std::puts("chassis_semantics test: PASS");
    return EXIT_SUCCESS;
}
