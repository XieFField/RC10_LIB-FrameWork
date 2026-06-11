#include "test_chassis_semantics_harness.h"

namespace chassis_semantics_test
{

// 覆盖驱动 PID 调参、坐标轴映射、硬件极性和低速驱动抑制等“基础语义”。
// 这些 case 通常不需要完整控制循环，重点是验证配置字段和几何变换不会互相串味。
TEST_CASE("testDrivePidEnableEdgeReadsBackRuntimeIntoCleanSharedCache")
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

TEST_CASE("testDrivePidDirtyCacheBlocksRuntimeReadbackOverwrite")
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

TEST_CASE("testDrivePidSharedApplyPushesSameParamsToAllVescsAndAlignsAppliedStamp")
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

TEST_CASE("testDrivePidApplySkipsNullHandlesAndStillUpdatesAppliedStamp")
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

TEST_CASE("testSteerPidEnableEdgeReadsBackRuntimeIntoCleanSharedCache")
{
    DrivePidTuneHarness harness;
    configureDrivePidTuneHarness(harness);

    PID_Param_Config runtime_speed_cfg{};
    runtime_speed_cfg.kp = 23.0f;
    runtime_speed_cfg.ki = 0.33f;
    runtime_speed_cfg.kd = 0.07f;
    runtime_speed_cfg.output_limit = 6789.0f;
    PID_Param_Config runtime_angle_cfg{};
    runtime_angle_cfg.kp = 4.2f;
    runtime_angle_cfg.kd = 0.12f;
    runtime_angle_cfg.deadband = 0.04f;
    runtime_angle_cfg.output_limit = 321.0f;
    harness.steer_motors[0].pid_init(runtime_speed_cfg, 0.44f, runtime_angle_cfg, 0.66f);

    harness.chassis.debug_pid_tune_.steer_speed_pid_cfg = PID_Param_Config{};
    harness.chassis.debug_pid_tune_.steer_angle_pid_cfg = PID_Param_Config{};
    harness.chassis.debug_pid_tune_.steer_speed_pid_td_ratio = 0.0f;
    harness.chassis.debug_pid_tune_.steer_angle_pid_i_separa = 0.0f;
    harness.chassis.debug_pid_tune_.steer_speed_pid_apply_stamp = 0U;
    harness.chassis.debug_pid_tune_.steer_speed_pid_applied_stamp = 0U;
    harness.chassis.debug_pid_tune_.steer_angle_pid_apply_stamp = 0U;
    harness.chassis.debug_pid_tune_.steer_angle_pid_applied_stamp = 0U;
    harness.chassis.debug_control_.common.enable = false;
    harness.chassis.debug_enable_last_cycle_ = false;

    harness.chassis.syncDebugSteerPidTuneFromRuntimeOnEnableEdge();
    harness.chassis.debug_control_.common.enable = true;
    harness.chassis.syncDebugSteerPidTuneFromRuntimeOnEnableEdge();

    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_speed_pid_cfg.kp, runtime_speed_cfg.kp, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_speed_pid_cfg.ki, runtime_speed_cfg.ki, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_speed_pid_cfg.kd, runtime_speed_cfg.kd, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_speed_pid_cfg.output_limit, runtime_speed_cfg.output_limit, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_angle_pid_cfg.kp, runtime_angle_cfg.kp, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_angle_pid_cfg.kd, runtime_angle_cfg.kd, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_angle_pid_cfg.deadband, runtime_angle_cfg.deadband, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_angle_pid_cfg.output_limit, runtime_angle_cfg.output_limit, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_speed_pid_td_ratio, 0.44f, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_angle_pid_i_separa, 0.66f, 1.0e-6f);
}

TEST_CASE("testSteerPidDirtyCacheBlocksRuntimeReadbackOverwrite")
{
    DrivePidTuneHarness harness;
    configureDrivePidTuneHarness(harness);

    PID_Param_Config manual_speed_cfg{};
    manual_speed_cfg.kp = 31.0f;
    manual_speed_cfg.ki = 0.88f;
    manual_speed_cfg.output_limit = 7777.0f;
    PID_Param_Config manual_angle_cfg{};
    manual_angle_cfg.kp = 5.5f;
    manual_angle_cfg.kd = 0.22f;
    manual_angle_cfg.output_limit = 444.0f;
    PID_Param_Config runtime_speed_cfg{};
    runtime_speed_cfg.kp = 9.0f;
    PID_Param_Config runtime_angle_cfg{};
    runtime_angle_cfg.kp = 1.5f;
    harness.steer_motors[0].pid_init(runtime_speed_cfg, 0.11f, runtime_angle_cfg, 0.22f);

    harness.chassis.debug_pid_tune_.steer_speed_pid_cfg = manual_speed_cfg;
    harness.chassis.debug_pid_tune_.steer_angle_pid_cfg = manual_angle_cfg;
    harness.chassis.debug_pid_tune_.steer_speed_pid_td_ratio = 0.91f;
    harness.chassis.debug_pid_tune_.steer_angle_pid_i_separa = 0.37f;
    harness.chassis.debug_pid_tune_.steer_speed_pid_apply_stamp = 12U;
    harness.chassis.debug_pid_tune_.steer_speed_pid_applied_stamp = 10U;
    harness.chassis.debug_pid_tune_.steer_angle_pid_apply_stamp = 14U;
    harness.chassis.debug_pid_tune_.steer_angle_pid_applied_stamp = 13U;

    harness.chassis.syncDebugSteerPidTuneFromRuntime();

    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_speed_pid_cfg.kp, manual_speed_cfg.kp, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_speed_pid_cfg.ki, manual_speed_cfg.ki, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_speed_pid_cfg.output_limit, manual_speed_cfg.output_limit, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_angle_pid_cfg.kp, manual_angle_cfg.kp, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_angle_pid_cfg.kd, manual_angle_cfg.kd, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_angle_pid_cfg.output_limit, manual_angle_cfg.output_limit, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_speed_pid_td_ratio, 0.91f, 1.0e-6f);
    EXPECT_NEAR(harness.chassis.debug_pid_tune_.steer_angle_pid_i_separa, 0.37f, 1.0e-6f);
}

TEST_CASE("testSteerPidSharedApplyPushesSameParamsToAllM3508sAndAlignsAppliedStamps")
{
    DrivePidTuneHarness harness;
    configureDrivePidTuneHarness(harness);

    PID_Param_Config shared_speed_cfg{};
    shared_speed_cfg.kp = 28.0f;
    shared_speed_cfg.ki = 0.42f;
    shared_speed_cfg.kd = 0.03f;
    shared_speed_cfg.deadband = 0.6f;
    shared_speed_cfg.output_limit = 10000.0f;
    PID_Param_Config shared_angle_cfg{};
    shared_angle_cfg.kp = 4.8f;
    shared_angle_cfg.kd = 0.16f;
    shared_angle_cfg.deadband = 0.02f;
    shared_angle_cfg.output_limit = 520.0f;

    harness.chassis.debug_pid_tune_.steer_speed_pid_cfg = shared_speed_cfg;
    harness.chassis.debug_pid_tune_.steer_angle_pid_cfg = shared_angle_cfg;
    harness.chassis.debug_pid_tune_.steer_speed_pid_td_ratio = 0.25f;
    harness.chassis.debug_pid_tune_.steer_angle_pid_i_separa = 0.75f;
    harness.chassis.debug_pid_tune_.steer_speed_pid_apply_stamp = 61U;
    harness.chassis.debug_pid_tune_.steer_speed_pid_applied_stamp = 60U;
    harness.chassis.debug_pid_tune_.steer_angle_pid_apply_stamp = 71U;
    harness.chassis.debug_pid_tune_.steer_angle_pid_applied_stamp = 70U;

    harness.chassis.applyDebugSteerPidRuntimeTuning();

    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(harness.steer_motors[i].get_speed_pid_params().kp, shared_speed_cfg.kp, 1.0e-6f);
        EXPECT_NEAR(harness.steer_motors[i].get_speed_pid_params().ki, shared_speed_cfg.ki, 1.0e-6f);
        EXPECT_NEAR(harness.steer_motors[i].get_speed_pid_params().kd, shared_speed_cfg.kd, 1.0e-6f);
        EXPECT_NEAR(harness.steer_motors[i].get_speed_pid_params().deadband, shared_speed_cfg.deadband, 1.0e-6f);
        EXPECT_NEAR(harness.steer_motors[i].get_speed_pid_params().output_limit, shared_speed_cfg.output_limit, 1.0e-6f);
        EXPECT_NEAR(harness.steer_motors[i].get_angle_pid_params().kp, shared_angle_cfg.kp, 1.0e-6f);
        EXPECT_NEAR(harness.steer_motors[i].get_angle_pid_params().kd, shared_angle_cfg.kd, 1.0e-6f);
        EXPECT_NEAR(harness.steer_motors[i].get_angle_pid_params().deadband, shared_angle_cfg.deadband, 1.0e-6f);
        EXPECT_NEAR(harness.steer_motors[i].get_angle_pid_params().output_limit, shared_angle_cfg.output_limit, 1.0e-6f);
        EXPECT_NEAR(harness.steer_motors[i].get_speed_pid_td_ratio(), 0.25f, 1.0e-6f);
        EXPECT_NEAR(harness.steer_motors[i].get_angle_pid_i_separa_threshold(), 0.75f, 1.0e-6f);
    }
    EXPECT_TRUE(harness.chassis.debug_pid_tune_.steer_speed_pid_applied_stamp == 61U);
    EXPECT_TRUE(harness.chassis.debug_pid_tune_.steer_angle_pid_applied_stamp == 71U);
}

TEST_CASE("testSteerPidApplySkipsNullHandlesAndStillUpdatesAppliedStamps")
{
    DrivePidTuneHarness harness;
    configureDrivePidTuneHarness(harness);

    harness.chassis.wheel_config_[1].steer_motor_h = nullptr;
    harness.chassis.wheel_config_[2].steer_motor_h = nullptr;

    PID_Param_Config shared_speed_cfg{};
    shared_speed_cfg.kp = 35.0f;
    PID_Param_Config shared_angle_cfg{};
    shared_angle_cfg.kp = 6.2f;

    harness.chassis.debug_pid_tune_.steer_speed_pid_cfg = shared_speed_cfg;
    harness.chassis.debug_pid_tune_.steer_angle_pid_cfg = shared_angle_cfg;
    harness.chassis.debug_pid_tune_.steer_speed_pid_td_ratio = 0.18f;
    harness.chassis.debug_pid_tune_.steer_angle_pid_i_separa = 0.28f;
    harness.chassis.debug_pid_tune_.steer_speed_pid_apply_stamp = 81U;
    harness.chassis.debug_pid_tune_.steer_speed_pid_applied_stamp = 80U;
    harness.chassis.debug_pid_tune_.steer_angle_pid_apply_stamp = 91U;
    harness.chassis.debug_pid_tune_.steer_angle_pid_applied_stamp = 90U;

    harness.chassis.applyDebugSteerPidRuntimeTuning();

    EXPECT_TRUE(harness.chassis.debug_pid_tune_.steer_speed_pid_applied_stamp == 81U);
    EXPECT_TRUE(harness.chassis.debug_pid_tune_.steer_angle_pid_applied_stamp == 91U);
    EXPECT_NEAR(harness.steer_motors[0].get_speed_pid_params().kp, shared_speed_cfg.kp, 1.0e-6f);
    EXPECT_NEAR(harness.steer_motors[3].get_speed_pid_params().kp, shared_speed_cfg.kp, 1.0e-6f);
    EXPECT_NEAR(harness.steer_motors[0].get_angle_pid_params().kp, shared_angle_cfg.kp, 1.0e-6f);
    EXPECT_NEAR(harness.steer_motors[3].get_angle_pid_params().kp, shared_angle_cfg.kp, 1.0e-6f);
    EXPECT_NEAR(harness.steer_motors[0].get_speed_pid_td_ratio(), 0.18f, 1.0e-6f);
    EXPECT_NEAR(harness.steer_motors[3].get_angle_pid_i_separa_threshold(), 0.28f, 1.0e-6f);
}

TEST_CASE("testExternalCommandMapsToInternalBodyAxesWithoutChangingOmega")
{
    Chassis::ExternalCommand command{};
    command.coord = Chassis::Coordinate::kBody;
    command.vel_x = 1.25f;
    command.vel_y = -2.50f;
    command.omega_z = 0.75f;

    const Chassis::BodyCommand body = Chassis::mapExternalCommandToBody(command);

    EXPECT_NEAR(body.vel_x, -1.25f, 1.0e-6f);
    EXPECT_NEAR(body.vel_y, 2.50f, 1.0e-6f);
    EXPECT_NEAR(body.omega_z, 0.75f, 1.0e-6f);
}

TEST_CASE("testDebugBodySpeedLeftStickUpTargetsCurrentTwoThreeSide")
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.max_vel_x_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_vel_y_ = 2.0f;
    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 1U;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = 0U;

    chassis.airjoy_data_.left_y = 1.0f;
    chassis.airjoy_data_.left_x = 0.0f;
    chassis.applyDebugTargetOverride(Chassis::DebugMode::kBodySpeed);

    EXPECT_TRUE(chassis.input_target_data_.mode == Chassis::Mode::kBodySpeedMode);
    EXPECT_NEAR(chassis.input_target_data_.vel_x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.input_target_data_.vel_y, -2.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.input_target_data_.omega_z, 0.0f, 1.0e-6f);
}

TEST_CASE("testDebugBodySpeedLeftStickLeftTargetsCurrentThreeFourSide")
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.max_vel_x_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_vel_y_ = 2.0f;
    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 1U;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = 0U;

    chassis.airjoy_data_.left_y = 0.0f;
    chassis.airjoy_data_.left_x = -1.0f;
    chassis.applyDebugTargetOverride(Chassis::DebugMode::kBodySpeed);

    EXPECT_TRUE(chassis.input_target_data_.mode == Chassis::Mode::kBodySpeedMode);
    EXPECT_NEAR(chassis.input_target_data_.vel_x, 2.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.input_target_data_.vel_y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.input_target_data_.omega_z, 0.0f, 1.0e-6f);
}

TEST_CASE("testPlannerAxisNormalizationDoesNotDependOnDebugStyleOmegaFlip")
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

TEST_CASE("testSteerGeometryUsesSignedInstallationAngleOnly")
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

TEST_CASE("testRuntimeZeroAndMotorPolarityOnlyAffectMotorLocalConversion")
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

TEST_CASE("testSteerMotorSignAndRuntimeZeroOffsetStayAsIndependentMappingStages")
{
    const float raw_motor_total_rad = jia::degToRadF32(75.0f);
    const float signed_local_total_rad = Chassis::mapRawSteerMotorTotalToSignedLocalTotal(raw_motor_total_rad, -1.0f);
    const float corrected_local_total_rad = Chassis::applyHomingRuntimeZeroOffset(signed_local_total_rad, jia::degToRadF32(20.0f));

    EXPECT_NEAR(jia::radToDegF32(signed_local_total_rad), -75.0f, 1.0e-4f);
    EXPECT_NEAR(jia::radToDegF32(corrected_local_total_rad), -55.0f, 1.0e-4f);
    EXPECT_NEAR(Chassis::removeHomingRuntimeZeroOffset(corrected_local_total_rad, jia::degToRadF32(20.0f)), signed_local_total_rad, 1.0e-6f);
    EXPECT_NEAR(Chassis::mapSignedLocalTotalToRawSteerMotorTotal(signed_local_total_rad, -1.0f), raw_motor_total_rad, 1.0e-6f);
}

TEST_CASE("testTelemetrySnapshotKeepsTargetAndActualYawSemanticsSeparate")
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

TEST_CASE("testTelemetrySnapshotPreservesWheelTargetsWithoutModeDependentReinterpretation")
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

TEST_CASE("testDriveMotorHardwarePolarityMapsCurrentWithoutLeakingIntoGeometry")
{
    Chassis::SteerCalibration calibration{};
    calibration.drive_motor_sign = -1.0f;

    EXPECT_NEAR(Chassis::mapWheelCurrentToDriveMotorCurrent(3000.0f, calibration), -3000.0f, 1.0e-6f);
    EXPECT_NEAR(Chassis::mapWheelCurrentToDriveMotorCurrent(-1200.0f, calibration), 1200.0f, 1.0e-6f);

    calibration.drive_motor_sign = 1.0f;
    EXPECT_NEAR(Chassis::mapWheelCurrentToDriveMotorCurrent(3000.0f, calibration), 3000.0f, 1.0e-6f);
}

TEST_CASE("testDefaultDriveMotorPolarityMatchesPositiveBodyXHardwareLayout")
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

TEST_CASE("testPlannerInputNormalizationKeepsWorldBodyAndSteerOnlySemanticsExplicit")
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

TEST_CASE("testNormalizedBodyCommandKeepsDebugAndApiRoutesSemanticallyAligned")
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

TEST_CASE("testHomingRuntimeZeroOffsetOnlyDependsOnEdgeGeometryAndRawMotorAngle")
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

TEST_CASE("testDebugRouteClassificationSeparatesInputInjectionFromModuleOverride")
{
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(false, 1) == Chassis::DebugControlRoute::kDisabled);
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(true, 1) == Chassis::DebugControlRoute::kTargetInjection);
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(true, 8) == Chassis::DebugControlRoute::kTargetInjection);
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(true, 9) == Chassis::DebugControlRoute::kTargetInjection);
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(true, 20) == Chassis::DebugControlRoute::kTargetInjection);
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(true, 30) == Chassis::DebugControlRoute::kModuleOverride);
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(true, 31) == Chassis::DebugControlRoute::kTargetInjection);
    EXPECT_TRUE(Chassis::classifyDebugControlRoute(true, 32) == Chassis::DebugControlRoute::kTargetInjection);
}

TEST_CASE("testDebugModuleOverrideRouteSeparatesRetiredMode20AlignHomingAndSingleWheelModes")
{
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(1) == Chassis::DebugModuleOverrideRoute::kNone);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(20) == Chassis::DebugModuleOverrideRoute::kNone);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(21) == Chassis::DebugModuleOverrideRoute::kAlignForward);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(22) == Chassis::DebugModuleOverrideRoute::kHomingObserve);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(30) == Chassis::DebugModuleOverrideRoute::kSingleWheelIsolated);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(31) == Chassis::DebugModuleOverrideRoute::kNone);
    EXPECT_TRUE(Chassis::classifyDebugModuleOverrideRoute(32) == Chassis::DebugModuleOverrideRoute::kNone);
}

TEST_CASE("testResolveDebugModeFallsBackToTorqueFreeWhenMode20IsRetired")
{
    Chassis chassis;
    EXPECT_TRUE(chassis.resolveDebugMode(20) == Chassis::DebugMode::kTorqueFree);
    EXPECT_TRUE(chassis.resolveDebugMode(255) == Chassis::DebugMode::kTorqueFree);
    EXPECT_TRUE(chassis.resolveDebugMode(9) == Chassis::DebugMode::kSteerDegAndDriveSpeed);
    EXPECT_TRUE(chassis.resolveDebugMode(30) == Chassis::DebugMode::kSingleWheelIsolated);
    EXPECT_TRUE(chassis.resolveDebugMode(31) == Chassis::DebugMode::kTorqueFree);
    EXPECT_TRUE(chassis.resolveDebugMode(32) == Chassis::DebugMode::kTorqueFree);
}

TEST_CASE("testLowSpeedDriveSuppressionCanBeDisabled")
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

TEST_CASE("testProjectedDriveUsesReachableSteerInsteadOfIdealVector")
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

TEST_CASE("testHighSpeedDriveSuppressionTightensAndReleasesThroughPlannerOutput")
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

TEST_CASE("testLowSpeedDriveSuppressionDoesNotReenterInsideNearZeroHysteresisBand")
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

TEST_CASE("testHighSpeedDriveSuppressionWaitsUntilNearZeroExitBeforeEnabling")
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

} // namespace chassis_semantics_test
