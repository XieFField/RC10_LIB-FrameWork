#include "test_chassis_semantics_harness.h"

namespace chassis_semantics_test
{

// 覆盖 mode30 单轮调试、JustFloat 观测 payload 和调试注入路径。
// 这类测试更关心“目标轮是否被隔离”和“调试镜像是否按约定发布”，不是底盘整体轨迹。
TEST_CASE("testMode30SingleWheelDirectJoystickIsolationOnlyLetsTargetWheelMove")
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

TEST_CASE("testMode30SingleWheelDirectDriveCanUseSCurveShaping")
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

TEST_CASE("testMode30CommonWheelIndexAliasCanDirectlySelectTargetWheel")
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

TEST_CASE("testMode30DirectDriveIgnoresAllHomedGateForTargetWheel")
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

TEST_CASE("testMode30SingleWheelRemovedModes31And32DoNotEnterModuleOverride")
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

TEST_CASE("testMode30SingleWheelUsesConfiguredAxesAndInversion")
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

TEST_CASE("testMode30SingleWheelSharedDeadzoneSuppressesContinuousAndStepInputs")
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

TEST_CASE("testMode30SingleWheelSharedDeadzoneRemapsContinuousInputFromEdge")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.single_wheel.input_deadzone = 0.3f;
    chassis.debug_control_.single_wheel.drive.input_mode_raw = static_cast<unsigned char>(Chassis::DirectAxisInputMode::kRcContinuous);
    chassis.debug_control_.single_wheel.drive.command_limit = 1000.0f;
    chassis.airjoy_data_.left_x = 0.65f;
    chassis.airjoy_data_.right_x = 0.65f;

    EXPECT_TRUE(runHostDebugControlCycle(chassis));

    const float remapped = (0.65f - 0.3f) / (1.0f - 0.3f);
    EXPECT_NEAR(chassis.debug_control_.single_wheel.steer.command_value, remapped * 90.0f, 1.0e-4f);
    EXPECT_NEAR(chassis.debug_control_.single_wheel.drive.command_value, remapped * 1000.0f, 1.0e-4f);
    EXPECT_NEAR(steer_motors[1].getTargetTotalAngle(), remapped * 90.0f, 1.0e-4f);
    EXPECT_NEAR(drive_motors[1].getTargetRPM(), remapped * 1000.0f, 1.0e-4f);
}

TEST_CASE("testMode30SingleWheelSharedDeadzoneRemapsStepThresholdDecision")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.single_wheel.input_deadzone = 0.3f;
    chassis.debug_control_.single_wheel.drive.input_mode_raw = static_cast<unsigned char>(Chassis::DirectAxisInputMode::kRcStep);
    chassis.debug_control_.single_wheel.drive.step_threshold = 0.2f;
    chassis.debug_control_.single_wheel.drive.step_value = 200.0f;

    chassis.airjoy_data_.right_x = 0.42f;
    EXPECT_TRUE(runHostDebugControlCycle(chassis));
    EXPECT_NEAR(chassis.debug_control_.single_wheel.drive.command_value, 0.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[1].getTargetRPM(), 0.0f, 1.0e-6f);

    chassis.airjoy_data_.right_x = 0.50f;
    EXPECT_TRUE(runHostDebugControlCycle(chassis));
    EXPECT_NEAR(chassis.debug_control_.single_wheel.drive.command_value, 200.0f, 1.0e-6f);
    EXPECT_NEAR(drive_motors[1].getTargetRPM(), 200.0f, 1.0e-6f);
}

TEST_CASE("testMode30SingleWheelAxisEnablesAndEstopGateOutputs")
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

TEST_CASE("testMode30SingleWheelSteerPlannerSupportsSCurveAndTrapezoid")
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

TEST_CASE("testMode30SingleWheelDrivePlannerSupportsTrapezoidAndIgnoresGlobalManualProfileParams")
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

TEST_CASE("testRefreshDebugMirrorPublishesHomingDiagnosticsForObserveMode")
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

TEST_CASE("testMode30SingleWheelNonTargetCommandTypesBypassPlanner")
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

TEST_CASE("testMode30SingleWheelPlannerStateResetsWhenWheelAndPlannerModeChange")
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

TEST_CASE("testJustFloatSingleWheelProfileUsesObserveWheelIndex")
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

TEST_CASE("testJustFloatSingleWheelPayloadUsesObserveWheelIndex")
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

TEST_CASE("testJustFloatSingleWheelObserveIndexFallsBackToZeroWhenOutOfRange")
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

TEST_CASE("testJustFloatSingleWheelDriveOnlyPayloadUsesObserveWheelIndex")
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

TEST_CASE("testJustFloatSingleWheelDriveOnlyObserveIndexFallsBackToZeroWhenOutOfRange")
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

TEST_CASE("testMode30SingleWheelDirectDriveUsesManualInputWithoutRemovedStepGenerator")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelDriveVescHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.single_wheel.drive.command_value = 100.0f;

    EXPECT_TRUE(runHostDebugControlCycle(chassis));
    EXPECT_NEAR(drive_motors[1].getTargetRPM(), 100.0f, 1.0e-4f);
}

TEST_CASE("testRemovedDebugOutputFamilyRawValueFallsBackToOffAndDoesNotEmitPayload")
{
    Chassis chassis;
    testHostResetJustFloatCapture();
    configureDebugOutputFamily(chassis, 3U);
    chassis.time_ms_ = 120U;

    emitDebugOutputForHost(chassis, true);

    EXPECT_TRUE(!g_test_justfloat_capture.called);
}

TEST_CASE("testMode30DirectDriveIgnoresAllHomedGateForTargetWheel")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelDriveVescHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.single_wheel.drive.command_value = 180.0f;

    // mode30 目标轮直控本来就应绕过全车 homing gate，虚拟负载也应该跟着这条语义走。
    chassis.computeSingleWheelIsolatedCommandsMode30(1U, false);

    EXPECT_NEAR(drive_motors[1].getTargetRPM(), 180.0f, 1.0e-4f);
}

TEST_CASE("testMode30DirectDriveIgnoresOtherWheelSteerFaultForTargetWheel")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelDriveVescHarness(chassis, steer_motors, drive_motors);

    chassis.debug_control_.single_wheel.drive.command_value = 180.0f;
    chassis.wheel_config_[0].steer_fault_state = Chassis::SteerFaultState::kRecovering;

    // 单轮虚拟负载只服务目标轮速度环，不该被其他轮的 steer fault 全车级短路。
    chassis.computeSingleWheelIsolatedCommandsMode30(1U, true);

    EXPECT_NEAR(drive_motors[1].getTargetRPM(), 180.0f, 1.0e-4f);
}

TEST_CASE("testRemovedDrivePidLoadProfileRawValueFallsBackToYawPidSafeProfile")
{
    Chassis chassis;
    testHostResetJustFloatCapture();
    configureDebugOutputFamily(chassis, 2U);
    configureJustFloatProfile(chassis, 3U);
    chassis.debug_output_.justfloat.yaw_pid.period_ms = 0U;
    chassis.debug_output_runtime_.justfloat.yaw_pid.last_ms = 0U;
    chassis.time_ms_ = 120U;
    chassis.yaw_pid_trace_.mode_tag = 0.5f;
    chassis.yaw_pid_trace_.target_yaw_rad = 0.25f;
    chassis.debug_mirror_.all_homed = true;
    chassis.debug_mirror_.steer_fault_any_active = false;
    for (int i = 0; i < 4; ++i)
    {
        chassis.debug_mirror_.motion_direction_guard_active[i] = false;
    }
    chassis.debug_mirror_.reverse_intent_active = false;

    emitDebugOutputForHost(chassis, true);

    EXPECT_TRUE(g_test_justfloat_capture.called);
    EXPECT_TRUE(g_test_justfloat_capture.size == 15U);
    EXPECT_NEAR(g_test_justfloat_capture.values[0], 0.12f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[1], 0.5f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[2], 0.25f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[12], 1.0f, 1.0e-6f);
}

TEST_CASE("testJustFloatDriveZeroStopBrakeTraceEmitsFixed12ChannelPayloadWhenBrakeInactive")
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveContinuityHarness(chassis, drive_motors);

    testHostResetJustFloatCapture();
    configureDebugOutputFamily(chassis, 2U);
    configureJustFloatProfile(chassis, 4U);
    chassis.debug_output_.justfloat.drive_zero_stop_brake.period_ms = 0U;
    chassis.debug_output_runtime_.justfloat.drive_zero_stop_brake.last_ms = 0U;
    chassis.time_ms_ = 220U;
    chassis.debug_control_.common.observe_wheel_index = 2U;
    chassis.debug_drive_zero_stop_brake_trace_.observe_wheel_idx = 2.0f;
    chassis.debug_drive_zero_stop_brake_trace_.target_rpm = 180.0f;
    chassis.debug_drive_zero_stop_brake_trace_.feedback_rpm = 175.0f;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.target_data_.vel_x = 0.015f;
    chassis.target_data_.vel_y = 0.0f;
    chassis.target_data_.omega_z = 0.25f;
    chassis.drive_zero_stop_brake_active_[2] = false;
    chassis.drive_zero_stop_active_ = false;
    chassis.wheel_config_[2].corrected_drive_omega_rad_s = 1.4f;
    drive_motors[2].setTargetCurrent(0.0f);
    drive_motors[2].setFeedbackCurrent(321.0f);

    emitDebugOutputForHost(chassis, true);

    EXPECT_TRUE(g_test_justfloat_capture.called);
    EXPECT_TRUE(g_test_justfloat_capture.size == 12U);
    EXPECT_NEAR(g_test_justfloat_capture.values[0], 0.22f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[1], 2.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[2], 180.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[3], 175.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[4], 0.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[5], 0.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[6], 0.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[7], 321.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[8], 0.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[9], 0.07f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[10], 0.015f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[11], 0.25f, 1.0e-6f);
}

TEST_CASE("testJustFloatDriveZeroStopBrakeTraceEmitsBrakeStateAndCurrent")
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveContinuityHarness(chassis, drive_motors);

    testHostResetJustFloatCapture();
    configureDebugOutputFamily(chassis, 2U);
    configureJustFloatProfile(chassis, 4U);
    chassis.debug_output_.justfloat.drive_zero_stop_brake.period_ms = 0U;
    chassis.debug_output_runtime_.justfloat.drive_zero_stop_brake.last_ms = 0U;
    chassis.time_ms_ = 360U;
    chassis.debug_control_.common.observe_wheel_index = 1U;
    chassis.debug_drive_zero_stop_brake_trace_.observe_wheel_idx = 1.0f;
    chassis.debug_drive_zero_stop_brake_trace_.target_rpm = 90.0f;
    chassis.debug_drive_zero_stop_brake_trace_.feedback_rpm = 110.0f;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.target_data_.vel_x = 0.0f;
    chassis.target_data_.vel_y = 0.0f;
    chassis.target_data_.omega_z = -0.4f;
    chassis.drive_zero_stop_brake_active_[1] = true;
    chassis.drive_zero_stop_active_ = true;
    chassis.wheel_config_[1].corrected_drive_omega_rad_s = 3.2f;
    drive_motors[1].setBrake(25000.0f);
    drive_motors[1].setFeedbackCurrent(-18600.0f);

    emitDebugOutputForHost(chassis, true);

    EXPECT_TRUE(g_test_justfloat_capture.called);
    EXPECT_TRUE(g_test_justfloat_capture.size == 12U);
    EXPECT_NEAR(g_test_justfloat_capture.values[0], 0.36f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[1], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[2], 90.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[3], 110.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[4], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[5], 25000.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[6], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[7], -18600.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[8], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[9], 0.16f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[10], 0.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[11], -0.4f, 1.0e-6f);
}

TEST_CASE("testJustFloatHomingTraceOverviewEmitsFixed21ChannelPayload")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelDebugHarness(chassis, steer_motors, drive_motors);
    for (int i = 0; i < 4; ++i)
    {
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
    }
    testHostResetJustFloatCapture();
    configureDebugOutputFamily(chassis, 2U);
    configureJustFloatProfile(chassis, 5U);
    chassis.debug_output_.justfloat.homing_trace.view_raw = 0U;
    chassis.debug_output_.justfloat.homing_trace.overview.period_ms = 0U;
    chassis.debug_output_runtime_.justfloat.homing_trace.overview.last_ms = 0U;
    chassis.time_ms_ = 480U;
    chassis.first_boot_homing_delay_.pending = true;
    chassis.first_boot_homing_delay_.active = true;
    chassis.first_boot_homing_delay_.elapsed_ms = 123U;
    chassis.homing_start_request_ = true;
    chassis.steer_fault_any_active_ = true;
    chassis.wheel_config_[0].homing_state = Chassis::HomingState::kIdle;
    chassis.wheel_config_[1].homing_state = Chassis::HomingState::kSearch;
    chassis.wheel_config_[2].homing_state = Chassis::HomingState::kEdgeDetected;
    chassis.wheel_config_[3].homing_state = Chassis::HomingState::kReady;
    chassis.wheel_config_[0].homing_last_edge_is_falling = false;
    chassis.wheel_config_[1].homing_last_edge_is_falling = true;
    chassis.wheel_config_[2].homing_last_edge_is_falling = false;
    chassis.wheel_config_[3].homing_last_edge_is_falling = true;
    setPhotogateStateForWheel(0, false);
    setPhotogateStateForWheel(1, true);
    setPhotogateStateForWheel(2, false);
    setPhotogateStateForWheel(3, true);

    emitDebugOutputForHost(chassis, false);

    EXPECT_TRUE(g_test_justfloat_capture.called);
    EXPECT_TRUE(g_test_justfloat_capture.size == 21U);
    EXPECT_NEAR(g_test_justfloat_capture.values[0], 0.48f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[1], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[2], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[3], 123.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[4], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[5], 0.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[6], 0.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[7], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[8], 2.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[9], 5.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[10], 0.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[11], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[12], 0.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[13], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[14], 0.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[15], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[16], 0.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[17], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[18], chassis.wheel_config_[0].homing_search_rpm, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[19], chassis.wheel_config_[1].homing_search_rpm, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[20], 1.0f, 1.0e-6f);
}

TEST_CASE("testJustFloatHomingTraceObserveWheelDetailEmitsFixed25ChannelPayload")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSingleWheelDebugHarness(chassis, steer_motors, drive_motors);
    configureDriveContinuityHarness(chassis, drive_motors);
    for (int i = 0; i < 4; ++i)
    {
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
    }

    testHostResetJustFloatCapture();
    configureDebugOutputFamily(chassis, 2U);
    configureJustFloatProfile(chassis, 5U);
    chassis.debug_output_.justfloat.homing_trace.view_raw = 1U;
    chassis.debug_output_.justfloat.homing_trace.detail.period_ms = 0U;
    chassis.debug_output_runtime_.justfloat.homing_trace.detail.last_ms = 0U;
    chassis.time_ms_ = 612U;
    chassis.debug_control_.common.observe_wheel_index = 2U;
    chassis.first_boot_homing_delay_.pending = false;
    chassis.first_boot_homing_delay_.active = true;
    chassis.first_boot_homing_delay_.elapsed_ms = 456U;
    chassis.homing_start_request_ = true;
    chassis.wheel_config_[2].homing_state = Chassis::HomingState::kSearch;
    chassis.wheel_config_[2].homing_last_sensor_active = true;
    chassis.wheel_config_[2].homing_last_edge_is_falling = true;
    chassis.wheel_config_[2].homing_edge_confirm_count = 2U;
    chassis.wheel_config_[2].homing_last_confirm_edge_is_falling = true;
    chassis.wheel_config_[2].homing_search_timeout_armed = true;
    chassis.wheel_config_[2].homing_elapsed_s = 0.789f;
    chassis.wheel_config_[2].homing_zero_valid = false;
    chassis.wheel_config_[2].steer_fault_state = Chassis::SteerFaultState::kRecovering;
    chassis.wheel_config_[2].steer_fault_rehome_request = true;
    chassis.wheel_config_[2].homing_auto_retry_attempt_count = 3U;
    chassis.wheel_config_[2].homing_auto_retry_wait_active = true;
    chassis.wheel_config_[2].homing_auto_retry_wait_elapsed_ms = 654U;
    chassis.wheel_config_[2].homing_auto_retry_armed_by_recovery_failure = true;
    chassis.wheel_config_[2].theta_oa_to_owi_rad = jia::degToRadF32(90.0f);
    chassis.wheel_config_[2].homing_runtime_zero_offset_rad = jia::degToRadF32(17.0f);
    chassis.wheel_config_[2].homing_hold_corrected_local_total_rad = jia::degToRadF32(123.0f);
    chassis.wheel_config_[2].corrected_steer_motor_total_angle_rad = jia::degToRadF32(30.0f);
    chassis.wheel_config_[2].target_steer_motor_total_angle_rad = jia::degToRadF32(45.0f);
    chassis.wheel_config_[2].steer_feedback_current_mA = 2345.0f;
    chassis.wheel_config_[2].steer_feedback_angle_delta_rad = jia::degToRadF32(2.5f);
    setPhotogateStateForWheel(2, true);
    steer_motors[2].setTargetRPM(321.0f);

    emitDebugOutputForHost(chassis, false);

    EXPECT_TRUE(g_test_justfloat_capture.called);
    EXPECT_TRUE(g_test_justfloat_capture.size == 25U);
    EXPECT_NEAR(g_test_justfloat_capture.values[0], 0.612f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[1], 2.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[2], 0.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[3], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[4], 456.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[5], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[6], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[7], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[8], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[9], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[10], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[11], 2.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[12], 0.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[13], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[14], 0.789f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[15], 0.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[16], 2.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[17], 1.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[18], 120.0f, 1.0e-4f);
    EXPECT_NEAR(g_test_justfloat_capture.values[19], 135.0f, 1.0e-4f);
    EXPECT_NEAR(g_test_justfloat_capture.values[20], 17.0f, 1.0e-4f);
    EXPECT_NEAR(g_test_justfloat_capture.values[21], 123.0f, 1.0e-4f);
    EXPECT_NEAR(g_test_justfloat_capture.values[22], chassis.wheel_config_[2].homing_search_rpm, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[23], 3.0f, 1.0e-6f);
    EXPECT_NEAR(g_test_justfloat_capture.values[24], 1.0f, 1.0e-6f);
}

TEST_CASE("testDebugOmegaZInjectionModeOffKeepsManualOmegaInput")
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.max_vel_x_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_vel_y_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_omega_z_ = 3.0f;
    chassis.debug_control_.injection.translation_input_deadzone = 0.0f;
    chassis.debug_control_.injection.rotation_input_deadzone = 0.0f;
    chassis.airjoy_data_.left_y = 0.0f;
    chassis.airjoy_data_.left_x = 0.0f;
    chassis.airjoy_data_.right_x = 0.5f;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = 0U;

    chassis.applyDebugTargetOverride(Chassis::DebugMode::kBodySpeed);

    EXPECT_TRUE(chassis.input_target_data_.mode == Chassis::Mode::kBodySpeedMode);
    EXPECT_NEAR(chassis.input_target_data_.omega_z, 1.5f, 1.0e-6f);
}

TEST_CASE("testDebugOmegaZInjectionModeStepOverridesManualOmegaInput")
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

TEST_CASE("testDebugOmegaZInjectionModeSineOverridesManualOmegaInput")
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

TEST_CASE("testDebugOmegaZInjectionDoesNotAffectLockToTarget")
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

TEST_CASE("testDebugXParkBrakeModesRouteToDedicatedLockYawTargets")
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.max_vel_x_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_vel_y_ = 3.0f;
    chassis.runtime_strategy_cfg_.max_omega_z_ = 4.0f;
    chassis.debug_control_.injection.translation_input_deadzone = 0.0f;
    chassis.debug_control_.injection.rotation_input_deadzone = 0.0f;
    chassis.debug_control_.injection.lock_rot_z = -0.75f;
    chassis.airjoy_data_.left_x = 0.25f;
    chassis.airjoy_data_.left_y = -0.5f;

    chassis.applyDebugTargetOverride(Chassis::DebugMode::kBodyLockNowXParkBrake);
    EXPECT_TRUE(chassis.input_target_data_.mode == Chassis::Mode::kBodySpeedLockNowRotZWithXParkBrakeMode);
    EXPECT_NEAR(chassis.input_target_data_.vel_x, -0.25f * 2.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.input_target_data_.vel_y, 0.5f * 3.0f, 1.0e-6f);

    chassis.applyDebugTargetOverride(Chassis::DebugMode::kWorldLockNowXParkBrake);
    EXPECT_TRUE(chassis.input_target_data_.mode == Chassis::Mode::kWorldSpeedLockNowRotZWithXParkBrakeMode);

    chassis.applyDebugTargetOverride(Chassis::DebugMode::kBodyLockToXParkBrake);
    EXPECT_TRUE(chassis.input_target_data_.mode == Chassis::Mode::kBodySpeedLockToRotZWithXParkBrakeMode);
    EXPECT_NEAR(chassis.input_target_data_.rot_z, -0.75f, 1.0e-6f);

    chassis.applyDebugTargetOverride(Chassis::DebugMode::kWorldLockToXParkBrake);
    EXPECT_TRUE(chassis.input_target_data_.mode == Chassis::Mode::kWorldSpeedLockToRotZWithXParkBrakeMode);
    EXPECT_NEAR(chassis.input_target_data_.rot_z, -0.75f, 1.0e-6f);
}

TEST_CASE("testRefreshDebugMirrorPublishesXParkPriorityBrakeFields")
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.xpark_priority_brake_cfg_.residual_enter_m_s = 0.08f;
    chassis.runtime_strategy_cfg_.xpark_priority_brake_cfg_.residual_exit_m_s = 0.10f;
    chassis.runtime_strategy_cfg_.xpark_priority_brake_cfg_.entry_delay_ms = 12U;
    chassis.current_mode_flag_.use_xpark_priority_brake = true;
    chassis.xpark_priority_brake_gate_active_ = true;
    chassis.xpark_priority_brake_skip_yaw_reengage_active_ = true;
    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = static_cast<unsigned char>(Chassis::DebugMode::kBodyLockToXParkBrake);

    chassis.refreshDebugMirror(false);

    EXPECT_TRUE(chassis.debug_mirror_.xpark_priority_brake_mode_active);
    EXPECT_TRUE(chassis.debug_mirror_.xpark_priority_brake_threshold_active);
    EXPECT_TRUE(chassis.debug_mirror_.xpark_priority_brake_skip_yaw_reengage);
    EXPECT_NEAR(chassis.debug_mirror_.xpark_priority_brake_residual_enter_m_s, 0.08f, 1.0e-6f);
    EXPECT_NEAR(chassis.debug_mirror_.xpark_priority_brake_residual_exit_m_s, 0.10f, 1.0e-6f);
    EXPECT_NEAR(chassis.debug_mirror_.xpark_priority_brake_entry_delay_ms, 12.0f, 1.0e-6f);
}

TEST_CASE("testDebugSteerDegAndDriveSpeedModeMapsLeftXAndRightXToInterface")
{
    Chassis chassis;
    chassis.debug_control_.injection.translation_input_deadzone = 0.0f;
    chassis.debug_control_.injection.rotation_input_deadzone = 0.0f;
    chassis.debug_control_.injection.steer_deg_limit = 180.0f;
    chassis.debug_control_.injection.drive_speed_m_s_limit = 1.2f;
    chassis.airjoy_data_.left_x = 0.0f;
    chassis.airjoy_data_.right_x = 0.0f;

    chassis.applyDebugTargetOverride(Chassis::DebugMode::kSteerDegAndDriveSpeed);

    EXPECT_TRUE(chassis.input_target_data_.mode == Chassis::Mode::kSteerAngleAndDriveSpeedMode);
    EXPECT_NEAR(chassis.input_target_data_.steer_lock_angle_deg, 90.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.input_target_data_.drive_lock_speed_m_s, 0.0f, 1.0e-6f);

    chassis.airjoy_data_.left_x = 0.25f;
    chassis.airjoy_data_.right_x = -0.5f;

    chassis.applyDebugTargetOverride(Chassis::DebugMode::kSteerDegAndDriveSpeed);

    EXPECT_TRUE(chassis.input_target_data_.mode == Chassis::Mode::kSteerAngleAndDriveSpeedMode);
    EXPECT_NEAR(chassis.input_target_data_.steer_lock_angle_deg, 135.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.input_target_data_.drive_lock_speed_m_s, -0.6f, 1.0e-6f);
    EXPECT_NEAR(chassis.input_target_data_.omega_z, 0.0f, 1.0e-6f);
}

TEST_CASE("testDebugTargetInjectionDeadzoneRemapsTranslationAndRotationInputs")
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.max_vel_x_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_vel_y_ = 3.0f;
    chassis.runtime_strategy_cfg_.max_omega_z_ = 4.0f;
    chassis.debug_control_.injection.translation_input_deadzone = 0.2f;
    chassis.debug_control_.injection.rotation_input_deadzone = 0.4f;
    chassis.airjoy_data_.left_x = 0.5f;
    chassis.airjoy_data_.left_y = -0.6f;
    chassis.airjoy_data_.right_x = -0.7f;

    chassis.applyDebugTargetOverride(Chassis::DebugMode::kBodySpeed);

    const float left_x_remapped = (0.5f - 0.2f) / (1.0f - 0.2f);
    const float left_y_remapped = -(0.6f - 0.2f) / (1.0f - 0.2f);
    const float right_x_remapped = -(0.7f - 0.4f) / (1.0f - 0.4f);
    EXPECT_TRUE(chassis.input_target_data_.mode == Chassis::Mode::kBodySpeedMode);
    EXPECT_NEAR(chassis.input_target_data_.vel_x, -left_x_remapped * 2.0f, 1.0e-5f);
    EXPECT_NEAR(chassis.input_target_data_.vel_y, -left_y_remapped * 3.0f, 1.0e-5f);
    EXPECT_NEAR(chassis.input_target_data_.omega_z, right_x_remapped * 4.0f, 1.0e-5f);
}

TEST_CASE("testDebugTargetInjectionDeadzoneControlsStepAndSteerDriveMapping")
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.max_omega_z_ = 3.0f;
    chassis.debug_control_.injection.translation_input_deadzone = 0.2f;
    chassis.debug_control_.injection.rotation_input_deadzone = 0.4f;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = static_cast<unsigned char>(Chassis::DebugOmegaZInjectionMode::kStep);
    chassis.debug_control_.injection.steer_deg_limit = 180.0f;
    chassis.debug_control_.injection.drive_speed_m_s_limit = 1.2f;

    chassis.airjoy_data_.right_x = 0.55f;
    chassis.applyDebugTargetOverride(Chassis::DebugMode::kBodySpeed);
    EXPECT_NEAR(chassis.input_target_data_.omega_z, 0.0f, 1.0e-6f);

    chassis.airjoy_data_.right_x = 0.8f;
    chassis.applyDebugTargetOverride(Chassis::DebugMode::kBodySpeed);
    EXPECT_NEAR(chassis.input_target_data_.omega_z, 3.0f, 1.0e-6f);

    chassis.airjoy_data_.left_x = 0.3f;
    chassis.airjoy_data_.right_x = -0.7f;
    chassis.applyDebugTargetOverride(Chassis::DebugMode::kSteerDegAndDriveSpeed);

    const float left_x_remapped = (0.3f - 0.2f) / (1.0f - 0.2f);
    const float right_x_remapped = -(0.7f - 0.4f) / (1.0f - 0.4f);
    EXPECT_NEAR(chassis.input_target_data_.steer_lock_angle_deg, 90.0f + left_x_remapped * 180.0f, 1.0e-5f);
    EXPECT_NEAR(chassis.input_target_data_.drive_lock_speed_m_s, right_x_remapped * 1.2f, 1.0e-5f);
}

} // namespace chassis_semantics_test
