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

TEST_CASE("testMode30SingleWheelDriveStepGeneratorOverridesManualInput")
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

TEST_CASE("testDriveVirtualLoadInjectsBiasOnlyForSelectedWheelAndPidMode")
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

TEST_CASE("testMode30DriveVirtualLoadIgnoresAllHomedGateForTargetWheel")
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

TEST_CASE("testMode30DriveVirtualLoadIgnoresOtherWheelSteerFaultForTargetWheel")
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

TEST_CASE("testJustFloatDrivePidLoadProfileEmitsFixed16ChannelPayload")
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
    chassis.debug_drive_load_trace_.observe_wheel_idx = 2.0f;
    chassis.debug_drive_load_trace_.target_rpm = 180.0f;
    chassis.debug_drive_load_trace_.feedback_rpm = 175.0f;
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
    chassis.debug_drive_load_trace_.observe_wheel_idx = 1.0f;
    chassis.debug_drive_load_trace_.target_rpm = 90.0f;
    chassis.debug_drive_load_trace_.feedback_rpm = 110.0f;
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

TEST_CASE("testDebugOmegaZInjectionModeOffKeepsManualOmegaInput")
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

TEST_CASE("testDebugSteerDegAndDriveSpeedModeMapsLeftXAndRightXToInterface")
{
    Chassis chassis;
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

} // namespace chassis_semantics_test
