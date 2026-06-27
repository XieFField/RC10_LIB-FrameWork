#include "test_chassis_semantics_harness.h"

namespace chassis_semantics_test
{

// 覆盖 planner 输出到电机命令之间的交付层，尤其是 zero-stop brake、残余速度收尾、
// drive alpha/omega 限幅后的连续性。这里的断言直接保护“目标速度决定刹车模式”等语义。
TEST_CASE("testSuppressedDriveDoesNotAccumulateHiddenAccelState")
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

TEST_CASE("testDriveReleaseResumesFromDeliveredSpeedNotVirtualSpeed")
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

TEST_CASE("testDriveReleaseHasNoVelocityJumpAfterZeroHold")
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

TEST_CASE("testPlannerTargetMayChangeWhileDeliveredStateRemainsContinuous")
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

TEST_CASE("testTorqueFreeAndNotHomedPathsResetDriveDeliveryStateConsistently")
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

TEST_CASE("testDriveZeroStopUsesBrakeWhenResidualSpeedIsStillHigh")
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

TEST_CASE("testDriveZeroStopStillUsesBrakeWithNativeVescSpeedLoop")
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveZeroStopHarness(chassis, drive_motors);

    for (int i = 0; i < 4; ++i)
    {
        drive_motors[i].setRpmControlMode(VESC_RPM_CONTROL_NATIVE_ERPM);
    }

    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(0.0f));
    setWheelResidualSpeedMps(chassis, 0, 0.15f);

    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_NEAR(drive_motors[0].getTargetBrake(), 1200.0f, 1.0e-6f);
}

TEST_CASE("testNormalControlZeroStopUsesBrakeAfterBodyCommandDropsToZero")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = false;
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_assist = true;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_current_mA = 1200.0f;
    chassis.input_target_data_.mode = Chassis::Mode::kBodySpeedMode;

    for (int i = 0; i < 4; ++i)
    {
        drive_motors[i].setRpmControlMode(VESC_RPM_CONTROL_NATIVE_ERPM);
    }

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;
    EXPECT_TRUE(runHostControlCycle(chassis));
    EXPECT_TRUE(std::fabs(drive_motors[0].getTargetRPM()) > 1.0e-6f);

    const float residual_feedback_rpm = jia::radsToRpmF32(0.15f / chassis.runtime_strategy_cfg_.wheel_radius_m_);
    drive_motors[0].setFeedbackRpm(residual_feedback_rpm);
    drive_motors[0].resetLastCommandObservation();

    chassis.input_target_data_.vel_x = 0.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;
    EXPECT_TRUE(runHostControlCycle(chassis));

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_NEAR(drive_motors[0].getTargetBrake(), 1200.0f, 1.0e-6f);
}

TEST_CASE("testSteerDegAndDriveSpeedSkipsZeroStopWhenDriveSpeedIsNonZero")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = false;
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_assist = true;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_current_mA = 1200.0f;
    chassis.runtime_strategy_cfg_.yaw_lock_zero_stop_release_hold_ms = 2U;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;

    for (int i = 0; i < 4; ++i)
    {
        drive_motors[i].setRpmControlMode(VESC_RPM_CONTROL_NATIVE_ERPM);
    }

    chassis.setSteerDegAndDriveSpeed(30.0f, 0.30f);
    chassis.setModeFlag();

    EXPECT_TRUE(runHostControlCycle(chassis));

    EXPECT_TRUE(!chassis.drive_zero_stop_active_);
    EXPECT_TRUE(!chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kRpm);
    EXPECT_TRUE(std::fabs(drive_motors[0].getTargetRPM()) > 1.0e-6f);
}

TEST_CASE("testSteerDegAndDriveSpeedStillTurnsSteerWhenDriveSpeedIsZero")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = false;
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_assist = true;
    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kHoldLast;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.enable = false;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_current_mA = 1200.0f;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;

    for (int i = 0; i < 4; ++i)
    {
        drive_motors[i].setRpmControlMode(VESC_RPM_CONTROL_NATIVE_ERPM);
    }

    chassis.setSteerDegAndDriveSpeed(30.0f, 0.0f);
    chassis.setModeFlag();

    EXPECT_TRUE(runHostControlCycle(chassis));

    EXPECT_TRUE(chassis.input_target_data_.mode == Chassis::Mode::kSteerAngleAndDriveSpeedMode);
    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_NEAR(chassis.wheel_config_[0].target_steer_motor_total_angle_rad, jia::degToRadF32(30.0f), 1.0e-6f);
    EXPECT_NEAR(chassis.planned_data_.steer_angle_oa_rad[0], jia::degToRadF32(30.0f), 1.0e-6f);
    EXPECT_NEAR(chassis.debug_mirror_.target_oa_deg[0], 30.0f, 1.0e-6f);
    EXPECT_NEAR(steer_motors[0].getTargetTotalAngle(), 30.0f, 1.0e-4f);
    EXPECT_NEAR(drive_motors[0].getTargetRPM(), 0.0f, 1.0e-6f);
}

TEST_CASE("testDriveZeroStopDoesNotWaitForPlannedTailToFullyDecay")
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveZeroStopHarness(chassis, drive_motors);

    // 模拟“整车目标已归零，但速度规划执行帧还留着一个减速尾巴”的现场。
    chassis.input_target_data_.mode = Chassis::Mode::kBodySpeedMode;
    chassis.input_target_data_.vel_x = 0.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;
    chassis.target_data_.vel_x = 0.0f;
    chassis.target_data_.vel_y = 0.0f;
    chassis.target_data_.omega_z = 0.0f;
    chassis.current_mode_flag_.is_wheel_torque_free = false;
    chassis.current_mode_flag_.is_world_speed_mode = false;
    chassis.current_mode_flag_.is_lock_now_rot_z = false;
    chassis.current_mode_flag_.is_lock_to_rot_z = false;

    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(1.0f));
    setWheelResidualSpeedMps(chassis, 0, 0.15f);

    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_NEAR(drive_motors[0].getTargetBrake(), 1200.0f, 1.0e-6f);
}

TEST_CASE("testYawLockZeroStopBrakesBeforePureRotationSteerPlanning")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = false;
    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kHoldLast;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.enable = false;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_assist = true;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_current_mA = 1200.0f;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.input_target_data_.mode = Chassis::Mode::kBodySpeedLockNowRotZMode;
    chassis.input_target_data_.vel_x = 0.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;
    chassis.current_mode_flag_.is_lock_now_rot_z = true;
    chassis.target_data_.vel_x = 0.0f;
    chassis.target_data_.vel_y = 0.0f;
    chassis.target_data_.omega_z = 1.0f;
    chassis.planned_data_.vel_x = 0.0f;
    chassis.planned_data_.vel_y = 0.0f;
    chassis.planned_data_.omega_z = 1.0f;
    chassis.last_planned_data_.vel_x = 0.2f;
    chassis.last_planned_data_.vel_y = 0.0f;
    chassis.last_planned_data_.omega_z = 1.0f;

    setWheelResidualSpeedMps(chassis, 0, 0.15f);
    chassis.computeModuleCommands(chassis.planned_data_);
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_NEAR(drive_motors[0].getTargetBrake(), 1200.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[0].target_steer_motor_total_angle_rad, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.actuator_command_frame_.steer_oa_total_rad[0], 0.0f, 1.0e-6f);

    for (unsigned int i = 0; i < chassis.runtime_strategy_cfg_.yaw_lock_zero_stop_release_hold_ms; ++i)
    {
        setWheelResidualSpeedMps(chassis, 0, 0.0f);
        chassis.computeModuleCommands(chassis.planned_data_);
        chassis.applyModuleCommands(true);
    }

    const float pure_rotation_oa_rad =
        std::atan2(-chassis.planned_data_.omega_z * chassis.wheel_config_[0].pos_x_m,
                   chassis.planned_data_.omega_z * chassis.wheel_config_[0].pos_y_m);
    EXPECT_NEAR(chassis.actuator_command_frame_.steer_oa_total_rad[0], pure_rotation_oa_rad, 1.0e-6f);
}

TEST_CASE("testYawLockZeroStopKeepsBrakeLatchAfterPlannedTranslationTailDecays")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = false;
    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kHoldLast;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.enable = false;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_assist = true;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_current_mA = 1200.0f;
    chassis.runtime_strategy_cfg_.yaw_lock_zero_stop_release_hold_ms = 2U;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.input_target_data_.mode = Chassis::Mode::kBodySpeedLockNowRotZMode;
    chassis.input_target_data_.vel_x = 0.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;
    chassis.current_mode_flag_.is_lock_now_rot_z = true;
    chassis.target_data_.vel_x = 0.0f;
    chassis.target_data_.vel_y = 0.0f;
    chassis.target_data_.omega_z = 1.0f;
    chassis.planned_data_.vel_x = 0.0f;
    chassis.planned_data_.vel_y = 0.0f;
    chassis.planned_data_.omega_z = 1.0f;
    chassis.last_planned_data_.vel_x = 0.2f;
    chassis.last_planned_data_.vel_y = 0.0f;
    chassis.last_planned_data_.omega_z = 1.0f;

    setWheelResidualSpeedMps(chassis, 0, 0.15f);
    chassis.computeModuleCommands(chassis.planned_data_);
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_NEAR(chassis.actuator_command_frame_.steer_oa_total_rad[0], 0.0f, 1.0e-6f);

    drive_motors[0].resetLastCommandObservation();
    chassis.last_planned_data_.vel_x = 0.0f;
    chassis.last_planned_data_.vel_y = 0.0f;
    setWheelResidualSpeedMps(chassis, 0, 0.15f);
    chassis.computeModuleCommands(chassis.planned_data_);
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_NEAR(drive_motors[0].getTargetBrake(), 1200.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.actuator_command_frame_.steer_oa_total_rad[0], 0.0f, 1.0e-6f);

    for (unsigned int i = 0; i < chassis.runtime_strategy_cfg_.yaw_lock_zero_stop_release_hold_ms; ++i)
    {
        setWheelResidualSpeedMps(chassis, 0, 0.0f);
        chassis.computeModuleCommands(chassis.planned_data_);
        chassis.applyModuleCommands(true);
    }

    const float pure_rotation_oa_rad =
        std::atan2(-chassis.planned_data_.omega_z * chassis.wheel_config_[0].pos_x_m,
                   chassis.planned_data_.omega_z * chassis.wheel_config_[0].pos_y_m);
    EXPECT_TRUE(!chassis.drive_zero_stop_active_);
    EXPECT_TRUE(!chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kRpm);
    EXPECT_NEAR(chassis.actuator_command_frame_.steer_oa_total_rad[0], pure_rotation_oa_rad, 1.0e-6f);
}

TEST_CASE("testYawLockZeroStopBrakeLatchOverridesDriveAlphaDecelGuard")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = false;
    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kHoldLast;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.enable = false;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = true;
    chassis.runtime_strategy_cfg_.max_drive_alpha_rad_s2_ = 10.0f;
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_assist = true;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_current_mA = 1200.0f;
    chassis.runtime_strategy_cfg_.yaw_lock_zero_stop_release_hold_ms = 2U;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.input_target_data_.mode = Chassis::Mode::kBodySpeedLockNowRotZMode;
    chassis.current_mode_flag_.is_lock_now_rot_z = true;
    chassis.target_data_.omega_z = 1.0f;
    chassis.planned_data_.omega_z = 1.0f;
    chassis.last_planned_data_.vel_x = 0.2f;
    chassis.last_planned_data_.omega_z = 1.0f;
    chassis.last_drive_omega_cmd_rad_s_[0] = 10.0f;

    setWheelResidualSpeedMps(chassis, 0, 0.15f);
    chassis.computeModuleCommands(chassis.planned_data_);
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_NEAR(chassis.actuator_command_frame_.steer_oa_total_rad[0], 0.0f, 1.0e-6f);
}

TEST_CASE("testYawLockPureYawWithoutTranslationHistorySkipsBrakeLatchDespiteResidual")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = false;
    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kHoldLast;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.enable = false;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_assist = true;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_current_mA = 1200.0f;
    chassis.runtime_strategy_cfg_.yaw_lock_zero_stop_release_hold_ms = 2U;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.input_target_data_.mode = Chassis::Mode::kBodySpeedLockNowRotZMode;
    chassis.current_mode_flag_.is_lock_now_rot_z = true;
    chassis.target_data_.omega_z = 1.0f;
    chassis.planned_data_.omega_z = 1.0f;

    setWheelResidualSpeedMps(chassis, 0, 0.15f);
    chassis.computeModuleCommands(chassis.planned_data_);
    chassis.applyModuleCommands(true);

    const float pure_rotation_oa_rad =
        std::atan2(-chassis.planned_data_.omega_z * chassis.wheel_config_[0].pos_x_m,
                   chassis.planned_data_.omega_z * chassis.wheel_config_[0].pos_y_m);
    EXPECT_TRUE(!chassis.drive_zero_stop_active_);
    EXPECT_TRUE(!chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kRpm);
    EXPECT_NEAR(chassis.actuator_command_frame_.steer_oa_total_rad[0], pure_rotation_oa_rad, 1.0e-6f);
}

TEST_CASE("testYawLockZeroStopReleaseHoldWaitsAfterResidualEntersNearZero")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = false;
    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kHoldLast;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.enable = false;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_assist = true;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_current_mA = 1200.0f;
    chassis.runtime_strategy_cfg_.yaw_lock_zero_stop_release_hold_ms = 2U;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.input_target_data_.mode = Chassis::Mode::kBodySpeedLockNowRotZMode;
    chassis.input_target_data_.vel_x = 0.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;
    chassis.current_mode_flag_.is_lock_now_rot_z = true;
    chassis.target_data_.vel_x = 0.0f;
    chassis.target_data_.vel_y = 0.0f;
    chassis.target_data_.omega_z = 1.0f;
    chassis.planned_data_.vel_x = 0.0f;
    chassis.planned_data_.vel_y = 0.0f;
    chassis.planned_data_.omega_z = 1.0f;
    chassis.last_planned_data_.vel_x = 0.2f;
    chassis.last_planned_data_.vel_y = 0.0f;
    chassis.last_planned_data_.omega_z = 1.0f;

    setWheelResidualSpeedMps(chassis, 0, 0.15f);
    chassis.computeModuleCommands(chassis.planned_data_);
    chassis.applyModuleCommands(true);
    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);

    setWheelResidualSpeedMps(chassis, 0, chassis.getNearZeroEnterSpeedMps());
    chassis.computeModuleCommands(chassis.planned_data_);
    EXPECT_TRUE(chassis.yaw_lock_zero_stop_decel_context_active_);
    EXPECT_TRUE(chassis.yaw_lock_zero_stop_release_hold_elapsed_ms_ == 1U);
    EXPECT_NEAR(chassis.actuator_command_frame_.steer_oa_total_rad[0], 0.0f, 1.0e-6f);
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.yaw_lock_zero_stop_decel_context_active_);
    EXPECT_TRUE(chassis.yaw_lock_zero_stop_release_hold_elapsed_ms_ > 0U);
    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_NEAR(chassis.actuator_command_frame_.steer_oa_total_rad[0], 0.0f, 1.0e-6f);
}

TEST_CASE("testYawLockZeroStopReleaseHoldResetsWhenResidualRebounds")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = false;
    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kHoldLast;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.enable = false;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_assist = true;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_current_mA = 1200.0f;
    chassis.runtime_strategy_cfg_.yaw_lock_zero_stop_release_hold_ms = 20U;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.input_target_data_.mode = Chassis::Mode::kBodySpeedLockNowRotZMode;
    chassis.input_target_data_.vel_x = 0.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;
    chassis.current_mode_flag_.is_lock_now_rot_z = true;
    chassis.target_data_.vel_x = 0.0f;
    chassis.target_data_.vel_y = 0.0f;
    chassis.target_data_.omega_z = 1.0f;
    chassis.planned_data_.vel_x = 0.0f;
    chassis.planned_data_.vel_y = 0.0f;
    chassis.planned_data_.omega_z = 1.0f;
    chassis.last_planned_data_.vel_x = 0.2f;
    chassis.last_planned_data_.vel_y = 0.0f;
    chassis.last_planned_data_.omega_z = 1.0f;

    setWheelResidualSpeedMps(chassis, 0, 0.15f);
    chassis.computeModuleCommands(chassis.planned_data_);
    chassis.applyModuleCommands(true);

    setWheelResidualSpeedMps(chassis, 0, chassis.getNearZeroEnterSpeedMps());
    chassis.computeModuleCommands(chassis.planned_data_);
    chassis.applyModuleCommands(true);

    setWheelResidualSpeedMps(chassis, 0, chassis.getNearZeroExitSpeedMps() + 0.001f);
    chassis.computeModuleCommands(chassis.planned_data_);
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_NEAR(chassis.actuator_command_frame_.steer_oa_total_rad[0], 0.0f, 1.0e-6f);
}

TEST_CASE("testDriveZeroStopSettlesToZeroCurrentWhenResidualEntersNearZero")
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
    EXPECT_TRUE(drive_motors[0].getResetSpeedPidStateCallCount() == 1U);

    setWheelResidualSpeedMps(chassis, 0, chassis.getNearZeroEnterSpeedMps());
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(0.0f));
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(!chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kCurrent);
    EXPECT_NEAR(drive_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
}

TEST_CASE("testDriveZeroStopReleaseClearsPidStateBeforeReturningToRpmLoop")
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

TEST_CASE("testDriveZeroStopSettleZeroCurrentCanBeDisabled")
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveZeroStopHarness(chassis, drive_motors);
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_settle_zero_current = false;

    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(0.0f));
    setWheelResidualSpeedMps(chassis, 0, chassis.getNearZeroEnterSpeedMps());

    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_NEAR(drive_motors[0].getTargetBrake(), 1200.0f, 1.0e-6f);
}

TEST_CASE("testDriveZeroStopSettleHysteresisDoesNotToggleAroundNearZeroBand")
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

    setWheelResidualSpeedMps(chassis, 0, 0.02f);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(0.0f));
    chassis.applyModuleCommands(true);
    EXPECT_TRUE(!chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kCurrent);

    setWheelResidualSpeedMps(chassis, 0, 0.031f);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(0.0f));
    chassis.applyModuleCommands(true);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
}

TEST_CASE("testDriveZeroStopKeepsBrakeAtTargetExitBoundary")
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveZeroStopHarness(chassis, drive_motors);

    advanceDriveZeroStopCycle(chassis, 0.0f, 0.15f, 0U);
    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);

    const float exit_boundary_drive_rad_s =
        chassis.getNearZeroExitSpeedMps() / chassis.runtime_strategy_cfg_.wheel_radius_m_;
    advanceDriveZeroStopCycle(chassis, exit_boundary_drive_rad_s, 0.15f, 1U);

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);

    const float outside_exit_drive_rad_s =
        (chassis.getNearZeroExitSpeedMps() + 0.001f) / chassis.runtime_strategy_cfg_.wheel_radius_m_;
    advanceDriveZeroStopCycle(chassis, outside_exit_drive_rad_s, 0.15f, 2U);

    EXPECT_TRUE(!chassis.drive_zero_stop_active_);
    EXPECT_TRUE(!chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kRpm);
}

TEST_CASE("testDriveZeroStopReentersBrakeWhenResidualLeavesNearZeroExit")
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveZeroStopHarness(chassis, drive_motors);

    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(0.0f));
    setWheelResidualSpeedMps(chassis, 0, 0.06f);
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_TRUE(drive_motors[0].getResetSpeedPidStateCallCount() == 1U);

    setWheelResidualSpeedMps(chassis, 0, 0.015f);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(0.0f));
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_TRUE(drive_motors[0].getResetSpeedPidStateCallCount() == 1U);

    setWheelResidualSpeedMps(chassis, 0, 0.004f);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(0.0f));
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(!chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kCurrent);
    EXPECT_TRUE(drive_motors[0].getResetSpeedPidStateCallCount() == 1U);

    setWheelResidualSpeedMps(chassis, 0, 0.031f);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(0.0f));
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_TRUE(drive_motors[0].getResetSpeedPidStateCallCount() == 1U);
}

TEST_CASE("testDriveZeroStopBrakeRampBuildsUpAcrossCycles")
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveZeroStopHarness(chassis, drive_motors);
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_ramp_time_ms = 4U;

    advanceDriveZeroStopCycle(chassis, 0.0f, 0.15f, 0U);
    const float first_brake = drive_motors[0].getTargetBrake();

    advanceDriveZeroStopCycle(chassis, 0.0f, 0.15f, 1U);
    const float second_brake = drive_motors[0].getTargetBrake();

    advanceDriveZeroStopCycle(chassis, 0.0f, 0.15f, 2U);
    const float third_brake = drive_motors[0].getTargetBrake();

    advanceDriveZeroStopCycle(chassis, 0.0f, 0.15f, 4U);
    const float final_brake = drive_motors[0].getTargetBrake();

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_TRUE(first_brake > 0.0f);
    EXPECT_TRUE(first_brake < second_brake);
    EXPECT_TRUE(second_brake < third_brake);
    EXPECT_NEAR(final_brake, 1200.0f, 1.0e-6f);
}

TEST_CASE("testDriveZeroStopBrakeRampDisabledKeepsStepBrakeBehavior")
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveZeroStopHarness(chassis, drive_motors);
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_ramp_time_ms = 0U;

    advanceDriveZeroStopCycle(chassis, 0.0f, 0.15f, 0U);

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_NEAR(drive_motors[0].getTargetBrake(), 1200.0f, 1.0e-6f);
}

TEST_CASE("testDriveZeroStopBrakeRampStopsWhenResidualSettlesUnderTargetGate")
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveZeroStopHarness(chassis, drive_motors);
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_ramp_time_ms = 10U;

    advanceDriveZeroStopCycle(chassis, 0.0f, 0.15f, 0U);
    advanceDriveZeroStopCycle(chassis, 0.0f, 0.15f, 1U);
    EXPECT_TRUE(drive_motors[0].getTargetBrake() < 1200.0f);

    advanceDriveZeroStopCycle(chassis, 0.0f, 0.01f, 2U);

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(!chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kCurrent);
    EXPECT_NEAR(drive_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
}

TEST_CASE("testDriveZeroStopBrakeRampRestartsFromZeroAfterTargetExitsAndReentersBrakeGate")
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveZeroStopHarness(chassis, drive_motors);
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_ramp_time_ms = 4U;

    advanceDriveZeroStopCycle(chassis, 0.0f, 0.15f, 0U);
    advanceDriveZeroStopCycle(chassis, 0.0f, 0.15f, 2U);
    const float brake_before_release = drive_motors[0].getTargetBrake();

    advanceDriveZeroStopCycle(chassis, 1.0f, 0.15f, 3U);
    EXPECT_TRUE(!chassis.drive_zero_stop_brake_active_[0]);

    advanceDriveZeroStopCycle(chassis, 0.0f, 0.15f, 4U);
    const float brake_after_reenter = drive_motors[0].getTargetBrake();

    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(brake_before_release > brake_after_reenter);
    EXPECT_TRUE(brake_after_reenter > 0.0f);
    EXPECT_TRUE(brake_after_reenter < 1200.0f);
}

TEST_CASE("testLowSpeedDriveSuppressionBypassesWhenResidualSpeedAboveThreshold")
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

TEST_CASE("testLowSpeedDriveSuppressionReenabledWhenResidualSpeedDropsBelowThreshold")
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

    EXPECT_NEAR(gate_scales[0], 1.0f, 1.0e-6f);
    EXPECT_NEAR(gate_scales[2], 1.0f, 1.0e-6f);
}

TEST_CASE("testLowSpeedDriveSuppressionUsesGlobalWorstWheelError")
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

TEST_CASE("testGlobalMaxResidualSpeedControlsLowSpeedSuppressionForAllWheels")
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

TEST_CASE("testRefreshDebugMirrorSeparatesPlannedAndDeliveredDriveDiagnostics")
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

TEST_CASE("testHardGateFromXParkHoldsAllDriveUntilAllWheelsPassCloseAngle")
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

} // namespace chassis_semantics_test
