#include "test_chassis_semantics_harness.h"

namespace chassis_semantics_test
{

// 覆盖 X-Park 进入门、保持门和舵向零电流保持。
// target command 决定静止意图，actual residual 只参与进入前确认；锁存后的保持/退出另按专用门限验证。
TEST_CASE("testXParkActivatesOnlyAfterActualResidualWheelSpeedHoldsForEntryDelay")
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

TEST_CASE("testXParkHoldCounterResetsImmediatelyWhenCommandWheelSpeedExitsThreshold")
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

TEST_CASE("testXParkResidualWheelSpeedAboveThresholdKeepsGateClosed")
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

TEST_CASE("testXParkCommandThresholdTreatsPointZeroFiveAsMovingDespiteLargeResidualFilter")
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

TEST_CASE("testXParkResidualThresholdStillBlocksEntryWhenCommandIsStationary")
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

    EXPECT_TRUE(planner_input.command_stationary_intent);
    EXPECT_TRUE(!chassis.xpark_gate_active_);
    EXPECT_TRUE(!planner_input.allow_xpark_pose);
    EXPECT_TRUE(planner_input.max_residual_speed_m_s > chassis.getNearZeroEnterSpeedMps());
}

TEST_CASE("testXParkUsesIndependentCommandThresholdWhileResidualMayUseLargeNearZeroFilter")
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

TEST_CASE("testXParkAllowsEntryWhenBothCommandAndResidualAreBelowTheirOwnThresholds")
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

TEST_CASE("testXParkDoesNotEnterWhenCommandExceedsDedicatedCommandThreshold")
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

TEST_CASE("testXParkKeepsLatchedGateWhenResidualRisesAfterEntry")
{
    Chassis chassis;
    configureXParkTriggerHarness(chassis);
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.10f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.15f;

    Chassis::Data command{};
    command.vel_x = 0.005f;

    for (jia::u32 i = 0; i < chassis.runtime_strategy_cfg_.xpark_entry_delay_ms; ++i)
    {
        chassis.makeSwervePlannerInput(command);
    }

    EXPECT_TRUE(chassis.xpark_gate_active_);

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 4.0f;
    }

    Chassis::SwervePlannerInput planner_input = chassis.makeSwervePlannerInput(command);

    EXPECT_TRUE(planner_input.command_stationary_intent);
    EXPECT_TRUE(chassis.xpark_gate_active_);
    EXPECT_TRUE(planner_input.allow_xpark_pose);
    EXPECT_TRUE(planner_input.max_residual_speed_m_s > chassis.getNearZeroExitSpeedMps());

    command.vel_x = 0.06f;
    planner_input = chassis.makeSwervePlannerInput(command);

    EXPECT_TRUE(!planner_input.command_stationary_intent);
    EXPECT_TRUE(!chassis.xpark_gate_active_);
    EXPECT_TRUE(!planner_input.allow_xpark_pose);
}

TEST_CASE("testXParkSteerHoldSettlingEntryResetsSpeedPidOnlyOnce")
{
    XParkSteerHoldHarness harness;
    configureXParkSteerHoldHarness(harness);
    Chassis &chassis = harness.chassis;

    const float xpark_target_oa_rad = getWheelXParkTargetOaRad(chassis, 0);
    const Chassis::ActuatorCommandFrame command_frame = makeXParkSteerCommandFrame(chassis);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), command_frame);

    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.5f));
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kSettling);
    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_settle_ms == 1U);
    EXPECT_TRUE(harness.steer_motors[0].getResetSpeedPidStateCallCount() == 1U);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kTotalAngle);

    harness.steer_motors[0].resetLastCommandObservation();
    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.4f));
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kLatchedZeroCurrent);
    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_settle_ms == 2U);
    EXPECT_TRUE(harness.steer_motors[0].getResetSpeedPidStateCallCount() == 1U);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kCurrent);
}

TEST_CASE("testXParkSteerHoldRequiresConsecutiveSettlingBeforeLatchedZeroCurrent")
{
    XParkSteerHoldHarness harness;
    configureXParkSteerHoldHarness(harness);
    Chassis &chassis = harness.chassis;

    const float xpark_target_oa_rad = getWheelXParkTargetOaRad(chassis, 0);
    const Chassis::ActuatorCommandFrame command_frame = makeXParkSteerCommandFrame(chassis);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), command_frame);

    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.5f));
    chassis.applyModuleCommands(true);
    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kSettling);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kTotalAngle);

    harness.steer_motors[0].resetLastCommandObservation();
    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.4f));
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kLatchedZeroCurrent);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kCurrent);
    EXPECT_NEAR(harness.steer_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
}

TEST_CASE("testXParkSteerHoldZeroSettleHoldStillRequiresSettleCondition")
{
    XParkSteerHoldHarness harness;
    configureXParkSteerHoldHarness(harness);
    Chassis &chassis = harness.chassis;

    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.settle_hold_ms = 0U;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.settle_angle_deg = 0.1f;

    const float xpark_target_oa_rad = getWheelXParkTargetOaRad(chassis, 0);
    const Chassis::ActuatorCommandFrame command_frame = makeXParkSteerCommandFrame(chassis);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), command_frame);

    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.5f));
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kSettling);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kTotalAngle);

    harness.steer_motors[0].resetLastCommandObservation();
    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.05f));
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kLatchedZeroCurrent);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kCurrent);
}

TEST_CASE("testXParkSteerHoldLatchedZeroCurrentRejectsBoundaryJitterWithoutCommandFlap")
{
    XParkSteerHoldHarness harness;
    configureXParkSteerHoldHarness(harness);
    Chassis &chassis = harness.chassis;

    const float xpark_target_oa_rad = getWheelXParkTargetOaRad(chassis, 0);
    const Chassis::ActuatorCommandFrame command_frame = makeXParkSteerCommandFrame(chassis);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), command_frame);

    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.5f));
    chassis.applyModuleCommands(true);
    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.4f));
    chassis.applyModuleCommands(true);

    for (int cycle = 0; cycle < 4; ++cycle)
    {
        harness.steer_motors[0].resetLastCommandObservation();
        const float jitter_deg = ((cycle % 2) == 0) ? 2.9f : 0.7f;
        setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(jitter_deg));
        chassis.applyModuleCommands(true);
        EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kLatchedZeroCurrent);
        EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kCurrent);
        EXPECT_NEAR(harness.steer_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
    }
}

TEST_CASE("testXParkSteerHoldSingleSampleExitSpikeDoesNotDropLatchedZeroCurrent")
{
    XParkSteerHoldHarness harness;
    configureXParkSteerHoldHarness(harness);
    Chassis &chassis = harness.chassis;

    const float xpark_target_oa_rad = getWheelXParkTargetOaRad(chassis, 0);
    const Chassis::ActuatorCommandFrame command_frame = makeXParkSteerCommandFrame(chassis);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), command_frame);

    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.5f));
    chassis.applyModuleCommands(true);
    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.4f));
    chassis.applyModuleCommands(true);

    harness.steer_motors[0].resetLastCommandObservation();
    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(3.2f));
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kLatchedZeroCurrent);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kCurrent);
    EXPECT_NEAR(harness.steer_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
}

TEST_CASE("testNonXParkSteerCommandDoesNotTriggerSteerHold")
{
    XParkSteerHoldHarness harness;
    configureXParkSteerHoldHarness(harness);
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
    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kInactive);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kTotalAngle);
    EXPECT_TRUE(harness.steer_motors[0].getTargetCurrent() == 0.0f);
}

TEST_CASE("testXParkSteerHoldRequiresSustainedExitBeforeReacquiringPositionControl")
{
    XParkSteerHoldHarness harness;
    configureXParkSteerHoldHarness(harness);
    Chassis &chassis = harness.chassis;

    const float xpark_target_oa_rad = getWheelXParkTargetOaRad(chassis, 0);
    const Chassis::ActuatorCommandFrame command_frame = makeXParkSteerCommandFrame(chassis);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), command_frame);

    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.5f));
    chassis.applyModuleCommands(true);
    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.4f));
    chassis.applyModuleCommands(true);

    harness.steer_motors[0].resetLastCommandObservation();
    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(4.0f));
    chassis.applyModuleCommands(true);
    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kLatchedZeroCurrent);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kCurrent);

    harness.steer_motors[0].resetLastCommandObservation();
    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(4.0f));
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kSettling);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kTotalAngle);
    EXPECT_NEAR(harness.steer_motors[0].getTargetTotalAngle(),
                jia::radToDegF32(command_frame.steer_corrected_local_total_rad[0]),
                1.0e-6f);
}

TEST_CASE("testXParkSteerHoldZeroReacquireHoldStillRequiresExitCondition")
{
    XParkSteerHoldHarness harness;
    configureXParkSteerHoldHarness(harness);
    Chassis &chassis = harness.chassis;

    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.settle_hold_ms = 0U;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.reacquire_hold_ms = 0U;

    const float xpark_target_oa_rad = getWheelXParkTargetOaRad(chassis, 0);
    const Chassis::ActuatorCommandFrame command_frame = makeXParkSteerCommandFrame(chassis);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), command_frame);

    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.05f));
    chassis.applyModuleCommands(true);
    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kLatchedZeroCurrent);

    harness.steer_motors[0].resetLastCommandObservation();
    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(2.0f));
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kLatchedZeroCurrent);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kCurrent);

    harness.steer_motors[0].resetLastCommandObservation();
    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(4.0f));
    chassis.applyModuleCommands(true);

    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kSettling);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kTotalAngle);
    EXPECT_NEAR(harness.steer_motors[0].getTargetTotalAngle(),
                jia::radToDegF32(command_frame.steer_corrected_local_total_rad[0]),
                1.0e-6f);
}

TEST_CASE("testXParkSteerHoldDoesNotOverrideRecoveringSearchOrFaultControlOwnership")
{
    XParkSteerHoldHarness harness;
    configureXParkSteerHoldHarness(harness);
    Chassis &chassis = harness.chassis;

    const float xpark_target_oa_rad = getWheelXParkTargetOaRad(chassis, 0);
    const Chassis::ActuatorCommandFrame command_frame = makeXParkSteerCommandFrame(chassis);
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), command_frame);
    setWheelOaAngleRad(chassis, 0, xpark_target_oa_rad - jia::degToRadF32(0.5f));

    chassis.wheel_config_[0].steer_fault_state = Chassis::SteerFaultState::kRecovering;
    chassis.wheel_config_[0].homing_state = Chassis::HomingState::kSearch;
    harness.steer_motors[0].resetLastCommandObservation();
    chassis.applyModuleCommands(true);
    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kInactive);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() != M3508::CommandKind::kCurrent);

    chassis.wheel_config_[0].steer_fault_state = Chassis::SteerFaultState::kLatched;
    chassis.wheel_config_[0].homing_state = Chassis::HomingState::kFault;
    harness.steer_motors[0].resetLastCommandObservation();
    chassis.applyModuleCommands(true);
    EXPECT_TRUE(chassis.wheel_config_[0].xpark_steer_hold_phase == Chassis::XParkSteerHoldPhase::kInactive);
    EXPECT_TRUE(harness.steer_motors[0].getLastCommandKind() == M3508::CommandKind::kCurrent);
}

} // namespace chassis_semantics_test
