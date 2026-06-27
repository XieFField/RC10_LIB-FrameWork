#include "test_chassis_semantics_harness.h"

namespace chassis_semantics_test
{

namespace
{

void configureSteerAngleFeedforwardPlanner(Chassis &chassis)
{
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = true;
    chassis.runtime_strategy_cfg_.max_steer_alpha_rad_s2_ = 20000.0f;
    chassis.runtime_strategy_cfg_.enable_drive_omega_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_angle_feedforward = true;
    chassis.runtime_strategy_cfg_.steer_angle_feedforward_lead_s = 0.02f;
    chassis.runtime_strategy_cfg_.steer_angle_feedforward_max_lead_rad = jia::degToRadF32(8.0f);
    chassis.runtime_strategy_cfg_.steer_angle_feedforward_settle_error_rad = jia::degToRadF32(3.0f);

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
        chassis.last_steer_rate_cmd_rad_s_[i] = 0.0f;
    }
}

void configureMotionDirectionGuardPlanner(Chassis &chassis)
{
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.steering_strategy_mode = Chassis::SteeringStrategyMode::kShortestPath;
    chassis.runtime_strategy_cfg_.enable_motion_direction_guard = true;
    chassis.runtime_strategy_cfg_.motion_direction_guard.min_actual_speed_m_s = 0.10f;
    chassis.runtime_strategy_cfg_.motion_direction_guard.target_min_speed_m_s = 0.05f;
    chassis.runtime_strategy_cfg_.motion_direction_guard.anti_motion_dot_threshold = 0.0f;
    chassis.runtime_strategy_cfg_.motion_direction_guard.prefer_forward_margin_deg = 20.0f;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = jia::degToRadF32(175.0f);
    }

    chassis.current_data_.vel_x = 0.0f;
    chassis.current_data_.vel_y = 1.0f;
}

} // namespace

// 覆盖舵轮 planner 的翻转解、倒车意图、方向 slew 和统一限幅。
// 这里的测试保护的是“轮级最优解”和“整车一致缩放”的边界，不直接检查电机下发细节。
TEST_CASE("testSteerAngleFeedforwardLeadsPlannerCommandAfterSecondOrderProfile")
{
    Chassis chassis;
    configureSteerAngleFeedforwardPlanner(chassis);

    Chassis::Data command{};
    command.vel_y = 1.0f;

    const Chassis::SwervePlannerOutput output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));
    Chassis::ActuatorCommandFrame frame{};
    chassis.buildActuatorCommandFrame(output, frame);

    const float expected_lead_rad = jia::degToRadF32(8.0f);
    EXPECT_NEAR(output.steer_cmd_oa_total_rad[0] - output.planned_oa_total_rad[0], expected_lead_rad, 1.0e-5f);
    EXPECT_NEAR(frame.steer_cmd_oa_total_rad[0], output.steer_cmd_oa_total_rad[0], 1.0e-6f);
    EXPECT_NEAR(frame.steer_oa_total_rad[0], output.planned_oa_total_rad[0], 1.0e-6f);

    float expected_projected_drive[4] = {};
    chassis.computeProjectedDriveFromPlannedSteer(chassis.makeSwervePlannerInput(command).command,
                                                  output.planned_oa_total_rad,
                                                  expected_projected_drive);
    EXPECT_NEAR(output.projected_drive_omega_rad_s[0], expected_projected_drive[0], 1.0e-6f);
}

TEST_CASE("testSteerAngleFeedforwardScalesDownInsideSettleErrorWindow")
{
    Chassis chassis;
    configureSteerAngleFeedforwardPlanner(chassis);
    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = jia::degToRadF32(88.5f);
    }

    Chassis::Data command{};
    command.vel_y = 1.0f;

    const Chassis::SwervePlannerOutput output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));

    const float expected_uncapped_lead_rad = chassis.runtime_strategy_cfg_.max_steer_alpha_rad_s2_ *
                                             Chassis::period_ *
                                             chassis.runtime_strategy_cfg_.steer_angle_feedforward_lead_s;
    const float expected_full_lead_rad = std::min(expected_uncapped_lead_rad,
                                                  chassis.runtime_strategy_cfg_.steer_angle_feedforward_max_lead_rad);
    const float expected_scaled_lead_rad = expected_full_lead_rad * 0.5f;
    EXPECT_NEAR(output.steer_cmd_oa_total_rad[0] - output.planned_oa_total_rad[0], expected_scaled_lead_rad, 1.0e-4f);
}

TEST_CASE("testSteerAngleFeedforwardDropsToZeroWhenSteerErrorIsZero")
{
    Chassis chassis;
    configureSteerAngleFeedforwardPlanner(chassis);
    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = jia::degToRadF32(90.0f);
    }

    Chassis::Data command{};
    command.vel_y = 1.0f;

    const Chassis::SwervePlannerOutput output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));

    EXPECT_NEAR(output.steer_cmd_oa_total_rad[0], output.planned_oa_total_rad[0], 1.0e-6f);
}

TEST_CASE("testSteerAngleFeedforwardDisabledForUniformSteerDriveMode")
{
    Chassis chassis;
    configureSteerAngleFeedforwardPlanner(chassis);

    Chassis::Data command{};
    Chassis::SwervePlannerInput input = chassis.makeSwervePlannerInput(command);
    input.force_uniform_steer_drive = true;
    input.uniform_steer_oa_mod_rad = jia::degToRadF32(90.0f);
    input.uniform_drive_omega_abs = 5.0f;
    input.uniform_drive_sign = 1.0f;
    for (int i = 0; i < 4; ++i)
    {
        input.wheel_speed_m_s[i] = 1.0f;
    }

    const Chassis::SwervePlannerOutput output = chassis.planSwerveModules(input);

    EXPECT_NEAR(output.steer_cmd_oa_total_rad[0], output.planned_oa_total_rad[0], 1.0e-6f);
}

TEST_CASE("testDriveOmegaPlannerLimitUsesUniformScaleAcrossAllWheels")
{
    Chassis chassis;
    configureXParkWheelGeometry(chassis);
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
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

TEST_CASE("testFlipSolutionPrefersSmallSteerDeltaAndInvertsDriveOnQuadrantCrossing")
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_motion_direction_guard = false;
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

TEST_CASE("testReverseIntentBypassesTranslationalDirectionSlewOnNearOppositeCommand")
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

TEST_CASE("testDirectionSlewCrossesPiBoundaryByShortestPath")
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

TEST_CASE("testReverseIntentDoesNotFallIntoZeroHoldOrSuppressionWhenSteerIsReachable")
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg = 25.0f;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.min_scale = 0.0f;
    chassis.runtime_strategy_cfg_.enable_motion_direction_guard = false;
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

TEST_CASE("testAlwaysForwardModeIgnoresReverseIntentOverride")
{
    Chassis chassis;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_motion_direction_guard = false;
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

TEST_CASE("testMotionDirectionGuardBlocksFlipThatPushesAgainstActualMotion")
{
    Chassis chassis;
    configureMotionDirectionGuardPlanner(chassis);

    Chassis::Data command{};
    command.vel_x = 0.0f;
    command.vel_y = -1.0f;

    const Chassis::SwervePlannerOutput output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));

    EXPECT_TRUE(!output.flipped_drive_direction[0]);
    EXPECT_TRUE(output.motion_direction_guard_active[0]);
    EXPECT_TRUE(output.projected_drive_omega_rad_s[0] > 0.0f);
}

TEST_CASE("testMotionDirectionGuardIgnoresActualSpeedBelowThreshold")
{
    Chassis chassis;
    configureMotionDirectionGuardPlanner(chassis);
    chassis.current_data_.vel_x = 0.09f;

    Chassis::Data command{};
    command.vel_x = 0.0f;
    command.vel_y = -1.0f;

    const Chassis::SwervePlannerOutput output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));

    EXPECT_TRUE(output.flipped_drive_direction[0]);
    EXPECT_TRUE(!output.motion_direction_guard_active[0]);
    EXPECT_TRUE(output.projected_drive_omega_rad_s[0] < 0.0f);
}

TEST_CASE("testMotionDirectionGuardIgnoresTargetSpeedBelowThreshold")
{
    Chassis chassis;
    configureMotionDirectionGuardPlanner(chassis);

    Chassis::Data command{};
    command.vel_x = 0.0f;
    command.vel_y = -0.04f;

    const Chassis::SwervePlannerOutput output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));

    EXPECT_TRUE(output.flipped_drive_direction[0]);
    EXPECT_TRUE(!output.motion_direction_guard_active[0]);
}

TEST_CASE("testMotionDirectionGuardUsesAntiMotionDotThreshold")
{
    Chassis chassis;
    configureMotionDirectionGuardPlanner(chassis);
    chassis.runtime_strategy_cfg_.motion_direction_guard.anti_motion_dot_threshold = -0.20f;

    Chassis::Data command{};
    command.vel_x = 0.0f;
    command.vel_y = -1.0f;

    const Chassis::SwervePlannerOutput output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));

    EXPECT_TRUE(output.flipped_drive_direction[0]);
    EXPECT_TRUE(!output.motion_direction_guard_active[0]);
}

TEST_CASE("testMotionDirectionGuardRespectsPreferForwardMargin")
{
    Chassis chassis;
    configureMotionDirectionGuardPlanner(chassis);
    chassis.runtime_strategy_cfg_.motion_direction_guard.prefer_forward_margin_deg = 2.0f;

    Chassis::Data command{};
    command.vel_x = 0.0f;
    command.vel_y = -1.0f;

    const Chassis::SwervePlannerOutput output =
        chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));

    EXPECT_TRUE(output.flipped_drive_direction[0]);
    EXPECT_TRUE(!output.motion_direction_guard_active[0]);
}

TEST_CASE("testDriveAlphaDeliveryLimitUsesUniformScaleAcrossAllWheels")
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

TEST_CASE("testDriveAlphaDeliveryLimitUsesWorstWheelScaleWhenLastValuesDiffer")
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

TEST_CASE("testDriveAlphaDeliveryLimitUsesUniformScaleWhileNotHomed")
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

TEST_CASE("testDriveDeliveryLimitAlphaThenOmegaStillKeepsSharedProgress")
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

} // namespace chassis_semantics_test
