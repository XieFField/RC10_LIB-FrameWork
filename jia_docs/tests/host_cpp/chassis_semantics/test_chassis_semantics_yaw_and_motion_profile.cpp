#include "test_chassis_semantics_harness.h"

namespace chassis_semantics_test
{

// 覆盖 yaw PID 调试输出、lock yaw 语义、手动 S-curve/Jerk 运动规划和快速反向。
// 这些 case 主要防止调试输入、规划器历史状态和输出符号在模式切换时互相污染。
static void configureYawLockSwitchHarness(Chassis &chassis)
{
    configureYawPidTraceHarness(chassis);
    chassis.rot_z_pid_period_ = 0U;
    chassis.rot_z_pid_count_ = 0U;
    chassis.lock_now_rot_z_shift_count_ = 0U;
    chassis.lock_now_rot_z_shift_time_ms_ = 0U;
    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kLegacy;
}

static void runApiPlannerCycleForYawLockSwitch(Chassis &chassis)
{
    chassis.setModeFlag();
    chassis.resolvePlannerTargetData();
    chassis.last_planned_data_ = chassis.planned_data_;
}

static bool runApiControlCycleForYawLockSwitch(Chassis &chassis)
{
    chassis.setModeFlag();
    return runHostControlCycle(chassis);
}

static void expectLockNowTargetReanchoredToYaw(const Chassis &chassis, float expected_yaw_rad)
{
    EXPECT_NEAR(chassis.target_data_.rot_z, expected_yaw_rad, 1.0e-6f);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, expected_yaw_rad, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.target_yaw_rad, expected_yaw_rad, 1.0e-6f);
}

TEST_CASE("testJustFloatYawPidProfileDispatchEmitsFixed15ChannelPayload")
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

TEST_CASE("testLockToYawPidTracePublishesTargetErrorAndPidFireState")
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

TEST_CASE("testLockToYawThenLockNowKeepsTheEffectiveLockedYaw")
{
    Chassis chassis;
    configureYawPidTraceHarness(chassis);

    chassis.rot_z_pid_period_ = 0U;
    chassis.rot_z_pid_count_ = 0U;
    chassis.max_lock_to_rot_z_rad_s_ = 0.1f;
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

TEST_CASE("testApiLockToYawTargetIsRateLimitedAcrossPlannerCycles")
{
    Chassis chassis;
    configureYawLockSwitchHarness(chassis);
    chassis.max_lock_to_rot_z_rad_s_ = 0.1f;
    chassis.lock_yaw_pid_deadband_enter_deg_ = 0.0f;
    chassis.lock_yaw_pid_deadband_exit_deg_ = 0.0f;
    chassis.input_hwt_rot_z_ = 0.0f;

    chassis.setSpeed_LockToYaw(Chassis::Coordinate::kBody, 0.0f, 0.0f, 1.0f);
    runApiPlannerCycleForYawLockSwitch(chassis);

    const float expected_first_step = chassis.max_lock_to_rot_z_rad_s_ * Chassis::period_;
    EXPECT_NEAR(chassis.target_data_.rot_z, expected_first_step, 1.0e-6f);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, expected_first_step, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.target_yaw_rad, expected_first_step, 1.0e-6f);
    EXPECT_NEAR(chassis.rot_z_pid_.last_target, jia::radToDegF32(expected_first_step), 1.0e-6f);

    runApiPlannerCycleForYawLockSwitch(chassis);

    const float expected_second_step = 2.0f * chassis.max_lock_to_rot_z_rad_s_ * Chassis::period_;
    EXPECT_NEAR(chassis.target_data_.rot_z, expected_second_step, 1.0e-6f);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, expected_second_step, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.target_yaw_rad, expected_second_step, 1.0e-6f);
    EXPECT_NEAR(chassis.rot_z_pid_.last_target, jia::radToDegF32(expected_second_step), 1.0e-6f);
}

TEST_CASE("testDebugLockToYawTargetInjectionIsRateLimitedAcrossPlannerCycles")
{
    Chassis chassis;
    configureYawLockSwitchHarness(chassis);
    chassis.max_lock_to_rot_z_rad_s_ = 0.2f;
    chassis.lock_yaw_pid_deadband_enter_deg_ = 0.0f;
    chassis.lock_yaw_pid_deadband_exit_deg_ = 0.0f;
    chassis.input_hwt_rot_z_ = 0.0f;
    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 5U;
    chassis.debug_control_.injection.lock_rot_z = 1.0f;

    runDebugPlannerCycleForHost(chassis);

    const float expected_first_step = chassis.max_lock_to_rot_z_rad_s_ * Chassis::period_;
    EXPECT_NEAR(chassis.target_data_.rot_z, expected_first_step, 1.0e-6f);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, expected_first_step, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.target_yaw_rad, expected_first_step, 1.0e-6f);

    runDebugPlannerCycleForHost(chassis);

    const float expected_second_step = 2.0f * chassis.max_lock_to_rot_z_rad_s_ * Chassis::period_;
    EXPECT_NEAR(chassis.target_data_.rot_z, expected_second_step, 1.0e-6f);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, expected_second_step, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.target_yaw_rad, expected_second_step, 1.0e-6f);
}

TEST_CASE("testLockToYawTargetLowPassDefaultsToBypassAndCanFilterPidInput")
{
    Chassis chassis;
    configureYawLockSwitchHarness(chassis);
    chassis.max_lock_to_rot_z_rad_s_ = 1000.0f;
    chassis.lock_yaw_pid_deadband_enter_deg_ = 0.0f;
    chassis.lock_yaw_pid_deadband_exit_deg_ = 0.0f;
    chassis.input_hwt_rot_z_ = 0.0f;
    chassis.setSpeed_LockToYaw(Chassis::Coordinate::kBody, 0.0f, 0.0f, 0.01f);

    runApiPlannerCycleForYawLockSwitch(chassis);
    EXPECT_NEAR(chassis.yaw_pid_trace_.target_yaw_rad, 0.01f, 1.0e-6f);
    EXPECT_NEAR(chassis.rot_z_pid_.last_target, jia::radToDegF32(0.01f), 1.0e-6f);

    chassis.setSpeed(Chassis::Coordinate::kBody, 0.0f, 0.0f, 0.0f);
    runApiPlannerCycleForYawLockSwitch(chassis);

    chassis.lock_yaw_pid_target_lpf_alpha_ = 0.25f;
    chassis.input_hwt_rot_z_ = 0.0f;
    chassis.setSpeed_LockToYaw(Chassis::Coordinate::kBody, 0.0f, 0.0f, 0.04f);
    runApiPlannerCycleForYawLockSwitch(chassis);
    EXPECT_NEAR(chassis.yaw_pid_trace_.target_yaw_rad, 0.04f, 1.0e-6f);

    chassis.setSpeed_LockToYaw(Chassis::Coordinate::kBody, 0.0f, 0.0f, 0.08f);
    runApiPlannerCycleForYawLockSwitch(chassis);
    const float expected_filtered_target = 0.04f + 0.25f * (0.08f - 0.04f);
    EXPECT_NEAR(chassis.target_data_.rot_z, expected_filtered_target, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.target_yaw_rad, expected_filtered_target, 1.0e-6f);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, 0.08f, 1.0e-6f);
    EXPECT_NEAR(chassis.rot_z_pid_.last_target, jia::radToDegF32(expected_filtered_target), 1.0e-6f);
}

TEST_CASE("testLockYawPidDeadbandUsesEnterExitHysteresis")
{
    Chassis chassis;
    configureYawLockSwitchHarness(chassis);
    chassis.max_lock_to_rot_z_rad_s_ = 1000.0f;
    chassis.lock_yaw_pid_deadband_enter_deg_ = 1.0f;
    chassis.lock_yaw_pid_deadband_exit_deg_ = 2.0f;
    chassis.input_hwt_rot_z_ = 0.0f;

    chassis.setSpeed_LockToYaw(Chassis::Coordinate::kBody, 0.0f, 0.0f, jia::degToRadF32(0.5f));
    runApiPlannerCycleForYawLockSwitch(chassis);
    EXPECT_TRUE(chassis.lock_yaw_pid_deadband_active_);
    EXPECT_NEAR(chassis.yaw_pid_trace_.pid_compute_fired, 0.0f, 1.0e-6f);
    EXPECT_TRUE(chassis.rot_z_pid_.calc_count == 0U);

    chassis.setSpeed_LockToYaw(Chassis::Coordinate::kBody, 0.0f, 0.0f, jia::degToRadF32(1.5f));
    runApiPlannerCycleForYawLockSwitch(chassis);
    EXPECT_TRUE(chassis.lock_yaw_pid_deadband_active_);
    EXPECT_NEAR(chassis.yaw_pid_trace_.pid_compute_fired, 0.0f, 1.0e-6f);
    EXPECT_TRUE(chassis.rot_z_pid_.calc_count == 0U);

    chassis.setSpeed_LockToYaw(Chassis::Coordinate::kBody, 0.0f, 0.0f, jia::degToRadF32(2.2f));
    runApiPlannerCycleForYawLockSwitch(chassis);
    EXPECT_TRUE(!chassis.lock_yaw_pid_deadband_active_);
    EXPECT_NEAR(chassis.yaw_pid_trace_.pid_compute_fired, 1.0f, 1.0e-6f);
    EXPECT_TRUE(chassis.rot_z_pid_.calc_count == 1U);
    EXPECT_NEAR(chassis.rot_z_pid_.last_target, 2.2f, 1.0e-5f);

    chassis.lock_yaw_pid_deadband_enter_deg_ = 3.0f;
    chassis.lock_yaw_pid_deadband_exit_deg_ = 1.0f;
    chassis.setSpeed(Chassis::Coordinate::kBody, 0.0f, 0.0f, 0.0f);
    runApiPlannerCycleForYawLockSwitch(chassis);
    chassis.input_hwt_rot_z_ = 0.0f;
    chassis.setSpeed_LockToYaw(Chassis::Coordinate::kBody, 0.0f, 0.0f, jia::degToRadF32(3.5f));
    runApiPlannerCycleForYawLockSwitch(chassis);
    EXPECT_TRUE(chassis.rot_z_pid_.calc_count == 2U);
}

TEST_CASE("testApiBodyLockNowReanchorsAfterBodySpeedMode")
{
    Chassis chassis;
    configureYawLockSwitchHarness(chassis);

    chassis.input_hwt_rot_z_ = 0.20f;
    chassis.setSpeed_LockToYaw(Chassis::Coordinate::kBody, 0.0f, 0.4f, 0.20f);
    runApiPlannerCycleForYawLockSwitch(chassis);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, 0.20f, 1.0e-6f);

    chassis.input_hwt_rot_z_ = -0.55f;
    chassis.setSpeed(Chassis::Coordinate::kBody, 0.0f, 0.4f, 0.0f);
    runApiPlannerCycleForYawLockSwitch(chassis);

    chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kBody, 0.0f, 0.4f, 0.0f);
    runApiPlannerCycleForYawLockSwitch(chassis);

    expectLockNowTargetReanchoredToYaw(chassis, -0.55f);
}

TEST_CASE("testApiWorldLockNowReanchorsAfterWorldSpeedMode")
{
    Chassis chassis;
    configureYawLockSwitchHarness(chassis);

    chassis.input_hwt_rot_z_ = 0.15f;
    chassis.setSpeed_LockToYaw(Chassis::Coordinate::kWorld, 0.1f, 0.2f, 0.15f);
    runApiPlannerCycleForYawLockSwitch(chassis);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, 0.15f, 1.0e-6f);

    chassis.input_hwt_rot_z_ = 0.85f;
    chassis.setSpeed(Chassis::Coordinate::kWorld, 0.1f, 0.2f, 0.0f);
    runApiPlannerCycleForYawLockSwitch(chassis);

    chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kWorld, 0.1f, 0.2f, 0.0f);
    runApiPlannerCycleForYawLockSwitch(chassis);

    expectLockNowTargetReanchoredToYaw(chassis, 0.85f);
}

TEST_CASE("testApiManualLockNowReanchorsAfterBodySpeedMode")
{
    Chassis chassis;
    configureYawLockSwitchHarness(chassis);

    chassis.input_hwt_rot_z_ = -0.10f;
    chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kBody, 0.0f, 0.4f, 0.7f);
    runApiPlannerCycleForYawLockSwitch(chassis);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, -0.10f, 1.0e-6f);

    chassis.input_hwt_rot_z_ = 0.62f;
    chassis.setSpeed(Chassis::Coordinate::kBody, 0.0f, 0.4f, 0.0f);
    runApiPlannerCycleForYawLockSwitch(chassis);

    chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kBody, 0.0f, 0.4f, 0.0f);
    runApiPlannerCycleForYawLockSwitch(chassis);

    expectLockNowTargetReanchoredToYaw(chassis, 0.62f);
}

TEST_CASE("testDebugBodyLockNowReanchorsAfterBodySpeedMode")
{
    Chassis chassis;
    configureYawLockSwitchHarness(chassis);
    chassis.debug_control_.common.enable = true;

    chassis.input_hwt_rot_z_ = 0.20f;
    chassis.debug_control_.common.mode_raw = 5U;
    chassis.debug_control_.injection.lock_rot_z = 0.20f;
    runDebugPlannerCycleForHost(chassis);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, 0.20f, 1.0e-6f);

    chassis.input_hwt_rot_z_ = -0.35f;
    chassis.debug_control_.common.mode_raw = 1U;
    chassis.airjoy_data_.right_x = 0.0f;
    runDebugPlannerCycleForHost(chassis);

    chassis.debug_control_.common.mode_raw = 3U;
    runDebugPlannerCycleForHost(chassis);

    expectLockNowTargetReanchoredToYaw(chassis, -0.35f);
}

TEST_CASE("testDebugWorldLockNowReanchorsAfterWorldSpeedMode")
{
    Chassis chassis;
    configureYawLockSwitchHarness(chassis);
    chassis.debug_control_.common.enable = true;

    chassis.input_hwt_rot_z_ = -0.25f;
    chassis.debug_control_.common.mode_raw = 6U;
    chassis.debug_control_.injection.lock_rot_z = -0.25f;
    runDebugPlannerCycleForHost(chassis);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, -0.25f, 1.0e-6f);

    chassis.input_hwt_rot_z_ = 0.48f;
    chassis.debug_control_.common.mode_raw = 2U;
    chassis.airjoy_data_.right_x = 0.0f;
    runDebugPlannerCycleForHost(chassis);

    chassis.debug_control_.common.mode_raw = 4U;
    runDebugPlannerCycleForHost(chassis);

    expectLockNowTargetReanchoredToYaw(chassis, 0.48f);
}

TEST_CASE("testDebugBodyLockNowReanchorsAfterSteerDegAndDriveSpeedMode")
{
    Chassis chassis;
    configureYawLockSwitchHarness(chassis);
    chassis.debug_control_.common.enable = true;

    chassis.input_hwt_rot_z_ = 0.30f;
    chassis.debug_control_.common.mode_raw = 5U;
    chassis.debug_control_.injection.lock_rot_z = 0.30f;
    runDebugPlannerCycleForHost(chassis);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, 0.30f, 1.0e-6f);

    chassis.input_hwt_rot_z_ = -0.42f;
    chassis.debug_control_.common.mode_raw = 9U;
    chassis.airjoy_data_.left_x = 0.25f;
    chassis.airjoy_data_.right_x = 0.40f;
    runDebugPlannerCycleForHost(chassis);

    chassis.debug_control_.common.mode_raw = 3U;
    chassis.airjoy_data_.right_x = 0.0f;
    runDebugPlannerCycleForHost(chassis);

    expectLockNowTargetReanchoredToYaw(chassis, -0.42f);
}

TEST_CASE("testDebugBodyLockNowReanchorsAfterModuleOverrideModes")
{
    const unsigned char override_modes[] = {21U, 22U, 30U};
    for (const unsigned char override_mode : override_modes)
    {
        Chassis chassis;
        configureYawLockSwitchHarness(chassis);
        chassis.debug_control_.common.enable = true;

        chassis.input_hwt_rot_z_ = 0.18f;
        chassis.debug_control_.common.mode_raw = 5U;
        chassis.debug_control_.injection.lock_rot_z = 0.18f;
        runDebugPlannerCycleForHost(chassis);
        EXPECT_NEAR(chassis.lock_now_rot_z_target_, 0.18f, 1.0e-6f);

        chassis.input_hwt_rot_z_ = 0.72f;
        chassis.debug_control_.common.mode_raw = override_mode;
        runDebugPlannerCycleForHost(chassis);

        chassis.debug_control_.common.mode_raw = 3U;
        chassis.airjoy_data_.right_x = 0.0f;
        runDebugPlannerCycleForHost(chassis);

        expectLockNowTargetReanchoredToYaw(chassis, 0.72f);
    }
}

TEST_CASE("testApiBodyLockNowReanchorsAfterTorqueFreeMode")
{
    Chassis chassis;
    configureYawLockSwitchHarness(chassis);

    chassis.input_hwt_rot_z_ = 0.12f;
    chassis.setSpeed_LockToYaw(Chassis::Coordinate::kBody, 0.0f, 0.3f, 0.12f);
    runApiPlannerCycleForYawLockSwitch(chassis);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, 0.12f, 1.0e-6f);

    chassis.input_hwt_rot_z_ = -0.64f;
    chassis.setWheelTorqueFreeMode();
    runApiPlannerCycleForYawLockSwitch(chassis);

    chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kBody, 0.0f, 0.3f, 0.0f);
    runApiPlannerCycleForYawLockSwitch(chassis);

    expectLockNowTargetReanchoredToYaw(chassis, -0.64f);
}

TEST_CASE("testApiBodyLockNowReanchorsAfterZeroCurrentRequest")
{
    Chassis chassis;
    configureYawLockSwitchHarness(chassis);

    chassis.input_hwt_rot_z_ = -0.18f;
    chassis.setSpeed_LockToYaw(Chassis::Coordinate::kBody, 0.0f, 0.3f, -0.18f);
    runApiPlannerCycleForYawLockSwitch(chassis);
    EXPECT_NEAR(chassis.lock_now_rot_z_target_, -0.18f, 1.0e-6f);

    chassis.input_hwt_rot_z_ = 0.53f;
    chassis.setZeroCurrent();
    runApiPlannerCycleForYawLockSwitch(chassis);

    chassis.setSpeed_LockNowYaw(Chassis::Coordinate::kBody, 0.0f, 0.3f, 0.0f);
    runApiPlannerCycleForYawLockSwitch(chassis);

    expectLockNowTargetReanchoredToYaw(chassis, 0.53f);
}

TEST_CASE("testLockNowYawPidTraceDistinguishesManualShiftAndHoldStates")
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

TEST_CASE("testLockYawPidForcedOmegaWithZeroTranslationDrivesPureRotationThroughFullControlCycle")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);
    configureYawPidTraceHarness(chassis);

    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_assist = true;
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_current_mA = 1200.0f;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = false;
    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kLegacy;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg = 1.0f;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.min_scale = 0.0f;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_omega_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_angle_feedforward = false;
    chassis.runtime_strategy_cfg_.xpark_entry_delay_ms = 0U;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.enable = false;

    chassis.rot_z_pid_period_ = 0U;
    chassis.rot_z_pid_count_ = 0U;
    chassis.lock_yaw_pid_deadband_enter_deg_ = 0.0f;
    chassis.lock_yaw_pid_deadband_exit_deg_ = 0.0f;
    chassis.lock_yaw_pid_target_lpf_alpha_ = 1.0f;
    chassis.rot_z_pid_.forced_output = 0.8f;
    chassis.input_hwt_rot_z_ = 0.0f;
    chassis.input_hwt_omega_z_ = 0.0f;

    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 5U;
    chassis.debug_control_.injection.lock_rot_z = 0.5f;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = 0U;
    chassis.airjoy_data_.left_x = 0.0f;
    chassis.airjoy_data_.left_y = 0.0f;
    chassis.airjoy_data_.right_x = 0.0f;

    EXPECT_TRUE(runDebugControlCycleForHost(chassis));

    EXPECT_TRUE(chassis.rot_z_pid_.calc_count == 1U);
    EXPECT_NEAR(chassis.yaw_pid_trace_.pid_compute_fired, 1.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.pid_output_omega_rad_s, 0.8f, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.final_omega_cmd_rad_s, 0.8f, 1.0e-6f);
    EXPECT_NEAR(chassis.target_data_.vel_x, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.target_data_.vel_y, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.target_data_.omega_z, 0.8f, 1.0e-6f);
    EXPECT_NEAR(chassis.planned_data_.omega_z, 0.8f, 1.0e-6f);
    EXPECT_TRUE(chassis.computeMaxCommandWheelSpeedMps(chassis.target_data_) > chassis.getNearZeroExitSpeedMps());
    EXPECT_TRUE(!chassis.drive_zero_stop_active_);
    EXPECT_TRUE(!chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kRpm);

    float max_abs_drive_rad_s = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        const float abs_drive_rad_s = std::fabs(chassis.actuator_command_frame_.drive_omega_rad_s[i]);
        max_abs_drive_rad_s = (abs_drive_rad_s > max_abs_drive_rad_s) ? abs_drive_rad_s : max_abs_drive_rad_s;
    }
    EXPECT_TRUE(max_abs_drive_rad_s > 1.0e-6f);
    const float expected_oa0 =
        std::atan2(-chassis.planned_data_.omega_z * chassis.wheel_config_[0].pos_x_m,
                   chassis.planned_data_.omega_z * chassis.wheel_config_[0].pos_y_m);
    EXPECT_NEAR(chassis.actuator_command_frame_.steer_oa_total_rad[0], expected_oa0, 1.0e-6f);
    EXPECT_NEAR(chassis.wheel_config_[0].target_steer_motor_total_angle_rad,
                chassis.actuator_command_frame_.steer_cmd_corrected_local_total_rad[0],
                1.0e-6f);
}

TEST_CASE("testYawLockPureRotationBetweenNearZeroEnterExitKeepsRotationalSteerTarget")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);
    configureYawPidTraceHarness(chassis);
    setWheelPoseToXPark(chassis);

    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.xpark_command_threshold_cfg_.enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.xpark_command_threshold_cfg_.exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.xpark_entry_delay_ms = 0U;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = false;
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_assist = true;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_omega_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_angle_feedforward = false;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.enable = false;

    chassis.rot_z_pid_period_ = 0U;
    chassis.rot_z_pid_count_ = 0U;
    chassis.lock_yaw_pid_deadband_enter_deg_ = 0.0f;
    chassis.lock_yaw_pid_deadband_exit_deg_ = 0.0f;
    chassis.lock_yaw_pid_target_lpf_alpha_ = 1.0f;
    chassis.rot_z_pid_.forced_output = 0.08f;
    chassis.input_hwt_rot_z_ = 0.0f;
    chassis.input_hwt_omega_z_ = 0.0f;

    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 5U;
    chassis.debug_control_.injection.lock_rot_z = 0.5f;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = 0U;
    chassis.airjoy_data_.left_x = 0.0f;
    chassis.airjoy_data_.left_y = 0.0f;
    chassis.airjoy_data_.right_x = 0.0f;

    EXPECT_TRUE(runDebugControlCycleForHost(chassis));

    const float expected_oa0 =
        std::atan2(-chassis.planned_data_.omega_z * chassis.wheel_config_[0].pos_x_m,
                   chassis.planned_data_.omega_z * chassis.wheel_config_[0].pos_y_m);
    EXPECT_NEAR(chassis.yaw_pid_trace_.final_omega_cmd_rad_s, 0.08f, 1.0e-6f);
    EXPECT_NEAR(chassis.planned_data_.omega_z, 0.08f, 1.0e-6f);
    EXPECT_TRUE(chassis.computeMaxCommandWheelSpeedMps(chassis.planned_data_) > chassis.getNearZeroEnterSpeedMps());
    EXPECT_TRUE(chassis.computeMaxCommandWheelSpeedMps(chassis.planned_data_) < chassis.getNearZeroExitSpeedMps());
    EXPECT_NEAR(chassis.actuator_command_frame_.steer_oa_total_rad[0], expected_oa0, 1.0e-6f);
    EXPECT_TRUE(!chassis.drive_zero_stop_active_);
}

TEST_CASE("testYawLockPureRotationUsesTargetOmegaForSteerIntentBeforePlannedOmegaCrossesNearZero")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);
    configureYawPidTraceHarness(chassis);
    setWheelPoseToXPark(chassis);

    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.xpark_command_threshold_cfg_.enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.xpark_command_threshold_cfg_.exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.xpark_entry_delay_ms = 0U;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = false;
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_assist = true;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_omega_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_angle_feedforward = false;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.enable = false;
    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kLegacy;
    chassis.runtime_strategy_cfg_.max_alpha_z_acc_ = 0.02f;
    chassis.runtime_strategy_cfg_.max_alpha_z_dec_ = 0.02f;

    chassis.rot_z_pid_period_ = 0U;
    chassis.rot_z_pid_count_ = 0U;
    chassis.lock_yaw_pid_deadband_enter_deg_ = 0.0f;
    chassis.lock_yaw_pid_deadband_exit_deg_ = 0.0f;
    chassis.lock_yaw_pid_target_lpf_alpha_ = 1.0f;
    chassis.rot_z_pid_.forced_output = 0.8f;
    chassis.input_hwt_rot_z_ = 0.0f;
    chassis.input_hwt_omega_z_ = 0.0f;

    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 5U;
    chassis.debug_control_.injection.lock_rot_z = 0.5f;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = 0U;
    chassis.airjoy_data_.left_x = 0.0f;
    chassis.airjoy_data_.left_y = 0.0f;
    chassis.airjoy_data_.right_x = 0.0f;

    EXPECT_TRUE(runDebugControlCycleForHost(chassis));

    const float expected_oa0 =
        std::atan2(-chassis.target_data_.omega_z * chassis.wheel_config_[0].pos_x_m,
                   chassis.target_data_.omega_z * chassis.wheel_config_[0].pos_y_m);
    EXPECT_NEAR(chassis.target_data_.omega_z, 0.8f, 1.0e-6f);
    EXPECT_TRUE(chassis.computeMaxCommandWheelSpeedMps(chassis.planned_data_) < chassis.getNearZeroEnterSpeedMps());
    EXPECT_TRUE(chassis.computeMaxCommandWheelSpeedMps(chassis.target_data_) > chassis.getNearZeroExitSpeedMps());
    EXPECT_NEAR(chassis.actuator_command_frame_.steer_oa_total_rad[0], expected_oa0, 1.0e-6f);
}

TEST_CASE("testYawLockStepDeadbandEntryDeceleratesDriveContinuouslyAfterSteerAligned")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);
    configureYawPidTraceHarness(chassis);

    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.xpark_command_threshold_cfg_.enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.xpark_command_threshold_cfg_.exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.xpark_entry_delay_ms = 0U;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = false;
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_assist = true;
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_current_mA = 1200.0f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg = 1.0f;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.min_scale = 0.0f;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = true;
    chassis.runtime_strategy_cfg_.max_drive_alpha_rad_s2_ = 10.0f;
    chassis.runtime_strategy_cfg_.enable_drive_omega_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_angle_feedforward = false;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.enable = false;
    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kSCurve;
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = false;
    chassis.runtime_strategy_cfg_.manual_yaw_alpha_acc_ = 20.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_alpha_dec_ = 20.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_jerk_acc_ = 200.0f;
    chassis.runtime_strategy_cfg_.manual_yaw_jerk_dec_ = 200.0f;

    chassis.rot_z_pid_period_ = 0U;
    chassis.rot_z_pid_count_ = 0U;
    chassis.lock_yaw_pid_deadband_enter_deg_ = 1.0f;
    chassis.lock_yaw_pid_deadband_exit_deg_ = 2.0f;
    chassis.lock_yaw_pid_target_lpf_alpha_ = 1.0f;
    chassis.rot_z_pid_.forced_output = 1.2f;
    chassis.input_hwt_rot_z_ = 0.0f;
    chassis.input_hwt_omega_z_ = 0.0f;

    chassis.setSpeed_LockToYaw(Chassis::Coordinate::kBody, 0.0f, 0.0f, jia::degToRadF32(8.0f));

    EXPECT_TRUE(runApiControlCycleForYawLockSwitch(chassis));
    EXPECT_NEAR(chassis.yaw_pid_trace_.pid_compute_fired, 1.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.final_omega_cmd_rad_s, 1.2f, 1.0e-6f);
    EXPECT_TRUE(chassis.launch_hold_active_);
    EXPECT_NEAR(chassis.planned_data_.omega_z, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.actuator_command_frame_.drive_omega_rad_s[0], 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.last_drive_omega_cmd_rad_s_[0], 0.0f, 1.0e-6f);

    for (int i = 0; i < 4; ++i)
    {
        setWheelOaAngleRad(chassis, i, chassis.launch_hold_preview_cache_.selected_oa_total_rad[i]);
        steer_motors[i].setFeedbackTotalAngleDeg(jia::radToDegF32(chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad));
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
    }

    const float drive_step_rad_s = chassis.runtime_strategy_cfg_.max_drive_alpha_rad_s2_ * Chassis::period_;

    bool observed_released_drive = false;
    for (int cycle = 0; cycle < 120; ++cycle)
    {
        EXPECT_TRUE(runApiControlCycleForYawLockSwitch(chassis));
        EXPECT_NEAR(chassis.yaw_pid_trace_.final_omega_cmd_rad_s, 1.2f, 1.0e-6f);
        EXPECT_TRUE(!chassis.drive_zero_stop_active_);
        EXPECT_TRUE(!chassis.low_speed_residual_bypass_active_);

        for (int i = 0; i < 4; ++i)
        {
            EXPECT_TRUE(!chassis.drive_zero_stop_brake_active_[i]);
            const float delivered_drive = chassis.wheel_config_[i].target_drive_omega_rad_s;
            EXPECT_NEAR(chassis.last_drive_omega_cmd_rad_s_[i], delivered_drive, 1.0e-6f);
            chassis.wheel_config_[i].corrected_drive_omega_rad_s = delivered_drive;
            steer_motors[i].setFeedbackTotalAngleDeg(jia::radToDegF32(chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad));
            observed_released_drive = observed_released_drive || (std::fabs(delivered_drive) > (3.0f * drive_step_rad_s));
        }

        if (observed_released_drive)
        {
            break;
        }
    }

    EXPECT_TRUE(observed_released_drive);

    float previous_delivered_drive_rad_s[4] = {
        chassis.last_drive_omega_cmd_rad_s_[0],
        chassis.last_drive_omega_cmd_rad_s_[1],
        chassis.last_drive_omega_cmd_rad_s_[2],
        chassis.last_drive_omega_cmd_rad_s_[3],
    };

    chassis.input_hwt_rot_z_ = chassis.yaw_pid_trace_.target_yaw_rad - jia::degToRadF32(0.2f);
    EXPECT_TRUE(runApiControlCycleForYawLockSwitch(chassis));
    EXPECT_NEAR(chassis.yaw_pid_trace_.pid_compute_fired, 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.yaw_pid_trace_.final_omega_cmd_rad_s, 0.0f, 1.0e-6f);

    for (int i = 0; i < 4; ++i)
    {
        const float delivered_drive = chassis.wheel_config_[i].target_drive_omega_rad_s;
        EXPECT_TRUE(std::fabs(delivered_drive - previous_delivered_drive_rad_s[i]) <= drive_step_rad_s + 1.0e-5f);
        EXPECT_NEAR(chassis.last_drive_omega_cmd_rad_s_[i], delivered_drive, 1.0e-6f);
    }
}

TEST_CASE("testYawLockMidRotationSteerErrorJitterDoesNotTogglePlannerDriveTarget")
{
    Chassis chassis;
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg = 1.0f;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.min_scale = 0.0f;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_omega_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_angle_feedforward = false;

    Chassis::Data command{};
    command.vel_x = 0.0f;
    command.vel_y = 0.0f;
    command.omega_z = 1.2f;

    const float steady_target_oa_rad[4] = {
        std::atan2(-command.omega_z * chassis.wheel_config_[0].pos_x_m,
                   command.omega_z * chassis.wheel_config_[0].pos_y_m),
        std::atan2(-command.omega_z * chassis.wheel_config_[1].pos_x_m,
                   command.omega_z * chassis.wheel_config_[1].pos_y_m),
        std::atan2(-command.omega_z * chassis.wheel_config_[2].pos_x_m,
                   command.omega_z * chassis.wheel_config_[2].pos_y_m),
        std::atan2(-command.omega_z * chassis.wheel_config_[3].pos_x_m,
                   command.omega_z * chassis.wheel_config_[3].pos_y_m),
    };
    const float jitter_error_deg[4] = {0.8f, 1.2f, 0.8f, 1.2f};
    float min_abs_planner_drive_rad_s = 1000000.0f;
    float max_abs_planner_drive_rad_s = 0.0f;
    float min_abs_projected_drive_rad_s = 1000000.0f;

    for (int cycle = 0; cycle < 4; ++cycle)
    {
        for (int i = 0; i < 4; ++i)
        {
            const float error_sign = ((cycle + i) % 2 == 0) ? 1.0f : -1.0f;
            setWheelOaAngleRad(chassis, i, steady_target_oa_rad[i] + error_sign * jia::degToRadF32(jitter_error_deg[i]));
        }

        const Chassis::SwervePlannerOutput output =
            chassis.planSwerveModules(chassis.makeSwervePlannerInput(command));

        const float abs_projected_drive_rad_s = std::fabs(output.projected_drive_omega_rad_s[0]);
        const float abs_planner_drive_rad_s = std::fabs(output.final_drive_omega_rad_s[0]);
        min_abs_projected_drive_rad_s = (abs_projected_drive_rad_s < min_abs_projected_drive_rad_s) ? abs_projected_drive_rad_s : min_abs_projected_drive_rad_s;
        min_abs_planner_drive_rad_s = (abs_planner_drive_rad_s < min_abs_planner_drive_rad_s) ? abs_planner_drive_rad_s : min_abs_planner_drive_rad_s;
        max_abs_planner_drive_rad_s = (abs_planner_drive_rad_s > max_abs_planner_drive_rad_s) ? abs_planner_drive_rad_s : max_abs_planner_drive_rad_s;
    }

    EXPECT_TRUE(min_abs_projected_drive_rad_s > 1.0f);
    EXPECT_TRUE(max_abs_planner_drive_rad_s > 1.0f);
    EXPECT_TRUE(min_abs_planner_drive_rad_s > max_abs_planner_drive_rad_s * 0.5f);
}

TEST_CASE("testLaunchFromXParkHoldsBodyAndDriveAtZeroUntilAllWheelsAligned")
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

TEST_CASE("testNormalLaunchSCurveWaitsForSteerAlignmentBeforeAccumulatingBodyPlan")
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

TEST_CASE("testManualSCurveProfileLegacyModeKeepsCurrentAccelerationStepSemantics")
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

TEST_CASE("testManualSCurveProfileProducesSofterFirstStepAndContinuousAcceleration")
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

TEST_CASE("testManualSCurveProfileToggleResetsShapingHistory")
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

    const float expected_legacy_vel_x =
        (chassis.runtime_strategy_cfg_.max_acc_xy_acc_ * Chassis::period_ < chassis.target_data_.vel_x)
            ? (chassis.runtime_strategy_cfg_.max_acc_xy_acc_ * Chassis::period_)
            : chassis.target_data_.vel_x;
    EXPECT_NEAR(chassis.planned_data_.vel_x, expected_legacy_vel_x, 1.0e-6f);
    EXPECT_NEAR(chassis.last_drive_omega_cmd_rad_s_[0], 0.0f, 1.0e-6f);
    EXPECT_TRUE(!chassis.trans_dir_freeze_active_);
    EXPECT_TRUE(chassis.trans_dir_ref_valid_);
    EXPECT_NEAR(chassis.trans_dir_ref_rad_, 0.0f, 1.0e-6f);
}

TEST_CASE("testManualSCurveProfileStartsBrakingBeforeTargetWhenRemainingErrorIsTooSmallForCurrentAcceleration")
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

TEST_CASE("testManualSCurveProfileSmallResidualClampClearsInternalAcceleration")
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

TEST_CASE("testManualSCurveProfileManualOnlyModeFallsBackForApiSource")
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

TEST_CASE("testManualSCurveProfileManualOnlyModeUsesSCurveForDebugSource")
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

TEST_CASE("testManualSCurveProfileRapidReverseBleedsPositiveTrendBeforeBuildingNegativeTrend")
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

TEST_CASE("testDebugBodySpeedOmegaTargetFlipsSignImmediatelyWithRightStickDirection")
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

TEST_CASE("testDebugBodySpeedModeCanEnterZeroStopBrakeAndExposeGateState")
{
    Chassis chassis;
    VESC_Motor drive_motors[4];
    configureDriveZeroStopHarness(chassis, drive_motors);

    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 1U;
    chassis.debug_control_.common.observe_wheel_index = 0U;
    chassis.debug_control_.injection.omega_z_injection_mode_raw = 0U;
    chassis.airjoy_data_.left_y = 0.0f;
    chassis.airjoy_data_.left_x = 0.0f;
    chassis.airjoy_data_.right_x = 0.0f;
    drive_motors[0].setFeedbackRpm(jia::radsToRpmF32(0.15f / chassis.runtime_strategy_cfg_.wheel_radius_m_));

    EXPECT_TRUE(runDebugControlCycleForHost(chassis));

    EXPECT_TRUE(chassis.drive_zero_stop_active_);
    EXPECT_TRUE(chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kBrake);
    EXPECT_NEAR(drive_motors[0].getTargetBrake(), 1200.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.computeMaxCommandWheelSpeedMps(chassis.target_data_), 0.0f, 1.0e-6f);
}

TEST_CASE("testDebugSteerDegAndDriveSpeedModeSkipsZeroStopWhenSpeedIsNonZero")
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
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 9U;
    chassis.debug_control_.injection.steer_deg_limit = 180.0f;
    chassis.debug_control_.injection.drive_speed_m_s_limit = 1.0f;
    chassis.airjoy_data_.left_x = 0.25f;
    chassis.airjoy_data_.right_x = 0.4f;

    for (int i = 0; i < 4; ++i)
    {
        drive_motors[i].setRpmControlMode(VESC_RPM_CONTROL_NATIVE_ERPM);
    }

    EXPECT_TRUE(runDebugControlCycleForHost(chassis));

    EXPECT_TRUE(chassis.input_target_data_.mode == Chassis::Mode::kSteerAngleAndDriveSpeedMode);
    EXPECT_TRUE(!chassis.drive_zero_stop_active_);
    EXPECT_TRUE(!chassis.drive_zero_stop_brake_active_[0]);
    EXPECT_TRUE(drive_motors[0].getLastCommandKind() == VESC_Motor::CommandKind::kRpm);
    EXPECT_TRUE(std::fabs(drive_motors[0].getTargetRPM()) > 1.0e-6f);
}

TEST_CASE("testDebugBodySpeedOmegaRapidReverseUnderSCurveKeepsOldSignForFirstPlannerStep")
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

TEST_CASE("testDebugBodySpeedOmegaRapidReverseEventuallyCrossesNegativeAfterEnoughCycles")
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

TEST_CASE("testJerkProfileRapidReverseEventuallyCrossesZeroAndBuildsOppositeSign")
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

TEST_CASE("testDebugBodySpeedOmegaRapidReverseEventuallyChangesPredictedActuatorOmegaSign")
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

TEST_CASE("testDebugBodySpeedTranslationRapidReverseEventuallyChangesPredictedActuatorDirection")
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

            baseline_axis_value = test_x_axis ? predicted_vel_y : predicted_vel_x;
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

            const float current_axis_value = test_x_axis ? predicted_vel_y : predicted_vel_x;
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

} // namespace chassis_semantics_test
