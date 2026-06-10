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
