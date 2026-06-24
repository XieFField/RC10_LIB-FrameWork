#include "test_chassis_semantics_harness.h"

namespace chassis_semantics_test
{

// 覆盖光电门、舵轮冻结故障、故障恢复和迟到上电 homing。
// 这些 case 会直接操纵反馈电流、角度和光电门状态，用来保护故障锁存/恢复的时序边界。
TEST_CASE("testStationaryPhotogateTogglesDoNotSelfLockNormalReadyState")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.target_data_.vel_x = 0.0f;
    chassis.target_data_.vel_y = 0.0f;
    chassis.target_data_.omega_z = 0.0f;

    for (int cycle = 0; cycle < 6; ++cycle)
    {
        setPhotogateStateForWheel(0, (cycle % 2) != 0);
        const bool all_homed = runHostControlCycle(chassis);
        EXPECT_TRUE(all_homed);
        EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kReady);
        EXPECT_NEAR(drive_motors[0].getTargetRPM(), 0.0f, 1.0e-6f);
        EXPECT_NEAR(steer_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
    }
}

TEST_CASE("testHomingSearchRpmDefaultsToCompileTimeMacro")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(chassis.wheel_config_[i].homing_search_rpm, JIA_CHASSIS_HOMING_SEARCH_RPM, 1.0e-6f);
    }

    chassis.wheel_config_[0].homing_state = Chassis::HomingState::kSearch;
    chassis.wheel_config_[0].homing_zero_valid = false;
    chassis.wheel_config_[0].homing_last_sensor_active = false;
    steer_motors[0].setFeedbackCurrent(1200.0f);

    runHostControlCycle(chassis);

    EXPECT_NEAR(steer_motors[0].getTargetRPM(), JIA_CHASSIS_HOMING_SEARCH_RPM, 1.0e-6f);
}

TEST_CASE("testFirstBootHomingDelayKeepsInitialStartIdleThenReleasesSearchOnce")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.startHoming();

#if JIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_ENABLE && (JIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_MS > 0U)
    EXPECT_TRUE(chassis.first_boot_homing_delay_.pending);
    EXPECT_TRUE(chassis.first_boot_homing_delay_.active);
    EXPECT_TRUE(chassis.first_boot_homing_delay_.elapsed_ms == 0U);

    for (unsigned int cycle = 0; cycle + 1U < JIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_MS; ++cycle)
    {
        const bool all_homed = runHostControlCycle(chassis);
        EXPECT_TRUE(!all_homed);
        for (int i = 0; i < 4; ++i)
        {
            EXPECT_TRUE(chassis.wheel_config_[i].homing_state == Chassis::HomingState::kIdle);
            EXPECT_NEAR(steer_motors[i].getTargetRPM(), 0.0f, 1.0e-6f);
        }
    }

    bool released = false;
    for (int release_cycle = 0; release_cycle < 2; ++release_cycle)
    {
        const bool all_homed_after_release = runHostControlCycle(chassis);
        EXPECT_TRUE(!all_homed_after_release);
        EXPECT_TRUE(!chassis.first_boot_homing_delay_.pending);
        EXPECT_TRUE(!chassis.first_boot_homing_delay_.active);
        EXPECT_TRUE(chassis.first_boot_homing_delay_.elapsed_ms >= JIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_MS);

        bool all_search = true;
        for (int i = 0; i < 4; ++i)
        {
            if (chassis.wheel_config_[i].homing_state != Chassis::HomingState::kSearch)
            {
                all_search = false;
                break;
            }
        }
        if (all_search)
        {
            released = true;
            for (int i = 0; i < 4; ++i)
            {
                EXPECT_NEAR(steer_motors[i].getTargetRPM(), chassis.wheel_config_[i].homing_search_rpm, 1.0e-6f);
            }
            break;
        }
    }
    EXPECT_TRUE(released);

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].homing_state = Chassis::HomingState::kReady;
        chassis.wheel_config_[i].homing_zero_valid = true;
        chassis.wheel_config_[i].homing_hold_corrected_local_total_rad =
            chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad;
    }

    chassis.startHoming();
    const bool all_homed_second_start = runHostControlCycle(chassis);
    EXPECT_TRUE(!all_homed_second_start);
    EXPECT_TRUE(!chassis.first_boot_homing_delay_.pending);
    EXPECT_TRUE(!chassis.first_boot_homing_delay_.active);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_TRUE(chassis.wheel_config_[i].homing_state == Chassis::HomingState::kSearch);
        EXPECT_NEAR(steer_motors[i].getTargetRPM(), chassis.wheel_config_[i].homing_search_rpm, 1.0e-6f);
    }
#else
    const bool all_homed = runHostControlCycle(chassis);
    EXPECT_TRUE(!all_homed);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_TRUE(chassis.wheel_config_[i].homing_state == Chassis::HomingState::kSearch);
        EXPECT_NEAR(steer_motors[i].getTargetRPM(), chassis.wheel_config_[i].homing_search_rpm, 1.0e-6f);
    }
#endif
}

TEST_CASE("testSteerFaultRecoveryRehomeBypassesFirstBootHomingDelay")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.startHoming();
    chassis.wheel_config_[0].homing_state = Chassis::HomingState::kFault;
    chassis.wheel_config_[0].steer_fault_state = Chassis::SteerFaultState::kRecovering;
    chassis.wheel_config_[0].steer_fault_rehome_request = true;
    chassis.wheel_config_[0].homing_zero_valid = true;

    const bool all_homed = runHostControlCycle(chassis);
    EXPECT_TRUE(!all_homed);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);
    EXPECT_TRUE(!chassis.wheel_config_[0].steer_fault_rehome_request);
    EXPECT_TRUE(!chassis.wheel_config_[0].homing_zero_valid);
    EXPECT_NEAR(steer_motors[0].getTargetRPM(), chassis.wheel_config_[0].homing_search_rpm, 1.0e-6f);

#if JIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_ENABLE && (JIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_MS > 0U)
    EXPECT_TRUE(chassis.first_boot_homing_delay_.pending);
    EXPECT_TRUE(chassis.first_boot_homing_delay_.active);
    EXPECT_TRUE(chassis.first_boot_homing_delay_.elapsed_ms > 0U);
    for (int i = 1; i < 4; ++i)
    {
        EXPECT_TRUE(chassis.wheel_config_[i].homing_state == Chassis::HomingState::kIdle);
        EXPECT_NEAR(steer_motors[i].getTargetRPM(), 0.0f, 1.0e-6f);
    }
#endif
}

TEST_CASE("testReadyStationaryWheelsCanStillEnterXParkWithoutTriggeringSteerFault")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.xpark_entry_delay_ms = 5U;
    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kXPark;
    chassis.input_target_data_.vel_x = 0.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
        steer_motors[i].setFeedbackCurrent(120.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
    }

    bool xpark_armed = false;
    for (int cycle = 0; cycle < 12; ++cycle)
    {
        const bool all_homed = runHostControlCycle(chassis);
        EXPECT_TRUE(all_homed);
        for (int i = 0; i < 4; ++i)
        {
            EXPECT_TRUE(chassis.wheel_config_[i].homing_state == Chassis::HomingState::kReady);
            EXPECT_TRUE(!chassis.debug_mirror_.steer_fault_active[i]);
        }
        if (chassis.xpark_gate_active_)
        {
            xpark_armed = true;
            break;
        }
    }

    EXPECT_TRUE(xpark_armed);
    EXPECT_TRUE(chassis.xpark_gate_active_);
    EXPECT_TRUE(std::fabs(chassis.debug_mirror_.target_oa_deg[0]) > 1.0e-3f);
    EXPECT_NEAR(drive_motors[0].getTargetRPM(), 0.0f, 1.0e-6f);
}

TEST_CASE("testSingleWheelSteerFreezeFaultStopsVehicleAndFreezesFaultedWheelPath")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 2) ? 6000.0f : 200.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
        drive_motors[i].setTargetCurrent(777.0f);
    }

    bool saw_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        const bool all_homed = runHostControlCycle(chassis);
        if (chassis.wheel_config_[2].homing_state == Chassis::HomingState::kFault)
        {
            saw_fault = true;
            EXPECT_TRUE(!all_homed);
            EXPECT_NEAR(chassis.current_data_.vel_x, 0.0f, 1.0e-6f);
            EXPECT_NEAR(chassis.current_data_.vel_y, 0.0f, 1.0e-6f);
            EXPECT_NEAR(chassis.current_data_.omega_z, 0.0f, 1.0e-6f);
            for (int i = 0; i < 4; ++i)
            {
                EXPECT_NEAR(chassis.wheel_config_[i].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
                EXPECT_NEAR(drive_motors[i].getTargetCurrent(), 0.0f, 1.0e-6f);
            }
            EXPECT_NEAR(steer_motors[2].getTargetCurrent(), 0.0f, 1.0e-6f);
            EXPECT_NEAR(chassis.wheel_config_[2].target_drive_omega_rad_s, 0.0f, 1.0e-6f);
            EXPECT_NEAR(chassis.wheel_config_[2].target_steer_motor_total_angle_rad, 0.0f, 1.0e-6f);
            break;
        }
    }

    EXPECT_TRUE(saw_fault);
}

TEST_CASE("testSingleWheelSteerFreezeFaultDoesNotRequireHugeCurrentMagnitude")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 0) ? 600.0f : 20.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
    }

    bool saw_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        const bool all_homed = runHostControlCycle(chassis);
        if (chassis.wheel_config_[0].homing_state == Chassis::HomingState::kFault)
        {
            saw_fault = true;
            EXPECT_TRUE(!all_homed);
            EXPECT_TRUE(chassis.debug_mirror_.steer_fault_active[0]);
            EXPECT_NEAR(chassis.debug_mirror_.steer_feedback_current_mA[0], 600.0f, 1.0e-6f);
            EXPECT_TRUE(chassis.debug_mirror_.steer_feedback_current_freeze_ms[0] >= 5.0f);
            EXPECT_NEAR(drive_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
            break;
        }
    }

    EXPECT_TRUE(saw_fault);
}

TEST_CASE("testSteerFaultThresholdsAreDebugTunableAndMirrorPublishesDecisionInputs")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = true;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.active_current_min_mA = 700.0f;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.freeze_duration_ms = 5U;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 0) ? 600.0f : 20.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
    }

    for (int cycle = 0; cycle < 8; ++cycle)
    {
        runHostControlCycle(chassis);
    }

    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kReady);
    EXPECT_TRUE(!chassis.debug_mirror_.steer_fault_active[0]);
    EXPECT_TRUE(chassis.debug_mirror_.steer_fault_control_intent[0]);
    EXPECT_TRUE(!chassis.debug_mirror_.steer_fault_freeze_candidate[0]);
    EXPECT_NEAR(chassis.debug_mirror_.steer_feedback_current_mA[0], 600.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.debug_mirror_.steer_feedback_current_delta_mA[0], 0.0f, 1.0e-6f);
    EXPECT_NEAR(chassis.debug_mirror_.steer_feedback_angle_delta_rad[0], 0.0f, 1.0e-6f);
    EXPECT_TRUE(chassis.debug_mirror_.steer_feedback_current_freeze_ms[0] < 5.0f);

    chassis.runtime_strategy_cfg_.steer_fault_cfg.active_current_min_mA = 500.0f;

    bool saw_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[0].homing_state == Chassis::HomingState::kFault)
        {
            saw_fault = true;
            break;
        }
    }

    EXPECT_TRUE(saw_fault);
    EXPECT_TRUE(chassis.debug_mirror_.steer_fault_active[0]);
    EXPECT_TRUE(chassis.debug_mirror_.steer_fault_freeze_candidate[0]);
    EXPECT_TRUE(chassis.debug_mirror_.steer_feedback_current_freeze_ms[0] >= 5.0f);
}

TEST_CASE("testFaultedWheelOnlyRehomesAfterCurrentTogglesAndMotionResumesAfterRecoveryCompletes")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 1) ? 5000.0f : 100.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
        drive_motors[i].setTargetCurrent(777.0f);
    }

    bool latched_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[1].homing_state == Chassis::HomingState::kFault)
        {
            latched_fault = true;
            break;
        }
    }
    EXPECT_TRUE(latched_fault);

    for (int cycle = 0; cycle < 3; ++cycle)
    {
        runHostControlCycle(chassis);
        EXPECT_TRUE(chassis.wheel_config_[1].homing_state == Chassis::HomingState::kFault);
        EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kReady);
        EXPECT_TRUE(chassis.wheel_config_[2].homing_state == Chassis::HomingState::kReady);
        EXPECT_TRUE(chassis.wheel_config_[3].homing_state == Chassis::HomingState::kReady);
        for (int i = 0; i < 4; ++i)
        {
            EXPECT_NEAR(drive_motors[i].getTargetCurrent(), 0.0f, 1.0e-6f);
        }
    }

    steer_motors[1].setFeedbackCurrent(-5000.0f);
    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[1].homing_state == Chassis::HomingState::kSearch);
    EXPECT_NEAR(steer_motors[1].getTargetRPM(), chassis.wheel_config_[1].homing_search_rpm, 1.0e-6f);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kReady);
    EXPECT_TRUE(chassis.wheel_config_[2].homing_state == Chassis::HomingState::kReady);
    EXPECT_TRUE(chassis.wheel_config_[3].homing_state == Chassis::HomingState::kReady);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(drive_motors[i].getTargetCurrent(), 0.0f, 1.0e-6f);
    }

    for (int cycle = 0; cycle < 2; ++cycle)
    {
        runHostControlCycle(chassis);
        EXPECT_TRUE(chassis.wheel_config_[1].homing_state == Chassis::HomingState::kSearch);
    }

    finishWheelHomingByEdgeAndAlign(chassis, 1, steer_motors);

    EXPECT_TRUE(chassis.wheel_config_[1].homing_state == Chassis::HomingState::kReady);
    EXPECT_TRUE(runHostControlCycle(chassis));
    EXPECT_TRUE(std::fabs(drive_motors[0].getTargetRPM()) > 1.0e-3f);
    EXPECT_TRUE(std::fabs(drive_motors[1].getTargetRPM()) > 1.0e-3f);
}

TEST_CASE("testLatchedFaultedWheelKeepsPureZeroCurrentCommandWithoutOverwritingControlMode")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 2) ? 5000.0f : 100.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
    }

    bool latched_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[2].homing_state == Chassis::HomingState::kFault)
        {
            latched_fault = true;
            break;
        }
    }
    EXPECT_TRUE(latched_fault);

    steer_motors[2].setTargetRPM(321.0f);
    steer_motors[2].setTargetTotalAngle(654.0f);

    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[2].homing_state == Chassis::HomingState::kFault);
    EXPECT_NEAR(steer_motors[2].getTargetCurrent(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(steer_motors[2].getTargetRPM(), 321.0f, 1.0e-6f);
    EXPECT_NEAR(steer_motors[2].getTargetTotalAngle(), 654.0f, 1.0e-6f);
}

TEST_CASE("testRecoveryImmediatelyReopensSteerSearchAfterFaultLatchSidePidReset")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 0) ? 5000.0f : 100.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
    }

    bool latched_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[0].homing_state == Chassis::HomingState::kFault)
        {
            latched_fault = true;
            break;
        }
    }
    EXPECT_TRUE(latched_fault);

    steer_motors[0].setFeedbackCurrent(-5000.0f);
    runHostControlCycle(chassis);

    EXPECT_TRUE(chassis.wheel_config_[0].steer_fault_state == Chassis::SteerFaultState::kRecovering);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);
    EXPECT_NEAR(steer_motors[0].getTargetRPM(), chassis.wheel_config_[0].homing_search_rpm, 1.0e-6f);
}

TEST_CASE("testWheel2RecoveryReissuesHomingSearchRpmAfterFirstCommandDrop")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 1) ? 5000.0f : 100.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
    }

    bool latched_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[1].homing_state == Chassis::HomingState::kFault)
        {
            latched_fault = true;
            break;
        }
    }
    EXPECT_TRUE(latched_fault);

    steer_motors[1].resetRpmCommandObservation();
    steer_motors[1].setDropFirstRpmCommand(true);
    steer_motors[1].setFeedbackCurrent(-5000.0f);

    runHostControlCycle(chassis);

    EXPECT_TRUE(chassis.wheel_config_[1].homing_state == Chassis::HomingState::kSearch);
    EXPECT_TRUE(chassis.wheel_config_[1].steer_fault_state == Chassis::SteerFaultState::kRecovering);
    EXPECT_TRUE(steer_motors[1].getRpmCommandCallCount() == 2U);
    EXPECT_NEAR(steer_motors[1].getAcceptedTargetRPM(), chassis.wheel_config_[1].homing_search_rpm, 1.0e-6f);

    runHostControlCycle(chassis);

    EXPECT_TRUE(chassis.wheel_config_[1].homing_state == Chassis::HomingState::kSearch);
    EXPECT_TRUE(chassis.wheel_config_[1].steer_fault_state == Chassis::SteerFaultState::kRecovering);
    EXPECT_TRUE(steer_motors[1].getRpmCommandCallCount() == 3U);
    EXPECT_NEAR(steer_motors[1].getAcceptedTargetRPM(), chassis.wheel_config_[1].homing_search_rpm, 1.0e-6f);

    finishWheelHomingByEdgeAndAlign(chassis, 1, steer_motors);

    EXPECT_TRUE(chassis.wheel_config_[1].homing_state == Chassis::HomingState::kReady);
    EXPECT_TRUE(chassis.wheel_config_[1].steer_fault_state == Chassis::SteerFaultState::kNone);
}

TEST_CASE("testXParkStaticReconnectRehomesWithoutNewVelocityCommand")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.xpark_entry_delay_ms = 3U;
    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kXPark;
    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
        steer_motors[i].setFeedbackCurrent((i == 1) ? 5200.0f : 120.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
    }

    bool latched_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[1].homing_state == Chassis::HomingState::kFault)
        {
            latched_fault = true;
            break;
        }
    }

    EXPECT_TRUE(latched_fault);
    EXPECT_TRUE(chassis.wheel_config_[1].steer_fault_state == Chassis::SteerFaultState::kLatched);

    chassis.input_target_data_.vel_x = 0.0f;
    for (int cycle = 0; cycle < 6; ++cycle)
    {
        runHostControlCycle(chassis);
    }

    EXPECT_TRUE(chassis.xpark_gate_active_);

    steer_motors[1].setFeedbackCurrent(-5200.0f);
    runHostControlCycle(chassis);

    EXPECT_TRUE(chassis.xpark_gate_active_);
    EXPECT_TRUE(chassis.wheel_config_[1].steer_fault_state == Chassis::SteerFaultState::kRecovering);
    EXPECT_TRUE(chassis.wheel_config_[1].homing_state == Chassis::HomingState::kSearch);
    EXPECT_NEAR(steer_motors[1].getTargetRPM(), chassis.wheel_config_[1].homing_search_rpm, 1.0e-6f);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(drive_motors[i].getTargetCurrent(), 0.0f, 1.0e-6f);
    }
}

TEST_CASE("testIdleReadyWheelFreezeCanLatchFaultWithoutMotionIntent")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 0.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;
    chassis.current_mode_flag_.is_wheel_torque_free = false;
    chassis.input_target_data_.zero_current_all = false;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 0) ? 5600.0f : 120.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
        chassis.wheel_config_[i].homing_state = Chassis::HomingState::kReady;
        chassis.wheel_config_[i].homing_zero_valid = true;
    }

    bool latched_fault = false;
    for (int cycle = 0; cycle < 10; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[0].homing_state == Chassis::HomingState::kFault)
        {
            latched_fault = true;
            break;
        }
    }

    EXPECT_TRUE(latched_fault);
    EXPECT_TRUE(chassis.wheel_config_[0].steer_fault_state == Chassis::SteerFaultState::kLatched);
    EXPECT_TRUE(chassis.debug_mirror_.steer_fault_freeze_candidate[0]);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(drive_motors[i].getTargetCurrent(), 0.0f, 1.0e-6f);
    }
}

TEST_CASE("testXParkReadyWheelFreezeCanLatchFaultWithoutMotionIntent")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.xpark_entry_delay_ms = 2U;
    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kXPark;
    chassis.input_target_data_.vel_x = 0.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 1) ? 5700.0f : 120.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
    }

    bool xpark_armed = false;
    bool latched_fault = false;
    for (int cycle = 0; cycle < 12; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.xpark_gate_active_)
        {
            xpark_armed = true;
        }
        if (chassis.wheel_config_[1].homing_state == Chassis::HomingState::kFault)
        {
            latched_fault = true;
            break;
        }
    }

    EXPECT_TRUE(xpark_armed);
    EXPECT_TRUE(latched_fault);
    EXPECT_TRUE(chassis.wheel_config_[1].steer_fault_state == Chassis::SteerFaultState::kLatched);
    EXPECT_TRUE(chassis.debug_mirror_.steer_fault_xpark_stationary_hold[1]);
    EXPECT_TRUE(chassis.debug_mirror_.steer_fault_freeze_candidate[1]);
}

TEST_CASE("testReconnectRehomeRequestSurvivesZeroIntentAndStationaryHold")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.xpark_entry_delay_ms = 2U;
    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kXPark;
    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
        steer_motors[i].setFeedbackCurrent((i == 0) ? 5400.0f : 120.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
    }

    bool latched_fault = false;
    for (int cycle = 0; cycle < 10; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[0].homing_state == Chassis::HomingState::kFault)
        {
            latched_fault = true;
            break;
        }
    }

    EXPECT_TRUE(latched_fault);

    chassis.input_target_data_.vel_x = 0.0f;
    for (int cycle = 0; cycle < 4; ++cycle)
    {
        runHostControlCycle(chassis);
    }

    EXPECT_TRUE(chassis.xpark_gate_active_);

    steer_motors[0].setFeedbackCurrent(-5400.0f);
    runHostControlCycle(chassis);

    EXPECT_TRUE(chassis.xpark_gate_active_);
    EXPECT_TRUE(!chassis.wheel_config_[0].steer_fault_rehome_request);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);
    EXPECT_TRUE(chassis.wheel_config_[0].steer_fault_state == Chassis::SteerFaultState::kRecovering);
    EXPECT_TRUE(chassis.debug_mirror_.steer_fault_xpark_stationary_hold[0]);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(drive_motors[i].getTargetCurrent(), 0.0f, 1.0e-6f);
    }
}

TEST_CASE("testXParkLatchedFaultDoesNotRecoverWhenCurrentToggleBelowThreshold")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.xpark_entry_delay_ms = 2U;
    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kXPark;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.recovery_current_delta_mA = 7000.0f;
    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 2) ? 5600.0f : 120.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
    }

    bool latched_fault = false;
    for (int cycle = 0; cycle < 10; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[2].homing_state == Chassis::HomingState::kFault)
        {
            latched_fault = true;
            break;
        }
    }
    EXPECT_TRUE(latched_fault);

    chassis.input_target_data_.vel_x = 0.0f;
    for (int cycle = 0; cycle < 4; ++cycle)
    {
        runHostControlCycle(chassis);
    }

    EXPECT_TRUE(chassis.xpark_gate_active_);

    steer_motors[2].setFeedbackCurrent(-100.0f);
    runHostControlCycle(chassis);

    EXPECT_TRUE(chassis.wheel_config_[2].steer_fault_state == Chassis::SteerFaultState::kLatched);
    EXPECT_TRUE(chassis.wheel_config_[2].homing_state == Chassis::HomingState::kFault);
    EXPECT_TRUE(!chassis.wheel_config_[2].steer_fault_rehome_request);
    EXPECT_NEAR(steer_motors[2].getTargetRPM(), 0.0f, 1.0e-6f);
}

TEST_CASE("testXParkRecoveringWheelKeepsDriveBlockedUntilReady")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.xpark_entry_delay_ms = 2U;
    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kXPark;
    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 3) ? 5300.0f : 120.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
        drive_motors[i].setTargetCurrent(888.0f);
    }

    bool latched_fault = false;
    for (int cycle = 0; cycle < 10; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[3].homing_state == Chassis::HomingState::kFault)
        {
            latched_fault = true;
            break;
        }
    }
    EXPECT_TRUE(latched_fault);

    chassis.input_target_data_.vel_x = 0.0f;
    for (int cycle = 0; cycle < 4; ++cycle)
    {
        runHostControlCycle(chassis);
    }

    steer_motors[3].setFeedbackCurrent(-5300.0f);
    runHostControlCycle(chassis);

    EXPECT_TRUE(chassis.wheel_config_[3].steer_fault_state == Chassis::SteerFaultState::kRecovering);
    EXPECT_TRUE(chassis.wheel_config_[3].homing_state == Chassis::HomingState::kSearch);
    EXPECT_TRUE(chassis.debug_mirror_.steer_fault_any_active);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(drive_motors[i].getTargetCurrent(), 0.0f, 1.0e-6f);
    }
}

TEST_CASE("testIgnoreDuringXParkHoldDoesNotBlockRecoveryForAlreadyLatchedFault")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.xpark_entry_delay_ms = 2U;
    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kXPark;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.ignore_during_xpark_hold = true;
    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 0) ? 5100.0f : 120.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
    }

    bool latched_fault = false;
    for (int cycle = 0; cycle < 10; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[0].homing_state == Chassis::HomingState::kFault)
        {
            latched_fault = true;
            break;
        }
    }
    EXPECT_TRUE(latched_fault);

    chassis.input_target_data_.vel_x = 0.0f;
    for (int cycle = 0; cycle < 4; ++cycle)
    {
        runHostControlCycle(chassis);
    }

    EXPECT_TRUE(chassis.xpark_gate_active_);
    EXPECT_TRUE(chassis.debug_mirror_.steer_fault_xpark_stationary_hold[0]);

    steer_motors[0].setFeedbackCurrent(-5100.0f);
    runHostControlCycle(chassis);

    EXPECT_TRUE(chassis.wheel_config_[0].steer_fault_state == Chassis::SteerFaultState::kRecovering);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);
    EXPECT_TRUE(chassis.debug_mirror_.steer_fault_xpark_stationary_hold[0]);
}

TEST_CASE("testHomingRequiresThreeConsistentEdgesAndHoldsConfirmedAverageAngle")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.wheel_config_[0].homing_falling_edge_mech_rad = jia::degToRadF32(150.0f);
    chassis.wheel_config_[0].homing_rising_edge_mech_rad = jia::degToRadF32(-30.0f);
    chassis.wheel_config_[0].homing_zero_offset_rad = jia::degToRadF32(-30.0f);
    chassis.wheel_config_[0].homing_runtime_zero_offset_rad = chassis.wheel_config_[0].homing_zero_offset_rad;
    chassis.wheel_config_[0].homing_state = Chassis::HomingState::kSearch;
    chassis.wheel_config_[0].homing_zero_valid = false;
    chassis.wheel_config_[0].homing_last_sensor_active = false;
    chassis.wheel_config_[1].homing_state = Chassis::HomingState::kSearch;
    chassis.wheel_config_[1].homing_zero_valid = false;
    steer_motors[0].setFeedbackCurrent(1200.0f);

    setPhotogateStateForWheel(0, false);
    steer_motors[0].setFeedbackTotalAngleDeg(10.0f);
    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);

    steer_motors[0].setFeedbackTotalAngleDeg(10.0f);
    setPhotogateStateForWheel(0, true);
    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);
    EXPECT_TRUE(!chassis.wheel_config_[0].homing_zero_valid);

    steer_motors[0].setFeedbackTotalAngleDeg(190.0f);
    setPhotogateStateForWheel(0, false);
    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);
    EXPECT_TRUE(!chassis.wheel_config_[0].homing_zero_valid);

    steer_motors[0].setFeedbackTotalAngleDeg(370.0f);
    setPhotogateStateForWheel(0, true);
    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kEdgeDetected);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_zero_valid);

    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kOffsetApply);

    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kContinuousAngleReady);

    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kReady);

    const float expected_average_offset_rad =
        (jia::degToRadF32(-70.0f) + jia::degToRadF32(-70.0f) + jia::degToRadF32(-70.0f)) / 3.0f;
    const float expected_hold_rad = jia::degToRadF32(370.0f) + expected_average_offset_rad;

    EXPECT_NEAR(chassis.wheel_config_[0].homing_runtime_zero_offset_rad, expected_average_offset_rad, 1.0e-5f);

    const float target_after_ready_deg = steer_motors[0].getTargetTotalAngle();
    runHostControlCycle(chassis);

    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kReady);
    EXPECT_TRUE(chassis.wheel_config_[1].homing_state == Chassis::HomingState::kSearch);
    EXPECT_TRUE(std::fabs(target_after_ready_deg) > 1.0e-3f);
    EXPECT_NEAR(steer_motors[0].getTargetTotalAngle(), jia::radToDegF32(expected_hold_rad - expected_average_offset_rad), 1.0e-4f);
    EXPECT_NEAR(drive_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
}

TEST_CASE("testHomingThirdEdgeOutsideToleranceImmediatelyFaults")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.wheel_config_[0].homing_state = Chassis::HomingState::kSearch;
    chassis.wheel_config_[0].homing_zero_valid = false;
    chassis.wheel_config_[0].homing_last_sensor_active = false;
    steer_motors[0].setFeedbackCurrent(1200.0f);

    setPhotogateStateForWheel(0, false);
    steer_motors[0].setFeedbackTotalAngleDeg(0.0f);
    runHostControlCycle(chassis);

    setPhotogateStateForWheel(0, true);
    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);

    steer_motors[0].setFeedbackTotalAngleDeg(180.0f);
    setPhotogateStateForWheel(0, false);
    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);

    steer_motors[0].setFeedbackTotalAngleDeg(360.0f + 16.0f);
    setPhotogateStateForWheel(0, true);
    runHostControlCycle(chassis);

    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kFault);
    EXPECT_TRUE(!chassis.wheel_config_[0].homing_zero_valid);
}

TEST_CASE("testHomingEdgeDeltaToleranceBoundaryUsesCompileTimeMacro")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);
    const float within_tolerance_deg = JIA_CHASSIS_HOMING_EDGE_DELTA_TOLERANCE_DEG * 0.5f;
    const float outside_tolerance_deg = JIA_CHASSIS_HOMING_EDGE_DELTA_TOLERANCE_DEG + 1.0f;

    chassis.wheel_config_[0].homing_state = Chassis::HomingState::kSearch;
    chassis.wheel_config_[0].homing_zero_valid = false;
    chassis.wheel_config_[0].homing_last_sensor_active = false;
    steer_motors[0].setFeedbackCurrent(1200.0f);

    setPhotogateStateForWheel(0, false);
    steer_motors[0].setFeedbackTotalAngleDeg(0.0f);
    runHostControlCycle(chassis);

    setPhotogateStateForWheel(0, true);
    runHostControlCycle(chassis);

    steer_motors[0].setFeedbackTotalAngleDeg(180.0f + within_tolerance_deg);
    setPhotogateStateForWheel(0, false);
    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);

    steer_motors[0].setFeedbackTotalAngleDeg(360.0f + outside_tolerance_deg);
    setPhotogateStateForWheel(0, true);
    runHostControlCycle(chassis);

    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kEdgeDetected);
}

TEST_CASE("testHomingAfterAllWheelsReadyResumesZeroCurrentAndBodySpeedModes")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.wheel_config_[0].homing_state = Chassis::HomingState::kSearch;
    chassis.wheel_config_[0].homing_zero_valid = false;
    chassis.input_target_data_.mode = Chassis::Mode::kBodySpeedMode;
    chassis.input_target_data_.vel_y = 1.0f;

    finishWheelHomingByThreeConsistentEdges(chassis, 0, steer_motors);

    EXPECT_TRUE(runHostControlCycle(chassis));
    EXPECT_TRUE(std::fabs(drive_motors[0].getTargetRPM()) > 1.0e-3f);

    chassis.setZeroCurrent();
    runHostControlCycle(chassis);
    EXPECT_NEAR(drive_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
    EXPECT_NEAR(steer_motors[0].getTargetCurrent(), 0.0f, 1.0e-6f);
}

TEST_CASE("testLatePowerOnHomingDoesNotTimeoutBeforeFirstSteerFeedback")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.wheel_config_[0].homing_state = Chassis::HomingState::kSearch;
    chassis.wheel_config_[0].homing_zero_valid = false;
    chassis.wheel_config_[0].homing_last_sensor_active = false;
    chassis.wheel_config_[0].homing_last_edge_is_falling = false;
    chassis.wheel_config_[0].homing_elapsed_s = 0.0f;
    chassis.wheel_config_[0].homing_zero_offset_rad = 0.0f;
    chassis.wheel_config_[0].homing_runtime_zero_offset_rad = 0.0f;
    chassis.wheel_config_[0].homing_falling_edge_mech_rad = 0.0f;
    chassis.wheel_config_[0].homing_rising_edge_mech_rad = 0.0f;

    steer_motors[0].setFeedbackCurrent(0.0f);
    steer_motors[0].setFeedbackTotalAngleDeg(0.0f);
    setPhotogateStateForWheel(0, false);

    for (int cycle = 0; cycle < 6; ++cycle)
    {
        runHostControlCycle(chassis);
        EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);
        EXPECT_NEAR(chassis.wheel_config_[0].homing_elapsed_s, 0.0f, 1.0e-6f);
        EXPECT_TRUE(chassis.wheel_config_[0].homing_state != Chassis::HomingState::kFault);
    }

    steer_motors[0].setFeedbackCurrent(1200.0f);
    steer_motors[0].setFeedbackTotalAngleDeg(40.0f);

    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);

    setPhotogateStateForWheel(0, true);
    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);

    steer_motors[0].setFeedbackTotalAngleDeg(220.0f);
    setPhotogateStateForWheel(0, false);
    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);

    steer_motors[0].setFeedbackTotalAngleDeg(400.0f);
    setPhotogateStateForWheel(0, true);
    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kEdgeDetected);

    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kOffsetApply);

    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kContinuousAngleReady);

    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kReady);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_zero_valid);
}

TEST_CASE("testLatePowerOnHomingStillTimeoutsAfterFeedbackStartsWithoutEdge")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.wheel_config_[0].homing_state = Chassis::HomingState::kSearch;
    chassis.wheel_config_[0].homing_zero_valid = false;
    chassis.wheel_config_[0].homing_last_sensor_active = false;
    chassis.wheel_config_[0].homing_last_edge_is_falling = false;
    chassis.wheel_config_[0].homing_elapsed_s = 0.0f;
    chassis.wheel_config_[0].homing_zero_offset_rad = 0.0f;
    chassis.wheel_config_[0].homing_runtime_zero_offset_rad = 0.0f;

    steer_motors[0].setFeedbackCurrent(0.0f);
    steer_motors[0].setFeedbackTotalAngleDeg(0.0f);
    setPhotogateStateForWheel(0, false);

    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);

    steer_motors[0].setFeedbackCurrent(1400.0f);
    steer_motors[0].setFeedbackTotalAngleDeg(25.0f);
    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kSearch);

    const float armed_elapsed_s = chassis.wheel_config_[0].homing_elapsed_s;
    for (int cycle = 0; cycle < 6; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[0].homing_state == Chassis::HomingState::kFault)
        {
            break;
        }
    }

    EXPECT_TRUE(chassis.wheel_config_[0].homing_state == Chassis::HomingState::kFault);
    EXPECT_TRUE(chassis.wheel_config_[0].homing_elapsed_s > armed_elapsed_s);
}

TEST_CASE("testRecoveryRehomeTimeoutRelatchesFault")
{
    Chassis chassis;
    TestMotor steer_motors[4];
    VESC_Motor drive_motors[4];
    configureSteerFaultRecoveryHarness(chassis, steer_motors, drive_motors);

    chassis.input_target_data_.vel_x = 1.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        steer_motors[i].setFeedbackCurrent((i == 3) ? 5500.0f : 100.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
        drive_motors[i].setTargetCurrent(777.0f);
    }

    bool first_fault = false;
    for (int cycle = 0; cycle < 8; ++cycle)
    {
        runHostControlCycle(chassis);
        if (chassis.wheel_config_[3].homing_state == Chassis::HomingState::kFault)
        {
            first_fault = true;
            break;
        }
    }
    EXPECT_TRUE(first_fault);

    steer_motors[3].setFeedbackCurrent(-5500.0f);
    runHostControlCycle(chassis);
    EXPECT_TRUE(chassis.wheel_config_[3].homing_state == Chassis::HomingState::kSearch);

    setPhotogateStateForWheel(3, false);
    for (int cycle = 0; cycle < 5; ++cycle)
    {
        runHostControlCycle(chassis);
    }

    EXPECT_TRUE(chassis.wheel_config_[3].homing_state == Chassis::HomingState::kFault);
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(drive_motors[i].getTargetCurrent(), 0.0f, 1.0e-6f);
    }
}

} // namespace chassis_semantics_test
