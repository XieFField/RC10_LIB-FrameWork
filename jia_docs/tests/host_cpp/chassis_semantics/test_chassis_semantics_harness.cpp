#include "test_chassis_semantics_harness.h"

namespace chassis_semantics_test
{

// harness 实现只放跨分片复用的准备步骤和单周期驱动。
// 如果某段逻辑只服务一个 TEST_CASE，优先留在对应分片里，避免公共层变成新的大杂烩。
void configureDrivePidTuneHarness(DrivePidTuneHarness &harness)
{
    for (int i = 0; i < 4; ++i)
    {
        harness.chassis.wheel_config_[i].steer_motor_h = &harness.steer_motors[i];
        harness.chassis.wheel_config_[i].drive_motor_h = &harness.drive_vescs[i];
        harness.chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        harness.drive_vescs[i].pid_init(PID_Param_Config{}, 0.0f);
        harness.drive_vescs[i].setRpmControlMode(VESC_RPM_CONTROL_PID_CURRENT);
    }
}

void setDriveRuntimeDerivativeFirst(VESC_Motor &drive_motor, bool derivative_first)
{
    drive_motor.set_speed_pid_derivative_first(derivative_first);
}

void setPhotogateStateForWheel(int wheel_idx, bool active_high)
{
    static GPIO_TypeDef *const kPorts[4] = {
        kPHOTOGATE_1_GPIO_Port,
        kPHOTOGATE_2_GPIO_Port,
        kPHOTOGATE_3_GPIO_Port,
        kPHOTOGATE_4_GPIO_Port,
    };
    static const unsigned short kPins[4] = {
        kPHOTOGATE_1_Pin,
        kPHOTOGATE_2_Pin,
        kPHOTOGATE_3_Pin,
        kPHOTOGATE_4_Pin,
    };

    testHostSetPhotogate(kPorts[wheel_idx], kPins[wheel_idx], active_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void configureSingleWheelDebugHarness(Chassis &chassis, TestMotor steer_motors[4], VESC_Motor drive_motors[4])
{
    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].steer_motor_h = &steer_motors[i];
        chassis.wheel_config_[i].drive_motor_h = &drive_motors[i];
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].target_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].target_steer_motor_total_angle_rad = 0.0f;
    }
}

void configureSingleWheelIsolatedPlannerHarness(Chassis &chassis, TestMotor steer_motors[4], VESC_Motor drive_motors[4])
{
    configureSingleWheelDebugHarness(chassis, steer_motors, drive_motors);
    configureDriveContinuityHarness(chassis, drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kSCurve;
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg = 8.0f;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.min_scale = 0.2f;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;

    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 30U;
    chassis.debug_control_.common.control_wheel_index = 1U;
    chassis.debug_control_.common.observe_wheel_index = 1U;
    chassis.input_target_data_.mode = Chassis::Mode::kBodySpeedMode;
    chassis.input_target_data_.vel_x = 0.4f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;
    chassis.input_hwt_rot_z_ = 0.0f;
    chassis.current_mode_flag_.is_world_speed_mode = false;
    chassis.current_mode_flag_.is_lock_now_rot_z = false;
    chassis.current_mode_flag_.is_lock_to_rot_z = false;
    chassis.current_mode_flag_.is_wheel_torque_free = false;
}

void configureSingleWheelIsolatedDirectHarness(Chassis &chassis, TestMotor steer_motors[4], VESC_Motor drive_motors[4])
{
    configureSingleWheelDebugHarness(chassis, steer_motors, drive_motors);
    configureDriveContinuityHarness(chassis, drive_motors);

    chassis.runtime_strategy_cfg_.manual_speed_profile_mode = Chassis::ManualSpeedProfileMode::kSCurve;
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;

    chassis.debug_control_.common.enable = true;
    chassis.debug_control_.common.mode_raw = 30U;
    chassis.debug_control_.common.control_wheel_index = 1U;
    chassis.debug_control_.common.observe_wheel_index = 1U;
    chassis.input_target_data_.mode = Chassis::Mode::kBodySpeedMode;
    chassis.debug_control_.single_wheel.estop = false;
    chassis.debug_control_.single_wheel.input_deadzone = 0.0f;

    chassis.debug_control_.single_wheel.steer.enable = true;
    chassis.debug_control_.single_wheel.steer.input_mode_raw = static_cast<unsigned char>(Chassis::DirectAxisInputMode::kRcContinuous);
    chassis.debug_control_.single_wheel.steer.input_axis_raw = static_cast<unsigned char>(Chassis::SingleWheelInputAxis::kLeftX);
    chassis.debug_control_.single_wheel.steer.invert_input = false;
    chassis.debug_control_.single_wheel.steer.command_type_raw = static_cast<unsigned char>(Chassis::DirectSteerCommandType::kSingleTurnDeg);
    chassis.debug_control_.single_wheel.steer.command_value = 0.0f;
    chassis.debug_control_.single_wheel.steer.command_limit = 90.0f;
    chassis.debug_control_.single_wheel.steer.step_threshold = 0.5f;
    chassis.debug_control_.single_wheel.steer.step_value = 45.0f;
    chassis.debug_control_.single_wheel.steer.planner_mode_raw = static_cast<unsigned char>(Chassis::SingleWheelPlannerMode::kOff);
    chassis.debug_control_.single_wheel.steer.scurve.acc_acc = 60.0f;
    chassis.debug_control_.single_wheel.steer.scurve.acc_dec = 60.0f;
    chassis.debug_control_.single_wheel.steer.scurve.jerk_acc = 400.0f;
    chassis.debug_control_.single_wheel.steer.scurve.jerk_dec = 400.0f;
    chassis.debug_control_.single_wheel.steer.scurve.settle_vel_eps = 1.0e-4f;
    chassis.debug_control_.single_wheel.steer.scurve.settle_acc_eps = 0.05f;
    chassis.debug_control_.single_wheel.steer.trapezoid.acc = 120.0f;
    chassis.debug_control_.single_wheel.steer.trapezoid.dec = 120.0f;

    chassis.debug_control_.single_wheel.drive.enable = true;
    chassis.debug_control_.single_wheel.drive.input_mode_raw = static_cast<unsigned char>(Chassis::DirectAxisInputMode::kRcContinuous);
    chassis.debug_control_.single_wheel.drive.input_axis_raw = static_cast<unsigned char>(Chassis::SingleWheelInputAxis::kRightX);
    chassis.debug_control_.single_wheel.drive.invert_input = false;
    chassis.debug_control_.single_wheel.drive.command_type_raw = static_cast<unsigned char>(Chassis::DirectDriveCommandType::kRpm);
    chassis.debug_control_.single_wheel.drive.command_value = 0.0f;
    chassis.debug_control_.single_wheel.drive.command_limit = 1000.0f;
    chassis.debug_control_.single_wheel.drive.step_threshold = 0.5f;
    chassis.debug_control_.single_wheel.drive.step_value = 200.0f;
    chassis.debug_control_.single_wheel.drive.planner_mode_raw = static_cast<unsigned char>(Chassis::SingleWheelPlannerMode::kOff);
    chassis.debug_control_.single_wheel.drive.scurve.acc_acc = 2.0f;
    chassis.debug_control_.single_wheel.drive.scurve.acc_dec = 3.0f;
    chassis.debug_control_.single_wheel.drive.scurve.jerk_acc = 20.0f;
    chassis.debug_control_.single_wheel.drive.scurve.jerk_dec = 30.0f;
    chassis.debug_control_.single_wheel.drive.scurve.settle_vel_eps = 1.0e-4f;
    chassis.debug_control_.single_wheel.drive.scurve.settle_acc_eps = 0.05f;
    chassis.debug_control_.single_wheel.drive.trapezoid.acc = 2.0f;
    chassis.debug_control_.single_wheel.drive.trapezoid.dec = 3.0f;
}

void configureSingleWheelDriveVescHarness(Chassis &chassis, TestMotor steer_motors[4], VESC_Motor drive_motors[4])
{
    configureSingleWheelIsolatedDirectHarness(chassis, steer_motors, drive_motors);
    chassis.runtime_strategy_cfg_.manual_speed_profile_manual_only = true;
    chassis.debug_control_.common.control_wheel_index = 1U;
    chassis.debug_control_.common.observe_wheel_index = 1U;
    chassis.debug_control_.single_wheel.drive.enable = true;
    chassis.debug_control_.single_wheel.estop = false;
    chassis.debug_control_.single_wheel.drive.command_type_raw = static_cast<unsigned char>(Chassis::DirectDriveCommandType::kRpm);
    chassis.debug_control_.single_wheel.drive.input_mode_raw = static_cast<unsigned char>(Chassis::DirectAxisInputMode::kCached);
    chassis.debug_control_.single_wheel.drive.command_value = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        drive_motors[i].setRpmControlMode(VESC_RPM_CONTROL_PID_CURRENT);
        chassis.wheel_config_[i].homing_state = Chassis::HomingState::kReady;
    }
}

bool runHostDebugControlCycle(Chassis &chassis)
{
    chassis.isDebugMode();
    chassis.setModeFlag();
    chassis.resolvePlannerTargetData();
    chassis.refreshActuatorLimitState();
    chassis.updatePlannedMotionData();
    chassis.updateWheelFeedback();
    // Host harness 需要和真实 runThread() 保持同一时序：
    // 先推进 chassis 级首次上电统一等待窗口，再逐轮推进 homing 状态机。
    chassis.updateFirstBootHomingDelayState();

    bool all_homed = true;
    for (int i = 0; i < 4; ++i)
    {
        if (!chassis.updateHomingState(chassis.wheel_config_[i]))
        {
            all_homed = false;
        }
    }

    if (!all_homed && chassis.input_target_data_.zero_current_all)
    {
        chassis.input_target_data_.zero_current_all = false;
    }
    // 只要首次上电统一等待窗口还 active，就不能提前清掉 homing_start_request_，
    // 否则 host 夹具会和真实线程一样，把首次请求在进入 Search 之前提前消费掉。
    if (!chassis.first_boot_homing_delay_.active)
    {
        chassis.homing_start_request_ = false;
    }

    if (chassis.applyDebugModuleOverride(all_homed))
    {
        chassis.updateCurrentData(all_homed);
        chassis.refreshDebugMirror(all_homed);
        chassis.last_planned_data_ = chassis.planned_data_;
        return all_homed;
    }

    chassis.computeModuleCommands(chassis.planned_data_);
    chassis.applyModuleCommands(all_homed);
    chassis.updateCurrentData(all_homed);
    chassis.refreshDebugMirror(all_homed);
    chassis.last_planned_data_ = chassis.planned_data_;
    return all_homed;
}

void configureDriveContinuityHarness(Chassis &chassis, VESC_Motor drive_motors[4])
{
    chassis.runtime_strategy_cfg_.max_drive_alpha_rad_s2_ = 10.0f;
    chassis.runtime_strategy_cfg_.max_drive_omega_rad_s_ = 1000.0f;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = true;
    chassis.runtime_strategy_cfg_.enable_drive_omega_limit_ = false;
    chassis.input_target_data_.zero_current_all = false;
    chassis.current_mode_flag_.is_wheel_torque_free = false;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].drive_motor_h = &drive_motors[i];
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].target_drive_omega_rad_s = 0.0f;
        chassis.last_drive_omega_cmd_rad_s_[i] = 0.0f;
        chassis.planned_data_.drive_omega_rad_s[i] = 0.0f;
        drive_motors[i].pid_init(PID_Param_Config{}, 0.0f);
        drive_motors[i].setRpmControlMode(VESC_RPM_CONTROL_PID_CURRENT);
        drive_motors[i].setTargetRPM(0.0f);
    }
}

void configureDriveZeroStopHarness(Chassis &chassis, VESC_Motor drive_motors[4])
{
    configureDriveContinuityHarness(chassis, drive_motors);

    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_zero_stop_assist = true;
    chassis.runtime_strategy_cfg_.drive_zero_stop_brake_current_mA = 1200.0f;
}

void setWheelResidualSpeedMps(Chassis &chassis, int wheel_idx, float residual_speed_m_s)
{
    const float wheel_radius_m = chassis.runtime_strategy_cfg_.wheel_radius_m_;
    chassis.wheel_config_[wheel_idx].corrected_drive_omega_rad_s =
        (std::fabs(wheel_radius_m) > 1.0e-6f) ? (residual_speed_m_s / wheel_radius_m) : 0.0f;
}

void configureXParkWheelGeometry(Chassis &chassis)
{
    const float half_track_m = 0.20f;
    const float positions[4][2] = {
        {half_track_m, half_track_m},
        {-half_track_m, half_track_m},
        {-half_track_m, -half_track_m},
        {half_track_m, -half_track_m},
    };

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].pos_x_m = positions[i][0];
        chassis.wheel_config_[i].pos_y_m = positions[i][1];
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
    }
}

void setWheelPoseToXPark(Chassis &chassis)
{
    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad =
            std::atan2(chassis.wheel_config_[i].pos_y_m, chassis.wheel_config_[i].pos_x_m);
    }
}

void configureHardGateLaunchHarness(Chassis &chassis, VESC_Motor drive_motors[4])
{
    configureDriveContinuityHarness(chassis, drive_motors);
    configureXParkWheelGeometry(chassis);
    setWheelPoseToXPark(chassis);

    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = true;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.close_angle_deg = 1.0f;
    chassis.runtime_strategy_cfg_.low_speed_drive_suppression.min_scale = 0.0f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.max_acc_xy_acc_ = 1.0f;
    chassis.runtime_strategy_cfg_.max_acc_xy_dec_ = 1.0f;
    chassis.runtime_strategy_cfg_.max_alpha_z_acc_ = 2.0f;
    chassis.runtime_strategy_cfg_.max_alpha_z_dec_ = 2.0f;
    chassis.xpark_gate_active_ = true;
}

void configureXParkTriggerHarness(Chassis &chassis)
{
    const float half_track_m = 0.20f;
    const float positions[4][2] = {
        {half_track_m, half_track_m},
        {-half_track_m, half_track_m},
        {-half_track_m, -half_track_m},
        {half_track_m, -half_track_m},
    };

    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_enter_m_s = 0.01f;
    chassis.runtime_strategy_cfg_.near_zero_cfg_.base_exit_m_s = 0.03f;
    chassis.runtime_strategy_cfg_.xpark_entry_delay_ms = 10U;
    chassis.xpark_gate_active_ = false;
    chassis.xpark_stationary_hold_ms_ = 0U;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].pos_x_m = positions[i][0];
        chassis.wheel_config_[i].pos_y_m = positions[i][1];
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
    }
}

void configureSteerFaultRecoveryHarness(Chassis &chassis, TestMotor steer_motors[4], VESC_Motor drive_motors[4])
{
    testHostResetPhotogates();

    chassis.runtime_strategy_cfg_.wheel_radius_m_ = 0.05f;
    chassis.runtime_strategy_cfg_.enable_low_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_high_speed_drive_suppression = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_omega_limit_ = false;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.enable = true;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.ignore_during_xpark_hold = true;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.freeze_current_delta_mA = 1.0f;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.active_current_min_mA = 500.0f;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.freeze_angle_delta_rad = 1.0e-4f;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.freeze_duration_ms = 5U;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.recovery_current_delta_mA = 1000.0f;
    chassis.runtime_strategy_cfg_.steer_fault_cfg.recovery_toggle_threshold = 1U;
    chassis.runtime_strategy_cfg_.max_acc_xy_acc_ = 1000.0f;
    chassis.runtime_strategy_cfg_.max_acc_xy_dec_ = 1000.0f;
    chassis.runtime_strategy_cfg_.max_alpha_z_acc_ = 1000.0f;
    chassis.runtime_strategy_cfg_.max_alpha_z_dec_ = 1000.0f;
    chassis.input_target_data_.zero_current_all = false;
    chassis.input_target_data_.mode = Chassis::Mode::kBodySpeedMode;
    chassis.input_target_data_.vel_x = 0.0f;
    chassis.input_target_data_.vel_y = 0.0f;
    chassis.input_target_data_.omega_z = 0.0f;
    chassis.current_mode_flag_.is_wheel_torque_free = false;
    chassis.current_mode_flag_.is_world_speed_mode = false;
    chassis.current_mode_flag_.is_lock_now_rot_z = false;
    chassis.current_mode_flag_.is_lock_to_rot_z = false;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].pos_x_m = 0.0f;
        chassis.wheel_config_[i].pos_y_m = 0.0f;
        chassis.wheel_config_[i].theta_oa_to_owi_rad = 0.0f;
        chassis.wheel_config_[i].steer_motor_sign = 1.0f;
        chassis.wheel_config_[i].drive_motor_sign = 1.0f;
        chassis.wheel_config_[i].steer_motor_h = &steer_motors[i];
        chassis.wheel_config_[i].drive_motor_h = &drive_motors[i];
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
        chassis.wheel_config_[i].homing_falling_edge_mech_rad = 0.0f;
        chassis.wheel_config_[i].homing_rising_edge_mech_rad = 0.0f;
        chassis.wheel_config_[i].homing_search_rpm = JIA_CHASSIS_HOMING_SEARCH_RPM;
        chassis.wheel_config_[i].homing_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].homing_timeout_s = 0.003f;
        chassis.wheel_config_[i].homing_state = Chassis::HomingState::kReady;
        chassis.wheel_config_[i].homing_last_sensor_active = false;
        chassis.wheel_config_[i].homing_last_edge_is_falling = false;
        chassis.wheel_config_[i].homing_zero_valid = true;
        chassis.wheel_config_[i].homing_elapsed_s = 0.0f;
        chassis.wheel_config_[i].homing_runtime_zero_offset_rad = 0.0f;
        chassis.wheel_config_[i].corrected_steer_motor_total_angle_rad = 0.0f;
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].target_steer_motor_total_angle_rad = 0.0f;
        chassis.wheel_config_[i].target_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].steer_target_velocity_rad_s = 0.0f;
        chassis.wheel_config_[i].flipped_drive_direction = false;
        chassis.last_drive_omega_cmd_rad_s_[i] = 0.0f;
        chassis.last_steer_rate_cmd_rad_s_[i] = 0.0f;
        chassis.actuator_command_frame_.drive_omega_rad_s[i] = 0.0f;
        chassis.actuator_command_frame_.steer_oa_total_rad[i] = 0.0f;
        chassis.actuator_command_frame_.steer_corrected_local_total_rad[i] = 0.0f;
        chassis.actuator_command_frame_.steer_cmd_oa_total_rad[i] = 0.0f;
        chassis.actuator_command_frame_.steer_cmd_corrected_local_total_rad[i] = 0.0f;
        chassis.planned_data_.drive_omega_rad_s[i] = 0.0f;
        chassis.planned_data_.steer_angle_oa_rad[i] = 0.0f;
        chassis.current_data_.drive_omega_rad_s[i] = 0.0f;
        chassis.current_data_.steer_angle_oa_rad[i] = 0.0f;
        steer_motors[i].setFeedbackCurrent(0.0f);
        steer_motors[i].setFeedbackRpm(0.0f);
        steer_motors[i].setFeedbackTotalAngleDeg(0.0f);
        steer_motors[i].setTargetCurrent(0.0f);
        steer_motors[i].setTargetRPM(0.0f);
        steer_motors[i].setTargetTotalAngle(0.0f);
        drive_motors[i].setFeedbackCurrent(0.0f);
        drive_motors[i].setFeedbackRpm(0.0f);
        drive_motors[i].setFeedbackTotalAngleDeg(0.0f);
        drive_motors[i].pid_init(PID_Param_Config{}, 0.0f);
        drive_motors[i].setRpmControlMode(VESC_RPM_CONTROL_PID_CURRENT);
        drive_motors[i].setTargetCurrent(0.0f);
        drive_motors[i].setTargetRPM(0.0f);
        drive_motors[i].setTargetTotalAngle(0.0f);
        setPhotogateStateForWheel(i, false);
    }
}

bool runHostControlCycle(Chassis &chassis)
{
    chassis.resolvePlannerTargetData();
    chassis.refreshActuatorLimitState();
    chassis.updatePlannedMotionData();
    chassis.updateWheelFeedback();
    // 普通 host control cycle 也要复用和真实线程一致的放行顺序，避免测试与量产时序分叉。
    chassis.updateFirstBootHomingDelayState();

    bool all_homed = true;
    for (int i = 0; i < 4; ++i)
    {
        if (!chassis.updateHomingState(chassis.wheel_config_[i]))
        {
            all_homed = false;
        }
    }

    if (!all_homed && chassis.input_target_data_.zero_current_all)
    {
        chassis.input_target_data_.zero_current_all = false;
    }
    // 统一等待窗口未结束前，启动请求必须继续保留，直到四轮真正获得 Idle -> Search 的放行资格。
    if (!chassis.first_boot_homing_delay_.active)
    {
        chassis.homing_start_request_ = false;
    }

    chassis.computeModuleCommands(chassis.planned_data_);
    chassis.applyModuleCommands(all_homed);
    chassis.updateCurrentData(all_homed);
    chassis.refreshDebugMirror(all_homed);
    chassis.last_planned_data_ = chassis.planned_data_;
    return all_homed;
}

void runDebugPlannerCycleForHost(Chassis &chassis)
{
    chassis.isDebugMode();
    chassis.setModeFlag();
    chassis.resolvePlannerTargetData();
    chassis.refreshActuatorLimitState();
    chassis.updatePlannedMotionData();
    chassis.last_planned_data_ = chassis.planned_data_;
}

bool runDebugControlCycleForHost(Chassis &chassis)
{
    chassis.isDebugMode();
    chassis.setModeFlag();
    return runHostControlCycle(chassis);
}

void emitDebugOutputForHost(Chassis &chassis, bool all_homed)
{
    chassis.refreshDebugMirror(all_homed);
    chassis.emitDebugOutputByMode(all_homed);
}

void configureDebugOutputFamily(Chassis &chassis, unsigned char family_raw)
{
    chassis.debug_output_.output_enable = true;
    chassis.debug_output_.output_family_raw = family_raw;
}

void configureJustFloatProfile(Chassis &chassis, unsigned char profile_raw)
{
    chassis.debug_output_.justfloat.profile_raw = profile_raw;
}

void configureSingleWheelPayload(Chassis &chassis, unsigned char payload_raw)
{
    chassis.debug_output_.justfloat.single_wheel_payload_raw = payload_raw;
}

void configureYawPidTraceHarness(Chassis &chassis)
{
    testHostResetJustFloatCapture();
    configureDebugOutputFamily(chassis, 2U);
    configureJustFloatProfile(chassis, 2U);
    chassis.debug_output_.justfloat.yaw_pid.period_ms = 0U;
    chassis.debug_output_runtime_.justfloat.yaw_pid.last_ms = 0U;
    configureSingleWheelPayload(chassis, 0U);
    chassis.time_ms_ = 100U;
    chassis.input_hwt_omega_z_ = 0.25f;
    chassis.debug_mirror_.all_homed = true;
    chassis.debug_mirror_.steer_fault_any_active = false;
    chassis.debug_mirror_.high_speed_drive_suppression_active = false;
    chassis.debug_mirror_.reverse_intent_active = false;
}

void finishWheelHomingByThreeConsistentEdges(Chassis &chassis, int wheel_idx, TestMotor steer_motors[4])
{
    bool sensor_high = chassis.wheel_config_[wheel_idx].homing_last_sensor_active;
    setPhotogateStateForWheel(wheel_idx, sensor_high);
    steer_motors[wheel_idx].setFeedbackCurrent(1200.0f);
    runHostControlCycle(chassis);

    steer_motors[wheel_idx].setFeedbackTotalAngleDeg(0.0f);
    sensor_high = !sensor_high;
    setPhotogateStateForWheel(wheel_idx, sensor_high);
    runHostControlCycle(chassis);

    steer_motors[wheel_idx].setFeedbackTotalAngleDeg(180.0f);
    sensor_high = !sensor_high;
    setPhotogateStateForWheel(wheel_idx, sensor_high);
    runHostControlCycle(chassis);

    steer_motors[wheel_idx].setFeedbackTotalAngleDeg(360.0f);
    sensor_high = !sensor_high;
    setPhotogateStateForWheel(wheel_idx, sensor_high);
    runHostControlCycle(chassis);
    runHostControlCycle(chassis);
    runHostControlCycle(chassis);
    runHostControlCycle(chassis);
}

void finishWheelHomingByEdgeAndAlign(Chassis &chassis, int wheel_idx, TestMotor steer_motors[4])
{
    finishWheelHomingByThreeConsistentEdges(chassis, wheel_idx, steer_motors);
}

Chassis::ActuatorCommandFrame makeDriveOnlyCommandFrame(float drive_omega_rad_s)
{
    Chassis::ActuatorCommandFrame frame{};
    for (int i = 0; i < 4; ++i)
    {
        frame.drive_omega_rad_s[i] = drive_omega_rad_s;
    }
    return frame;
}

Chassis::ActuatorCommandFrame makePerWheelDriveCommandFrame(float drive0, float drive1, float drive2, float drive3)
{
    Chassis::ActuatorCommandFrame frame{};
    frame.drive_omega_rad_s[0] = drive0;
    frame.drive_omega_rad_s[1] = drive1;
    frame.drive_omega_rad_s[2] = drive2;
    frame.drive_omega_rad_s[3] = drive3;
    return frame;
}

Chassis::SwervePlannerOutput makeNeutralPlannerOutput()
{
    Chassis::SwervePlannerOutput output{};
    output.high_speed_suppression_scale = 1.0f;
    for (int i = 0; i < 4; ++i)
    {
        output.low_speed_suppression_scale[i] = 1.0f;
    }
    return output;
}

void advanceDriveZeroStopCycle(Chassis &chassis,
                               float command_drive_rad_s,
                               float residual_speed_m_s,
                               TickType_t time_ms)
{
    chassis.time_ms_ = time_ms;
    chassis.storePlannedActuatorFrame(makeNeutralPlannerOutput(), makeDriveOnlyCommandFrame(command_drive_rad_s));
    setWheelResidualSpeedMps(chassis, 0, residual_speed_m_s);
    chassis.applyModuleCommands(true);
}

float getWheelXParkTargetOaRad(const Chassis &chassis, int wheel_idx)
{
    return chassis.getXParkAngle(chassis.wheel_config_[wheel_idx]);
}

float getWheelOParkTargetOaRad(const Chassis &chassis, int wheel_idx)
{
    return chassis.getOParkAngle(chassis.wheel_config_[wheel_idx]);
}

void setWheelOaAngleRad(Chassis &chassis, int wheel_idx, float oa_rad)
{
    chassis.wheel_config_[wheel_idx].corrected_steer_motor_total_angle_rad =
        chassis.mapWheelOaTotalToCorrectedLocal(chassis.wheel_config_[wheel_idx], oa_rad);
}

Chassis::ActuatorCommandFrame makeXParkSteerCommandFrame(Chassis &chassis)
{
    Chassis::ActuatorCommandFrame frame{};
    for (int i = 0; i < 4; ++i)
    {
        const float xpark_target_oa_rad = getWheelXParkTargetOaRad(chassis, i);
        frame.steer_oa_total_rad[i] = xpark_target_oa_rad;
        frame.steer_corrected_local_total_rad[i] =
            chassis.mapWheelOaTotalToCorrectedLocal(chassis.wheel_config_[i], xpark_target_oa_rad);
        frame.steer_cmd_oa_total_rad[i] = frame.steer_oa_total_rad[i];
        frame.steer_cmd_corrected_local_total_rad[i] = frame.steer_corrected_local_total_rad[i];
    }
    return frame;
}

Chassis::ActuatorCommandFrame makeOParkSteerCommandFrame(Chassis &chassis)
{
    Chassis::ActuatorCommandFrame frame{};
    for (int i = 0; i < 4; ++i)
    {
        const float opark_target_oa_rad = getWheelOParkTargetOaRad(chassis, i);
        frame.steer_oa_total_rad[i] = opark_target_oa_rad;
        frame.steer_corrected_local_total_rad[i] =
            chassis.mapWheelOaTotalToCorrectedLocal(chassis.wheel_config_[i], opark_target_oa_rad);
        frame.steer_cmd_oa_total_rad[i] = frame.steer_oa_total_rad[i];
        frame.steer_cmd_corrected_local_total_rad[i] = frame.steer_corrected_local_total_rad[i];
    }
    return frame;
}

void configureXParkSteerHoldHarness(XParkSteerHoldHarness &harness)
{
    Chassis &chassis = harness.chassis;
    configureDriveContinuityHarness(chassis, harness.drive_motors);
    configureXParkWheelGeometry(chassis);

    chassis.runtime_strategy_cfg_.idle_posture_mode = Chassis::IdlePostureMode::kXPark;
    chassis.runtime_strategy_cfg_.enable_drive_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_drive_omega_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_rate_limit_ = false;
    chassis.runtime_strategy_cfg_.enable_steer_alpha_limit_ = false;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.enable = true;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.entry_angle_deg = 1.0f;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.exit_angle_deg = 3.0f;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.settle_angle_deg = 0.6f;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.settle_target_rate_deg_s = 2.0f;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.settle_hold_ms = 2U;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.reacquire_hold_ms = 2U;
    chassis.runtime_strategy_cfg_.xpark_steer_hold_cfg_.entry_reset_enable = true;
    chassis.input_target_data_.zero_current_all = false;
    chassis.current_mode_flag_.is_wheel_torque_free = false;
    chassis.current_mode_flag_.is_world_speed_mode = false;
    chassis.current_mode_flag_.is_lock_now_rot_z = false;
    chassis.current_mode_flag_.is_lock_to_rot_z = false;
    chassis.debug_control_.common.enable = false;
    chassis.xpark_gate_active_ = true;

    for (int i = 0; i < 4; ++i)
    {
        chassis.wheel_config_[i].steer_motor_h = &harness.steer_motors[i];
        chassis.wheel_config_[i].homing_state = Chassis::HomingState::kReady;
        chassis.wheel_config_[i].steer_fault_state = Chassis::SteerFaultState::kNone;
        chassis.wheel_config_[i].corrected_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].target_drive_omega_rad_s = 0.0f;
        chassis.wheel_config_[i].target_steer_motor_total_angle_rad = 0.0f;
        chassis.wheel_config_[i].steer_target_velocity_rad_s = 0.0f;
        chassis.last_drive_omega_cmd_rad_s_[i] = 0.0f;
        chassis.last_steer_rate_cmd_rad_s_[i] = 0.0f;
        harness.steer_motors[i].setTargetCurrent(0.0f);
        harness.steer_motors[i].setTargetTotalAngle(0.0f);
        harness.steer_motors[i].resetLastCommandObservation();
        setWheelOaAngleRad(chassis, i, getWheelXParkTargetOaRad(chassis, i));
    }
}

Chassis::SwervePlannerInput makeGatePlannerInput(float steering_error_deg,
                                                 float command_vel_x,
                                                 float command_omega_z,
                                                 float max_residual_speed_m_s)
{
    Chassis::SwervePlannerInput input{};
    input.command.vel_x = command_vel_x;
    input.command.omega_z = command_omega_z;
    input.max_residual_speed_m_s = max_residual_speed_m_s;
    for (int i = 0; i < 4; ++i)
    {
        input.residual_speed_m_s[i] = max_residual_speed_m_s;
        input.current_oa_total_rad[i] = 0.0f;
        input.wheel_speed_m_s[i] = std::fabs(command_vel_x);
    }
    input.current_oa_total_rad[0] = 0.0f;
    return input;
}

} // namespace chassis_semantics_test
