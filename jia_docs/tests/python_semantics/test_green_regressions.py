from __future__ import annotations

from homing_state.update_homing_state_regression import run_all as run_homing_state_green
from swerve_port.four_steer_debug_homing_regression import (
    test_green_single_wheel_debug_survives_gate_when_homed,
    test_green_single_wheel_steer_uses_nearest_equivalent_angle,
    test_green_homing_edge_uses_configured_edge_angles,
    test_green_logic_level_respects_active_low_configuration,
    test_green_single_wheel_soft_target_respects_limit_profile,
    test_green_single_wheel_drive_gate_closes_when_steer_error_large,
)
from time_stamp_us64.time_stamp_us64_regression import run_all as run_time_stamp_us64_green


def test_homing_state_green_suite() -> None:
    run_homing_state_green()


def test_time_stamp_us64_green_suite() -> None:
    run_time_stamp_us64_green()


def test_swerve_port_green_suite() -> None:
    test_green_single_wheel_debug_survives_gate_when_homed()
    test_green_single_wheel_steer_uses_nearest_equivalent_angle()
    test_green_homing_edge_uses_configured_edge_angles()
    test_green_logic_level_respects_active_low_configuration()
    test_green_single_wheel_soft_target_respects_limit_profile()
    test_green_single_wheel_drive_gate_closes_when_steer_error_large()
