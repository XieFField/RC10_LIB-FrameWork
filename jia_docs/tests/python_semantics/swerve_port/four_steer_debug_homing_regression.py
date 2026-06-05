from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from enum import Enum, auto


class HomingState(Enum):
    IDLE = auto()
    SEARCH = auto()
    EDGE_DETECTED = auto()
    OFFSET_APPLY = auto()
    CONTINUOUS_ANGLE_READY = auto()
    READY = auto()
    FAULT = auto()


@dataclass
class WheelState:
    theta_oa_to_owi_rad: float = 0.0
    homing_sensor_active_high: bool = True
    homing_zero_offset_rad: float = 0.0
    homing_runtime_zero_offset_rad: float = 0.0
    homing_last_sensor_raw_high: bool = False
    homing_state: HomingState = HomingState.SEARCH
    homing_zero_valid: bool = False
    homing_elapsed_s: float = 0.0
    homing_timeout_s: float = 5.0
    homing_falling_edge_mech_rad: float = math.radians(60.0)
    homing_rising_edge_mech_rad: float = math.radians(-120.0)


def broken_single_wheel_effective_drive(is_wheel_torque_free: bool, target_rpm: float) -> float:
    if is_wheel_torque_free:
        return 0.0
    return target_rpm


def fixed_single_wheel_effective_drive(is_single_wheel_debug: bool, all_homed: bool, target_rpm: float) -> float:
    if not all_homed:
        return 0.0
    if is_single_wheel_debug:
        return target_rpm
    return target_rpm


def broken_single_wheel_target_local_rad(
    *,
    current_oa_total_rad: float,
    target_oa_mod_rad: float,
    theta_oa_to_owi_rad: float,
) -> float:
    del current_oa_total_rad
    return target_oa_mod_rad - theta_oa_to_owi_rad


def nearest_equivalent_angle(current_rad: float, target_mod_rad: float) -> float:
    two_pi = 2.0 * math.pi
    delta = (target_mod_rad - current_rad) % two_pi
    if delta > math.pi:
        delta -= two_pi
    return current_rad + delta


def fixed_single_wheel_target_local_rad(
    *,
    current_oa_total_rad: float,
    target_oa_mod_rad: float,
    theta_oa_to_owi_rad: float,
) -> float:
    nearest_oa_total = nearest_equivalent_angle(current_oa_total_rad, target_oa_mod_rad)
    return nearest_oa_total - theta_oa_to_owi_rad


def limit_position_second_order(
    *,
    current_value: float,
    current_rate: float,
    target_value: float,
    max_rate: float,
    max_accel: float,
    dt_s: float,
) -> tuple[float, float]:
    safe_dt = 1.0e-3 if dt_s <= 1.0e-6 else dt_s
    delta_value = target_value - current_value
    desired_rate = max(-max_rate, min(max_rate, delta_value / safe_dt))
    rate_delta_limit = max_accel * safe_dt

    next_rate = current_rate
    if desired_rate > current_rate + rate_delta_limit:
        next_rate = current_rate + rate_delta_limit
    elif desired_rate < current_rate - rate_delta_limit:
        next_rate = current_rate - rate_delta_limit
    else:
        next_rate = desired_rate

    next_rate = max(-max_rate, min(max_rate, next_rate))
    step_value = next_rate * safe_dt
    if abs(step_value) > abs(delta_value):
        step_value = delta_value
        next_rate = step_value / safe_dt
    return current_value + step_value, next_rate


def broken_update_homing_state(
    wheel: WheelState,
    *,
    sensor_raw_high: bool,
    raw_total_angle_rad: float,
) -> None:
    is_edge = sensor_raw_high != wheel.homing_last_sensor_raw_high
    if is_edge:
        is_falling = wheel.homing_last_sensor_raw_high and (not sensor_raw_high)
        edge_mech_oa_rad = math.radians(60.0) if is_falling else math.radians(-120.0)
        edge_local_corrected_rad = edge_mech_oa_rad - wheel.theta_oa_to_owi_rad
        wheel.homing_runtime_zero_offset_rad = edge_local_corrected_rad + wheel.homing_zero_offset_rad - raw_total_angle_rad
        wheel.homing_zero_valid = True
        wheel.homing_state = HomingState.EDGE_DETECTED
    wheel.homing_last_sensor_raw_high = sensor_raw_high


def fixed_update_homing_state(
    wheel: WheelState,
    *,
    sensor_raw_high: bool,
    raw_total_angle_rad: float,
) -> None:
    is_edge = sensor_raw_high != wheel.homing_last_sensor_raw_high
    if is_edge:
        is_falling = wheel.homing_last_sensor_raw_high and (not sensor_raw_high)
        edge_mech_oa_rad = wheel.homing_falling_edge_mech_rad if is_falling else wheel.homing_rising_edge_mech_rad
        edge_local_corrected_rad = edge_mech_oa_rad - wheel.theta_oa_to_owi_rad
        wheel.homing_runtime_zero_offset_rad = edge_local_corrected_rad + wheel.homing_zero_offset_rad - raw_total_angle_rad
        wheel.homing_zero_valid = True
        wheel.homing_state = HomingState.EDGE_DETECTED
    wheel.homing_last_sensor_raw_high = sensor_raw_high


def broken_logic_active(raw_high: bool, active_high: bool) -> bool:
    del active_high
    return raw_high


def fixed_logic_active(raw_high: bool, active_high: bool) -> bool:
    return raw_high if active_high else (not raw_high)


def broken_single_wheel_soft_steer_target(
    *,
    current_local_rad: float,
    current_oa_total_rad: float,
    target_oa_mod_rad: float,
    theta_oa_to_owi_rad: float,
) -> tuple[float, float]:
    del current_oa_total_rad
    del target_oa_mod_rad
    del theta_oa_to_owi_rad
    return current_local_rad + math.radians(90.0), 0.0


def fixed_single_wheel_soft_steer_target(
    *,
    current_local_rad: float,
    current_oa_total_rad: float,
    target_oa_mod_rad: float,
    theta_oa_to_owi_rad: float,
    last_rate_rad_s: float,
    max_rate_rad_s: float,
    max_accel_rad_s2: float,
    dt_s: float,
) -> tuple[float, float]:
    target_oa_total_rad = nearest_equivalent_angle(current_oa_total_rad, target_oa_mod_rad)
    selected_local_total_rad = target_oa_total_rad - theta_oa_to_owi_rad
    return limit_position_second_order(
        current_value=current_local_rad,
        current_rate=last_rate_rad_s,
        target_value=selected_local_total_rad,
        max_rate=max_rate_rad_s,
        max_accel=max_accel_rad_s2,
        dt_s=dt_s,
    )


def broken_single_wheel_drive_gate(
    *,
    target_drive_rpm: float,
    steer_error_deg: float,
    release_error_deg: float,
) -> float:
    del steer_error_deg
    del release_error_deg
    return target_drive_rpm


def fixed_single_wheel_drive_gate(
    *,
    target_drive_rpm: float,
    steer_error_deg: float,
    release_error_deg: float,
) -> float:
    if abs(steer_error_deg) > max(0.0, release_error_deg):
        return 0.0
    return target_drive_rpm


def test_red_single_wheel_debug_is_not_masked() -> None:
    effective = broken_single_wheel_effective_drive(is_wheel_torque_free=True, target_rpm=120.0)
    assert effective == 120.0, "当前坏实现会把单轮直控错误清零；这个断言应失败"


def test_green_single_wheel_debug_survives_gate_when_homed() -> None:
    effective = fixed_single_wheel_effective_drive(is_single_wheel_debug=True, all_homed=True, target_rpm=120.0)
    assert effective == 120.0


def test_red_single_wheel_steer_should_pick_nearest_equivalent_angle() -> None:
    current_oa = math.radians(350.0)
    target_oa = math.radians(10.0)
    target_local = broken_single_wheel_target_local_rad(
        current_oa_total_rad=current_oa,
        target_oa_mod_rad=target_oa,
        theta_oa_to_owi_rad=0.0,
    )
    delta_deg = math.degrees(target_local - current_oa)
    assert abs(delta_deg) <= 30.0, "当前坏实现会让单轮直控从 350° 绕远路转到 10°；这个断言应失败"


def test_green_single_wheel_steer_uses_nearest_equivalent_angle() -> None:
    current_oa = math.radians(350.0)
    target_oa = math.radians(10.0)
    target_local = fixed_single_wheel_target_local_rad(
        current_oa_total_rad=current_oa,
        target_oa_mod_rad=target_oa,
        theta_oa_to_owi_rad=0.0,
    )
    delta_deg = math.degrees(target_local - current_oa)
    assert abs(delta_deg - 20.0) < 1.0e-9


def test_red_homing_edges_should_be_configurable() -> None:
    wheel = WheelState(
        theta_oa_to_owi_rad=math.radians(30.0),
        homing_zero_offset_rad=math.radians(5.0),
        homing_last_sensor_raw_high=True,
        homing_falling_edge_mech_rad=math.radians(40.0),
    )
    broken_update_homing_state(wheel, sensor_raw_high=False, raw_total_angle_rad=math.radians(10.0))
    expected = math.radians(40.0 - 30.0 + 5.0 - 10.0)
    assert abs(wheel.homing_runtime_zero_offset_rad - expected) < 1.0e-9, "当前坏实现写死60度；这个断言应失败"


def test_green_homing_edges_follow_config() -> None:
    wheel = WheelState(
        theta_oa_to_owi_rad=math.radians(30.0),
        homing_zero_offset_rad=math.radians(5.0),
        homing_last_sensor_raw_high=True,
        homing_falling_edge_mech_rad=math.radians(40.0),
        homing_rising_edge_mech_rad=math.radians(-100.0),
    )
    fixed_update_homing_state(wheel, sensor_raw_high=False, raw_total_angle_rad=math.radians(10.0))
    expected = math.radians(40.0 - 30.0 + 5.0 - 10.0)
    assert abs(wheel.homing_runtime_zero_offset_rad - expected) < 1.0e-9


def test_red_logic_active_mirror_should_follow_sensor_polarity() -> None:
    logic_active = broken_logic_active(raw_high=False, active_high=False)
    assert logic_active is True, "当前坏实现若只镜像 raw 电平，看不出 low-active 传感器已触发；这个断言应失败"


def test_green_logic_active_mirror_follows_sensor_polarity() -> None:
    logic_active = fixed_logic_active(raw_high=False, active_high=False)
    assert logic_active is True


def test_red_single_wheel_soft_steer_should_not_jump_to_target_in_one_tick() -> None:
    next_local, _ = broken_single_wheel_soft_steer_target(
        current_local_rad=0.0,
        current_oa_total_rad=0.0,
        target_oa_mod_rad=math.radians(90.0),
        theta_oa_to_owi_rad=0.0,
    )
    assert next_local < math.radians(10.0), "当前坏实现会一拍直接跳到目标角；这个断言应失败"


def test_green_single_wheel_soft_steer_is_rate_limited() -> None:
    next_local, next_rate = fixed_single_wheel_soft_steer_target(
        current_local_rad=0.0,
        current_oa_total_rad=0.0,
        target_oa_mod_rad=math.radians(90.0),
        theta_oa_to_owi_rad=0.0,
        last_rate_rad_s=0.0,
        max_rate_rad_s=math.radians(120.0),
        max_accel_rad_s2=math.radians(600.0),
        dt_s=0.01,
    )
    assert abs(math.degrees(next_local) - 0.06) < 1.0e-9
    assert abs(math.degrees(next_rate) - 6.0) < 1.0e-9


def test_red_single_wheel_drive_should_wait_until_steer_error_is_small() -> None:
    effective_rpm = broken_single_wheel_drive_gate(
        target_drive_rpm=120.0,
        steer_error_deg=35.0,
        release_error_deg=5.0,
    )
    assert effective_rpm == 0.0, "当前坏实现会在舵角误差还很大时提前放开驱动；这个断言应失败"


def test_green_single_wheel_drive_waits_until_steer_error_is_small() -> None:
    blocked_rpm = fixed_single_wheel_drive_gate(
        target_drive_rpm=120.0,
        steer_error_deg=35.0,
        release_error_deg=5.0,
    )
    released_rpm = fixed_single_wheel_drive_gate(
        target_drive_rpm=120.0,
        steer_error_deg=3.0,
        release_error_deg=5.0,
    )
    assert blocked_rpm == 0.0
    assert released_rpm == 120.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("red", "green", "red-soft-steer", "green-soft-steer", "red-drive-gate", "green-drive-gate"), required=True)
    args = parser.parse_args()

    if args.mode == "red":
        test_red_single_wheel_debug_is_not_masked()
        test_red_single_wheel_steer_should_pick_nearest_equivalent_angle()
        test_red_homing_edges_should_be_configurable()
        test_red_logic_active_mirror_should_follow_sensor_polarity()
    elif args.mode == "red-soft-steer":
        test_red_single_wheel_soft_steer_should_not_jump_to_target_in_one_tick()
    elif args.mode == "green-soft-steer":
        test_green_single_wheel_soft_steer_is_rate_limited()
    elif args.mode == "red-drive-gate":
        test_red_single_wheel_drive_should_wait_until_steer_error_is_small()
    elif args.mode == "green-drive-gate":
        test_green_single_wheel_drive_waits_until_steer_error_is_small()
    else:
        test_green_single_wheel_debug_survives_gate_when_homed()
        test_green_single_wheel_steer_uses_nearest_equivalent_angle()
        test_green_homing_edges_follow_config()
        test_green_logic_active_mirror_follows_sensor_polarity()

    print(f"{args.mode} check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
