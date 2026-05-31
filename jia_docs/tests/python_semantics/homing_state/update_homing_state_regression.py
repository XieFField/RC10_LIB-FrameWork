from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from jia_docs.tests.shared.python_test_support import make_mode_parser


PERIOD_S = 0.001


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
    homing_enabled: bool = False
    homing_gpio_port_present: bool = False
    homing_zero_offset_rad: float = 0.0
    homing_timeout_s: float = 5.0
    homing_state: HomingState = HomingState.IDLE
    homing_last_sensor_active: bool = False
    homing_zero_valid: bool = False
    homing_elapsed_s: float = 0.0
    homing_runtime_zero_offset_rad: float = 0.0


def update_homing_state(
    wheel: WheelState,
    *,
    homing_start_request: bool,
    sensor_active: bool,
    raw_total_angle_rad: float,
) -> bool:
    if (not wheel.homing_enabled) or (not wheel.homing_gpio_port_present):
        wheel.homing_state = HomingState.READY
        wheel.homing_zero_valid = True
        wheel.homing_runtime_zero_offset_rad = wheel.homing_zero_offset_rad
        return True

    if wheel.homing_state == HomingState.IDLE:
        if homing_start_request:
            wheel.homing_state = HomingState.SEARCH
            wheel.homing_elapsed_s = 0.0
        wheel.homing_last_sensor_active = sensor_active
        return False

    if wheel.homing_state == HomingState.SEARCH:
        wheel.homing_elapsed_s += PERIOD_S
        is_edge = sensor_active != wheel.homing_last_sensor_active
        is_initial_active = sensor_active and (wheel.homing_elapsed_s <= PERIOD_S + 1.0e-6)
        if is_edge or is_initial_active:
            wheel.homing_state = HomingState.EDGE_DETECTED
            wheel.homing_runtime_zero_offset_rad = wheel.homing_zero_offset_rad - raw_total_angle_rad
            wheel.homing_zero_valid = True
        elif wheel.homing_elapsed_s > wheel.homing_timeout_s:
            wheel.homing_state = HomingState.FAULT
        wheel.homing_last_sensor_active = sensor_active
        return False

    if wheel.homing_state == HomingState.EDGE_DETECTED:
        wheel.homing_state = HomingState.OFFSET_APPLY
        return False

    if wheel.homing_state == HomingState.OFFSET_APPLY:
        wheel.homing_state = HomingState.CONTINUOUS_ANGLE_READY
        return False

    if wheel.homing_state == HomingState.CONTINUOUS_ANGLE_READY:
        wheel.homing_state = HomingState.READY
        return True

    return wheel.homing_state == HomingState.READY


def test_disabled_homing_goes_ready() -> None:
    wheel = WheelState(homing_enabled=False, homing_gpio_port_present=False, homing_zero_offset_rad=0.42)
    ok = update_homing_state(wheel, homing_start_request=False, sensor_active=False, raw_total_angle_rad=0.0)
    assert ok is True
    assert wheel.homing_state == HomingState.READY
    assert wheel.homing_zero_valid is True
    assert abs(wheel.homing_runtime_zero_offset_rad - 0.42) < 1.0e-9


def test_idle_without_request_stays_idle() -> None:
    wheel = WheelState(homing_enabled=True, homing_gpio_port_present=True, homing_state=HomingState.IDLE)
    ok = update_homing_state(wheel, homing_start_request=False, sensor_active=True, raw_total_angle_rad=0.0)
    assert ok is False
    assert wheel.homing_state == HomingState.IDLE
    assert wheel.homing_last_sensor_active is True


def test_idle_with_request_enters_search_and_resets_elapsed() -> None:
    wheel = WheelState(
        homing_enabled=True,
        homing_gpio_port_present=True,
        homing_state=HomingState.IDLE,
        homing_elapsed_s=1.23,
    )
    ok = update_homing_state(wheel, homing_start_request=True, sensor_active=False, raw_total_angle_rad=0.0)
    assert ok is False
    assert wheel.homing_state == HomingState.SEARCH
    assert wheel.homing_elapsed_s == 0.0


def test_search_edge_detection_latches_runtime_offset() -> None:
    wheel = WheelState(
        homing_enabled=True,
        homing_gpio_port_present=True,
        homing_state=HomingState.SEARCH,
        homing_zero_offset_rad=0.5,
        homing_last_sensor_active=False,
    )
    ok = update_homing_state(wheel, homing_start_request=True, sensor_active=True, raw_total_angle_rad=0.2)
    assert ok is False
    assert wheel.homing_state == HomingState.EDGE_DETECTED
    assert wheel.homing_zero_valid is True
    assert abs(wheel.homing_runtime_zero_offset_rad - 0.3) < 1.0e-9


def test_search_initial_active_is_treated_as_detection() -> None:
    wheel = WheelState(
        homing_enabled=True,
        homing_gpio_port_present=True,
        homing_state=HomingState.SEARCH,
        homing_zero_offset_rad=1.0,
        homing_last_sensor_active=True,
    )
    ok = update_homing_state(wheel, homing_start_request=True, sensor_active=True, raw_total_angle_rad=0.25)
    assert ok is False
    assert wheel.homing_state == HomingState.EDGE_DETECTED
    assert abs(wheel.homing_runtime_zero_offset_rad - 0.75) < 1.0e-9


def test_search_timeout_enters_fault() -> None:
    wheel = WheelState(
        homing_enabled=True,
        homing_gpio_port_present=True,
        homing_state=HomingState.SEARCH,
        homing_timeout_s=0.0015,
        homing_elapsed_s=0.001,
        homing_last_sensor_active=False,
    )
    ok = update_homing_state(wheel, homing_start_request=True, sensor_active=False, raw_total_angle_rad=0.0)
    assert ok is False
    assert wheel.homing_state == HomingState.FAULT


def test_edge_detected_pipeline_reaches_ready() -> None:
    wheel = WheelState(
        homing_enabled=True,
        homing_gpio_port_present=True,
        homing_state=HomingState.EDGE_DETECTED,
        homing_zero_valid=True,
    )

    ok = update_homing_state(wheel, homing_start_request=True, sensor_active=True, raw_total_angle_rad=0.0)
    assert ok is False
    assert wheel.homing_state == HomingState.OFFSET_APPLY

    ok = update_homing_state(wheel, homing_start_request=True, sensor_active=True, raw_total_angle_rad=0.0)
    assert ok is False
    assert wheel.homing_state == HomingState.CONTINUOUS_ANGLE_READY

    ok = update_homing_state(wheel, homing_start_request=True, sensor_active=True, raw_total_angle_rad=0.0)
    assert ok is True
    assert wheel.homing_state == HomingState.READY


def test_ready_returns_true_and_fault_returns_false() -> None:
    ready_wheel = WheelState(homing_enabled=True, homing_gpio_port_present=True, homing_state=HomingState.READY)
    fault_wheel = WheelState(homing_enabled=True, homing_gpio_port_present=True, homing_state=HomingState.FAULT)
    assert update_homing_state(ready_wheel, homing_start_request=False, sensor_active=False, raw_total_angle_rad=0.0) is True
    assert update_homing_state(fault_wheel, homing_start_request=False, sensor_active=False, raw_total_angle_rad=0.0) is False


def run_all() -> None:
    test_disabled_homing_goes_ready()
    test_idle_without_request_stays_idle()
    test_idle_with_request_enters_search_and_resets_elapsed()
    test_search_edge_detection_latches_runtime_offset()
    test_search_initial_active_is_treated_as_detection()
    test_search_timeout_enters_fault()
    test_edge_detected_pipeline_reaches_ready()
    test_ready_returns_true_and_fault_returns_false()


def main() -> int:
    parser = make_mode_parser("green", default="green")
    parser.parse_args()
    run_all()
    print("green check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
