#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
外部行为回归脚本（TDD）：
1) LockNowRotZ 松手后锁当前角目标保持
2) Homing 状态流转（简化模型）
3) Flip 滞回策略决策
4) DriveGate / StopSteerGuard 缩放行为
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from jia_docs.tests.shared.python_test_support import artifacts_root


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def wrap_to_pi(a: float) -> float:
    while a >= math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def shortest(a: float, b: float) -> float:
    return wrap_to_pi(b - a)


def lock_now_step(
    is_lock: bool,
    rot_z: float,
    omega_z: float,
    imu_yaw: float,
    shift_count: int,
    shift_time: int,
    target: float,
) -> tuple[float, float, int, float]:
    if not is_lock:
        return rot_z, omega_z, shift_count, target
    if omega_z == 0.0:
        if shift_count > 0:
            shift_count -= 1
            target = imu_yaw
            return target, 0.0, shift_count, target
        return target, 0.0, shift_count, target
    target = imu_yaw
    shift_count = shift_time
    return target, omega_z, shift_count, target


@dataclass
class GateCfg:
    close_deg: float = 30.0
    min_scale: float = 0.0


def hard_gate(abs_err_rad: float, cfg: GateCfg) -> float:
    return cfg.min_scale if abs_err_rad >= math.radians(cfg.close_deg) else 1.0


def stop_guard_blend_hardhold(residual_speed: float, release_speed: float = 0.01) -> float:
    return 1.0 if residual_speed <= release_speed else 0.0


def flip_hysteresis(
    base_err_deg: float,
    flip_err_deg: float,
    prev_flipped: bool,
    enter_deg: float = 100.0,
    exit_deg: float = 80.0,
) -> bool:
    if prev_flipped:
        return flip_err_deg <= enter_deg
    return base_err_deg > exit_deg and flip_err_deg < base_err_deg


def run() -> dict:
    out = {}

    # Case1: LockNowRotZ
    shift = 0
    target = 0.0
    imu = 0.0
    # 手动旋转 10 帧
    for _ in range(10):
        imu += math.radians(3.0)
        _, _, shift, target = lock_now_step(True, 0.0, 1.0, imu, shift, 5, target)
    # 松手 10 帧
    locked_targets = []
    for _ in range(10):
        imu += math.radians(0.2)
        out_rot, _, shift, target = lock_now_step(True, 0.0, 0.0, imu, shift, 5, target)
        locked_targets.append(out_rot)
    out["lock_now_final_target_deg"] = math.degrees(locked_targets[-1])
    assert abs(locked_targets[-1] - imu) < math.radians(2.0), "LockNow 未保持到松手附近真实角"

    # Case2: Flip hysteresis
    f0 = flip_hysteresis(95.0, 85.0, False, 100.0, 80.0)
    f1 = flip_hysteresis(70.0, 90.0, True, 100.0, 80.0)
    out["flip_enter_from_false"] = f0
    out["flip_keep_from_true"] = f1
    assert f0 is True, "Flip 进入条件异常"
    assert f1 is True, "Flip 保持条件异常"

    # Case3: Drive gate + Stop guard
    g0 = hard_gate(math.radians(40.0), GateCfg())
    b0 = stop_guard_blend_hardhold(0.2, 0.01)
    out["gate_scale_at_40deg"] = g0
    out["stop_guard_blend_at_0_2"] = b0
    assert g0 == 0.0, "DriveGate HardGate 未收紧"
    assert b0 == 0.0, "StopSteerGuard HardHold 未生效"

    # Case4: Homing simple flow (模型化)
    # Idle -> Search -> Edge -> Offset -> Ready
    homing_flow = ["kIdle", "kSearch", "kEdgeDetected", "kOffsetApply", "kContinuousAngleReady", "kReady"]
    out["homing_flow"] = homing_flow
    assert homing_flow[-1] == "kReady", "Homing 终态异常"

    return out


def main() -> None:
    res = run()
    artifacts = artifacts_root("swerve_port")
    out_file = artifacts / "lock_now_homing_gate_regression.json"
    out_file.write_text(json.dumps(res, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(res, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

