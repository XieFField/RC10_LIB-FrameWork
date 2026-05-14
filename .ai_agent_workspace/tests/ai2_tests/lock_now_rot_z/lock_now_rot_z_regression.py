from __future__ import annotations

import argparse
from dataclasses import dataclass


LOCK_NOW_ROT_Z_SHIFT_TIME_MS = 3
ROT_Z_PID_PERIOD = 1


@dataclass
class LockNowState:
    input_target_rot_z: float = 0.0
    target_rot_z: float = 0.0
    input_hwt_rot_z: float = 0.0
    planned_omega_z: float = 0.0
    lock_now_rot_z_shift_count: int = 0
    rot_z_pid_count: int = 0
    lock_now_rot_z_target: float = 0.0


def old_is_lock_now_rot_z(state: LockNowState, omega_z: float) -> tuple[float, float]:
    rot_z = state.target_rot_z
    if omega_z == 0.0:
        if state.lock_now_rot_z_shift_count > 0:
            state.lock_now_rot_z_shift_count -= 1
            out_rot_z = state.input_hwt_rot_z
            out_omega_z = 0.0
        else:
            out_rot_z = rot_z
            if state.rot_z_pid_count >= ROT_Z_PID_PERIOD:
                state.rot_z_pid_count = 0
                out_omega_z = rot_z - state.input_hwt_rot_z
            else:
                out_omega_z = state.planned_omega_z
            state.rot_z_pid_count += 1
    else:
        out_rot_z = state.input_hwt_rot_z
        out_omega_z = omega_z
        state.lock_now_rot_z_shift_count = LOCK_NOW_ROT_Z_SHIFT_TIME_MS
    state.target_rot_z = out_rot_z
    state.planned_omega_z = out_omega_z
    return out_rot_z, out_omega_z


def fixed_is_lock_now_rot_z(state: LockNowState, omega_z: float) -> tuple[float, float]:
    if omega_z == 0.0:
        if state.lock_now_rot_z_shift_count > 0:
            state.lock_now_rot_z_shift_count -= 1
            state.lock_now_rot_z_target = state.input_hwt_rot_z
            out_rot_z = state.lock_now_rot_z_target
            out_omega_z = 0.0
        else:
            out_rot_z = state.lock_now_rot_z_target
            if state.rot_z_pid_count >= ROT_Z_PID_PERIOD:
                state.rot_z_pid_count = 0
                out_omega_z = state.lock_now_rot_z_target - state.input_hwt_rot_z
            else:
                out_omega_z = state.planned_omega_z
            state.rot_z_pid_count += 1
    else:
        state.lock_now_rot_z_target = state.input_hwt_rot_z
        out_rot_z = state.lock_now_rot_z_target
        out_omega_z = omega_z
        state.lock_now_rot_z_shift_count = LOCK_NOW_ROT_Z_SHIFT_TIME_MS
    state.target_rot_z = out_rot_z
    state.planned_omega_z = out_omega_z
    return out_rot_z, out_omega_z


def simulate(step_fn) -> tuple[float, float]:
    state = LockNowState()

    # runThread() 每周期都会先把 target_data_.rot_z 重置成 input_target_data_.rot_z
    # 这里用这个赋值来复现四舵轮当前的数据流。
    def begin_cycle():
        state.target_rot_z = state.input_target_rot_z

    # 用户正在主动旋转，IMU 航向已经转到 1.2 rad。
    state.input_hwt_rot_z = 1.2
    begin_cycle()
    step_fn(state, omega_z=0.8)

    # 用户松手后，缓冲期内继续读到 1.2 rad 的真实当前角。
    for _ in range(LOCK_NOW_ROT_Z_SHIFT_TIME_MS):
        state.input_hwt_rot_z = 1.2
        begin_cycle()
        step_fn(state, omega_z=0.0)

    # 缓冲结束后的第一个 PID 周期：如果没有持久保存当前角，就会回退到旧 rot_z=0。
    state.input_hwt_rot_z = 1.2
    begin_cycle()
    return step_fn(state, omega_z=0.0)


def run_current_bug_demo() -> None:
    out_rot_z, _ = simulate(old_is_lock_now_rot_z)
    assert abs(out_rot_z - 1.2) < 1.0e-6, (
        "当前实现应当在松手后继续锁住最近一次真实 IMU 角 1.2 rad，"
        f"但实际 out_rot_z 回退成了 {out_rot_z:.6f} rad。"
    )


def run_fixed_behavior_check() -> None:
    out_rot_z, _ = simulate(fixed_is_lock_now_rot_z)
    assert abs(out_rot_z - 1.2) < 1.0e-6, (
        f"修复后 out_rot_z 应保持为最近一次真实 IMU 角 1.2 rad，实际得到 {out_rot_z:.6f} rad。"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        choices=("red", "green"),
        default="red",
        help="red: 复现当前缺口；green: 校验修复后的目标行为",
    )
    args = parser.parse_args()

    if args.mode == "red":
        run_current_bug_demo()
    else:
        run_fixed_behavior_check()

    print(f"{args.mode} check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
