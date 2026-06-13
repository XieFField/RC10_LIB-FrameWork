# 当前主线总览

本页回答当前 `jia_docs` 接手时最重要的三个问题：现在主线是什么、先读什么、先跑什么。

## 当前主线状态

截至 `2026-06-09`，`jia/develop_1` 在 `2026-06-05` doctest / zero-stop / X-Park / RUNTIME_MIN 收口之后，又合入了 `AAA-Path` 路径规划线，并完成 yaw lock、homing、调试开关和 MDK 编译恢复的一轮稳定化。当前 RC10 调试主线可以概括为：

- `mode30 / mode31 / mode32` 的调试路径沿用 2026-05-31 merge 后的统一语义。
- `AAA-Path` 路径规划正式版已合入，重点改动集中在 `APP_Path.*`、`APP_Speedplanner.*` 与 `omni_chassisSetup.*`。
- `drive -> VESC_Motor*` 句柄收口、drive PID 调参与 VESC 本地 PID 速度环启用仍是当前 drive 调试基础。
- `SingleWheelTrace` payload 保持 `steer_only`、`drive_only`、`steer_and_drive` 三类口径。
- `chassis_semantics` 宿主测试已经接入 doctest，并拆成 runner、共享 harness 与 7 个行为域分片。
- zero-stop 当前按“目标门控进入/退出 + active 期间 residual 收尾”两层职责理解。
- X-Park 静止保持锁存后退出主要看 target / command exit 门；`debug9 / kSteerAngleAndDriveSpeedMode` 会显式释放 X-Park pose 与 steer hold 覆盖。
- `chassis.h` 顶部定义 `RUNTIME_MIN` / `FULL_DEBUG` 编译档位；默认固件为 `RUNTIME_MIN`，host 语义测试显式使用 `FULL_DEBUG`。
- yaw lock 已修复跨模式旧锁角复用、目标规划/PID 输入、目标限速和减速阶段先刹再锁语义。
- 光电 homing 三边沿确认误校准已修复，并由 host harness 与 `steer_fault_homing_recovery` 分片覆盖。
- 调试总开关和单轮 UART8 trace 编译开关已收口；当前最新固件状态按 `759fbf57` 关闭调试模式后理解。
- `02dbf00a` 已清理合并残留并恢复 MDK 编译，是当前最新保存点。

## 推荐阅读顺序

1. [latest-handoff.md](latest-handoff.md)
2. [onramp.md](onramp.md)
3. [../tests/README.md](../tests/README.md)
4. [../handoff/INDEX.md](../handoff/INDEX.md)

## 当前推荐验证入口

优先执行统一入口：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/run_tests.ps1
```

如果只需要细跑当前底盘宿主语义套件：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_main.ps1
```

如果只需要确认瘦身固件档：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_slim_smoke.ps1
```

如需兼容性确认，再按需运行旧路径包装入口或 VESC 刹车专项：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/tdd/chassis_semantics/run_test.ps1
powershell -ExecutionPolicy Bypass -File jia_docs/tests/tdd/chassis_semantics/run_pid_reconnect_test.ps1
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/vesc_brake/run_test.ps1
```

## 已知基线情况

- `jia_docs/tests/run_tests.ps1`：当前统一入口，应作为继续开发时的默认验证路径。
- `jia_docs/tests/host_cpp/chassis_semantics/run_main.ps1`：当前 chassis doctest 主入口，使用 `FULL_DEBUG` 编译 runner、harness 与行为域分片。
- `jia_docs/tests/host_cpp/chassis_semantics/run_slim_smoke.ps1`：`RUNTIME_MIN` 极限运行固件档 smoke，验证可编译、可实例化和 `Chassis` 对象体积下降。
- `jia_docs/tests/tdd/chassis_semantics/run_test.ps1`：旧路径兼容入口，转发到当前 chassis 主入口，不再作为新增测试首选位置。
- `jia_docs/tests/host_cpp/swerve_core/run_test.ps1`：历史对照入口，当前因生产源文件缺失登记为 known xfail。

## 接下来继续开发时的重点

- 若继续底盘联调，优先围绕 `mode30` 单轮 RPM + `VESC_RPM_CONTROL_PID_CURRENT` 组合验证。
- 若继续路径规划，优先围绕 `APP_Path.*`、`APP_Speedplanner.*`、`omni_chassisSetup.*` 做 MDK 与实车路径复核。
- 若继续 yaw lock / zero-stop / X-Park 语义侧开发，优先看 2026-06-09 handoff、2026-06-05 handoff 与 `yaw_and_motion_profile`、`drive_delivery_zero_stop`、`xpark_gate_and_hold` 分片。
- 若继续 homing / 舵向恢复链路，优先看 `steer_fault_homing_recovery` 分片和三边沿确认修复。
- 若继续协议或上位机语义，优先核对 `SingleWheelTrace` 与 drive-load trace 的通道解释。
- 若继续固件瘦身，保持 `RUNTIME_MIN` 默认档与 `FULL_DEBUG` host 语义套件的分层，不要把调试回归挪进 slim smoke。
