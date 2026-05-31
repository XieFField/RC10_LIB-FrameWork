# 当前主线总览

本页用来快速回答三个问题：

1. 当前 `jia/develop` 主线已经并入了什么
2. 继续开发时优先关注什么
3. 先跑哪些验证入口最能确认上下文

## 当前主线状态

截至 `2026-05-31`，`jia/develop` 已完成 `jia/wait/codex_1/7`、`jia/wait/codex_7/1`、`jia/wait/codex_6/1` 的合并收口。当前 RC10 调试主线可以概括为：

- `mode30 / mode31 / mode32` 的调试路径已沿用最新统一语义
- `drive -> VESC_Motor*` 句柄收口已经完成，drive PID 调参与上层 `Motor_Base` 解耦
- VESC 本地 PID 速度环默认在工程初始化层启用，方便直接上车联调
- `SingleWheelTrace` payload 已扩展为 `steer_only`、`drive_only`、`steer_and_drive`
- `APP_Utils` 数学入口已统一收口到公共 API
- drive PID 第二步虚拟负载整定链路已并入主线
- 当前主机侧入口在 2026-05-31 handoff 中记录为 `PASS`

## 推荐阅读顺序

1. [latest-handoff.md](latest-handoff.md)
2. [onramp.md](onramp.md)
3. [../handoff/INDEX.md](../handoff/INDEX.md)

## 当前推荐验证入口

优先执行统一入口：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/run_tests.ps1
```

如果你在做兼容性确认，再单独跑这些旧入口：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_test.ps1
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_pid_reconnect_test.ps1
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/vesc_brake/run_test.ps1
```

## 已知基线情况

- `jia_docs/tests/run_tests.ps1`：统一入口，应该成为默认执行路径
- `host_cpp/chassis_semantics/run_test.ps1`：2026-05-31 handoff 记录为 `PASS`
- `host_cpp/chassis_semantics/run_pid_reconnect_test.ps1`：2026-05-31 handoff 记录为 `PASS`
- `host_cpp/vesc_brake/run_test.ps1`：2026-05-31 handoff 记录为 `PASS`

## 接下来继续开发时的重点

- 若继续底盘联调，优先围绕 `mode30` 单轮 RPM + `VESC_RPM_CONTROL_PID_CURRENT` 组合验证
- 若继续协议或语义侧开发，优先核对 `SingleWheelTrace` 的 9 通道/多通道分流是否仍符合预期
- 若继续清理 host baseline，建议单独开任务，不要与当前主线 merge 语义混在一起
