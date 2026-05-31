# 当前主线总览

本页用于快速回答三个问题：

1. 当前 `jia/develop` 主线已经并入了什么。
2. 现在继续开发时优先关注什么。
3. 先跑哪些验证脚本能够最快确认上下文。

## 当前主线状态

截至 `2026-05-26`，`jia/develop` 已完成 `jia/archive/codex_2/6` 与 `jia/archive/codex_1/6` 的合并收口，当前 RC10 底盘调试主线可以概括为：

- `mode30 / mode31 / mode32` 的调试路由沿用最新统一语义。
- `DebugOutputFamily + JustFloatProfile + Binary` 输出族已经并回同一主线，不再分叉维护。
- 手操速度规划默认仍保持 `manual_speed_profile_mode = kSCurve`。
- `X-Park`、`fault gate`、`homing gate`、`current = 0` 保护语义保持有效。

## 2026-05-26 这一轮并入的关键变化

### 1. drive 轮句柄与调参边界收口

- `drive_motor_h` 已明确收窄为 `VESC_Motor*`，不再把 drive 侧 PID 运行时调参能力扩散回 `Motor_Base`。
- drive 轮共享一套 PID 调参缓存，支持统一回读、统一 apply、统一对齐 `applied_stamp`。
- enable 上升沿的 runtime readback 与 apply 安全边界已经固定到当前主线。

### 2. VESC 本地 PID 速度环默认启用

- `VESC_Motor` 已具备本地 PID 速度环电流下发模式。
- 当前工程初始化层默认让四个 drive 轮进入本地 PID 速度闭环，便于直接上车联调。
- 当前主线语义以“`raw pid output + bias = total current`”为准，非对应控制模式下 `raw/total` 观测会主动清零。

### 3. SingleWheelTrace payload 语义扩展

- `JustFloatProfile::kSingleWheelTrace` 现在包含：
  - `kSteerOnly`
  - `kDriveOnly`
  - `kSteerAndDrive`
- 其中 `kSteerOnly` 与 `kDriveOnly` 都可能是 9 通道，接收端不能再只靠长度判断语义。

### 4. APP_Utils 数学入口收口

- `APP_Utils` 的 DSP / std 数学后端切换已统一收口到公共头与公共 API。
- chassis 热路径已切到统一数学入口，减少内部重复包装和重复计算。
- 对应 host 回归测试已补到 `app_utils backend` 与 `app_utils math` 两组。

### 5. drive PID 第二步虚拟负载整定链路并入

- `mode30` 单轮隔离调试路径已经接入 drive RPM 自动阶跃与虚拟负载整定能力。
- 新增 `JustFloatProfile::kDrivePidLoadTune`，提供 15 通道观测输出。
- 虚拟负载 bias 仅在单轮隔离、RPM 指令、目标 drive 为 `VESC_RPM_CONTROL_PID_CURRENT` 且未被保护逻辑拦截时注入，其余路径会主动清零 bias。

## 推荐阅读顺序

1. [latest-handoff.md](latest-handoff.md)
2. [onramp.md](onramp.md)
3. [../handoff/INDEX.md](../handoff/INDEX.md)

## 当前推荐验证入口

优先执行以下两个脚本，确认当前主线调试语义与 host 回归入口：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/tdd/chassis_semantics/run_test.ps1
powershell -ExecutionPolicy Bypass -File jia_docs/tests/tdd/chassis_semantics/run_pid_reconnect_test.ps1
```

如果需要确认 VESC 本地 PID 速度环相关回归，再执行：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/legacy_ai_tests/vesc_brake/run_test.ps1
```

## 当前已知基线情况

- `run_pid_reconnect_test.ps1`：最新 handoff 记录为 `PASS`
- `legacy_ai_tests/vesc_brake/run_test.ps1`：最新 handoff 记录为 `PASS`
- `chassis_semantics/run_test.ps1`：最新 merge handoff 记录仍有 `18` 个 baseline 失败
  - 这些失败在该轮 merge 前的 `ORIG_HEAD` 基线上已存在
  - 该轮新增的共享 PID、load tune、step generator、trace 相关测试已通过

## 接下来继续开发时的重点

- 若继续底盘联调，优先围绕 `mode30` 单轮 RPM + `VESC_RPM_CONTROL_PID_CURRENT` 组合验证。
- 若继续协议侧开发，优先确认 `SingleWheelTrace` 9 通道分流不再按长度硬判。
- 若要继续清理 host baseline 失败，建议单开任务，不与当前 merge 语义整理混做。
