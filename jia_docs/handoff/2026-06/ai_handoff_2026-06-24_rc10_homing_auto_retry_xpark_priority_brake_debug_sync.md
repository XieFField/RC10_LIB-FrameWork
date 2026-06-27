# AI handoff 2026-06-24 RC10 homing auto-retry / X-Park priority brake / debug sync

本文档用于承接已完成并已验证的 chassis 交接层更新。它不是控制逻辑继续迭代说明，而是把已经落地的控制语义、验证结果、边界和后续接手顺序整理成当前默认入口。

## 本轮目标

本轮目标是文档收口，不是继续扩展控制功能：

- 将已完成的 `homing auto-retry`
- 将已完成的 `X-Park priority brake`
- 将已完成的底盘 debug mode 接入

正式沉入 `jia_docs` 的交接层与入口层，便于下一位接手者直接续接。

## 已完成的代码事实

### 1. 单舵轮 homing auto-retry

已完成的核心语义如下：

- 只重试失败单舵轮
- 由宏控制 `enable / max attempts / interval`
- 达到最大次数后停在 `Fault`
- `recovery re-home` 失败也走同一条单舵轮重试链
- 不扩展到其他协议或其他功能

现有宏：

- `JIA_CHASSIS_HOMING_AUTO_RETRY_ENABLE = 1`
- `JIA_CHASSIS_HOMING_AUTO_RETRY_MAX_ATTEMPTS = 5U`
- `JIA_CHASSIS_HOMING_AUTO_RETRY_INTERVAL_MS = 1000U`

现有状态字段：

- `homing_auto_retry_attempt_count`
- `homing_auto_retry_wait_active`
- `homing_auto_retry_wait_elapsed_ms`
- `homing_auto_retry_armed_by_recovery_failure`

debug mirror 字段：

- `homing_auto_retry_attempt_count[4]`
- `homing_auto_retry_wait_active[4]`
- `homing_auto_retry_wait_elapsed_ms[4]`
- `homing_auto_retry_armed_by_recovery_failure[4]`

语义修正已完成：

- auto-retry 不会误伤 steer fault latched 恢复链
- 只有普通 homing fault，或 recovery re-home 失败，才进入 auto-retry
- `Recovering + rehome_request` 的立即进 `Search` 分支已加条件，避免重复走 `kFault` 分支里的重试计数
- `latchSteerFault()` 里 `homing_auto_retry_armed_by_recovery_failure` 只在进入 latch 前处于 `SteerFaultState::kRecovering` 时置真

### 2. X-Park priority brake

已新增公开接口：

- `setSpeed_LockNowYaw_XParkBrake(...)`
- `setSpeed_LockToYaw_XParkBrake(...)`

最终行为：

- 非零运动阶段继续沿用原 `LockNowYaw / LockToYaw` 的 yaw 语义
- 静止收尾阶段改走更早进入 `X-Park` 的 residual 门限
- `zero-stop` 释放后不再恢复 yaw lock，而是直接进入 `X-Park`
- 若 residual 一开始就满足门限，可直接进 `X-Park`

产品决策已明确：

- `XParkPriorityBrakeConfig` 不再保留 `enable` 开关
- 是否启用由“是否调用新接口 / 是否处于新 mode”决定

### 3. 底盘 debug mode 接入

已新增 4 个 debug mode：

- `kBodyLockNowXParkBrake`
- `kWorldLockNowXParkBrake`
- `kBodyLockToXParkBrake`
- `kWorldLockToXParkBrake`

已完成接线：

- `resolveDebugMode()` 已补 raw mode 解析
- `applyDebugTargetOverride()` 已补到 `*_XParkBrake` 目标接口的路由
- `refreshDebugMirror()` 已发布：
  - `xpark_priority_brake_mode_active`
  - `xpark_priority_brake_threshold_active`
  - `xpark_priority_brake_skip_yaw_reengage`
  - `xpark_priority_brake_residual_enter_m_s`
  - `xpark_priority_brake_residual_exit_m_s`
  - `xpark_priority_brake_entry_delay_ms`

边界保持不变：

- 不扩展串口协议
- 只补底盘内部 debug mode 和 mirror 观测链

## 已验证结果

当前已确认的 host 结果：

- `powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_main.ps1`
  - `210/210 passed`
- `powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_slim_smoke.ps1`
  - `JIA_CHASSIS_PROFILE=1`
  - `sizeof(Chassis)=2856`

这组数字是当前文档层基线，后续不再沿用旧的 `199 passed` / `2736`。

## 代码边界

- 这轮不是继续扩控逻辑
- 这轮不新增串口协议或其他外部功能
- 这轮不把 host 结果当作 Keil / MDK / 实车的最终替代

## 仍未覆盖的风险

- host 通过，不代表 Keil / MDK / 实车已经覆盖
- `homing auto-retry`、`X-Park priority brake`、`debug mode` 仍需要板上 / 工程侧继续确认
- `500U`、`100.0f` 仍是当前代码事实，不是最终板上最优值的证明

## 推荐接手顺序

1. 先读 `jia_docs/active/overview.md`
2. 再读本文件
3. 再读 `jia_docs/active/onramp.md`
4. 需要更完整背景时再看 `jia_docs/handoff/2026-06/ai_handoff_2026-06-09_rc10_path_yaw_homing_build_sync.md`

## 下一步关注点

- Keil / MDK 编译一致性
- 板上 `first_boot_homing_delay_` 与 `homing_search_rpm = 100.0f` 的实际手感确认
- `mode30` 单舵轮 debug 与整车 debug deadzone 的板上观测
- `X-Park priority brake` 在板上的刹停和姿态稳定感

## 提交锚点

本轮交接锚点：`3a913c3713c1aab3f96c3e4e25e2b9abb704eb61`
