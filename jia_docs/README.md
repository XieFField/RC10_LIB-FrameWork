# jia_docs 说明

该目录用于 AI 协作过程资料归档，不参与产品代码编译。

## 目录约定

- `handoff/`：当前迭代交接文档（按年月分层）
- `history/`：已归档的稳定交接记录（按年月分层）
- `tests/`：AI 侧测试样例与脚本
- `artifacts/`：临时产物、参考材料、过程附件

## 最新交接入口

- RC10 最新交接文档：
  - [handoff/2026-05/ai_handoff_2026-05-25_2344_rc10_debug_merge_semantics_sync.md](handoff/2026-05/ai_handoff_2026-05-25_2344_rc10_debug_merge_semantics_sync.md)
  - [handoff/2026-05/ai_handoff_2026-05-24_2158_rc10_scurve_lock_yaw_context_sync.md](handoff/2026-05/ai_handoff_2026-05-24_2158_rc10_scurve_lock_yaw_context_sync.md)
  - [handoff/2026-05/ai_handoff_2026-05-23_2140_rc10_yaw_pid_vofa_trace.md](handoff/2026-05/ai_handoff_2026-05-23_2140_rc10_yaw_pid_vofa_trace.md)
  - [handoff/2026-05/ai_handoff_2026-05-23_0226_rc10_steer_fault_recovery_pid_guard.md](handoff/2026-05/ai_handoff_2026-05-23_0226_rc10_steer_fault_recovery_pid_guard.md)
  - [handoff/2026-05/ai_handoff_2026-05-21_1201_rc10_drive_gate_release_sync.md](handoff/2026-05/ai_handoff_2026-05-21_1201_rc10_drive_gate_release_sync.md)
  - [handoff/2026-05/ai_handoff_2026-05-22_0121_rc10_near_zero_suppression_refactor.md](handoff/2026-05/ai_handoff_2026-05-22_0121_rc10_near_zero_suppression_refactor.md)
  - [handoff/2026-05/ai_handoff_2026-05-22_1343_rc10_context_sync_after_fault_probe_revert.md](handoff/2026-05/ai_handoff_2026-05-22_1343_rc10_context_sync_after_fault_probe_revert.md)
- 交接索引：
  - [handoff/INDEX.md](handoff/INDEX.md)

## 本轮主题

- 当前文档主线已推进到：
  - `jia/develop` 已按 `jia/codex_2/3 -> jia/codex_1/3` 顺序完成两次 merge，当前入口文档对应的是这条合并后的统一底盘调试主线；
  - `mode30 / mode31 / mode32` 调试路由以 `jia/codex_2/3` 的模式语义为准，单轮隔离直控、full-gate 兼容入口和旧执行层直控入口已经收口到同一套 `DebugControl` 配置边界；
  - `DebugOutputFamily + JustFloatProfile + Binary` 输出族重构已并回当前主线，`text / justfloat / binary` 调试输出不再分叉；
  - `mode1` 手操 jerk 反向跨零修复已保留，反向换向时不再把目标加速度错误清零。
- 当前最新一轮同时补充了：
  - 当前稳定默认值继续保持主线口径：`manual_speed_profile_mode = kSCurve`，X-Park 阈值保持 `0.01 / 0.03`；
  - 舵向 fault gate、homing gate 和 `current = 0` 保护未因这轮合并回退；
  - 宿主测试入口仍以 `jia_docs/tests/tdd/chassis_semantics/run_test.ps1` 和 `run_pid_reconnect_test.ps1` 为主；
  - 新的 merge baseline handoff 会优先说明合并提交、语义归属、验证结果和后续联调关注点。

## 命名规则

- 新交接文档统一使用：`ai_handoff_YYYY-MM-DD_HHMM_<topic>.md`

## 维护约定

- 每次迭代进行中：文档先落在 `handoff/`
- 迭代稳定后：从 `handoff/` 迁移到 `history/` 并更新索引
- 默认不删除历史记录；若要瘦身，单独做按日期清理
