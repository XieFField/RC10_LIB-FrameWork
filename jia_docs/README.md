# jia_docs 说明

该目录用于 AI 协作过程资料归档，不参与产品代码编译。

## 目录约定

- `handoff/`：当前迭代交接文档（按年月分层）
- `history/`：已归档的稳定交接记录（按年月分层）
- `tests/`：AI 侧测试样例与脚本
- `artifacts/`：临时产物、参考材料、过程附件

## 最新交接入口

- RC10 最新交接文档：
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
  - 舵向断链故障恢复闭环已经落地并进入稳定上下文；
  - yaw 位置环 VOFA `JustFloat` 调试链路已补齐，便于 `LockTo` / `LockNow` 联调；
  - 手操主链路已切入 S 形速度规划第二版；
  - `LockToYaw -> LockNowYaw` 锁角连续性与默认调试参数已同步收口。
- 当前最新一轮同时补充了：
  - jerk 受限手操速度成形、提前制动与近目标贴合语义；
  - `reverse_intent`、低速抑制与默认调试入口的当前联调参数；
  - `LockToYaw` 切回 `LockNowYaw` 时的锁角继承修复与宿主回归；
  - S 形手操速度规划重新并回舵向故障恢复主线后的上下文同步。

## 命名规则

- 新交接文档统一使用：`ai_handoff_YYYY-MM-DD_HHMM_<topic>.md`

## 维护约定

- 每次迭代进行中：文档先落在 `handoff/`
- 迭代稳定后：从 `handoff/` 迁移到 `history/` 并更新索引
- 默认不删除历史记录；若要瘦身，单独做按日期清理
