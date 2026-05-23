# jia_docs 说明

该目录用于 AI 协作过程资料归档，不参与产品代码编译。

## 目录约定

- `handoff/`：当前迭代交接文档（按年月分层）
- `history/`：已归档的稳定交接记录（按年月分层）
- `tests/`：AI 侧测试样例与脚本
- `artifacts/`：临时产物、参考材料、过程附件

## 最新交接入口

- RC10 最新交接文档：
  - [handoff/2026-05/ai_handoff_2026-05-23_0226_rc10_steer_fault_recovery_pid_guard.md](handoff/2026-05/ai_handoff_2026-05-23_0226_rc10_steer_fault_recovery_pid_guard.md)
  - [handoff/2026-05/ai_handoff_2026-05-21_1201_rc10_drive_gate_release_sync.md](handoff/2026-05/ai_handoff_2026-05-21_1201_rc10_drive_gate_release_sync.md)
  - [handoff/2026-05/ai_handoff_2026-05-22_0121_rc10_near_zero_suppression_refactor.md](handoff/2026-05/ai_handoff_2026-05-22_0121_rc10_near_zero_suppression_refactor.md)
  - [handoff/2026-05/ai_handoff_2026-05-22_1343_rc10_context_sync_after_fault_probe_revert.md](handoff/2026-05/ai_handoff_2026-05-22_1343_rc10_context_sync_after_fault_probe_revert.md)
- 交接索引：
  - [handoff/INDEX.md](handoff/INDEX.md)

## 本轮主题

- 当前文档主线已推进到：
  - 舵向断链故障检测闭环；
  - 故障锁存后整车 drive 全停；
  - 故障恢复后仅故障轮重新 homing；
  - 锁故障即清舵向闭环状态，避免重连首拍吃到残留 PID 输出。
- 当前最新一轮同时补充了：
  - 宿主测试对 photogate / 舵向电流反馈的可控桩；
  - 断链锁存阶段纯 `current=0` 语义修复；
  - 独立 PID reconnect 风险证据化；
  - 下一位 agent 的继续排查建议。

## 命名规则

- 新交接文档统一使用：`ai_handoff_YYYY-MM-DD_HHMM_<topic>.md`

## 维护约定

- 每次迭代进行中：文档先落在 `handoff/`
- 迭代稳定后：从 `handoff/` 迁移到 `history/` 并更新索引
- 默认不删除历史记录；若要瘦身，单独做按日期清理
