# 最新交接入口

当前最推荐优先阅读的 handoff 是：

- [../handoff/2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md](../handoff/2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md)

这份文档是 `2026-05-31` 三路 `jia/wait/*` merge 收口后的主线交接入口，重点说明：

- `jia/wait/codex_1/7`、`jia/wait/codex_7/1`、`jia/wait/codex_6/1` 的最终语义归属
- zero-stop 刹车观测、X-Park 死区零电流策略开关、静止舵轮 fault latch / rehome 恢复链路
- 当前推荐验证命令与三条宿主测试入口通过状态

## 与 2026-05-26 handoff 的关系

继续向前追溯时，上一份主线级 merge handoff 是：

- [../handoff/2026-05/ai_handoff_2026-05-26_rc10_drive_pid_load_tune_merge.md](../handoff/2026-05/ai_handoff_2026-05-26_rc10_drive_pid_load_tune_merge.md)

同主题仍有一份 payload 补充说明：

- [../handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md](../handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md)

两者关系如下：

- `ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md`
  - 是当前最新主线级 handoff，优先级最高
  - 用于接手三路 wait 分支 merge 后的最终语义与验证状态
- `ai_handoff_2026-05-26_rc10_drive_pid_load_tune_merge.md`
  - 是上一阶段主线 merge handoff
  - 用于理解 zero-stop / drive-load merge 之前的收口背景
- `ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md`
  - 是单点协议语义补充
  - 主要解释 `SingleWheelTrace` 的 payload 分类与 9 通道判定风险

如果只读一份，请先读 merge handoff；如果要处理上位机解析、VOFA 侧脚本或 trace 协议问题，再补读 payload 说明。

## 继续向前追溯时的前一站

若需要理解这次三路 merge 之前的主线基线入口，可继续阅读：

- [../handoff/2026-05/ai_handoff_2026-05-25_2344_rc10_debug_merge_semantics_sync.md](../handoff/2026-05/ai_handoff_2026-05-25_2344_rc10_debug_merge_semantics_sync.md)
