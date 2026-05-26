# 最新交接入口

当前最推荐优先阅读的 handoff 是：

- [../handoff/2026-05/ai_handoff_2026-05-26_rc10_drive_pid_load_tune_merge.md](../handoff/2026-05/ai_handoff_2026-05-26_rc10_drive_pid_load_tune_merge.md)

这份文档是 `2026-05-26` RC10 `/6` 合并收口后的主线交接入口，重点说明：

- `jia/archive/codex_2/6` 与 `jia/archive/codex_1/6` 合并后的最终语义归属
- drive PID 共享调参、VESC 本地 PID 速度环、虚拟负载整定与 15 通道 trace 输出
- 当前推荐验证命令与 baseline 失败现状

## 2026-05-26 两份 handoff 的关系

同一天还有一份补充说明文档：

- [../handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md](../handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md)

两者关系如下：

- `ai_handoff_2026-05-26_rc10_drive_pid_load_tune_merge.md`
  - 是当前主线级 handoff，优先级最高
  - 用于接手 RC10 底盘调试 merge 语义与验证状态
- `ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md`
  - 是单点协议语义补充
  - 主要解释 `SingleWheelTrace` 的 payload 分类与 9 通道判定风险

如果只读一份，请先读 merge handoff；如果要处理上位机解析、VOFA 侧脚本或 trace 协议问题，再补读 payload 说明。

## 继续向前追溯时的前一站

若需要理解 `/6` 合并前的基线入口，可继续阅读：

- [../handoff/2026-05/ai_handoff_2026-05-25_2344_rc10_debug_merge_semantics_sync.md](../handoff/2026-05/ai_handoff_2026-05-25_2344_rc10_debug_merge_semantics_sync.md)
