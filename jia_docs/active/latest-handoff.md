# 最新交接入口

当前最推荐优先阅读的 handoff 是：

- [../handoff/2026-06/ai_handoff_2026-06-24_rc10_homing_auto_retry_xpark_priority_brake_debug_sync.md](../handoff/2026-06/ai_handoff_2026-06-24_rc10_homing_auto_retry_xpark_priority_brake_debug_sync.md)

这份文档承接了前一轮 chassis 主线收口，重点补齐：

- `homing auto-retry`
- `X-Park priority brake`
- 底盘 debug mode 接入
- 当前 host 验证结果更新

## 与上一轮 handoff 的关系

继续向前追溯时，上一个主线 handoff 仍然是：

- [../handoff/2026-06/ai_handoff_2026-06-09_rc10_path_yaw_homing_build_sync.md](../handoff/2026-06/ai_handoff_2026-06-09_rc10_path_yaw_homing_build_sync.md)

再上一个主线收口 handoff 是：

- [../handoff/2026-06/ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md](../handoff/2026-06/ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md)

更早的 merge / payload 背景仍可参考：

- [../handoff/2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md](../handoff/2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md)
- [../handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md](../handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md)

如果只读一份，先读 2026-06-24 这份；如果要继续追 `path / yaw / homing` 主线背景，再补 2026-06-09 这份。
