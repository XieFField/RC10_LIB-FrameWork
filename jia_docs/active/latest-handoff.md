# 最新交接入口

当前最推荐优先阅读的 handoff 是：

- [../handoff/2026-06/ai_handoff_2026-06-24_rc10_homing_search_diagonal_reverse_sync.md](../handoff/2026-06/ai_handoff_2026-06-24_rc10_homing_search_diagonal_reverse_sync.md)

这份文档承接了前一轮 chassis 主线收口，重点补齐：

- 光电门回零 `HomingState::kSearch` 搜索阶段的编译期固定方向表 `JIA_CHASSIS_HOMING_SEARCH_RPM_SIGN[4]`
- 默认 `0 + 2` 对角轮反转，`1 + 3` 保持同向
- 该方向分裂只影响光电门回零搜索下发 RPM，不改正常运动、recovery re-home、zero-stop、X-Park 和其他模式
- 当前 host 验证结果更新为 `211/211 passed`

## 与上一轮 handoff 的关系

继续向前追溯时，上一个主线 handoff 仍然是：

- [../handoff/2026-06/ai_handoff_2026-06-24_rc10_homing_auto_retry_xpark_priority_brake_debug_sync.md](../handoff/2026-06/ai_handoff_2026-06-24_rc10_homing_auto_retry_xpark_priority_brake_debug_sync.md)

再往前追 `path / yaw / homing` 主线背景时，参考：

- [../handoff/2026-06/ai_handoff_2026-06-09_rc10_path_yaw_homing_build_sync.md](../handoff/2026-06/ai_handoff_2026-06-09_rc10_path_yaw_homing_build_sync.md)

再上一个主线收口 handoff 是：

- [../handoff/2026-06/ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md](../handoff/2026-06/ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md)

更早的 merge / payload 背景仍可参考：

- [../handoff/2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md](../handoff/2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md)
- [../handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md](../handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md)

如果只读一份，先读 `rc10_homing_search_diagonal_reverse_sync` 这份；如果要继续追 auto-retry / X-Park priority brake / debug 接入背景，再补同日上一份 handoff。
