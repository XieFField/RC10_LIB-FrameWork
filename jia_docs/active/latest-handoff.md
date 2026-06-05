# 最新交接入口

当前最推荐优先阅读的 handoff 是：

- [../handoff/2026-06/ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md](../handoff/2026-06/ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md)

这份文档接在 `2026-05-31` 三路 `jia/wait/*` merge 收口之后，重点说明：

- `chassis_semantics` 迁移到 doctest runner、共享 harness 与 7 个行为域分片后的当前入口。
- zero-stop 的目标门控层与 residual 收尾层如何分工。
- X-Park 进入门、锁存保持和 target / command exit 门口径。
- `RUNTIME_MIN` / `FULL_DEBUG` 两套 chassis 编译档位，以及 host 测试如何分别覆盖它们。
- 本轮吸收 `511daa1e` 后，`debug9 / kSteerAngleAndDriveSpeedMode` 不再被 X-Park pose 或零电流保持覆盖。

## 与上一阶段 handoff 的关系

继续向前追溯时，上一份主线级 merge handoff 是：

- [../handoff/2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md](../handoff/2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md)

更早的 drive PID / payload 说明仍然有参考价值：

- [../handoff/2026-05/ai_handoff_2026-05-26_rc10_drive_pid_load_tune_merge.md](../handoff/2026-05/ai_handoff_2026-05-26_rc10_drive_pid_load_tune_merge.md)
- [../handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md](../handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md)

阅读关系如下：

- `2026-06-05 rc10_chassis_doctest_zero_stop_sync`：当前最新主线级 handoff，优先用于接手 doctest 拆分、zero-stop / X-Park 语义、固件瘦身档和 debug9/X-Park 释放语义。
- `2026-05-31 rc10_wait_1_7_7_1_6_1_merge`：上一阶段 wait 分支 merge 收口 handoff，用于理解三路 wait 分支的最终语义归属与当时验证状态。
- `2026-05-26 rc10_drive_pid_load_tune_merge`：用于理解 zero-stop / drive-load merge 之前的收口背景。
- `2026-05-26 singlewheeltrace_payload_semantics`：用于处理 `SingleWheelTrace`、上位机解析或 VOFA 侧脚本问题。

如果只读一份，请先读 2026-06-05 handoff；如果要追 merge 背景，再补读 2026-05-31 handoff。
