# 最新交接入口

当前最推荐优先阅读的 handoff 是：

- [../handoff/2026-06/ai_handoff_2026-06-09_rc10_path_yaw_homing_build_sync.md](../handoff/2026-06/ai_handoff_2026-06-09_rc10_path_yaw_homing_build_sync.md)

这份文档接在 `2026-06-05` doctest / zero-stop / X-Park / RUNTIME_MIN 收口之后，重点说明：

- `AAA-Path` 路径规划正式版合入后的主要触点。
- yaw lock 跨模式旧锁角、目标规划/PID 输入、目标限速和减速先刹再锁的修复链。
- 光电 homing 三边沿确认误校准修复和 host 测试覆盖。
- 调试开关从启用总开关/输出能力到关闭调试模式的状态收口。
- `02dbf00a` 清理合并残留并恢复 MDK 编译的最新保存点。

## 与上一阶段 handoff 的关系

继续向前追溯时，上一份主线级 handoff 是：

- [../handoff/2026-06/ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md](../handoff/2026-06/ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md)

再上一份主线级 merge handoff 是：

- [../handoff/2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md](../handoff/2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md)

更早的 drive PID / payload 说明仍然有参考价值：

- [../handoff/2026-05/ai_handoff_2026-05-26_rc10_drive_pid_load_tune_merge.md](../handoff/2026-05/ai_handoff_2026-05-26_rc10_drive_pid_load_tune_merge.md)
- [../handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md](../handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md)

阅读关系如下：

- `2026-06-09 rc10_path_yaw_homing_build_sync`：当前最新主线级 handoff，优先用于接手路径规划合入、yaw lock 修复链、homing 三边沿确认、调试开关状态和 MDK 编译恢复。
- `2026-06-05 rc10_chassis_doctest_zero_stop_sync`：上一阶段主线级 handoff，用于接手 doctest 拆分、zero-stop / X-Park 语义、固件瘦身档和 debug9/X-Park 释放语义。
- `2026-05-31 rc10_wait_1_7_7_1_6_1_merge`：上一阶段 wait 分支 merge 收口 handoff，用于理解三路 wait 分支的最终语义归属与当时验证状态。
- `2026-05-26 rc10_drive_pid_load_tune_merge`：用于理解 zero-stop / drive-load merge 之前的收口背景。
- `2026-05-26 singlewheeltrace_payload_semantics`：用于处理 `SingleWheelTrace`、上位机解析或 VOFA 侧脚本问题。

如果只读一份，请先读 2026-06-09 handoff；如果要追 doctest / zero-stop / X-Park 背景，再补读 2026-06-05 handoff。
