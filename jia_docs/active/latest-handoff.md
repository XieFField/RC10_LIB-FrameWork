# 最新交接入口

当前最推荐优先阅读的 handoff 是：

- [../handoff/2026-06/ai_handoff_2026-06-24_rc10_chassis_airjoy_homing_debug_context_sync.md](../handoff/2026-06/ai_handoff_2026-06-24_rc10_chassis_airjoy_homing_debug_context_sync.md)

这份文档把当前底盘主线重新收口成新的默认入口，重点补齐：

- AirJoy/Lora 遥控输入链已经切换到 `RC10_AirJoy_Data_S`。
- 首次上电整车 homing 延时机制、默认值与调试观察口径。
- debug 摇杆 deadzone 重映射语义，以及它只作用于 debug 链的边界。
- 当前 `chassis.h` 默认宏基线：`RUNTIME_MIN`、`homing_search_rpm = 100.0f`、`first_boot_homing_delay_ms = 500U`。
- 当前 host 语义测试入口和最近一轮通过状态。

## 与上一阶段 handoff 的关系

继续向前追溯时，上一份主线级 handoff 是：

- [../handoff/2026-06/ai_handoff_2026-06-09_rc10_path_yaw_homing_build_sync.md](../handoff/2026-06/ai_handoff_2026-06-09_rc10_path_yaw_homing_build_sync.md)

再上一份主线级收口 handoff 是：

- [../handoff/2026-06/ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md](../handoff/2026-06/ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md)

上一阶段 merge 主线和 payload 补充仍然有参考价值：

- [../handoff/2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md](../handoff/2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md)
- [../handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md](../handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md)

阅读关系如下：

- `2026-06-24 rc10_chassis_airjoy_homing_debug_context_sync`：当前最新默认入口，优先用于接手 AirJoy 输入链、首次上电 homing 延时、debug deadzone 和当前默认宏基线。
- `2026-06-09 rc10_path_yaw_homing_build_sync`：上一阶段主线 handoff，用于接手 AAA-Path、yaw lock 修复链、homing 三边沿确认、debug 状态收口与 MDK 编译恢复。
- `2026-06-05 rc10_chassis_doctest_zero_stop_sync`：上一阶段主线 handoff，用于接手 doctest 分片、zero-stop / X-Park 语义和 `RUNTIME_MIN` / `FULL_DEBUG` 分层。
- `2026-05-31 rc10_wait_1_7_7_1_6_1_merge`：上一阶段 wait merge 收口 handoff，用于追溯 merge 语义归属。
- `2026-05-26 singlewheeltrace_payload_semantics`：用于处理 `SingleWheelTrace`、上位机解析或 VOFA 侧脚本问题。

如果只读一份，请先读 2026-06-24 handoff；如果要追 path / yaw / homing 主线背景，再补读 2026-06-09 handoff。
