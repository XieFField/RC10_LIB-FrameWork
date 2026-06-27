# 最新交接入口

当前最推荐优先阅读的 handoff 仍是：

- [../handoff/2026-06/ai_handoff_2026-06-24_rc10_homing_search_diagonal_reverse_sync.md](../handoff/2026-06/ai_handoff_2026-06-24_rc10_homing_search_diagonal_reverse_sync.md)

这份文档承接了前一轮 chassis 主线收口，重点补齐：

- 光电门回零 `HomingState::kSearch` 搜索阶段的编译期固定方向表 `JIA_CHASSIS_HOMING_SEARCH_RPM_SIGN[4]`
- 默认 `0 + 2` 对角轮反转，`1 + 3` 保持同向
- 该方向分裂只影响光电门回零搜索下发 RPM，不改正常运动、recovery re-home、zero-stop、X-Park 和其他模式
- 当时 host 验证结果更新为 `211/211 passed`

## 2026-06-27 最新提交补充

`2026-06-24` handoff 之后，最新事实来自最近两次提交：

- `fc1f0fac`：修复反向 Homing 搜索边沿确认，允许三边沿确认同时接受正向和反向搜索序列；第二边沿按绝对角差接近 `180` 度校验，第三边沿按绝对角差接近 `360` 度校验，并增加方向一致性检查。
- `fc1f0fac`：新增反向搜索成功进入 Ready、反向搜索第三边沿超容差进入 Fault 的 host 语义测试；搜索 RPM 符号测试改为跟随 `JIA_CHASSIS_HOMING_SEARCH_RPM_SIGN` 配置。
- `cbe88bff`：将默认底盘 profile 切换为 `JIA_CHASSIS_PROFILE_FULL_DEBUG`，调整 motion direction guard 默认阈值参数，并保留 homing 搜索方向表的对角反向默认配置。
- 两次提交记录的验证入口均为 `powershell -ExecutionPolicy Bypass -File jia_docs/tests/run_tests.ps1`。

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

如果只读一份，先读 `rc10_homing_search_diagonal_reverse_sync` 这份，再叠加上面的 `2026-06-27` 提交补充；如果要继续追 auto-retry / X-Park priority brake / debug 接入背景，再补同日上一份 handoff。
