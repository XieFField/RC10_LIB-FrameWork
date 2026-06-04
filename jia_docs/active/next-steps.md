# 当前待办与联调关注点

本页用于接替旧 `plan.txt` 的待办入口，保留仍然有效的事项，并对齐 2026-05-31 的最新 handoff 状态。

## 从旧 `plan.txt` 迁移出的未完成项

以下条目在旧待办里仍未标记为 `pass`，现在继续视为开放事项：

1. 完成 `S` 型速度规划的后续落地与收口
2. 测试对外接口
3. 完成舵轮校准调用接口

## 从最新 handoff 提取的联调关注点

围绕 [ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md](../handoff/2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md)，继续联调时建议优先关注：

1. `mode30` 单轮 RPM + `VESC_RPM_CONTROL_PID_CURRENT` 的联动行为
2. 自动阶跃器的相位切换是否符合预期
3. bias 清零是否覆盖以下保护边界：
   - `zero-current`
   - `torque-free`
   - `not-homed`
   - 非目标轮
4. `kDrivePidLoadTune` 的 15 通道 payload 顺序是否与上位机脚本一致

## 现在不再这样描述

- 不再把 `jia_docs/tests/tdd/chassis_semantics/run_test.ps1` 描述为“仍有 18 个 baseline 失败”
- 2026-05-31 handoff 已记录当前主机侧入口为 `PASS`
- 旧 `run_test.ps1` 系列现在应按兼容包装入口理解，不应再作为主要结论来源

## 如果你要继续清理历史项

- 若要继续消化历史 baseline，建议单开任务，不要和当前主线交接混在一起
- 若要继续追 `trace` 或上位机语义，优先补读 `SingleWheelTrace payload` 说明 handoff
- 若要继续推进主机测试重构，优先以 `jia_docs/tests/run_tests.ps1` 为唯一主入口对齐文档
