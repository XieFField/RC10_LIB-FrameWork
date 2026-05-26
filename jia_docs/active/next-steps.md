# 当前待办与联调关注点

本页用于接替旧 `plan.txt` 的待办入口，保留仍然有效的项目，并合并最新 handoff 中仍值得继续跟进的事项。

## 从旧 `plan.txt` 迁移出的未完成项

以下条目在旧待办里未标记 `pass`，仍视为开放事项：

1. 完成 `S` 型速度规划的后续落地与收口
2. 测试对外接口
3. 完成舵轮校准调用接口

## 从最新 handoff 提取的联调关注点

围绕 [ai_handoff_2026-05-26_rc10_drive_pid_load_tune_merge.md](../handoff/2026-05/ai_handoff_2026-05-26_rc10_drive_pid_load_tune_merge.md)，继续联调时建议优先关注：

1. `mode30` 单轮 RPM + `VESC_RPM_CONTROL_PID_CURRENT` 的联动行为
2. 自动阶跃器的相位切换是否符合预期
3. bias 清零是否覆盖以下保护边界：
   - `zero-current`
   - `torque-free`
   - `not-homed`
   - 非目标轮
4. `kDrivePidLoadTune` 的 15 通道 payload 顺序是否与上位机脚本一致

## 若要继续清理 baseline 失败

- `jia_docs/tests/tdd/chassis_semantics/run_test.ps1` 在最新 merge handoff 中仍记录 `18` 个 baseline 失败。
- 这些失败并非本轮整理新引入的问题。
- 若要继续消化，建议单开任务，不要与 handoff/文档结构重构混做。

## 建议的下一步顺序

1. 先完成当前文档与导航重构收口
2. 再决定是继续底盘联调，还是单独清 baseline 失败
3. 若进入协议/上位机联调，再补读 `SingleWheelTrace payload` 补充 handoff
