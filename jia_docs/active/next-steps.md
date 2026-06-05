# 当前待办与联调关注点

本页用于接替旧 `plan.txt` 的待办入口，保留仍然有效的事项，并对齐 2026-06-05 的当前 handoff 状态。

## 从旧 `plan.txt` 迁移出的未完成项

以下条目在旧待办里仍未标记为 `pass`，现在继续视为开放事项：

1. 完成 `S` 型速度规划的后续落地与收口
2. 测试对外接口
3. 完成舵轮校准调用接口

## 从当前 handoff 提取的联调关注点

围绕 [ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md](../handoff/2026-06/ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md)，继续联调时建议优先关注：

1. `mode30` 单轮 RPM + `VESC_RPM_CONTROL_PID_CURRENT` 的联动行为
2. zero-stop active 后 residual 收尾到 brake / zero-current 的行为
3. X-Park 进入门、锁存保持、target / command exit 门是否符合预期
4. `debug9 / kSteerAngleAndDriveSpeedMode` 从 X-Park 零电流保持切出时是否稳定恢复舵角目标
5. `RUNTIME_MIN` 默认固件档是否持续保持可编译、可运行和对象体积下降
6. `kDrivePidLoadTune` 的 15 通道 payload 顺序是否与上位机脚本一致

## 现在不再这样描述

- 不再把 `jia_docs/tests/tdd/chassis_semantics/run_test.ps1` 描述为“仍有 18 个 baseline 失败”。
- 2026-05-31 handoff 只作为上一阶段验证记录；当前入口以 2026-06-05 doctest / zero-stop / slim smoke 主线说明为准。
- 旧 `run_test.ps1` 系列现在应按兼容包装入口理解，不应再作为主要结论来源。
- `RUNTIME_MIN` slim smoke 不承载 debug9、DebugMirror、串口输出等调试语义回归。

## 如果你要继续清理历史项

- 若要继续消化历史 baseline，建议单开任务，不要和当前主线交接混在一起。
- 若要继续追 `trace` 或上位机语义，优先补读 `SingleWheelTrace payload` 说明 handoff。
- 若要继续推进主机测试重构，优先以 `jia_docs/tests/run_tests.ps1` 为唯一主入口对齐文档。
