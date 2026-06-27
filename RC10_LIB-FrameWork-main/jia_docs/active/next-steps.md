# 当前待办与联调关注点

本页用于接替旧 `plan.txt` 的待办入口，保留仍然有效的事项，并对齐 2026-06-09 的当前 handoff 状态。

## 从旧 `plan.txt` 迁移出的未完成项

以下条目来自旧待办，但需要按 2026-06-09 主线重新理解：

1. `S` 型速度规划与路径规划正式版已经合入，下一步改为 MDK 工程、实车路径和比赛流程复核。
2. 测试对外接口仍开放，下一步应优先确认当前 `run_tests.ps1`、host doctest 与路径规划实车验证之间的边界。
3. 舵轮校准调用接口仍需结合 homing 三边沿确认修复做实车确认，不再按纯文档旧待办理解。

## 从当前 handoff 提取的联调关注点

围绕 [ai_handoff_2026-06-09_rc10_path_yaw_homing_build_sync.md](../handoff/2026-06/ai_handoff_2026-06-09_rc10_path_yaw_homing_build_sync.md)，继续联调时建议优先关注：

1. 路径规划正式版合入后，`APP_Path.*`、`APP_Speedplanner.*`、`omni_chassisSetup.*` 的 MDK 编译、烧录和实车路径行为。
2. yaw lock 减速阶段“先刹再锁”是否符合上板手感、目标规划和 zero-stop / brake 交互预期。
3. homing 三边沿确认修复后，光电边沿抖动、误触发、静止 fault latch 与 rehome 链路是否稳定。
4. 关闭调试模式后，`RUNTIME_MIN` 默认固件档与 `FULL_DEBUG` host 语义回归是否继续分层清晰。
5. `02dbf00a` 之后继续保持 MDK 编译入口检查，避免路径规划、机械臂和底盘头文件合并残留复发。
6. `kDrivePidLoadTune` 与 `SingleWheelTrace` 的 payload 顺序是否仍与上位机脚本一致。

## 现在不再这样描述

- 不再把 `jia_docs/tests/tdd/chassis_semantics/run_test.ps1` 描述为“仍有 18 个 baseline 失败”。
- 2026-06-05 handoff 只作为上一阶段验证记录；当前入口以 2026-06-09 path / yaw / homing / build 主线说明为准。
- 旧 `run_test.ps1` 系列现在应按兼容包装入口理解，不应再作为主要结论来源。
- `RUNTIME_MIN` slim smoke 不承载 debug9、DebugMirror、串口输出等调试语义回归。

## 如果你要继续清理历史项

- 若要继续消化历史 baseline，建议单开任务，不要和当前主线交接混在一起。
- 若要继续追 `trace` 或上位机语义，优先补读 `SingleWheelTrace payload` 说明 handoff。
- 若要继续推进主机测试重构，优先以 `jia_docs/tests/run_tests.ps1` 为唯一主入口对齐文档。
- 若要继续路径规划调试，优先记录实车验证结果，不要把实车问题埋进旧 `plan.txt`。
