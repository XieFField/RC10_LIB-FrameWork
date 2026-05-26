# jia_docs

`jia_docs` 用于归档 RC10 / AI 协作过程中的交接文档、测试资产、过程产物与当前接手入口，不参与产品代码编译。

## 当前主线

当前建议先看：

- [active/overview.md](active/overview.md)

这份总览已经收口到 `2026-05-26` 最新 RC10 调试主线，重点覆盖：

- `drive -> VESC_Motor*` 句柄收口
- drive 共享 PID 调参入口
- VESC 本地 PID 速度环默认启用
- `SingleWheelTrace` payload 扩展
- `APP_Utils` 数学入口收口
- drive PID 第二步虚拟负载整定链路并入

## 最新 handoff

当前最推荐优先阅读：

- [handoff/2026-05/ai_handoff_2026-05-26_rc10_drive_pid_load_tune_merge.md](handoff/2026-05/ai_handoff_2026-05-26_rc10_drive_pid_load_tune_merge.md)

配套补充说明：

- [handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md](handoff/2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md)

如果你不确定先读哪一份，先看：

- [active/latest-handoff.md](active/latest-handoff.md)

## 主验证入口

继续开发时，建议先跑这两个入口确认当前宿主回归上下文：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/tdd/chassis_semantics/run_test.ps1
powershell -ExecutionPolicy Bypass -File jia_docs/tests/tdd/chassis_semantics/run_pid_reconnect_test.ps1
```

如果你要继续跟进 VESC 本地 PID 速度环，再补跑：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/legacy_ai_tests/vesc_brake/run_test.ps1
```

## 下一步待办

当前仍开放的待办与联调关注点已迁移到：

- [active/next-steps.md](active/next-steps.md)

旧的 `plan.txt` 将逐步退役为兼容入口，不再作为主入口维护。

## 目录说明

- [active/](active/)
  - 当前有效入口层：主线总览、最新 handoff、接手顺序、下一步待办
- [catalog/](catalog/)
  - 元数据层：记录当前主线、handoff、tests、artifacts 的统一口径
- [handoff/](handoff/)
  - 当前迭代交接流，按年月归档
- [history/](history/)
  - 已冻结的稳定归档
- [tests/](tests/)
  - AI / TDD / legacy 测试资产与回归入口
- [artifacts/](artifacts/)
  - 过程参考物、摘要产物、辅助附件

## 历史与兼容入口

- 当前交接索引：
  - [handoff/INDEX.md](handoff/INDEX.md)
- 历史归档索引：
  - [history/INDEX.md](history/INDEX.md)
- 接手顺序：
  - [active/onramp.md](active/onramp.md)
