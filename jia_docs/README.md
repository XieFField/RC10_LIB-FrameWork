# jia_docs

`jia_docs` 用于归档 RC10 / AI 协作过程中的交接文档、测试资料、过程产物与当前接手入口，不参与产品代码编译。

## 首选入口

如果你只想快速接手当前 RC10 / AI 协作状态，按这个顺序读：

1. [active/overview.md](active/overview.md)
2. [active/latest-handoff.md](active/latest-handoff.md)
3. [active/onramp.md](active/onramp.md)
4. [tests/README.md](tests/README.md)

`active/overview.md` 是当前主线状态的高层入口；根 README 只保留稳定导航，避免重复维护容易过期的主线细节。

## 当前 handoff

当前最推荐优先阅读：

- [active/latest-handoff.md](active/latest-handoff.md)

完整索引：

- [handoff/INDEX.md](handoff/INDEX.md)

## 主验证入口

继续开发时，优先跑统一入口：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/run_tests.ps1
```

如果只需要细跑当前 chassis doctest 宿主语义套件：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_main.ps1
```

如果只需要确认 chassis `RUNTIME_MIN` 极限运行固件档仍可编译运行：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_slim_smoke.ps1
```

兼容入口、专项入口和历史入口请看 [tests/README.md](tests/README.md)。

## 下一步待办

当前仍开放的待办与联调关注点已迁移到：

- [active/next-steps.md](active/next-steps.md)

旧的 `plan.txt` 会逐步退场，只作为历史兼容入口保留。

## 目录说明

- [active/](active/)：当前有效入口层，总览、最新 handoff、接手顺序、下一步待办。
- [catalog/](catalog/)：元数据层，登记当前主线、测试套件、handoff 与 artifact 关联。
- [handoff/](handoff/)：当前迭代交接流，按年月归档。
- [history/](history/)：已冻结的稳定归档。
- [tests/](tests/)：测试说明、统一入口、兼容包装入口、历史保留区与相关产物。
- [artifacts/](artifacts/)：过程参考物、摘要产物、辅助附件。

## 历史入口

- 当前交接索引：[handoff/INDEX.md](handoff/INDEX.md)
- 历史归档索引：[history/INDEX.md](history/INDEX.md)
- 接手顺序：[active/onramp.md](active/onramp.md)
