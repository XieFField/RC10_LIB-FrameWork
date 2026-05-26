# 接手顺序

如果你是下一位接手 `jia_docs` 或 RC10 调试主线的人，建议按下面顺序进入：

## 第一步：先读当前入口

1. [overview.md](overview.md)
2. [latest-handoff.md](latest-handoff.md)

这两页会先告诉你：

- 当前主线已经并入什么
- 现在推荐读哪份 handoff
- 当前已知 baseline 状态如何

## 第二步：跑主验证脚本

优先执行：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/tdd/chassis_semantics/run_test.ps1
powershell -ExecutionPolicy Bypass -File jia_docs/tests/tdd/chassis_semantics/run_pid_reconnect_test.ps1
```

如果你要继续跟进 VESC 本地 PID 速度环，再执行：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/legacy_ai_tests/vesc_brake/run_test.ps1
```

## 第三步：按需要进入细分资料

- 想继续追当前迭代交接：
  - 看 [../handoff/INDEX.md](../handoff/INDEX.md)
- 想追历史稳定归档：
  - 看 [../history/INDEX.md](../history/INDEX.md)
- 想进入测试资产：
  - 看 [../tests/README.md](../tests/README.md)
- 想查看过程附件或参考物：
  - 看 [../artifacts/README.md](../artifacts/README.md)

## 第四步：决定继续方向

- 若继续底盘联调：优先看 `mode30`、VESC 本地 PID、drive load tune 相关 handoff
- 若继续 trace / 上位机语义：补读 `SingleWheelTrace payload` 补充说明
- 若继续清 baseline：把 host 失败单独拆任务处理
