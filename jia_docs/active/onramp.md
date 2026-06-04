# 接手顺序

如果你是下一位接手 `jia_docs` 或 RC10 调试主线的人，建议按下面顺序进入。

## 第一步：先看当前入口

1. [overview.md](overview.md)
2. [latest-handoff.md](latest-handoff.md)

这两页会先告诉你：

- 当前主线已经并入了什么
- 现在推荐看哪份 handoff
- 当前已知 baseline 状态如何

## 第二步：先跑统一入口

优先执行：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/run_tests.ps1
```

如果你在做兼容验证，也可以按需单独跑这些旧入口：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_test.ps1
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_pid_reconnect_test.ps1
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/vesc_brake/run_test.ps1
```

这些旧脚本现在都应视为兼容包装入口，而不是首选主入口。

## 第三步：按需要深入分层资料

- 想继续追当前交接，去看 [../handoff/INDEX.md](../handoff/INDEX.md)
- 想追历史稳定归档，去看 [../history/INDEX.md](../history/INDEX.md)
- 想看测试分层说明，去看 [../tests/README.md](../tests/README.md)
- 想看过程产物与参考物，去看 [../artifacts/README.md](../artifacts/README.md)

## 第四步：决定后续方向

- 若继续底盘联调，优先看 `mode30`、VESC 本地 PID、drive load tune 相关 handoff
- 若继续协议信号/上位机语义，补看 `SingleWheelTrace payload` 说明
- 若继续清理 host baseline，把历史失败和当前主线验证分开处理，不要混成一个任务
