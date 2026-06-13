# AI handoff 2026-06-09 RC10 path / yaw / homing / build sync

本页记录 `2026-06-09` 前后 `jia/develop_1` 在路径规划合入、底盘 yaw lock 稳定化、homing 三边沿确认、调试开关收口和 MDK 编译恢复上的最新交接状态。它接在 `2026-06-05` doctest / zero-stop / X-Park / RUNTIME_MIN handoff 之后，是当前进入 `jia_docs` 时的最新主线级交接。

## 当前结论

- `AAA-Path` 路径规划线已经合入 `jia/develop_1`，主要覆盖 `APP_Path.*`、`APP_Speedplanner.*` 与 `omni_chassisSetup.*`。
- yaw lock 已完成一轮连续修复：跨模式旧锁角复用、目标规划限速、PID 输入口径，以及减速阶段先刹再锁。
- 光电 homing 三边沿确认误校准已修复，host 语义测试已经补上对应 harness 与恢复链路覆盖。
- 调试开关经历“启用总开关/输出能力”到“关闭调试模式”的收口；当前最新保存点应按关闭调试后的状态理解。
- `02dbf00a fix(build): 清理合并残留恢复 MDK 编译` 是当前最新收口提交，说明路径与底盘合入后已经做过 MDK 编译残留清理。

## 相关提交

- `0ee72f23 Merge remote-tracking branch 'origin/AAA-Path'`
  - 将路径规划正式版合入当前主线。
- `bedca82e 注释没用的代码，完成对路径规划正式版的修改，可合并`
  - 更新 `RC10_LIB/APP/Inc/APP_Path.h`、`RC10_LIB/APP/Src/APP_Path.cpp`、`APP_Speedplanner.*` 与 `omni_chassisSetup.*`。
- `ec4c93e3 test(chassis): 扩大 yaw lock 模式切换排查覆盖`
  - 扩展 `test_chassis_semantics_yaw_and_motion_profile.cpp`，暴露跨模式锁角复用风险。
- `a4f9d912 fix(chassis): 修复 yaw lock 跨模式复用旧锁角`
  - 修复 mode 切换后继续沿用旧锁角的问题。
- `3eccb859 test(chassis): 暴露 yaw lock 目标限速失效`
  - 为 yaw lock 目标限速和 PID 输入问题补 red 侧证据。
- `8c365887 fix(chassis): 修复 yaw lock 目标规划与 PID 输入`
  - 修复 yaw lock 目标规划、限速与 PID 输入口径。
- `c9af3414 tune(chassis): 调整驱动速度环与 yaw 锁角参数`
  - 调整 `APP_PID.cpp`、`Setup_ConfigInit.cpp` 与 `chassis.cpp` 中的驱动速度环和 yaw lock 参数。
- `257966a7 fix(chassis): yaw 锁角减速阶段先刹再锁`
  - 将 yaw lock 减速阶段改为先刹车再锁角，并在 `drive_delivery_zero_stop` 分片补充交互覆盖。
- `95207017 test(chassis): 覆盖 homing 三边沿确认语义`
  - 在 harness 与 `steer_fault_homing_recovery` 分片补充三边沿确认测试。
- `719ed9d2 fix(chassis): 修复光电 homing 三边沿确认误校准`
  - 修复 homing 三边沿确认造成的误校准。
- `9fe0bff1 fix(chassis): 启用调试总开关和输出功能`
  - 打通调试总开关和输出能力。
- `c43680a0 refactor(chassis): 合并单轮UART8追踪编译开关`
  - 收口单轮 UART8 追踪相关编译开关。
- `759fbf57 关闭调试模式`
  - 将当前固件状态切回关闭调试模式。
- `02dbf00a fix(build): 清理合并残留恢复 MDK 编译`
  - 清理 `Robot_Arm.h` 与 `chassis.h` 中的合并残留，恢复 MDK 编译。

## 当前阅读顺序

如果只接手当前状态，建议先读本页，再按需要补读上一阶段 handoff：

1. 本页：路径规划合入、yaw lock、homing、debug 状态、MDK build 收口。
2. [ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md](ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md)：doctest 分片、zero-stop / X-Park、RUNTIME_MIN / FULL_DEBUG 分层。
3. [../2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md](../2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md)：上一阶段三路 wait merge 收口。
4. [../2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md](../2026-05/ai_handoff_2026-05-26_1124_singlewheeltrace_payload_semantics.md)：`SingleWheelTrace` payload 与上位机解析口径。

## 测试入口

当前默认仍从统一入口进入：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/run_tests.ps1
```

底盘 host doctest 主入口仍是：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_main.ps1
```

`RUNTIME_MIN` 瘦身档 smoke 仍是：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_slim_smoke.ps1
```

本轮新增或强化关注的测试分片：

- `test_chassis_semantics_yaw_and_motion_profile.cpp`：yaw lock 跨模式、目标规划、限速与 PID 输入主回归入口。
- `test_chassis_semantics_drive_delivery_zero_stop.cpp`：drive delivery、zero-stop，以及 yaw lock 减速先刹再锁交互覆盖。
- `test_chassis_semantics_steer_fault_homing_recovery.cpp`：homing 三边沿确认、舵向 fault 与恢复链路覆盖。

## 后续关注

- 路径规划已合入，但仍建议结合 MDK 工程和实车流程复核 `APP_Path.*`、`APP_Speedplanner.*`、`omni_chassisSetup.*` 的真实运行路径。
- yaw lock 已有 host 语义回归，后续重点是上板确认减速先刹再锁是否符合驾驶手感与比赛策略。
- homing 三边沿确认已修，后续重点是实车确认光电边沿抖动、误触发和 rehome 链路。
- 关闭调试模式后继续保留 `FULL_DEBUG` host 回归与 `RUNTIME_MIN` 默认固件档分层，不要把调试语义塞回 slim smoke。
- 继续保留 MDK 编译入口检查，尤其是路径规划、机械臂和底盘头文件合并后的工程文件残留。
