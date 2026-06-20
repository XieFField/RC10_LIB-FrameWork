# RC10 `wait /7` merge handoff

时间: `2026-05-31`
主题: `rc10_wait_1_7_7_1_6_1_merge`

## 本次完成内容

- 在 `jia/develop` 上按真实 merge 顺序完成三路分支并回:
  - `jia/wait/codex_1/7` -> commit `0589a304`
  - `jia/wait/codex_7/1` -> commit `96964a24`
  - `jia/wait/codex_6/1` -> commit `c4ab4404`
- merge 过程中没有使用 cherry-pick，也没有改写为线性历史。
- `jia/wait/codex_7/1` 合并时仅在 `jia_docs/tests/tdd/chassis_semantics/test_chassis_semantics.cpp` 出现文本冲突。
- `MDK-ARM/Frame_T.uvoptx` 没有跟随 `/7/1` 的本地 IDE 状态改动回主线，按当时 `HEAD` 保留。

## 最终语义归属

- 从 `jia/wait/codex_1/7` 保留:
  - `JustFloatProfile::kDriveZeroStopBrakeTrace`
  - `emitUart8VofaDriveZeroStopBrakeTrace()` 的 12 通道 zero-stop 刹车观测
  - `Motor_VESC` 与宿主 stub 的刹车只读观测接口
  - zero-stop gate 优先依据整车 body/world 目标静止意图判定
  - native eRPM 路径下 zero-stop brake 仍可触发
  - `drive_zero_stop_brake_release_speed_m_s` / `drive_zero_stop_brake_reenter_speed_m_s`
  - `drive_zero_stop_settled_[4]` 的最终停稳阶段语义

- 从 `jia/wait/codex_7/1` 叠加保留:
  - `XParkSteerDeadbandConfig::zero_current_release_enable`
  - X-Park 死区内默认直接释放零电流的策略开关
  - 关闭该策略位后，恢复“冻结当前角度并继续走位置环”的旧语义
  - `steer_speed_pid_settle_reset_cfg_` 与单次 reset 判稳收尾
  - `testJustFloatDrivePidLoadProfileEmitsFixed16ChannelPayload()` 的现主线 drive-load 观测口径

- 从 `jia/wait/codex_6/1` 最后收口保留:
  - 舵轮冻结/断链判定不再依赖显式运动意图
  - X-Park 静止态与普通静止态下，ready 轮都可进入 fault latch
  - 已锁故障后，无需新的速度指令也可继续走 rehome / recovering 链路
  - `steer_fault_xpark_stationary_hold`、`steer_fault_freeze_candidate` 等调试镜像保留

## 手工决议点

- `test_chassis_semantics.cpp` 的 `main()` 注册列表手工解冲突为:
  - 保留 `testJustFloatDrivePidLoadProfileEmitsFixed16ChannelPayload()`
  - 同时保留两条 zero-stop brake trace 回归:
    - `testJustFloatDriveZeroStopBrakeTraceEmitsFixed12ChannelPayloadWhenBrakeInactive()`
    - `testJustFloatDriveZeroStopBrakeTraceEmitsBrakeStateAndCurrent()`
- 没有把 `/7/1` 对 zero-stop 的旧口径回退带回主线。
- 没有把 `/6/1` 的静止断链修复覆盖掉前两步的 zero-stop / X-Park 新逻辑。

## 关键文件

- `User/Setup/Inc/chassis.h`
- `User/Setup/Src/chassis.cpp`
- `RC10_LIB/Motor/Inc/Motor_VESC.h`
- `jia_docs/tests/tdd/chassis_semantics/stubs/Motor_DJI.h`
- `jia_docs/tests/tdd/chassis_semantics/stubs/Motor_VESC.h`
- `jia_docs/tests/tdd/chassis_semantics/test_chassis_semantics.cpp`

## 验证命令

```powershell
powershell -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\jia_docs\tests\tdd\chassis_semantics\run_test.ps1
powershell -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\jia_docs\tests\tdd\chassis_semantics\run_pid_reconnect_test.ps1
powershell -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\jia_docs\tests\legacy_ai_tests\vesc_brake\run_test.ps1
git diff --check
```

## 验证结果

- `jia_docs/tests/tdd/chassis_semantics/run_test.ps1`: `PASS`
- `jia_docs/tests/tdd/chassis_semantics/run_pid_reconnect_test.ps1`: `PASS`
- `jia_docs/tests/legacy_ai_tests/vesc_brake/run_test.ps1`: `PASS`
- 这次三路 merge 后，没有保留旧的 baseline 失败，当前宿主测试入口已全部通过。

## 后续联调关注点

- 上板优先确认:
  - zero-stop brake trace 的 12 通道 payload 顺序
  - drive-load trace 现口径与上位机脚本是否一致
  - X-Park 死区零电流释放开关在开/关两种配置下的实际舵向行为
  - 静止 fault latch 后的 rehome 恢复链路是否与宿主测试一致
- 如果后续继续调试 trace / VOFA 侧脚本，先读本文件，再补读 `2026-05-26` 的 payload 说明 handoff。
