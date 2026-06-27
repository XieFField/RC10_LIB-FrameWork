# RC10 `/6` merge handoff

时间: `2026-05-26`
主题: `rc10_drive_pid_load_tune_merge`

## 本次完成内容

- 在当前 `jia/develop` 上先合并了 `jia/archive/codex_2/6`
  - 保留主线 `mode30 / mode31 / mode32`、统一 `DebugOutputFamily + JustFloatProfile + Binary` 分发、`kSCurve` 默认值、X-Park / homing / fault / zero-current 保护语义
  - 保留 `codex_2/6` 的 `drive_motor_h -> VESC_Motor*` 收窄
  - 保留四轮共享一套 drive speed PID 调参缓存
  - 保留 enable-edge runtime readback、脏缓存保护、`nullptr` 安全 apply、至少一个 VESC apply 成功后才推进 `applied_stamp`

- 随后开始合并 `jia/archive/codex_1/6`
  - 冲突文件手工前向合并:
    - `User/Setup/Inc/chassis.h`
    - `User/Setup/Src/chassis.cpp`
    - `jia_docs/tests/tdd/chassis_semantics/stubs/Motor_VESC.h`
    - `jia_docs/tests/tdd/chassis_semantics/test_chassis_semantics.cpp`
  - 没有把 `codex_1/6` 的并行 `debug_drive_pid_tune_` per-wheel 模型带回主线
  - 改为把 `codex_1/6` 的第二步虚拟负载整定链路叠加到现有共享 drive PID 模型上

## 最终语义归属

- 主线保留:
  - `mode30 / mode31 / mode32` 路由边界
  - `manual_speed_profile_manual_only` 限制
  - `manual_speed_profile_mode = kSCurve`
  - X-Park、fault gate、homing gate、`current = 0` 保护优先级

- `/6` 叠加:
  - `Motor_VESC` 本地速度环电流语义固定为 `raw pid output + bias = total current`
  - 非 `SET_PID_SPEED_CURRENT` 模式时 `raw/total` 观测主动清零
  - `mode30` drive RPM 路径支持自动阶跃器
  - drive 虚拟负载 bias 只在以下条件下注入:
    - 单轮隔离
    - drive 命令类型为 RPM
    - 目标 drive 为 `VESC_RPM_CONTROL_PID_CURRENT`
    - 未被 zero-current / torque-free / fault / 非目标轮 / not-homed 拦截
  - 其他路径统一主动 `setSpeedPidCurrentBias(0.0f)`
  - 新增 `JustFloatProfile::kDrivePidLoadTune`
  - 新增 15 通道 load trace:
    - `time / wheel / target_rpm / feedback_rpm / total_current / pid_current / bias / J / B / Tc / omega / alpha / step_phase / virtual_load_enable / stepgen_enable`

## 关键文件

- `User/Setup/Inc/chassis.h`
- `User/Setup/Src/chassis.cpp`
- `RC10_LIB/Motor/Inc/Motor_VESC.h`
- `RC10_LIB/Motor/Src/Motor_VESC.cpp`
- `jia_docs/tests/tdd/chassis_semantics/stubs/Motor_VESC.h`
- `jia_docs/tests/tdd/chassis_semantics/test_chassis_semantics.cpp`
- `jia_docs/tests/legacy_ai_tests/vesc_brake/test_motor_vesc.cpp`

## 验证命令

```powershell
powershell -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\jia_docs\tests\tdd\chassis_semantics\run_test.ps1
powershell -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\jia_docs\tests\tdd\chassis_semantics\run_pid_reconnect_test.ps1
powershell -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\jia_docs\tests\legacy_ai_tests\vesc_brake\run_test.ps1
git diff --check
```

## 验证结果

- `run_pid_reconnect_test.ps1`: `PASS`
- `legacy_ai_tests/vesc_brake/run_test.ps1`: `PASS`
- `chassis_semantics/run_test.ps1`: 仍有 `18` 个失败
  - 这些失败在本次 merge 前的 `ORIG_HEAD` 基线中同样存在
  - 本次 `/6` 合并过程中一度新增过额外失败，已回收至基线 `18`
  - 新增的共享 PID / load tune / step generator / trace 相关测试已通过
- `git diff --check`: 仅有 CRLF 预警，无内容格式错误

## 后续联调关注点

- 完成 merge 后仍需 `git add` 冲突文件并结束当前 merge
- 建议上板验证:
  - `mode30` 单轮 RPM + `VESC_RPM_CONTROL_PID_CURRENT`
  - 自动阶跃相位切换
  - bias 清零是否覆盖 zero-current / torque-free / not-homed / 非目标轮
  - `kDrivePidLoadTune` 15 通道 payload 顺序是否与上位机脚本一致
- 若要继续收敛 `chassis_semantics` 的 18 个 baseline 失败，建议单开任务，不与本次 merge 收尾混在一起
