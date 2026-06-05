# AI handoff 2026-06-05 RC10 chassis doctest / zero-stop sync

本页记录 `2026-06-05` 前后 RC10 底盘主线在宿主测试结构、zero-stop / X-Park 语义、调试参数口径和固件瘦身档上的收口状态。它接在 `2026-05-31` 三路 wait merge handoff 之后，作为当前进入 `jia_docs` 时的最新主线级交接。

## 当前结论

- `chassis_semantics` 宿主测试已经从单文件手写 registry 迁移到 doctest 主入口，并拆成 runner、共享 harness 与多个行为域分片。
- zero-stop 语义已经收口为“目标门控进入/退出 + active 期间 residual 收尾”两层职责。
- X-Park 静止保持的进入/退出口径已对齐：进入前可用 residual 做安全确认，锁存保持后退出主要看 target / command exit 门。
- `debug9 / kSteerAngleAndDriveSpeedMode` 激活时不允许 X-Park pose 接管，也会释放已锁存的 X-Park steer hold，让统一舵角目标重新下发。
- `chassis.h` 顶部定义 `RUNTIME_MIN` / `FULL_DEBUG` 编译档位；默认发布固件使用 `RUNTIME_MIN`，host 语义测试显式使用 `FULL_DEBUG`。

## 相关提交

- `b13be4af test(chassis): 接入 doctest 主测试入口`
  - 引入 vendored doctest 单头文件。
  - 接入 `test_chassis_semantics_main.cpp` 与 `run_main.ps1` 主入口。
- `447a200a test(chassis): 拆分 chassis_semantics doctest 套件`
  - 删除旧单体 `test_chassis_semantics.cpp` 与手写 registry。
  - 新增共享 harness 和 7 个行为域分片。
- `08129bb8 feat(chassis): 增加 zero-stop residual 收尾开关`
  - 增加 active 期间 residual 收尾到零电流的开关语义。
- `52f7454f feat(chassis): 添加极限运行固件瘦身档`
  - 默认固件档切到 `RUNTIME_MIN`。
  - host doctest 主套件显式切到 `FULL_DEBUG`。
  - 新增 `run_slim_smoke.ps1` 验证瘦身档可编译运行和对象体积下降。
- 本轮合入 `511daa1e` 分支意图
  - debug9 统一舵向/驱动模式不再被 X-Park pose 或零电流保持覆盖。
  - `xpark_gate_and_hold` 分片补充对应 FULL_DEBUG 回归。

## 测试入口

当前默认从统一入口进入：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/run_tests.ps1
```

如果只需要细跑底盘宿主语义套件：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_main.ps1
```

如果只需要确认发布/比赛固件瘦身档：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_slim_smoke.ps1
```

旧 `jia_docs/tests/tdd/chassis_semantics/run_test.ps1` 仍作为兼容包装入口保留，但新增测试和新增文档不应再把它写成首选入口。

## doctest 分片状态

`host_cpp/chassis_semantics` 当前由以下部分组成：

- `test_chassis_semantics_main.cpp`：doctest runner。
- `test_chassis_semantics_harness.h/.cpp`：公共桩、配置 helper 与循环驱动工具。
- `test_chassis_semantics_drive_pid_and_mapping.cpp`：drive PID 与映射语义。
- `test_chassis_semantics_single_wheel_debug.cpp`：单轮调试与 trace payload。
- `test_chassis_semantics_drive_delivery_zero_stop.cpp`：drive 下发、zero-stop、brake / zero-current 收尾。
- `test_chassis_semantics_yaw_and_motion_profile.cpp`：yaw 与运动规划语义。
- `test_chassis_semantics_swerve_planner_flip_reverse.cpp`：swerve planner、flip / reverse 行为。
- `test_chassis_semantics_xpark_gate_and_hold.cpp`：X-Park 进入、保持、退出门控，以及 debug9 释放 X-Park 覆盖的回归。
- `test_chassis_semantics_steer_fault_homing_recovery.cpp`：舵向 fault、homing 与恢复链路。

维护约定：

- 新增 case 优先放进对应行为域分片。
- 只有跨分片复用的桩、配置和循环 helper 才放入 harness。
- 不再新增手写 registry 条目；发现、过滤和失败定位交给 doctest。

## zero-stop / X-Park 语义口径

zero-stop 现在按两层理解：

1. 目标门控层：决定 drive zero-stop 是否进入或退出 active 状态。
2. 末端收尾层：zero-stop 已 active 后，residual 只决定当前轮继续 brake，还是在满足 near-zero 条件后切到 zero-current 收尾。

`enable_drive_zero_stop_settle_zero_current` 的含义是：允许 active 期间 residual 足够小后切到零电流收尾；关闭后 active 期间保持 brake 分支。

X-Park 现在按下面口径理解：

- 进入前可结合 actual / residual 做安全确认。
- 进入保持后，退出主要看 target / command exit 门。
- residual 不应再被描述为“踢出保持态”的核心条件。
- debug9 统一舵向/驱动模式是显式调试目标，应优先于 X-Park pose 和 X-Park steer hold。

## 编译档位口径

- `JIA_CHASSIS_PROFILE_RUNTIME_MIN`：比赛/发布固件默认档，关闭调试接管、单轮直控、调试输出、PID 调参缓存、DebugMirror 和 TaskPerf 大缓存。
- `JIA_CHASSIS_PROFILE_FULL_DEBUG`：host 语义测试和调试器观察档，保留所有调试字段和输出路径。
- `run_main.ps1` 始终显式使用 `FULL_DEBUG`；debug9/X-Park 等调试语义回归放在这里。
- `run_slim_smoke.ps1` 始终显式使用 `RUNTIME_MIN`；它只验证瘦身档编译、基础实例化和 `sizeof(Chassis)` 下降。

## 后续关注

- 上板确认 `mode30` 单轮 RPM 与 `VESC_RPM_CONTROL_PID_CURRENT` 的组合行为。
- 对齐上位机脚本对 drive-load trace 与 zero-stop trace 的通道解释。
- 继续推进旧 `plan.txt` 中尚未完成的 S 型速度规划、测试对外接口和舵轮校准调用接口。
