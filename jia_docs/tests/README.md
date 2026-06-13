# tests 说明

这个目录存放当前推荐的测试入口、host C++ 语义测试、Python 语义回归、历史保留区，以及统一调度脚本。主入口是 `jia_docs/tests/run_tests.ps1`；旧路径保留为兼容包装，不再作为新增测试的首选位置。

## 现在优先运行什么

继续开发时，优先执行：

1. `jia_docs/tests/run_tests.ps1`
2. `jia_docs/tests/host_cpp/chassis_semantics/run_main.ps1`
3. `jia_docs/tests/host_cpp/chassis_semantics/run_slim_smoke.ps1`

默认 `run_tests.ps1` 的 `smoke` 已经会调度 `host_cpp.chassis_semantics.slim_smoke`；单独跑 `run_slim_smoke.ps1` 更适合定点复查 `RUNTIME_MIN` 瘦身档，而不是每次统一入口后的必做第二步。

如果只想确认 chassis 发布固件瘦身档是否还能编译运行，直接跑：

- `jia_docs/tests/host_cpp/chassis_semantics/run_slim_smoke.ps1`

如果要继续跟进 VESC 刹车相关宿主回归，再看：

- `jia_docs/tests/host_cpp/vesc_brake/run_test.ps1`

如果要继续看 Python 语义回归，再看：

- `jia_docs/tests/python_semantics/swerve_port/lock_now_homing_gate_regression.py`

## host_cpp/

当前 host C++ 测试由统一入口调度。active 套件包括：

- `host_cpp/chassis_semantics/`
- `host_cpp/vesc_brake/`

`host_cpp/chassis_semantics/` 是当前最主要的底盘宿主语义套件：

- `run_main.ps1` 使用 `JIA_CHASSIS_PROFILE_FULL_DEBUG` 编译 doctest runner、harness 和行为域分片。它保留所有调试字段，服务语义回归、debug9/X-Park 等调试链路验证，以及调试器可观察性。
- `run_slim_smoke.ps1` 使用 `JIA_CHASSIS_PROFILE_RUNTIME_MIN` 编译最小 smoke。它不跑调试语义，只验证极限运行固件档可编译、可实例化，并输出 `sizeof(Chassis)`。
- `test_chassis_semantics_harness.h/.cpp` 放公共测试桩、配置函数和循环驱动 helper。新增行为测试时优先复用这里已有 setup，不要在分片里复制一套。
- 行为分片按 `test_chassis_semantics_<domain>.cpp` 命名。新增 case 时优先放进对应行为域，只有跨域公共准备逻辑才放入 harness。
- 旧的手写 `test_chassis_semantics_registry.inc` 已退役；测试发现、过滤和失败定位交给 doctest。

当前 chassis_semantics 分片：

- `test_chassis_semantics_drive_pid_and_mapping.cpp`
- `test_chassis_semantics_single_wheel_debug.cpp`
- `test_chassis_semantics_drive_delivery_zero_stop.cpp`
  - 覆盖 drive delivery、zero-stop，并承接 2026-06-09 yaw lock 减速阶段先刹再锁的交互回归。
- `test_chassis_semantics_yaw_and_motion_profile.cpp`
  - 覆盖 yaw lock 跨模式旧锁角、目标规划、目标限速与 PID 输入口径，是 2026-06-08/09 yaw 修复链的主回归入口。
- `test_chassis_semantics_swerve_planner_flip_reverse.cpp`
- `test_chassis_semantics_xpark_gate_and_hold.cpp`
  - 覆盖 X-Park 进入门、保持态、target / command exit 门，以及 debug9 释放 X-Park pose / steer hold 覆盖的回归。
- `test_chassis_semantics_steer_fault_homing_recovery.cpp`
  - 覆盖 homing 三边沿确认、舵向 fault 与恢复链路，是 2026-06-08 homing 修复的主回归入口。

`host_cpp/*/build/` 下的内容视为可重建缓存；长期证据应优先登记到 `artifacts/` 或 handoff 中，并写清楚由哪个脚本重建。

## 编译档位

`User/Setup/Inc/chassis.h` 顶部定义底盘 profile：

- `JIA_CHASSIS_PROFILE_RUNTIME_MIN`：比赛/发布固件默认档，关闭调试接管、单轮直控、调试输出、PID 调参缓存、DebugMirror 和 TaskPerf 大缓存。
- `JIA_CHASSIS_PROFILE_FULL_DEBUG`：host 语义测试和调试器观察档，保留所有调试字段和输出路径。

新增测试时，如果测试读取 `debug_control_`、`debug_output_`、`debug_pid_tune_`、`debug_mirror_` 等内部调试字段，应放进 `run_main.ps1` 的 FULL_DEBUG doctest 套件。只验证瘦身档编译、尺寸或基础运行时，应放到 slim smoke 或后续专门的 runtime-min 分片。

## 兼容路径

旧 `tdd/` 路径仍保留这些兼容包装入口，但不再作为新增测试的首选位置：

- `run_test.ps1`
- `run_pid_reconnect_test.ps1`
- `run_app_utils_backend_test.ps1`
- `run_app_utils_math_test.ps1`

## 维护约定

- 新增高价值测试入口时，优先在 `jia_docs/tests/tests.yaml` 和 `jia_docs/catalog/tests.yaml` 登记。
- 统一入口写在 `jia_docs/tests/run_tests.ps1`；旧路径只保留兼容用途。
- 需要长期对照的内容放入 `historical/`，不要让历史保留区影响默认执行路径。
- 文档使用 UTF-8 中文；提交前检查新增中文说明和提交信息没有乱码。
