# 当前主线总览

本页回答当前 `jia_docs` 接手时最重要的三个问题：现在主线是什么、先读什么、先跑什么。

## 当前主线状态

截至 `2026-06-24`，当前 RC10 底盘主线应按“2026-06-09 path / yaw / homing / build 主线 + 最近这几轮输入链、首次上电延时、debug deadzone 和默认宏基线补充”来理解。当前状态可以概括为：

- `AAA-Path` 路径规划正式版仍然是当前主线背景，上一阶段路径、yaw lock、homing 与 build 收口继续有效。
- 底盘线程内部的遥控缓存类型已经切换为 `communication::RC10_AirJoy_Data_S`，并通过 `communication::Lora_communication::GetInstance()->update_airjoy_data(&airjoy_data_)` 主动拉取 AirJoy 数据。
- 首次上电整车 homing 延时已落地到 chassis 级运行态 `first_boot_homing_delay_`，其观察口径已经固定为 `pending / active / elapsed_ms`。
- debug 摇杆 deadzone 已改成“死区内归零、出区后从死区边界重新起算”的重映射语义：
  - 整车 debug 接管链使用平移组/旋转组 deadzone。
  - mode30 单轮 debug 继续使用 `single_wheel.input_deadzone`，但语义也升级为重映射。
- 这套 debug deadzone 只作用于 debug 链，不影响正常非 debug 的整车控制链路。
- 当前 `chassis.h` 默认宏基线已经切换为：
  - `JIA_CHASSIS_PROFILE = JIA_CHASSIS_PROFILE_RUNTIME_MIN`
  - `JIA_CHASSIS_HOMING_SEARCH_RPM = 100.0f`
  - `JIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_MS = 500U`
- `chassis_semantics` 宿主测试仍然是当前底盘语义验证主入口，最近一轮已知状态为：
  - `run_main.ps1`：`199 passed`
  - `run_slim_smoke.ps1`：`sizeof(Chassis)=2736`

## 推荐阅读顺序

1. [latest-handoff.md](latest-handoff.md)
2. [onramp.md](onramp.md)
3. [../tests/README.md](../tests/README.md)
4. [../handoff/INDEX.md](../handoff/INDEX.md)

## 当前推荐验证入口

优先执行统一入口：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/run_tests.ps1
```

如果只需要细跑当前底盘宿主语义套件：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_main.ps1
```

如果只需要确认瘦身固件档：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_slim_smoke.ps1
```

## 已知基线情况

- `jia_docs/tests/run_tests.ps1`：当前统一入口，应作为继续开发时的默认验证路径。
- `jia_docs/tests/host_cpp/chassis_semantics/run_main.ps1`：当前 chassis doctest 主入口，使用 `FULL_DEBUG` 编译 runner、harness 与行为域分片。
- `jia_docs/tests/host_cpp/chassis_semantics/run_slim_smoke.ps1`：`RUNTIME_MIN` 极限运行固件档 smoke，验证可编译、可实例化和 `Chassis` 对象体积下降。
- `jia_docs/tests/tdd/chassis_semantics/run_test.ps1`：旧路径兼容入口，转发到当前 chassis 主入口，不再作为新增测试首选位置。
- `jia_docs/tests/host_cpp/swerve_core/run_test.ps1`：历史对照入口，当前因生产源文件缺失登记为 known xfail。

## 接下来继续开发时的重点

- 若继续底盘联调，优先围绕首次上电 homing 延时、`homing_search_rpm = 100.0f`、以及 `mode30` 单轮 RPM + `VESC_RPM_CONTROL_PID_CURRENT` 组合验证。
- 若继续路径规划，优先围绕 `APP_Path.*`、`APP_Speedplanner.*` 和 `omni_chassisSetup.*` 做 MDK 与实车路径复核。
- 若继续 debug 手感与观察链，优先确认整车 debug deadzone 和单轮 debug deadzone 的板上行为是否符合预期。
- 若继续 homing / 舵向恢复链路，优先看 `steer_fault_homing_recovery` 分片、`first_boot_homing_delay_` 运行态，以及首次上电延时是否按 `pending / active / elapsed_ms` 正常推进。
- 若继续协议或上位机语义，优先核对 AirJoy/Lora 输入链、`SingleWheelTrace` 以及当前仍保留的 trace 解释口径。
- 若继续固件瘦身，保持 `RUNTIME_MIN` 默认档与 `FULL_DEBUG` host 语义套件的分层，不要把调试回归拉进 slim smoke。
