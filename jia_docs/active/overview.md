# 当前主线总览

截至 `2026-06-24`，当前 RC10 底盘主线应按“2026-06-24 homing search diagonal reverse 收口 + homing auto-retry / X-Park priority brake / debug 接口收口 + 2026-06-09 path / yaw / homing / build 主线背景”来理解。当前状态可概括为：

- `AAA-Path` 路径规划正式版仍然是当前主线背景，上一个阶段路径、yaw lock、homing 和 build 收口继续有效。
- 底盘线程内部的遥控缓存类型已经切换为 `communication::RC10_AirJoy_Data_S`，并通过 `communication::Lora_communication::GetInstance()->update_airjoy_data(&airjoy_data_)` 主动拉取 AirJoy 数据。
- 首次上电整车 homing 延时已经落地到 chassis 级统一运行态 `first_boot_homing_delay_`，其观察口径固定为 `pending / active / elapsed_ms`。
- debug 摇杆 deadzone 已改成“死区内归零、出区后从死区边界重新起算”的重映射语义：整车 debug 控管链使用平移组 / 旋转组 deadzone，mode30 单舵轮 debug 继续使用 `single_wheel.input_deadzone`。
- 这套 debug deadzone 只作用于 debug 链，不影响正常非 debug 的整车控制链路。
- 当前 `chassis.h` 默认宏基线已经切换为：
  - `JIA_CHASSIS_PROFILE = JIA_CHASSIS_PROFILE_RUNTIME_MIN`
  - `JIA_CHASSIS_HOMING_SEARCH_RPM = 100.0f`
  - `JIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_MS = 500U`
- 光电门回零 `HomingState::kSearch` 搜索阶段已使用编译期固定方向表 `JIA_CHASSIS_HOMING_SEARCH_RPM_SIGN[4]`：`0 + 2` 对角轮反转，`1 + 3` 保持同向；该方向表只影响搜索阶段 RPM 下发。
- 新 handoff 路径：`jia_docs/handoff/2026-06/ai_handoff_2026-06-24_rc10_homing_search_diagonal_reverse_sync.md`。
- `chassis_semantics` host 主入口当前已知状态为：`run_main.ps1` 通过，最新同步数字为 `211/211 passed`；上一轮 `run_slim_smoke.ps1` 通过与 `sizeof(Chassis)=2856` 仍作为背景记录。
- host 通过不等于 Keil / MDK / 实车已覆盖。

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

如果只需细跑当前底盘宿主语义套件：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_main.ps1
```

如果只需确认瘦身固件档：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_slim_smoke.ps1
```

## 接手提醒

- 继续底盘联调时，优先核对首次上电 homing 延时、`homing_search_rpm = 100.0f`、以及 `mode30` 单舵轮 RPM + `VESC_RPM_CONTROL_PID_CURRENT` 组合。
- 继续路径规划时，优先围绕 `APP_Path.*`、`APP_Speedplanner.*` 和 `omni_chassisSetup.*` 做 MDK 与实车路径复核。
- 继续调试手感与观测链时，优先确认整车 debug deadzone 和单舵轮 debug deadzone 的板上表现。
- 继续 homing / 舵向恢复链时，优先看 `steer_fault_homing_recovery`、`first_boot_homing_delay_` 的运行态，以及首次上电延时是否按 `pending / active / elapsed_ms` 推进。
- 继续光电门回零搜索方向时，优先核对 `JIA_CHASSIS_HOMING_SEARCH_RPM_SIGN[4]` 和轮位顺序 `0=左前, 1=左后, 2=右后, 3=右前`，保持 `0 + 2` 反转、`1 + 3` 同向。
- 继续协议或上位机语义时，优先核对 AirJoy/Lora 输入链、`SingleWheelTrace` 和当前保留的 trace 解释口径。
- 继续固件瘦身时，保持 `RUNTIME_MIN` 默认档和 `FULL_DEBUG` host 语义套件分层，不要把调试语义回流到 slim smoke。
