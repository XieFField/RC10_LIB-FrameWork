# 当前主线总览

截至 `2026-06-27`，当前 RC10 底盘主线应按“2026-06-27 反向 Homing 搜索边沿确认 + 默认 FULL_DEBUG 调试配置 + 2026-06-24 homing search diagonal reverse / auto-retry / X-Park priority brake / debug 接口收口 + 2026-06-09 path / yaw / homing / build 主线背景”来理解。当前状态可概括为：

- `AAA-Path` 路径规划正式版仍然是当前主线背景，上一个阶段路径、yaw lock、homing 和 build 收口继续有效。
- 底盘线程内部的遥控缓存类型已经切换为 `communication::RC10_AirJoy_Data_S`，并通过 `communication::Lora_communication::GetInstance()->update_airjoy_data(&airjoy_data_)` 主动拉取 AirJoy 数据。
- 首次上电整车 homing 延时已经落地到 chassis 级统一运行态 `first_boot_homing_delay_`，其观察口径固定为 `pending / active / elapsed_ms`。
- debug 摇杆 deadzone 已改成“死区内归零、出区后从死区边界重新起算”的重映射语义：整车 debug 控管链使用平移组 / 旋转组 deadzone，mode30 单舵轮 debug 继续使用 `single_wheel.input_deadzone`。
- 这套 debug deadzone 只作用于 debug 链，不影响正常非 debug 的整车控制链路。
- `HomingState::kSearch` 三边沿确认已经同时接受正向和反向搜索序列：第二边沿按绝对角差接近 `180` 度校验，第三边沿按绝对角差接近 `360` 度校验，并增加方向一致性检查以避免正反抖动序列误通过。
- 当前 `chassis.h` 默认宏基线已经切换为：
  - `JIA_CHASSIS_PROFILE = JIA_CHASSIS_PROFILE_FULL_DEBUG`
  - `JIA_CHASSIS_HOMING_SEARCH_RPM = 100.0f`
  - `JIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_MS = 500U`
- motion direction guard 默认阈值参数已调整，使实际运动方向约束更早介入；继续联调时应重点观察实车运动方向约束是否符合预期。
- 光电门回零 `HomingState::kSearch` 搜索阶段已使用编译期固定方向表 `JIA_CHASSIS_HOMING_SEARCH_RPM_SIGN[4]`：`0 + 2` 对角轮反转，`1 + 3` 保持同向；该方向表只影响搜索阶段 RPM 下发。
- 当前 handoff 仍先读 `jia_docs/handoff/2026-06/ai_handoff_2026-06-24_rc10_homing_search_diagonal_reverse_sync.md`，但必须叠加 `2026-06-27` 两次提交事实理解。
- `chassis_semantics` host 主入口当前最新引用验证来自 `2026-06-27` 两次提交的 `jia_docs/tests/run_tests.ps1`；`211/211 passed`、`run_slim_smoke.ps1` 通过与 `sizeof(Chassis)=2856` 是上一轮背景记录，不再作为最新数字。
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

- 继续底盘联调时，优先核对首次上电 homing 延时、`homing_search_rpm = 100.0f`、默认 `FULL_DEBUG` profile、motion direction guard 阈值，以及 `mode30` 单舵轮 RPM + `VESC_RPM_CONTROL_PID_CURRENT` 组合。
- 继续路径规划时，优先围绕 `APP_Path.*`、`APP_Speedplanner.*` 和 `omni_chassisSetup.*` 做 MDK 与实车路径复核。
- 继续调试手感与观测链时，优先确认整车 debug deadzone 和单舵轮 debug deadzone 的板上表现。
- 继续 homing / 舵向恢复链时，优先看 `steer_fault_homing_recovery`、反向搜索三边沿确认、`first_boot_homing_delay_` 的运行态，以及首次上电延时是否按 `pending / active / elapsed_ms` 推进。
- 继续光电门回零搜索方向时，优先核对 `JIA_CHASSIS_HOMING_SEARCH_RPM_SIGN[4]` 和轮位顺序 `0=左前, 1=左后, 2=右后, 3=右前`，保持 `0 + 2` 反转、`1 + 3` 同向；搜索 RPM 符号测试现在跟随配置，不再绑定固定默认值。
- 继续协议或上位机语义时，优先核对 AirJoy/Lora 输入链、`SingleWheelTrace` 和当前保留的 trace 解释口径。
- 继续固件瘦身时，保持 `RUNTIME_MIN` slim smoke 与当前默认 `FULL_DEBUG` host 语义套件分层，不要把调试语义回流到 slim smoke。
