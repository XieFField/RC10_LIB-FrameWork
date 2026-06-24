# AI handoff 2026-06-24 RC10 chassis AirJoy / homing / debug context sync

本页用于把当前仍然有效的底盘主线状态重新收口成新的默认交接入口。它接在 `2026-06-09` path / yaw / homing / build 主线 handoff 之后，重点补齐最近这几轮底盘输入链、首次上电 homing 延时、debug 摇杆 deadzone、默认宏基线，以及当前 host 语义测试状态。

## 当前结论

- 底盘线程内部的遥控缓存类型已经从 `RmPocketData_t` 切换到 `communication::RC10_AirJoy_Data_S`。
- `runThread()` 当前通过 `communication::Lora_communication::GetInstance()->update_airjoy_data(&airjoy_data_)` 主动拉取 AirJoy 数据，不再在底盘线程里直接使用 `CrsfReceiver::getControlData(...)`。
- 首次上电整车 homing 延时已经落地到 chassis 级统一运行态 `first_boot_homing_delay_`，由 `pending / active / elapsed_ms` 三个字段描述是否尚未消耗、是否正在等待，以及已经累计了多少毫秒。
- debug 摇杆 deadzone 已改成“死区内归零，出区后从死区边界重新起算”的重映射语义：
  - 整车 debug 接管链按平移组/旋转组分 deadzone。
  - mode30 单轮 debug 继续使用 `single_wheel.input_deadzone`，但行为也升级为同样的重映射语义。
- 这套 debug deadzone 只作用于 debug 链，不影响正常非 debug 的整车控制链路。
- 当前 `chassis.h` 的默认宏基线已经切换为：
  - `JIA_CHASSIS_PROFILE = JIA_CHASSIS_PROFILE_RUNTIME_MIN`
  - `JIA_CHASSIS_HOMING_SEARCH_RPM = 100.0f`
  - `JIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_MS = 500U`

## 代码语义现状

### 1. 遥控输入链

- `airjoy_data_` 当前是 `communication::RC10_AirJoy_Data_S`。
- 底盘线程每拍都会同步一次 AirJoy 缓存，供 `setModeFlag()`、`resolvePlannerTargetData()` 和 debug 注入路径直接消费。
- 当前真正使用的手柄字段仍然集中在：
  - `left_x`
  - `left_y`
  - `right_x`
  - `right_y`

### 2. 首次上电 homing 延时

- 首次上电延时只作用于“本次上电后的第一次整车 homing”。
- 延时窗口内四个舵轮都会停在 `kIdle`，不会进入 `kSearch`，也不会下发搜索 RPM。
- 后续再次手动 `startHoming()` 不再等待；steer fault 触发的 recovery re-home 也不走这道延时门。
- 调试器里如果要确认这段延时有没有正常执行，优先观察：
  - `first_boot_homing_delay_.pending`
  - `first_boot_homing_delay_.active`
  - `first_boot_homing_delay_.elapsed_ms`
  - `homing_start_request_`
  - `wheel_config_[0..3].homing_state`

### 3. Debug 摇杆 deadzone

- 整车 debug 接管链当前先对摇杆输入做 deadzone 重映射，再生成：
  - `target_vel_x`
  - `target_vel_y`
  - `target_omega_z`
  - `kSteerDegAndDriveSpeed` 下的舵角/驱动速度目标
- 单轮 debug 的连续输入和 step 判定都已经改为基于重映射后的轴值。
- 当前相关运行态字段是：
  - `debug_control_.injection.translation_input_deadzone`
  - `debug_control_.injection.rotation_input_deadzone`
  - `debug_control_.single_wheel.input_deadzone`

## 当前默认基线

如果下一位接手者什么都不改、直接按当前源码理解，默认基线是：

- `JIA_CHASSIS_PROFILE = JIA_CHASSIS_PROFILE_RUNTIME_MIN`
  - 默认固件按瘦身档理解，不把 `FULL_DEBUG` 调试语义默认带进发布档。
- `JIA_CHASSIS_HOMING_SEARCH_RPM = 100.0f`
  - 当前舵向回零搜索转速默认基线已经提到 100。
- `JIA_CHASSIS_FIRST_BOOT_HOMING_DELAY_MS = 500U`
  - 当前默认首次上电整车 homing 会先等待 500ms 再统一放行。

这些值属于当前代码事实，不代表板上联调已经证明它们是最终最优值。后续如需继续联调，应该把“当前默认值”和“板上最终推荐值”明确区分。

## 验证入口与当前状态

当前继续接手时，默认从这些入口验证：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs/tests/run_tests.ps1
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_main.ps1
powershell -ExecutionPolicy Bypass -File jia_docs/tests/host_cpp/chassis_semantics/run_slim_smoke.ps1
```

本轮交接时，已明确同步到文档中的最近验证状态是：

- `jia_docs/tests/host_cpp/chassis_semantics/run_main.ps1`：通过
- `jia_docs/tests/host_cpp/chassis_semantics/run_slim_smoke.ps1`：通过

其中最近一轮与当前底盘输入链、首次上电延时、debug deadzone 同步后的 host 基线是：

- `run_main.ps1`：`199 passed`
- `run_slim_smoke.ps1`：`sizeof(Chassis)=2736`

## 与上一阶段 handoff 的关系

这份文档现在替代 `2026-06-09` 那份文档，成为新的默认主线交接入口；但它不是要覆盖历史，而是把仍然有效的主线状态重新编排一遍，方便下一位接手者不用自己把多份 handoff 再拼起来。

继续向前追背景时，推荐顺序是：

1. 本页：AirJoy / homing delay / debug deadzone / 当前默认宏基线
2. [ai_handoff_2026-06-09_rc10_path_yaw_homing_build_sync.md](ai_handoff_2026-06-09_rc10_path_yaw_homing_build_sync.md)：AAA-Path、yaw lock 修复链、homing 三边沿确认、debug 状态收口与 MDK 编译恢复
3. [ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md](ai_handoff_2026-06-05_rc10_chassis_doctest_zero_stop_sync.md)：doctest 分片、zero-stop / X-Park、RUNTIME_MIN / FULL_DEBUG 分层
4. [../2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md](../2026-05/ai_handoff_2026-05-31_rc10_wait_1_7_7_1_6_1_merge.md)：上一阶段 wait merge 主线

## 后续接手关注点

- 板上继续联调首次上电 homing 延时，重点核对 `500U` 是否真的合适，必要时再和 `homing_search_rpm = 100.0f` 一起联调。
- 若继续看 debug 手感，优先确认 deadzone 重映射对：
  - `applyDebugTargetOverride()` 整车接管链
  - `mode30` 单轮 debug
  的手感是否符合预期。
- 若继续看遥控输入链，默认应以 AirJoy/Lora 这条链为主，不要再按旧 CRSF 直取口径理解底盘线程。
- 若继续整理文档或接手主线状态，优先维护本页和 `active/` 入口层，不要再让最新入口停留在 2026-06-09 的旧状态。
