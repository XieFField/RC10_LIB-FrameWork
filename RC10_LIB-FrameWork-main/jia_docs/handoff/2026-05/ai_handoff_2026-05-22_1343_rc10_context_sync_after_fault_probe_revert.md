# RC10 底盘 AI 交接（2026-05-22 13:43）
生成时间：2026-05-22 13:43（Asia/Shanghai）  
仓库路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork`  
当前分支：`Jia6_temp`  
当前 HEAD：`2f698ee`  

## 1. 当前代码状态

当前工作区是干净的，最近两次关键提交为：

- `2f698ee` `tune(底盘): 调整舵向角加速度默认限幅`
- `b7b86d5` `refactor(底盘): 收口近零抑制链路并删除停车转向保护`

当前底盘主线仍然是：

- 低速抑制 / 高速抑制双层结构
- `NearZeroThresholdConfig` 统一近零边界
- X-Park 与 launch hold 保留
- Stop Steer Guard 已删除

## 2. 这几轮已经稳定落地的内容

### 2.1 近零与抑制链路

已经稳定落地并提交的主线：

- 旧 `DriveAttenuationMode + 多种 gate 变体` 已收口
- 当前只保留：
  - `enable_low_speed_drive_suppression`
  - `low_speed_drive_suppression`
  - `enable_high_speed_drive_suppression`
  - `high_speed_drive_suppression`
- `NearZeroThresholdConfig` 统一服务于：
  - 近零滞回
  - 低速抑制边界
  - 高速抑制启停
  - X-Park 静止判定

### 2.2 Stop Steer Guard

停车转向保护整套已删除，不再保留：

- `enable_stop_steer_guard`
- `StopSteerGuardStrategy`
- `stopSteerGuardBlend(...)`
- `residual_drive_is_moving`
- `getStopGuardReleaseSpeedMps()`

### 2.3 X-Park / launch hold

当前仍保留并通过宿主测试：

- X-Park 进入延时
- X-Park 退出边界
- X-Park 起步整车 launch hold

### 2.4 drive 限幅统一语义

当前仍保留并通过宿主测试：

- `drive omega` uniform scale
- `drive alpha` shared scale
- delivered / planned continuity 相关回归

### 2.5 本轮最新已提交小改动

已提交到 `2f698ee`：

- [User/Setup/Inc/chassis.h](D:/desktop/2026RC/Control/2026RC-Team1-R1Code/RC10_LIB-FrameWork/User/Setup/Inc/chassis.h)
  - `enable_steer_alpha_limit_` 默认改为 `true`
  - `max_steer_alpha_rad_s2_` 默认值改为 `20000.0f`
- [MDK-ARM/Frame_T.uvprojx](D:/desktop/2026RC/Control/2026RC-Team1-R1Code/RC10_LIB-FrameWork/MDK-ARM/Frame_T.uvprojx)
  - 工程优化等级从 `2` 改为 `3`
- [User/Setup/Src/chassis.cpp](D:/desktop/2026RC/Control/2026RC-Team1-R1Code/RC10_LIB-FrameWork/User/Setup/Src/chassis.cpp)
  - 仅去掉文件头 BOM，无功能语义改动

## 3. 已尝试但已经完全回退的内容

这部分非常重要，下一位 agent 不要把它当成当前代码基线。

本轮曾尝试在底盘层加入“舵向电机断开/卡死检测 + 恢复后自动整车 rehome”方案，包括：

- 公开接口 `requestSwerveOpticalRehome()`
- `SteerFaultType`
- 每轮故障检测状态
- 静止冻结 / 动作期无响应 / 大电流不动 / 跳变四种触发
- 故障保护
- 恢复 probe
- 自动 rehome
- 对应 debug mirror
- 对应宿主测试桩和测试用例

这整套现在已经全部回退，没有保留在当前代码里。

回退原因不是“完全不可行”，而是当前用户决定不要这轮方案继续留在主线里。

## 4. 对故障检测尝试的排查结论

虽然代码已经回退，但有两个排查结论值得留给下一位 agent：

### 4.1 宿主测试原本不能真实覆盖完整 homing 边沿流程

原因：

- `jia_docs/tests/tdd/chassis_semantics/stubs/main.h` 里原始 `HAL_GPIO_ReadPin()` 固定返回低电平
- 所以此前宿主测试实际上无法真实模拟 `Search -> EdgeDetected -> ... -> Ready`

这说明如果下一轮再碰 homing / 光电校准链路，应该优先补“可控光电输入桩”。

### 4.2 自动 rehome 等待态最容易卡死的位置

本轮排查时确认过：

- `waiting_rehome_complete_`
- `steer_fault_protection_active_`
- `all_homed`
- 各轮 `homing_state`

是最关键的卡点。

尤其是一个很重要的实现教训：

- 如果“等待自动 rehome 完成”直接拦截掉 `applyModuleCommands()` 顶层
- 那么 homing 搜索命令本身也可能发不下去
- 结果就是状态机看起来在等 rehome，但实际上根本没法继续推进

这条经验现在只存在于交接文档里，不在当前代码里。

## 5. 当前建议读取口径

下一位 agent 若继续分析底盘近零 / 抑制 / X-Park，请统一按下面口径理解现状：

- `near_zero_cfg_`
- `enable_low_speed_drive_suppression`
- `low_speed_drive_suppression`
- `enable_high_speed_drive_suppression`
- `high_speed_drive_suppression`
- `idle_posture_mode`
- `xpark_entry_delay_ms`

不要再按这些旧概念继续理解当前代码：

- `DriveAttenuationMode`
- `HardGate / SoftGate / Curve / Adaptive`
- `VectorConsistency`
- `StopSteerGuard`

## 6. 当前测试状态

最近一次明确执行并通过的宿主测试命令：

```powershell
powershell -ExecutionPolicy Bypass -File jia_docs\tests\tdd\chassis_semantics\run_test.ps1
```

结果：`PASS`

当前仍存在但未处理的 warning：

- `APP_Utils.h` 中 `arm_sin_f32 / arm_cos_f32` 的 `constexpr` warning

这不是本轮重点，也不影响宿主测试通过。

## 7. 如果新 agent 继续干什么，最推荐从哪开始

### 推荐方向 A：继续做实车行为解释与参数梳理

优先做：

- 近零 / 低速抑制 / 高速抑制 / X-Park 的链路说明
- 关键调参项职责划分
- 现有实车异常的证据化测试

### 推荐方向 B：如果重启“舵向故障检测”方案

不要直接接着旧对话印象写代码，建议按下面顺序重开：

1. 先补可控 photogate 宿主测试桩
2. 先做完整 homing 状态推进测试
3. 再做静止电流抖动不误判测试
4. 最后才重新设计故障检测与自动 rehome

### 明确不建议

- 不要假设当前主线里还保留 `requestSwerveOpticalRehome()` 或 `SteerFaultType`
- 不要在没有重新补测试桩的前提下声称“完整 homing 已被宿主测试覆盖”

## 8. 一句话交接结论

> 当前 RC10 主线已经稳定收口在“近零统一边界 + 低速抑制 / 高速抑制 + X-Park / launch hold”这套结构上；舵向电机断线/卡死检测方案只是尝试过并已全部回退，下一位 agent 如果要重开这条线，应从可控 homing 测试桩重新起步。
