# RC10 四舵轮工程 AI 交接（2026-05-21 12:01）

生成时间：2026-05-21 12:01（Asia/Shanghai）  
仓库路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork`  
当前分支：`Jia6_temp`  
当前 HEAD：`1b35c8a`  

工作区状态：

- 当前任务相关代码与宿主测试已提交到 `1b35c8a`
- 工作区仍保留两个无关 IDE 脏文件：
  - `MDK-ARM/Frame_T.uvoptx`
  - `MDK-ARM/Frame_T.uvprojx`
- 另有一个未提交的本地运行默认值调参差异：
  - `User/Setup/Inc/chassis.h`
  - 当前仅涉及默认参数：
    - `drive_gate_close_angle_deg: 0.1 -> 1.0`
    - `vector_consistency.enable: true -> false`
- 上述 `chassis.h` 本地差异未纳入 `1b35c8a`，后续若要保留，应单独评审其作为默认值是否合理
- 上述两个文件不属于本轮任务产物，也不应被描述为当前底盘控制改动

## 1. 本轮新增能力

- 执行层 `drive release continuity` 已收口：
  - `storePlannedActuatorFrame()` 不再推进 drive 历史状态
  - `applyModuleCommands()` 成为 drive 最终下发、alpha limit、历史回写的唯一收口层
- `HardGate` 新增独立整车最大残余速度禁入阈值：
  - `drive_gate_hard_disable_residual_speed_m_s`
- `computeDriveGateScales()` 已改为直接消费 `SwervePlannerInput`
- `debug_mirror_` 已新增 drive 释放诊断量：
  - `planned_drive_target_rpm`
  - `delivered_drive_target_rpm`
  - `hard_gate_bypassed_by_residual_speed`
  - `max_residual_speed_m_s`
- 宿主 TDD 已新增覆盖：
  - drive 抑制期间不偷跑历史状态
  - drive 释放后从 delivered 状态继续加速
  - HardGate 在高残余速度下旁路、降回阈值内重新启用
  - Soft/Curve gate 不受该阈值影响
  - planned / delivered 诊断字段分离

## 2. 已确认验证状态

- MCU 宿主 TDD：通过
  - `powershell -ExecutionPolicy Bypass -File jia_docs\tests\tdd\chassis_semantics\run_test.ps1`
- 当前最新功能提交：
  - `1b35c8a`：收口驱动释放连续性并新增 HardGate 残余速度阈值
- 仍存在宿主编译 warning：
  - 来自 `APP_Utils.h` 中 `arm_sin_f32` / `arm_cos_f32` 的 `constexpr` warning
  - 本轮未处理，且不影响当前 TDD 结果

## 3. 当前实车联调重点

1. 释放首拍体感是否仍异常  
   重点区分“planner 放得快”还是“delivered 真没按 alpha limit 起步”
2. `planned_drive_target_rpm` 与 `delivered_drive_target_rpm` 是否分离正常  
   若前者涨得快而后者仍平滑，说明执行层连续性是对的，体感异常更可能来自 planner gate re-entry
3. `max_residual_speed_m_s` 超阈值时 `hard_gate_bypassed_by_residual_speed` 是否按预期为真  
   若高残余速度下仍被硬掐，优先检查阈值设置与当前策略是否确实为 `kHardGate`

## 4. 当前统一口径

- 释放异常优先沿 MCU 数据链定位，不先改面板显示补偿
- 先看：
  1. `planned_drive_target_rpm`
  2. `delivered_drive_target_rpm`
  3. `hard_gate_bypassed_by_residual_speed`
  4. `max_residual_speed_m_s`
- 若后续仍需进一步优化释放体感，下一步应先讨论 planner gate re-entry 速度，而不是回退当前执行层连续性修复

## 5. 关键代码观察点

- `makeSwervePlannerInput()`
- `computeDriveGateScales()`
- `applyModuleCommands()`
- `refreshDebugMirror()`
- `jia_docs/tests/tdd/chassis_semantics/test_chassis_semantics.cpp`

## 6. 回退锚点

- 当前推荐回退锚点：`1b35c8a`
- 上一阶段主链路锚点：
  - `c932ef9`
  - `c9787ca`

## 7. 一句话交接结论

> RC10 当前已经补齐“drive release continuity + HardGate 高残余速度旁路 + planned/delivered 诊断”，下一阶段应先拿这组诊断量做实车释放问题定位，而不是继续盲改显示层或回退执行层连续性修复。
