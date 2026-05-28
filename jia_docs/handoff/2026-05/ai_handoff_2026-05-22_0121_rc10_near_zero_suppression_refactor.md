# RC10 四舵轮工程 AI 交接（2026-05-22 01:21）

生成时间：2026-05-22 01:21（Asia/Shanghai）  
仓库路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork`  
当前分支：`Jia6_temp`  
当前 HEAD：`06304d1`  

工作区状态：

- 当前代码与宿主测试改动已完成本地回归验证
- 本轮准备提交的主线文件：
  - `User/Setup/Inc/chassis.h`
  - `User/Setup/Src/chassis.cpp`
  - `jia_docs/tests/tdd/chassis_semantics/test_chassis_semantics.cpp`
  - `jia_docs/plan.txt`
- 本轮额外同步正式交接文档与入口：
  - `jia_docs/handoff/2026-05/ai_handoff_2026-05-22_0121_rc10_near_zero_suppression_refactor.md`
  - `jia_docs/README.md`
  - `jia_docs/handoff/INDEX.md`

## 1. 本轮修改总览

本轮主线已从旧的 `Drive Attenuation / Drive Gate / VectorConsistency / StopSteerGuard` 多套并行语义，收口为更清晰的近零抑制链路：

1. `Drive Attenuation` 多模式体系收缩为：
   - `enable_low_speed_drive_suppression + low_speed_drive_suppression`
   - `enable_high_speed_drive_suppression + high_speed_drive_suppression`
2. 低速抑制与高速抑制统一复用 `NearZeroThresholdConfig enter/exit` 做边界滞回
3. 删除停车转向保护 `Stop Steer Guard` 整套逻辑
4. 宿主测试整体迁移到“低速/高速抑制 + near-zero 滞回 + X-Park 启动保持”新语义

## 2. 配置模型变化

- 删除旧的模式与策略枚举/配置：
  - `DriveAttenuationMode`
  - `DriveGateScope`
  - `AdaptiveGatePhase`
  - `StopSteerGuardStrategy`
  - `Cosine/Hard/Soft/Continuous/Adaptive` 参数块
  - `VectorConsistencyConfig`
- `StrategyConfig` 当前对外应读取的新口径为：
  - `near_zero_cfg_`
  - `enable_low_speed_drive_suppression`
  - `low_speed_drive_suppression`
  - `enable_high_speed_drive_suppression`
  - `high_speed_drive_suppression`
- `NearZeroThresholdConfig` 已成为以下逻辑的统一边界基准：
  - X-Park 静止判定
  - 低速抑制残余速度旁路
  - 高速抑制速度启停门

## 3. 行为变化

- 低速驱动抑制已收口为“近零/起步找向压驱动”：
  - 按四轮最大舵角误差计算统一缩放
  - 不再支持 per-wheel scope、soft/curve/cosine/adaptive 变体
- 低速抑制新增残余速度旁路滞回：
  - 高残余速度时旁路
  - 回落后按 near-zero enter/exit 恢复
- 高速抑制由原 `vector consistency gate` 改名收口为 `updateHighSpeedDriveSuppression(...)`：
  - 先过 near-zero 平移速度门
  - 再按方向误差与 ETA 进入/退出
  - 最后按 ramp 收放
- `launch hold` 触发条件改为依赖低速抑制语义：
  - 只看 `enable_low_speed_drive_suppression`
  - `low_speed_drive_suppression.min_scale`
  - `low_speed_drive_suppression.close_angle_deg`
- 停车转向保护混合修正路径已整套删除：
  - 不再根据残余速度对停车阶段舵角目标做二次 blend
  - `residual_drive_is_moving` 与 `getStopGuardReleaseSpeedMps()` 链路已移除

## 4. 调试与运行态变化

- 规划输出与运行态命名已统一到新口径：
  - `gate_or_cos_scale` -> `low_speed_suppression_scale`
  - `vector_gate_*` -> `high_speed_*`
- 运行时状态新增：
  - `high_speed_trans_gate_active_`
  - `low_speed_residual_bypass_active_`
- 运行时状态删除：
  - `adaptive_gate_scale_`
  - `adaptive_gate_phase_`
  - `hard_gate_bypassed_by_residual_speed_`
- `debug_mirror_` 当前重点观察量：
  - `high_speed_drive_suppression_scale`
  - `high_speed_dir_err_deg`
  - `high_speed_eta_max_s`
  - `high_speed_drive_suppression_active`
  - `low_speed_drive_suppression_bypassed_by_residual_speed`
  - `max_residual_speed_m_s`
- 已移除：
  - `nz_stop_guard_release_m_s`
  - 旧 `vec_*`
  - 旧 `hard_gate_bypassed_*`

## 5. 测试变化

- 用例体系从“DriveAttenuation 多模式”迁移到“低速/高速抑制双开关”语义
- 已删除或迁移：
  - `kNone / kCosine / kSoftGate / kContinuousCurve / kAdaptiveGate` 相关测试
  - stop-guard 相关测试尾巴
- 已新增关键覆盖：
  - 低速抑制可关闭后不缩放
  - 低速抑制残余速度旁路与恢复
  - 低速抑制在 near-zero 滞回带内不反复重入
  - 低速抑制采用全局最差轮误差
  - 高速抑制在 near-zero exit 前不启用
  - 高速抑制启用后可收紧并释放
- 现有关键回归仍通过：
  - X-Park 进入延时与退出边界
  - X-Park 起步 `launch hold`
  - `drive omega` uniform scale
  - `drive alpha` shared scale

## 6. 验证结果

- 宿主 TDD：通过
  - `powershell -ExecutionPolicy Bypass -File jia_docs\tests\tdd\chassis_semantics\run_test.ps1`
- 当前仍存在宿主编译 warning：
  - 来自 `APP_Utils.h` 中 `arm_sin_f32 / arm_cos_f32` 的 `constexpr` warning
  - 本轮未处理，不影响当前宿主测试通过

## 7. 当前统一口径

- 当前底盘近零区相关主线，已经统一为：
  - `NearZeroThresholdConfig`
  - `low_speed_drive_suppression`
  - `high_speed_drive_suppression`
  - `X-Park / launch hold`
- 后续若再调 near-zero 区域行为，优先从以下顺序排查：
  1. `near_zero_cfg_.base_enter_m_s`
  2. `near_zero_cfg_.base_exit_m_s`
  3. `low_speed_drive_suppression.close_angle_deg / min_scale`
  4. `high_speed_drive_suppression.dir_err_* / eta_* / ramp_*`
- 不再从旧 `DriveAttenuationMode / HardGate / VectorConsistency / StopSteerGuard` 概念出发分析现状

## 8. 一句话交接结论

> RC10 当前已完成“近零边界统一 + 低速/高速抑制双段收口 + 停车转向保护删除 + 宿主测试迁移”，后续联调应统一按 `NearZeroThresholdConfig + low/high suppression + X-Park launch hold` 这一套新口径工作。
