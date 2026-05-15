# 时间戳按时钟源收敛交接文档（RTOS+SysTick）

生成时间：2026-05-15 22:23（Asia/Shanghai）  
仓库路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork`  
当前分支：`Jia6_temp`  
文档路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\.ai_agent_workspace\handoff\ai_handoff_2026-05-15_2223_time_stamp_us64_port.md`

## 1. 本轮目标与结果

- 目标：将 RTOS+SysTick 的 64 位时间戳实现收敛到 BSP，并提升接口封装度。  
- 结果：已完成目录收敛、命名空间统一、接口收口，且不影响原有 `BSP_TimeStamp` 主链路。

## 2. 当前真实状态（关键结论）

1. TIM 时钟源时间戳仍由 `BSP_TimeStamp` 提供，业务主链路不变。  
2. RTOS+SysTick 实现位于：  
   - `RC10_LIB/BSP_Driver/Inc/BSP_RtosTimeStampUs64.h`  
   - `RC10_LIB/BSP_Driver/Src/BSP_RtosTimeStampUs64.cpp`  
3. 命名空间已统一为 `namespace jia`（不再使用 `jia::time`）。  
4. 对外接口仅保留：`RtosTimeStampUs64::getTimeUs()`。  
5. `ticksToUs64/composeTimeUs64` 已下沉到 `cpp` 匿名命名空间，仅内部可见。

## 3. 工程接线状态

- `MDK-ARM/Frame_T.uvprojx` 已包含：
  - `BSP_RtosTimeStampUs64.h`
  - `BSP_RtosTimeStampUs64.cpp`
- APP 层兼容头 `APP_TimeStampUs64.h` 已移除，不再保留 APP 时间戳入口。

## 4. 测试资产与验证

测试资产位置：  
`.ai_agent_workspace/tests/ai2_tests/time_stamp_us64/`

覆盖要点：  
1. 零除保护与整除换算  
2. 子 tick 截断语义  
3. 静态接口约束（`namespace jia` + `getTimeUs` 对外唯一入口）

建议执行：  
```powershell
python .ai_agent_workspace/tests/ai2_tests/time_stamp_us64/time_stamp_us64_regression.py
```

## 5. 使用建议（按时钟源）

- 需要 TIM 连续时基：继续使用 `TimeStamp::getInstance()`。  
- 需要 RTOS Tick + SysTick 组合时基：使用 `jia::RtosTimeStampUs64::getTimeUs()`。  

## 6. 一句话结论

> 时间戳模块已按“时钟源”完成结构收敛，RTOS+SysTick 侧接口做到零参数取值，调用方无需关心换算细节，维护边界更清晰。
