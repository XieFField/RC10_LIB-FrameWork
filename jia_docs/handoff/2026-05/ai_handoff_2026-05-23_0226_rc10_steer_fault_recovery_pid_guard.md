# RC10 底盘 AI 交接（2026-05-23 02:26）

生成时间：2026-05-23 02:26（Asia/Shanghai）  
仓库路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork`  
当前分支：`Jia6_temp`  
当前 HEAD：`2f698ee`  

## 1. 本轮主题一句话总结

本轮把 RC10 四舵轮底盘补成了“舵向断链检测 -> 故障锁存整车停机 -> 链路恢复后仅故障轮 rehome -> 恢复正常控制”的闭环，并针对重连疯转风险加入了“锁故障即清舵向闭环残留 + fault 锁存阶段纯 `current=0`”保护。

## 2. 当前底盘稳定语义

### 2.1 舵向故障检测口径

- 只检测舵向链路，不处理 drive 链路 rehome。
- 每周期读取舵向 `getCurrent()` 与 `getTotalAngle()`。
- 故障锁存判据是“电流冻结 + 角度冻结”同时持续达到阈值。
- 恢复判据是 fault 锁存后电流反馈重新连续跳动，达到 `recovery_toggle_threshold`。

### 2.2 整车停机与恢复策略

- 任一轮舵向故障时，整车 drive 全部下发 `current=0`。
- 故障恢复后只让故障轮重新进入 homing，其他轮保持原有 ready / homed 状态。
- 故障轮 rehome 完成前，整车仍保持 blocked，不恢复正常运动控制。
- 如果恢复后的 rehome 再次超时，会重新回到 fault latch。

### 2.3 关键保护逻辑

- 本轮最终方案是“进入断开状态即清舵向闭环状态”，不是“恢复时再清并额外等待一拍”。
- `latchSteerFault()` 内会立即调用 `resetSteerMotorClosedLoopState(...)`，清理 `M3508` 的 speed PID、angle PID 和对应时间计数。
- fault 锁存普通分支现在只允许故障轮舵向 `setTargetCurrent(0)`，不再继续用 `setTargetRPM(0)` / `setTargetTotalAngle(0)` 覆盖模式。
- 这点很关键，因为之前虽然调用过 `setTargetCurrent(0)`，但后续又被非电流模式覆盖，最终并不是纯 `CURRENT_CONTROL`。

## 3. 关键实现结论

- 恢复时不需要为了“等 PID 清空”再人为多等一拍，因为 PID 已经在 fault latch 的第一时间被清掉。
- 断链锁存期间不能再把舵向命令写成 `RPM=0` 或 `TotalAngle=0`，否则会重新打开底层闭环路径，破坏“纯 current=0”保护语义。
- 若后续还要继续排查“实车重连瞬间快速抽一下”的问题，优先看 `kSearch` 恢复当拍与底层电机 `update()` 的真实节拍关系，而不是先改 fault 状态机大方向。

## 4. 测试结论与证据链

### 4.1 宿主回归

执行：

```powershell
powershell -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\jia_docs\tests\tdd\chassis_semantics\run_test.ps1
```

结果：`PASS`

宿主回归已覆盖的核心点包括：

- 舵向电流冻结 + 角度冻结触发单轮 fault latch。
- 任一轮 fault latch 后整车 drive 全停。
- fault 锁存期间故障轮保持纯 `current=0`，不再用 `RPM/TotalAngle` 覆盖控制模式。
- 电流恢复跳动后仅故障轮重入 homing，其他轮保持 ready。
- 故障轮 homing 完成前整车仍不恢复。
- 恢复后若 rehome 再次超时，会重新锁 fault。
- X-Park / near-zero / suppression 现有语义没有被新的 fault 闭环破坏。

### 4.2 独立 PID reconnect 风险验证

执行：

```powershell
powershell -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\jia_docs\tests\tdd\chassis_semantics\run_pid_reconnect_test.ps1
```

结果：`PASS`

独立 PID reconnect 测试给出的数值证据是：

- `pid_reconnect_current_before_disconnect=4199.298828`
- `pid_reconnect_first_current_after_reenable=-6661.386719`

这个测试证明的是：如果重新打开底层速度环，第一拍确实会出现明显的 current jump 风险。  
它证明的是“风险入口存在”，不是直接证明“实车一定疯转”，但已经足够说明为什么底盘层必须在 fault latch 时提前清理闭环状态。

## 5. 本轮文件主线

### 5.1 底盘逻辑

- `User/Setup/Inc/chassis.h`
- `User/Setup/Src/chassis.cpp`

核心内容：

- `SteerFaultState`
- 每轮 fault 运行态字段
- `SteerFaultConfig`
- `debug_mirror_` 的 fault 诊断观测项
- 锁 fault 即清闭环残留
- fault 锁存阶段纯 `current=0`

### 5.2 测试与证据

- `jia_docs/tests/tdd/chassis_semantics/test_chassis_semantics.cpp`
- `jia_docs/tests/tdd/chassis_semantics/test_steer_pid_reconnect.cpp`
- `jia_docs/tests/tdd/chassis_semantics/run_test.ps1`
- `jia_docs/tests/tdd/chassis_semantics/run_pid_reconnect_test.ps1`
- `jia_docs/tests/tdd/chassis_semantics/stubs/main.h`
- `jia_docs/tests/tdd/chassis_semantics/stubs/stm32h7xx_hal.h`
- `jia_docs/tests/tdd/chassis_semantics/stubs/BSP_TimeStamp.h`
- `jia_docs/tests/tdd/chassis_semantics/stubs_src/test_host_globals.cpp`
- `jia_docs/tests/tdd/chassis_semantics/pid_reconnect_stubs/`

## 6. 下一位 agent 的阅读建议

建议顺序：

1. 先读本文件，统一当前稳定语义。
2. 再看 `jia_docs/tests/tdd/chassis_semantics/test_chassis_semantics.cpp` 里 fault / recovery / rehome 相关用例。
3. 再看 `jia_docs/tests/tdd/chassis_semantics/test_steer_pid_reconnect.cpp`，理解“为什么必须在 fault latch 就清 PID”。
4. 如继续排查实车疯转，优先核对 `kSearch` 恢复当拍与电机层 `update()` 节拍关系，以及实车 CAN 恢复时反馈刷新的先后顺序。

## 7. 一句话交接结论

> 当前 RC10 主线已经不再是“fault 探测尝试后回退”的状态，而是已经落成了可测试的舵向断链检测与恢复闭环；其中最关键的保护结论是：fault 锁存时必须立即清舵向闭环残留，并且锁存阶段只能保持纯 `current=0`，不能再用 `RPM=0 / TotalAngle=0` 覆盖模式。
