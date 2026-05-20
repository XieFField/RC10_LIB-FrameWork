# FourSteer 调试 PID 读参链路重构上下文交接文档

生成时间：2026-05-15 22:00（Asia/Shanghai）  
仓库路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork`  
当前分支：`Jia6_temp`  
HEAD：`5264c3d`  
文档路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\jia_docs\handoff\ai_handoff_2026-05-15_2200_pid_debug_readback.md`

## 1. 本轮目标与实际完成情况

- 目标：
  - 解决 FourSteer 调试 PID 时“看不到运行真值、缓存可能漂移”的问题。
  - 实现“先读后改”调参链路，并消除 `M3508` 内对 PID 参数的重复缓存。
- 实际完成（已完成）：
  - 调试使能上升沿同步：进入 debug enable 时，从电机运行态回读 PID 到 `debug_pid_tune_`。
  - 未 apply 改动保护：若某轮 `apply_stamp != applied_stamp`，该轮跳过回读覆盖。
  - PID 读参去重：`td_ratio / i_separa` 改为从 PID 类直接读取，不再由 `M3508` 再存一份。
  - 提交已完成：`5264c3d`。
- 实际完成（部分完成）：
  - 已完成代码级静态核对，但未执行 Keil 编译和板测。
- 本轮未做：
  - 未修改 PID 算法与控制律。
  - 未调整 mode20/mode30 的外部调试语义。
  - 未做硬件实车验证。

## 2. 关键状态结论（当前真实状态）

- 结论1：调试 PID 已支持“enable 上升沿自动回读基线”。
  - 依据：`User/Setup/Src/chassis.cpp:1387`、`User/Setup/Src/chassis.cpp:2036`。
- 结论2：回读会保护未 apply 的人工改动，不会覆盖调试器刚写入值。
  - 依据：`User/Setup/Src/chassis.cpp:2059`~`2064`。
- 结论3：`td_ratio` 与 `i_separa` 已从 PID 类本体直读。
  - 依据：`RC10_LIB/Motor/Src/Motor_DJI.cpp:280`、`RC10_LIB/Motor/Src/Motor_DJI.cpp:285`。
- 结论4：PID 类已暴露对应 getter。
  - 依据：`RC10_LIB/APP/Inc/APP_PID.h:105`、`RC10_LIB/APP/Inc/APP_PID.h:201`。
- 结论5：当前工作区干净，可作为后续工作基线。
  - 依据：`git status --short` 结果为空（本次文档生成时刻）。

## 3. 关键代码改动（按模块）

### 3.1 PID 层（参数真值可观测）

- 改了什么：
  - `PID_Position` 新增 `get_i_separa_threshold() const`。
  - `PID_Incremental` 新增 `get_td_ratio() const`。
- 为什么改：
  - 让 `td_ratio/i_separa` 与 `params` 一样可从源头读取，避免中间层二次缓存。
- 影响范围：
  - 仅新增只读接口，不改变 `set_params` 或计算流程。
- 涉及文件：
  - `RC10_LIB/APP/Inc/APP_PID.h`

### 3.2 电机层（M3508 读参去重）

- 改了什么：
  - `M3508` 保留对外 getter 签名，内部改为直接转发 PID getter。
  - 删除 `speed_pid_td_ratio_`、`angle_pid_i_separa_threshold_` 成员。
  - `pid_init()` 去除缓存赋值，仅保留 `set_params(...)`。
- 为什么改：
  - 消除双份状态，降低“写一处、读另一处”不同步风险。
- 影响范围：
  - 行为等价于读取运行态真值；接口兼容。
- 涉及文件：
  - `RC10_LIB/Motor/Inc/Motor_DJI.h`
  - `RC10_LIB/Motor/Src/Motor_DJI.cpp`

### 3.3 底盘调试链路（上升沿同步）

- 改了什么：
  - 新增 `syncDebugSteerPidTuneFromRuntimeOnEnableEdge()` 和 `syncDebugSteerPidTuneFromRuntime()`。
  - 在 `isDebugMode()` 前置调用上升沿同步。
  - 新增观察字段：
    - `debug_enable_last_cycle_`
    - `debug_pid_tune_.synced_on_enable_edge`
- 为什么改：
  - 进入调试时先拉取当前运行参数，调参起点更可靠。
- 影响范围：
  - 不改变 `apply_stamp/applied_stamp` 的写入生效机制。
- 涉及文件：
  - `User/Setup/Inc/chassis.h`
  - `User/Setup/Src/chassis.cpp`

## 4. 已验证结果

### 4.1 提交与工作区状态验证（已通过）

- 命令：
  - `git branch --show-current`
  - `git rev-parse --short HEAD`
  - `git status --short`
- 结果：
  - 分支：`Jia6_temp`
  - HEAD：`5264c3d`
  - 工作区：干净

### 4.2 关键符号连通性验证（已通过）

- 命令：
  - `rg -n "get_i_separa_threshold|get_td_ratio|..."`
- 结果：
  - `chassis` 同步函数 -> `M3508` getter -> PID getter 链路可追溯，符号存在且行号稳定。

### 4.3 编译/功能/硬件验证（未执行）

- 未执行：
  - `MDK-ARM/Frame_T.uvprojx` 编译验证
  - host 行为回归测试
  - 实车板测

## 5. 风险点与未完成项

- 风险1：把“静态核对通过”误当“工程编译通过”。
  - 后果：接手人误以为可直接上板。
  - 建议：先做 Keil 编译，再做最小调试路径验证。
- 风险2：调试器在线改参与自动同步时序理解错误。
  - 后果：误判参数被覆盖。
  - 建议：先理解 `dirty` 判定（`apply_stamp != applied_stamp`）后再操作。
- 风险3：文件编码与行尾策略（历史上有 BOM/编码污染经验）。
  - 后果：大范围注释乱码或无意义 diff。
  - 建议：改注释前先确认编码，避免批量转码工具直接覆盖。

## 6. 接手第一小时执行清单

1. 验证仓库状态：
   - `git status --short`
   - `git log --oneline -n 3`
2. 验证关键链路符号：
   - `rg -n "syncDebugSteerPidTuneFromRuntimeOnEnableEdge|get_speed_pid_td_ratio|get_td_ratio" ...`
3. 编译验证（只验证不改动）：
   - 使用 `MDK-ARM/Frame_T.uvprojx` 做一次完整构建。
4. 调试语义走查：
   - 查看 `User/Setup/Src/chassis.cpp:2036`~`2077`，确认上升沿同步与 dirty 保护逻辑。
5. 若编译通过，再安排最小板测：
   - 仅验证“enable 上升沿回读是否生效”“未 apply 改动是否被保护”。

## 7. 回退与兜底策略

- 若出现调试链路异常（仅针对本轮改动）：
  - 回退到 `5264c3d^` 查看前一版本行为。
- 不可动区域（排障前不要改）：
  - `apply_stamp/applied_stamp` 判定语义
  - `isDebugMode()` 的主模式解析分支
  - 轮级 `dirty` 保护逻辑

## 8. 一句话交接结论

> 当前代码已完成 FourSteer 调试 PID 的“先读后改”与读参去重重构，接口兼容且工作区干净；但 Keil 编译与实车验证尚未执行，接手后应先做编译与最小板测闭环。
