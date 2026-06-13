# RC10 底盘调试 merge baseline 语义收口交接

生成时间：2026-05-25 23:44（Asia/Shanghai）<br>
仓库路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork`

相关提交：
- `0aed4c23` `merge(底盘调试): 合并 jia/codex_2/3 并收入口 mode30 单轮调试主线`
- `828b8ee0` `merge(底盘调试): 合并 jia/codex_1/3 并统一输出家族与 mode1 跨零语义`

相关参考路径：
- `User/Setup/Inc/chassis.h`
- `User/Setup/Src/chassis.cpp`
- `jia_docs/tests/tdd/chassis_semantics/test_chassis_semantics.cpp`

## 一句话总结

当前 `jia/develop` 已经不是两套底盘调试入口并行存在的状态，而是完成了 `jia/codex_2/3 -> jia/codex_1/3` 顺序合并后的统一主线：调试入口语义以 `codex_2/3` 为准，调试输出家族重构和 `mode1` 跨零修复吸收自 `codex_1/3`，同时保留当前主线已经验证过的稳定默认值和保护语义。

## 当前 merge baseline 在解决什么

这次收口的目标不是继续引入新调试模式，而是把已经并入 `jia/develop` 的两批能力合成一套清晰、可追溯、可继续联调的底盘调试主线：

- 调试入口层只保留一套模式语义，不再让 `mode30` 一类入口在不同分支上各说各话。
- 调试输出层统一回到 `DebugOutputFamily + JustFloatProfile + Binary`，避免 `JustFloat`、文本和二进制输出各自维护一套 runtime 逻辑。
- 手操规划层继续沿用当前主线的 S 曲线默认值与跨零修复，不因为 merge 把反向换向语义冲掉。
- 保护层维持已有 fault gate、homing gate 和 `current = 0` 保护，不让调试链路反向覆盖控制主线。

## 语义归属与最终口径

### 1. `mode30 / mode31 / mode32` 以 `jia/codex_2/3` 为准

当前主线保留的是 `jia/codex_2/3` 收口后的模式号含义：

- `mode30`：单轮隔离直控入口，用于单轮调试与摇杆直控链路。
- `mode31`：full-gate 兼容入口，用于验证单轮调试经过完整 gate 之后的行为。
- `mode32`：旧执行层直控入口，保留给历史执行路径直通调试。

这部分同时带入了 `DebugControl` 的嵌套结构，把 `common / injection / single_wheel / legacy_direct` 的配置边界整理到同一处，后续如果继续扩展调试模式，应在这套边界内加，而不是再开平行入口。

### 2. `DebugOutputFamily + JustFloatProfile + Binary` 来自 `jia/codex_1/3`

输出侧最终采用 `jia/codex_1/3` 的重构口径：

- `DebugOutputFamily` 负责统一 text / justfloat / binary 的运行时分流。
- `JustFloatProfile` 保留固定 profile 语义，避免不同调试模式各自拼接一套 `JustFloat` 输出。
- `Binary` 输出并回同一族接口，避免后续为二进制链路做重复的模式判断。

这次 merge 的落点不是简单“多一个输出模式”，而是把输出入口收敛成统一家族。对于联调来说，这意味着新旧调试模式共享一套可观测框架，后续补字段时也更容易保持协议一致。

### 3. `mode1` 跨零修复继续保留

`mode1` 的手操 jerk / 反向跨零修复来自 `jia/codex_1/3`，并且已经保留在当前主线：

- 快速反向切换时，不再因为跨零逻辑把目标加速度错误清零。
- 反向换向仍然遵循当前主线的速度成形语义，而不是退回成“反向一来就硬切”的旧行为。
- 这部分与当前 `manual_speed_profile_mode = kSCurve` 主线默认值是配套的，不能孤立理解。

## 当前仍保持不变的主线约束

这轮 merge 之后，以下稳定约束继续成立：

- `manual_speed_profile_mode = kSCurve` 保留为当前默认速度规划模式。
- X-Park 阈值保持当前主线口径 `0.01 / 0.03`。
- 舵向 fault gate、homing gate 与 `current = 0` 保护不回退。
- 不额外引入新的调试模式号，也不再保留两套互相打架的主入口。

## 验证结果

本轮 merge baseline 已通过以下验证：

- `powershell -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\jia_docs\tests\tdd\chassis_semantics\run_test.ps1`
- `powershell -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\jia_docs\tests\tdd\chassis_semantics\run_pid_reconnect_test.ps1`
- `git diff --check`

重点验收口径如下：

- `mode30 / mode31 / mode32` 路由已按新语义分工，旧执行层直控没有串到单轮隔离链路里。
- `DebugOutput` 15 通道 `JustFloat` 契约继续可用，输出家族重构没有破坏既有观测协议。
- `mode1` 反向换向时不再出现跨零后目标加速度被清零的回归。
- fault gate / homing gate / `current = 0` 保护仍优先于调试链路。

## 与前几份 handoff 的边界

这份文档的职责是记录 merge baseline 与语义归属，不替代前面几份专题交接：

- 如果要看 fault recovery、rehome、PID 清理和保护闭环，优先看 `ai_handoff_2026-05-23_0226_rc10_steer_fault_recovery_pid_guard.md`。
- 如果要看 yaw 位置环的 VOFA `JustFloat` 15 通道观测语义，优先看 `ai_handoff_2026-05-23_2140_rc10_yaw_pid_vofa_trace.md`。
- 如果要看 S 曲线、`LockToYaw -> LockNowYaw` 锁角继承和默认调试参数收口，再补读 `ai_handoff_2026-05-24_2158_rc10_scurve_lock_yaw_context_sync.md`。

本文件只回答一个问题：当前 `jia/develop` 合并完 `jia/codex_2/3` 和 `jia/codex_1/3` 之后，哪部分语义来自哪里，以及最终主线到底以什么为准。

## 后续关注点

如果后面继续沿这条主线联调，建议优先盯住下面几类问题：

- `mode30 / mode31 / mode32` 在新增调试字段或观测量时，是否仍保持单轮隔离、full-gate、旧执行层直控三者边界清晰。
- `DebugOutputFamily` 下的 `JustFloatProfile` 是否继续维持固定通道契约，尤其是已有 15 通道 profile 不要被局部重排。
- `mode1` 跨零修复是否与后续任何速度规划调整保持一致，避免又把反向换向退回旧缺陷。
- fault gate、homing gate、`current = 0` 保护是否始终先于调试直控路径生效，防止未来改 debug mode 时把保护短路。

## 交接结论

截至 `828b8ee0`，RC10 当前底盘调试主线已经完成一次明确的语义收口：入口模式语义以 `jia/codex_2/3` 为主，输出家族与 `mode1` 跨零修复吸收自 `jia/codex_1/3`，默认值与保护行为继续沿用 `jia/develop` 已验证的稳定口径。后续工作应基于这条统一主线继续推进，而不是重新分叉出第二套调试入口。
