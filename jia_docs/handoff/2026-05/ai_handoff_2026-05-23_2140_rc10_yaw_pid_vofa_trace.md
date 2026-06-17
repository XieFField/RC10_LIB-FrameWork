# RC10 底盘 yaw PID VOFA JustFloat 调试交接

生成时间：2026-05-23 21:40（Asia/Shanghai）  
仓库路径：`C:\Users\ayou\.codex\worktrees\f316\RC10_LIB-FrameWork`

相关参考路径：
- `User/Setup/Inc/chassis.h`
- `User/Setup/Src/chassis.cpp`
- `RC10_LIB/APP/Inc/APP_debugTool.h`

## 背景

这次交接的是一条专门给 yaw 位置环调参用的 VOFA `JustFloat` 调试链路。它的目标不是替代现有底盘调试输出，而是在不改变现有控制语义的前提下，把 `rot_z_pid_` 相关的关键观测量统一输出到一组固定通道里，方便集中查看 `LockTo`、`LockNow` 和后续带平移联调时的闭环状态。

这条链路只做可观测，不做在线写参；外部控制 API 保持不变，只扩展调试输出接口和内部 trace 状态。

## 改动点

- 新增专用 VOFA `JustFloat` 模式 `kYawPidJustFloat`。
- 保持现有 `mode2 / mode3 / mode4 / mode5` 语义不变。
- 支持 `LockTo` 和 `LockNow` 两种调参观察场景，建议顺序是先 `LockTo`，再 `LockNow`。
- 新增最小配置项：
  - `yaw_pid_justfloat_period_ms`
  - `yaw_pid_justfloat_last_ms`
- 新增 trace 字段：
  - `mode_tag`
  - `target_yaw_rad`
  - `feedback_yaw_rad`
  - `error_deg`
  - `manual_omega_in_rad_s`
  - `pid_output_omega_rad_s`
  - `final_omega_cmd_rad_s`
  - `feedback_yaw_rate_rad_s`
  - `shift_remaining_ms`
  - `pid_compute_fired`
  - `steer_fault_any_active`
  - `all_homed`
  - `high_speed_suppression_active`
  - `reverse_intent_active`

## 15 通道表

这 15 个通道的顺序就是协议语义，接收端和 VOFA 端必须按同一顺序解释。若未来要改字段，建议按版本升级处理，不要局部改名或插槽。

| 通道 | 字段名 | 含义 |
|---|---|---|
| ch0 | `time_s` | 采样时间，单位 s |
| ch1 | `mode_tag` | 当前调试模式标签，用于区分 `LockTo`、`LockNow` 等状态 |
| ch2 | `target_yaw_rad` | 目标航向角，单位 rad |
| ch3 | `feedback_yaw_rad` | 实际反馈航向角，单位 rad |
| ch4 | `error_deg` | 航向误差，单位 deg |
| ch5 | `manual_omega_in_rad_s` | 手动输入的角速度分量，单位 rad/s |
| ch6 | `pid_output_omega_rad_s` | PID 计算输出的角速度分量，单位 rad/s |
| ch7 | `final_omega_cmd_rad_s` | 最终下发给底盘的角速度命令，单位 rad/s |
| ch8 | `feedback_yaw_rate_rad_s` | 航向角速度反馈，单位 rad/s |
| ch9 | `shift_remaining_ms` | 平移/切换阶段剩余时间，单位 ms |
| ch10 | `pid_compute_fired` | PID 本轮是否触发计算，通常用 0/1 表示 |
| ch11 | `steer_fault_any_active` | 转向故障总标志，1 表示本次曲线作废 |
| ch12 | `all_homed` | 是否已全部回零/归位，通常用 0/1 表示 |
| ch13 | `high_speed_suppression_active` | 高速抑制是否激活，通常用 0/1 表示 |
| ch14 | `reverse_intent_active` | 是否存在反向意图，通常用 0/1 表示 |

说明：
- `steer_fault_any_active` 必须保留在可见通道里，便于一眼判断这组曲线还能不能用于调参。
- 这轮只做观测，不做在线写参；通道内容是给 VOFA 看板和人工分析用的。
- 如果后续需要更多诊断信息，优先考虑扩展 trace 内部状态，而不是破坏这 15 个位置的固定顺序。

## 调试流程

建议按 0 到 4 五个阶段推进，尽量不要跳步。

### 0. 上电前置检查

- 确认 `kYawPidJustFloat` 已被正确接入调试输出路径。
- 确认 `yaw_pid_justfloat_period_ms` 和 `yaw_pid_justfloat_last_ms` 的默认值合理。
- 确认转向故障、归位状态和高低速抑制状态都能正常写入 trace。

### 1. `LockTo` 基线调参

- 先只看目标朝向到反馈朝向的闭环形状。
- 确认 `pid_output_omega_rad_s` 与 `final_omega_cmd_rad_s` 的关系符合预期。
- 先把角度环的基础响应调顺，不要急着叠加平移工况。

### 2. `LockNow` 交接调参

- 验证松手保持、目标冻结和退场缓冲是否符合预期。
- 观察 `shift_remaining_ms`、`pid_compute_fired` 以及 `mode_tag` 的切换是否一致。
- 确认交接阶段没有引入明显的角速度突变。

### 3. 带平移联调

- 在有 `vx / vy` 叠加的条件下复核 yaw 闭环。
- 重点看 `manual_omega_in_rad_s` 和 `final_omega_cmd_rad_s` 的耦合效果。
- 检查高速度抑制是否会改变调参判断。

### 4. 边界验证

- 验证故障触发时曲线是否正确失效。
- 验证 `all_homed`、`reverse_intent_active`、`high_speed_suppression_active` 的状态是否能稳定反映现场条件。
- 验证长时间运行下通道顺序和采样节奏没有漂移。

## 测试点

### 静态/宿主测试

- 检查 VOFA `JustFloat` 打包顺序是否严格等于 15 通道定义。
- 检查新增 trace 字段是否都能被写入且不会破坏旧模式。
- 检查 `kYawPidJustFloat` 是否只影响调试输出，不影响现有 `mode2 / mode3 / mode4 / mode5` 语义。
- 检查 `LockTo`、`LockNow` 的调试观测量是否能在宿主环境下正确构造。

### 实车验证

- 验证 `LockTo` 下 yaw 收敛是否稳定，且曲线可读。
- 验证 `LockNow` 松手保持阶段是否能正确冻结目标并平稳退场。
- 验证带平移联调时 yaw 与平移叠加后是否仍可接受。
- 验证触发 `steer_fault_any_active=1` 时，VOFA 曲线能否被正确判定为作废。

## 后续建议

- 如果后续要继续扩展，建议先冻结通道版本号，再改字段含义。
- 若要增加更多调试信息，优先扩展内部 trace 状态，避免破坏固定 15 通道协议。
- 后续实现时，优先检查：
  - `User/Setup/Src/chassis.cpp` 中的 VOFA 输出路径
  - `User/Setup/Inc/chassis.h` 中的调试模式枚举和相关成员
  - `RC10_LIB/APP/Inc/APP_debugTool.h` 中的 `printf_DMA_JustFloat(...)` 接口
- 这条链路仍应保持只读调试属性，不建议把在线写参混进来。

## 交接结论

本次新增的是一条专用、只读、顺序固定的 yaw 位置环 VOFA 调试链路。它把 `LockTo / LockNow / 带平移联调` 的观测量统一到同一组 `JustFloat` 通道中，同时保持现有 `mode2 / mode3 / mode4 / mode5` 语义和外部控制接口不变。
