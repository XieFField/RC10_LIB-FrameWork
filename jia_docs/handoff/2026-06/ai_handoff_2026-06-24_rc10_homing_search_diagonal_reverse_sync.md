# AI handoff 2026-06-24 RC10 homing search diagonal reverse sync

本文档用于记录本轮光电门回零搜索方向优化的交接状态。它只收口一件事：把舵轮 homing 搜索方向改成编译期固定的对角轮反向表，以减小 `HomingState::kSearch` 阶段的底盘净旋转。

## 本轮变更

- 在 `User/Setup/Inc/chassis.h` 增加了编译期搜索方向表 `JIA_CHASSIS_HOMING_SEARCH_RPM_SIGN[4]`。
- 在 `User/Setup/Src/chassis.cpp` 的回零搜索下发路径里，`setSteerMotorTargetRPM()` 改为按轮位乘方向符号。
- 默认采用 `0 + 2` 反转，`1 + 3` 保持同向。
- 只改光电门回零搜索，不改正常运动、recovery re-home、zero-stop、X-Park 或其他模式。

## 轮位约定

当前轮位顺序保持和 `chassis.cpp` 初始化一致：

- `0` = 左前
- `1` = 左后
- `2` = 右后
- `3` = 右前

因此默认方向表是：

- `0` 反转
- `1` 同向
- `2` 反转
- `3` 同向

## 已验证结果

当前 host 语义验证结果已更新为：

- `jia_docs/tests/host_cpp/chassis_semantics/run_main.ps1`
  - `211/211 passed`

这次验证同时确认：

- 回零状态机仍可进入 `Search -> EdgeDetected -> Ready`
- `homing_auto_retry` 语义未被破坏
- 回零搜索下发的 RPM 方向已按轮位分裂，而不是四轮同号

## 代码边界

- 方向表是编译期常量，不新增运行时开关。
- 如果后续板上实测需要换成另一组对角轮，只改这一张表即可。
- 本轮不改 `homing_search_rpm` 默认值，不改回零边沿逻辑，不改 recovery 链。

## 后续接手建议

1. 优先读本文件。
2. 再看 `jia_docs/active/overview.md`。
3. 如需追前一轮主线背景，再看 `2026-06-24 rc10_homing_auto_retry_xpark_priority_brake_debug_sync`。

## 提交锚点

本轮交接锚点：`211/211 passed`
