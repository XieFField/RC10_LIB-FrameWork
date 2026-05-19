# `updateHomingState()` 回归测试说明

这个目录用于放置四舵轮 `updateHomingState(WheelConfig &wheel)` 的外部状态机回归测试。

## 目的

- 固化回零状态机的关键状态迁移
- 防止后续在注释增强、回零策略调整或字段语义修改时把状态机悄悄改坏

## 文件

- `update_homing_state_regression.py`
  - 当前先提供 `green` 回归，覆盖高价值状态机分支
  - 重点验证：
    - 禁用回零直接 `Ready`
    - `Idle -> Search`
    - `Search` 边沿检测
    - `Search` 上电即高电平
    - `Search` 超时进入 `Fault`
    - `EdgeDetected -> OffsetApply -> ContinuousAngleReady -> Ready`
    - `Ready/Fault` 返回值语义

## 运行方式

```powershell
python .\tests\homing_state\update_homing_state_regression.py --mode green
```
