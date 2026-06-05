# swerve_port 回归测试说明

## 目标
验证本轮四舵轮迁移中的关键行为语义：
- `LockNowRotZ` 松手后锁当前角
- 回零状态机关键流转语义（模型化）
- 翻转滞回（`flip_enter/flip_exit`）决策
- 驱动抑制（DriveGate）与停车抑制（StopSteerGuard）基础行为

## 运行方式
```powershell
python D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai_2\tests\swerve_port\lock_now_homing_gate_regression.py
```

## 输出位置
- 终端打印 JSON 摘要
- 同步写入：
`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai_2\artifacts\swerve_port\lock_now_homing_gate_regression.json`

## 说明
- 该脚本为“外部行为回归 harness”，用于 TDD 的 Red/Green 证据与语义回归。
- 它不直接链接 MCU 工程目标文件，不替代最终工程编译验证。

