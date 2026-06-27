# `LockNowRotZ` 回归测试说明

这个目录用于放置四舵轮 `LockNowRotZ` 当前角锁定缺口的外部行为回归 harness。

## 目的

- 在不改仓库生产代码前，先复现“松手并过缓冲后，锁角目标回退到旧 `rot_z`”的问题
- 在生产代码修复后，用同一个状态模型验证“锁角目标保持为最近一次真实 IMU 角”

## 文件

- `lock_now_rot_z_regression.py`
  - `--mode red`：复现当前缺口，预期失败
  - `--mode green`：校验修复后行为，预期通过

## 运行方式

```powershell
python .\tests\lock_now_rot_z\lock_now_rot_z_regression.py --mode red
python .\tests\lock_now_rot_z\lock_now_rot_z_regression.py --mode green
```

## 状态模型

脚本最小化复现以下四舵轮数据流：

- `input_target_data_.rot_z`
- `target_data_.rot_z`
- `input_hwt_rot_z_`
- `omega_z`
- `lock_now_rot_z_shift_count_`
- `planned_data_.omega_z`

它不尝试模拟完整底盘，仅验证 “LockNowRotZ 在缓冲结束后到底锁的是不是最近一次真实当前角”。
