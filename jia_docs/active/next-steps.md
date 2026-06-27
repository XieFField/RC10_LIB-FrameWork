# 当前待办与联调关注点

本页用于接替旧 `plan.txt` 的待办入口，保留仍然有效的事项，并对齐 2026-06-24 的当前 handoff 状态。

## 当前仍然有效的联调关注点

围绕 [ai_handoff_2026-06-24_rc10_homing_auto_retry_xpark_priority_brake_debug_sync.md](../handoff/2026-06/ai_handoff_2026-06-24_rc10_homing_auto_retry_xpark_priority_brake_debug_sync.md)，继续联调时建议优先关注：

1. 首次上电整车 homing 延时默认值 `500U` 是否真的合适，重点结合 `first_boot_homing_delay_.pending / active / elapsed_ms` 和四轮 `homing_state` 板上观察。
2. `JIA_CHASSIS_HOMING_SEARCH_RPM = 100.0f` 的当前默认基线是否与现有舵轮校准手感匹配，必要时和首次上电延时一起联调。
3. AirJoy/Lora 输入链切换后，底盘线程里的真实摇杆数据是否稳定，零位漂移、映射符号和更新时间机是否符合预期。
4. debug 摇杆 deadzone 重映射在两条链上的实际手感：
   - 整车 debug 控管链：平移组 / 旋转组 deadzone
   - mode30 单舵轮 debug：`single_wheel.input_deadzone`
5. 继续保持 `RUNTIME_MIN` 默认固件档与 `FULL_DEBUG` host 语义套件回归分层清晰，不要把 debug 语义回流到 slim smoke。
6. 继续保持 MDK 编译入口检查，避免路径规划、机械臂和底盘头文件合并残留再次破坏工程编译。
7. 继续跟踪 `X-Park priority brake` 的板上刹停手感、姿态稳定性和 residual 门限行为。
8. 继续跟踪 `homing auto-retry` 在板上的等待间隔、最大次数停 `Fault`、以及 recovery re-home 失败链。

## 不再重复描述的旧状态

- 不再把当前主线入口停留在 2026-06-09；默认交接入口已切到 2026-06-24 新总览。
- 不再把 `jia_docs/tests/tdd/chassis_semantics/run_test.ps1` 描述为当前主入口；它仍是兼容包装入口，不是首选入口。
- 不再把 `RUNTIME_MIN` slim smoke 当作 debug 语义回归入口；它只负责瘦身档可编译、可实例化和尺寸基线。
- 不再继续按旧 CRSF 直取口径理解底盘线程输入链；当前默认应以 AirJoy/Lora 主动拉取理解。
