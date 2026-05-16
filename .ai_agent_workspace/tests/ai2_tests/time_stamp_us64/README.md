# `BSP_RtosTimeStampUs64` 回归测试说明

## 目标
- 验证内部换算语义（零除保护、整除行为、子 tick 截断）保持稳定。
- 验证对外接口已收口为零参数时间戳接口：`getTimeUs()`。
- 验证命名空间已统一为 `namespace jia`。

## 运行方式
```powershell
python .ai_agent_workspace/tests/ai2_tests/time_stamp_us64/time_stamp_us64_regression.py
```

## 覆盖说明
- `tick_rate_hz == 0` 返回 `0`。
- 常规 `ticks -> us` 换算结果正确。
- `sub_tick_us > tick_period_us` 时执行截断。
- 静态约束：
  - BSP 头/源存在 `class RtosTimeStampUs64`
  - 头文件包含 `getTimeUs`，不再暴露 `TicksToUs64/ComposeTimeUs64`
  - 源码命名空间为 `namespace jia`
