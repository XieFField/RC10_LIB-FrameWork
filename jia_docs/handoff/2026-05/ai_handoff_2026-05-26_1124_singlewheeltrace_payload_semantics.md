# SingleWheelTrace payload 交接说明

生成时间：2026-05-26 11:24（Asia/Shanghai）

## 背景

`SingleWheelTrace` 现在已经不是“只看 payload 长度就能判断”的协议了。上位机如果只按通道数分流，很容易把不同语义的 trace 误当成同一类数据。

## 现有 payload

当前实现中，`JustFloatProfile::kSingleWheelTrace` 下的 payload 分为三类：

1. `kSteerOnly`：9 通道，只输出单轮 steer 观测值。
2. `kDriveOnly`：9 通道，只输出单轮 drive 观测值。
3. `kSteerAndDrive`：17 通道，输出 steer + drive 两套观测值。

另外，`kYawPid` 是另一条独立的 `JustFloatProfile`，固定 15 通道，不要混入 `SingleWheelTrace` 的语义里理解。

## 9 通道注意点

如果上位机侧已经把“9 通道”当成默认单轮 trace 入口，需要额外确认 `profile_raw` 和 `single_wheel_payload_raw`，不能只用长度判断。现在 `kSteerOnly` 和 `kDriveOnly` 都是 9 通道，但字段含义不同。

## 接收端注意事项

- 先看 `output_family_raw`，确认是不是 `JustFloat`。
- 再看 `profile_raw`，确认是 `kSingleWheelTrace` 还是 `kYawPid`。
- 在 `kSingleWheelTrace` 下，再根据 `single_wheel_payload_raw` 判定是 `kSteerOnly`、`kDriveOnly` 还是 `kSteerAndDrive`。
- 不要把“通道数相同”直接等同于“语义相同”。

## 相关文件

- `User/Setup/Inc/chassis.h`
- `User/Setup/Src/chassis.cpp`
- `jia_docs/tests/tdd/chassis_semantics/test_chassis_semantics.cpp`
- `jia_docs/README.md`
