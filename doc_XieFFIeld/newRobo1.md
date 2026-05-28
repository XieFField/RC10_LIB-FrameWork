# newRobo1 项目上下文摘要

## 项目概况

STM32H7 平台，四舵轮底盘 + 串联刚体机械臂。FreeRTOS 多任务架构。编码 GB2312 → 已转为 UTF-8。

---

## 1. 机械臂云台旋转系统

### 核心文件
- `User/Control/Inc/Robot_Arm.h` — `Arm_InitData_S`、`calc_legal_rotate_target`、`Rotate_Strategy_E`
- `User/Control/Src/Robot_Arm.cpp` — 旋转策略 + 电机角度换算
- `User/Setup/Inc/Arm_Setup.h` — `sanitizeRotateAngle`、`isRotateAllowed`
- `User/Setup/Src/Arm_Setup.cpp` — `manualControl`、`manual_store`、`manual_takeout`、`calibrateMotor`

### 旋转合法区
- **禁止区**: `(rotate_start, rotate_end)` = `(135°, 265°)`（默认）
- **合法区**: `[rotate_end, 360°] ∪ [0°, rotate_start]`
- `Arm_InitData_S` 新增字段: `rotate_end = 265.0f` / `rotate_start = 135.0f`
- 校准位置: `0.001°`（结构上 0° 上电，`rotateAngle_to_MotorTotalAngle(0.001f)` 重定位）

### 高度三档旋转限制 (`sanitizeRotateAngle`)
| 高度 | 行为 |
|------|------|
| `< lock_h` (0.04m) | 强制 0.0° |
| `lock_h ~ safe_h` (0.04~0.08m) | 钳制 [0°, 135°]；若在存储区且 h>lock_h+0.005 → 自由 |
| `≥ safe_h` (0.08m) | 自由 |

### 下降锁定 (`manualControl`)
- h < `lock_h + 0.01m` 时，`|angle - 0.0| > 0.8°` → 禁止抬升

### `calc_legal_rotate_target` 算法
1. 入参 fmodf 归一化 0~360
2. 目标在禁止区 → 钳制到最近合法边界
3. 最短路径穿越禁止区判断（弧线交集，用 `<` `>` 严格不等式）
4. 穿越 → 取相反策略方向 (NEGATIVE/POSITIVE)；不穿越 → SHORTEST
5. `|shortest| < 10°` → 强制 SHORTEST 收敛
6. 策略应用 → 返回 `fmodf` 归一化 0~360
7. 内部 `rotate_strategy_` (private) 供 `update()` 电机段使用

### `update()` 电机段策略感知回绕
```cpp
float k;
if (rotate_strategy_ == ROTATE_PATH_POSITIVE)      k = ceilf(diff/360);
else if (rotate_strategy_ == ROTATE_PATH_NEGATIVE) k = floorf(diff/360);
else                                               k = roundf(diff/360);
float target_arm_total = target_arm + k * 360.0f;
```
`ceilf` 保证 POSITIVE 时目标在电流之上；`floorf` 保证 NEGATIVE 时在电流之下。

### 关键私有成员
- `Rotate_Strategy_E rotate_strategy_` (private)
- `float prev_rotate_target_` (连续值，供连续性修正)
- `float prev_norm_target_` (检测目标是否变化)

### 上电防转竞态
- `calibration_seen_` 标志：校准完成后首帧强制 `ARM_CALIBRATE`，给 FSM 一帧缓冲避免误入手操
- `setArmStatus()` 在 `!isArmcalibrated()` 时只允许 `ARM_CALIBRATE`

---

## 2. 四舵轮底盘 `Chassis_Swerve`（Demo 模块）

### 核心文件
- `User/debug/Inc/chassis_swerve_demo.h`
- `User/debug/Src/chassis_swerve_demo.cpp`

### 轮组几何（取 chassis.cpp）
| 轮 | x (m) | y (m) | L (m) | θ (rad) | θ_oa_to_owi | drive_sign |
|----|-------|-------|-------|---------|-------------|------------|
| 0  | -0.39 |  0.40 | 0.559 | 2.338 | -90° |  -1 |
| 1  | -0.39 | -0.40 | 0.559 | 3.934 |   0° |  -1 |
| 2  |  0.39 | -0.40 | 0.559 | 5.488 |  90° |   1 |
| 3  |  0.39 |  0.40 | 0.559 | 0.798 | 180° |   1 |

切向量 = θ + PI/2。轮半径 0.052m。

### 坐标系
- 世界→车体: `bx = vx*c + vy*s`, `by = -vx*s + vy*c` (标准旋转)
- 车体→世界: `wx = vx*c - vy*s`, `wy = vx*s + wy*c` (逆旋转)
- 车体内: bx=前进, by=左移
- `kinematic_calc` 解算时: `v_body = (body_speed.x, -body_speed.y)` (y 取反适配)

### `set_world_speed` / `set_body_speed`
- 互相写入 world_speed 和 body_speed，保证零速检测一致

### 回零状态机 (对齐 chassis.cpp)
```
kIdle → kSearch → kEdgeDetected → kOffsetApply → kAlignToZero → kReady
```
- offset 计算（对齐 chassis）: `edge_local = edge_mech_oa - theta_oa_to_owi; offset = edge_local + zero_offset - raw_total`
- 角度转换链: `rawTotalToCorrectedLocalRad` (raw→local) / `correctedLocalToRawTotalDeg` (local→raw) / `localToOARad` / `oaToLocalRad`
- `setTargetAngle(0~360°)`，M3508 内建 circular PID

### 翻转逻辑
- 简化: `|delta| > PI/2` → 翻转 180° + 驱动反向
- 无 RC 奇偶判断

### 驱动门控
- `Fliter_Ramp_S` 新增 `max_decel_`（刹车/启动分离）
- `set_max_linear_vel(v)` / `set_max_angular_vel(w)`
- 零速锁轮: `zero_lock_delay_s_` 时间门（0=跳过），锁到 X 姿态

### 任务 `Swerve_Task_Demo`
- SWB=0x00: STOP; SWB=0x01: 世界系; SWB=0x02: 车体系
- `update_yaw()` 从 Locate_Setup 取实时 yaw

---

## 3. FSM 控制器

### 关键修复
- `manual_store()` `rotate_state`: 加了旋转在位判定后才跳 `lower_state`
- `STATE_BACK` 中 `manual_store()` 调用: 返回 true 后不再重入，用 `isbackdone` 守卫
- 上电校准期间 `arm_status_` 竞态: `calibration_seen_` 缓冲 + `setArmStatus` 守卫

---

## 4. 编码

项目原为 GB2312，已通过 PowerShell + `[Text.Encoding]::GetEncoding(20936)` 转为 UTF-8。
未来所有编辑注意使用 UTF-8 编码。

---

## 5. 调试端口

- UART7: CRSF 遥控 (CrsfReceiver)
- UART8: 调试打印 (Debug_Printf)
- FDCAN1/2: DJI 电机 + VESC
- FDCAN3: 备用（硬件收发器芯片待确认）

---

## 6. 编译环境

- Keil MDK-ARM (STM32H7)
- FreeRTOS
- ARM DSP 库
- CRLF 行尾
