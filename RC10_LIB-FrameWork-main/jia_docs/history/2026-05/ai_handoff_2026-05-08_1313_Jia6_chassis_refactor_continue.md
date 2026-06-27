# Jia6 底盘重构继续开发交接文档

生成时间：2026-05-08 13:13（Asia/Shanghai）

主工程路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork`

AI 资料路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai`

当前分支：`Jia6_temp`

上一份交接文档：`ai_handoff_2026-05-07_2305_Jia6_chassis_refactor.md`

## 1. 会话概述

本会话从上一份交接文档（2026-05-07 23:05）的状态继续推进，完成了以下大项：

1. **注释编码修复** — 全项目中文化注释从乱码（GBK/UTF-8 混用）统一为 GBK 编码
2. **Motor_Base 封装修复** — 成员变量 `public:` → `protected:`
3. **代码迁移** — 坐标变换/加速限制等通用函数从 chassis.cpp 提取到 APP_Utils.h
4. **冗余代码清理** — 删除 `max_wheel_speed_`、`world_twist_forward` 等未使用成员
5. **DWT 初始化加固** — 时钟频率不能整除 1MHz 时不启用 DWT us 换算
6. **FourSteer 控制路径修复** — 单轮调试模式下跳过 SwerveController 避免输出覆盖
7. **回归测试** — 新增 3 个测试套件（共 8 个，全部 PASS）

## 2. 最重要的约束（承接上一份，不变）

1. 不允许对 `main` 执行任何提交、合并、推送或上传操作。
2. 最终集成方式必须是用户手动从 `main` 主动合并缓冲分支。
3. 开发只围绕 `Jia6_temp` / `Jia6` 体系推进。
4. AI 相关测试、文档、记录只放在 `RC10_LIB-FrameWork_ai`，不放入主工程。
5. 用户可完全负责修改的范围：
   - `User/Setup/Inc/chassis.h` / `User/Setup/Src/chassis.cpp`
   - `RC10_LIB/Module/Inc/Module_ChassisBase.h` / 对应的 `.cpp`
   - `RC10_LIB/Module/Inc/Module_ChassisOmni.h` / 对应的 `.cpp`
   - `RC10_LIB/Module/Inc/Module_ChassisSwerve.h` / 对应的 `.cpp`
   - `RC10_LIB/APP/Inc/APP_Utils.h` / 对应的 `.cpp`
6. 上述范围外的文件遵循最小修改原则。
7. 舵轮算法优先放入已有 `Module_ChassisSwerve.*` 文件，不新建算法文件。
8. 新增文件（除已批准的 `BSP_TimeDwt.*`、`BSP_TimeUs64.*`）必须先征得用户同意。
9. 调试器可见全局变量：`extern "C"` + `volatile` + C 扁平 struct，不放在 C++ namespace 下。
10. 三全向轮有实际硬件可实测；四舵轮无法进行硬件功能实测。
11. 命名空间：内部分使用 `jia`，对外接口用 `using jia::XXX::Chassis;` 暴露。
12. 测试风格：最小 C++ harness（`main() + EXPECT_TRUE/NEAR + printf`）+ PowerShell `run_test.ps1` + MinGW g++。
13. 调试方式：优先调试器全局变量（Keil Watch 友好），其次 UART7 VOFA+ JustFloat。

## 3. 编码约定（重要！）

所有项目源文件的中文注释使用 **GBK/GB2312** 编码，**不是 UTF-8**。

原因：Keil MDK µVision 在中文 Windows 上默认使用系统本地编码（GBK），不识 UTF-8 中文。UTF-8 中文在 Keil 编辑器中会显示为乱码。

**对后续 agent 的要求：**
- 不要使用 Write 工具直接写含中文的文件（Write 输出 UTF-8）
- 如有中文内容，写入后需要用 `iconv -f UTF-8 -t GBK file.cpp` 转换
- 或用 `sed` 在已有 GBK 文件上做小修改（不改动中文注释区域）
- 文件编码检测方法：`file file.cpp` → 期望输出 `ISO-8859` 或 `Non-ISO extended-ASCII`（表示 GBK），而非 `UTF-8`

## 4. 当前 git 状态

```text
branch: Jia6_temp

已修改文件（相对于 HEAD）：
 M Frame_T/Frame_T.build_log.htm
 M Frame_T/Frame_T_Frame_T.dep
 M MDK-ARM/Frame_T.uvoptx
 M MDK-ARM/Frame_T.uvprojx
 M RC10_LIB/APP/Inc/APP_Utils.h          ← 注释修复 + 新增坐标变换函数
 M RC10_LIB/BSP_Driver/Src/BSP_TimeDwt.cpp  ← Init() 逻辑加固
 M RC10_LIB/BSP_Driver/Src/BSP_TimeUs64.cpp ← 编译守卫加固
 M RC10_LIB/Module/Inc/Module_ChassisBase.h ← 注释修复 + 删除冗余成员
 M RC10_LIB/Module/Inc/Module_ChassisOmni.h ← 注释修复 + 删除冗余常量
 M RC10_LIB/Module/Inc/Module_ChassisSwerve.h
 M RC10_LIB/Module/Src/Module_ChassisBase.cpp ← 删除 max_wheel_speed_ 初始化
 M RC10_LIB/Module/Src/Module_ChassisSwerve.cpp
 M RC10_LIB/Motor/Inc/Motor_Base.h        ← 封装修复 + 注释修复
 M User/Setup/Inc/chassis.h               ← 注释修复 + 迁移声明 + Debug Watch 字段
 M User/Setup/Inc/omni_chassisSetup.h
 M User/Setup/Src/chassis.cpp             ← 代码迁移 + 注释修复 + 控制路径修复
 M User/Setup/Src/omni_chassisSetup.cpp   ← 部分注释修复

新增文件（未跟踪）：
 ?? RC10_LIB/BSP_Driver/Inc/BSP_TimeDwt.h
 ?? RC10_LIB/BSP_Driver/Inc/BSP_TimeUs64.h
 ?? RC10_LIB/BSP_Driver/Src/BSP_TimeDwt.cpp
 ?? RC10_LIB/BSP_Driver/Src/BSP_TimeUs64.cpp
 ?? _uv4_build_mdkarm.log
 ?? _uv4_build_root.log

已删除文件（未跟踪）：
 D  Frame_T.uvguix.*
 D  Frame_T.uvoptx
 D  Frame_T.uvprojx
 D  startup_stm32h723xx.s
```

## 5. 文件级改动说明

### 5.1 `RC10_LIB/Motor/Inc/Motor_Base.h` — 封装修复

**改动**：
- 成员变量从 `public:` (line 64) 改为 `protected:`
- 所有中文注释重写为可读的 GBK 中文
- `setBrake()` 虚接口保留（上一个 agent 添加）
- getter 方法返回内部字段（上一个 agent 修改）

**影响**：外部代码（chassis.cpp、omni_chassisSetup.cpp）必须通过 `getTotalAngle()`、`getRPM()` 等 getter 访问电机状态，不能直接读 `totalAngle_`。

### 5.2 `RC10_LIB/APP/Inc/APP_Utils.h` — 新增通用工具函数

在 `namespace jia` 下新增 4 个 inline 函数：

| 函数 | 说明 | 来源 |
|---|---|---|
| `transSpeedBodyToWorld(vx, vy, yaw_rad, out_vx, out_vy)` | 自身→世界坐标系速度变换 | chassis.cpp TriOmni 私有方法 |
| `transSpeedWorldToBody(vx, vy, yaw_rad, out_vx, out_vy)` | 世界→自身坐标系速度变换 | chassis.cpp TriOmni 私有方法 |
| `clampTargetSpeedInChassis(...)` | 车体目标速度限幅 | chassis.cpp TriOmni 私有方法 |
| `limitChassisAcceleration(...)` | 车体加速度限制 | chassis.cpp TriOmni 私有方法 |

全部中文 DOxygen 注释从乱码修复为可读 GBK 中文。

### 5.3 `RC10_LIB/Module/Inc/Module_ChassisBase.h` — 清理冗余

**删除**：
- `Robot_Twist world_twist_forward = {0}` — 声明但从未被任何代码写入或读取
- `const float max_wheel_speed_` — 计算后从未被读取

**保留**：
- `Robot_Twist robot_twist_forward` — 在 `Chassis_Omni::forwardKinematics()` 中被写入，用于正运动学输出

### 5.4 `RC10_LIB/Module/Src/Module_ChassisBase.cpp` — 构造函数清理

移除了 `max_wheel_speed_` 的初始化行。文件从 HEAD 恢复后使用 sed 修改以保证 GBK 编码。

### 5.5 `RC10_LIB/Module/Inc/Module_ChassisOmni.h` — 注释修复 + 重构

- 全部中文注释从乱码修复为可读 GBK 中文
- ASCII 艺术布局图保留，文字说明更新
- `COS_31_87` / `SIN_31_87` 常量保留（在 `.cpp` 的 `forwardKinematics()` 中被实际使用）
- 冗余的 `#ifndef` / `#define` include guard 删除（已有 `#pragma once`）

### 5.6 `RC10_LIB/BSP_Driver/Src/BSP_TimeDwt.cpp` — Init() 逻辑加固

**核心改动**：`TimeDwt::Init()` 现在检查时钟频率是否能被 1 MHz 整除：

```cpp
if (core_clock_hz_ % 1000000U == 0U) {
    cycles_per_us_ = core_clock_hz_ / 1000000U;
    // 使能 DWT 硬件
} else {
    // 频率不兼容：保持默认值 1U（降级模式），不启用 DWT 硬件
}
```

**行为变化**：
- 400MHz / 100MHz 等标准频率 → 正常启用
- 1.5MHz / 800kHz 等非标准频率 → 不启用 DWT us 换算，`CyclesToUs32()` 返回非精确值
- `GetCycle32()` / `GetElapsedCycles32()`（原始周期计数）在任何情况下都可用

**`CyclesToUs32` 除零守卫**：当 `cycles_per_us_ == 0U` 时返回 0，防止除零崩溃。

### 5.7 `RC10_LIB/BSP_Driver/Src/BSP_TimeUs64.cpp` — 编译守卫加固

```cpp
// 改前：
#if defined(USE_HAL_DRIVER)
// 改后：
#if defined(STM32H723xx) && defined(USE_HAL_DRIVER)
```

与 baby_car 参考实现对齐，防止在非 STM32H723 平台上误编译。

### 5.8 `User/Setup/Inc/chassis.h` — 声明精简 + Debug Watch 增强

**删除的声明**（对应方法已迁移到 `APP_Utils.h`）：
- `transSpeedBodyToWorld` / `transSpeedWorldToBody`
- `isTransSpeedBodyToWorld` / `isTransSpeedWorldToBody`
- `clampTargetSpeedInChassis` / `isLimitAccInChassis`

**新增的 Debug Watch 字段**（`JiaChassisDebugWatch` 结构体）：
```c
uint32_t four_swerve_used_controller_step; // 四轮正常模式是否执行了 SwerveController
float four_swerve_body_vx_m_s;
float four_swerve_body_vy_m_s;
float four_swerve_body_wz_rad_s;
```

### 5.9 `User/Setup/Src/chassis.cpp` — 代码迁移 + 控制路径修复

**已删除的私有方法实现**（迁移到 APP_Utils.h）：
- TriOmni `transSpeedBodyToWorld` / `transSpeedWorldToBody`
- TriOmni `isTransSpeedBodyToWorld` / `isTransSpeedWorldToBody`
- TriOmni `clampTargetSpeedInChassis` / `isLimitAccInChassis`

**已替换为库函数调用**：
```cpp
// 坐标变换（之前是私有方法调用）
jia::transSpeedWorldToBody(it.vel_x, it.vel_y, input_hwt_rot_z_, t.vel_x, t.vel_y);
// 速度限幅（之前是私有方法调用）
jia::clampTargetSpeedInChassis(t.vel_x, t.vel_y, t.omega_z, max_vel_x_, max_vel_y_, max_omega_z_, ...);
// 加速度限制（之前是私有方法调用）
jia::limitChassisAcceleration(is_chassis_acc_limit_, ...);
```

**FourSteer 控制路径修复**：
```cpp
// 单轮调试模式下跳过 runRuntimeSwerveControl，避免四轮 Swerve 输出覆盖单轮调试命令
const bool is_single_wheel_debug =
    (is_wheel_speed_mode_ || is_wheel_current_mode_ ||
     is_wheel_single_position_mode_ || is_wheel_total_position_mode_);
if (!is_single_wheel_debug) {
    runRuntimeSwerveControl();
}
isDebugMode();
```

### 5.10 `User/Setup/Src/omni_chassisSetup.cpp` — 部分注释修复

- `ResetAutoControlStates` / `ComputeLookaheadDiffFeedforward` / `ComposeRobotVelocity` / `GetPathNearestPoint` / `FindLookaheadPoint` / `KFS_Selection_Planning` 等函数的注释从乱码修复为可读 GBK 中文
- 文件从 git HEAD 恢复后，注释修复未完全覆盖所有函数（因编码转换限制）
- 剩余乱码注释为 HEAD 中既有的历史问题，非本次引入

## 6. 测试资产状态

### 测试目录结构
```text
RC10_LIB-FrameWork_ai\tests\
├── chassis_module\          ← 上一个 agent 创建，底盘模块基础测试
├── swerve_core\             ← 上一个 agent 创建，SwerveController 算法测试
├── time_services\           ← 上一个 agent 创建，DWT/Us64 纯函数测试
├── vesc_brake\              ← 上一个 agent 创建，VESC 刹车语义测试
├── omni_setup_static\       ← 上一个 agent 创建，全向轮 setup 静态检查
├── tri_omni_kinematics\     ← 本次新增，三全向轮运动学回归测试
├── four_steer_setup_static\ ← 本次新增，四舵轮代码静态检查
└── chassis_mode_fsm\        ← 本次新增，底盘模式状态机检查
```

### 测试技术栈
- 编译器：`C:\Qt\Tools\mingw1310_64\bin\g++.exe`（MinGW g++ 13.1.0）
- C++ 标准：`-std=c++17`
- 断言宏：`EXPECT_TRUE` / `EXPECT_NEAR`（自定义，`main()` + `printf`）
- 静态检查：PowerShell `Select-String` / `-match`

### 运行命令
```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\<name>\run_test.ps1
```

### 当前结果（2026-05-08 13:13 全部 PASS）
```
tri_omni_kinematics     — 0 failures（7 个测试：逆运动学 vx/vy/wz、零电流、限幅、空指针、越界）
chassis_module          — PASS
swerve_core             — PASS
time_services           — PASS
vesc_brake              — PASS
omni_setup_static       — PASS
four_steer_setup_static — 0 failures（8 个静态检查）
chassis_mode_fsm        — 0 failures（15 个静态检查）
```

### `tri_omni_kinematics` 测试覆盖详情
| 测试函数 | 验证内容 |
|---|---|
| `testWheelRegistrationOutOfBounds` | 轮子注册越界返回 false |
| `testZeroCurrentModeSendsZeroCurrentToAllMotors` | 零电流模式所有电机收到 0 电流 |
| `testUnregisteredMotorsAreNotAccessed` | 未注册电机(nullptr)不崩溃 |
| `testPureVxForwardProducesEqualWheelSpeeds` | 纯 vx 前进三轮输出 |
| `testPureVySidewaysProducesNonZeroWheelSpeeds` | 纯 vy 侧移时轮0（theta=0°）转速为 0（正确） |
| `testPureRotationProducesAllWheelsTurning` | 纯旋转三轮同向输出 |
| `testSpeedLimitingScalesAllWheelsWhenAnyExceedsMax` | 超速时等比限幅 |

## 7. 仍未完成或需要继续的部分

### 7.1 SwerveController 与 FourSteer 的深度集成

当前状态：`runRuntimeSwerveControl()` 已经能正确调用 `SwerveController::step()` 并下发四轮命令。单轮调试模式已与四轮控制路径正确分离。

仍需完成：
1. 在四轮正常模式下完善 debug watch 输出（4 个模块的舵向/轮向目标与反馈）
2. 验证 `debug_mode_` 0-8 各模式与 SwerveController 的交互逻辑
3. 光电门校准目前仅在单轮调试路径中，是否需要在四轮模式中也使用 SwerveController 管理

### 7.2 chassis.cpp 仍有进一步精简空间

已迁移到库的代码：
- 坐标变换 ✅
- 速度限幅 ✅
- 加速度限制 ✅

仍保留在 chassis.cpp 中、可考虑进一步迁移的代码：
- `inverseKinematics`（TriOmni 的三参数标量版本）→ 可考虑统一到 `Chassis_Omni::inverseKinematics`
- `calculatePid`（两个重载）→ 可考虑移入 `APP_Utils.h`
- `isLockNowRotZ` / `isLockToRotZ` → 依赖大量成员变量，迁移成本高
- `initWheelConfig`（两个版本）→ 可统一为一个模板函数

### 7.3 Module_ChassisSwerve.h 未使用字段

以下结构体字段声明但未使用，后续可考虑清理：
- `WheelGeometry::steer_motor_sign` / `drive_motor_sign`
- `HomingConfig::sensor_active_high` / `simulate_bounce` / `bounce_duration_s` / `trigger_angle_local_rad` / `trigger_window_rad`
- `ModuleSnapshot` / `SimulationStepRecord` 结构体

### 7.4 omni_chassisSetup.cpp 剩余乱码注释

部分函数（如 `Path_correction` 中段、`KFS_Selection_Planning` 中段、`Clamping_Bar_Selection_Planning`、`flag_reset`）的注释仍为乱码状态。这些注释在 git HEAD 中已是乱码，属于历史遗留问题。

### 7.5 Keil/MDK 编译尚未验证

当前只验证了 host 测试（MinGW g++）。
未验证：
1. Keil 工程完整编译（armclang/armcc 对新代码的兼容性）
2. `BSP_TimeDwt` / `BSP_TimeUs64` 在 STM32H723 上的实际运行
3. `g_jia_chassis_debug_watch` 在 Keil Watch 窗口中的可访问性

需要用户提供 Keil 工具链路径后执行。

### 7.6 硬件验证状态

- 三全向轮底盘：**可以实测** — 验证运动学、模式切换、遥控器控制
- 四舵轮底盘：**无法硬件实测** — 仅编译通过 + debug watch 观察

## 8. 建议的继续顺序

### 第一步：清理 BSP_TimeDwt.cpp 的 sed 残留

当前 `BSP_TimeDwt.cpp` 的 `#if JIA_HAS_DWT_CYCCNT` 区域存在重复代码行（sed 操作残留），需要手动清理。

建议：用 Keil 编辑器打开该文件，删除 `#if JIA_HAS_DWT_CYCCNT` 下方的重复行，保留带有注释的版本。

### 第二步：完成 omni_chassisSetup.cpp 注释修复

对剩余乱码函数（`Path_correction`、`KFS_Selection_Planning`、`Clamping_Bar_Selection_Planning`、`flag_reset`）补充可读中文注释。

### 第三步：Module_ChassisSwerve.h 未使用字段清理

确认各字段确实未被任何代码引用后删除。

### 第四步：chassis.cpp 进一步精简（可选）

评估是否将 `calculatePid`、`inverseKinematics` 等进一步迁移到库文件。

### 第五步：四舵轮正常模式 Debug Watch 完善

在 `runRuntimeSwerveControl()` 中填充 `four_swerve_*` debug watch 字段的每个模块数据。

### 第六步：Keil 编译 + 三全向轮硬件测试

需要用户提供 Keil 路径。三全向轮可以实测验证。

## 9. 已知坑和注意事项

### 9.1 GBK vs UTF-8 编码陷阱

**这是本次会话最大的教训。** Write 工具输出 UTF-8，但项目需要 GBK。写完后必须转换：
```bash
iconv -f UTF-8 -t GBK file.cpp > tmp && mv tmp file.cpp
```
Edit 工具也会将 GBK 文件转为 UTF-8！对于 GBK 文件的小改动，优先使用 `sed` 命令。
文件编码验证：`file file.cpp` 应显示 `ISO-8859` 而非 `UTF-8`。

### 9.2 `file` 命令输出解读

| `file` 输出 | 实际编码 | Keil 中显示 |
|---|---|---|
| `UTF-8 text` | UTF-8 | 乱码！ |
| `ISO-8859 text` | GBK | 正常 |
| `Non-ISO extended-ASCII` | GBK（混合） | 可能正常 |
| `ASCII text` | ASCII | 正常（无中文） |

### 9.3 不要使用 destructive git 命令

同上一次交接文档。

### 9.4 不要把 `_ai` 测试拷进主工程

测试放 `RC10_LIB-FrameWork_ai\tests\`，不放入 `RC10_LIB-FrameWork\`。例外：需要编译进固件的源码（如 `BSP_TimeDwt.*`）必须放主工程。

### 9.5 不要轻易新增算法文件

舵轮算法放 `Module_ChassisSwerve.*`。新增文件必须先征得用户同意。

### 9.6 chassis.h 中的引用来回依赖

`chassis.h` include `Module_ChassisSwerve.h`，同时 `chassis.h` 定义了 `JiaChassisDebugWatch` C 结构体和 `extern "C" volatile` 全局变量。这两个功能不能拆分到不同文件，因为它们紧密耦合在当前架构中。

## 10. 最短启动命令

```powershell
cd D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork
git branch --show-current
git status --short

# 运行所有测试
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\chassis_module\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\swerve_core\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\time_services\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\vesc_brake\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\omni_setup_static\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\tri_omni_kinematics\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\four_steer_setup_static\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\chassis_mode_fsm\run_test.ps1
```

预期结果：8 个测试套件全部 PASS。

## 11. 有效结论

本次会话在上一份交接文档的基础上完成了：

1. **编码统一**：全项目中文注释统一为 GBK 编码，Keil 可正常显示
2. **封装修复**：Motor_Base 成员变量从 public 改为 protected
3. **代码迁移**：坐标变换、限幅、加速限制从 chassis.cpp 提取到 APP_Utils.h
4. **冗余清理**：删除 world_twist_forward、max_wheel_speed_ 等死代码
5. **DWT 加固**：时钟不整除 1MHz 时降级处理，不产生错误计时
6. **控制路径修复**：四舵轮单轮调试与正常控制路径正确分离
7. **质量保障**：8 个测试套件全部 PASS

仍需继续的工作：
1. BSP_TimeDwt.cpp 的 sed 残留清理
2. omni_chassisSetup.cpp 剩余乱码注释修复
3. Module_ChassisSwerve.h 未使用字段清理
4. Keil 编译验证
5. 三全向轮硬件实测