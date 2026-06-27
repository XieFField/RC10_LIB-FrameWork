# Jia6 底盘重构与舵轮迁移交接文档

生成时间：2026-05-07 23:05（Asia/Shanghai）

主工程路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork`

AI 资料路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai`

当前分支：`Jia6_temp`

## 1. 文档目的

这份文档给下一个模型或后续接手者使用，记录当前 `Jia6_temp` 分支上的真实工作树状态、已经完成的代码改动、已验证测试、仍未完成的风险点，以及继续开发时必须遵守的边界。

这不是最终验收报告。当前主工程工作树仍是 dirty 状态，尚未提交，尚未合并，尚未推送。

## 2. 最重要的约束

1. 不允许对 `main` 执行任何提交、合并、推送或上传操作。
2. 最终集成方式必须是用户手动从 `main` 主动合并缓冲分支，不能由 agent 代替用户操作 `main`。
3. 开发只应围绕 `Jia6_temp` / `Jia6` 体系推进。
4. AI 相关测试、文档、记录只放在 `D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai`，不要放入主工程目录。
5. 用户允许主要负责修改的范围包括：
   - `User\Setup\Inc\chassis.h`
   - `User\Setup\Src\chassis.cpp`
   - `RC10_LIB\Module\Inc\Module_ChassisBase.h`
   - `RC10_LIB\Module\Inc\Module_ChassisOmni.h`
   - `RC10_LIB\Module\Inc\Module_ChassisSwerve.h`
   - `RC10_LIB\APP\Inc\APP_Utils.h`
   - 上述文件对应的源文件或头文件
6. 上述范围之外的文件尽量最小修改。
7. 舵轮算法优先放入已有 `RC10_LIB\Module\Inc\Module_ChassisSwerve.h` 和 `RC10_LIB\Module\Src\Module_ChassisSwerve.cpp`，不要新建算法文件。除已经批准的 `BSP_TimeDwt.*`、`BSP_TimeUs64.*` 外，如需新增文件必须先征得用户同意。
8. 调试器可见全局变量采用 `extern "C"` + `volatile` + 扁平 C struct，不放在 C++ namespace 下，避免 Keil Watch 对 C++ namespace 符号访问不稳定。
9. 不改 VOFA 现有 18 通道顺序。
10. 当前四舵轮无法上硬件实测；三全向轮在本轮修改后有硬件可以检验。

## 3. 当前 git 状态快照

命令：

```powershell
git branch --show-current
git status --short
git diff --stat
git diff --check
```

结果摘要：

```text
branch: Jia6_temp
```

当前主工程 dirty 文件：

```text
 M Frame_T.uvprojx
 M MDK-ARM/Frame_T.uvprojx
 M RC10_LIB/APP/Inc/APP_Utils.h
 M RC10_LIB/Module/Inc/Module_ChassisBase.h
 M RC10_LIB/Module/Inc/Module_ChassisOmni.h
 M RC10_LIB/Module/Inc/Module_ChassisSwerve.h
 M RC10_LIB/Module/Src/Module_ChassisSwerve.cpp
 M RC10_LIB/Motor/Inc/Motor_Base.h
 M User/Setup/Inc/chassis.h
 M User/Setup/Inc/omni_chassisSetup.h
 M User/Setup/Src/chassis.cpp
 M User/Setup/Src/omni_chassisSetup.cpp
?? RC10_LIB/BSP_Driver/Inc/BSP_TimeDwt.h
?? RC10_LIB/BSP_Driver/Inc/BSP_TimeUs64.h
?? RC10_LIB/BSP_Driver/Src/BSP_TimeDwt.cpp
?? RC10_LIB/BSP_Driver/Src/BSP_TimeUs64.cpp
```

`git diff --stat` 当前摘要：

```text
 Frame_T.uvprojx                              |  30 ++
 MDK-ARM/Frame_T.uvprojx                      |  30 ++
 RC10_LIB/APP/Inc/APP_Utils.h                 | 252 ++++++------
 RC10_LIB/Module/Inc/Module_ChassisBase.h     | 109 +++---
 RC10_LIB/Module/Inc/Module_ChassisOmni.h     |  44 +--
 RC10_LIB/Module/Inc/Module_ChassisSwerve.h   | 206 +++++++++-
 RC10_LIB/Module/Src/Module_ChassisSwerve.cpp | 556 ++++++++++++++++++++++++++-
 RC10_LIB/Motor/Inc/Motor_Base.h              |  48 +--
 User/Setup/Inc/chassis.h                     | 539 ++++++++++++++------------
 User/Setup/Inc/omni_chassisSetup.h           | 115 +++---
 User/Setup/Src/chassis.cpp                   | 212 ++++++++--
 User/Setup/Src/omni_chassisSetup.cpp         | 150 ++++----
 12 files changed, 1646 insertions(+), 645 deletions(-)
```

`git diff --check` 结果：

```text
没有 whitespace error。
只有多处 “LF will be replaced by CRLF the next time Git touches it” 提示。
```

注意：`git diff --stat` 不统计 untracked 的 4 个新增 BSP 时间文件。

## 4. 当前测试状态

测试都放在：

`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests`

当前已有 5 组 host 测试：

```text
tests\chassis_module
tests\swerve_core
tests\time_services
tests\vesc_brake
tests\omni_setup_static
```

2026-05-07 23:05 复跑结果：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\chassis_module\run_test.ps1
# chassis_module test: PASS

powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\swerve_core\run_test.ps1
# swerve_core test: PASS

powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\time_services\run_test.ps1
# time_services test: PASS

powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\vesc_brake\run_test.ps1
# PASS
# vesc_brake test: PASS

powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\omni_setup_static\run_test.ps1
# omni_setup_static test: PASS
```

测试技术栈：

```text
不引入 GoogleTest/Catch2。
使用最小 C++ harness：main() + EXPECT_TRUE/EXPECT_NEAR/printf。
PowerShell run_test.ps1 调 MinGW g++ 编译运行。
静态检查类测试直接用 PowerShell 读取源码。
```

当前 MinGW 路径来自测试脚本：

```text
C:\Qt\Tools\mingw1310_64\bin\g++.exe
```

## 5. 已完成的主要工作

### 5.1 VESC 刹车语义

目标：让 RPM 模式和刹车模式边界清晰，避免 `setTargetRPM(0)` 被 VESC 层隐藏改成刹车。

当前状态：

1. `Motor_Base` 已新增虚接口：

```cpp
virtual void setBrake(float brake_current)
{
    (void)brake_current;
};
```

2. `VESC_Motor` 当前代码里已有：

```cpp
void setBrake(float brake_current) override;
float target_brake_current_ = 0.0f;
```

3. `Motor_VESC.cpp` 当前行为：

```cpp
VESC_Motor::setTargetRPM(float rpm_set)
{
    target_rpm_ = rpm_set;
    mode_ = SET_eRPM;
    target_eRPM_ = RPM_to_eRPM(rpm_set);
    reset_otherParam();
}

VESC_Motor::setBrake(float brake_current)
{
    target_brake_current_ = brake_current;
    mode_ = SET_BRAKE;
    reset_otherParam();
}
```

4. `SET_eRPM` 分支发送 `CAN_CMD_SET_ERPM`，即使目标 eRPM 是 0 也仍走 ERPM。
5. `SET_BRAKE` 分支发送 `CAN_CMD_SET_CURRENT_BRAKE`。
6. `_ai/tests/vesc_brake` 已验证：
   - `setTargetRPM(0)` 发送 ERPM 0，不进入 brake。
   - `setBrake(70000)` 发送 current brake。
   - `setTargetRPM(120)` 在 21 pole-pair 下转换为 2520 eRPM。

注意：

`RC10_LIB\Motor\Inc\Motor_VESC.h` 和 `RC10_LIB\Motor\Src\Motor_VESC.cpp` 当前没有出现在 dirty diff 中，说明这些 VESC 侧改动应已经存在于当前基线或前序工作中。当前 dirty diff 只有 `Motor_Base.h` 仍有相关改动。

### 5.2 FourSteerChassis 单舵轮调试刹车路径

当前 `User\Setup\Inc\chassis.h` / `User\Setup\Src\chassis.cpp` 中，`FourSteerChassis::Chassis` 已加入轮向刹车调试变量：

```cpp
bool is_drive_force_brake_enabled_ = false;
bool is_drive_zero_rpm_brake_enabled_ = true;
f32 drive_force_brake_current_ = 70000.0f;
f32 drive_zero_rpm_brake_current_ = 70000.0f;
f32 drive_zero_rpm_threshold_rpm_ = 30.0f;
```

已新增底盘层封装：

```cpp
void setDriveWheelBrake(WheelConfig &wheel, f32 brake_current);
void applyDriveWheelDebugCommand(WheelConfig &wheel, f32 drive_target_rpm);
```

`applyDriveWheelDebugCommand(...)` 当前优先级：

1. `is_drive_force_brake_enabled_ == true`：强制刹车，使用 `drive_force_brake_current_`。
2. 否则，如果零速刹车开启且 `abs(drive_target_rpm) <= drive_zero_rpm_threshold_rpm_`：零速显式刹车，使用 `drive_zero_rpm_brake_current_`。
3. 否则：正常 `setDriveWheelTargetRpm(wheel, drive_target_rpm)`。

`isDebugMode()` 内单舵轮轮向下发现在统一走：

```cpp
applyDriveWheelDebugCommand(wheel, drive_target_rpm);
```

校准期间当前逻辑仍先把 `drive_target_rpm = 0.0f`，再经过同一个 helper；如果零速刹车开着，校准期间轮向会被显式刹住。

VOFA 18 通道顺序没有改。

### 5.3 Debug Watch 全局符号

新增了 Keil Watch 友好的 C 链接全局快照：

头文件：`User\Setup\Inc\chassis.h`

```cpp
extern "C" {
typedef struct JiaChassisDebugWatch
{
    ...
} JiaChassisDebugWatch;

extern volatile JiaChassisDebugWatch g_jia_chassis_debug_watch;
}
```

定义文件：`User\Setup\Src\chassis.cpp`

```cpp
extern "C" {
volatile JiaChassisDebugWatch g_jia_chassis_debug_watch = {};
}
```

字段覆盖：

1. 当前底盘类型：`active_chassis_type`，约定 3=三全向，4=四舵轮。
2. 任务循环统计：`loop_count`、`last_tick_ms`。
3. DWT 统计：last cycles、last/min/max us。
4. Us64 统计：last/min/max us。
5. 当前模式和调试模式。
6. 三全向目标、规划和反馈轮速。
7. 四舵轮单轮调试索引、光电门、校准状态、舵向和轮向观测量。
8. 四舵轮刹车状态：`four_drive_brake_mode`，0=rpm，1=zero-rpm brake，2=force brake。

设计原因：

Keil/Arm µVision 对 C++ namespace 符号 Watch 访问不稳定，用户之前明确要求参考 `baby_car_ai_coding` 的扁平全局变量做法。因此对外调试入口不用 `jia::...` namespace 变量，而是 `extern "C"` 扁平符号。

### 5.4 DWT + RTOS 统一 64 位时间戳

已新增 4 个文件：

```text
RC10_LIB\BSP_Driver\Inc\BSP_TimeDwt.h
RC10_LIB\BSP_Driver\Src\BSP_TimeDwt.cpp
RC10_LIB\BSP_Driver\Inc\BSP_TimeUs64.h
RC10_LIB\BSP_Driver\Src\BSP_TimeUs64.cpp
```

`BSP_TimeDwt`：

1. 使用 DWT CYCCNT 做短区间周期/微秒统计。
2. `Init(core_clock_hz)` 保存 HCLK 和 cycles/us。
3. host 测试环境没有 `USE_HAL_DRIVER` 时返回 0，便于 PC 测试。

`BSP_TimeUs64`：

1. 使用 FreeRTOS tick + SysTick 当前计数合成 64 位微秒时间。
2. 不另开硬件定时器，符合用户“与 RTOS 时钟统一”的要求。
3. host 测试环境返回 0，同时暴露纯函数 `TicksToUs64`、`ComposeTimeUs64` 供测试。

已接入 `User\Setup\Src\chassis.cpp`：

```cpp
InitChassisPerfCounter();
PublishChassisPerfCounter(...)
```

三全向和四舵轮任务循环中均记录：

```cpp
const uint32_t chassis_task_start_cycle = TimeDwt::GetCycle32();
const uint64_t chassis_task_start_us = TimeStampUs64::GetTimeUs();
...
PublishChassisPerfCounter(...)
```

注意：

当前未做 Keil/MDK 实机编译，因为环境 PATH 中没有确认可用的 Keil/UV4/UV5/armclang。之前建议：如果需要 MDK 编译，让用户提供 Keil/UV4/UV5/armclang 路径，不要全盘递归扫盘。

### 5.5 Keil 工程文件接入

`Frame_T.uvprojx` 和 `MDK-ARM\Frame_T.uvprojx` 已加入：

```text
RC10_LIB\Module\Src\Module_ChassisSwerve.cpp
RC10_LIB\BSP_Driver\Inc\BSP_TimeDwt.h
RC10_LIB\BSP_Driver\Src\BSP_TimeDwt.cpp
RC10_LIB\BSP_Driver\Inc\BSP_TimeUs64.h
RC10_LIB\BSP_Driver\Src\BSP_TimeUs64.cpp
```

当前 `Select-String` 能在两个 uvprojx 中找到上述文件条目。

之前 XML 解析曾通过，但本次交接前没有重新执行 XML 解析脚本。继续前建议再用 PowerShell 或 Python 解析一次两个 uvprojx。

### 5.6 Module_ChassisSwerve 舵轮算法迁移

当前已在已有文件中承载舵轮算法：

```text
RC10_LIB\Module\Inc\Module_ChassisSwerve.h
RC10_LIB\Module\Src\Module_ChassisSwerve.cpp
```

没有新增舵轮算法文件，符合用户要求。

已实现的主要类型：

```cpp
namespace jia::swerve
{
constexpr std::size_t kModuleCount = 4;

enum class IdlePostureMode;
enum class HomingState;

struct Vec2;
struct WheelGeometry;
struct HomingConfig;
struct SharedLimits;
struct SwerveModuleConfig;
struct SwerveConfig;
struct ChassisCommand;
struct ModuleFeedback;
struct ModuleCommand;
struct HomingSensorSample;
struct HomingTracker;
struct ModuleSnapshot;
struct SimulationStepRecord;

class SwerveController;
}
```

已实现的核心函数：

```cpp
wrapToPi(...)
wrapTo2Pi(...)
clampValue(...)
shortestAngularDistance(...)
nearestEquivalentAngle(...)
magnitude(...)
makeXParkAngle(...)
estimateChassisMotion(...)
toString(HomingState)
isHomingReady(...)
resetHomingTracker(...)
applyHomingCorrection(...)
updateHomingTracker(...)
SwerveController::step(...)
```

`SwerveController::step(...)` 当前支持：

1. 根据车体 `vx/vy/wz` 对每个舵轮计算局部轮速向量。
2. 静止时 HoldLast 或 XPark。
3. 最短舵向选择。
4. 必要时驱动反向，减少舵角转动。
5. 余弦补偿。
6. 轮向角速度限幅。
7. 舵向二阶运动限制。
8. 轮向加速度限制。

`_ai/tests/swerve_core` 当前覆盖：

1. 角度 wrap/最短角距离。
2. 纯 `vx` 输出。
3. 最短转向导致驱动反向。
4. 四舵轮正解估计。

### 5.7 Module_ChassisBase / Module_ChassisOmni 初步重构

`Module_ChassisBase.h` 当前已做了兼容性方向的修正：

1. `set_ControlMode`、`set_Target`、`registerWheelMotor`、`getWheelTargetRPM` 旧接口保留。
2. `CURRENT_ZERO_MODE` 和 `SPEED_ZERO_MODE` 不再硬编码访问前三/四个轮，而是遍历 `WheelCount`。
3. 遍历时对 `wheels_[i] != nullptr` 做保护，避免空指针电机注册时崩溃。
4. `registerWheelMotor` 对越界 index 返回 false。

`Module_ChassisOmni.h` 当前改动较小，主要与 `Chassis_Base` 新行为兼容。

`_ai/tests/chassis_module` 当前覆盖：

1. `Chassis_Omni<3>` 注册越界检查。
2. 零电流/零速度模式只访问已注册电机。
3. 三全向 `ROBOT_SPEED_MODE` 逆解并下发 RPM。
4. 当任一轮超过最大 RPM 时，所有轮统一按比例缩放。

### 5.8 omni_chassisSetup 越界修复

`User\Setup\Inc\omni_chassisSetup.h` 中 `OmniChassis_Setup::init()` 原先对 `Chassis_Omni<3>` 检查了 `wheels_[3]`，存在越界风险。

当前已修复为循环检查 0..2：

```cpp
init_flag = false;
for (uint8_t i = 0; i < 3; ++i)
{
    if (this->wheels_[i] == nullptr)
        return;
}
```

新增静态测试：

```text
_ai\tests\omni_setup_static\run_test.ps1
```

该测试读取源码，阻止 `Chassis_Omni<3>` setup 再出现 `wheels_[3]`。

## 6. 目前没有完成或需要谨慎继续的部分

### 6.1 FourSteerChassis 尚未真正完整接入 SwerveController

当前 `Module_ChassisSwerve` 算法已迁移并测试通过，但 `User\Setup\Src\chassis.cpp` 中 `FourSteerChassis::isDebugMode()` 仍主要是单舵轮调试路径。

现状：

1. 单轮调试可以选择舵向 rpm/current/单圈角/多圈角模式。
2. 轮向 debug RPM 经过 `applyDriveWheelDebugCommand` 支持显式刹车。
3. 光电门校准逻辑仍在单轮 debug path 内。
4. `debug_mode_` 仍只是把 `setTargetBodySpeedMode` 等目标写入 `input_target_data_`，但没有完整在四轮上使用 `SwerveController::step` 输出四个舵轮目标。

下一步应做：

1. 先写 `_ai` 测试，静态检查或 host harness，确保 `FourSteerChassis` 的四轮路径会调用 `SwerveController`。
2. 再在 `FourSteerChassis` 中增加 `swerve::SwerveConfig`、`swerve::SwerveController` 或可重置配置的 controller。
3. 避免大改单轮调试路径。建议保留单轮调试作为硬件 bring-up 模式，再增加四轮正常控制路径。

被中断前准备做但尚未执行的下一步：

```text
补两个最小测试：
1. 要求 SwerveController 支持运行期重新配置。
2. 静态检查 FourSteerChassis setup 是否真正调用库层舵轮算法。
```

这两个测试还没有创建，主工程也还没有因此发生改动。

### 6.2 SwerveController 当前配置不可运行期替换

当前 `SwerveController` 构造函数接收 `const SwerveConfig& config` 并复制到 `config_`。

如果 `FourSteerChassis::init()` 中需要先默认构造 `Chassis`，后续再根据实际 wheel geometry 配置 controller，那么需要一种运行期配置策略：

方案 A：

```cpp
SwerveController controller_;
void configure(const SwerveConfig& config);
```

但当前没有默认构造函数，也没有 `configure`。

方案 B：

```cpp
SwerveConfig swerve_config_;
SwerveController swerve_controller_{swerve_config_};
```

然后在构造/成员初始化时给默认 config，`init()` 中如果需要改配置则可能要新增 `setConfig` 或重新构造对象。嵌入式代码不建议引入动态分配。

建议：

先用测试锁住 `configure(...)` 行为，再最小化新增：

```cpp
SwerveController();
void configure(const SwerveConfig& config);
```

`configure` 内复制 config 并 `reset()`。

### 6.3 代码编码/注释 diff 噪声很大

当前 diff 中存在大量中文注释编码变化。典型文件：

```text
RC10_LIB\APP\Inc\APP_Utils.h
RC10_LIB\Module\Inc\Module_ChassisBase.h
RC10_LIB\Module\Inc\Module_ChassisOmni.h
User\Setup\Inc\chassis.h
User\Setup\Src\chassis.cpp
User\Setup\Inc\omni_chassisSetup.h
User\Setup\Src\omni_chassisSetup.cpp
RC10_LIB\Motor\Inc\Motor_Base.h
```

风险：

1. 功能改动被注释编码噪声淹没，review 困难。
2. 未来与 main/Jia6 合并冲突会显著放大。
3. 对用户“不方便修改别人文件”的边界不友好。

建议下一步优先收口：

1. 不要盲目 `git checkout --`，因为里面混有真实功能改动。
2. 对每个文件用 `git diff` 单独看，区分真实功能改动和纯注释编码改写。
3. 优先恢复纯注释/编码变动，保留真实功能行。
4. 如要批量恢复，请先保存 patch 或分块使用 `apply_patch`，不要 destructive 操作。

特别注意：

`RC10_LIB\Motor\Inc\Motor_Base.h` 当前 diff 里不仅有 `setBrake` 和 getter 默认返回内部字段的真实改动，也有大量注释乱码变化。不要整文件回退，否则会丢 `setBrake` 等必要功能。

### 6.4 Motor_Base getter 默认返回内部字段的影响需确认

当前 `Motor_Base` 的默认 getter 从原先返回 0 改成了返回基类字段：

```cpp
virtual float getRPM() const { return rpm_; }
virtual float getCurrent() const { return current_; }
virtual float getAngle() const { return angle_; }
virtual float getTotalAngle() const { return totalAngle_; }
```

这有助于抽象指针 `Motor_Base*` 下读取通用状态，但它是对外部电机基类行为的改变。需要确认：

1. 所有派生类是否本来就覆盖 getter。
2. 基类字段是否会被派生类同步更新。
3. 对未覆盖 getter 的电机类型，这种默认行为是否符合预期。

如果不确定，建议保留 `setBrake`，但谨慎评估 getter 默认返回字段是否必须；这是潜在合并风险点。

### 6.5 私有成员访问问题只做了部分抽象化

用户之前指出 `chassis.cpp` 中存在依赖电机类内部变量公开化的问题，例如舵向 total angle 读取。

当前代码路径已经更多使用 getter：

```cpp
wheel.smh->getTotalAngle()
wheel.smh->getTargetTotalAngle()
wheel.dmh->getTotalAngle()
```

但仍需继续检查：

1. `M3508` / DJI 电机类中这些 getter 是否本来就是稳定 public 接口。
2. `relocate_totalAngle(...)` 是否是合适的校准接口。
3. 未来合并 main 前，不能依赖“为了调试把私有成员改 public”的状态。

建议下一步用静态搜索检查：

```powershell
Select-String -Path User\Setup\Src\chassis.cpp,User\Setup\Inc\chassis.h -Pattern 'totalAngle_|angle_|rpm_|current_|target_'
```

然后只对必要外部类补最小 getter/setter，不做大范围外部重构。

### 6.6 Keil/MDK 编译尚未验证

当前只验证了 host 测试。

未验证：

1. Keil 工程完整编译。
2. armclang/armcc 对新增 C++17 或头文件内容的兼容性。
3. `BSP_TimeUs64.cpp` 中 FreeRTOS critical section、SysTick 寄存器宏在当前 STM32H723 工程下是否全部可见。
4. `TimeDwt` 的 DWT 条件编译宏是否在 Keil 下正确打开。

建议继续前先确认 Keil 命令行路径：

```text
UV4.exe / UV5.exe / armclang / armcc
```

不要全盘递归扫盘，用户之前倾向明确路径后再执行。

## 7. 文件级说明

### 7.1 `RC10_LIB\Module\Inc\Module_ChassisSwerve.h`

当前是舵轮算法对外头文件。

包含：

1. 舵轮几何配置。
2. homing 状态机配置。
3. 共享限幅参数。
4. 四模块 command/feedback/snapshot 结构。
5. 纯函数声明。
6. `SwerveController` 类声明。

注意：

当前没有继承 `Chassis_Base`，而是独立 `jia::swerve` 算法命名空间。这样便于 host 测试和后续嵌入现有 `FourSteerChassis`。

### 7.2 `RC10_LIB\Module\Src\Module_ChassisSwerve.cpp`

当前是核心算法实现。

重点审查点：

1. `solve3x3` 是内部最小线性方程求解，用于正解估计。
2. `estimateChassisMotion` 当前将每个舵轮分解为沿轮向速度和横向 0 约束，做最小二乘正解。
3. `updateHomingTracker` 当前是可测试的 homing 状态机，但 FourSteer 实际光电门校准还没有接入这个 homing tracker。
4. `SwerveController::step` 当前默认一次处理 4 个模块。

### 7.3 `User\Setup\Inc\chassis.h`

当前包含：

1. `JiaChassisDebugWatch` C struct。
2. `extern volatile JiaChassisDebugWatch g_jia_chassis_debug_watch;`
3. 三全向 `TriOmniChassis::Chassis` 原有 setup 类。
4. 四舵轮 `FourSteerChassis::Chassis` 原有 setup 类，已加入刹车调试变量和 helper 声明。

风险：

1. 文件 diff 很大，很多是中文注释编码变化。
2. FourSteer 仍未形成清晰的库层 `Module_ChassisSwerve` 调用边界。
3. 继续重构时要避免一次性把三全向和四舵轮都大改，优先用测试锁住行为。

### 7.4 `User\Setup\Src\chassis.cpp`

当前包含：

1. `g_jia_chassis_debug_watch` 定义。
2. DWT/Us64 初始化与发布 helper。
3. 三全向任务循环性能统计和 debug watch 写入。
4. 四舵轮任务循环性能统计和 debug watch 写入。
5. FourSteer 单舵轮调试刹车路径。

重要现状：

`FourSteerChassis::runThread()` 当前仍是：

```cpp
ihrz_ = hwt->get_yaw_rad();
ihoz_ = hwt->get_yaw_speed_rad();

isDebugMode();

PublishChassisPerfCounter(4U, ...);
vTaskDelayUntil(&time_ms_, period_ms_);
```

也就是说四舵轮正常运动学主循环尚未接入 `SwerveController::step`。

### 7.5 `RC10_LIB\BSP_Driver\Inc/Src\BSP_TimeDwt.*`

新增文件，已加入 uvprojx。

host 测试覆盖：

1. `CyclesToUs32`
2. `GetElapsedCycles32`
3. `GetElapsedUs32`
4. 低于 1MHz core clock 时 cycles/us fallback 为 1

### 7.6 `RC10_LIB\BSP_Driver\Inc/Src\BSP_TimeUs64.*`

新增文件，已加入 uvprojx。

host 测试覆盖：

1. `TicksToUs64`
2. `ComposeTimeUs64`
3. sub tick 超过一个 tick 周期时 clamp
4. host 环境 `GetTimeUs() == 0`

### 7.7 `User\Setup\Inc/Src\omni_chassisSetup.*`

当前混有真实修复和大量注释编码变化。

真实修复：

1. `OmniChassis_Setup::init()` 不再访问 `wheels_[3]`。
2. 自动控制状态 reset、前馈注释等有中文可读化变化，但这些多数是注释噪声。

建议继续清理：

1. 保留越界修复。
2. 如无必要，尽量恢复纯注释编码 diff，降低合并冲突。

## 8. 当前 `_ai` 测试资产说明

### 8.1 `tests\chassis_module`

用途：锁住 `Module_ChassisBase` / `Module_ChassisOmni` 基础行为。

运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\chassis_module\run_test.ps1
```

覆盖：

1. 轮电机注册越界。
2. 零电流/零速度模式空指针安全。
3. 三全向 robot speed 逆解。
4. 轮速统一缩放。

### 8.2 `tests\swerve_core`

用途：锁住 `Module_ChassisSwerve` 纯算法。

运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\swerve_core\run_test.ps1
```

覆盖：

1. 角度 wrap。
2. 纯 vx 输出。
3. 最短转向/驱动反向。
4. 四舵轮正解。

### 8.3 `tests\time_services`

用途：锁住 `BSP_TimeDwt` / `BSP_TimeUs64` 纯函数换算。

运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\time_services\run_test.ps1
```

覆盖：

1. DWT cycle -> us。
2. 32 位差值 wrap 行为。
3. RTOS tick -> us。
4. tick + sub tick 组合。

### 8.4 `tests\vesc_brake`

用途：锁住 VESC 显式刹车语义。

运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\vesc_brake\run_test.ps1
```

覆盖：

1. RPM 0 仍发送 ERPM。
2. `setBrake(70000)` 发送 CURRENT_BRAKE。
3. 正 RPM eRPM 转换。

### 8.5 `tests\omni_setup_static`

用途：静态防回归，阻止 `Chassis_Omni<3>` setup 再访问 `wheels_[3]`。

运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\omni_setup_static\run_test.ps1
```

覆盖：

1. `User\Setup\Inc\omni_chassisSetup.h` 内不得出现 `wheels_[3]`。

## 9. 建议的继续顺序

### 第一步：收口 diff 噪声

目标：降低未来合并风险。

建议命令：

```powershell
git diff -- RC10_LIB\APP\Inc\APP_Utils.h
git diff -- RC10_LIB\Module\Inc\Module_ChassisBase.h
git diff -- RC10_LIB\Module\Inc\Module_ChassisOmni.h
git diff -- User\Setup\Inc\omni_chassisSetup.h
git diff -- User\Setup\Src\omni_chassisSetup.cpp
git diff -- RC10_LIB\Motor\Inc\Motor_Base.h
```

处理原则：

1. 保留真实功能改动。
2. 恢复纯注释/编码变动。
3. 每处理一个文件后跑对应 host 测试。

### 第二步：为 SwerveController 运行期配置写测试

建议新增测试到：

```text
D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\swerve_core
```

测试目标：

1. 默认构造 controller 后可 `configure(config)`。
2. `configure` 后 `config()` 返回新配置。
3. `configure` 会 reset 内部限幅历史，避免旧速度/舵向 rate 残留。

先看测试失败，再改 `Module_ChassisSwerve.h/.cpp`。

### 第三步：接入 FourSteerChassis 四轮正常控制路径

建议最小路径：

1. 在 `FourSteerChassis::Chassis` 中新增 `swerve::SwerveConfig` 和 `swerve::SwerveController` 成员。
2. `init()` 中基于现有四轮位置配置 `SwerveConfig`。
3. 在非单轮调试模式或特定 `debug_mode_` 中调用 `SwerveController::step`。
4. 每个轮：
   - 舵向：`selected_target_steer_motor_total_angle_rad` 转为 deg 后调用 `setSteerWheelTargetTotalAngleDeg`。
   - 轮向：`selected_target_drive_omega_rad_s` 转 RPM 后调用 `applyDriveWheelDebugCommand` 或新的统一轮向输出 helper。
5. 先不要删除单轮调试逻辑，因为当前单轮硬件调试路径仍有价值。

建议新增 `_ai` 静态或 host 测试：

1. 检查 `chassis.cpp` 中 FourSteer 正常路径包含 `SwerveController::step`。
2. 如能抽出纯函数，直接 host 测试四轮命令映射。

### 第四步：继续清理 Module_ChassisBase / Omni 过期接口

不要一次性删除太多。

每删一个旧函数或变量前：

1. 用 `Select-String` 搜索引用。
2. 补 `_ai` 测试或静态检查。
3. 确认三全向 setup 仍能工作。

### 第五步：Keil 编译

需要用户提供 Keil 路径后执行。

建议验证：

1. 两个 uvprojx XML 可解析。
2. 主工程完整编译。
3. `BSP_TimeDwt` / `BSP_TimeUs64` 宏在 STM32H723 + FreeRTOS 环境可编译。
4. `g_jia_chassis_debug_watch` 能在 Keil Watch 中直接访问。

### 第六步：硬件验证

三全向：

1. 确认 `OmniChassis_Setup::init()` 不再因 `wheels_[3]` 越界导致异常。
2. 验证 `CURRENT_ZERO_MODE` / `SPEED_ZERO_MODE` 只作用于注册电机。
3. 验证路径/自动控制逻辑无回归。

四舵轮：

1. 当前只能做编译、静态输出、debug watch 观察。
2. 不声明硬件通过。
3. 单轮 VESC 刹车可沿用原单轮台架方式测试。

## 10. 注意事项和已知坑

### 10.1 `rg.exe` 当前曾出现 Access is denied

之前运行 `rg` 搜索时出现：

```text
Program 'rg.exe' failed to run: Access is denied
```

可用 PowerShell `Select-String` 代替：

```powershell
Select-String -Path 'file1','file2' -Pattern 'xxx' -Context 1,3
```

### 10.2 不要使用 destructive git 命令

不要执行：

```powershell
git reset --hard
git checkout -- .
```

当前 dirty tree 里混有真实功能和编码噪声，粗暴回退会丢工作。

### 10.3 不要改 main

任何涉及 `main` 的命令都应停止并让用户手动操作。

禁止 agent 执行：

```powershell
git checkout main
git switch main
git merge Jia6
git merge Jia6_temp
git push origin main
```

### 10.4 不要把 `_ai` 测试拷进主工程

测试、文档、记录放这里：

```text
D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai
```

不要放这里：

```text
D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork
```

例外：真正要编译进固件的源码，例如已批准的 `BSP_TimeDwt.*`、`BSP_TimeUs64.*`，必须放主工程。

### 10.5 不要轻易新增算法文件

用户已明确要求舵轮算法优先放在已有：

```text
RC10_LIB\Module\Inc\Module_ChassisSwerve.h
RC10_LIB\Module\Src\Module_ChassisSwerve.cpp
```

如果确实要新建算法文件，先向用户说明必要性并获得同意。

## 11. 下一位接手者的最短启动命令

建议从这里开始：

```powershell
cd D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork
git branch --show-current
git status --short
git diff --stat

powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\chassis_module\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\swerve_core\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\time_services\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\vesc_brake\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\omni_setup_static\run_test.ps1
git diff --check
```

预期当前结果：

```text
branch: Jia6_temp
5 个 host 测试全部 PASS
git diff --check 无 whitespace error，只有 CRLF warning
```

## 12. 建议给用户汇报时的表述边界

可以说：

1. VESC 显式刹车语义已有 host 测试通过。
2. `Module_ChassisSwerve` 核心舵轮算法已迁入库层并有 host 测试通过。
3. DWT + RTOS 统一 Us64 时间戳已接入底盘任务，并有纯函数 host 测试。
4. `g_jia_chassis_debug_watch` 已按 Keil Watch 友好方式暴露。
5. 三全向 setup 的 `wheels_[3]` 越界风险已修复并有静态测试。

不要说：

1. 不要说主工程已完整编译通过，除非后续真的跑过 Keil/MDK 编译。
2. 不要说四舵轮功能已完成，因为 FourSteer 尚未完整接入 `SwerveController::step`。
3. 不要说四舵轮硬件已验证，因为当前没有硬件实测条件。
4. 不要说当前改动已可合并 main，因为 diff 噪声和未完成项仍较多。

## 13. 当前有效结论

当前工作已经完成了底层支撑：

1. VESC 刹车语义清晰化。
2. 舵轮核心算法迁移到 `Module_ChassisSwerve`。
3. 三全向/底盘基类初步安全重构。
4. DWT + RTOS Us64 双统计。
5. Keil Watch 友好全局 debug snapshot。
6. `_ai` host 测试资产可重复验证。

但仍处于“中间集成态”：

1. FourSteer 还没完整使用 `SwerveController` 做四轮下发。
2. 大量中文注释编码 diff 需要收口。
3. Keil 编译未验证。
4. 四舵轮硬件未验证。
5. 还没有提交，也不应该在没收口前推送或合并。

