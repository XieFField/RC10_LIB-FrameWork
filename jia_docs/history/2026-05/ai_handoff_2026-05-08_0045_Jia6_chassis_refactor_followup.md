# Jia6 底盘重构与四舵轮接库交接文档（续）

生成时间：2026-05-08 00:45（Asia/Shanghai）

主工程路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork`

AI 资料路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai`

当前分支：`Jia6_temp`

## 1. 这份文档的用途

这份文档是对上一份

`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\ai_handoff_2026-05-07_2305_Jia6_chassis_refactor.md`

的续写，目的是把本轮后续实现、当前真实验证结果、尚未完成的边界、以及下一个接手人最应该先看的事情全部压实。

这不是“已经全部完成”的验收报告。

当前代码已经明显从“只剩单舵轮 bring-up 链路”的中间态，推进到了“库层舵轮算法可运行期配置 + 四舵轮正常四轮路径已经接到任务循环里 + 现有 host/静态回归全部通过 + `MDK-ARM\Frame_T.uvprojx` 命令行可编译”的状态。

但仍然有几个明确的未完成项，尤其是：

1. 四舵轮真实 4 模块几何/零位/正反号仍然只是默认可配置表，不是实车最终参数。
2. 四舵轮“锁朝向 / 锁到 yaw”语义在新的正常四轮路径里还没有完整重建，只保留了接口和目标写入入口。
3. 四舵轮硬件没有实际联调结论。
4. `g_jia_chassis_debug_watch` 仍主要偏向单舵轮 bring-up 观察，不是完整四轮运行态观测面。

## 2. 最重要的边界条件

这些边界本轮继续严格遵守：

1. 不允许对 `main` 做任何提交、合并、推送、上传操作。
2. 最终集成方式仍然只能由用户自己从 `main` 主动合并缓冲分支。
3. 本轮实现继续只在 `Jia6_temp` 工作线上推进。
4. 舵轮算法继续只放在已有文件：
   - `RC10_LIB\Module\Inc\Module_ChassisSwerve.h`
   - `RC10_LIB\Module\Src\Module_ChassisSwerve.cpp`
5. 除已批准的时间服务文件外，本轮没有新建主工程算法文件。
6. 旧 `BSP_TimeStamp` 没有被全仓库替换掉；底盘局部并行使用 `BSP_TimeDwt` / `BSP_TimeUs64`。
7. 对外调试入口仍然使用扁平 `extern "C"` + `volatile` 的 `g_jia_chassis_debug_watch`，没有改成 namespace 下符号。

## 3. 当前应当视为“唯一有效”的 Keil 工程文件

当前应当只把下面这个文件视为正确工程文件：

`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\MDK-ARM\Frame_T.uvprojx`

当前根目录旧工程文件：

`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\Frame_T.uvprojx`

已经被删除，并且用户已经明确说明它是**过期文件，不是本工程正确工程文件**。

因此：

1. 之后的 MDK/UV4/UV5 编译验证，只应针对 `MDK-ARM\Frame_T.uvprojx`。
2. 不要再把根目录 `Frame_T.uvprojx` 的编译状态作为源码正确与否的依据。

## 4. 本轮实际完成的代码工作

### 4.1 `Module_ChassisSwerve` 现在支持运行期重配置

本轮在：

- `RC10_LIB\Module\Inc\Module_ChassisSwerve.h`
- `RC10_LIB\Module\Src\Module_ChassisSwerve.cpp`

新增了：

```cpp
void configure(const SwerveConfig& config);
```

语义是：

1. 用新的 `config` 覆盖内部 `config_`
2. 调用 `reset()`
3. 清掉内部的 steer / drive 限幅历史

这一步的目的，是解决 `FourSteerChassis::Chassis` 必须先默认构造，再在 `init()` 阶段根据实际轮子布局把 controller 配起来的问题。

本轮没有新增舵轮算法文件，仍然满足“舵轮算法只放在已有 `Module_ChassisSwerve.h/.cpp`”的限制。

### 4.2 四舵轮类内部已经具备运行期 `SwerveConfig` / `SwerveController`

本轮在：

- `User\Setup\Inc\chassis.h`

给 `jia::FourSteerChassis::Chassis` 加入了内部成员：

```cpp
jia::swerve::SwerveConfig swerve_config_;
jia::swerve::SwerveController swerve_controller_{swerve_config_};
RuntimeSwerveDebugSnapshot swerve_runtime_debug_;
```

这里的 `RuntimeSwerveDebugSnapshot` 是类内部调试快照，不是对外全局 watch 结构。

它主要用于：

1. 记录 `config_ready`
2. 记录是否真的跑了 controller step
3. 缓存最近一次 `ChassisCommand`
4. 缓存最近一次 4 模块 `feedback`
5. 缓存最近一次 4 模块 `output`

### 4.3 四舵轮正常四轮路径已经接入任务循环

本轮在：

- `User\Setup\Src\chassis.cpp`

新增并接入了下面这几组 helper：

```cpp
void initWheelConfig(...);
void configureDefaultSwerve();
bool buildRuntimeSwerveMotion(...);
void captureRuntimeSwerveFeedback(...);
void applyRuntimeSwerveCommands(...);
void runRuntimeSwerveControl();
```

核心变化是：

1. `runThread()` 中现在会先执行 `runRuntimeSwerveControl();`
2. `runRuntimeSwerveControl()` 内部会：
   - 组 `ChassisCommand`
   - 采 4 个模块反馈
   - 调 `swerve_controller_.step(...)`
   - 把 steer total angle 目标写回现有 `setSteerWheelTargetTotalAngleDeg(...)`
   - 把 drive omega 目标转成 RPM 再走现有 `applyDriveWheelDebugCommand(...)`

也就是说，四舵轮主线程已经不再是“只有 `isDebugMode()` 单轮路径”的状态。

### 4.4 单舵轮 bring-up / 光电门 / VOFA 路径仍然保留

本轮没有删掉现有单舵轮调试链路。

仍然保留的内容包括：

1. `debug_mode_`
2. `debug_wheel_index_`
3. `buildDebugSetpoint(...)`
4. 单轮 steer 目标生成
5. 单轮 drive RPM / brake 下发
6. 光电门校准流程
7. VOFA 18 通道输出顺序

这符合当前实际需求：四舵轮整车路径已经接库，但硬件还不能整车实测；单轮 bring-up 路径仍然有价值。

### 4.5 四舵轮默认配置表已经落地，但它不是最终实车参数

本轮在 `configureDefaultSwerve()` 中补了一个**默认可配置表**，用来让四舵轮正常路径真正能跑起来。

当前默认值是：

1. `wheel_radius_m = swr_`
2. `max_drive_omega_rad_s = max_wheel_omega_`
3. `stationary_speed_epsilon_m_s = 0.001f`
4. 几何默认用：
   - `x = ±0.37 m`
   - `y = ±0.375 m`
   - `rot = {45, 135, -135, -45} deg`
5. `theta_oa_to_owi_rad = 0`
6. `steer_motor_sign = 1`
7. `drive_motor_sign = 1`
8. `homing.enabled = false`

必须强调：

这只是“让软件结构、host 测试、运行期接线成立”的默认表。

它**不是**四舵轮真实实车的最终几何/零位/符号配置。

后续如果要上真实四舵轮硬件，必须由用户提供或现场确认：

1. 四个模块的真实位置定义
2. 舵向零位偏置
3. steer / drive 正方向
4. `theta_oa_to_owi` 的实际映射

### 4.6 `Module_ChassisBase` 已经按安全逻辑收口

本轮把：

`RC10_LIB\Module\Inc\Module_ChassisBase.h`

重建成干净基线 + 真改动保留的形式。

保留的真实逻辑是：

1. `CURRENT_ZERO_MODE` 不再硬编码访问前 3/4 个轮子
2. 统一按 `WheelCount` 遍历
3. 访问前先检查 `wheels_[i] != nullptr`
4. `SPEED_ZERO_MODE` 同样按遍历 + 空指针保护处理

这项收口已经由 `_ai/tests/chassis_module` 补强测试锁住。

### 4.7 `Motor_Base` 已经保留最小必要抽象补口

本轮把：

`RC10_LIB\Motor\Inc\Motor_Base.h`

重建成干净基线 + 真改动保留的形式。

保留的真实逻辑是：

1. `virtual void setBrake(float brake_current)` 继续存在
2. `getRPM()` / `getCurrent()` / `getAngle()` / `getTotalAngle()` 的默认实现返回内部字段，而不是无条件 `0`
3. `getTargetRPM()` / `getTargetCurrent()` / `getTargetAngle()` / `getTargetTotalAngle()` 继续保留

这保证了四舵轮代码可以通过 `Motor_Base*` 读取/下发统一状态，而不用再依赖“把别人的私有成员临时改 public”。

### 4.8 `omni_chassisSetup.h` 已经清成干净版本，并保留越界修复

本轮把：

`User\Setup\Inc\omni_chassisSetup.h`

重建成干净基线 + 真改动保留的形式。

保留的真改动只有一条：

```cpp
for (uint8_t i = 0; i < 3; ++i)
{
    if (this->wheels_[i] == nullptr)
        return;
}
```

也就是：

1. 不再访问 `wheels_[3]`
2. `Chassis_Omni<3>` 的初始化检查只覆盖 0..2 三个轮位

`omni_chassisSetup.cpp` 已经回退到干净基线，没有继续保留注释编码噪声。

## 5. 当前已经完成的验证

### 5.1 Host / 静态回归

本轮重新跑过并通过：

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

powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\four_swerve_path\run_test.ps1
# four_swerve_path test: PASS
```

### 5.2 本轮新增 / 补强的测试

本轮新增或更新了这些 `_ai` 测试资产：

1. `tests\swerve_core\test_swerve_core.cpp`
   - 增加了 `configure()` 行为测试
   - 覆盖重配置后限幅历史重置

2. `tests\four_swerve_path\run_test.ps1`
   - 静态检查四舵轮类内部存在运行期 `SwerveConfig`
   - 静态检查存在运行期 `SwerveController`
   - 静态检查 `init()` 中调用 `swerve_controller_.configure(swerve_config_)`
   - 静态检查任务循环中存在 `runRuntimeSwerveControl()`
   - 静态检查正常路径存在 `swerve_controller_.step(...)`
   - 静态检查单轮 `applyDriveWheelDebugCommand(...)` 未回退

3. `tests\chassis_module\test_chassis_module.cpp`
   - 新增 4 轮基类零模式空指针安全测试

### 5.3 MDK 编译验证

用户后来提供的 Keil 路径：

`D:\Keil_v5\UV4\UV4.exe`

已经用于命令行编译验证。

#### 5.3.1 正确工程文件

当前应使用：

`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\MDK-ARM\Frame_T.uvprojx`

#### 5.3.2 编译结果

命令行日志摘要如下：

```text
*** Using Compiler 'V6.24', folder: 'D:\Keil_v5\ARM\ARMCLANG\Bin'
Build target 'Frame_T'
...
compiling Module_ChassisSwerve.cpp...
compiling Module_ChassisBase.cpp...
compiling BSP_TimeDwt.cpp...
compiling BSP_TimeUs64.cpp...
compiling Motor_VESC.cpp...
compiling chassis.cpp...
compiling Setup_ConfigInit.cpp...
compiling omni_chassisSetup.cpp...
linking...
Program Size: Code=202064 RO-data=11292 RW-data=696 ZI-data=157528
FromELF: creating hex file...
"Frame_T\Frame_T.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed: 00:00:08
```

产物已生成在：

`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\MDK-ARM\Frame_T`

包括：

1. `Frame_T.axf`
2. `Frame_T.hex`
3. `Frame_T.map`

#### 5.3.3 过期根目录工程文件

根目录旧文件：

`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\Frame_T.uvprojx`

用户已明确说明是过期文件，并且当前已经删除。

因此，之后不要再拿它的编译器版本绑定或编译状态做判断。

## 6. 当前工作树真实状态

本轮文档写作时，`git status --short --branch` 真实结果摘要：

```text
## Jia6_temp...origin/Jia6_temp
 D Frame_T.uvguix.10176
 D Frame_T.uvguix.31463
 D Frame_T.uvguix.86130
 D Frame_T.uvguix.Daisy
 D Frame_T.uvguix.admin
 D Frame_T.uvguix.ayou
 D Frame_T.uvguix.naoganlin
 D Frame_T.uvoptx
 D Frame_T.uvprojx
 M Frame_T/Frame_T.build_log.htm
 M Frame_T/Frame_T_Frame_T.dep
 M MDK-ARM/Frame_T.uvoptx
 M MDK-ARM/Frame_T.uvprojx
 M RC10_LIB/Module/Inc/Module_ChassisBase.h
 M RC10_LIB/Module/Inc/Module_ChassisSwerve.h
 M RC10_LIB/Module/Src/Module_ChassisSwerve.cpp
 M RC10_LIB/Motor/Inc/Motor_Base.h
 M User/Setup/Inc/chassis.h
 M User/Setup/Inc/omni_chassisSetup.h
 M User/Setup/Src/chassis.cpp
 D startup_stm32h723xx.s
 ?? RC10_LIB/BSP_Driver/Inc/BSP_TimeDwt.h
 ?? RC10_LIB/BSP_Driver/Inc/BSP_TimeUs64.h
 ?? RC10_LIB/BSP_Driver/Src/BSP_TimeDwt.cpp
 ?? RC10_LIB/BSP_Driver/Src/BSP_TimeUs64.cpp
 ?? _uv4_build_mdkarm.log
 ?? _uv4_build_root.log
```

这里需要特别注意：

### 6.1 这些删除不是本轮“功能结论”的核心

当前工作树里有一批根目录旧工程文件删除：

1. `Frame_T.uvprojx`
2. `Frame_T.uvoptx`
3. 多个 `Frame_T.uvguix.*`

用户已经明确说这些是过期文件。

### 6.2 `startup_stm32h723xx.s` 当前也是删除态

这个文件当前也显示为删除：

`startup_stm32h723xx.s`

本轮没有恢复它，也没有围绕它做主结论。

但因为 `MDK-ARM\Frame_T.uvprojx` 编译已经通过，所以至少对当前有效工程文件来说，它不是本轮编译路径的 blocker。

后续如果要做工程文件清理，建议由熟悉工程组织的人确认：

1. 它是不是也属于已废弃根目录工程遗留
2. 还是误删但恰好不影响当前 `MDK-ARM` 工程

### 6.3 两个 `_uv4_build_*.log` 是本轮本地验证产物

本轮为命令行编译验证额外生成了：

1. `_uv4_build_mdkarm.log`
2. `_uv4_build_root.log`

这两个文件只是本地验证日志，不属于源码功能改动的一部分。

## 7. 现在还没有完成的内容

这里必须明确，不要把它们误当成“已经闭环”：

### 7.1 四舵轮真实几何 / 零位 / 正反号仍未固化

当前 `configureDefaultSwerve()` 只是默认可配置表。

还缺：

1. 四模块真实 `pos_x / pos_y`
2. `theta_oa_to_owi`
3. steer motor sign
4. drive motor sign
5. 每轮舵向零位偏置

没有这些，不能宣称四舵轮整车实车运动学已正确。

### 7.2 四舵轮 lock-yaw 语义没有完整迁移进新的正常四轮路径

这是当前最需要诚实标出来的一个点。

虽然：

1. `setTargetBodySpeedLockNowRotZMode(...)`
2. `setTargetBodySpeedLockToRotZMode(...)`
3. `setTargetWorldSpeedLockNowRotZMode(...)`
4. `setTargetWorldSpeedLockToRotZMode(...)`

这些接口仍然保留，

但新的 `buildRuntimeSwerveMotion(...)` 目前只稳定处理了：

1. body/world 速度来源
2. 直接 `omega_z` 传入
3. 基础限幅

它没有像三全向那套老路径一样，把四舵轮运行期的 `lock_now_rot_z` / `lock_to_rot_z` 重新完整挂上 PID 与缓冲状态机。

所以当前最准确的说法是：

> 四舵轮“普通 body/world 速度 + 直接 omega_z”路径已经接库层。
> 四舵轮“锁朝向 / 锁到某 yaw”的运行期完整语义还没有做完。

### 7.3 `g_jia_chassis_debug_watch` 仍然偏单轮 bring-up 观察

虽然它仍然是唯一对外 watch 入口，但当前字段更偏：

1. 单轮调试索引
2. 光电门
3. 校准角
4. 单轮 steer / drive 目标与反馈

类内部虽然有 `RuntimeSwerveDebugSnapshot`，

但它目前没有下沉到对外全局 watch struct。

因此：

1. 正常四轮路径“已经在跑”
2. 但 Keil Watch 里“完整 4 轮输出摘要”还没有成为统一对外观测面

### 7.4 四舵轮硬件没有整车实测

本轮没有改变这个事实。

当前能说的是：

1. 代码路径接通了
2. host/静态回归过了
3. `MDK-ARM` 工程编译过了

不能说的是：

1. 四舵轮整车运动学实车通过
2. 四轮舵向/轮向最终方向定义正确

## 8. 下一个接手人建议按这个顺序继续

### 第一步：先确认“当前正确工程文件”共识

只使用：

`MDK-ARM\Frame_T.uvprojx`

不要再围绕根目录旧工程文件展开分析。

### 第二步：确认四舵轮真实机械参数

至少补齐下面这张表：

1. 模块索引 0/1/2/3 分别对应哪一个物理轮位
2. 每轮 `pos_x`
3. 每轮 `pos_y`
4. 每轮 steer 正方向
5. 每轮 drive 正方向
6. 每轮舵向零位偏置
7. 每轮 `theta_oa_to_owi`

没有这张表，就不要急着宣称四舵轮算法正确。

### 第三步：补四舵轮运行期 lock-yaw 语义

建议把当前四舵轮正常路径的 `buildRuntimeSwerveMotion(...)` 扩到与旧三全向语义一致：

1. `lock_now_rot_z`
2. `lock_to_rot_z`
3. 相关缓冲计数
4. 必要时接入 PID

这一步做完后，再补新的 host/静态护栏测试。

### 第四步：扩 `g_jia_chassis_debug_watch`

建议在不破坏原单轮 bring-up 字段的前提下，增加：

1. 四轮是否真正执行了 controller step
2. 最近一次 4 轮 steer total angle 目标摘要
3. 最近一次 4 轮 drive omega 目标摘要
4. 当前配置是否 ready

这样后续实机联调时，调试器和串口观察面会更一致。

### 第五步：再做硬件验证

建议顺序：

1. 先单轮
2. 再双轮对称
3. 最后四轮整车

不要一开始就全车联调。

## 9. 可直接复用的命令

### 9.1 回归测试

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\chassis_module\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\swerve_core\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\time_services\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\vesc_brake\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\omni_setup_static\run_test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork_ai\tests\four_swerve_path\run_test.ps1
```

### 9.2 命令行 MDK 编译

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -b 'D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\MDK-ARM\Frame_T.uvprojx' -j0 -o 'D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork\_uv4_build_mdkarm.log'
```

### 9.3 当前工作树快速查看

```powershell
git status --short --branch
git diff --stat
git diff --check
```

## 10. 当前最准确的一句话总结

当前这条 `Jia6_temp` 工作线已经从“单舵轮 bring-up 为主、四舵轮库层未接入”的中间态，推进到了：

> `Module_ChassisSwerve` 已支持运行期重配置，四舵轮正常四轮路径已经接入任务循环并通过 host/静态回归与 `MDK-ARM\Frame_T.uvprojx` 编译验证，但四舵轮真实几何/零位/正反号、完整 lock-yaw 语义以及整车硬件结论仍然没有完成。

不要把它说成“已经做完四舵轮底盘”，但也不要再把它当成“还只是单舵轮实验代码”。
