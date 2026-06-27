# PID dt 统一注入优化方案

## 背景

当前 `PID_Position::pid_calc()` 和 `PID_Incremental::pid_calc()` 各自独立调用
`TimeStamp::getInstance().getSeconds()` 获取时间戳。

在 1kHz 控制频率下，12 个电机 × 2 层 PID = 每 ms 约 24 次 `getSeconds()` 调用。
但同一控制周期内所有 PID 共享同一个 dt，完全没必要各取各的。

## 优化目标

- 减少 `getSeconds()` 调用次数（从 ~24/ms 降到 ~4/ms）
- 保持功能完全兼容，现有调用方式不受影响
- 不改动 PID 算法逻辑（积分分离、微分先行、TD、死区、环形模式等全部保留）

## 涉及文件

| 文件 | 改动 |
|------|------|
| `RC10_LIB/APP/Inc/APP_PID.h` | `PID_Position` 和 `PID_Incremental` 各新增一个 `pid_calc(target, feedback, dt)` 重载声明 |
| `RC10_LIB/APP/Src/APP_PID.cpp` | 实现上述两个重载函数；不传 dt 时保留旧行为 |
| `User/Control/Src/Robot_Arm.cpp` | `Robot_Arm::update()` 中统一获取 dt 后注入到 pid_calc |
| `RC10_LIB/Motor/Src/Motor_DJI.cpp` | `M3508::update()` 和 `M2006::update()` 中注入 dt（需从外部传入或从基类获取） |

## 改动方案

### 步骤一 — PID_Position 增加重载（APP_PID.h + APP_PID.cpp）

**头文件声明**（`APP_PID.h`，`PID_Position` 类 public 区域）：

```cpp
// 外部注入 dt 版本（推荐），调用方负责传入 dt，省去 getSeconds 开销
float pid_calc(float target, float feedback, float dt);

// 原有自动获取 dt 版本（保留，向后兼容）
float pid_calc(float target, float feedback);
```

**实现**（`APP_PID.cpp`）：

```cpp
float PID_Position::pid_calc(float target, float feedback, float dt)
{
    // 直接使用注入的 dt，不调用 getSeconds
    dt_ = dt;

    if (isFirst_)
    {
        isFirst_ = false;
        dt_ = dt_error_;
        error_last_ = target - feedback;
        feedback_last_ = feedback;
        is_in_dead_zone_ = false;
    }

    if (dt_ <= 0.0f || dt_ > 0.1f)
        dt_ = dt_error_;

    // --- 以下与原 pid_calc 完全相同 ---
    error_ = target - feedback;
    // ... P/I/D 计算、死区、环形、限幅 ...
    return output_;
}
```

### 步骤二 — PID_Incremental 增加重载（同上）

**头文件声明**（`APP_PID.h`，`PID_Incremental` 类 public 区域）：

```cpp
float pid_calc(float target, float feedback, float dt);
```

**实现**（`APP_PID.cpp`）：

```cpp
float PID_Incremental::pid_calc(float target, float feedback, float dt)
{
    dt_ = dt;

    if (dt_ <= 0.0f)
        dt_ = 0.001f;

    // --- 以下与原 pid_calc 完全相同 ---
    // TD、死区、P/I/D 增量、限幅 ...
    return output_;
}
```

> 原有两个 `pid_calc(target, feedback)` 函数保持不变，内部改为调用 `pid_calc(target, feedback, self_computed_dt)` 即可复用逻辑。

### 步骤三 — Robot_Arm::update() 注入 dt（Robot_Arm.cpp）

`Robot_Arm::update()` 中已有 `dt_` 计算。在该函数末尾注入 dt 到各电机的 PID：

```cpp
void Robot_Arm::update()
{
    // ... 现有逻辑：读取 now_time_s_，计算 dt_，读取关节状态 ...

    // 旋转电机 PID（注入 dt）
    if (motor_rotate_ != nullptr) {
        // ... 现有 strategy/ramp 逻辑 ...
        // 原: motor_rotate_->setTargetTotalAngle(xxx);
        // 需改为: motor_rotate_->update_with_dt(dt_); 或传参方式
    }
    // ... stretch/launch/pitch 同样处理 ...
}
```

**注意**：`M3508::update()` 和 `M2006::update()` 内部调用 `speed_pid_.pid_calc()` 和 `angle_pid_.pid_calc()`。要让 dt 注入生效，有两种方式：

**方式 A（改动小）**：给 `M3508::update(float dt)` 和 `M2006::update(float dt)` 增加重载。

**方式 B（更彻底）**：`Motor_Base` 基类新增 `set_dt(float dt)` 接口，电机的 `update()` 内部从成员变量读取 dt。调用方周期性地调用 `motor->set_dt(dt)`。

### 步骤四 — Motor_DJI.cpp 适配

`M3508::update()` 内部：

```cpp
// 改前
target_current_ = speed_pid_.pid_calc(target_rpm_, this->rpm_);

// 改后（注入 dt 版本）
target_current_ = speed_pid_.pid_calc(target_rpm_, this->rpm_, injected_dt_);
```

## 兼容性保证

- 不传 dt 参数时：`pid_calc(target, feedback)` 行为与旧版完全一致，自动获取时间戳
- 传 dt 参数时：跳过 getSeconds，使用注入值
- 其他调用方（如 VESC、舵向电机 PID）不受影响，渐进式迁移

## 验证方法

1. 编译通过
2. 手操模式：机械臂各关节运动正常，PID 响应无明显变化
3. 自动模式：KFS 流程正常
4. 可选：用 debug 打印对比改前后 `dt_` 值（应完全一致）

## 后续扩展

- 底盘 `OmniChassis_Setup::Path_correction()` 中的 `pid_pos_x/pid_pos_y` 也可注入 dt
- `chassis` 四舵轮电机 PID 同理
- 武器系统 `WeaponSage` 的电机 PID 同理
