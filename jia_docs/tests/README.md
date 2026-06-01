# tests 说明

该目录用于存放 AI 侧回归脚本、宿主侧 TDD 套件、历史回归保留集与最小复现实例，不直接参与产品固件编译。

## 当前推荐先跑哪两个入口

继续开发时，优先执行：

1. `jia_docs/tests/tdd/chassis_semantics/run_test.ps1`
2. `jia_docs/tests/tdd/chassis_semantics/run_pid_reconnect_test.ps1`

原因：

- 它们是当前主维护的宿主侧回归入口；
- 最新主线 handoff 直接引用了这两个脚本；
- 它们最适合先确认当前 `chassis` 与 `PID reconnect` 语义还在不在预期边界内。

如果你是从 AI 语义 harness 角度切入，再补看：

- `jia_docs/tests/ai2_tests/swerve_port/lock_now_homing_gate_regression.py`

## 分层说明

### `tdd/`

当前主维护的宿主侧编译 / TDD 套件。

主入口：

- `tdd/chassis_semantics/run_test.ps1`
- `tdd/chassis_semantics/run_pid_reconnect_test.ps1`

补充入口：

- `tdd/chassis_semantics/run_app_utils_backend_test.ps1`
- `tdd/chassis_semantics/run_app_utils_math_test.ps1`

### `ai2_tests/`

当前主线 AI 语义回归与外部行为 harness 集合，适合做协议、状态机、行为边界验证。

当前较值得优先关注的子集：

- `ai2_tests/swerve_port/`
  - 聚合验证 `LockNowRotZ`、homing、flip、`DriveGate`、`StopSteerGuard`
- `ai2_tests/lock_now_rot_z/`
  - `LockNowRotZ` 缺陷 red/green 回归
- `ai2_tests/homing_state/`
  - `updateHomingState()` 外部状态机回归
- `ai2_tests/time_stamp_us64/`
  - 时间戳接口与换算语义回归

### `legacy_ai_tests/`

历史回归保留区，主要用于行为对照和局部回归。

可按用途粗分为：

- 核心编译/执行型：
  - `swerve_core`
  - `time_services`
  - `chassis_module`
  - `vesc_brake`
  - `tri_omni_kinematics`
- 静态断言或结构检查型：
  - `four_swerve_path`
  - `omni_setup_static`
  - `chassis_mode_fsm`
  - `four_steer_setup_static`

如果你需要一个偏历史主入口，先看：

- `legacy_ai_tests/swerve_core/run_test.ps1`

如果你要跟进当前 VESC 本地 PID 速度环相关行为，先看：

- `legacy_ai_tests/vesc_brake/run_test.ps1`

## 维护约定

- 新增测试时，优先补对应子目录的 `README.md` 或入口说明。
- 高价值入口尽量在 `jia_docs/catalog/tests.yaml` 中登记。
- 不要把 build 产物当成长期版本资产保留在这里，除非它本身就是调试所需证据。
