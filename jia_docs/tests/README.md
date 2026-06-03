# tests 说明

这个目录用于存放测试说明、主机侧测试套件、Python 语义回归、历史保留区，以及统一入口脚本和结果产物。主入口已经迁移为 `jia_docs/tests/run_tests.ps1`，旧脚本仍保留为兼容包装入口，不再作为首选路径。

## 现在优先看什么

继续开发时，优先执行：

1. `jia_docs/tests/run_tests.ps1`
2. `jia_docs/tests/host_cpp/chassis_semantics/run_test.ps1`

如果你要继续跟进 VESC 刹车相关历史对照，再看：

- `jia_docs/tests/historical/vesc_brake/run_test.ps1`

如果你要继续看 Python 语义回归，再看：

- `jia_docs/tests/python_semantics/swerve_port/lock_now_homing_gate_regression.py`

## 分层说明

### `host_cpp/`

当前主机侧 C++ 测试集合，默认由统一入口调度。

当前 active 套件：

- `host_cpp/chassis_semantics/`
- `host_cpp/vesc_brake/`
- `host_cpp/swerve_core/`

其中 `host_cpp/chassis_semantics/` 仍保留这些兼容包装入口：

- `run_test.ps1`
- `run_pid_reconnect_test.ps1`
- `run_app_utils_backend_test.ps1`
- `run_app_utils_math_test.ps1`

### `python_semantics/`

由原 `ai2_tests/` 迁移而来的 Python 语义回归集合。

当前重点子集：

- `python_semantics/swerve_port/`
- `python_semantics/lock_now_rot_z/`
- `python_semantics/homing_state/`
- `python_semantics/time_stamp_us64/`

### `historical/`

历史保留区，不默认运行，主要用于行为对照和旧回归定位。

可关注的旧套件包括：

- `historical/swerve_core/`
- `historical/vesc_brake/`
- `historical/chassis_module/`
- `historical/time_services/`
- `historical/tri_omni_kinematics/`
- `historical/serial1_protocol_host/`

### `shared/`

共享的测试代码、公共 fixture、可复用脚本片段放在这里。

### `artifacts/`

过程参考物、结果摘要和可追溯输出放在这里，不放业务源代码。

## 维护约定

- 新增测试时，优先在 `jia_docs/catalog/tests.yaml` 里登记高价值入口。
- 统一入口写在 `jia_docs/tests/run_tests.ps1`，旧路径只保留兼容用途。
- 需要长期对照的内容放入 `historical/`，不要让历史保留区影响默认执行路径。
