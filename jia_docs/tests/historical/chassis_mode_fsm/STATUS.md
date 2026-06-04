保留原因：底盘 mode 状态机的历史静态回归快照。
运行方式：`powershell -ExecutionPolicy Bypass -File jia_docs/tests/historical/chassis_mode_fsm/run_test.ps1`
当前可信度：仅作历史对照，不纳入默认主回归。
覆盖关系：未明确被新 active suite 完整替代。
重新激活条件：出现 mode 状态机回归问题，需要恢复持续维护时。
