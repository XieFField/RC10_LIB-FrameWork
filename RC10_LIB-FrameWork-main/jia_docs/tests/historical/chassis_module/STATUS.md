保留原因：chassis module 历史编译回归。
运行方式：`powershell -ExecutionPolicy Bypass -File jia_docs/tests/historical/chassis_module/run_test.ps1`
当前可信度：适合作为旧实现对照，不作为默认主回归。
覆盖关系：部分行为已被 `host_cpp/chassis_semantics` 吸收，但非完全等价。
重新激活条件：需要追溯旧 chassis module 编译或接口退化问题时。
