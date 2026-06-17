保留原因：time services 历史编译回归。
运行方式：`powershell -ExecutionPolicy Bypass -File jia_docs/tests/historical/time_services/run_test.ps1`
当前可信度：历史对照用途。
覆盖关系：部分时间语义由 `python_semantics/time_stamp_us64` 覆盖，但非同层验证。
重新激活条件：time services 代码再改动或需要追溯旧行为时。
