# historical

这个目录存放冻结的历史测试资产。

约定：

- 不纳入默认 `smoke` / `active` / `extended` 回归
- 只在需要历史对照、局部追溯或兼容性排查时按 suite 单独执行
- 默认不做风格升级，只做最小兼容维护

重新激活某个 historical suite 时，先在 `jia_docs/tests/tests.yaml` 中将其从 `historical` 提升为 `active`，再评估是否迁入 `host_cpp` 或 `python_semantics`
