# jia_docs 说明

该目录用于 AI 协作过程资料归档，不参与产品代码编译。

## 目录约定

- `handoff/`：当前迭代交接文档（按年月分层）
- `history/`：已归档的稳定交接记录（按年月分层）
- `tests/`：AI 侧测试样例与脚本
- `artifacts/`：临时产物、参考材料、过程附件

## 最新交接入口

- RC10 最新交接文档：
  - [handoff/2026-05/ai_handoff_2026-05-20_2330_rc10_swerve_pipeline_sync.md](handoff/2026-05/ai_handoff_2026-05-20_2330_rc10_swerve_pipeline_sync.md)
- 交接索引：
  - [handoff/INDEX.md](handoff/INDEX.md)

## 本轮主题

- 当前文档主线已从“上下文同步”推进到：
  - 统一输入语义；
  - 显式舵轮规划流水；
  - 宿主 TDD 扩展；
  - 面向实车联调的低风险递进验证准备。

## 命名规则

- 新交接文档统一使用：`ai_handoff_YYYY-MM-DD_HHMM_<topic>.md`

## 维护约定

- 每次迭代进行中：文档先落在 `handoff/`
- 迭代稳定后：从 `handoff/` 迁移到 `history/` 并更新索引
- 默认不删除历史记录；若要瘦身，单独做按日期清理
