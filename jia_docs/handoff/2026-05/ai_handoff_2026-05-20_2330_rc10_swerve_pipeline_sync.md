# RC10 四舵轮工程 AI 交接（2026-05-20 23:30）

生成时间：2026-05-20 23:30（Asia/Shanghai）  
仓库路径：`D:\desktop\2026RC\Control\2026RC-Team1-R1Code\RC10_LIB-FrameWork`  
当前分支：`Jia6_temp`  
当前 HEAD：`c9787ca`  
工作区状态：

- 当前任务相关文件已提交到 `c9787ca`
- 工作区仍保留两个无关 IDE 脏文件：
  - `MDK-ARM/Frame_T.uvoptx`
  - `MDK-ARM/Frame_T.uvprojx`
- 上述两个文件不属于本轮改动产物，也不应作为当前任务异常处理

## 1. 当前目标

- 维持 RC10 四舵轮底盘主控制链路稳定。
- 把“遥控/外部输入 -> 底盘目标 -> 舵轮规划 -> 电机下发”收敛成更单一的语义链。
- 与舵轮调试面板工程保持统一显示契约，优先完成实车联调验证。

## 2. 本轮已完成核心改动

- 已建立统一输入语义层：
  - `CommandInputSource`
  - `NormalizedBodyCommand`
- 已建立显式舵轮规划数据对象：
  - `SwervePlannerInput`
  - `SwervePlannerOutput`
  - `ActuatorCommandFrame`
- `computeModuleCommands()` 已从巨石实现收敛为阶段调度器。
- 舵轮规划当前已显式覆盖 5 级主链路：
  1. `body twist -> ideal wheel vectors`
  2. `ideal vectors -> flip solution`
  3. `flip solution -> reachable steer plan`
  4. `reachable steer plan -> projected drive`
  5. `projected drive -> final gated drive`
- `applyModuleCommands()` 继续收口到“安全门控 + 执行下发”为主。
- `AlignToZero` 的目标重算已抽成独立 helper，减少执行层内部语义散点。

## 3. 已确认验证状态

- MCU 宿主 TDD：通过
  - `powershell -ExecutionPolicy Bypass -File jia_docs\tests\tdd\chassis_semantics\run_test.ps1`
- 面板 Python 回归：通过
  - `python -m unittest tests.test_swerve_serial_adapter tests.test_webviz_vehicle_animation tests.test_webviz_vehicle_animation_contract tests.test_webviz_app`
- 面板 Node/WebViz 回归：通过
  - `node --test tests/webviz_vehicle_animation_bindings_test.mjs tests/webviz_frontend_state_test.mjs`

## 4. 当前已稳定口径

- MCU 当前主改造已经落到“统一输入语义 + 显式 5 级舵轮规划流水”。
- 面板当前不是主改造仓，而是回归验证方。
- 两边共同契约当前默认保持现有 `SwerveTelemetryV2` 负载形状。
- 当前推荐排查顺序保持不变：
  1. 输入映射
  2. MCU 上报
  3. 解析映射
  4. 画布 / 主视图坐标

## 5. 当前未完全闭环点

- 实车方向一致性仍需首轮验证，尤其要确认：
  - 四轮转向不同步时的横滑是否明显降低
  - 纯自转与边转边移是否被新的 gate 误伤
- 零点 / 安装角 / 极性命名仍可继续清理，但当前骨架已稳定，不再建议继续散点翻符号。

## 6. 实车测试建议顺序

1. 回零：
   - 确认四轮稳定进入 `AlignToZero -> Ready`
   - 无单轮来回抖动或停在错误 OA
2. 低速直行 / 斜移：
   - 观察转向未完成时驱动是否更克制
   - 观察横滑是否比之前明显减小
3. 纯自转：
   - 确认 vector consistency gate 不误伤正常纯自转
4. 边转边移：
   - 确认混合工况下方向一致性优于旧实现
5. 大角度切向：
   - 重点验证你最关心的“不同转向行程”场景
   - 观察是否仍有某一轮先强拉底盘横蹭

## 7. 代码接手重点

- 输入标准化从 `resolvePlannerTargetData()` 开始收口到 `normalized_body_command_`。
- 舵轮规划的主观察点是：
  - `makeSwervePlannerInput()`
  - `planSwerveModules()`
  - `buildActuatorCommandFrame()`
  - `storePlannedActuatorFrame()`
- 如果后续要继续清理零点链路，优先沿：
  - raw
  - signed local
  - corrected local
  - OA
 这条分层继续收口，不要再回到混写公式。

## 8. 回退锚点

- 当前推荐回退锚点：`c9787ca`
- 上一关键结构保存点：
  - `13293ef`
  - `f08b076`
  - `85dba74`

## 9. 一句话交接结论

> RC10 当前已经完成主链路结构性收口，下一阶段不应继续盲目扩协议或重写显示层，而应优先完成实车低风险递进验证，并基于实测结果做定点修正。
