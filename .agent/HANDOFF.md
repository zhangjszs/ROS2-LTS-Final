# HANDOFF

## 本轮概要

exec-20261026 **会话2**（接会话1）：清空今日队列的**全部三单**——#96 合入 main、#69 修就绪探测单点失效、#95 修规划器几何缺陷（**从「部分完成」推进到「完成」**，端到端闭环首次全绿）。**收尾状态：三单全部 squash merge 进 `main`、两条临时分支已删、三个 Issue 均已转 `in-review` 并贴执行报告。**

| 事项 | 结论 |
| --- | --- |
| #96 合入 skidpad_closed_loop_smoke.sh | **完成**：PR #97 已 squash merge 进 `main`（`7733cbb`），未挂载，CI 绿（attempt 2） |
| #69 DDS 门禁区分环境性/真失败 | **完成**：PR #98 已 squash merge 进 `main`（`cc616e4`），CI run `40355980015` attempt 3 = success，临时分支已删，Issue #69 转 `in-review` + 执行报告已贴 |
| #95 skidpad_planner 路径生成三处几何缺陷 | **完成**：PR #99 已 squash merge 进 `main`（`b4d2d1a`），CI run `40368098627` = success，临时分支已删，Issue #95 转 `in-review` + 更新后的执行报告已贴；仅判据 1 待 Planner 裁定 |

## 已完成

- **#96**：`scripts/skidpad_closed_loop_smoke.sh` 经 PR #97 合入 `main`（`7733cbb`），**保持未挂载**（`drive_gates.sh`/`ci.yml` grep 无命中）。改动仅 `scripts/`（422 行）。负样本臂 rc=1 且原因指向「无 pathlimits 产出」。CI run `40336692425` attempt 2 = success。→ Issue #96 已 `in-review` + 执行报告已贴。
- **#69**：
  - 新增 `scripts/lib/node_readiness.sh`（共用件）：**进程存活为主判据**（`ps -o stat=` 排僵尸态 —— `kill -0` 对僵尸仍返回成功）、`ros2 node list` 降为辅助确认、真失败立即判红并点名、环境性失败提示后由数据面定成败、节点视图查询可注入（`PROBE_NODE_LIST_CMD`）。
  - `skidpad_finish_smoke.sh` / `skidpad_closed_loop_smoke.sh`：`start()` 登记「节点名 + PID」；就绪探测换两段式；数据面失败首行补「疑环境性失败（DDS 发现未收敛）」。
  - 新增 `scripts/node_readiness_selftest.sh`（确定性构造三种情形），挂进 `lint_shell.sh` 的 `[3/4]` 段（**drive_gates 门清单不变**）。
  - 验证：正样本 A/B 对照 —— `main`（无本改动）默认预算下同样红（index 346/349），本分支 `BUDGET_SEC=420` 绿（`finish_event_count: 1`）；负样本 `REQUIRED_LAPS=3` 仍 rc=1。
- **#95**（Core 层，`src/planning/skidpad_planner/`）：
  - **会话1 的成果（`bc6b0b5`）**：跨圆不再平均（同心→仍取平均；不同心→相位感知单选）；`Circle2D::rms_residual` + `max_fit_rms` 质量护栏；`OrderAlongCorridorCore` / `AppendMutualMidpointsCore` / `AnchorPathAtEgoCore` / `ExtendWithPrevPathCore`；`ComputeCenterlineCore` 排序语义逐位不变（#24 差分夹具的事实来源）。
  - **会话2 的成果（`5da9fbf` + `f99951b`）——切点走廊太短的根因修复**：新增 `include/corridor_model_core.hpp`（纯 std）：
    - `FitConcentricWallsCore`：给定瓣心，用半径直方图找**两条同心壁**；壁半径残差 RMS、弧角跨度、「瓣心在锥桶云外侧」、「两壁之间留空」四道判据把关；中线半径 = (r_in + r_out)/2。
    - `FitCorridorModelCore`：瓣心候选按 (半径, 方位角) 网格多起点搜索 + 细化；评分 = 支持度 − 3×相位误差 − 1×连续性误差；「半径/半宽比 ≤ 10」排除直走廊伪弧。
    - `GenerateCorridorArcCore`：弧的切向必须与自车航向同向（旧实现比较弧两端 x，弧顶处退化 → 方向选反 → x 回跳 11 次）；转向角上限 90°（判据 3：路径 x 不得回退）。
    - `ComputeCorridorPathCore` 迁入本头（生产入口，瓣模型法 → 圆法 → 中点法）。
  - 新增 `test/test_corridor_model.cpp`（12 例纯 std），按 #40 登记进 `tests/core_standalone/CMakeLists.txt`（+ `ament_add_gtest`）。

## 未完成 / 进行中（下一棒最优先看这里）

- **今日队列内无未完成项**（#96 / #69 / #95 全部合入 main + 转 in-review + 贴报告）。
- **待复核**：`main` 的 push CI（`b4d2d1a` 之后又合入了 `cc616e4`）需确认最终一次 push run 是绿（CI 的并发组会取消被新 push 取代的旧 run，需看**最后一次** run 的结论，别把 cancelled 当失败）。
- **待 Planner 处置（非 Executor 可决）**：#95 判据 1 的表述裁定；auto-discovered **#100**（切点换瓣）；#92 的挂载收口。
- **#95 判据 1 需 Planner 裁定**：中线到所在瓣心恒为 9.125 m，字面条件「路径进入两圆心 ±1.5 m 邻域」与 #90 已验收的真值互斥。已按最近意实现并单测锁定，建议改写为「非单圆 + 落在某个真实瓣的中线上（到瓣心 9.125 ± 0.3 m）」。
- **换瓣能力（新增 auto-discovered）**：真值中心线是「右瓣 2 圈 + 左瓣 2 圈」，切点处两瓣几何镜像对称 → 规划器靠连续性保持单瓣，不会自主换瓣。端到端脚本目标是 2π（单圈）已达成，但整条赛道（4π + 换瓣）仍跑不完 → 需赛道标记或产品决策，建议单独立项。

## 验证情况

| 验证 | 结果 |
| --- | --- |
| `colcon test --packages-select skidpad_planner` | **639 tests, 0 errors, 0 failures** |
| `colcon test --packages-select track_benchmark` | **71 tests, 0 errors, 0 failures** |
| `bash scripts/core_standalone_check.sh` | **35/35 PASS**（ASan+UBSan clean，含新增 `test_corridor_model`） |
| `bash scripts/lint_cpp.sh` | rc=0 · `✔ cppcheck passed with 0 issues!`（注：本地 clang-format 是 21.1.8，CI 是 apt 旧版，**尾注释对齐会打架** → 新代码注释一律独占行） |
| `bash scripts/lint_shell.sh` | rc=0 · 19 脚本 clean + 就绪探测自测全过 |
| `bash scripts/node_readiness_selftest.sh` | 三情形断言全过（就绪 / 环境性不判红 / 真失败点名） |
| `bash scripts/skidpad_closed_loop_smoke.sh build/skidpad_closed_95f` | **rc=0 PASS** · `cum_right_angle_rad 6.2864`、`max_lateral_dev_m 0.433`、`pathlimits_finite True`；PP 日志 `Approaching path end`/`refusing goal`/`Crosstrack` 计数**全 0** |
| `ENABLE_PLANNER=0 skidpad_closed_loop_smoke.sh` | rc=1（负样本可证伪守住） |
| `bash scripts/skidpad_finish_smoke.sh`（正样本，默认 180 s 预算） | rc=1 —— **既有环境问题**：A/B 对照 `main` 上同样红；`BUDGET_SEC=420` → rc=0 绿（真值中心线喂入，不依赖规划器） |
| `bash scripts/skidpad_finish_smoke.sh`（负样本） | rc=1（按设计判红） |
| PR #97（#96）CI | **success**（run `40336692425` attempt 2） |
| PR #98（#69）CI | **success**（run `40355980015` attempt 3） |
| PR #99（#95）CI | **success**（run `40368098627`；`40367022848` 因 clang-format 判红 → `f99951b` 修复，`40367937681` 被新 push 取消） |

**未跑**：`bash scripts/drive_gates.sh` 全量门禁（本轮改动的覆盖由两个 PR 的 CI 承担：build/test/core-standalone/lint + 各 runtime 门禁）。

## 风险与注意事项

1. **#95 仍未挂 `skidpad_closed_loop_smoke.sh` 进 `drive_gates.sh`/`ci.yml`**（#95 范围约束）。现在正样本在本机已绿，可作为 #92 的收口前置，但挂载动作属 #92，请 Planner 排期。
2. **本机默认 180 s 预算下 `skidpad_finish_smoke.sh` 正样本红是既有环境问题**（非代码）：A/B 对照 `main` 上同样红；`BUDGET_SEC=420` 绿。本机复现绿请加大预算。
3. **CI 的 ros-apt-source 步骤偶发 404**（GitHub API/下载侧抖动，attempt 1 判红、re-run 绿）。遇到先 re-run 再判断是不是自己的锅。
4. **clang-format 版本不一致**：本地 21.1.8 vs CI apt 旧版，**尾注释对齐**规则不同 → 新增/改动的 C++ 代码里注释一律**独占行**，不要写尾随注释（本轮已因此红过一次）。
5. **`ComputeCenterlineCore` 是差分基线**：任何改动它都会破坏 `test_icp_apf_planner.cpp` 的 `DifferentialEquivalenceWithLegacyPCL`（1000 组逐点比对）。要改排序语义，先更新 `docs/DEFECT_DIFFERENTIAL.md`（#24 级变更）。
6. **不要用「壁弧角 ≥ 40°」当直走廊判据**：实测会让模型在真实位姿闪断 → 退回中点法产生跨瓣路径 → 端到端横偏 7.88 m。直走廊靠「半径/半宽比 ≤ 10」排除（伪弧 15.4 vs 真走廊 5.6~7.6）。

## 给下一棒的第一步建议

1. **先确认 `main` 的最后一次 push CI 是绿**（`gh run list --branch main --limit 3`；注意 cancelled ≠ failure）。
2. **不要再动 #95 的算法**（已全绿并合入 main，改动越多风险越大）；判据 1 的裁定交给 Planner。
3. 若要继续推进 S2b：#92 的**挂载收口**（闭环脚本正样本已绿，可挂进 `drive_gates.sh`/`ci.yml`）；切点换瓣见 auto-discovered **#100**（需赛道标记或产品决策，单独立项）。

## 今日日报

- 已写入 `.agent/DIARY/2026-10-26.md`（会话1 + 会话2 追加）。

## 给 Planner 的信号

- **需要 Planner 介入（两项）**：
  1. **#95 判据 1 的表述**：字面条件（路径进入两**圆心** ±1.5 m 邻域）与 #90 已验收的真值中心线互斥（真值 349 点到两圆心的最小距离恒为 9.125 m，邻域内 0 点）；建议改为"非单圆 + 落在某个真实瓣的中线上"，或把"全量 108 锥桶下覆盖两瓣"单列为全局建图里程碑。
  2. **#92 的挂载收口**：`skidpad_closed_loop_smoke.sh` 现在有绿的正样本臂了（rc=0 / 6.2864 rad），可以进入「挂进 drive_gates.sh / ci.yml」的收口动作（**本轮未挂**，遵守 #95 范围约束）。
- **#69、#96 无需介入**：判据已逐条 `[x]` 或有实测证据，CI 绿、已合入 main，可验收关闭。
- **#95**：除判据 1 的裁定外，其余 8 条判据均达成且有实测证据，可验收关闭。
- **新增 auto-discovered 1 条**：切点处两瓣镜像对称 → 规划器无法自主换瓣（真值是右瓣 2 圈 + 左瓣 2 圈）。已报 Issue，等 Planner 定优先级。
