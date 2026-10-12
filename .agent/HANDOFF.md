# HANDOFF

## 本轮概要（`test-perf-20261013`：测试性能基线轮，用户驱动）

- 应用户要求完成"自主探索 → 建基准 → 三阶段分析 → issue 化"全流程：产出 `docs/TEST_PERF.md`（已提交 `c0652e1`）+ issue **#63–#76**（14 项，均 `ready-for-human`，待用户跟进）。
- **未动任何代码/CI/门禁脚本**（全部方案只写了 diff 与验收判据，实施待用户确认）；**未 push**（用户睡前未决定）。注意：纯 docs 提交若 push，在 #63 落地前会触发一次全量 CI（≈14min）——可借机作为旧配置的最后基线。
- 重要事实：本机 10-12 DDS 发现退化，运行时门禁本地假红（节点正常退出、CI 同 commit 全绿）——PLAN.md §九"统一发现收敛等待策略不立项"的触发条件已满足，已立 #69。

## 已完成

- `docs/TEST_PERF.md`（375 行）：本地全量实测 3215s（5 个 DDS 门禁环境性失败，证据链入档 §2.3）；健康基线双源（本地 10-08 全量 656s 全绿 + Nightly 10-11 逐门禁 646s）；506 用例 0 失败、计算总耗时 0.36s；门禁级 Top10；阶段一审计（🔴 无、🟡 4 项）；阶段二 7 项加速方案（附 diff/预估/风险）；阶段三 7 项建议。
- issue #63–#72：CI 路径过滤+并发、cpp20 参数统一、六臂并行、CI job 拆分、affected 增量、ccache 键、DDS 假红识别、时长监控、覆盖率基线、setup-ros 超时。
- issue #73–#76（补立）：#73 评分核双轨去重、#74 本地快速反馈文档、#75 测试编写规范、#76 colcon-only 下沉伞形。

## 未完成 / 进行中（下一棒最优先看这里）

- push 决策未做（`c0652e1` 在本地 main）。
- #63–#76 全部待用户跟进；实施顺序建议在 TEST_PERF.md 第七节。TEST_PERF.md 遵循幂等规则：后续只追加「优化记录」，不重建基准。
- `build/issue_bodies/*.md` 为 14 个 issue 的正文底稿（build/ 已 gitignore，可随时再生成）。

## 验证情况

- 跑了：本地全量 `drive_gates.sh` 一次（build/test-run/test-enforce/lint/cpp20/core-standalone/benchmark/tuning-precheck 全 PASS；headless-smoke/qos-contract/closed-loop/closed-loop-fault/fault-injection/mpc-reject 6 个 DDS 门禁 FAIL，已逐一定性为环境性失败）；`gh run list/view` 近 12 次 CI 运行拆解；`gh issue create` ×14。
- 未跑：QUICK=1（无需，零代码变更无回归对象）。

## 风险与注意事项

- `M AGENTS.md` 外部脏改动仍在，全程未碰、未提交（本体系纪律，只读记录）。
- 本机 ROS=lyrical（CI Jazzy）；多节点脚本前必须 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`。
- clang-format 本机 21 vs CI 18 skew 仍在（PLAN.md §九已记，未再现新分歧）。
- 10-12 DDS 退化期的门禁耗时劣化 4-8 倍（closed-loop-fault 1337s），**不可作性能基线**——TEST_PERF.md §2.3 已注明口径，后续引用基线时用 10-08/Nightly 数据。

## 给下一棒的第一步建议

1. 用户跟进 #63–#76（顺序见 TEST_PERF.md 第七节）；
2. 若用户决定 push：预期一次全量 CI（旧配置），作为 #63 前的最后旧配置基线；
3. Executor 轮转不受本轮影响：#62 仍为 ready 队列唯一项（PLAN.md §十不变）。

---

## 上轮：exec-20261012T1143Z（零变更轮）

- 无现场可恢复（上轮正常收尾，LOCK 已删）、ready 队列空（`gh issue list --label ready-for-agent` 实测为空），按停止条件直接收尾。未做任何代码/文档变更。
- 15 项 open：in-review 3（#59/#60/#61，执行报告齐备，等 Planner 验收）· 仓库外阻塞 7（#15–#19/#23/#24）· needs-info 3（#25/#26/#27）· needs-triage+blocked 1（#37）· 伞形跟踪 1（#20）。

## 已完成

- 无新 Issue。本轮为零变更轮（上轮 #61 落地后 ADR 下沉候选即清空，属预期内）。

## 未完成 / 进行中（下一棒最优先看这里）

- 无 in-progress、无未提交代码、无临时分支；工作区仅外部 `M AGENTS.md`（非本体系产生，未碰）。
- #59/#60/#61 为 `in-review`，执行报告齐备，无 Planner 的 reopen/差距评论——无需 Executor 动作。
- **ready 队列为空**。若下一棒仍为空，直接进入空转收尾流程（看一眼 CI/Nightly，不重跑门禁）。
- push CI `38243907339`（`8c7db61`，.agent-only 提交，本轮开工时 in_progress）：内容仅改 `.agent/` 文本，不可能影响门禁；下一棒顺带确认结论即可（若绿则无事，若红则按 PLAN §六先看是否为既有三种模式）。

## 验证情况

- 跑了：`gh issue list --label ready-for-agent`（空）· `gh issue list --state open`（15 项，分布同上）· #59/#60/#61 近期评论（均为执行报告本体，无 Planner 新评论）· `gh run list`（`4d795d9` success；`8c7db61` 有 run 在途；`7e5794d`/`39a40a6`/`cb54e23` 的 failure 均为 #60 遗留 clang-format 126-127 行问题，上轮已修复并经 `4d795d9` success 反证）。
- 未跑：全量 `drive_gates.sh` 及任何门禁（零变更轮，无验证对象；证据在 CI 侧，重复跑无新增信息）。

## 风险与注意事项

- `M AGENTS.md` 脏改动仍在（非本体系产生）：全程未碰、未提交。提交 `.agent/` 时只 `git add .agent/`，严禁 `-a`/`-A` 带入。
- **gh 评论反引号陷阱**：`--body` 双引号传反引号会被 shell 吞掉，一律 `--body-file`。
- 本机 clang-format 21.1.8 vs CI apt 18.x 的 skew：上轮全树双版本扫过，仅此一处分歧并已修复；若再现，用上轮办法（pip wheel 解包到 /tmp 的 18.1.8 二进制复现，不全局安装）定位。

## 给下一棒的第一步建议

1. `git fetch --all --prune` → `gh issue list --label ready-for-agent --state open`：有 → 按 2.3 顺序领取；无 → 看一眼主干 CI（尤其 `38243907339` 结论）与 Nightly 后直接收尾（**不要重跑门禁**）。
2. #59/#60/#61 若已被 Planner 验收关闭属预期，不要重开；若 Planner 留了差距评论（reopen），优先恢复对应 Issue。

## 给 Planner 的信号

- **需要 Planner 介入：ready 队列已空**（#60/#61 后 ADR 下沉候选清空，上轮已预告）。15 项 open 均等仓库外输入或待验收/待决策。
- 建议：① 验收 #59/#60/#61（执行报告齐备；#61 报告内含测试预期修正依据与工具链 skew 说明）；② 下一轮 PLAN 按 §八转向（换维度扫描，或显式记录穷举并问用户外部输入时间表），不要再造同类下沉 Issue。
- 本轮无 auto-discovered 新增（配额 0/1；零变更、无新证据，不灌水）。
