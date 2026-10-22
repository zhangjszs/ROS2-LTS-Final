# PLAN.md — 路线、当前阶段与可执行队列

> 维护者：Planning Agent（时间流接力）｜最后更新：2026-10-22T11:00+08:00（`plan-20261022`：S2b 首批 3 项 ready-for-agent 保持就绪，Nightly 连续 10 天全绿入账，等候 Execution 会话开工）
> 事实基线：`main` = `origin/main` = `b30af56`（0/0 同步）。本轮入账 Nightly 10-21 run `39676356203` 与 `b30af56` CI run `39627061249`，状态全绿。环境事实见 `.agent/ENV.md`（D-017 口径）。
> 文档体量约定：本文件只写**当前**状态与结论；历史以 `git log -- .agent/PLAN.md` 与各 Issue 评论为准。

## 〇、本轮结论：正式启动 Milestone S2b「八字绕环仿真闭环与规划加固」

在 S2a「直线加速事件闭环」与首批 P3 积压全面达成并验收出清后，依据用户交互掌舵指令（裁 **D-022**），项目由维持模式切换为新功能推进态，正式开启 **Milestone S2b「八字绕环仿真闭环与规划加固」**。

首批通过 4.2 ready 门禁的 3 项任务已在远端 GitHub Issues 正式立项入轨：
1. **#88（P1）**：`TrackGenerator` 补齐八字绕环规范中心线（进场 + 右双圈 + S 弯过渡切线 + 左双圈 + 出场停稳）；
2. **#87（P2）**：`skidpad_planner` 节点对齐契约 QoS（kQosPath / kQosState）与默认话题 `/planning/pathlimits`，消除重复圆拟合计算；
3. **#89（P2）**：`skidpad_planner` 核心边界参数清洗与退化守卫（非法值回退默认并构造期 WARN，防除零与清空锥桶）。

## 一、当前方向

当前主线推进 **Milestone S2b 八字绕环仿真闭环与规划加固**：
1. **赛道几何真值完善**：将 `TrackGenerator::GenerateSkidpad()` 由当前的单圈右半圆扩充为 FSAC 规范的双圆全景轨迹，消除基线评测撞锥缺陷；
2. **规划契约与健壮性**：将 `skidpad_planner` 全面纳入 D-002 接口契约单一权威（`makeQoS`、标准话题），清洗未定义输入参数；
3. **低速闭环与门禁建立**：基于现有纯 INS 站心定位基线（D-015）与 Pure Pursuit 基线控制器，打通仿真中的八字多节点闭环与可证伪门禁。

## 二、今日目标（2026-10-22）

- [x] 核查入账 Nightly 10-21 运行结果（run `39676356203`，16m17s，全绿，连续 10 天全绿达成）
- [x] 核对主干 HEAD `b30af56` 状态与 push CI run `39627061249`（15m23s，全绿）
- [x] 复核 GitHub Issues 与 4.2 ready 门禁，确认 S2b 首批 3 项任务（#88、#87、#89）无漂移、可立即开工
- [x] 更新 `TASKS.md` 与 `PLAN.md` 镜像至 2026-10-22 事实基线
- [x] 提交推送 `.agent/` 变更，按 D-012 提醒用户发起 Execution 会话开工 S2b 队列
## 三、当前 Milestone：Milestone S2b「八字绕环仿真闭环与规划加固」（进度：0 / 3 ready 任务开工）

### 目标与验收
- **目标**：在可复现仿真（纯 INS 基线）中完成八字绕环闭环——赛道生成器产出符合 FSAC 规则的完整八字轨迹（右双圈 + 切线换圆 + 左双圈 + 出场）；`skidpad_planner` 接入契约 QoS 与参数清洗守卫；离线 benchmark 与多节点 ROS 仿真均能稳定完赛。
- **完成判据**：
  1. **#88 几何真值**：`TrackGenerator::GenerateSkidpad()` 生成完整八字参考中心线（含曲率极值与各段连续性单测断言）；导出 `skidpad_track.csv`；
  2. **#87 契约对齐**：`skidpad_planner` 经 `makeQoS` 接入 `kQosPath` / `kQosState`，默认输出话题对齐 `/planning/pathlimits`；
  3. **#89 边界鲁棒**：参数非法值（NaN/负值/除零）构造期清洗回退默认并 WARN，核心计算防除零兜底，5+ 非法组合用例通过；
  4. **闭环门禁（后续任务）**：建立类似 `accel-finish` 的八字绕环闭环门禁或基线更新，证明端到端数据流动与可控性。
- **完成信号（Success signal）**：S2b 核心任务验收合入 + `core_standalone_check.sh` 干净 + 完整 drive_gates 全 PASS + 远端 CI 全绿。
- **明确非目标（Non-goals）**：
  - 恪守 D-015：不引入 ESKF / LocationMapper / GTSAM 因子图，坚持纯 INS 站心基线；
  - 恪守 D-003：不修改 MPC 控制律与判据，不私动比较基线；
  - 恪守 D-004：不宣称实车能力，不替代实车/台架关闭条件；
  - 恪守 D-002：不更改公共契约话题类型与字段定义。

## 四、执行队列（给 Executor 的建议顺序，3 项 ready）

> **队列状态**：3 项（处于 2–5 规范区间内，全部通过 4.2 ready 门禁）。
> **文件冲突规避与依赖分析**：
> - **#88（P1）**：独占 `src/simulation/track_benchmark/`，无文件冲突，建议最先开工并合入；
> - **#87（P2）与 #89（P2）**：同处 `src/planning/skidpad_planner/`。#87 触及适配层与 launch，#89 触及 core 算法与单测。**两单建议严格串行**（先 #87 合入 main、pull rebase 后再开工 #89，或在同一分支连续提交）。
> - 推荐执行顺序：**#88 → #87 → #89**。

1. **#88** `[S2b] TrackGenerator 补齐八字绕环完整中心线（右双圈+过渡切入+左双圈+出场）`（P1）
   - 落点：`src/simulation/track_benchmark/src/track_generator.cpp`、`test/test_track_generator.cpp`、`src/simulation/vehicle_simulator/tracks/skidpad_track.csv`
2. **#87** `[S2b] skidpad_planner 节点对齐契约 QoS 与接口话题规范（kQosPath / kQosState / 默认话题对齐）`（P2）
   - 落点：`src/planning/skidpad_planner/src/skidpad_planner_node.cpp`、`config/skidpad_planner.yaml`、`huat_launch`
3. **#89** `[S2b] skidpad_planner 核心边界参数清洗与退化守卫（非法值回退默认与 WARN）`（P2）
   - 落点：`src/planning/skidpad_planner/include/centerline_core.hpp`、`src/planning/skidpad_planner/src/icp_apf_planner.cpp`、`test/`

## 五、已完成（阶段性摘要）

- **S0 可重复启动**：已完成（#13/#14 已关闭）。
- **S2a 直线加速事件闭环**：✅ 全部达成（#78、#79、#80、#81、#82 五单关闭，四层判据收敛）。
- **首批 P3 积压清理**：✅ 全部达成（#83、#84、#85、#86 四单验收关闭，CI 全绿）。
- **ADR 下沉线**：全部完成（#59/#60/#61/#62 均关闭）。

## 六、CI / Nightly 状态（本轮复核）

- **主干 origin/main 全绿**：
  - 最近提交全部 CI success（#83 `cb09a6b` · #84 `c7d7e8c` · #85 `4d303d1` · #86 `6b6697f` · agent 提交 `39600468883` / `39614818759` / `39616221072` / `39624708526`，以及 `b30af56` CI `39627061249` 15m23s 全绿）。
- **Nightly 连续 10 天 success**（10-13 起，最新 10-21 `39676356203` 16m17s）。
- 本地与远端 0/0 同步，无未推送提交，无 dangling 临时分支。
## 七、已知阻塞与依赖（D-013 口径逐项复验）

7 项外部阻塞（#15/#16/#17/#18/#19/#24/#37）维持 blocked，无外部数据/运行时到位；3 项 needs-info（#25/#26/#27）待真实数据。
S2b 的当前 3 项任务不依赖上述任何外部输入，纯仿真可推进。

## 八、下一阶段

- **S2b 闭环建立**：完成 #88/#87/#89 基础层后，立项八字仿真闭环冒烟门禁（如 `skidpad_finish_smoke.sh`）与离线基线回归。
- **S3 高速循迹**：待 S2b 稳定后评估高速工况。

## 九、已明确放弃 / 不做

- 调参腿已收口，不冻结参数（D-003）。
- 不引入复杂定位后端（D-015）：ESKF / LocationMapper / GTSAM 因子图在真实 bag（#18）到位前不进 ready。
- 底盘自建不等于提前关闭 #15（D-016）。
- 不把 `FSAC/2024` 当对照基线（D-013）。

## 十、Decision Gate 状态与给 Executor 的指令

- **Decision Gate**：裁定 **D-022**（启动 S2b）；当前**无 pending 决策**。
- **给 Executor 的指令**：
  > 请按 **#88 → #87 → #89** 顺序领取开工；#87 与 #89 同处 `skidpad_planner`，请严格串行执行并留意契约 QoS 规范。
- **D-012 循环提醒**：Ready 队列（#88、#87、#89）已就绪待领，可直接发起 Execution 会话开工。
