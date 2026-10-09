# PLAN.md — 路线、当前阶段与可执行队列

> 维护者：Planning Agent（时间流接力）｜最后更新：2026-10-09T14:55Z（`plan-20261009T1455Z`）
> 事实基线：main `eb120e7`（与 origin/main 一致；`git fetch --all --prune` 已同步；`git pull --rebase` 因未暂存的外部 `M AGENTS.md` 跳过——非本体系产生，任何一棒都不碰）；**无 LOCK**；环境事实见 `.agent/ENV.md`。

## 一、当前阶段

路线图来源：#20「ROS 2 车队开发路线：运行基线 → 可信仿真 → 低速闭环 → 性能优化」（S0–S3）。

- **S0 可重复启动：完成**（#13/#14 已关闭；CI/lint/headless/qos 门常绿）。
- **S1 可信接口与仿真 / S2 低速稳定闭环：软件侧基本完成，剩余卡仓库外输入**：
  - #15 共用执行器适配层：软件侧（codec/标定单一来源/停车帧工厂）已交付并审计；剩余 = VCU 协议资料 + 台架实车标定。
  - #16 任务/安全状态与最终仲裁：软件侧（状态机/信任门/新鲜度租约/故障注入矩阵）闭环；剩余 = 硬件急停链路 + 实车停车预算。
  - #17 可复现仿真/KPI/回归：③④已合入并进 CI 硬门；①②受实车标定与感知扰动数据阻塞。
  - #19 Pure Pursuit 低速基线：A 阶段基线 + B 骨架（诊断出口 / 调参-比较集分离）已在；B 完整公平对比依赖 #17 扰动场景。
  - #18 rosbag 数据集与感知质量评估：需真实 bag。
- **S3 性能优化**：#19 B 待 S2 条件满足。
- **展示（D-007-B）**：最小展示入口 #57 已关闭；CLAUDE.md 文档同步 #58 本轮验收关闭；完整展示（#26）与博客（#25）待 #17①② / #18 证据。
- **仓库外输入未到位期间的软件侧方向**：仅余 ADR 下沉候选可执行——本轮已将最可行者定级入队（#59），详见 §三/§四。

## 二、当前 Milestone 目标与完成判据

- 目标（S1+S2）：接口/编码/单位/坐标系一致；固定场景可复现且 KPI 能识别失败；选定赛制低速闭环可重复完成，停车/失效/重启行为通过。
- 完成判据：以各 Issue 验收标准为准；其中依赖实车/台架/硬件的条目**不得以仅仿真证据关闭**（见 D-004）。
- Issue 台账（本 `gh` 实测，共 12 open + 本轮新立 #59 后 13 open）：ready 1（#59）· 仓库外阻塞 7（#15–#19 / #23 / #24）· 展示待条件 3（#25/#26/#27 needs-info）· 待实车翻转 1（#37 needs-triage + blocked）· 路线图伞形 1（#20，仅跟踪）。
- 本轮进展：**#58 验收通过并关闭**（详见 §三）；新立 **#59**（P3, type:test, ready-for-agent）入 ready 队列；§九 下沉候选清单修正（两项实际已完成，划掉）。

## 三、软件侧缺口扫描与近期 CI 事件（本轮更新）

- **#58（2026-10-09T14:51Z 验收通过关闭）**：Planner 独立复核（非仅采信报告）——`grep -n 'demo.sh' CLAUDE.md` 第 56 行命中且位于冒烟脚本清单区块内；格式与相邻 smoke 行对齐；`git show 0f87d25 --stat` 仅 CLAUDE.md 1 insertion；`grep -rn '/home/' CLAUDE.md` 无输出；`bash scripts/lint_shell.sh` rc=0（bash -n 15 脚本干净、无 --no-daemon 回归）；CI 双 success（merge `37940137342` + chore `37942367376`）。验收 comment 留档于 #58。
- **本轮缺口扫描结论（2026-10-09T14:55Z）：五项复核，一项真实候选落地；其余不灌水。** ① **测试登记合规**：38 个 `src/**/test/test_*.cpp`，26 个 `add_core_test` 注册 + 12 个排除，排除清单逐条有据（PCL/msgs/rclcpp 传递依赖），无漂移；② **CLAUDE.md 冒烟清单 15/15 覆盖** `scripts/*.sh`，#58 后无新漂移；③ 无开放 PR、无 auto-discovered 积压；④ **§九 下沉候选清单过时**：velocity_profiler（#24 的 `test_defect_differential` 已注册 `velocity_profiler.cpp`）与 safety_monitor 看门狗（#50 `test_reject_watchdog`）**实际已完成**——从候选区划掉；⑤ **Executor 连续 3 轮空转**（10-08 两轮 + 10-09T1446Z 轮）并发信号求介入——本轮回应：将剩余可行候选定级 ready 化。
- **新立 #59**（P3, type:test, ready-for-agent）：lidar_cluster 置信度评分几何核下沉——`ScoreAspectPenalty`/`ScoreSizePenalty`/`ScoringParams` 原样搬移 + PCA 由 `pcl::computeMeanAndCovarianceMatrix` 改为手写协方差 + `Eigen::SelfAdjointEigenSolver`（数学等价）+ 新 core_standalone 测试；边界、等价性论证、防空跑判据均已在 Issue 写死。已过 4.2 六条门禁。
- **skidpad `ComputeCenterline` 下沉：评估后暂不立项。** `ClusterCones` 本身仅是 y 分侧（无算法量）；真正算法在 `ComputeCenterline`→`AppendMidpoints`（PCL `KdTreeFLANN` 最近邻 + 中点配对 + 最近邻串链 + 去重）。下沉需把 KD-tree 最近邻替换为 O(n·m) 或等价查询器并做输出等价论证（tie-breaking 不确定性），边界比 lidar_cluster 复杂一档；留候选区（§九），触发条件：#59 交付后或队列再空时下一轮细化。
- 历史留痕（压缩备查）：#55（mpc-reject-smoke 假失败，`a99b600` 修复，post-fix Nightly 2/2 全绿关闭）/ #56（PATH 误判，零代码关闭）；pre-fix 10-04/10-05 同树全绿属间歇性时序窗口。

## 四、当前 ready 队列（给 Executor 的建议顺序）

1. **#59** lidar_cluster：置信度评分几何核下沉为纯 std core 并纳入 sanitizer 门（P3，`ready-for-agent`；参考 `git show` #53/#54 同模式）

- 队列 = **1 项**。为何只有 1 项：P0–P2 全部阻塞于仓库外输入（实车/台架/bag/ROS1 环境，用户 10-07 确认暂无到位）；软件侧唯一**完全**过 4.2 门禁的候选即 #59；skidpad 下沉边界未细化（§三），不硬塞——宁缺毋滥。
- #59 门禁核对：目标明确（点名函数/文件/新测试）/ 范围清楚（含"明确不包含"）/ 无依赖 / 无未决取舍（PCA 方案已指定）/ 验收逐条可判定（含 `ctest -N` 防空跑）/ 不触红线（纯测试增强，可回滚）。
- **#20 优先级豁免（长期约定）**：#20 为路线图伞形跟踪 issue（无执行体、随 S0–S3 存活），刻意不设 priority 标签，避免被饥饿规则误关；后续 Planning 不必再纠。
- **队列空转预案（回应 Executor 连续空转信号）**：#59 交付后若外部输入仍未到位，下一轮 Planner 必须主动做以下之一，不得再次空转收场：① 细化 skidpad `ComputeCenterline` 下沉候选并过门禁入队；② 对其他 PCL/msgs 排除模块（ground_segmentation、distortion_adjuster 等）评估同类下沉；③ 仍无合格候选 → PLAN 显式记录"已穷举扫描，等外部输入"，并考虑就外部输入时间表问用户一次（D-004 相关，非红线）。

## 五、已完成（软件侧摘要）

- 已关闭：S0 全部（#13/#14）、#22、#30–#36、#38–#54、#56、#55、#57、**#58（本轮）** 等。
- #17 ③④、#15/#16/#19 软件侧、#24 合成夹具已合入并附证据（见各 issue 评论与 git 历史）。
- **2026-10-02 ready 批（软件侧最后一批）**：#53（`eb75d08`）、#52（`fe2d97b`）、#54（`dbb77c0`）——均经 Planning 独立复核后 CLOSED。
- **#56**（本机 ccache/mold 环境缺口，实为 PATH 误判）：零代码变更，10-07 验收 CLOSED。
- **#55**（`mpc-reject-smoke` 纯度预检假失败）：`a99b600` / merge `48a4241`，本机 + push CI + post-fix Nightly 2/2 三线全绿，10-08 验收 CLOSED。
- **#57**（最小展示入口）：`c79b71d` / merge `97de15d`（README.md +28 / scripts/demo.sh +222），本机两臂 + 失败演练 A/B + push CI + 10-08 Nightly 全绿，10-09 验收 CLOSED。
- **#58**（CLAUDE.md 冒烟清单补 demo.sh 一行）：`0f87d25` / merge `6b5fc27`，四标准独立复核 + lint rc=0 + CI 双 success，10-09T14:51Z 验收 CLOSED。

## 六、CI 状态（本轮核实）

- push CI（main `eb120e7`）：最近 6 次全 success——`37942367376`（`eb120e7`）→ `37940137342`（merge `6b5fc27`）→ `37934195755`（`3dceae7`）→ `37855491072` 前序各次。主干不红。
- Nightly：**10-08 `37855491072` success（22:45Z，16m23s，树含 #57 merge）**；10-07 `37696935383` success。10-09 Nightly 将于 22:xxZ 产生，下一棒若在其后请看一眼。
- 结论：主干与 Nightly 均绿；下一次 Nightly 若红，须先区分新根因与既有两种模式（真污染 / 端点未收敛）。

## 七、已知阻塞与原因（均有证据）

| Issue | 阻塞原因（仓库外） | 证据 |
|---|---|---|
| #15（P1, blocked） | VCU 协议资料、台架标定实值 | #15 评论关闭条件；`docs/BENCH_CALIBRATION_TEMPLATE.md` |
| #16（P1, blocked） | 硬件急停链路、实车停车预算 | #16 评论关闭条件 |
| #17（P1, blocked） | 模型速度范围实测、感知扰动量化（数据 → #18） | #17 评论关闭条件 |
| #19（P1, blocked） | 依赖 #17 扰动场景 | #19 B 进入条件 |
| #18（P2, blocked） | 真实 bag | #18 依赖段 |
| #23（P1, blocked） | ROS1 环境 / 外部驱动确认 | #23 依赖段 |
| #24（P1, blocked） | 依赖 #23 + ROS1 环境 | #24 依赖段 |
| #37（P3, needs-triage + blocked） | 需 #15 实车/台架标定完成后翻转 `use_arbiter` 默认 | #37 body（"未标定前不得翻转"；needs-triage 与 blocked 并存系作者有意保留的"标定前不翻转"标记） |
| #25/#26/#27（needs-info） | 展示范围与人力（D-007(B) 已决；剩余待 #17①② / #18 证据） | #27 总入口；#26 评论 |

## 八、下一阶段（待输入，非当前阻断）

- #59 交付（纯测试增强，预计小）→ 按 §四 空转预案处理，不得再次无扫描空转。
- S3 性能评估（#19 B）：需实车/数据输入。
- 展示工作：D-007(B) 的最小展示（#57）与文档同步（#58）已交付关闭；完整展示（#26：视频/bag/release）与博客（#25）待 #17①② / #18 证据，到位后返工更新 demo 产物（该返工风险已由用户接受）。

## 九、已明确放弃 / 不做

- 调参腿已收口（docs §11.3）：无新维度动机且未按新 round 预注册前，不再扫描；**不冻结参数**（D-003）。
- 不从运动学模型推出轮胎极限结论（#17 边界）。
- 不用仿真证据关闭实车依赖 issue（D-004）。
- **ADR 下沉候选现状（2026-10-09 修正，原清单已过时）**：velocity_profiler 梯形规划、safety_monitor 看门狗——**已完成**（分别由 #24 / #50 落地进 core_standalone），划掉；lidar_cluster 几何核——已立 **#59**；skidpad `IcpApFPlanner::ComputeCenterline`——暂缓（KD-tree 等价性论证未做，§三），触发条件见 §四。
- **"统一发现收敛等待策略"（2026-10-08 评估后不立项，维持）**：`qos_contract_check.sh` 端点断言为单次查询 + `sleep 3`，与 #55 同属"发现收敛"家族；但除已修的 `mpc-reject-smoke` 外，其余运行时门禁的节点等待均有重试环（逐脚本核过），且 qos-contract 历史唯一失败系 `--no-daemon` 回归（已由 `03a3cfe` 修复）——**无假红证据，不立项、不灌水**。触发条件：任一运行时门禁再现"空表 / pub-sub=0"类假红，则立项统一退避等待。

## 十、Decision Gate 状态与给 Executor 的指令

- **当前无阻断性 Decision Gate，无待用户决策项；ready 队列 1 项（#59 P3，测试增强）。**
- 下一棒 Executor：领取 **#59**（边界与验收标准已在 Issue 写死；`git show` #53/#54 作同模式参考；PCA 手写协方差方案已指定，无自选空间）。**#58 已关闭、无需再动**；Nightly 10-09（22:xxZ）若在其后产生，看一眼即可。
- 展示范围（#25/#26/#27）：D-007(B) 已生效；#26 仍为完整展示父跟踪（needs-info）。
- Executor 空闲（无 LOCK）；`in-review` 0 项；外部 `M AGENTS.md` 脏改动非本体系产生，任何一棒都不碰（只读记录）。
