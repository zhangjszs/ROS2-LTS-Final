# PLAN.md — 路线、当前阶段与可执行队列

> 维护者：Planning Agent（时间流接力）｜最后更新：2026-10-10T08:13Z（`plan-20261010T0813Z`）
> 事实基线：main `04981c5`（与 origin/main 一致，`git fetch --all --prune` 已同步——首次因 HTTP2 framing 报错失败、重试成功；无未推送提交）；**LOCK 过期**（`executor-20261009T1508Z`，`heartbeat_at` 2026-10-09T16:05Z，至本轮 16 小时无心跳 = TTL 30 min 的 32 倍）；工作区仅 `M .agent/STATE.md`（Executor 未提交的收尾）与外部 `M AGENTS.md`；环境事实见 `.agent/ENV.md`。

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
- **展示（D-007-B）**：最小展示入口 #57 已关闭；CLAUDE.md 文档同步 #58 已关闭；完整展示（#26）与博客（#25）待 #17①② / #18 证据。
- **仓库外输入未到位期间的软件侧方向**：仅余 ADR 下沉候选可执行——#59（lidar_cluster）已交付待收尾，本轮新立 #60/#61（skidpad 圆路径与中点路径），详见 §三/§四。

## 二、当前 Milestone 目标与完成判据

- 目标（S1+S2）：接口/编码/单位/坐标系一致；固定场景可复现且 KPI 能识别失败；选定赛制低速闭环可重复完成，停车/失效/重启行为通过。
- 完成判据：以各 Issue 验收标准为准；其中依赖实车/台架/硬件的条目**不得以仅仿真证据关闭**（见 D-004）。
- Issue 台账（本 `gh` 实测，共 **15 open**）：ready 2（#60/#61）· in-progress 1（#59，待 Executor 补收尾）· 仓库外阻塞 7（#15–#19 / #23 / #24）· 展示待条件 3（#25/#26/#27 needs-info）· 待实车翻转 1（#37 needs-triage + blocked）· 路线图伞形 1（#20，仅跟踪）。
- 本轮进展：#59 六标准独立复核全过（未关闭，见 §三）；新立 **#60 / #61**（P3, type:test, ready-for-agent）入 ready 队列；10-09 Nightly 入账；ADR 候选清单按 #59/#60/#61 实际状态更新。

## 三、软件侧缺口扫描与近期 CI 事件（本轮更新）

- **#59（Executor 轮次异常收尾，Planner 未关闭）**：Executor `executor-20261009T1508Z` 的收尾未完成——LOCK 未删、`in-review` 标签未转移、执行报告未留、`.agent/` 状态变更未提交（工作区仅剩 `M .agent/STATE.md`）。但代码侧证据表明工作**已实际完成并合入**：临时分支已删、merge `04981c5` 在 main、STATE.md 自述"未完成工作：无"。按契约 §9（不关闭/不重置 `in-progress` Issue）与 §4.1（疑似中断由下一棒 Executor 自行判断），**本轮 Planner 不关闭、不改标签**，仅在 Issue 留独立复核证据供下一棒直接使用。复核结论：**六条标准全部独立验证通过，零差距**——
  - `bash scripts/core_standalone_check.sh` rc=0，26/26 passed，`Test #26: test_scoring_core` 在 ASan+UBSan 下 Passed；`ctest -N --test-dir build/core_standalone` 显示 `Test #26: test_scoring_core` / Total 26（防空跑）；gtest XML `tests="31" failures="0" errors="0"`。
  - PCA 等价性：黄金样本由 Planner **独立手算复核**（45° 点列 → 1.75；(1,2,3) 共线 → 0.2339845，均与 Issue 预期一致）；并核对 PCL 源码 `/usr/include/pcl-1.15/pcl/common/impl/centroid.hpp` 确认按 N 归一 + 非 dense 跳过非有限点——协方差常数缩放不改特征向量方向，等价性论证成立。
  - `colcon build/test --packages-select lidar_cluster` 双 rc=0，`test-result --verbose` 0 errors/0 failures/0 skipped；**未改动的** `test_lidar_cluster_scoring` `tests="32" failures="0"` 仍全绿。
  - 新 core 头 include 仅 `<Eigen/Dense> <algorithm> <cmath> <numbers> <span>`，无 rclcpp/common_msgs/PCL（仅注释提及）；`git show 467d570 --stat` 7 files，无 benchmarks/config/launch。
  - lint 双 rc=0；CI merge `37955949456` success。
  - 非阻断观察：`utility.cpp:468` 把 cluster cloud 设为 `is_dense = true`，故旧 PCL dense 分支**不**跳过非有限点；新实现一律跳过。dense 云中混 NaN 时旧行为经 NaN 钳位塌缩成"惩罚 0"（静默返回 0 的潜在隐患），新行为算真实倾斜。两侧测试均无 NaN 用例，不构成回归；已记录备查，若团队想要 dense+NaN 边界测试可另立小 Issue。
- **新立 #60 / #61（本轮，过 4.2 六条门禁）**：
  - **#60**（P3, type:test, ready-for-agent）：skidpad **中点回退路径** `ComputeCenterline` 几何核下沉。`AppendMidpoints` 的 `pcl::KdTreeFLANN<pcl::PointXY>::nearestKSearch` → 暴力 O(n·m) 扫描（复现 float 往返：坐标 cast float、double 中累加平方距离、结果 cast float 后与 `max_dist²` 比较；平局用严格 `<` 取最小下标）；中点配对后的串链/去重/lookahead 过滤三段已是纯 std，原样搬移。
  - **#61**（P3, type:test, ready-for-agent）：skidpad **圆路径**（`GeneratePath` 主路径）`FitCircleTaubin` + `ComputeCircleCenterline` 下沉。**顺带暴露一个参数驱动的可达 UB**：`icp_apf_planner.cpp:332` `static_cast<int>(path_lookahead_ / path_spacing_)`，而 `path_spacing_` 来自 ROS 参数 `path_station_spacing`（`skidpad_planner_node.cpp:45`，默认 0.5，**无下界校验**）→ 置 0 时 `20.0/0.0=+inf` → `static_cast<int>(+inf)` 是 UB，随后循环 ~2^31 次（实车等价卡死）。与 #38 同类：只有进 sanitizer 门才会被抓到。#61 已要求在 core 几何层加参数守卫，把 UB 变成"返回空 → 走既有回退路径"。
- **KD-tree 等价性 Planner 预验证（2026-10-10T08:13Z，临时程序未入仓库）**：对 `pcl::KdTreeFLANN<pcl::PointXY>::nearestKSearch` 做差分——连续 double 坐标 **10000 次抽样：最近邻下标 0 处不一致、阈值判定 0 处翻转、输出中点 0 处不一致**（bit 级等价）；整数网格坐标（制造精确平局）5000 次抽样：下标 412 处、中点 282 处不一致——**平局时 KdTreeFLANN 选哪个等距锥桶取决于其内部树结构，属未文档化的库内部行为**。结论已写入 #60/#61：真实传感器坐标下精确平局概率为 0，差分测试的随机输入必须用连续值；新实现改为确定性的最小下标，需在执行报告中如实记录。
- **本轮缺口扫描结论**：① **测试登记合规**：`src/**/test/test_*.cpp` 全部在 `tests/core_standalone/CMakeLists.txt` 有交代（注册或排除清单逐条有据），#59 后无漂移（`core_standalone_check.sh` 的覆盖审计步骤即校验此点，本轮 rc=0）；② CLAUDE.md 冒烟清单无新漂移；③ 无开放 PR、无 auto-discovered 积压；④ Executor 连续空转信号已由 #60/#61 回应（§四）。
- 历史留痕（压缩备查）：#55（mpc-reject-smoke 假失败，`a99b600` 修复）/ #56（PATH 误判，零代码关闭）/ #57（最小展示入口）/ #58（CLAUDE.md 补 demo.sh 一行）均已关闭并各自独立复核。

## 四、当前 ready 队列（给 Executor 的建议顺序）

1. **#60** skidpad 中心线回退路径：`ComputeCenterline` 几何核下沉为纯 std core 并纳入 sanitizer 门（P3，`ready-for-agent`）
2. **#61** skidpad 圆路径几何核：`FitCircleTaubin` / `ComputeCircleCenterline` 下沉为纯 std core 并纳入 sanitizer 门（P3，`ready-for-agent`）

- 队列 = **2 项**（满足 2–5 要求）。#61 依赖 #60 搬移的 `Point2D` / `CenterlineParams`（避免两份定义），故 #60 在前；两者同文件同模式，#59 的 `git show 467d570` 可作参考。
- 为何仍只有 2 项：P0–P2 全部阻塞于仓库外输入（实车/台架/bag/ROS1 环境，用户 10-07 确认暂无到位）；软件侧过 4.2 门禁的候选即 skidpad 剩余两块几何核（ADR 候选清单里 velocity_profiler / safety_monitor 看门狗已完成，#59 落地 lidar_cluster，本轮到 skidpad）。**#60/#61 落地后，ADR 下沉候选清单即基本清空**——下一轮若外部输入仍未到位，需按 §八 转向（新维度扫描或问用户时间表），不得再造同类 Issue 灌水。
- **#59 收尾指令（重要）**：#59 代码已合入且 Planner 已六标准复核通过，但因 Executor 轮次异常收尾，`in-review` 标签未转移、执行报告未留。**下一棒 Executor 请优先补收尾**：补执行报告 comment（契约 1.6 模板）、转移 `in-progress → in-review`、删除过期 LOCK、提交 `.agent/` 状态变更（`chore(agent):`，勿带入外部 `M AGENTS.md`）。证据已由 Planner 于 #59 comment 备好，无需重新验证。若下一棒判断无现场可恢复而走"放弃"路径，也请在 Issue 留一句说明。**代码本身无需再动。**
- **#20 优先级豁免（长期约定）**：#20 为路线图伞形跟踪 issue（无执行体、随 S0–S3 存活），刻意不设 priority 标签，避免被饥饿规则误关；后续 Planning 不必再纠。

## 五、已完成（软件侧摘要）

- 已关闭：S0 全部（#13/#14）、#22、#30–#36、#38–#54、#56、#55、#57、#58 等。
- #17 ③④、#15/#16/#19 软件侧、#24 合成夹具已合入并附证据（见各 issue 评论与 git 历史）。
- **2026-10-02 ready 批（软件侧最后一批）**：#53（`eb75d08`）、#52（`fe2d97b`）、#54（`dbb77c0`）——均经 Planning 独立复核后 CLOSED。
- **#55**（`mpc-reject-smoke` 纯度预检假失败）：`a99b600` / merge `48a4241`，本机 + push CI + post-fix Nightly 2/2 三线全绿，10-08 验收 CLOSED。
- **#56**（本机 ccache/mold 环境缺口，实为 PATH 误判）：零代码变更，10-07 验收 CLOSED。
- **#57**（最小展示入口）：`c79b71d` / merge `97de15d`（README.md +28 / scripts/demo.sh +222），本机两臂 + 失败演练 A/B + push CI + 10-08 Nightly 全绿，10-09 验收 CLOSED。
- **#58**（CLAUDE.md 冒烟清单补 demo.sh 一行）：`0f87d25` / merge `6b5fc27`，四标准独立复核 + lint rc=0 + CI 双 success，10-09T14:51Z 验收 CLOSED。
- **#59**（lidar_cluster 评分几何核下沉）：`467d570` / merge `04981c5`，Planner 六标准独立复核全过、零差距；**因 Executor 轮次异常收尾，未关闭，等下一棒补 `in-review` 转移**（§三/§四）。

## 六、CI 状态（本轮核实）

- push CI（main `04981c5`）：最近 8 次全 success——`3797599751`（10-09 Nightly, schedule）→ `37955949456`（#59 merge `04981c5`）→ `37947921737`（chore plan-1455Z）→ `37946760540`（chore exec-1446Z）→ `37942367376` → `37940137342`（#58 merge）→ `37934195755` → `37855491072`。主干不红。
- Nightly：**10-09 `3797599751` success（22:08Z）——本轮入账**；10-08 `37855491072` success（22:45Z）；10-07 `37696935383` success。**10-09 Nightly 观察点已清零。**
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
| #37（P3, needs-triage + blocked） | 需 #15 实车/台架标定完成后翻转 `use_arbiter` 默认 | #37 body 与 10-05 三角定位评论（"标定证据到位前不翻转默认"） |

本轮复查：#15–#19 最后评论均为 10-02「软件侧收口审计」、#37 为 10-05 三角定位评论，**阻塞项均无新进展**，维持原判。#25/#26/#27（needs-info）待 #17①② / #18 证据，D-007-B 已生效。

## 八、下一阶段（待输入，非当前阻断）

- #59 收尾（下一棒 Executor 补 label/报告/LOCK/.agent 提交，代码不动）→ Planner 下一轮按 `in-review` 正常验收关闭。
- #60 → #61 交付（纯测试增强 + 一处 UB 守卫）。**两者落地后 ADR 下沉候选清单基本清空**，届时若外部输入仍未到位：① 换维度扫描（如 colcon 侧未进 sanitizer 的其他纯 std 数学、config 参数校验类守卫）；② 仍无合格候选 → PLAN 显式记录"已穷举扫描，等外部输入"，并就外部输入时间表问用户一次（D-004 相关，非红线）。不得再造同类下沉 Issue 灌水。
- S3 性能评估（#19 B）：需实车/数据输入。
- 展示工作：D-007(B) 的最小展示（#57）与文档同步（#58）已交付关闭；完整展示（#26：视频/bag/release）与博客（#25）待 #17①② / #18 证据，到位后返工更新 demo 产物（该返工风险已由用户接受）。

## 九、已明确放弃 / 不做

- 调参腿已收口（docs §11.3）：无新维度动机且未按新 round 预注册前，不再扫描；**不冻结参数**（D-003）。
- 不从运动学模型推出轮胎极限结论（#17 边界）。
- 不用仿真证据关闭实车依赖 issue（D-004）。
- **ADR 下沉候选现状（2026-10-10 更新）**：velocity_profiler 梯形规划、safety_monitor 看门狗计时——**已完成**（分别由 #24 / #50 落地进 core_standalone）；lidar_cluster 几何核——**已完成**（#59，代码已合入，待收尾）；skidpad `IcpApfPlanner::ComputeCenterline`（中点路径）——已立 **#60**；skidpad `FitCircleTaubin` / `ComputeCircleCenterline`（圆路径）——已立 **#61**。**ADR 正文里"后续候选"清单应随 #60/#61 落地再同步更新一次**（由 Executor 在做 #60/#61 时顺带改，与 #59 同一模式）。
- **KD-tree 最近邻替换暴力扫描的平局语义（2026-10-10 定，D-008）**：新实现用严格 `<` 取最小下标，确定性且跨 PCL 版本可复现；旧实现在精确平局时选哪个等距点由 `KdTreeFLANN` 内部树结构决定，属未文档化库行为，**不作为契约保留**。真实传感器坐标下精确平局概率为 0；差分测试的随机输入必须用连续值（整数网格会测到旧实现的未定义行为）。
- **"统一发现收敛等待策略"（2026-10-08 评估后不立项，维持）**：`qos_contract_check.sh` 端点断言为单次查询 + `sleep 3`，与 #55 同属"发现收敛"家族；但除已修的 `mpc-reject-smoke` 外，其余运行时门禁的节点等待均有重试环（逐脚本核过），且 qos-contract 历史唯一失败系 `--no-daemon` 回归（已由 `03a3cfe` 修复）——**无假红证据，不立项、不灌水**。触发条件：任一运行时门禁再现"空表 / pub-sub=0"类假红，则立项统一退避等待。

## 十、Decision Gate 状态与给 Executor 的指令

- **当前无阻断性 Decision Gate，无待用户决策项；ready 队列 2 项（#60 P3、#61 P3，均为测试增强 + 一处 UB 守卫）。**
- 下一棒 Executor 建议顺序：**先补 #59 收尾**（label/报告/LOCK/.agent 提交，代码不动，证据 Planner 已备好）→ **#60** → **#61**。#59 的 in-progress 标签在下一棒转移前，请勿新开分支动它的代码（现场已无未完成代码，收尾是纯流程动作）。
- Nightly 10-09 已入账（success）；10-10 Nightly 22:xxZ 将产生，下一棒若在其后看一眼即可。
- 展示范围（#25/#26/#27）：D-007(B) 已生效；#26 仍为完整展示父跟踪（needs-info）。
- Executor 未在运行（LOCK 过期 16 小时）；外部 `M AGENTS.md` 脏改动非本体系产生，任何一棒都不碰（只读记录）；`.agent/STATE.md` 的未提交改动属 Executor 上一轮收尾，下一棒提交时只 `git add .agent/`。
