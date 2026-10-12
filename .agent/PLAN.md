# PLAN.md — 路线、当前阶段与可执行队列

> 维护者：Planning Agent（时间流接力）｜最后更新：2026-10-12T13:34Z（`plan-20261012T1334Z`）
> 事实基线：main `9cbda10`（与 origin/main 一致，已推送；`git fetch --all --prune` 已同步；`git pull --rebase` 因外部 `M AGENTS.md` 未暂存改动而跳过——非本体系产生，任何一棒都不碰；无 LOCK（Executor 空闲）；工作区仅外部 `M AGENTS.md`；环境事实见 `.agent/ENV.md`。

## 一、当前阶段

路线图来源：#20「ROS 2 车队开发路线：运行基线 → 可信仿真 → 低速闭环 → 性能优化」（S0–S3）。

- **S0 可重复启动：完成**（#13/#14 已关闭；CI/lint/headless/qos 门常绿）。
- **S1 可信接口与仿真 / S2 低速稳定闭环：软件侧基本完成，剩余卡仓库外输入**（同上轮，未变）：
  - #15 共用执行器适配层：软件侧已交付并审计；剩余 = VCU 协议资料 + 台架实车标定。
  - #16 任务/安全状态与最终仲裁：软件侧闭环；剩余 = 硬件急停链路 + 实车停车预算。
  - #17 可复现仿真/KPI/回归：③④已合入并进 CI 硬门；①②受实车标定与感知扰动数据阻塞。
  - #19 Pure Pursuit 低速基线：A 基线 + B 骨架已在；B 完整公平对比依赖 #17 扰动场景。
  - #18 rosbag 数据集与感知质量评估：需真实 bag。
- **S3 性能优化**：#19 B 待 S2 条件满足。
- **展示（D-007-B）**：最小展示入口 #57 已关闭；CLAUDE.md 文档同步 #58 已关闭；完整展示（#26）与博客（#25）待 #17①② / #18 证据。
- **ADR 下沉线：#59/#60/#61 本轮全部验收关闭，下沉候选清单清空**；本轮新维度扫描仅立得 **#62**（直线拟合核，详见 §三/§四）。软件侧可执行池事实上见底，详见 §八。

## 二、当前 Milestone 目标与完成判据

- 目标（S1+S2）：接口/编码/单位/坐标系一致；固定场景可复现且 KPI 能识别失败；选定赛制低速闭环可重复完成，停车/失效/重启行为通过。
- 完成判据：以各 Issue 验收标准为准；其中依赖实车/台架/硬件的条目**不得以仅仿真证据关闭**（见 D-004）。
- Issue 台账（本 `gh` 实测，共 **12 open**，上轮 15 − 本轮关闭 #59/#60/#61）：ready 1（#62）· in-review 0 · in-progress 0 · 仓库外阻塞 7（#15–#19 / #23 / #24）· 展示待条件 3（#25/#26/#27 needs-info）· 待实车翻转 1（#37 needs-triage + blocked）· 路线图伞形 1（#20，仅跟踪）。
- 本轮进展：#59/#60/#61 逐条独立验收后全部 CLOSED（证据见 §三）；新立 #62 入 ready 队列；主干 push CI 连续 3 次 success（`4d795d9`/`8c7db61`/`9cbda10`）。

## 三、验收结论与近期 CI 事件（本轮更新）

- **#59（lidar_cluster 评分几何核）：通过关闭**（验收 comment `6125381041`）。Planner 10-11 已六标准独立复核全过；本轮确认合入后代码零再动（`git log 04981c5..HEAD -- src/sensors/lidar_cluster/` 为空）+ 当前 main 新跑 `core_standalone_check.sh` rc=0（`Test #26` Passed，28/28）+ 主干 CI success 链。残余观察（dense+NaN）留备查，不阻塞。
- **#60（skidpad 中点回退路径）：通过关闭**（验收 comment `6125381825`）。差分测试（种子 `20261011ULL`）在 `test_icp_apf_planner.cpp:204` 确认存在；`Test #27` 当前 main 全绿；报告时 `38151381892` 的 lint 失败经核对为 pre-existing 格式问题（下条），已由 `69ecc1e` 修为注释上移（diff 2 增 1 删、零语义），此后主干持续 success，不构成差距。D-008 平局语义不再复议。
- **#61（skidpad 圆路径）：通过关闭**（验收 comment `6125382685`）。七条字面全满足；两处有据偏离予以接受：① 半圆紧容差/共线 invalid 预期与原算法矛盾——本轮核对移植逐字等价（均值/矩/矩阵/特征筛选全同，仅字段改名），偏差属原算法固有，改算法会改变实车行为且超范围；且验收条文只对单位圆/偏心圆要求容差、对退化分支只要求走通不崩溃，字面均满足。② 守卫强于建议（含 NaN + isfinite），对合法配置零行为差。遗留（有限极大比值上溢/部分弧精度/双拟合冗余）均需行为决策，未另立 Issue，有需要时再议。
- **CI 事件：10-11 起主干 lint 门变红 → 已修复 → 三连绿**。根因：#60 合入的 `test_centerline_core.cpp:128` 行尾注释使 clang-format 本机 21.1.8 与 CI apt 18.x 产生互斥换行要求（Executor 用 18.1.8/21.1.8 双二进制复现并验证，均有证据）。影响：`39a40a6` 上 push ×2 + schedule ×1 + `cb54e23`/`7e5794d` 各 ×1 失败——均为同一 lint 单点，无其他失败。修复 `69ecc1e`（注释上移，零语义）后 `4d795d9`/`8c7db61`/`9cbda10` 三连 success。**10-11 Nightly（`38176152190`）因此 failure**——Nightly 连绿中断一次，原因同上（单点格式问题，非内容回归）；10-12 Nightly（21–22Z，在修复后主干上跑）预期恢复 success，下一轮入账核对。
- **新维度扫描（§八转向第一步，Executor 的给 Planner 信号已回应）**：对 `tests/core_standalone` 排除清单逐项评估（只读扫描 + Planner 亲自核对关键文件行号）——
  - **立项 #62**（line_detector 直线拟合核，rank 1）：`HoughFit/RansacFit/LeastSquaresFit/IsLineGood/PointToLineDistance/ApplyTemporalFilter` + 纯标量 config，近乎纯 std（仅 `HuatCone`/`Point32` Plain struct，无 rclcpp/Eigen/PCL），同包已有 `cone_boundary.h` concept 先例；UB 价值明确（`line_detector.cpp:45` NaN `rho_resolution` → float→int UB，与 #61 同类；负 spread 绕过、时序三分支/回退链零覆盖）。过 4.2 六条门禁，已立为 P3 ready。
  - **暂缓 Way/delaunay/kdtree 链**（rank 2）：确有未覆盖纯数学与 UB 点（`Way.cpp:179` size<5 时 `std::prev(end,5)`、`Circle.cpp` 共线除零等），但弥散在 ~1000 行且 API 纠缠（Eigen/消息返回/Node 日志），拆分=事实上的接口改造，风险与体量超出 P3 测试 Issue 的承载。记录在此，不立项；若将来外部输入仍无且决定啃它，需先拆成"KDTree+Point/Circle 先行、Way 后行"两步走。
  - **跳过**：`Node.hpp`（薄封装，零 sanitizer 价值）、encoder（已下沉 core 上的薄适配，无需拆分）、config 参数守卫类（307 处 `declare_parameter`，单点不成 Issue，#61 模式已覆盖其最高价值实例）。
  - 历史留痕（压缩备查）：#55（mpc-reject-smoke 假失败，`a99b600`）/ #56（PATH 误判，零代码关闭）/ #57（最小展示入口）/ #58（CLAUDE.md 补 demo.sh 一行）均已关闭并各自独立复核。

## 四、当前 ready 队列（给 Executor 的建议顺序）

1. **#62** straight_line_planner 直线拟合核下沉为纯 std core 并纳入 sanitizer 门（P3，`ready-for-agent`；Hough/RANSAC/LSQ/时序滤波 + NaN 守卫 + RANSAC 确定性种子）

- 队列 = **1 项**（低于 2–5 数字目标，系有意为之：扫描仅此一项过门禁，"不得灌水"优先于数字；Way 链暂缓理由见 §三）。
- #62 与 #59–#61 同文件不同包、同模式（`git show 7e8c07f --stat` 与 `test_circle_centerline_core.cpp` 可作参考；`cone_boundary.h` 是同包 concept 先例）。
- **#20 优先级豁免（长期约定）**：#20 为路线图伞形跟踪 issue（无执行体、随 S0–S3 存活），刻意不设 priority 标签，避免被饥饿规则误关；后续 Planning 不必再纠。

## 五、已完成（软件侧摘要）

- 已关闭：S0 全部（#13/#14）、#22、#30–#36、#38–#54、#56、#55、#57、#58 等。
- #17 ③④、#15/#16/#19 软件侧、#24 合成夹具已合入并附证据（见各 issue 评论与 git 历史）。
- **2026-10-02 ready 批（软件侧最后一批）**：#53（`eb75d08`）、#52（`fe2d97b`）、#54（`dbb77c0`）——均经 Planning 独立复核后 CLOSED。
- **#55**（`mpc-reject-smoke` 纯度预检假失败）：`a99b600` / merge `48a4241`，本机 + push CI + post-fix Nightly 2/2 三线全绿，10-08 验收 CLOSED。
- **#56**（本机 ccache/mold 环境缺口，实为 PATH 误判）：零代码变更，10-07 验收 CLOSED。
- **#57**（最小展示入口）：`c79b71d` / merge `97de15d`（README.md +28 / scripts/demo.sh +222），本机两臂 + 失败演练 A/B + push CI + 10-08 Nightly 全绿，10-09 验收 CLOSED。
- **#58**（CLAUDE.md 冒烟清单补 demo.sh 一行）：`0f87d25` / merge `6b5fc27`，四标准独立复核 + lint rc=0 + CI 双 success，10-09T14:51Z 验收 CLOSED。
- **#59**（lidar_cluster 评分几何核下沉）：`467d570` / merge `04981c5`，六标准独立复核 + 执行报告 + 本轮确认代码零再动，10-12 验收 CLOSED。
- **#60**（skidpad 中点回退路径下沉）：`4551602` / merge `cb54e23`（差分种子 `20261011ULL`）+ 零语义格式修复 `69ecc1e`，10-12 验收 CLOSED。
- **#61**（skidpad 圆路径下沉 + 参数守卫）：`7e8c07f` / merge `7e5794d`（13 用例，NaN-safe + isfinite 守卫），10-12 验收 CLOSED。

## 六、CI 状态（本轮核实）

- push CI（main `9cbda10`，最新提交）：`38244570254` success；`38243907339`（`8c7db61`，.agent-only）success；`38241684733`（`4d795d9`，#61+格式修复）success。三连绿，主干健康。
- **三种失败模式与应对认知维持有效**（§三事件属"真污染"中的最轻一级：单点格式问题，已修；另两种模式本轮未触发）：① 真污染（新引入失败需排查定位）；② 端点未收敛（`--no-daemon` / 发现收敛家族）；③ ros2-apt-source 404（环境准备阶段间歇性 curl 404，`gh run rerun <id> --failed` 即可）。
- push CI 近况：`38244570254`（`9cbda10` chore, success）→ `38243907339`（`8c7db61` chore, success）→ `38241684733`（`4d795d9` #61+格式修复, success）→ `38241256078`（`7e5794d`, failure，唯一原因 #60 遗留格式问题）→ `38151457809`（`39a40a6`, failure，同因）。
- Nightly：10-11 `38176152190` failure（同因，非内容回归，连绿中断一次）；10-10 `38086098600` success；10-09 `37997599751` success。**10-12 Nightly（21–22Z，修复后主干）待入账**，下一轮核对。
- 结论：主干已恢复全绿；10-11 Nightly 的 failure 有明确单点根因并已修复，无需介入。

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

本轮复查：被阻塞 8 项的 `updatedAt` 停在 10-05/10-09（`gh` 实测，无新评论无状态变化），**阻塞项均无新进展**，维持原判。#25/#26/#27（needs-info）待 #17①② / #18 证据，D-007-B 已生效。

## 八、下一阶段（待输入，非当前阻断）

- #62 交付（直线拟合核下沉 + NaN 守卫 + RANSAC 确定性种子；生产默认随机性不得改变，Issue 已写明）。
- **#62 落地后，软件侧可执行池即见底**（Way 链暂缓、灌水禁止）：届时 PLAN 显式记录"已穷举扫描，等外部输入"， routine 转为空转健康轮；外部输入（实车/台架/bag/ROS1 环境）时间表问题本轮已问用户一次（见轮末汇报 Q1），答复前不再重复问。
- S3 性能评估（#19 B）：需实车/数据输入。
- 展示工作：D-007(B) 的最小展示（#57）与文档同步（#58）已交付关闭；完整展示（#26：视频/bag/release）与博客（#25）待 #17①② / #18 证据，到位后返工更新 demo 产物（该返工风险已由用户接受）。

## 九、已明确放弃 / 不做

- 调参腿已收口（docs §11.3）：无新维度动机且未按新 round 预注册前，不再扫描；**不冻结参数**（D-003）。
- 不从运动学模型推出轮胎极限结论（#17 边界）。
- 不用仿真证据关闭实车依赖 issue（D-004）。
- **ADR 下沉候选现状（2026-10-12 更新）**：velocity_profiler 梯形规划（#24）、safety_monitor 看门狗（#50）、lidar_cluster 评分（#59）、skidpad 回退中心线（#60）、skidpad 圆路径（#61）——**全部完成**；straight_line_planner `ClusterCones` 已由 `cone_boundary.h` 下沉（test_cone_boundary 覆盖）；**直线拟合核（Hough/RANSAC/LSQ/时序）——已立 #62（本轮扫描唯一过门禁项）**；Way/delaunay/kdtree 链——**暂缓**（体量 ~1000 行 + API 纠缠，拆分即接口改造，超 P3 测试 Issue 承载；将来啃则 KDTree+Point/Circle 先行、Way 后行）。**ADR 正文的"后续候选"清单待 #62 落地后再同步一次**（Executor 顺带改，与 #59–#61 同一模式）。
- **KD-tree 最近邻替换暴力扫描的平局语义（2026-10-10 定，D-008，#60 已落实）**：不再复议。
- **"统一发现收敛等待策略"（2026-10-08 评估后不立项，维持）**：触发条件不变（任一运行时门禁再现"空表 / pub-sub=0"类假红则立项）。
- **clang-format 18（CI）/21（本机）skew（2026-10-12 观察，非决策）**：全树双版本扫过，仅 #60 一处分歧并已修复；若后续高频再现，再评估 pin CI 大版本（属 CI 流程变更，需用户决定，本轮不动 ci.yml、不提问）。
- **config 参数守卫类（2026-10-12 扫描结论，不立项）**：307 处 `declare_parameter`，#61 模式已覆盖其最高价值实例（UB 类）；其余多为范围/类型校验，无 sanitizer 可证伪性，不值得单独立项。

## 十、Decision Gate 状态与给 Executor 的指令

- **当前无阻断性 Decision Gate；待用户决策 1 项（外部输入时间表 Q1，见轮末汇报，不阻断 #62）。**
- 下一棒 Executor 建议顺序：**#62**（单项队列；RANSAC 种子方案选风险最小者并记录；生产默认随机性不得改变；ADR 正文后续候选清单顺带同步）。
- 10-12 Nightly 21–22Z 将产生（修复后主干首个 Nightly），下一棒若在其后看一眼入账即可。
- 展示范围（#25/#26/#27）：D-007(B) 已生效；#26 仍为完整展示父跟踪（needs-info）。
- Executor 未在运行（无 LOCK）；外部 `M AGENTS.md` 脏改动非本体系产生，任何一棒都不碰（只读记录）。
