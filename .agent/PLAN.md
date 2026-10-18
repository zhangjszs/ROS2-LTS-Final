# PLAN.md — 路线、当前阶段与可执行队列

> 维护者：Planning Agent（时间流接力）｜最后更新：2026-10-18（`plan-20261018` 轮：**连续第 6 天无 Executor 活动**、无待验收项（连续第 5 轮）、**复核 #62 发现背景技术断言失实并已更正正文+评论**、Nightly **六连绿**（10-12→10-17）、`TASKS.md` 重写刷新、今日队列仍 `#62` 单项）
> 事实基线：main `c2978bd`，**已与 origin/main 同步（0/0）**——2026-10-18 用户亲选 D-010=B 并授权立即推送，7 笔 docs/.agent-only 提交（`c0652e1`…`c2978bd`）已 `git push origin main`（`66a8d26..c2978bd`）进入远端备份，单点丢失风险解除。`git pull --rebase` 仍因外部 `M AGENTS.md` 未暂存改动无法执行，但已无分叉需要 rebase；该文件是外部本地指引，任何一棒都不碰、也不代提交；无 LOCK；`.agent/DIARY/` 仍仅 README（无 Executor 轮故无日报，属预期）；环境事实见 `.agent/ENV.md`。
> ⚠️ **连绿口径精度**：Nightly 10-12/13/14/15/16/17 六次 success 的 `head_sha` **全部是 `66a8d26`**——即截至本轮 push 之前，CI 验证的始终只是已推送代码。这 6 笔 docs/.agent 提交现已推送，**下一次 push/Nightly 将首次覆盖到包含规划记录的新基线**；引用"六连绿"时仍应注明那是 `66a8d26` 上的结果。

## 〇、本轮最重要的产出：#62 的技术前提被复核为失实（本轮新增，非昨日遗留）

按 §3「不要只信报告的文字」对 #62 逐条复核，**发现其背景第 1 条断言与代码实际行为不符**。已在 Issue 评论（`issuecomment-6245035826`）留证并重写正文。要点：

- **失实断言**：`hough_rho_resolution` 经 `std::max(0.05, ·)` 后 NaN 仍传播 → `static_cast<int>(NaN)` 是 UB。**实测不成立**——`std::max(a,b)` 是 `(a<b)?b:a`，与 NaN 的比较恒为 false，故 `line_detector.cpp:40` 已把 NaN 钳为 `0.05`，第 45 行输入恒在 `[0,400]`，无 UB。
- **真正的 UB 在 `line_detector.cpp:87`**，参数是 `hough_min_inlier_ratio`（无守卫，经 `straight_line_planner_node.cpp:68` 对外暴露）。UBSan 实测报 `nan is outside the range of representable values of type 'int'`。
- **遗漏的可达性前提**：`HoughFit` 仅在 `enable_hough=true` 时调用，而该参数**默认 false**（`line_detector.hpp:36`）。原验收用例若不显式构造此前置，会走不到被测分支而**假绿**——已在验收标准中要求"先红后绿 + 断言走到目标分支"。
- **任务价值不变**：直线拟合核仍无 sanitizer 覆盖、时序滤波三分支与回退链仍零直接单测、RANSAC 仍不可复现（`L130` `random_device` 播种）。#62 继续 ready。

> **方法论教训（值得记住）**：#62 立项时的依据是 Planner 自己的扫描结论，历经 5 轮"复核仍过门禁"却从未有人实际编译验证过那条断言。**"复核"不等于"验证"**——后续任何以"某处存在 UB/崩溃"为卖点的 Issue，Planner 在放行前应至少用一次性程序实测该断言（写在 `/tmp`，不入仓库），否则复制的很可能是一段听起来合理的推理而非事实。

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
- **ADR 下沉线：#59/#60/#61 全部验收关闭，下沉候选清单清空**；仅剩 **#62**（直线拟合核，**本轮已复核并更正其失实的技术前提**，详见 §三/§〇/§四）。软件侧可执行池事实上见底，详见 §八。

## 二、当前 Milestone 目标与完成判据

- 目标（S1+S2）：接口/编码/单位/坐标系一致；固定场景可复现且 KPI 能识别失败；选定赛制低速闭环可重复完成，停车/失效/重启行为通过。
- 完成判据：以各 Issue 验收标准为准；其中依赖实车/台架/硬件的条目**不得以仅仿真证据关闭**（见 D-004）。
- Issue 台账（本 `gh` 实测，共 **27 open**，与上轮持平；逐项 `updatedAt` 本轮复核**零变动**）：ready-for-agent 1（#62，0 comment，`updatedAt` 停在 10-12T13:39Z，**已连续 6 天未被领取**）· in-review 0 · in-progress 0 · 仓库外阻塞 8（#15–#19 / #23 / #24 / #37，`updatedAt` 停在 10-05/10-09，无新评论 ⇒ D-009 的"输入到位即返工"触发条件未命中）· 展示待条件 3（#25/#26/#27 needs-info）· ready-for-human 14（#63–#76，用户跟进，均无变化）· 路线图伞形 1（#20，仅跟踪，优先级豁免见 §四）。
- 本轮进展：自 `1687e2a`（plan-20261017）起 git 零新提交、#62 仍 0 comment、27 项 Issue `updatedAt` 无一变动——**10-17→10-18 无 Executor 活动**（连续第五轮无验收对象；#62 自 10-12 立为 ready 后连续 6 天未被领取）。CI 侧无新 push 运行（本地未推）；Nightly 10-17 success 已入账（见 §三）。本轮治理动作：① 六路实查（git log / 分支 / stash / LOCK / PR / Issue `updatedAt`）确认零执行；② **实测复核 #62 的技术前提，判定其背景断言失实，已更正正文与验收标准并留证评论**（本轮唯一的实质性技术产出）；③ `TASKS.md` 按 Issue 实测重写刷新；④ D-010 计数推进至第 3 天（仍不推）。

## 三、验收结论与近期 CI 事件（本轮更新）

- **`plan-20261018` 轮验收结论：无待验收项（连续第五轮）**。证据（六路实查）：`git log 1687e2a..HEAD` 为空；27 open 的 `updatedAt` 逐项与上轮一致（#62 仍停 `2026-10-12T13:39:45Z`、**0 comment**）；in-review 0 · in-progress 0；无本地/远程新分支（仅 `main`）、无 stash、无 LOCK、无 open PR；#59/#60/#61 仍 CLOSED（`updatedAt` 停在 10-12T13:36-37Z，无 reopen）。上轮活动 `1687e2a`（plan-20261017 本身）已复核 `--stat`：仅 `.agent/` 4 文件，无代码触碰。**昨日（10-17）日报缺失，已用 git log + Issue 反推**：10-17 无 Executor 轮（除规划轮本身外零活动），零产出即零报告义务，非链路断裂。
- **本轮实质性产出 —— #62 技术前提复核（详见 §〇）**：`std::max(0.05, NaN)` 实测返回 `0.05`，原文"NaN 传播致 float→int UB"**不成立**；真正无守卫的是 `line_detector.cpp:87` 的 `hough_min_inlier_ratio`（UBSan 实测报错）；另发现 `enable_hough` 默认 false 带来的**假绿风险**。已更正 Issue 正文 + 留证评论 `issuecomment-6245035826`。**这是 6 天零执行期间本轮唯一有实质技术增量的产出**——它证明队列里并非"无事可做"，而是这些任务的**前提本身需要被验证**。
- **Nightly 10-16（`created_at 2026-10-16T20:25:09Z`）/ 10-17（`2026-10-17T19:59:13Z`）success 入账**：自 10-12 修复 lint 单点后**连绿 6 次**（10-12 / 10-13 ×2 / 10-14 / 10-15 / 10-16 / 10-17）。⚠️ 口径同文首：六次 `head_sha` 全为 `66a8d26` = origin/main，**不含本地未 push 的 6 笔**。

- **`plan-20261017` 轮验收结论：无待验收项（连续第四轮）**。证据（六路实查）：`git log f704480..HEAD` 为空；27 open 的 `updatedAt` 逐项与上轮一致（#62 仍停 `2026-10-12T13:39Z`、**0 comment**）；in-review 0 · in-progress 0；无本地/远程新分支、无 stash、无 LOCK、无 open PR；#59/#60/#61 仍 CLOSED 无 reopen。上轮唯一活动 `f704480`（plan-20261016 本身）已复核 `--stat`：仅 `.agent/` 4 文件（DECISIONS / DIARY README / PLAN / TASKS），无代码触碰。**昨日（10-16）日报缺失，已用 git log + Issue 反推**：10-16 无 Executor 轮（除规划轮本身外零活动），零产出即零报告义务，非链路断裂。
- **`plan-20261016` 轮验收结论：无待验收项（连续第三轮）**。证据：`git log 33d852f..HEAD` 为空；`gh` 实测 27 open 的 `updatedAt` 逐项与上轮一致（#62 仍停 `2026-10-12T13:39Z`、0 comment）；in-review 0、in-progress 0；#59/#60/#61 仍 CLOSED 无 reopen。**零产出即零报告义务**，不构成验收缺口，但说明**循环没转起来**：Planner 每天给出 1 项 ready，Executor 连续 4 天没领。本轮已把该事实作为最突出问题写在 §十 给 Executor 的指令里。
- **`.agent/DIARY/` 本轮补建**（此前从未建立，git 历史无记录）：契约 1.8 规定日报是 Planner 次日验收的**唯一输入**，该链路此前一直靠 HANDOFF/STATE/git log 代行。补 `.agent/DIARY/README.md` 写明命名（`YYYY-MM-DD.md`）、写入者（Executor）、追加不覆写与「更正：<原文> → <更正为>」规则。**Planner 不代写日报内容**（所有权属 Executor）。
- **Nightly 10-15 success 入账**（`created_at 2026-10-15T20:51Z`，head `66a8d26`）：自 10-12 修复 lint 单点后**连绿 4 次**（10-12 / 10-13 ×2 / 10-14 / 10-15，注：10-13 有两次运行）。⚠️ 口径精度：四次 `head_sha` 全为 `66a8d26` = **origin/main**，即 CI 验证的是已推送代码，**不含本地未 push 的 4 笔 docs/.agent 提交**；引用"主干连绿"时不得暗示本地基线已被 CI 覆盖。
- **push 决策登记为 D-010 pending（本轮治理修正）**：该决策自 10-13 HANDOFF 起悬空（"用户睡前未定"），10-13/10-15 两轮 PLAN §十 反复写"push 待用户决策"却**从未在 DECISIONS.md 建条目**——无编号、无选项、无默认项、无询问天数，不符 §6.3/§6.4。本轮补齐并按 6.3 起算 pending（第 1 天），默认方案 = **推送**（理由与代价见 D-010）。**在用户答复前仍不推**。

- **跨天间隙（10-13→10-15）验收结论：无待验收项**。自 `2ca0556` 起 git 零新提交；#62 零 comment、`updatedAt` 停在 2026-10-12T13:39Z；`gh` 实测 in-review 0、in-progress 0。#59/#60/#61 仍为 CLOSED，无 reopen、无新评论。连续第二轮无验收对象——属 D-009 前夜的预期内空转（#62 未被领取）。
- **昨日日报缺失（延续），已用 HANDOFF + STATE + git log + Issue 反推**：`.agent/DIARY/` 仍从未建立；HANDOFF 顶部仍为 `test-perf-20261013` 轮（10-13 后无新 Executor 轮）。反推结论：10-14 无执行活动，无证据断裂（零产出即零报告义务）。
- **Nightly 10-13（`38476242083`）/ 10-14（`38638548262`）success 入账**：均在 origin/main `66a8d26` 上跑；自 10-12 起 Nightly **三连绿**（10-12/13/14），lint 单点故障已彻底过去，主干健康无争议。

- **昨日日报缺失，已用 HANDOFF + STATE + git log + Issue 反推**：`.agent/DIARY/` 从未建立（git 历史无记录），昨日唯一产出为用户驱动的测试性能轮（HANDOFF 顶部 `test-perf-20261013` 节即其报告本体），无标准 Executor 执行报告链断裂问题——该轮本就不是标准执行轮。
- **测试性能轮（10-12/13，用户驱动）入账**：`docs/TEST_PERF.md`（381 行，`c0652e1`）+ 索引提交 `3208ae4` + issue **#63–#76**（14 项，`gh` 实测全部存在，均为 `ready-for-human` + P2/P3）。核对：文件存在且行数吻合（381 行；HANDOFF 记 375 行系初版计数，`3208ae4` 追加 8 行索引，属实）；零代码/CI/门禁变更（`git show --stat` 两提交仅 touched docs + .agent）；**未验收项**：本地全量实测结论（3215s、6 门禁环境性失败）采信 HANDOFF 证据链，未独立重跑（零代码变更，重跑无新增信息；且 10-12 本机 DDS 退化期数据口径已在 TEST_PERF.md §2.3 注明）。
- **Nightly 10-12（`38325320159`，head `66a8d26`）success 入账**：上轮预期"修复后主干首个 Nightly 恢复 success"兑现，10-11 的 failure 确认为单点格式问题、连绿恢复，无需介入。
- **#59/#60/#61 状态复核**：`gh` 实测三者均为 CLOSED，无 reopen、无新评论；in-review 队列为空，**本轮无待验收项**。

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

1. **#62** straight_line_planner 直线拟合核下沉为纯 std core 并纳入 sanitizer 门（P3，`ready-for-agent`；Hough/RANSAC/LSQ/时序滤波 + **第 87 行 `hough_min_inlier_ratio` 参数守卫** + RANSAC 确定性种子）——本轮复核：仍过 4.2 门禁、`updatedAt`/评论零变动，**已连续 6 天未被领取**；本轮已更正其失实的背景断言与验收标准（见 §〇）。

- 队列 = **1 项**（低于 2–5 数字目标，系有意为之：扫描仅此一项过门禁，"不得灌水"优先于数字；Way 链暂缓理由见 §三）。本轮未做新扫描——D-009 精神：不为找活而扫；且 #62 未交付、无新证据，重复扫描无新增信息。
- **#62 领取时必读**：正文 §当前问题 顶部的 Planner 更正块。核心变化 = 被测 UB 从 `hough_rho_resolution`（实际已被 `std::max` 保护）改为 `hough_min_inlier_ratio`（`line_detector.cpp:87`），且**所有 Hough 相关用例必须显式 `enable_hough=true`**，否则假绿。
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

- push CI（origin/main `66a8d26`）：最后一次 push CI 事件为 `38258621269`（cancelled，非失败）→ `38257427111` / `38244570254` / `38243907339` / `38241684733` success。**10-13 起再无新 CI run（含本轮实测至 10-18T02:19Z）**——因本地 6 笔提交未 push、origin 无新提交，属预期而非异常。本地基线：`1687e2a` 领先 origin/main `66a8d26` **6 个提交**（`c0652e1` docs TEST_PERF + `3208ae4` docs 索引 + `2ca0556` plan-20261013 + `33d852f` plan-20261015 + `f704480` plan-20261016 + `1687e2a` plan-20261017），均 docs/.agent-only；push 后预期一次全量 CI ≈14min，可作 #63 落地前**旧配置的最后基线**，但 push 决策现为 **D-010 pending（第 3 天）**，用户答复前不推。
- **三种失败模式与应对认知维持有效**（本轮未触发任何一种）：① 真污染（新引入失败需排查定位）；② 端点未收敛（`--no-daemon` / 发现收敛家族）；③ ros2-apt-source 404（环境准备阶段间歇性 curl 404，`gh run rerun <id> --failed` 即可）。
- push CI 近况：`38244570254`（`9cbda10` chore, success）→ `38243907339`（`8c7db61` chore, success）→ `38241684733`（`4d795d9` #61+格式修复, success）→ `38241256078`（`7e5794d`, failure，唯一原因 #60 遗留格式问题）→ `38151457809`（`39a40a6`, failure，同因）。
- Nightly：**10-17 success（`created_at 2026-10-17T19:59:13Z`）为最近一次**；10-18 本轮实测（02:19Z）尚未到调度点（约 20:00–22:00Z），属正常未到点，下一轮入账核对。**自 10-12 起连绿 6 次**（10-12 / 10-13 ×2 / 10-14 / 10-15 / 10-16 / 10-17），head **均为 `66a8d26`**（= origin/main，不含本地未 push 提交，见文首口径提示）。10-11 `39a40a6` failure 为已修的格式单点，非内容回归。
- 结论：**origin 主干全绿且稳定性提升**（Nightly 六连绿）；本地未 push 的 6 提交均为 docs/.agent-only，不携带回归风险，但也**未被 CI 验证过**——真正的风险不是回归，而是**单点丢失**（只存在于这台机器）。这是 D-010 默认方案取"推送"的主因。

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

本轮复查：被阻塞 8 项的 `updatedAt` 停在 10-05/10-09（`gh` 实测，无新评论无状态变化），**阻塞项均无新进展**，维持原判。#25/#26/#27（needs-info）待 #17①② / #18 证据，D-007-B 已生效。#63–#76（ready-for-human）为人类工作，不计入饥饿规则，不进 Executor 队列。

## 八、下一阶段（待输入，非当前阻断）

- #62 交付（直线拟合核下沉 + **`hough_min_inlier_ratio` 参数守卫（`line_detector.cpp:87`）** + `hough_rho_resolution` 静默退化收敛 + RANSAC 确定性种子；生产默认随机性不得改变，Issue 已写明）。
- **#62 落地后即进入维持模式（D-009，用户 2026-10-12 亲选 Q1-C）**：不再为找活而扫描，不再复问外部输入时间表；每轮只做 CI/Nightly 健康看护 + 已知阻塞的零成本复查（`updatedAt` 有变才深入），无事则空转收尾。任一外部输入实际到位即触发返工规划，不受本条限制。
- **本轮新增的前置动作**：#62 是 6 天来唯一 ready 项，且其立项依据刚被实测更正。重启执行轮时**建议先只跑验收标准第 4 条（先红后绿验证）**——若能复现 `line_detector.cpp:87` 的 UBSan 报错，说明任务真实可做；若复现不出来，说明 Planner 的扫描方法本身需要修正，后续 Issue 的立项标准都要重写。
- S3 性能评估（#19 B）：需实车/数据输入。
- 展示工作：D-007(B) 的最小展示（#57）与文档同步（#58）已交付关闭；完整展示（#26：视频/bag/release）与博客（#25）待 #17①② / #18 证据，到位后返工更新 demo 产物（该返工风险已由用户接受）。

## 九、已明确放弃 / 不做

- 调参腿已收口（docs §11.3）：无新维度动机且未按新 round 预注册前，不再扫描；**不冻结参数**（D-003）。
- 不从运动学模型推出轮胎极限结论（#17 边界）。
- 不用仿真证据关闭实车依赖 issue（D-004）。
- **ADR 下沉候选现状（2026-10-18 更新）**：velocity_profiler 梯形规划（#24）、safety_monitor 看门狗（#50）、lidar_cluster 评分（#59）、skidpad 回退中心线（#60）、skidpad 圆路径（#61）——**全部完成**；straight_line_planner `ClusterCones` 已由 `cone_boundary.h` 下沉（test_cone_boundary 覆盖）；**直线拟合核（Hough/RANSAC/LSQ/时序）——已立 #62，并于 2026-10-18 更正其技术前提**（原"NaN 传播致 float→int UB"经实测**不成立**：`std::max(0.05, NaN)` 返回 `0.05`；真实 UB 在 `line_detector.cpp:87` 的 `hough_min_inlier_ratio`，且需显式 `enable_hough=true` 才可达）；**Way/delaunay/kdtree 链——暂缓**（体量 ~1000 行 + API 纠缠，拆分即接口改造，超 P3 测试 Issue 承载；将来啃则 KDTree+Point/Circle 先行、Way 后行）。**ADR 正文的"后续候选"清单待 #62 落地后再同步一次**（Executor 顺带改，与 #59–#61 同一模式）。
- **KD-tree 最近邻替换暴力扫描的平局语义（2026-10-10 定，D-008，#60 已落实）**：不再复议。
- **"统一发现收敛等待策略"（2026-10-08 评估后不立项，维持）**：触发条件不变（任一运行时门禁再现"空表 / pub-sub=0"类假红则立项）。
- **clang-format 18（CI）/21（本机）skew（2026-10-12 观察，非决策）**：全树双版本扫过，仅 #60 一处分歧并已修复；若后续高频再现，再评估 pin CI 大版本（属 CI 流程变更，需用户决定，本轮不动 ci.yml、不提问）。
- **config 参数守卫类（2026-10-12 扫描结论，不立项）**：307 处 `declare_parameter`，#61 模式已覆盖其最高价值实例（UB 类）；其余多为范围/类型校验，无 sanitizer 可证伪性，不值得单独立项。

## 十、Decision Gate 状态与给 Executor 的指令

- **✅ D-010 已关闭（2026-10-18，用户亲选 B）**——本地 7 笔 docs/.agent-only 提交**已推送**（`66a8d26..c2978bd`），本地与 origin 重新 0/0。**当前无 pending 决策。** 预期触发一次全量 CI ≈14min（#63 落地前的旧配置基线），下一轮入账核对。
- **无阻断性 Decision Gate**；D-009 维持模式生效中，但 **#62 未交付前不进入空转**（维持模式的"每轮只做健康看护"是从 #62 交付之后才起算）。
- 下一棒 Executor 建议顺序：**#62**（单项队列；**先读正文 §当前问题 顶部的 Planner 更正块**——被测 UB 已改为 `line_detector.cpp:87` 的 `hough_min_inlier_ratio`，且所有 Hough 用例必须显式 `enable_hough=true`；RANSAC 种子方案选风险最小者并记录；生产默认随机性不得改变；ADR 正文后续候选清单顺带同步）。
- ⚠️ **循环连续第 6 天没有转——本轮再次如实上报**：#62 自 10-12 立为 ready 后**连续 6 天零 comment、未被领取**；10-13→10-17 各轮规划均无 Executor 活动。这不是任务问题（#62 过 4.2 门禁，本轮还替它补正了失实的技术前提），而是**执行轮没被发起来**。
- 📣 **新增长期职责（D-012，用户 2026-10-18 亲选）**：用户选择**维持每日循环**，并明确要求 **"下次会提醒发起 Execution"**。故自本条起，**每轮汇报只要 ready 队列非空且该任务已超过一轮未被领取，Planner 必须在摘要中显式提醒用户发起 Execution 会话**——这是 D-012 下的强制汇报项，不是可选项。
- **本轮新增的可执行性判断**：#62 的复核结果说明队列质量并非"无人可做"，而是**任务的立项依据需要被实测验证**。若用户打算重启执行轮，建议 Executor 领取 #62 时**先跑一遍更正后的验收标准 4（确认能先红后绿）**，这本身就是对"Planner 扫描结论是否可信"的第一次实测检验。
- **push 已完成**：本地与 origin/main 同步（0/0），Executor 可直接在 `c2978bd` 基线上开工 #62，无分叉、无 rebase 需求（外部 `M AGENTS.md` 脏改动不再阻塞，因已无落后提交）。**后续 `.agent/` 提交沿用同一节奏即可正常 push。**
- 展示范围（#25/#26/#27）：D-007(B) 已生效；#26 仍为完整展示父跟踪（needs-info）。
- Executor 未在运行（无 LOCK）；外部 `M AGENTS.md` 脏改动非本体系产生，任何一棒都不碰、也不代提交（只读记录）。
- **日报义务（`.agent/DIARY/` 已建）**：Executor 每轮收尾须写 `.agent/DIARY/YYYY-MM-DD.md`（一天一条、追加不覆写），这是次日 Planner 验收的唯一输入；漏写会直接打断跨天链路。本日至本轮检查时尚无 Executor 轮，无日报文件属预期（零产出即零报告义务）。
