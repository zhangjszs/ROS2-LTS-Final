# PLAN.md — 路线、当前阶段与可执行队列

> 维护者：Planning Agent（时间流接力）｜最后更新：2026-10-07T12:35Z（`plan-20261007T1235Z`）
> 事实基线：main `3a2e337`（与 origin/main 一致；远端无新提交，加外部未暂存 `M AGENTS.md` 未碰，故未执行 pull；`git fetch` 已同步）；无 LOCK（Executor 空闲）；环境事实见 `.agent/ENV.md`（PATH 条目）。

## 一、当前阶段

路线图来源：#20「ROS 2 车队开发路线：运行基线 → 可信仿真 → 低速闭环 → 性能优化」（S0–S3）。

- **S0 可重复启动：完成**（#13/#14 已关闭；CI/lint/headless/qos 门常绿）。
- **S1 可信接口与仿真 / S2 低速稳定闭环：进行中，软件侧基本完成，剩余卡仓库外输入**：
  - #15 共用执行器适配层：软件侧（codec/标定单一来源/停车帧工厂）已交付并审计；剩余 = VCU 协议资料 + 台架实车标定。
  - #16 任务/安全状态与最终仲裁：软件侧（状态机/信任门/新鲜度租约/故障注入矩阵）闭环；剩余 = 硬件急停链路 + 实车停车预算。
  - #17 可复现仿真/KPI/回归：③④已合入并进 CI 硬门；①②受实车标定与感知扰动数据阻塞。
  - #19 Pure Pursuit 低速基线：A 阶段基线 + B 骨架（诊断出口 / 调参-比较集分离）已在；B 完整公平对比依赖 #17 扰动场景。
  - #18 rosbag 数据集与感知质量评估：需真实 bag。
- **S3 性能优化**：#19 B 待 S2 条件满足。

## 二、当前 Milestone 目标与完成判据

- 目标（S1+S2）：接口/编码/单位/坐标系一致；固定场景可复现且 KPI 能识别失败；选定赛制低速闭环可重复完成，停车/失效/重启行为通过。
- 完成判据：以各 Issue 验收标准为准；其中依赖实车/台架/硬件的条目**不得以仅仿真证据关闭**（见 D-004）。

## 三、软件侧缺口扫描（本轮更新）

2026-10-02 版 PLAN 判定"软件侧已扫尽"。本轮核实 CI 证据时发现新缺口：

- **Nightly `drive_gates` 在 10-02/10-03 连续两次假失败**：`mpc-reject-smoke` 出口纯度预检报 `/vehicle_command 发布者数=未知（应为 1）`；10-04 与 10-05 在**同一代码树**（`64b5185`→`da869e5`，其间仅 `.agent/` chore 提交）上连续两晚全部 14 门通过（run `37233438190` / `37389535363`）。push CI 全绿 → 主干不红，属间歇性。
- 该预检（`scripts/mpc_reject_smoke.sh:115-124`）本身是 10-01 真污染假红（Nightly 36933923349）后由 `2c9a4e6` 加入的防线；现其自身在 `ros2 daemon stop` 后图重建未收敛即查询（12s 超时无退避），空返回/超时与真实多发布者（=2）被混为一谈。
- 已立 **#55**（P2, ready）：根因定位 + 预检退避重试，判据强度不变。
- 10-06 轮复查：间歇性假失败连续两晚未复现，但根因（`ros2 daemon stop` 后图发现未收敛即查询）仍在，硬门禁假红的误报成本未除 → #55 必要性与 P2 定级不变，验收标准（含"修复后 2 次 Nightly 全绿"）不变。
- 10-06 04:21Z 轮复查（距上一轮仅 ~14 分钟）：`in-review` / `in-progress` 均为 0，无新增 Issue/PR；open Issue 逐一过 4.1 清单无变化（blocked 7 项、needs-info 3 项、#37 needs-triage P3 均维持）；#55 重过 4.2 六条门禁仍全满足 → ready 队列维持 1 项，无灌水新增。
- 10-06 05:05Z 轮（Executor `exec-20261006T0625Z` 已交付）：#55 代码完成并合入（`a99b600`→merge `48a4241`，分支已删），Planning 独立复核后判定**部分通过**（标准1/2/3 满足：根因评论、退避重试+三分叉文本、两臂 10/10；标准4 缺 drive_gates 全绿——唯一红门 cpp20 系本机缺 ccache，stash 对照基线即红；标准5 待合入后 2 次 Nightly，属时间性未达）。#55 保持 `in-review` 不关闭、不打回（无代码返工项），关闭条件：post-fix 2 次 Nightly 全绿 + #56 关闭。push CI run `37415507053`（`48a4241`）success，主干不红。
- **#56**（auto-discovered，本轮已定级 P3/ready）：本机缺 ccache 致 drive_gates cpp20 门必红（`which ccache mold` 本轮复核仍空；CI 不受影响）。范围限定为 apt 单包安装 + 验证，不改脚本（D-005）；反向阻塞 #55 标准4。daemon 瞬态卡死（单次自愈）先观察，不单独立项。
- 10-06 08:03Z 轮复查（距 05:05Z 轮约 3h）：`gh issue list` 确认 open 集合无变化（blocked 7 项、needs-info 3 项、#37 needs-triage P3 维持；无新增 Issue/PR）；#55 仍 `in-review`（post-fix Nightly 尚未产生，见 §六）、#56 仍 ready（P3 定级与 apt 单包授权维持，不推翻——单包低风险可逆 + 无 sudo 则转 blocked 回退，D-005 未违反）；ready 队列维持 1 项，无灌水新增。
- 10-06 23:29Z Executor 收尾（`exec-20261006T2329Z`，`3e0468f`）：接管过期 LOCK 后 #56 三验收全绿 → 转 `in-review`；关键纠正——mold 缺失系 PATH 误判（`~/.local/bin/mold` 2.42.0 本体一直在，login 等效 PATH 即解），未装包、未改脚本、未清缓存；根因记入 ENV.md。
- **10-07 00:05Z 本轮（Planning 验收）**：#56 **验收通过并关闭**（独立复核：`which ccache/mold` 双命中；`check_cpp20.sh` 独立重跑 rc=0，19 pkgs；`build/drive_gates/last.json` 23:30Z 机器产物 11 门全 PASS hard_fail=0，代码树其后零变更故证据有效；报告"12 门"与文件"11 门"系 SKIP_BUILD 计数口径差，不影响结论）。#55 标准4 同步满足（同一 drive_gates 实跑，mpc-reject-smoke PASS）；标准5 达 1/2（post-fix 首个 Nightly `37539011241` success，14 门全绿；第 2 个观察点约 10-07 20:xxZ）。open 集合复查：除 #56 关闭外无变化（blocked 7 项、needs-info 3 项、#37 P3 维持；无新增 Issue/PR；P0 为 0）。外部未暂存 `M AGENTS.md`（ enriching 改写，44+/3-）非本体系产生，全程未碰；`git pull --rebase` 因此跳过，main 与 origin/main 仍一致。
- **10-07 本轮加时（用户驱动规划）**：用户质疑 ready 归零 → 加做软件侧缺口扫描（`src/scripts/tests/config` TODO 全仓 grep 仅 Doxyfile 模板字样；post-fix Nightly 日志关键字扫 warn/skip/retry/timeout/daemon 仅第三方噪音、无 daemon 异常；blocked 7 项确系仓库外输入）——无新可验证缺口，不灌水。同时用户三项亲定：① **D-007 推翻 A→B**（现在做最小展示，接受返工风险）：B 子集单立 **#57**（P2, ready-for-agent：`scripts/demo.sh` + README 展示小节，仅仿真证据；视频/大 bag/release/博客正文明确排除），#26 留作完整展示父跟踪（仍 needs-info，已留言），#25 博客保持 needs-info；② 仓库外输入暂无到位，blocked 维持；③ 外部 `M AGENTS.md` 保留并忽略（任何一棒不碰）。
- **10-07 12:35Z 本轮复查**：`gh run list` 确认 post-fix 第 2 个 Nightly 观察点尚未产生（最新仍为 `37539011241` 10-06T22:11Z success；其后仅 push CI `37572989939` / `37573490832` 均 success，无 in-progress run）→ #55 标准5 仍 1/2，保持 `in-review`，无新证据故不在 Issue 留言（沿 10-06 08:03Z 轮先例，避免无实质更新刷屏）。open 集合除 #56 已关闭外无变化（blocked 7 项、needs-info 3 项、#37 P3 维持；无新增 Issue/PR；P0 为 0）。#57 重过 4.2 六条门禁仍全满足（目标/范围/无依赖/D-007-B 已决/验收可判定/不触红线——Issue 正文含"不接入 ci.yml、不动 benchmarks"）→ ready 队列维持 1 项，无灌水新增。STATE.md 称"ready 队列空"已过期（早于 #57 立项），以本 PLAN 为准，不改写 Executor 文件。

## 四、当前 ready 队列（给 Executor 的建议顺序）

1. **#57** 提供最小可复现展示入口：demo.sh 一键正常/故障仿真演示 + README 展示小节（P2，`ready-for-agent`；D-007-B 用户亲定；范围：新增 demo.sh + README 小节，不碰 CI workflow；视频/bag/release/博客明确排除）

- #55 为 `in-review`（仅剩时间证据，无代码返工项，不在执行队列）。#57 已过 4.2 六条门禁：目标/范围（含不包含）明确、无依赖、无未决取舍（用户已定 B）、验收逐条可判定、不触红线（不改 CI 结构、不动 benchmarks）。#26 留作完整展示父跟踪（仍 needs-info），#25 博客保持 needs-info。

## 五、已完成（软件侧摘要）

- 已关闭：S0 全部（#13/#14）、#22/#30/#31/#32/#33/#34/#35/#36/#38/#39/#40/#41/#42/#43/#44/#45/#46/#47/#48/#49/#50/#51/#52/#53/#54 等。
- #17 ③④、#15/#16/#19 软件侧、#24 合成夹具已合入并附证据（见各 issue 评论与 git 历史）。
- **2026-10-02 ready 批（软件侧最后一批）**：#53（bicycle_model 纯 std 单测，`eb75d08`）、#52（`warm_start` 死配置生效，`fe2d97b`）、#54（sensor_simulator 感知核下沉纯 std core，`dbb77c0`）——均经 Planning 独立复核后 CLOSED（CI：run 37025171399 / 37026081026 success）。
- **#56**（本机 ccache/mold 环境缺口，实为 PATH 误判）：零代码变更，Planning 独立复核（`which` 双命中 + `check_cpp20.sh` 重跑 rc=0 + `last.json` 11 门全 PASS）后本轮 CLOSED。

## 六、CI 状态（本轮核实）

- push CI（main 当前 `3a2e337`）：run `37573490832` **success**（加时轮 `.agent` 提交：立 #57 + D-007→B）；前一 `.agent` 提交 run `37572989939` success（验收关闭 #56 那轮）。其间 `37437488247` failure 为纯 `.agent` chore 提交的偶发红（26s 即挂，非代码回归，其后 success 已覆盖）——主干不红。
- Nightly 时间线：09-25–09-30 全绿 → 10-01 `36933923349` 假失败（真污染模式，`2c9a4e6` 修复）→ 10-02 `37068578710` / 10-03 `37151769248` 假失败（"发布者数=未知"新模式）→ 10-04 `37233438190` / 10-05 `37389535363` 连续两晚全绿；**post-fix 首个 Nightly `37539011241`（10-06T22:11Z，sha `bda2d90` 含 `48a4241` 修复）success——14 门全 PASS 含 mpc-reject-smoke（48s），为 #55 标准5 的第 1/2 个观察点**；第 2 个观察点为下一次 schedule（约 10-07 20:xxZ）。
- 结论：主干不红（#55 改动经 push CI + post-fix Nightly 首绿验证未引入回归）；#55 关闭待第 2 次 post-fix Nightly。

## 七、已知阻塞与原因（均有证据；本轮已补 `blocked` 标签与优先级）

| Issue | 阻塞原因（仓库外） | 证据 |
|---|---|---|
| #15（P1, blocked） | VCU 协议资料、台架标定实值 | #15 评论关闭条件；`docs/BENCH_CALIBRATION_TEMPLATE.md` |
| #16（P1, blocked） | 硬件急停链路、实车停车预算 | #16 评论关闭条件 |
| #17（P1, blocked） | 模型速度范围实测、感知扰动量化（数据 → #18） | #17 评论关闭条件 |
| #19（P1, blocked） | 依赖 #17 扰动场景 | #19 B 进入条件 |
| #18（P2, blocked） | 真实 bag | #18 依赖段 |
| #23（P1, blocked） | ROS1 环境 / 外部驱动确认 | #23 依赖段 |
| #24（P1, blocked） | 依赖 #23 + ROS1 环境 | #24 依赖段 |
| #25/#26/#27（needs-info） | 展示范围与人力（产品决策，见 D-007） | #27 总入口 |
| #37（needs-triage, P3） | 需 #15 实车标定后翻转默认 | #37 body（本轮三角定位评论） |
| #55（P2, in-review） | 仅剩 post-fix 第 2 次 Nightly 全绿（时间性，约 10-07 20:xxZ schedule） | #55 验收更新评论（本轮；标准1–4 已满足） |
| #56（P3，已关闭） | ——本轮验收通过关闭；PATH 误判根因记入 ENV.md | #56 验收评论 + `build/drive_gates/last.json` |

## 八、下一阶段（待输入，非当前阻断）

- S3 性能评估（#19 B）：需 ① 实车/数据输入（用户确认暂无到位，blocked 维持）。
- 展示工作：D-007 已由用户 10-07 推翻为 B——最小展示（#57，ready）先行；完整展示（#26，视频/bag/release）与博客（#25）仍待 #17①②/#18 证据，到位后返工更新 demo 产物（接受的风险）。
- #57 与 #55（待第 2 次 Nightly）均不阻塞对方，可并行。

## 九、已明确放弃 / 不做

- 调参腿已收口（docs §11.3）：无新维度动机且未按新 round 预注册前，不再扫描；**不冻结参数**（D-003）。
- 不从运动学模型推出轮胎极限结论（#17 边界）。
- 不用仿真证据关闭实车依赖 issue（D-004）。
- 不顺手做 ADR 其余下沉候选（velocity_profiler 梯形规划、safety_monitor 看门狗、lidar_cluster 几何核、skidpad `IcpApFPlanner::ClusterCones`）——未定级、未 ready。

## 十、Decision Gate 状态与给 Executor 的指令

- **当前无阻断性 Decision Gate；ready 队列 1 项（#57 P2，最小展示）。**
- 下一棒 Executor：领取 #57（新增 `scripts/demo.sh` 串联已有 smoke 脚本 + README 展示小节；禁碰 CI workflow；视频/bag/release/博客正文一律不做）；验证前记得先 `export PATH="$HOME/.local/bin:$PATH"`（ENV.md）。#55 无需动（待约 10-07 20:xxZ 的第 2 次 Nightly，有新证据就在 #55 留言，不得 close）。
- 展示范围（#25/#26/#27）：D-007 已默认转正（暂缓，见 DECISIONS），未决事项已消除，不进 ready。
- Executor 空闲（无 LOCK）；`in-review` 1 项（#55，标准1–4 满足、标准5 达 1/2，验收评论已留）。外部 `M AGENTS.md` 脏改动非本体系产生，任何一棒都不碰（Planning/Executor 均只读记录）。
