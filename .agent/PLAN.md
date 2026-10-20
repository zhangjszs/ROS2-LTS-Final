# PLAN.md — 路线、当前阶段与可执行队列

> 维护者：Planning Agent（时间流接力）｜最后更新：2026-10-21（`plan-20261021` 轮：**验收关闭 #78/#79/#80/#81/#82 五单 ⇒ Milestone S2a「直线加速事件闭环」达成**；裁 D-018（`acceleration_v1` 口径变更→D-003 传导）；新建 pending D-019（S2a 后方向）；daemon 回填并入 #69）
> 事实基线：`main` = `761817e` = `origin/main`（0/0 同步）；本轮为**纯验收轮，零代码提交**；工作区仅外部 `M AGENTS.md`（非本体系产生，不碰、不代提交）。本轮 `.agent/` 提交后 push 至 origin。环境事实见 `.agent/ENV.md`（D-017 口径）。
> 文档体量约定：本文件只写**当前**状态与结论；历史以 `git log -- .agent/PLAN.md` 与各 Issue 评论为准。

## 〇、本轮结论：Milestone S2a「直线加速事件闭环」达成

本轮（`plan-20261021`）验收昨日 5 单并全部关闭 ⇒ **S2a 四条完成判据全部达成**（独立复核见 §三）：

| 判据 | 承接单 | 结论 |
| --- | --- | --- |
| 离线 benchmark 完赛门禁（finish/runout + 停稳） | **#81**（元数据）+ **#80**（判据核） | ✓ 达成 |
| ROS 多节点闭环完赛门禁（finish 事件恰好一次 + 锁存停 + accel-finish 硬门） | **#82** | ✓ 达成（含对旧 `blocked` 误判的更正） |
| 可证伪性（负样本 + 对照臂） | **#80**（`--no-post-finish-braking`）+ **#82**（`ENABLE_FINISH_EVENT=0`、`MAX_V=20`） | ✓ 达成 |
| 定位/感知加固收尾（纯 std 核 + UB 守卫） | **#79** + **#78** | ✓ 达成 |

⇒ 主线切换后的第一个端到端里程碑（D-014 直线加速）完成：仿真「感知→规划→控制→仿真」闭环 + 可证伪门禁均已落 main 且 CI 全绿。

**本轮 Planner 决策/治理**：裁 **D-018**（#80 改变 `acceleration_v1` 比较集口径，客观触及 D-003 冻结——结论：今日不重声明调参轮次（无活动轮、参数未冻结），转为「下一轮声明时须预注册声明此口径断裂」的长期约束）；daemon 预热回填 5 脚本并入 **#69**（ready-for-human）；新建 **pending D-019**（S2a 后方向，默认 B = 完成 P3 积压后进入 D-009 维持模式）。

## 一、当前阶段

路线图来源：#20「运行基线 → 可信仿真 → 低速闭环 → 性能优化」（S0–S3）。

- **S0 可重复启动：完成**（#13/#14 已关闭；CI/lint/headless/qos 门常绿）。
- **S1 可信接口与仿真：软件侧基本完成**，剩余卡仓库外输入（外部输入未到位，状态不变）：
  - #15 共用执行器适配层：软件侧已交付；剩余 = VCU 协议资料 + 台架标定实值（D-016 后性质确定）。
  - #16 任务/安全状态与最终仲裁：软件侧闭环；剩余 = 硬件急停链路 + 实车停车预算。
  - #17 可复现仿真/KPI/回归：③④已合入并进 CI 硬门；①②受实车标定与感知扰动数据阻塞。
  - #19 Pure Pursuit 低速基线：B 完整公平对比依赖 #17 扰动场景。
  - #18 rosbag 数据集与感知质量评估：需真实 bag。
  - #24 跨版本差分回放：需 ROS1 运行时（源码对照已由 #77 交付）。
- **S2a 直线加速事件闭环：✅ 完成（2026-10-21，4/4 判据达成）**——#81/#80/#82/#79/#78 五单验收关闭，见 §二/§三。
- **S2b 八字绕环 / S3 高速与性能**：待 S2a 闭环后按新证据评估（D-014、#19 B 仍依赖 S2b 条件）。
- **展示（D-007-B）**：最小展示入口 #57、文档同步 #58 已关闭；完整展示（#26）与博客（#25）待 #17①② / #18 证据。
- **ADR 下沉线：全部完成**——#59/#60/#61/#62 均关闭，无剩余高价值候选（KD-tree/Way/delaunay 链维持暂缓，见 §九）。

## 二、Milestone S2a「直线加速事件闭环」：✅ 已达成（2026-10-21）

- **目标**：在可复现仿真中完成直线加速闭环——车辆从起点加速、跨越 x=75 m 终点线、在 x≤100 m 的 25 m 缓冲区内停稳；**离线 benchmark** 与 **ROS 多节点闭环**两层输出都可判定「是否过线、何时过线、停在哪、是否停稳」。
- **现状（已达成）**：#81 已将 finish_x/runout 进元数据；#80 补 `finish_judge.hpp` 越线判定+缓冲停车，`acceleration` 基线重录 `stopped=true`；#82 补 ROS 闭环 `accel-finish` 硬门。原「跑完≠完赛」语义缺口已消除。
- **完成判据（四条，本轮已全部独立复核为真 ✅）**：
  1. ✅ #81 元数据 + #80 离线判据：`acceleration` 基线重录 `stopped=true`/`finish_crossed=true`/`stop_position_m=97.12≤100`/新增 `finish_time_s`+`stop_distance_m`；门禁第 9 节字段+时间线断言；core_standalone 32/32 亲验（`test_finish_judge`）。
  2. ✅ #82 ROS 闭环：`finish_event_node` 事件恰好 1 次 @x=75.0 + 锁存停 + `stop_position_m=84.7≤100`+`v_final=0`；纳入 drive_gates 为 `accel-finish` 硬门。
  3. ✅ 可证伪：离线 `--no-post-finish-braking` rc=1；闭环 `ENABLE_FINISH_EVENT=0` rc=1（原因逐字「未停车/越过缓冲区」）；`MAX_V=20` 对照臂证明绿不依赖限速（D-011 口径）。
  4. ✅ #79（GeodeticToEnu 纯 std 核，位模式零差异）/ #78（hough ratio 上界守卫，先红后绿）关闭。
- **非目标**：不宣称实车能力（D-004）；不引入 ESKF/SLAM/GTSAM（D-015）；不改控制律/调参（D-003 冻结）；不做八字/高速（D-014）；不改契约话题与 QoS（D-002）。
- **Issue 台账**（本 `gh` 实测，**28 open**）：ready-for-agent **3**（#83/#84/#85，均 P3 积压）· in-review 0 · in-progress 0 · 仓库外阻塞 **7**（#15–#19 / #24 / #37）· needs-info **3**（#25/#26/#27）· ready-for-human 14（#63–#76，不计饥饿规则）· 伞形 1（#20，优先级豁免见 §四）。**本轮关闭 #78/#79/#80/#81/#82 五单（净 −5）**。

## 三、验收结论（本轮：5 单全部通过）

昨日 in-review 五单逐条独立复核（`git show` + `core_standalone_check.sh` 32/32 + `build/drive_gates/last.json` 直读），**全部 pass 并关闭**：

- **#81 → pass 关闭**（comment `6296130011`）：`git show --stat 1aae347` 未含 `benchmarks/`/CSV 几何；core_standalone `test_track_generator` 亲验通过；消费方经元数据取 75/100。
- **#80 → pass 关闭**（comment `6296132845`）：`git show 3870249` 亲验 baseline `stopped` false→true + 4 完赛字段；`--no-post-finish-braking` rc=1；skidpad/trackdrive 基线最近改动为旧提交 `afbfa65`（本轮未触碰）；`test_finish_judge` 亲验通过；未含 config/control/launch。**遗留 → D-018**（口径变更触及 D-003 冻结）。
- **#78 → pass 关闭**（comment `6296135114`）：`git show f56ff36` 亲验守卫扩至 `ratio>1.0`；core_standalone `test_line_fit_core` 在 float-cast-overflow 下亲验通过；先红后绿 + `[0,1]` 黄金逐字不变。
- **#79 → pass 关闭**（comment `6296137702`）：`git show 0654d0e:geodetic_core.hpp` grep ROS 依赖 = 0；core_standalone `test_geodetic_core` 亲验通过；位模式差分 `TOTAL_MISMATCH=0`；未含 config/launch。
- **#82 → pass 关闭**（comment `6296139845`）：`build/drive_gates/accel-finish.log` 直读——正样本 rc=0 + `ENABLE_FINISH_EVENT=0` rc=1（可证伪）+ `MAX_V=20` 对照臂；完整 drive_gates 12 门全 PASS/440s；未含契约/QoS/控制律。
  - **验收同时确认 #82 的「误判更正」**：会话 2 曾交失实 `blocked` 报告（"本机 DDS 不可用"），会话 3 查明真因为 ros2cli daemon 未显式启动。本轮以 closed-loop/closed-loop-fault/fault-injection 三门留痕均 PASS 独立验证更正为真——本机 runtime 门禁**可用**。§3.4「报告失实须处理」的正向案例。
- **报告质量**：五单报告均属实，无「报告失实」需回滚者；#82 为 Executor 自我更正前轮失实结论，处理规范。

## 四、当前 ready 队列（今日：P3 积压清理，2026-10-21）

主线 S2a 已闭环，ready-for-agent 仅剩 3 个 P3 积压，做完即见底（下一步见 pending D-019）。

1. **#83**（P3）`hough_threshold` 死参数：三处声明但算法未读（实测 0/1000 输出逐位相同）。推荐方案 A（删除使暴露面=实现）；硬约束默认输出逐字不变。落点 `src/planning/straight_line_planner/`。
2. **#84**（P3）`temporal_filter_*`/`kCenterMargin` NaN/越界静默失效（实测 alpha=NaN→NaN 且 valid=true）：核心边界加参数清洗，非法值定义明确，不得以 valid=true 返回 NaN 线、不得静默清空锥桶。落点同上。
3. **#85**（P3）MIGRATION_BASELINE 回填 D-014/15/16 引用 + `docs/README.md` 文档索引挂入 5 篇；仅 docs/。

- ⚠ **文件冲突**：#83 与 #84 同处 `line_fit_core.hpp`/`test_line_fit_core.cpp`，且该文件已含 #78 落地的 `hough_min_inlier_ratio>1.0` 守卫——**不得覆盖**；两单**必须串行**（先其一 merge + `git pull --rebase` 再其二），不可并行开分支。#85 纯 docs 可任意时机。
- 队列 = 3 项（2–5 区间内）。#20 优先级豁免维持（伞形跟踪，无执行体）。

## 五、已完成（软件侧摘要）

- 已关闭：S0 全部（#13/#14）、#22、#30–#54、#56、#57、#58、#59、#60、#61、#62、#77、#23、**#78、#79、#80、#81、#82（S2a 五单，2026-10-21 验收关闭）**。
- 近期：**#81（`1aae347`/merge `0f48b3a`）、#80（`3870249`/merge `126cc83`）、#78（`f56ff36`/merge `a71b548`）、#79（`0654d0e`/merge `ea5e19b`）、#82（`343ecc3`+`40673b0`/merge `e5f9998`）——2026-10-21 本轮独立复核（core_standalone 32/32 + drive_gates 12 门全绿）后关闭，达成 S2a 里程碑**；更早 #77/#62（2026-10-20）。
- #57 最小展示入口（`c79b71d`/merge `97de15d`）、#58 CLAUDE.md 冒烟清单（`0f87d25`/merge `6b5fc27`）。
- 10-12/13 用户驱动测试性能轮：`docs/TEST_PERF.md` + #63–#76（14 项 `ready-for-human`）。

## 六、CI 状态（本轮核实）

- **origin/main `761817e` 全绿**：本轮为纯验收轮、**零代码提交**（不触发新 CI）。S2a 五单代码提交 CI 已复核全 success：#78 `39422351603`·#79 `39423649207`·#80 `39416500750`·#82 merge `39441369149`/尾 `39453577195`；#81 merge `0f48b3a` 亦 success。
- **Nightly**：主干维持连绿；本轮未跑 gh run 清单，下轮入账核对最新调度点结果。
- 三种失败模式认知维持有效：① 真污染；② 端点未收敛（勿用 `--no-daemon`，D-005）；③ ros2-apt-source 404（`gh run rerun <id> --failed`）。

## 七、已知阻塞与原因（均有证据，D-013 口径逐项复验）

| Issue | 阻塞原因（D-013 分类） | 本轮状态（2026-10-21） |
|---|---|---|
| #15（P1） | **资料缺失**：VCU 协议文档、台架标定实值（D-016 已确认非外部代码缺失） | `updatedAt` 10-19；无新外部输入，仍不可执行 |
| #16（P1） | **资料缺失**：硬件急停链路、实车停车预算 | 无变化（10-05） |
| #17（P1） | **资料缺失**：模型速度范围实测、感知扰动量化 | 无变化（10-05） |
| #19（P1） | **资料缺失**：依赖 #17 扰动场景 | 无变化（10-05） |
| #18（P2） | **资料缺失**：真实 bag | 无变化（10-05） |
| #24（P1） | **运行时缺失**：剩余验收需 ROS1 运行环境（本机无 Noetic、无 docker）；源码对照切片 #79 已交付关闭 | `updatedAt` 10-19；第 1 条已由 #79 关闭，本体仍卡运行时 |
| #37（P3，needs-triage+blocked） | **资料缺失**：需 #15 标定完成后翻转 `use_arbiter` 默认 | 无变化（10-09） |

> D-013 复查：本轮逐单核对 28 个 open Issue 的 `updatedAt`，除 #69 因 Planner 治理评论更新外，**无任何外部输入到位**，7 个 blocked 全部维持。另：#25/#26/#27（needs-info）待 #17①② / #18 证据，D-007-B 生效中；#63–#76 为 `ready-for-human`，不计饥饿规则、不进 Executor 队列。

## 八、下一阶段

- **✅ S2a 已闭环（2026-10-21）**：#81→#80→#82 主链 + #79/#78 加固全部验收关闭，四层判据（元数据 / 离线 / ROS 闭环 / 可证伪）达成。
- **今日（P3 积压）**：#83/#84（straight_line_planner 死参数与 NaN 守卫，**同文件族须串行、勿覆盖 #78 守卫**）、#85（迁移文档回填）；做完 ready-for-agent 见底。
- **下一步方向 = pending D-019**（默认 B）：完成 P3 后进入 D-009 维持模式——每轮只做 CI/Nightly 看护 + blocked 零成本复查（D-013 口径），不为找活而扫描。
- **八字/高速（S2b/S3）暂不进 ready**：D-014 已定 S2a 闭环后评估八字，但其定位精度要求高，D-015 暂缓 ESKF/SLAM、#18 真实 bag 未到位前无法验收（D-004）；强推属过度工程，需先解约束或经用户重新定义范围（见 D-019 选项 A）。
- **外部输入到位即返工**：任一 `blocked` Issue 出现协议/标定/bag 实据（`updatedAt`/新评论），即触发该线重新规划，不受 D-009 维持模式限制。

## 九、已明确放弃 / 不做

- 调参腿已收口（docs §11.3）：无新维度动机且未按新 round 预注册前不再扫描；**不冻结参数**（D-003）。
- 不从运动学模型推出轮胎极限结论（#17 边界）。
- 不用仿真证据关闭实车依赖 issue（D-004）。
- **ADR 下沉候选：全部完成**（velocity_profiler / safety_monitor 看门狗 / lidar_cluster / skidpad 两条路径 / 直线拟合核）；**Way/delaunay/kdtree 链暂缓**（~1000 行 + API 纠缠，拆分即接口改造，超 P3 承载）。
- KD-tree 平局语义：已由 D-008 定，不复议。
- "统一发现收敛等待策略"：不立项，触发条件不变（任一运行时门禁再现"空表 / pub-sub=0"类假红则立项）。
- clang-format 18(CI)/21(本机) skew：全树双版本扫过仅一处分歧且已修；若高频再现再评估 pin CI 大版本（属 CI 变更，需用户决定）。
- config 参数守卫类：307 处 `declare_parameter`；本轮按用户要求扩充队列时，新增了**有实测证据**的两个实例——#83（`hough_threshold` 声明但未读取）与 #84（NaN/越界导致静默失效或 NaN 输出）。其余无 sanitizer/无行为证据的守卫仍不单独立项。
- **不把 `FSAC/2024` 当作 #23/#24 的对照基线**（2026-10-19 实测：该仓无 `HUAT_PathLimits.msg`、无 `vehicle_interface_*` 包；对照基线一直是 `zhangjszs/ROS1-LTS-Final` = `FSAC/2025`）。
- **不引入复杂定位后端**（D-015）：ESKF / LocationMapper / GTSAM 因子图在真实 bag（#18）到位前不进 `ready`。
- **底盘“自建”不等于提前关闭 #15**（D-016）：自建是交付归属，但 VCU 协议与台架标定实值仍是关闭前置；不以“自建”名义绕过 D-004。

## 十、Decision Gate 状态与给 Executor 的指令

- **Decision Gate**：本轮裁 **D-018**（`acceleration_v1` 口径变更→D-003 传导，Planner 自决）；新建 **pending D-019**（S2a 后方向，默认 B）。D-014/D-015/D-016 用户亲选、D-017 Planner 自定、D-009/D-012 生效中。
- **给 Executor 的顺序**：**(#83 → #84，严格串行同文件族) + #85**（可并行/任意时机）；三单均不需外部输入、不触 1.10 红线。基线 `main`=`origin/main`（0/0），无 rebase 需求。**清理遗留**：远程分支 `origin/agent/issue-81-track-metadata`（#81 已合入却未删，违契约 1.9）请本轮顺带删除。
- 📣 **D-012 提醒项**：ready 队列（#83/#84/#85）为非空且自 10-20 立项后尚未被任何执行轮认领（昨日队列执行的是 S2a 五单）。**建议用户尽快发起 Execution 会话**（顺序见上）；若答复 pending D-019，可据此定下一 Milestone。
- 契约 1.10 红线本轮**均未触碰**：验收关闭 #78–#82 未改契约话题/QoS/认证/CI 流程（#82 仅复用既有 `/system/task_event` 与 `/system/stop`，经 `makeQoS` 构造）；D-018/D-019 属协议解释与方向询问，非红线变更。
- 外部 `M AGENTS.md` 脏改动非本体系产生：不碰、不代提交、不提问（只读记录）。
- **日报义务**：Executor 每轮收尾写 `.agent/DIARY/YYYY-MM-DD.md`（一天一条、追加不覆写、更正用新行追加）；这是次日验收唯一输入。10-20 日报已由上一执行轮产出并被本轮验收引用。
