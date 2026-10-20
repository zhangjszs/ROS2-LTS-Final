# TASKS.md — Issue 本地镜像（权威：GitHub Issue；本文件由 Planner 每轮整体重写）

> 维护者：Planning Agent｜最后更新：2026-10-20（`plan-20261020` 轮后：**#77/#62 验收关闭、#23 关闭；主线切至 S2a 直线加速事件闭环**；随后按用户要求**扩充队列，新增 #81–#85**；共 **33 open**）
> 规则：本文件只读镜像，Executor 改状态时先改 Issue 再同步对应行；冲突以 Issue 为准。
> 日报：`.agent/DIARY/2026-10-19.md` 已由上一执行轮产出（本轮验收已引用）；10-20 尚无 Executor 轮。Executor 收尾务必写 `DIARY/YYYY-MM-DD.md`（一天一条、追加不覆写）。
> 🆕 **本轮变化**：用户三项决策落定——**D-014 直线加速 / D-015 暂缓复杂定位后端 / D-016 底盘自建**；D-017 明确支持环境（ROS2 Jazzy/lyrical，入口 `scripts/drive_gates.sh`）。#23 关闭，赛制与定位的不确定性消除，队列首次拥有**面向端到端闭环**的 P1 任务。随后按用户要求补充立项 **#81–#85**（赛道元数据、ROS 闭环、死参数、参数守卫、迁移文档回填）。
> 🔄 **Executor 轮（2026-10-20）状态更新**：**#81 已完成并转 `in-review`**（提交 `1aae347` / merge `0f48b3a`，已推送 origin/main；报告 comment `6289660599`）。

## 今日队列（2026-10-20，给 Executor 的建议顺序）

1. **#81** ✅ **已完成（转 `in-review`）**：赛道元数据补齐：acceleration 的 `finish_x=75` / `runout_end_m=100` 进入 TrackDefinition 与 `tracks.json`（P2）—— 提交 `1aae347` / merge `0f48b3a`；消费方改用 `track->finish_x` / `track->runout_end_m` / `hasFinish()`，不得新增硬编码。
2. **#80** 直线加速完赛判定：跨越 x=75 终点线 + 25 m 缓冲区（x≤100）内停稳 + benchmark JSON 字段与门禁（P1，`ready-for-agent`）——依赖 #81（已就位）；落点 `src/simulation/track_benchmark/`。必须包含**可证伪红/绿对照**（移除制动时门禁必须红），并重生成 `benchmarks/baseline/acceleration.acceleration-75m.v1.json`（现行基线 `stopped=false` 即缺陷证据）。依赖说明见 comment `issuecomment-6273468400`。
3. **#82** ROS 闭环直线加速完赛：过线发布 `finish` 任务事件 → 锁存停 → 25 m 缓冲区内停稳（P2，`ready-for-agent`）——依赖 #81/#80；复用既有 `/system/task_event` 与 `/system/stop` 契约，扩展 `closed_loop_sim_smoke.sh` 并纳入 `drive_gates.sh`，需可证伪开关。
4. **#79** `vehicle_state::GeodeticToEnu` 下沉纯 std core + 无 ROS 确定性夹具（P3，`ready-for-agent`）——#24 第 1 条的切片（comment `6273165395`）。要求行为逐位一致（差分用例），不改节点对外接口。
5. **#78** `hough_min_inlier_ratio` 超大有限值守卫（P3，`ready-for-agent`）——`line_fit_core.hpp:212` 的 `static_cast<int>(std::ceil(...))` 对 `1e300` 触发 UBSan float→int 溢出。**先红后绿**：先复现 `1e+301 is outside the range...` 再修，补 `1e300` 用例。

> 队列 = 5 项，在 2–5 区间上限。顺序由依赖决定：#81 → #80 → #82 是 S2a 主链；#79/#78 为并行加固。**#83/#84/#85 已就绪但不在今日队列**（P3 积压：#83 死参数、#84 参数 NaN 守卫、#85 迁移文档回填），待主链消化后由下一轮排入。

## ready（可执行，7）

| Issue | 标题 | 优先级 | 依赖 / 要点 |
| --- | --- | --- | --- |
| #80 | 直线加速完赛判定（终点线 + 缓冲区停车） | P1 | 依赖 #81（已就位）；benchmark 基线重生成；可证伪性为硬验收 |
| #82 | ROS 闭环直线加速完赛：finish 事件 → 锁存停 → 缓冲区停稳 | P2 | 依赖 #81/#80；复用既有 task_event/stop 契约 |
| #79 | vehicle_state GeodeticToEnu 纯 std 核 + 无 ROS 夹具 | P3 | 无；行为逐位一致；不得改接口契约 |
| #78 | Hough `hough_min_inlier_ratio` 超大有限值 UB 守卫 | P3 | 无；UBSan 先红后绿；`line_fit_core.hpp:212` |
| #83 | `hough_threshold` 死参数（节点/YAML 暴露但算法未使用） | P3 | 无；实测 0 与 1000 输出逐位相同；方案 A 删除 / B 接线 |
| #84 | temporal_filter_* 与 kCenterMargin 的 NaN/越界静默失效 | P3 | 无；实测 alpha=NaN 输出 NaN 且 valid=true |
| #85 | MIGRATION_BASELINE 回填 D-014/15/16 并挂入文档入口 | P3 | 无；仅 docs/；不改变结论边界 |

## in-review（待验收，1）

| Issue | 标题 | 提交 | 报告 |
| --- | --- | --- | --- |
| #81 | 赛道元数据补齐：finish_x / runout_end_m 进入 TrackDefinition 与 tracks.json | `1aae347` / merge `0f48b3a` | comment `6289660599` |

## in-progress（0）

（空）

## 仓库外阻塞（7）

| Issue | 标题 | 阻塞原因（D-013 分类） | 本轮复核 |
| --- | --- | --- | --- |
| #15 | 共用执行器适配层与车辆标定配置（P1） | **资料缺失**：VCU 协议文档、台架标定实值（D-016 已确认非外部代码缺失） | `updatedAt` 10-19（本轮记录 D-016）；性质澄清，仍不可执行 |
| #16 | 统一任务与安全状态、最终指令仲裁（P1） | **资料缺失**：硬件急停链路、实车停车预算 | `updatedAt` 10-05，无新评论 |
| #17 | 可复现仿真场景、可信 KPI 与自动回归（P1） | **资料缺失**：模型速度范围实测、感知扰动量化 | 同上 |
| #19 | Pure Pursuit 低速基线与 MPC 公平对比（P1） | **资料缺失**：依赖 #17 扰动场景 | 同上 |
| #18 | rosbag 数据集与感知质量评估（P2） | **资料缺失**：真实 bag | 同上 |
| #24 | 跨 ROS 版本语义差分回放与故障回归（P1） | **运行时缺失**：验收第 2 条需 ROS1 运行环境（本机无 Noetic、无 docker）；第 1 条已拆给 #79 | `updatedAt` 10-19（拆分记录） |
| #37 | 翻转 use_arbiter 默认（P3，needs-triage + blocked） | **资料缺失**：需 #15 标定完成 | `updatedAt` 10-09，无新评论 |

> D-009 返工触发本轮**未命中**（无外部输入到位）。D-013 复查进度：已覆盖 #23（→决策缺失，已关闭）、#24（运行时缺失）、#15（资料缺失）；**剩余 #16/#17/#18/#19/#37 每轮查 1–2 个**。

## needs-info / 待条件（3）

| Issue | 标题 | 等待 |
| --- | --- | --- |
| #25 | 技术博客系列（P2） | #17①② / #18 证据 |
| #26 | 完整项目展示（P2，父跟踪） | 同上 |
| #27 | 跨仓库路线总入口（P2） | 同上 |

## ready-for-human（14，用户跟进，无变化）

#63 CI 路径过滤与并发取消（P2）· #64 `check_cpp20.sh` 参数统一（P2）· #65 closed-loop-fault 六臂并行（P2）· #66 CI job 三段拆分（P2）· #67 `affected_gates.sh`（P3）· #68 ccache 键头文件通配（P3）· #69 DDS 环境性失败区分（P2）· #70 门禁时长预算监控（P3）· #71 core_standalone 覆盖率基线（P3）· #72 Prepare ROS 2 步骤超时（P3）· #73 评分核测试双轨去重（P3）· #74 本地快速反馈文档（P3）· #75 测试编写规范入库（P3）· #76 伞形：12 个仅 colcon 轨测试下沉（P3）

> #63 的「before」实测基线仍有效：纯 docs/.agent 推送在旧配置下跑满 842 s / 693 s（comment `issuecomment-6258394941`）。

## 路线图伞形（1，长期跟踪，无优先级标签系有意豁免）

| Issue | 标题 |
| --- | --- |
| #20 | ROS 2 车队开发路线：运行基线 → 可信仿真 → 低速闭环 → 性能优化 |

## 已关闭（近期备查）

- **#77**（2026-10-20 验收关闭，7/7；`815d292`/merge `ab1ad6e`；comment `6273158492`）
- **#62**（2026-10-20 验收关闭，9/9；`8169e86`/merge `3ca7930`；comment `6273160633`；残余 UB → #78）
- **#23**（2026-10-20 决策收口关闭；comment `6273231512`；缺口 → #78/#79/#80/#15/#17/#19）
- #59 / #60 / #61（2026-10-12）；#55 / #56 / #57 / #58。

## 环境与流程注意（本轮实测）

- 基线：`main` = `76c3414` = `origin/main`（**0/0 同步**，无分叉）。工作区仅外部 `M AGENTS.md`（非本体系产生，不碰、不代提交；提交 `.agent/` 只 `git add` 指定文件，严禁 `-a`/`-A`）。
- 支持环境（D-017）：CI = ROS 2 **Jazzy**，本机 = **lyrical**；验证入口唯一为 `bash scripts/drive_gates.sh`（`QUICK=1` 快速轨）。**ROS1 运行时不在支持范围内**（仅源码对照基线 `FSAC/2025`）。
- **`gh` 正常**：`issue list/view/create/comment/close`、`run list/view` 全部可用；账号 `zhangjszs`。`gh` **无** `milestone` 子命令（勿再试）。
- **`gh issue list --json` 不接受 `commentCount` 字段**（输出为空）；用 `--json number,updatedAt,labels,title` + `gh issue view --json comments`。
- **`gh run view --json jobs` 偶发空输出**：需重试。
- CI：`76c3414` 的 push CI `39195951665` success；同分支 `ab1ad6e`/`3ca7930` 亦 success。
- Nightly **七连绿**（10-12/13×2/14/15/16/17/18），最新 `39161165288` @`bf26487` success（784 s）。10-19 的 Nightly 在本轮实测时刻（`2026-10-19T17:33Z`）未到调度点（约 20:00Z），**下一轮入账**。
