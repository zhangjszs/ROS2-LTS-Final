# TASKS.md — Issue 本地镜像（权威：GitHub Issue；本文件由 Planner 每轮整体重写）

> 维护者：Planning Agent｜最后更新：2026-10-20（`plan-20261020` 轮后：**#77/#62 验收关闭、#23 关闭；主线切至 S2a 直线加速事件闭环**；随后按用户要求**扩充队列，新增 #81–#85**；共 **33 open**）
> 规则：本文件只读镜像，Executor 改状态时先改 Issue 再同步对应行；冲突以 Issue 为准。
> 日报：`.agent/DIARY/2026-10-19.md` 已由上一执行轮产出（本轮验收已引用）；**`.agent/DIARY/2026-10-20.md` 已由本执行轮产出**（#81 + #80 + #78 + #79 四单，含 #82 阻塞记录）。
> 🆕 **本轮变化**：用户三项决策落定——**D-014 直线加速 / D-015 暂缓复杂定位后端 / D-016 底盘自建**；D-017 明确支持环境（ROS2 Jazzy/lyrical，入口 `scripts/drive_gates.sh`）。#23 关闭，赛制与定位的不确定性消除，队列首次拥有**面向端到端闭环**的 P1 任务。随后按用户要求补充立项 **#81–#85**（赛道元数据、ROS 闭环、死参数、参数守卫、迁移文档回填）。
> 🔄 **Executor 轮（2026-10-20）状态更新**：**#81 已完成并转 `in-review`**（提交 `1aae347` / merge `0f48b3a`，已推送 origin/main；报告 comment `6289660599`）。**#80 已完成并转 `in-review`**（提交 `3870249` / merge `126cc83`，已推送 origin/main；报告 comment `6290203705`）。**#78 已完成并转 `in-review`**（提交 `f56ff36` / merge `a71b548`，已推送 origin/main；报告 comment `6290840535`）。**#79 已完成并转 `in-review`**（提交 `0654d0e` / merge `ea5e19b`，已推送 origin/main；报告 comment `6291014069`）。**#82 转 `blocked`**（环境阻塞：本机 ROS2 多节点 DDS 发现不可用；报告 comment `6290651020`，**未提交任何代码**）。

## 今日队列（2026-10-20，给 Executor 的建议顺序）

1. **#81** ✅ **已完成（转 `in-review`）**：赛道元数据补齐：acceleration 的 `finish_x=75` / `runout_end_m=100` 进入 TrackDefinition 与 `tracks.json`（P2）—— 提交 `1aae347` / merge `0f48b3a`；消费方改用 `track->finish_x` / `track->runout_end_m` / `hasFinish()`，不得新增硬编码。
2. **#80** ✅ **已完成（转 `in-review`）**：直线加速完赛判定：跨越 x=75 终点线 + 25 m 缓冲区（x≤100）内停稳 + benchmark JSON 字段与门禁（P1）——提交 `3870249` / merge `126cc83`；新增纯 std 判据核 `finish_judge.hpp`（到达速度上限反推 + 越线/停稳状态机）+ 11 例单测 + 门禁第 9 节（JSON 字段断言 + 指令时间线断言 + `--no-post-finish-braking` 可证伪负样本）；重录 `acceleration` 基线（`stopped` false→true）。skidpad/trackdrive 基线文件未改（输出仅多 4 个缺省字段，测量值逐位未变）。⚠ 触及 D-003 调参冻结（acceleration 口径变化，待 Planner 判定是否重声明轮次），详见 comment `6290203705`。
3. **#82** ⛔ **转 `blocked`（环境阻塞，未提交代码）**：ROS 闭环直线加速完赛——本机 ROS2 多节点 DDS 发现不可用（8 组环境组合矩阵全败；40 s 订阅 `/localization/vehicle_state` 收到 0 条），无法产出验收第 1/2 条的闭环证据。报告 comment `6290651020` 含 6 条已完成调查结论，并建议拆为 **#82a**（纯 std 事件语义核，本机可验证）/ **#82b**（多节点闭环，待环境）。依赖 #81/#80 均已就位，环境恢复后可直接开工。
4. **#79** ✅ **已完成（转 `in-review`）**：`vehicle_state::GeodeticToEnu` 下沉纯 std core（新增 header-only `geodetic_core.hpp`）+ 8 例无 ROS 确定性夹具（#24 第 1 条切片，comment `6273165395`）——提交 `0654d0e` / merge `ea5e19b`；行为**逐位一致**（位模式差分：11 具名用例 + 245 网格点 × 3 轴，`TOTAL_MISMATCH=0`，-O0/-O2/-O3 三档均 `BITWISE_IDENTICAL`）；双轨注册到 `core_standalone` 与 `ament gtest`。
5. **#78** ✅ **已完成（转 `in-review`）**：`hough_min_inlier_ratio` 超大有限值守卫（P3）——提交 `f56ff36` / merge `a71b548`；守卫扩为 `ratio ∈ [0,1]`（与旧行为等价）；**先红后绿**：修复前 UBSan 复现 `1e+301 is outside the range of representable values of type 'int'`（EXIT=1），修复后 EXIT=0 且 `[0,1]` 内 6 组合法 ratio 的输出位模式逐字不变（`GOLD_IDENTICAL_RC=0`）。

> 队列 = 5 项，在 2–5 区间上限。顺序由依赖决定：#81 → #80 → #82 是 S2a 主链；#79/#78 为并行加固。**本轮结果：#81/#80/#78/#79 四单已完成并转 `in-review`；#82 因环境阻塞转 `blocked`（未提交代码）**。**#83/#84/#85 已就绪但不在今日队列**（P3 积压：#83 死参数、#84 参数 NaN 守卫、#85 迁移文档回填），待主链消化后由下一轮排入。

## ready（可执行，3）

| Issue | 标题 | 优先级 | 依赖 / 要点 |
| --- | --- | --- | --- |
| #83 | `hough_threshold` 死参数（节点/YAML 暴露但算法未使用） | P3 | 无；实测 0 与 1000 输出逐位相同；方案 A 删除 / B 接线 |
| #84 | temporal_filter_* 与 kCenterMargin 的 NaN/越界静默失效 | P3 | 无；实测 alpha=NaN 输出 NaN 且 valid=true |
| #85 | MIGRATION_BASELINE 回填 D-014/15/16 并挂入文档入口 | P3 | 无；仅 docs/；不改变结论边界 |

## in-review（待验收，4）

| Issue | 标题 | 提交 | 报告 |
| --- | --- | --- | --- |
| #81 | 赛道元数据补齐：finish_x / runout_end_m 进入 TrackDefinition 与 tracks.json | `1aae347` / merge `0f48b3a` | comment `6289660599` |
| #80 | 直线加速完赛判定（终点线 + 终点后缓冲区停稳 + benchmark 门禁） | `3870249` / merge `126cc83` | comment `6290203705` |
| #78 | Hough `hough_min_inlier_ratio` 超大有限值 UB 守卫（先红后绿） | `f56ff36` / merge `a71b548` | comment `6290840535` |
| #79 | vehicle_state GeodeticToEnu 下沉纯 std 核 + 无 ROS 合成夹具 | `0654d0e` / merge `ea5e19b` | comment `6291014069` |

## in-progress（0）

无（本轮 #78、#79 均已完成并转 `in-review`；#82 转 `blocked`，未提交代码）。

## 仓库外阻塞（8）

| Issue | 标题 | 阻塞原因（D-013 分类） | 本轮复核 |
| --- | --- | --- | --- |
| #15 | 共用执行器适配层与车辆标定配置（P1） | **资料缺失**：VCU 协议文档、台架标定实值（D-016 已确认非外部代码缺失） | `updatedAt` 10-19（本轮记录 D-016）；性质澄清，仍不可执行 |
| #16 | 统一任务与安全状态、最终指令仲裁（P1） | **资料缺失**：硬件急停链路、实车停车预算 | `updatedAt` 10-05，无新评论 |
| #17 | 可复现仿真场景、可信 KPI 与自动回归（P1） | **资料缺失**：模型速度范围实测、感知扰动量化 | 同上 |
| #19 | Pure Pursuit 低速基线与 MPC 公平对比（P1） | **资料缺失**：依赖 #17 扰动场景 | 同上 |
| #18 | rosbag 数据集与感知质量评估（P2） | **资料缺失**：真实 bag | 同上 |
| #82 | ROS 闭环直线加速完赛：finish 事件 → 锁存停 → 缓冲区停稳 | **运行时缺失**：本机 ROS2 多节点 DDS 发现不可用（VPN 网段 UDP 组播被挡；`FASTDDS_BUILTIN_TRANSPORTS=SHM` 反而使发现彻底失效；8 组环境组合矩阵全败；40 s 订阅收到 0 条 `/localization/vehicle_state`） | 2026-10-20（本轮实测；报告 comment `6290651020`；建议拆为 #82a 纯 std 事件语义核 + #82b 多节点闭环；**未提交任何代码**） |
| #24 | 跨 ROS 版本语义差分回放与故障回归（P1） | **运行时缺失**：验收第 2 条需 ROS1 运行环境（本机无 Noetic、无 docker）；第 1 条已拆给 #79 | `updatedAt` 10-19（拆分记录）；**第 1 条切片已由 #79 完成并转 in-review**（`0654d0e` / merge `ea5e19b`） |
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

- 基线：`main` = `ea5e19b`（#79 merge）= `origin/main`（**0/0 同步**，无分叉）。工作区仅外部 `M AGENTS.md`（非本体系产生，不碰、不代提交；提交 `.agent/` 只 `git add` 指定文件，严禁 `-a`/`-A`）。**该外部改动全程未碰**：本轮开始与结束均只观察到该文件为脏，未 add、未 commit。
- **远程遗留分支**：远端仍有 `agent/issue-81-track-metadata`（#81 的分支，已合入 main `0f48b3a` 却未删）；契约 1.9 要求合回主干后删除临时分支，Executor 在 #81 收尾时遗漏。本轮未擅自删（属前序遗留，删除远程分支影响面小但需 Planner 知悉）。
- 支持环境（D-017）：CI = ROS 2 **Jazzy**，本机 = **lyrical**；验证入口唯一为 `bash scripts/drive_gates.sh`（`QUICK=1` 快速轨）。**ROS1 运行时不在支持范围内**（仅源码对照基线 `FSAC/2025`）。
- **`gh` 正常**：`issue list/view/create/comment/close`、`run list/view` 全部可用；账号 `zhangjszs`。`gh` **无** `milestone` 子命令（勿再试）。
- **`gh issue list --json` 不接受 `commentCount` 字段**（输出为空）；用 `--json number,updatedAt,labels,title` + `gh issue view --json comments`。
- **`gh run view --json jobs` 偶发空输出**：需重试。
- CI：`0f48b3a` 的 push CI `39413891086` success；`126cc83` 的 CI `39416500750` **success**（13m31s）；`f5008d5` 的 CI `39418221488` **success**；#78 merge `a71b548` 的 CI `39422351603` 与 #79 merge `ea5e19b` 的 CI `39423649207` 本轮实测仍 `in_progress`（**结论待 Planner 复核**，run id 已写入 #78/#79 执行报告与本文件）。
- Nightly **八连绿**（10-12/13×2/14/15/16/17/18/19），最新 `39331301871` @`b27fc64` success（10-19 调度点已过，本轮入账）。
- **本机 DDS 发现失效（本轮实测，影响所有多节点 runtime 门禁）**：`ros2 node list` 恒为空；8 组环境组合矩阵（SHM / LOCALHOST_ONLY / RANGE / DOMAIN_ID）全败或仅偶发成功；数据面 40 s 收到 0 条 `/localization/vehicle_state`。**纯 std / 离线门禁（core_standalone、benchmark、lint、colcon test）不受影响**，可照常作为验收证据。
