# TASKS.md — Issue 本地镜像（权威：GitHub Issue；本文件由 Planner 每轮整体重写）

> 维护者：Planning Agent｜最后更新：2026-10-20（`plan-20261020` 轮后：**#77/#62 验收关闭、#23 关闭；主线切至 S2a 直线加速事件闭环**；随后按用户要求**扩充队列，新增 #81–#85**；共 **33 open**）
> 规则：本文件只读镜像，Executor 改状态时先改 Issue 再同步对应行；冲突以 Issue 为准。
> 日报：`.agent/DIARY/2026-10-19.md` 已由上一执行轮产出（本轮验收已引用）；**`.agent/DIARY/2026-10-20.md` 已由本执行轮产出**（#81 + #80 + #78 + #79 + #82 五单，含对 #82 误判的环境根因更正）。
> 🆕 **本轮变化**：用户三项决策落定——**D-014 直线加速 / D-015 暂缓复杂定位后端 / D-016 底盘自建**；D-017 明确支持环境（ROS2 Jazzy/lyrical，入口 `scripts/drive_gates.sh`）。#23 关闭，赛制与定位的不确定性消除，队列首次拥有**面向端到端闭环**的 P1 任务。随后按用户要求补充立项 **#81–#85**（赛道元数据、ROS 闭环、死参数、参数守卫、迁移文档回填）。
> 🔄 **Executor 轮（2026-10-20）状态更新**：**#81 已完成并转 `in-review`**（提交 `1aae347` / merge `0f48b3a`，已推送 origin/main；报告 comment `6289660599`）。**#80 已完成并转 `in-review`**（提交 `3870249` / merge `126cc83`，已推送 origin/main；报告 comment `6290203705`）。**#78 已完成并转 `in-review`**（提交 `f56ff36` / merge `a71b548`，已推送 origin/main；报告 comment `6290840535`）。**#79 已完成并转 `in-review`**（提交 `0654d0e` / merge `ea5e19b`，已推送 origin/main；报告 comment `6291014069`）。**#82 已完成并转 `in-review`**（提交 `343ecc3` + `40673b0` / merge `e5f9998`，已推送 origin/main；报告 comment `6295280493`）——⚠ **该单上一轮的 `blocked` 判定是误判**（真因：ros2cli daemon 未显式启动，非 DDS 发现不可用），已在 comment `6295280493` 的「歧义处理 0」逐条作废旧诊断矩阵并给出对照证据。

## 今日队列（2026-10-20，给 Executor 的建议顺序）

1. **#81** ✅ **已完成（转 `in-review`）**：赛道元数据补齐：acceleration 的 `finish_x=75` / `runout_end_m=100` 进入 TrackDefinition 与 `tracks.json`（P2）—— 提交 `1aae347` / merge `0f48b3a`；消费方改用 `track->finish_x` / `track->runout_end_m` / `hasFinish()`，不得新增硬编码。
2. **#80** ✅ **已完成（转 `in-review`）**：直线加速完赛判定：跨越 x=75 终点线 + 25 m 缓冲区（x≤100）内停稳 + benchmark JSON 字段与门禁（P1）——提交 `3870249` / merge `126cc83`；新增纯 std 判据核 `finish_judge.hpp`（到达速度上限反推 + 越线/停稳状态机）+ 11 例单测 + 门禁第 9 节（JSON 字段断言 + 指令时间线断言 + `--no-post-finish-braking` 可证伪负样本）；重录 `acceleration` 基线（`stopped` false→true）。skidpad/trackdrive 基线文件未改（输出仅多 4 个缺省字段，测量值逐位未变）。⚠ 触及 D-003 调参冻结（acceleration 口径变化，待 Planner 判定是否重声明轮次），详见 comment `6290203705`。
3. **#82** ✅ **已完成并转 `in-review`**（上一轮误判为环境阻塞）：`finish_event_node` 终点事件源（复用 #80 纯 std 判据核，首次越线恰好一次发 `finish`）+ `scripts/acceleration_finish_smoke.sh` 仲裁器链闭环门禁（纳入 `drive_gates.sh` 为 `accel-finish` 硬门，两臂）。实测：正样本 rc=0（`finish_event_count=1`、`stop_kind=REQUEST(2)`、`stop_position_m=84.730 ≤ 100.0`、`v_final=0.0`）；可证伪负样本 rc=1（原因指向“未停车/越过缓冲区”）；完整版 `drive_gates.sh` **12 门全 PASS、hard_fail=0、440s**。提交 `343ecc3`+`40673b0` / merge `e5f9998`；报告 comment `6295280493`。
4. **#79** ✅ **已完成（转 `in-review`）**：`vehicle_state::GeodeticToEnu` 下沉纯 std core（新增 header-only `geodetic_core.hpp`）+ 8 例无 ROS 确定性夹具（#24 第 1 条切片，comment `6273165395`）——提交 `0654d0e` / merge `ea5e19b`；行为**逐位一致**（位模式差分：11 具名用例 + 245 网格点 × 3 轴，`TOTAL_MISMATCH=0`，-O0/-O2/-O3 三档均 `BITWISE_IDENTICAL`）；双轨注册到 `core_standalone` 与 `ament gtest`。
5. **#78** ✅ **已完成（转 `in-review`）**：`hough_min_inlier_ratio` 超大有限值守卫（P3）——提交 `f56ff36` / merge `a71b548`；守卫扩为 `ratio ∈ [0,1]`（与旧行为等价）；**先红后绿**：修复前 UBSan 复现 `1e+301 is outside the range of representable values of type 'int'`（EXIT=1），修复后 EXIT=0 且 `[0,1]` 内 6 组合法 ratio 的输出位模式逐字不变（`GOLD_IDENTICAL_RC=0`）。

> 队列 = 5 项，在 2–5 区间上限。顺序由依赖决定：#81 → #80 → #82 是 S2a 主链；#79/#78 为并行加固。**本轮结果：#81/#80/#78/#79/#82 五单全部完成并转 `in-review`**（#82 上一轮的阻塞判定为误判，已更正）。**#83/#84/#85 已就绪但不在今日队列**（P3 积压：#83 死参数、#84 参数 NaN 守卫、#85 迁移文档回填），待主链消化后由下一轮排入。

## ready（可执行，3）

| Issue | 标题 | 优先级 | 依赖 / 要点 |
| --- | --- | --- | --- |
| #83 | `hough_threshold` 死参数（节点/YAML 暴露但算法未使用） | P3 | 无；实测 0 与 1000 输出逐位相同；方案 A 删除 / B 接线 |
| #84 | temporal_filter_* 与 kCenterMargin 的 NaN/越界静默失效 | P3 | 无；实测 alpha=NaN 输出 NaN 且 valid=true |
| #85 | MIGRATION_BASELINE 回填 D-014/15/16 并挂入文档入口 | P3 | 无；仅 docs/；不改变结论边界 |

## in-review（待验收，5）

| Issue | 标题 | 提交 | 报告 |
| --- | --- | --- | --- |
| #81 | 赛道元数据补齐：finish_x / runout_end_m 进入 TrackDefinition 与 tracks.json | `1aae347` / merge `0f48b3a` | comment `6289660599` |
| #80 | 直线加速完赛判定（终点线 + 终点后缓冲区停稳 + benchmark 门禁） | `3870249` / merge `126cc83` | comment `6290203705` |
| #78 | Hough `hough_min_inlier_ratio` 超大有限值 UB 守卫（先红后绿） | `f56ff36` / merge `a71b548` | comment `6290840535` |
| #79 | vehicle_state GeodeticToEnu 下沉纯 std 核 + 无 ROS 合成夹具 | `0654d0e` / merge `ea5e19b` | comment `6291014069` |
| #82 | ROS 闭环直线加速完赛（终点事件源 + 仲裁器锁存停 + accel-finish 硬门） | `343ecc3` + `40673b0` / merge `e5f9998` | comment `6295280493`（含对旧阻塞报告的作废声明） |

## in-progress（0）

无（今日队列五单 #81/#80/#78/#79/#82 均已完成并转 `in-review`；#82 先前那份“未提交代码”的阻塞记录已作废）。

## 仓库外阻塞（7）

> ⚠ **本轮更正**：#82 原先列入本节（“运行时缺失：本机 DDS 发现不可用”），**该判定是误判，已从上表移出**。
> 真因：ros2cli daemon 未显式启动（`drive_gates.sh` 原先只 `daemon stop` 并指望自动重建，
> 而在“每次执行都新建进程/会话”的沙箱里 daemon 随父进程退出）→ `ros2 node list` 恒超时返空、
> 节点被误判“未注册”。已修为 `stop` + 显式 `start` + `sleep`，**本机全部 runtime 门禁恢复可跑**（实测 12 门全绿）。
> 旧的“8 组环境组合矩阵全败”诊断作废：那 8 组的 publisher/subscriber 起在两次独立调用里
> = 两个独立沙箱（各自独立 `/dev/shm`），怎么调环境变量都不可能成功。详见 comment `6295280493`。

| Issue | 标题 | 阻塞原因（D-013 分类） | 本轮复核 |
| --- | --- | --- | --- |
| #15 | 共用执行器适配层与车辆标定配置（P1） | **资料缺失**：VCU 协议文档、台架标定实值（D-016 已确认非外部代码缺失） | `updatedAt` 10-19（本轮记录 D-016）；性质澄清，仍不可执行 |
| #16 | 统一任务与安全状态、最终指令仲裁（P1） | **资料缺失**：硬件急停链路、实车停车预算 | `updatedAt` 10-05，无新评论 |
| #17 | 可复现仿真场景、可信 KPI 与自动回归（P1） | **资料缺失**：模型速度范围实测、感知扰动量化 | 同上 |
| #19 | Pure Pursuit 低速基线与 MPC 公平对比（P1） | **资料缺失**：依赖 #17 扰动场景 | 同上 |
| #18 | rosbag 数据集与感知质量评估（P2） | **资料缺失**：真实 bag | 同上 |
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

- 基线：`main` = `origin/main`（**0/0 同步**，无分叉）；本轮最近一次**代码**提交为 #82 的 `343ecc3` + `40673b0` / merge `e5f9998`（其后的 `.agent` 收尾提交见 `git log --oneline -3`）。工作区仅外部 `M AGENTS.md`（非本体系产生，不碰、不代提交；提交 `.agent/` 只 `git add` 指定文件，严禁 `-a`/`-A`）。**该外部改动全程未碰**：本轮开始与结束均只观察到该文件为脏，未 add、未 commit。#82 的临时分支本地与远程均已删除（误推的远程分支已回收）。
- **远程遗留分支**：远端仍有 `agent/issue-81-track-metadata`（#81 的分支，已合入 main `0f48b3a` 却未删）；契约 1.9 要求合回主干后删除临时分支，Executor 在 #81 收尾时遗漏。本轮未擅自删（属前序遗留，删除远程分支影响面小但需 Planner 知悉）。
- 支持环境（D-017）：CI = ROS 2 **Jazzy**，本机 = **lyrical**；验证入口唯一为 `bash scripts/drive_gates.sh`（`QUICK=1` 快速轨）。**ROS1 运行时不在支持范围内**（仅源码对照基线 `FSAC/2025`）。
- **`gh` 正常**：`issue list/view/create/comment/close`、`run list/view` 全部可用；账号 `zhangjszs`。`gh` **无** `milestone` 子命令（勿再试）。
- **`gh issue list --json` 不接受 `commentCount` 字段**（输出为空）；用 `--json number,updatedAt,labels,title` + `gh issue view --json comments`。
- **`gh run view --json jobs` 偶发空输出**：需重试。
- CI：`0f48b3a` 的 push CI `39413891086` success；`126cc83` 的 CI `39416500750` **success**（13m31s）；`f5008d5` 的 CI `39418221488` **success**；#78 merge `a71b548` 的 CI `39422351603` **success**；#79 merge `ea5e19b` 的 CI `39423649207` **success**；.agent 收尾 `8bc9e19` 的 CI `39424306720` **success**；`.agent` 补记 `90dd749` 的 CI `39426424030` **success**；**#82 的两条代码提交 CI 均已复核 success**：merge `e5f9998` → `39441369149` **success**；尾提交 `40673b0` → `39453577195` **success**。本轮无因本次变更导致的 CI 失败需修复。
- Nightly **八连绿**（10-12/13×2/14/15/16/17/18/19），最新 `39331301871` @`b27fc64` success（10-19 调度点已过，本轮入账）。
- **更正（推定于本行原有的结论）——本机并没有“DDS 发现失效”，真因是 ros2cli daemon 未启动**：
  原文：“本机 DDS 发现失效（影响所有多节点 runtime 门禁）：`ros2 node list` 恒为空；8 组环境组合矩阵全败；40 s 收到 0 条 `/localization/vehicle_state`”
  → **更正为**：上述现象的真因是 `drive_gates.sh` 原先只 `ros2 daemon stop` 并指望后续调用自动重建，而在“每次执行都新建进程/会话”的环境里 daemon 随父进程退出 → `ros2 node list` 恒**超时**（rc=124，不是空表）→ 节点被误判“未注册”。`FASTDDS_BUILTIN_TRANSPORTS=SHM` 处方**一直有效**（AGENTS.md 记载没错）。已修为 `stop` + 显式 `start` + `sleep 3`，**本机完整版 `drive_gates.sh` 实测 12 门全 PASS、hard_fail=0、440s**（含 closed-loop / fault-injection / closed-loop-fault）。⇒ **本轮之前所有以“本机 runtime 门禁不可用”为由的“未执行”记录均需重新评估**（包括 #80 报告与 #79 报告里的相应条目）。诊断跨进程 DDS 时，两端必须在同一次执行内，否则是两个独立沙箱。
- 其余 5 个既有 runtime 脚本各自头部仍只 `daemon stop`（单独直跑时仍会假失败）；本轮按最小改动只修了统一入口 `drive_gates.sh` 与新脚本。建议 Planner 定级（可与 #69「DDS 环境性失败区分」合并）。
