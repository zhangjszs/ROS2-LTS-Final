# TASKS.md — Issue 本地镜像（权威：GitHub Issue；本文件由 Planner 每轮整体重写）

> 维护者：Planning Agent｜最后更新：2026-10-21T22:30:00+08:00（`plan-20261021d`：**#83/#84/#85/#86 四单全量验收入账并关闭**；ready-for-agent 队列见底 = 0，按 D-019 用户亲选 B 正式转入 D-009 维持模式）
> 规则：本文件为只读镜像，Executor 改状态时先改 Issue 再同步对应行；冲突以 Issue 为准。
> 权威来源状态：`gh issue list --state open` 实测 **25 open**（ready 0 · in-review 0 · in-progress 0 · 仓库外阻塞 7 · needs-info 3 · ready-for-human 14 · 路线图伞形 1）。
> 🆕 **本轮变化（plan-20261021d）**：
> 1. 今日队列原 4 单（#83、#84、#85、#86）经 Executor 会话 1 执行完毕并提交执行报告，本轮 Planner 独立复核代码 diff、运行命令证据与 CI 记录，全部判定**通过并关闭**（Issue 评论与关闭已在远端生效）。
> 2. CI 全绿：#83 `39596962618`、#84 `39599109053`、#85 `39599755595`、#86 `39599977048`、会话收尾 `39600468883` 均 success。
> 3. 遗留分支清理：`origin/agent/issue-81-track-metadata` 已由 Executor 按指令删除。
> 4. 队列状态：ready-for-agent 见底（0 项）。依 **D-019（用户 2026-10-21 亲选 B）**，项目在完成 P3 积压后正式转入 **D-009 维持模式**（CI/Nightly 看护 + blocked 零成本复查，不为找活而扫描）。
> 5. **Nightly 10-21 尚未到点（2026-10-21T15:19Z 实测）**：cron `0 18 * * *` + GitHub 调度延迟 2–4h（历史拉起 19:58–21:56Z），无 queued/in_progress 项 ⇒ 今日无可入账 run；连续 success 计数仍为 **9**（最新 10-20 `39498071564`）。**下一轮直接取新 run 入账**，预计落地 10-21T20:00–22:00Z。不以旧 run 假冒、不手动 dispatch 伪造。

## 今日队列（2026-10-21，给 Executor 的建议顺序）

> **当前队列为空（0 项）**：
> 今日队列四单（#83、#84、#85、#86）已在 exec-20261021 会话 1 串行执行完毕，并于本轮全部通过验收关闭。
> 当前可执行池无 ready-for-agent 任务。
> 依据 **D-019 用户亲选方案 B**，项目完成 P3 积压后正式转入 **D-009 维持模式**。
> Executor 今日无需领取新任务；下一执行阶段待外部输入到位（解除 blocked）或用户明确下发新 Milestone 方向。

## in-review（待验收，0）

无。今日四单已全部验收通过并关闭。

## ready（可执行，0）

无——队列见底。

## in-progress（0）

无。

## 仓库外阻塞（7，均为外部输入缺失，非代码可推进）

| Issue | 标题 | 阻塞原因（D-013 分类） | 本轮复核 |
|---|---|---|---|
| #15 | 共用执行器适配层与车辆标定配置（P1） | **资料缺失**：VCU 协议文档、台架标定实值（D-016 已确认外部驱动不存在需自建） | `updatedAt` 10-19；性质已澄清，仍无外部资料输入 |
| #16 | 统一任务与安全状态、最终指令仲裁（P1） | **资料缺失**：硬件急停链路、实车停车预算 | `updatedAt` 10-05，无新评论 |
| #17 | 可复现仿真场景、可信 KPI 与自动回归（P1） | **资料缺失**：模型速度范围实测、感知扰动量化 | `updatedAt` 10-05，无新评论 |
| #19 | Pure Pursuit 低速基线与 MPC 公平对比（P1） | **资料缺失**：依赖 #17 扰动场景 | `updatedAt` 10-05，无新评论 |
| #18 | rosbag 数据集与感知质量评估（P2） | **资料缺失**：真实 bag（亦是 D-015 定位后端与 S2b 八字的前置） | `updatedAt` 10-05，无新评论 |
| #24 | 跨 ROS 版本语义差分回放与故障回归（P1） | **运行时缺失**：验收第 2 条需 ROS1 运行环境（本机无 Noetic、无 docker）；第 1 条切片已由 #79 完成关闭 | `updatedAt` 10-19；本体仍卡运行时 |
| #37 | 翻转 use_arbiter 默认（P3，needs-triage + blocked） | **资料缺失**：需 #15 标定完成 | `updatedAt` 10-09，无新评论 |

> D-013 复查：本轮逐单复核 7 个 blocked Issue 的外部状态，无任何外部输入到位，7 单全部维持 blocked。依 D-009 维持模式，后续每轮仅做零成本复查，不主动扫描外部。

## needs-info / 待条件（3）

| Issue | 标题 | 等待 |
|---|---|---|
| #25 | 技术博客系列（P2） | #17①② / #18 证据 |
| #26 | 完整项目展示（P2，父跟踪） | 同上 |
| #27 | 跨仓库路线总入口（P2） | 同上 |

## ready-for-human（14，用户/人工跟进，不计入 agent 队列与饥饿规则）

#63 CI 路径过滤与并发取消（P2）· #64 `check_cpp20.sh` 参数统一（P2）· #65 closed-loop-fault 六臂并行（P2）· #66 CI job 三段拆分（P2）· #67 `affected_gates.sh`（P3）· #68 ccache 键头文件通配（P3）· #69 DDS 环境性失败区分（P2，含 daemon 预热回填其余 5 脚本范围）· #70 门禁时长预算监控（P3）· #71 core_standalone 覆盖率基线（P3）· #72 Prepare ROS 2 步骤超时（P3）· #73 评分核测试双轨去重（P3）· #74 本地快速反馈文档（P3）· #75 测试编写规范入库（P3）· #76 伞形：12 个仅 colcon 轨测试下沉（P3）

## 路线图伞形（1，长期跟踪，无优先级标签系有意豁免）

| Issue | 标题 |
|---|---|
| #20 | ROS 2 车队开发路线：运行基线 → 可信仿真 → 低速闭环 → 性能优化 |

## 已关闭（近期备查）

- **#86**（2026-10-21 验收关闭；`8eab81e`/merge `6b6697f`；comment `6318358288`）：AGENTS.md/CLAUDE.md daemon 口径纠错（stop → 显式 start → sleep 3；组播因果句撤回具体断言），正文改动仅 3 行，未碰脚本，逐行文本已供用户复核。CI `39599977048` success。
- **#85**（2026-10-21 验收关闭；`4ea8467`/merge `4d303d1`；comment `6318356016`）：docs/MIGRATION_BASELINE.md 回填 D-014/D-015/D-016 决策引用（五值枚举不变）；新增 docs/README.md 索引挂入 5 篇核心文档，12 链接解析验证全过。CI `39599755595` success。
- **#84**（2026-10-21 验收关闭；`4ff55d4`/merge `c7d7e8c`；comment `6318335866`）：straight_line_planner 时序滤波与分离参数非法值清洗（回退默认 + WARN）；新增 5 个非法参数用例，591 tests 0 fail，消除 NaN 输出与静默丢弃锥桶风险；未覆盖 #78 守卫。CI `39599109053` success。
- **#83**（2026-10-21 验收关闭；`db68a2a`/merge `cb09a6b`；comment `6318333740`）：straight_line_planner 移除未读取死参数 hough_threshold（方案 A，纯 4 行删除）；默认配置黄金样本逐字不变；未触碰 #78 守卫。CI `39596962618` success。
- **#82**（2026-10-21 验收关闭；`343ecc3`+`40673b0`/merge `e5f9998`；comment `6296139845`）：ROS 闭环完赛门禁 accel-finish（正样本 rc=0、负样本 rc=1、对照臂通过），12 门全 PASS。CI `39441369149`/`39453577195` success。
- **#80**（2026-10-21 验收关闭；`3870249`/merge `126cc83`；comment `6296132845`）：直线加速完赛判据核（finish_judge.hpp + 门禁第 9 节断言）；acceleration 基线重录 stopped=true（口径变更 → D-018）。CI `39416500750` success。
- **#81**（2026-10-21 验收关闭；`1aae347`/merge `0f48b3a`；comment `6296130011`）：赛道元数据 finish_x/runout 进 TrackDefinition+tracks.json。
- **#79**（2026-10-21 验收关闭；`0654d0e`/merge `ea5e19b`；comment `6296137702`）：GeodeticToEnu 下沉 geodetic_core.hpp 纯 std 核（位模式差分 TOTAL_MISMATCH=0）。CI `39423649207` success。
- **#78**（2026-10-21 验收关闭；`f56ff36`/merge `a71b548`；comment `6296135114`）：hough_min_inlier_ratio 增 [0,1] 上界守卫消除 float→int UB。CI `39422351603` success。
- 更早：#77 / #62 / #23（2026-10-20）；#59 / #60 / #61（2026-10-12）；#55 / #56 / #57 / #58。

## 环境与流程注意（本轮实测/复核）

- 基线：`main` = `origin/main` = `f3faa10`（**0/0 同步**，无分叉）。
- **远端遗留分支清理完毕**：`origin/agent/issue-81-track-metadata` 已由 Executor 亲验并成功删除，契约 1.9 纪律恢复。
- **独立复核证据（本轮 Planner 第一方实测）**：
  - `bash scripts/core_standalone_check.sh` → EXIT=0，**32/32 通过、ASan+UBSan 干净**（含新增非法参数清洗测试用例）。
  - `QUICK=1 bash scripts/drive_gates.sh` → 全部硬门禁通过（21s）。
  - `test -f` 抽验 docs 链接 ×13 → 全部存在无断链。
  - `gh run list` 核验远端 CI 5 run 全部 success。
- **支持环境（D-017）**：CI = ROS 2 **Jazzy**，本机 = **lyrical**；验证入口唯一为 `bash scripts/drive_gates.sh`。多节点 runtime 门禁必须 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`。
