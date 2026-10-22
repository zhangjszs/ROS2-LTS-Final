# TASKS.md — Issue 本地镜像（权威：GitHub Issue；本文件由 Planner 每轮整体重写）

> 维护者：Planning Agent｜最后更新：2026-10-22T11:00:00+08:00（`plan-20261022`：S2b 首批三单就绪等候执行，Nightly 10 连绿入账，GitHub Issues 权威状态全面对齐）
> 规则：本文件为只读镜像，Executor 改状态时先改 Issue 再同步对应行；冲突以 Issue 为准。
> 权威来源状态：`gh issue list --state open` 实测 **28 open**（ready 3 · in-review 0 · in-progress 0 · 仓库外阻塞 7 · needs-info 3 · ready-for-human 14 · 路线图伞形 1）。
> 🆕 **本轮变化（plan-20261021e）**：
> 1. S2a 验收出清后，依据用户交互指令亲选（裁 **D-022**），正式启动 **Milestone S2b「八字绕环仿真闭环与规划加固」**。
> 2. 在纯 INS 站心坐标系（D-015）基线下，立项首批 3 项 ready-for-agent 任务：
>    - **#87**（P2）：`skidpad_planner` 节点对齐契约 QoS（kQosPath / kQosState）与默认话题 `/planning/pathlimits`，消除重复圆拟合计算；
>    - **#88**（P1）：`TrackGenerator` 补齐八字绕环规范中心线（进场 + 右双圈 + 换圆过渡切线 + 左双圈 + 出场停稳）；
>    - **#89**（P2）：`skidpad_planner` 核心边界参数清洗与退化守卫（非法值回退默认并构造期 WARN，防除零与清空锥桶）。
> 3. 今日队列由 0 项恢复为 **3 项**（满足 2–5 项约束，全部通过 4.2 门禁）。

## 今日队列（2026-10-22，给 Executor 的建议顺序）

> **执行顺序建议**：
> - **#88（P1）** 位于 `src/simulation/track_benchmark/`，与规划器完全独立，提供八字赛道完整几何与真值中心线，建议优先合入。
> - **#87（P2）与 #89（P2）** 同处 `src/planning/skidpad_planner/`（#87 动 node/YAML/QoS，#89 动参数清洗/核心计算/单测）。为避免分支 rebase 冲突，**两单建议串行执行**：先 #87 合入 main，再拉分支开工 #89；或在同分支协同。
> - 推荐总体顺序：**#88 → #87 → #89**。

1. **#88** `[S2b] TrackGenerator 补齐八字绕环完整中心线（右双圈+过渡切入+左双圈+出场）`（P1，ready-for-agent）：
   - `GenerateSkidpad()` 补齐 FSAC 规则要求的完整中心线（进场直线 -> 右双圈 -> S 弯过渡切线 -> 左双圈 -> 出场直线），解决当前仅单右圆、长度 72.33m、基线录制撞锥 3 次的问题；导出同步 `skidpad_track.csv`；单测断言曲率极值与连续性。
2. **#87** `[S2b] skidpad_planner 节点对齐契约 QoS 与接口话题规范（kQosPath / kQosState / 默认话题对齐）`（P2，ready-for-agent）：
   - 节点发布/订阅接入 `common_msgs::contract::makeQoS`（消除 volatile 丢包陷阱）；默认输出话题改为标准契约话题 `/planning/pathlimits`；消除 `ComputeCircleCenterline` 中 Taubin 圆拟合的二次重复计算。
3. **#89** `[S2b] skidpad_planner 核心边界参数清洗与退化守卫（非法值回退默认与 WARN）`（P2，ready-for-agent）：
   - 纯函数 `SanitizeSkidpadConfig`：`center_margin`、`k_a`、`k_r`、`d_0`、`apf_step_limit` 非法值清洗，构造期 WARN，核心内部防除零兜底；补充 5+ 非法组合用例，消除清空锥桶与 NaN 路径隐患。

## in-review（待验收，0）

无。

## ready（可执行，3）

| Issue | 标题 | 优先级 | 模块 | 预估影响 |
|---|---|---|---|---|
| #88 | [S2b] TrackGenerator 补齐八字绕环完整中心线（右双圈+过渡切入+左双圈+出场） | P1 | `track_benchmark` | 赛道几何真值完善，基线评测前置 |
| #87 | [S2b] skidpad_planner 节点对齐契约 QoS 与接口话题规范（kQosPath / kQosState / 默认话题对齐） | P2 | `skidpad_planner` | 契约接口规范化，消除 QoS volatile 隐患 |
| #89 | [S2b] skidpad_planner 核心边界参数清洗与退化守卫（非法值回退默认与 WARN） | P2 | `skidpad_planner` | 算法边界鲁棒性，防除零与 NaN 路径 |

## in-progress（0）

无。

## 仓库外阻塞（7，均为外部输入缺失，非代码可推进）

| Issue | 标题 | 阻塞原因（D-013 分类） | 本轮复核 |
|---|---|---|---|
| #15 | 共用执行器适配层与车辆标定配置（P1） | **资料缺失**：VCU 协议文档、台架标定实值（D-016 已确认外部驱动不存在需自建） | `updatedAt` 10-19；性质已澄清，仍无外部资料输入 |
| #16 | 统一任务与安全状态、最终指令仲裁（P1） | **资料缺失**：硬件急停链路、实车停车预算 | `updatedAt` 10-05，无新评论 |
| #17 | 可复现仿真场景、可信 KPI 与自动回归（P1） | **资料缺失**：模型速度范围实测、感知扰动量化 | `updatedAt` 10-05，无新评论 |
| #19 | Pure Pursuit 低速基线与 MPC 公平对比（P1） | **资料缺失**：依赖 #17 扰动场景 | `updatedAt` 10-05，无新评论 |
| #18 | rosbag 数据集与感知质量评估（P2） | **资料缺失**：真实 bag（亦是 D-015 定位后端的前置） | `updatedAt` 10-05，无新评论 |
| #24 | 跨 ROS 版本语义差分回放与故障回归（P1） | **运行时缺失**：验收第 2 条需 ROS1 运行环境（本机无 Noetic、无 docker）；第 1 条切片已由 #79 完成关闭 | `updatedAt` 10-19；本体仍卡运行时 |
| #37 | 翻转 use_arbiter 默认（P3，needs-triage + blocked） | **资料缺失**：需 #15 标定完成 | `updatedAt` 10-09，无新评论 |

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

- **#86**（2026-10-21 验收关闭）：AGENTS.md/CLAUDE.md daemon 口径纠错（stop → 显式 start → sleep 3；组播因果句撤回具体断言）。
- **#85**（2026-10-21 验收关闭）：docs/MIGRATION_BASELINE.md 回填 D-014/D-015/D-016 决策引用；新增 docs/README.md 挂入核心文档索引。
- **#84**（2026-10-21 验收关闭）：straight_line_planner 时序滤波与分离参数非法值清洗（回退默认 + WARN，净增 5 用例）。
- **#83**（2026-10-21 验收关闭）：straight_line_planner 移除未读取死参数 hough_threshold（方案 A，纯 4 行删除）。
- **#82**（2026-10-21 验收关闭）：ROS 闭环完赛门禁 accel-finish（正样本 rc=0、负样本 rc=1、对照臂通过）。
- **#80**（2026-10-21 验收关闭）：直线加速完赛判据核（finish_judge.hpp + 门禁第 9 节断言；口径变更 → D-018）。
- **#81**（2026-10-21 验收关闭）：赛道元数据 finish_x/runout 进 TrackDefinition+tracks.json。
- **#79**（2026-10-21 验收关闭）：GeodeticToEnu 下沉 geodetic_core.hpp 纯 std 核。
- **#78**（2026-10-21 验收关闭）：hough_min_inlier_ratio 增 [0,1] 上界守卫消除 float→int UB。
- 更早：#77 / #62 / #23（2026-10-20）；#59 / #60 / #61（2026-10-12）；#55 / #56 / #57 / #58。

## 环境与流程注意

- 基线：`main` = `origin/main` = `b30af56`（**0/0 同步**）。
- 支持环境（D-017）：CI = ROS 2 **Jazzy**，本机 = **lyrical**；验证入口唯一为 `bash scripts/drive_gates.sh`。多节点 runtime 门禁必须 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`。
- 本轮已开工 S2b 首批队列，遵循常规 conventional commits，代码与 `.agent/` 提交分立。
