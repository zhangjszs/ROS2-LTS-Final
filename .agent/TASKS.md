# TASKS.md — Issue 本地镜像（权威：GitHub Issue；本文件由 Planner 每轮整体重写）

> 维护者：Planning Agent｜最后更新：2026-10-15（`plan-20261015` 轮：Issue 可用，共 **27 open**，与上轮持平，逐项核对一致后整体重写）
> 规则：本文件只读镜像，Executor 改状态时先改 Issue 再同步对应行；冲突以 Issue 为准。
> 说明：`.agent/DIARY/` 仍未建立、10-14 无 Executor 活动——昨日输入以 `HANDOFF.md` + `STATE.md` + git log 代行（见 PLAN §三）。

## 今日队列（2026-10-15，给 Executor 的建议顺序）

1. **#62** straight_line_planner 直线拟合核下沉为纯 std core 并纳入 sanitizer 门（P3，ready-for-agent）——唯一 ready（0 comment，`updatedAt` 停在 2026-10-12T13:39Z，本轮复核无变化），过 4.2 门禁，细节见 PLAN §四。

## ready（可执行，1）

| Issue | 标题 | 优先级 | 状态确认 |
|---|---|---|---|
| #62 | straight_line_planner 直线拟合核：Hough/RANSAC/LSQ/时序滤波下沉为纯 std core 并纳入 sanitizer 门 | P3 | ready-for-agent，2026-10-12T13:39Z 后无变化（本轮复核） |

## in-review（待验收，0）

（空——#59/#60/#61 已于 2026-10-12 验收关闭，无 reopen；#62 尚未开工。）

## in-progress（0）

（空，本轮 `gh` 实测。）

## 仓库外阻塞（8，无变化，updatedAt 停在 10-05/10-09，本轮复核）

| Issue | 标题 | 阻塞原因 |
|---|---|---|
| #15 | 共用执行器适配层与车辆标定配置（P1） | VCU 协议资料、台架标定实值 |
| #16 | 统一任务与安全状态、最终指令仲裁（P1） | 硬件急停链路、实车停车预算 |
| #17 | 可复现仿真场景、可信 KPI 与自动回归（P1） | 模型速度范围实测、感知扰动量化 |
| #19 | Pure Pursuit 低速基线与 MPC 公平对比（P1） | 依赖 #17 扰动场景 |
| #18 | rosbag 回放数据集与感知质量评估（P2） | 真实 bag |
| #23 | ROS1→ROS2 迁移清单（P1） | ROS1 环境 / 外部驱动确认 |
| #24 | 跨 ROS 版本语义差分回放与故障回归（P1） | 依赖 #23 + ROS1 环境 |
| #37 | 翻转 use_arbiter 默认（P3，needs-triage + blocked） | 需 #15 实车/台架标定完成后翻转 |

## needs-info / 待条件（3，D-007-B 生效中）

| Issue | 标题 | 等待 |
|---|---|---|
| #25 | 技术博客系列（P2） | #17①② / #18 证据 |
| #26 | 完整项目展示（P2，父跟踪） | #17①② / #18 证据 |
| #27 | 跨仓库路线总入口（P2） | 同上 |

## ready-for-human（14，用户跟进中，测试性能/质量批，2026-10-12/13 立项）

| Issue | 标题 | 优先级 |
|---|---|---|
| #63 | CI 增加路径过滤与并发取消 | P2 |
| #64 | check_cpp20.sh 统一构建参数 | P2 |
| #65 | closed-loop-fault 六臂并行化 | P2 |
| #66 | CI 拆分为 static / build-test / runtime-gates 三 job | P2 |
| #67 | affected_gates.sh 本地增量门禁 | P3 |
| #68 | ccache 缓存键纳入头文件通配 | P3 |
| #69 | DDS 门禁区分环境性失败与真失败 | P2 |
| #70 | 门禁时长预算监控 gate_budget_check.py | P3 |
| #71 | core_standalone 行覆盖率基线 | P3 |
| #72 | Prepare ROS 2 步骤级超时 | P3 |
| #73 | 评分核测试双轨去重 | P3 |
| #74 | 本地快速反馈文档节 | P3 |
| #75 | 测试编写规范入库 | P3 |
| #76 | colcon-only 下沉伞形跟踪 | P3 |

实施顺序与基线见 `docs/TEST_PERF.md` 第七节；TEST_PERF.md 幂等追加规则有效。均为人类工作，不进 Executor 队列。

## 路线图伞形（1，长期跟踪，无优先级标签系有意豁免）

| Issue | 标题 |
|---|---|
| #20 | ROS 2 车队开发路线：运行基线 → 可信仿真 → 低速闭环 → 性能优化 |

## 已关闭（近期备查）

- #59 / #60 / #61（2026-10-12 验收关闭，comment `6125381041` / `6125381825` / `6125382685`，本轮复核无 reopen）。
- #55（假失败）/ #56（PATH 误判零代码关闭）/ #57（最小展示入口）/ #58（CLAUDE.md 补一行）。
