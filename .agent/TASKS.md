# TASKS.md — Issue 本地镜像（权威：GitHub Issue；本文件由 Planner 每轮整体重写）

> 维护者：Planning Agent｜最后更新：2026-10-25T22:00+08:00（`plan-20261025`：#90/#91 验收关闭，S2b 5/6，#92 唯一 ready）
> 规则：本文件为只读镜像，Executor 改状态时先改 Issue 再同步对应行；冲突以 Issue 为准。
> 权威来源状态：`gh issue list --state open` 实测 **26 open**（ready 1 · in-review 0 · in-progress 0 · 仓库外阻塞 7 · needs-info 3 · ready-for-human 14 · 路线图伞形 1）。

## 今日队列（2026-10-25 起有效，给 Executor 的建议顺序）

> **执行顺序建议**：仅 1 项——#92 是 S2b 最后一项（收口件）。开工前**必读** Issue #92 的 Planner 交接 comment（plan-20261025）：#91 三条实测发现（PP 重锚劫持处置边界 / watchdog 覆盖按在线实测定 / Jazzy 订阅计数陷阱用 profiler 转发确认）。
> 调查先行：先实测 `src/planning/skidpad_planner` 输出几何/频率，再定数据面与门禁断言（恢复点见 HANDOFF「未完成」节 + STATE.md「#91 关键实测事实」）。

1. **#92** `[S2b] skidpad_planner 在线闭环接入冒烟：锥桶图驱动的八字数据面门禁`（P2，**needs-info**）
   - 以 #91 拓扑为基，pathsource 换在线 skidpad_planner（`output_pathlimits_topic` 重映射喂 profiler）；因规划器跨圆平均缺陷（见 #95）阻断车辆右圆推进，门禁脚本已落地分支但转 needs-info 待算法重构。

## ready（0）

无。

## in-review（0）

无。

## in-progress（0）

无。
## 仓库外阻塞（7，均为外部输入缺失，非代码可推进）

| Issue | 标题 | 阻塞原因（D-013 分类） | 本轮复核 |
|---|---|---|---|
| #15 | 共用执行器适配层与车辆标定配置（P1） | **资料缺失**：VCU 协议文档、台架标定实值（D-016 已确认外部驱动不存在需自建） | 无外部输入 |
| #16 | 统一任务与安全状态、最终指令仲裁（P1） | **资料缺失**：硬件急停链路、实车停车预算 | 无外部输入 |
| #17 | 可复现仿真场景、可信 KPI 与自动回归（P1） | **资料缺失**：模型速度范围实测、感知扰动量化（切片 #79 已关，第 2 条卡 ROS1 运行时归 #24） | 无外部输入 |
| #19 | Pure Pursuit 低速基线与 MPC 公平对比（P1） | **资料缺失**：依赖 #17 扰动场景 | 无外部输入 |
| #18 | rosbag 数据集与感知质量评估（P2） | **资料缺失**：真实 bag（亦是 D-015 定位后端的前置） | 无外部输入 |
| #24 | 跨 ROS 版本语义差分回放与故障回归（P1） | **运行时缺失**：验收第 2 条需 ROS1 运行环境（本机无 Noetic、无 docker） | 无外部输入 |
| #37 | 翻转 use_arbiter 默认（P3，needs-triage + blocked） | **资料缺失**：需 #15 标定完成 | 无外部输入 |

## needs-info / 待条件（4）

| Issue | 标题 | 等待 |
|---|---|---|
| #92 | [S2b] skidpad_planner 在线闭环接入冒烟：锥桶图驱动的八字数据面门禁 | #95 规划器跨圆平均与点序回跳算法重构 |
| #25 | 技术博客系列（P2） | #17①② / #18 证据 |
| #26 | 完整项目展示（P2，父跟踪） | 同上 |
| #27 | 跨仓库路线总入口（P2） | 同上 |
## ready-for-human（14，用户/人工跟进，不计入 agent 队列与饥饿规则）

#63 CI 路径过滤与并发取消（P2）· #64 `check_cpp20.sh` 参数统一（P2）· #65 closed-loop-fault 六臂并行（P2）· #66 CI job 三段拆分（P2）· #67 `affected_gates.sh`（P3）· #68 ccache 键头文件通配（P3）· #69 DDS 环境性失败区分（P2，含 daemon 预热回填其余 5 脚本范围 + D-021 两类环境陷阱）· #70 门禁时长预算监控（P3）· #71 core_standalone 覆盖率基线（P3）· #72 Prepare ROS 2 步骤超时（P3）· #73 评分核测试双轨去重（P3）· #74 本地快速反馈文档（P3）· #75 测试编写规范入库（P3）· #76 伞形：12 个仅 colcon 轨测试下沉（P3）

## 路线图伞形（1，长期跟踪，无优先级标签系有意豁免）

| Issue | 标题 |
|---|---|
| #20 | ROS 2 车队开发路线：运行基线 → 可信仿真 → 低速闭环 → 性能优化 |

## 已关闭（近期备查）

- **#90 / #91**（2026-10-25 验收关闭）：S2b 闭环门禁线前两单——判据核（元数据+绕角计圈+中心线 349 点）/ 真值完赛门禁（61.2s 正样本 + 双负样本可证伪 + CI step）；Planner 对 12 条验收标准独立复核（tracks.json/CSV/JSON 留痕/挂载点/diff 范围亲验），PR #93/#94 CI 全绿。
- **#88 / #87 / #89**（2026-10-24 验收关闭）：S2b 首批三单——八字完整中心线（259.836m，RMSE 0.411→0.320）/ 契约 QoS 对齐 + Taubin 去重 / 8 参数清洗双层防御（+7 用例）。
- **#86**（2026-10-21）：AGENTS.md/CLAUDE.md daemon 口径纠错。
- **#85**（2026-10-21）：MIGRATION_BASELINE.md 决策引用回填。
- **#84**（2026-10-21）：straight_line_planner 时序滤波与参数清洗。
- **#83**（2026-10-21）：straight_line_planner 移除死参数。
- **#82**（2026-10-21）：accel-finish ROS 闭环完赛门禁。
- **#80 / #81 / #79 / #78**（2026-10-21）：完赛判据核 / 赛道元数据 / GeodeticToEnu 下沉 / houston 上界守卫。
- 更早：#77 / #62 / #23（2026-10-20）；#59 / #60 / #61（2026-10-12）；#55–#58。

## 环境与流程注意

- 基线：`main` = `origin/main` = `8f159ab`（0/0 同步，工作树 clean）。
- **CI 状态**：PR #93/#94 merge run（`40101065937`/`40111773587`）与 docs run `40111929415`/`40112805739` 全部 success；push run `40113468765` 超时取消与 Nightly `40128214718`（daemon 超时假红）均已重新触发。
- **延后候选（已定策，见 PLAN §八）**：PP 锚点保留（等 #92 实测证实）/ speed_high_threshold 分场景（无活体消费方）/ accel-finish 补挂 PR CI（等 #63/#66 CI 提效）。Executor 勿在 #92 内顺手做这三项。
- 支持环境（D-017）：CI = ROS 2 **Jazzy**，本机 = **lyrical**；验证入口唯一为 `bash scripts/drive_gates.sh`；多节点 runtime 门禁必须 `export FASTDDS_BUILTIN_TRANSPORTS=SHM` + ros2 daemon 显式 start；Jazzy 下 `get_subscription_count()` 不刷新（#91 CI 实测），就绪探测用 profiler 转发确认。
- 代码与 `.agent/` 提交分立（conventional commits，正文 `Refs #<任务号>`）。
