# TASKS.md — Issue 本地镜像（权威：GitHub Issue；本文件由 Planner 每轮整体重写）

> 维护者：Planning Agent｜最后更新：2026-10-24T12:00+08:00（`plan-20261024`：验收关闭 S2b 首批 #88/#87/#89，立项第二批 #90/#91/#92 闭环门禁线）
> 规则：本文件为只读镜像，Executor 改状态时先改 Issue 再同步对应行；冲突以 Issue 为准。
> 权威来源状态：`gh issue list --state open` 实测 **28 open**（ready 3 · in-review 0 · in-progress 0 · 仓库外阻塞 7 · needs-info 3 · ready-for-human 14 · 路线图伞形 1）。

## 今日队列（2026-10-24 起有效，给 Executor 的建议顺序）

> **执行顺序建议（严格串行，依赖链 #90 → #91 → #92）**：
> - **#90（P1）** 纯 core 判据核 + 元数据 + 中心线导出，无 ROS 依赖，独立合入；
> - **#91（P1）** 依赖 #90 的判据核与中心线 CSV，复刻 #82 accel-finish 模式建真值驱动完赛门禁；
> - **#92（P2）** 依赖 #91 的拓扑 harness，把真值 pathsource 换成在线 skidpad_planner 做数据面门禁。
> - 三单落点有包含关系（#91 触 finish_event_node，#92 触 scripts），串行可免 rebase 冲突。

1. **#90** `[S2b] skidpad 完赛判据核：八字计圈状态机 + 赛道元数据 + 中心线导出（纯 std）`（P1，ready）
   - `TrackDefinition` 增 skidpad 元数据（圆心 ±9.125 / 半径 9.125 / 圈数 2）；纯 std `skidpad_finish_judge.hpp` 绕角累积计圈 + 相位状态机；`--export-tracks` 增中心线 CSV；退化轨迹单测。
2. **#91** `[S2b] skidpad_finish_smoke.sh：八字绕环 ROS 闭环完赛门禁（真值中心线驱动，可证伪）`（P1，ready）
   - pathsource 读 #90 中心线 CSV 发 `/planning/raw_pathlimits` → profiler → PP → arbiter → simulator（skidpad_track.csv）；finish_event_node 接 #90 判据核；断言 finished + REQUEST 停 + 横向偏差 ≤1.0m；负样本可证伪；接入 drive_gates 与 CI。
3. **#92** `[S2b] skidpad_planner 在线闭环接入冒烟：锥桶图驱动的八字数据面门禁`（P2，ready）
   - 以 #91 拓扑为基，pathsource 换在线 skidpad_planner（输出重映射喂 profiler）；断言节点注册 + ≥1Hz 有限 pathlimits + 右圆绕角 ≥2π + 横向 ≤1.0m；完赛为非目标（观察字段）；负样本屏蔽规划器必败。

## ready（可执行，3）

| Issue | 标题 | 优先级 | 模块 | 依赖 |
|---|---|---|---|---|
| #90 | [S2b] skidpad 完赛判据核：八字计圈状态机 + 赛道元数据 + 中心线导出（纯 std） | P1 | `track_benchmark` | 无 |
| #91 | [S2b] skidpad_finish_smoke.sh：八字绕环 ROS 闭环完赛门禁（真值中心线驱动，可证伪） | P1 | `scripts` + `track_benchmark` | #90 |
| #92 | [S2b] skidpad_planner 在线闭环接入冒烟：锥桶图驱动的八字数据面门禁 | P2 | `scripts` + `huat_launch` | #91 |

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

## needs-info / 待条件（3）

| Issue | 标题 | 等待 |
|---|---|---|
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

- **#88 / #87 / #89**（2026-10-24 验收关闭）：S2b 首批三单——八字完整中心线（259.836m，RMSE 0.411→0.320）/ 契约 QoS 对齐 + Taubin 去重 / 8 参数清洗双层防御（+7 用例）；三单 merge CI 全 success，证据独立复核通过。
- **#86**（2026-10-21）：AGENTS.md/CLAUDE.md daemon 口径纠错。
- **#85**（2026-10-21）：MIGRATION_BASELINE.md 决策引用回填。
- **#84**（2026-10-21）：straight_line_planner 时序滤波与参数清洗。
- **#83**（2026-10-21）：straight_line_planner 移除死参数。
- **#82**（2026-10-21）：accel-finish ROS 闭环完赛门禁。
- **#80 / #81 / #79 / #78**（2026-10-21）：完赛判据核 / 赛道元数据 / GeodeticToEnu 下沉 / houston 上界守卫。
- 更早：#77 / #62 / #23（2026-10-20）；#59 / #60 / #61（2026-10-12）；#55–#58。

## 环境与流程注意

- 基线：`main` = `origin/main` = `c7c55d6`（0/0 同步，工作树 clean）。
- **CI 尾巴待复核**：`c7c55d6`（10-24 凌晨 Executor 纯 `.agent/` 提交）run `40027760400` 本轮规划时仍在 in_progress——树等价绿 run（代码与 `be03070` 一致），预期无影响，下一棒开工前顺手 `gh run view 40027760400` 确认即可。
- 支持环境（D-017）：CI = ROS 2 **Jazzy**，本机 = **lyrical**；验证入口唯一为 `bash scripts/drive_gates.sh`；多节点 runtime 门禁必须 `export FASTDDS_BUILTIN_TRANSPORTS=SHM` + ros2 daemon 显式 start。
- 代码与 `.agent/` 提交分立（conventional commits，正文 `Refs #<任务号>`）。
