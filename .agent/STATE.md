# STATE.md — 时间流接力状态（当前活跃 Agent）

> agent-id：`exec-20261002T1509Z`｜时间范围：2026-10-02T15:09Z –（进行中）｜环境：本机 lyrical（CI 是 Jazzy）
> 上一棒：`plan-20261002T141500Z`（建 PLAN/DECISIONS、治理 ready 队列 #52/#53/#54，`06ce1d6`）。
> 环境探测缓存见 `.agent/ENV.md`。

## 当前活跃

- **#52**（ready, P3, bug/actuator）：`QpSettings::warm_start` 死配置 → 让字段生效 + 纯 std 单测。进行中。

## 本轮（exec-20261002T1509Z）进度

| 序 | Issue | 状态 | 代码 commit | 验证 |
|---|---|---|---|---|
| 1 | #53 vehicle_simulator 拆 bicycle_model 纯 std 单测进 #40 sanitizer 门 | **完成**（待 Planning 验收） | `eb75d08` | core_standalone 24/24；colcon 484/0 fail；lint 通过；纯度 grep clean |
| 2 | #52 `warm_start` 死配置生效 + 单测 | **进行中** | — | — |
| 3 | #54 sensor_simulator 感知核下沉纯 std core | 未开始 | — | — |

## 执行备注（下一棒注意）

- #53 代码已推送 `eb75d08`；#54 设了 GitHub 原生依赖 `blocked_by #53`，本用户指令要求本轮按 #53→#52→#54 连续执行，故依赖以"#53 代码完成"为准继续。
- 本机 ROS=lyrical（CI 是 Jazzy）：多节点脚本前 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`、`ROS_HOME=$PWD/build/.ros`；跑 scripts 前 source `/opt/ros/lyrical/setup.bash` 与 `install/setup.bash`。
- 清理只按 PID/PGID；严禁 `pkill -f <节点名>`；`pgrep -x` 对 >15 字符进程名恒空。
- 沙箱 /tmp 受限 → 临时产物写 `build/`。

## 历史摘要（详情见 git 历史 HANDOFF/STATE 版本）

- `plan-20261002T141500Z`：建 PLAN/DECISIONS；#52 转 ready（P3/bug）、新建 #53(P2)/#54(P3)（`06ce1d6`）。
- `exec-20261002T141121Z`：ready 扫描 0 个 → 建 auto-discovered #52（`c72bb67`）。
- `glm-20261002T125512Z`：#15/#16/#17/#19 证据映射审计（`b0a8cfe`）。
- `glm-20261002T120930Z`：r5 重扫 0/81 通过、调参腿收口。

## 活跃任务 / 待办（按优先级）

1. 本轮队列：#52（进行中）→ #54。
2. 等待人工/硬件输入：`#15/#16/#17/#19`（关闭条件见各 issue 评论）、`#18/#23/#24/#25/#26/#27`、`#37`（needs-triage）。

## 门禁状态（本轮）

- 主干 CI：`06ce1d6` 及之前 success；`eb75d08` push 后待 CI。
- 本机实测：`core_standalone_check.sh` 24/24 通过；`colcon test`（vehicle_simulator）484/0 fail；`lint_cpp.sh` 通过。

## 环境事实（沿用）

- 多节点脚本前 export FASTDDS_BUILTIN_TRANSPORTS=SHM、ROS_HOME=$PWD/build/.ros；source ROS **与** install/setup.bash；本机 ROS=lyrical。
- 构建：`MAKEFLAGS=-j4 colcon build --symlink-install --cmake-args ...`（CLI --cmake-args 会整体替换 defaults.yaml 同名列表，须重列 mold/ccache）。
