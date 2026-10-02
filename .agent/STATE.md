# STATE.md — 时间流接力状态（当前活跃 Agent）

> agent-id：`exec-20261002T1509Z`｜时间范围：2026-10-02T15:09Z –（进行中）｜环境：本机 lyrical（CI 是 Jazzy）
> 上一棒：`plan-20261002T141500Z`（建 PLAN/DECISIONS、治理 ready 队列 #52/#53/#54，`06ce1d6`）。
> 环境探测缓存见 `.agent/ENV.md`。

## 当前活跃

- **#54**（ready, P3, chore/simulation）：sensor_simulator 感知几何核下沉纯 std core。进行中。

## 本轮（exec-20261002T1509Z）进度

| 序 | Issue | 状态 | 代码 commit | 验证 |
|---|---|---|---|---|
| 1 | #53 拆 bicycle_model 纯 std 单测进 #40 sanitizer 门 | **完成**（待 Planning 验收） | `eb75d08` | core_standalone 24/24；colcon 484/0 fail；lint 通过 |
| 2 | #52 `warm_start` 死配置生效 + 单测 | **完成**（待 Planning 验收） | `fe2d97b` | core_standalone 24/24；colcon 486/0 fail；lint 通过 |
| 3 | #54 sensor_simulator 感知核下沉纯 std core | **进行中** | — | — |

## 执行备注（下一棒注意）

- #53（`eb75d08`）、#52（`fe2d97b`）代码均已推送；本棒未关闭任何 issue（由 Planning 验收裁定）。
- #54 设了原生依赖 `blocked_by #53`；因用户指令要求本轮按 #53→#52→#54 连续执行，本棒以"#53 代码完成"为准继续执行 #54。
- 本机 ROS=lyrical（CI 是 Jazzy）：多节点脚本前 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`、`ROS_HOME=$PWD/build/.ros`；跑 scripts 前 source `/opt/ros/lyrical/setup.bash` 与 `install/setup.bash`。
- 改动 C++ 后先 `clang-format -i <file>`（否则 `lint_cpp.sh` 判红：clang-format 是必查项）。
- 清理只按 PID/PGID；严禁 `pkill -f <节点名>`；`pgrep -x` 对 >15 字符进程名恒空。

## 历史摘要（详情见 git 历史 HANDOFF/STATE 版本）

- `plan-20261002T141500Z`：建 PLAN/DECISIONS；#52 转 ready（P3/bug）、新建 #53(P2)/#54(P3)（`06ce1d6`）。
- `exec-20261002T141121Z`：ready 扫描 0 个 → 建 auto-discovered #52（`c72bb67`）。
- `glm-20261002T125512Z`：#15/#16/#17/#19 证据映射审计（`b0a8cfe`）。

## 活跃任务 / 待办（按优先级）

1. 本轮队列：#54（进行中）。
2. 等待人工/硬件输入：`#15/#16/#17/#19`（关闭条件见各 issue 评论）、`#18/#23/#24/#25/#26/#27`、`#37`（needs-triage）。

## 门禁状态（本轮）

- 本机实测：#53 与 #52 各自 core_standalone 24/24 通过、colcon test 全绿（#52 后 486/0 fail）、`lint_cpp.sh` 通过。
- 主干 CI：`fe2d97b` push 后待 CI 结果。

## 环境事实（沿用）

- 多节点脚本前 export FASTDDS_BUILTIN_TRANSPORTS=SHM、ROS_HOME=$PWD/build/.ros；source ROS **与** install/setup.bash；本机 ROS=lyrical。
- 构建：`MAKEFLAGS=-j4 colcon build --packages-select <pkg> --symlink-install --cmake-args ...`（CLI --cmake-args 会整体替换 defaults.yaml 同名列表，须重列 mold/ccache）。
