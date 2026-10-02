# STATE.md — 时间流接力状态（当前活跃 Agent）

> agent-id：`exec-20261002T141121Z`｜时间范围：2026-10-02T14:11Z –（收尾中）｜环境：本机 lyrical（CI 是 Jazzy）
> 上一棒：`glm-20261002T125512Z`（#15/#16/#17/#19 证据映射审计，`b0a8cfe`）。
> 环境探测缓存见 `.agent/ENV.md`（人工可编辑覆盖）。

## 当前活跃

- 无（本轮为 ready-issue 扫描 + 台账，无代码任务）。

## 本轮（exec-20261002T141121Z）已完成

| 轮 | 任务 | 结论 | 证据 |
|---|---|---|---|
| 1 | **ready-issue 扫描**：全仓 `gh issue list -L 100 --state all`，按 `ready-for-agent` 过滤 | **0 个 ready issue**；唯一带标签的 open issue 是 #37（needs-triage，规则跳过），其余为路线图/待人工输入 | 本轮 `gh issue list` 输出 |
| 2 | **auto-discovered（规则上限 1 条）**：确认 `QpSettings::warm_start` 死配置 | 建 **#52**（挂 `needs-triage`，普通 open，**未定级/未 ready**） | #52；源码 `qp_solver.hpp:20` vs `qp_solver.cpp:34-45` |

## 执行备注（下一棒注意）

- 本轮结论：**无 ready issue 可执行**，软件侧待办延续上棒判断（全部卡人工/硬件输入）。按规则不臆造代码任务。
- 沙箱写类 Bash 本轮可用（`gh issue create` / commit / push 均正常）。
- 别重复 #15/#16/#17/#19 的软件侧审计；按各 issue 评论里的关闭条件等人工/硬件输入。
- 新建 tracked 项 #52（`warm_start` 死配置）**未 ready**：不得直接开修，须等 Planning Agent 定级/定范围。

## 历史摘要（详情见 git 历史 HANDOFF/STATE 版本）

- `glm-20261002T125512Z`：#15/#16/#17/#19 证据映射审计（`b0a8cfe`），四条保持 OPEN。
- `glm-20261002T120930Z`：r5 重扫 0/81 通过、调参腿收口（docs §11.3）。
- `glm-20261002T111615Z`：Nightly 假红根因修复，CI 37002942307 success。
- `qoder-20260929T231407Z`：#48/#50/#51 关闭（`0c24c11`）。
- `relay-2026-09-29-A`：#17 ③④（afbfa65/8d94444/69aec5b）。

## 活跃任务 / 待办（按优先级）

1. **等待人工/硬件输入**：`#15`（VCU 协议 + 台架标定）、`#16`（硬件急停 + 停车预算）、`#17`（模型速度范围实测 + 感知扰动量化）、`#19`（B 阶段完整公平对比）。
2. `#23/#24/#18/#25/#26/#27`：需真实 bag / ROS1 环境 / 外部驱动 / 人工决定展示范围。
3. `#37`（needs-triage）：规则跳过，需人工。
4. `#52`（本轮新建，needs-triage）：`QpSettings::warm_start` 死配置——待 Planning Agent 定级；未定级前不得开修。

## 门禁状态（本轮）

- 主干绿：CI 37005963746 @`0f96dc8` success；本轮起点 HEAD `b0a8cfe`。
- 本轮零代码改动，未触发 build/test（无变更可验）。

## 环境事实（沿用，本轮复验成立）

- 多节点脚本前 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`、`ROS_HOME=$PWD/build/.ros`；跑 scripts 前 source ROS 与 install/setup.bash。
- 清理只按 PID/PGID；严禁 `pkill -f <节点名>`；`pgrep -x` 对 >15 字符进程名恒空。
- 沙箱 /tmp 受限 → 临时产物写 `build/`。gh `run watch --interval` 不存在 → 轮询 `gh run view --json status`。
- `tuning_driver.sh` 续跑语义：旧 workdir 有产物会跳过实跑，强制重跑须换 `--workdir`。
