# HANDOFF.md — 给下一棒（relay 时间点交接）

当前棒：`exec-20261002T141121Z`（2026-10-02，单轮 ready 扫描 + 台账）。上一棒
`glm-20261002T125512Z` 完成 #15/#16/#17/#19 软件侧证据映射审计（`b0a8cfe`）。
先读 `.agent/STATE.md` 与 `.agent/ENV.md`。

## 一、本棒做了什么

1. **ready-issue 扫描**：全仓 `gh issue list -L 100 --state all`，按 `ready-for-agent`
   过滤——**0 个 ready issue**。open issue 现状：
   - `needs-triage`：`#37`（use_arbiter 默认需实车标定后，规则跳过）；
   - 路线图 / 待人工输入：`#15/#16/#17/#19`（软件侧已扫尽）、`#18/#23/#24/#25/#26/#27/#20`。
2. **auto-discovered（规则上限 1 条）**：确认 `QpSettings::warm_start` 为死配置并建
   **#52**（`needs-triage`，普通 open、**未定级、未 ready**）。证据：
   `qp_solver.hpp:20`（声明）vs `qp_solver.cpp:34-45`（只用 `warm_x`/内部 `z_/y_`，
   从不读 `settings_.warm_start`）。
3. **未改任何业务代码**；本轮零代码改动（无合规 ready issue，按规则不臆造任务）。

## 二、结论：本棒无可执行 ready issue

软件侧待办延续上一棒判断——**全部卡人工/硬件输入**，没有合规的下一步。
下一棒的第一动作应是**重新拉取 issue 状态**，看 Planning Agent 是否已新建/治理
ready issue（尤其 `#52` 是否被定级并转 `ready-for-agent`）。

## 三、下一棒的选题顺序

| 序 | 动作 | 注意 |
|---|---|---|
| 0 | 重跑 `gh issue list --label ready-for-agent --state open` | 若空且无 in-progress，做轻量维护轮 |
| 1 | 若 `#52` 被 Planning Agent 转 ready → 按该 issue 验收条件实现（warm_start 语义二选一 + 单测） | 未定级前**不得开修** |
| 2 | 等输入类：四条 issue 的剩余项若拿到实车/硬件证据，按各 issue 评论关闭条件接续 | 无输入不臆造 |
| 3 | `#24`（跨 ROS 版本语义差分回放）若 ROS1 环境可用可软件侧先行 | 需人工确认 ROS1 环境 |
| — | `#37` needs-triage | 按规则跳过 |

## 四、红线（全部仍然有效）

- 别用过仿真证据关闭 `#17/#15/#16/#19`——各 issue 评论已写明关闭条件。
- 别看过结果后改判据；判据变更走"重声明（新 round + 预注册）→ 重放/重扫"。
- 别动 `benchmarks/baseline/`；别往 `fsac.benchmark.kpi/v1` 加字段；改共用层必须 benchmark 门 rc=0。
- `#52` 未定级前不得直接改 `warm_start`。
- 别 force push / 自动 merge / main 上临时放宽门禁。
- 清理进程只按 PID/PGID；严禁 `pkill -f <节点名>`；`pgrep -x` 对 >15 字符进程名恒空。

## 五、本棒产物 / 复现

```bash
# 本棒 ready 扫描输出为空（证明 0 个 ready issue）：
gh issue list --state all -L 100 --json number,state,labels \
  --jq '.[]|select([.labels[].name]|index("ready-for-agent"))|"\(.number) \(.title)"'
gh issue view 52 --comments          # 本棒新建的 auto-discovered
git show --stat HEAD                 # 本棒 chore(agent) 台账提交
```

- 本棒代码 commit：**无**（零代码改动）。
- 本棒 agent 状态 commit：见 `git log -1 --format=%h`（`chore(agent): exec-20261002T141121Z ...`）。
- 远程：已 push 到 `origin/main`；本棒起点 `b0a8cfe`。
