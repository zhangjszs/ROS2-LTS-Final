# STATE.md — 时间流接力状态（当前活跃 Agent）

> agent-id：`glm-20261002T111615Z`｜时间范围：2026-10-02T11:16Z –（进行中）｜环境：本机 lyrical（CI 是 Jazzy）
> 上一棒：`qoder-20260929T231407Z`（轮 1–3 见 git 历史 a36ebb2 收尾；其轮 3 后追加了 `0c24c11` 修复 #51 并关闭，但未及写台账即中断）。
> 环境探测缓存见 `.agent/ENV.md`（人工可编辑覆盖）。

## 当前活跃

- 无（本轮收尾中，时间范围 2026-10-02T11:16Z – 2026-10-02T12:05Z）。下一棒序 1 = **修后正式选参**（见下）。

## 本轮（glm-20261002T111615Z）已完成

| 轮 | 任务 | commit / issue | CI |
|---|---|---|---|
| 1 | **Nightly 36933923349 假红根因修复**：①`safeStopResult` 手写停车帧漏设 `racing_status`（默认 0 上线，违反 makeSafeStopRaw"禁止手写停车帧"契约）→ 补 =4 + 单测钉住；②fault_injection 重启的仲裁器成孤儿、污染下道门禁出口 → `start_new_session`+killpg 整组清理 + pidfile 兜底；③mpc_reject_smoke 采样前新增 `/vehicle_command` 发布者数=1 纯度预检。本机：safety_monitor 61 测全绿、两臂冒烟 PASS、fault_injection 10/10 无残留、drive_gates 14/14（505s） | `2c9a4e6` | push CI 37002942307 **success**（Nightly 假红消除，待 10-02 晚 schedule 复证） |

## 历史摘要（详情见 git 历史 HANDOFF/STATE 版本）

- `qoder-20260929T231407Z`：#48 关闭（`1e70583`）、#50 关闭（`466c8ff`）、调参腿 r3 不采纳（`746e9e6`）、#51 关闭（`0c24c11`，C6 收紧为物理指标+1% 门槛；issue 已 CLOSED 但无证据评论）。
- `relay-2026-09-29-A`：#17 ③④（afbfa65/8d94444/69aec5b）。**#17 不得用仿真证据关闭**。

## 活跃任务 / 待办（按优先级）

1. **修后正式选参**：#51 收紧已落地，重扫网格（`bash scripts/tuning_driver.sh`，<5min）→ 有真实赢家才动参数（qp_solver.hpp 默认 + mpc 节点 declare + test_param_consistency 同步 + 全门 + 比较集一次、命令禁旋钮）；无赢家则 docs §9 追加记录。比较集评估权仍未消费。
2. #17/#19/#15/#16 软件侧收口（缺标定/台架，不为关闭跳过硬验收）。
3. #23/#24/#18/#25/#26/#27：需人工输入，无输入不臆造。
4. #37（needs-triage → 按规则跳过，需人工）。

## 门禁状态（轮 1 收尾）

- 本机全量 drive_gates：**14/14 PASS、hard_fail=0、505s**。
- colcon test：safety_monitor 61/61（新增 racing_status 钉子）；全仓 test-run/test-enforce 门 PASS。
- `benchmarks/baseline/` 未触碰；改共用层后 benchmark 门 PASS（benchmark_regression 含其中）。

## 环境事实（沿用上棒，本轮复验成立）

- 多节点脚本前 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`、`ROS_HOME=$PWD/build/.ros`；跑 scripts 前 source ROS **与** install/setup.bash。
- 清理只按 PID/PGID；**严禁 `pkill -f <节点名>`**。注意 `pgrep -x` 对 >15 字符进程名（如 command_arbiter_node）恒空 → 用 comm 截断名（command_arbiter）。
- 断言型 ros2 查询走 daemon（`ros2 topic info` 不可 --no-daemon）。
- shellcheck 在 `build/sc_root/usr/bin`，PATH 前缀后跑 lint_shell。
- 沙箱 /tmp 受限 → 临时产物写 `build/`。
- gh `run watch --interval` 不存在 → 轮询 `gh run view --json status`。
