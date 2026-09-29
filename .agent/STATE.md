# STATE.md — 时间流接力状态（当前活跃 Agent）

> agent-id：`relay-2026-09-29-A`｜时间范围：2026-09-29T14:20Z – 16:35Z｜环境：本机 lyrical（CI 是 Jazzy）
> 上一棒台账：`.omc/unattended/{progress,plan,decisions}.md`（`.omc/` 被 gitignore；本文件入 git，是接力主状态源）。

## 本轮（一次会话内三轮）做完的事

| 轮 | 任务 | commit / issue | CI |
|---|---|---|---|
| 1 | 同步：把上一棒**遗留未推送的 9 个 commit**（#38–#46）推上 main | `6e0d68d..e8822d5` | 36582919764 → **失败**（发现 #49） |
| 1 | 新建并修复 **#49 [P0]**：三条集成门禁的节点就绪判据从 `ros2 node list --no-daemon` 改回 daemon 图查询 + `ros2 daemon stop` 预清理 + 闭环故障门禁的场景间图排空等待 | `c56c0a4` | 36586172764 → **success**，#49 已关闭 |
| 2 | **#47 [P1]** 根因测量：前提**证伪**（判据在实例级严格单调），交付定级可观测性 + 单调性守护 + `docs/MPC_TUNING_FREEZE.md` §7 复盘 | `d0ecb52` | push 36591703745 success + 手动 nightly 36591912422 success → **#47 已关闭** |
| 2 | 新建 **#50 [P2]**：MPC 持续拒解在运行期不可见（由 §7.4 对照实验暴露） | issue #50 | 待做 |
| 3 | **#41 [P2]** 收尾：shellcheck-present 分支（CI 0.9.0 + 本机 0.11.0）与 `--strict` 两个分支都补了实测证据 | 已关闭 | — |

## #47 的关键结论（**别再重复调查**）

1. 取 trackdrive 闭环（`--corridor-scale 0.9`，k=150）产出的**同一批 725 个 QP 实例**冷启动重放，
   `max_iter` 50/150/300 的接受数 274 → 287 → 307，**逐实例反例 0 个** ⇒ "接受判据非单调"不成立。
2. 19.45% → 41.39% 是**跨轨迹**比率：`solves` 分母本身在变（725/1196/980），且解更准 ⇒ 车更快
   （平均车速 10.9 → 16.7 m/s）⇒ 更早出走廊 ⇒ 后续 QP 更难。闭环混沌，不是求解器毛病。
3. **"未收敛率"只数拒收拍**。真实拆分（trackdrive@0.9，k=50，725 拍）= 457 converged +
   127 accepted_approx + 141 failures。兜底带 `acceptable_primal=0.25` 是 50 维控制向量 ∞-范数的
   绝对量；单测里一个默认参数实例残差 0.079 ≈ 自身收敛目标的 750 倍，且 > 本拍下发的转角 0.060。
4. **收紧判据不是修复**（实测）：只收 `converged` ⇒ k=50 首拍即拒 ⇒ 此后状态不变、**全程零指令 14.5s**；
   150/300 档 `max|lat|` 恶化到 33.9/45.1 m。兜底带现在是承重墙。
5. `best-iterate`（历史最优残差）是死代码：冷启动单次运行里终端残差不劣化（0 反例），best == terminal。

## 门禁状态（HEAD `d0ecb52`）

- 本机全量：`lint_cpp` rc=0、`lint_shell` rc=0、`colcon build` rc=0（19 包）、
  `colcon test` **475 tests / 0 fail**、`core_standalone_check` 全绿（ASan+UBSan）、
  `benchmark_regression.sh` rc=0（三条 v1 基线 + profiler 第三臂基线**逐字节一致**）、
  `drive_gates.sh` **12/12 PASS、hard_fail=0（558s）**。
- 三条容差型集成门禁（Closed-loop simulation smoke / Closed-loop fault samples / Fault injection smoke）
  **已经是硬阻断**（`d2ef36c feat(#17): 三条容差型集成门禁由观察期转正为 CI 硬阻断`，
  `ci.yml` 里已无 `continue-on-error`）。#49 因此不是“观察期假失败”而是**硬门假失败 = 主干红**；
  现已修复，累计样本：ci.yml 连续 2 次 success（36586172764 @c56c0a4、36591703745 @d0ecb52），
  nightly 1 次全绿（36591912422 @d0ecb52，drive_gates 12/12 PASS、hard_fail=0、562s）。
  “转正”这件事没有待办了；剩下的只有**抗抖动：把“断言型查询不得用 --no-daemon”做成机检**（见 HANDOFF 二.4）。

## 环境事实（本机 ≠ CI，踩过才写下来的）

- 本机 lyrical，UDP 组播被 VPN 挡 → 多节点脚本前必须 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`。
- 断言型查询一律走 daemon；`--no-daemon` 在新进程里立即查图 → 空表假失败（#49 的根因）。
- 本机没有 shellcheck 且装不上系统包：可用路 = `apt-get download shellcheck && dpkg -x → build/sc_root/`
  再 `PATH=$PWD/build/sc_root/usr/bin:$PATH bash scripts/lint_shell.sh`。死路 = `pip --user`（PEP 668）、
  `python3 -m venv`（ensurepip 失败）。CI 是 0.9.0，本机解出来的是 0.11.0（建议层，不影响门禁）。
- `colcon build` 必须重列 mold/ccache（CLI `--cmake-args` 会整体替换 `colcon_defaults.yaml` 的同名列表），
  `MAKEFLAGS=-j4`；`colcon test --packages-ignore hello_world`（不带会假失败）。
- 沙箱 /tmp 只读 → 临时产物写 `build/`（本轮探针在 `build/probe47/`、`build/p47/`，都不入库）。
- 跑 `scripts/*.sh` 前先 source `/opt/ros/lyrical/setup.bash` 与 `install/setup.bash`，
  否则 `benchmark_regression.sh` 会报 `Package 'track_benchmark' not found` 而**误判成回归失败**。
