# STATE

- 更新时间：2026-11-13T01:20:00+08:00（exec-20261113 · 恢复 #101 收尾补交执行报告 + 同步 `eee6b39`）
- 当前任务：**今日队列已清空**——#101 → `in-review`（PR #104 已合入 `main` = `eee6b39`，执行报告已贴 Issue）、#100 → `in-review`（第一阶段调研已交）
- 分支：`main` = `eee6b39`（= `origin/main`）；已合并临时分支 `agent/issue-101-initial-lobe-prior` 已删除（本地与远端）；保留 `agent/issue-92-skidpad-closed-loop-smoke`（#92 未收口）与用户研究分支 `origin/research/ros1-pointcloud-shift`（Draft PR #105，不动）
- 未完成工作：无（队列 `ready-for-agent` = 0、`in-progress` = 0；#92 挂载前置已满足，待 Planner 验收关闭 #101 并把 #92 转 `ready-for-agent`）
- 最近提交（`main`）：
  - `eee6b39` fix(skidpad): #101 首瓣先验 + 门禁改带符号绕角判据（Planner 裁定 A） (#104)
  - `c1ac495` chore(agent): exec-20261028——#101 转 needs-info（选瓣歧义）、#100 调研交 in-review、补记 10-27 日报
  - `079f58f` fix(skidpad): #101 走廊模型退化的持久化修复——可信模型跨帧保持，路径不再塌缩 (#103)
- 今日日报：已写 `.agent/DIARY/2026-11-13.md`（含对 10-28 第二会话与 11-12 合入动作的现场恢复记录）
- CI 事实：
  - `main` 的 `eee6b39` push run `43143150151` = **success**（合入 #104 之后）。
  - PR #104 CI run `40726189514` = **success**。
  - Nightly 截至 `2026-11-11T23:57Z`（run `43123706038`）连续 **success**。
- 已知环境限制：
  1. **多节点 runtime 门禁必须 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`**（漏设 → DDS 发现不收敛假 FAIL）。
  2. **单包构建跑测试必须显式 `-DBUILD_TESTING=ON`**。
  3. 本机 `/tmp` 只读；`ROS_HOME`/`TMPDIR` 由脚本指向 `build/`；验证前 `export PATH="$HOME/.local/bin:$PATH"`。`benchmark_runner --export-tracks` 也只能写到 `build/` 下。
  4. 本机 sim 时间戳显示异常偏大（1793xxx）不影响判据。
  5. **本机 WSL2 仿真速度不足以在 180 s 预算内跑完 260 m 八字赛程**（`skidpad_finish_smoke.sh` 正样本默认预算红是既有环境问题，`BUDGET_SEC=420` 绿）。
  6. `bash scripts/lint_shell.sh` 的 shellcheck 段本机跳过（未装 shellcheck）。
  7. **本地 clang-format 21.1.8 vs CI apt 旧版**：尾随注释对齐规则不同 → 注释一律独占行。
  8. **CI 的「Prepare ROS 2 jazzy」会挂起**（#72）：卡 20+ 分钟即 `gh run cancel` + `gh run rerun`，不要改代码/CI。
  9. **`skidpad_closed_loop_smoke.sh` 单跑 26–60 s；跑测期间勿并行构建/测试**（会拖慢墙钟、改变判据边界）。
