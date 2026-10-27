# STATE

- 更新时间：2026-10-27T08:40:00+08:00（exec-20261026 会话3 复核补记）
- 当前任务：**今日队列已清空** —— #96 / #69 / #95 三单均已合入 `main` 并转 `in-review`（等待 Planner 验收，Executor 不 close）
- 分支：`main`（= `origin/main`，工作区 clean）；两条临时分支均已删
- 未完成工作：
  - **无**（队列内三单全部合入 main；会话3 只做复核与交接补记，无代码改动）。
  - 待 Planner 处置（四项，已用 questionnaire 提交）：① **#92 的挂载收口**（闭环脚本正样本已绿，是否现在挂进 `drive_gates.sh` / `ci.yml`）；② **#92「末速 < 0.1 m/s」口径**（字面与本单「完赛/停 非目标」互斥，实测末速 2.947 m/s）；③ **#95 判据 1 的表述**（字面与 #90 真值互斥，需裁定改写）；④ **auto-discovered #100**（切点换瓣能力缺失，需赛道标记或产品决策）。
- 最近提交（`main`）：
  - `cc616e4` fix(gates): 就绪探测改为进程存活优先，消除 DDS 发现退化的假红（#98 合入，@ main）
  - `b4d2d1a` fix(planner): 走廊中线改用同心双壁瓣模型，切点不再产生跨瓣伪路径（#99 合入，@ main）
  - `7733cbb` chore(gates): 合入 skidpad 在线闭环数据面门禁脚本（保持未挂载）（#97 合入，@ main）
- 今日日报：已写（`.agent/DIARY/2026-10-26.md`，会话1 + 会话2 追加）
- CI 事实：
  - PR #97（#96）run `40336692425` attempt 2 = **success**；attempt 1 曾因 ros-apt-source 的 GitHub API 404 判红（基础设施抖动，re-run 绿）。
  - PR #98（#69）run `40355980015` attempt 3 = **success**；attempt 1/2 均为 **cancelled**（并发组 cancel-in-progress，非失败）。
  - PR #99（#95）run `40368098627` = **success**（对应 `8444ca0`）；`40367022848` = failure（clang-format 版本差异，已由 `f99951b` 修掉），`40367937681` = cancelled（被新 push 取消）。
  - `main` push run：`40422709344`（`2869018`，**当前最终 HEAD**）attempt 2 = **success**（attempt 1 卡死在「Prepare ROS 2 jazzy」22 分钟，已 cancel 后 re-run 绿）；`40420437594`（`ccad4d9`）= **success**；`40420320368`（`cc616e4`）= **success**；`40418013066`（`b4d2d1a`）被更新 push 取代（cancelled ≠ failure）。三次 PR 级 CI（#97 / #98 / #99）全部 **success**。
  - 会话3 追加事实：#92 重入条件已**实质满足**（#95 修复已合入 main；正样本实测绕角 6.2864 rad ≥ 2π、横偏 0.433 m ≤ 1.0 m、20.04 Hz、全有限、1399 帧 0 违例；负样本 rc=1）——证据已贴至 #92 单内评论；**唯一字面未达项是「末速 < 0.1 m/s」**（实测 2.947 m/s），已如实上报待裁定，未自行改判据。
- 已知环境限制：
  1. **多节点 runtime 门禁必须 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`**（漏设 → DDS 发现不收敛假 FAIL）。
  2. **单包构建跑测试必须显式 `-DBUILD_TESTING=ON`**。
  3. 本机 `/tmp` 只读；`ROS_HOME`/`TMPDIR` 由脚本指向 `build/`；验证前 `export PATH="$HOME/.local/bin:$PATH"`。
  4. 本机 sim 时间戳显示异常偏大（1793xxx）不影响判据（Watch 用墙钟预算、节点用 sim 钟对齐）。
  5. **本机 WSL2 仿真速度不足以在 180 s 预算内跑完 260 m 八字赛程**：`skidpad_finish_smoke.sh` 正样本臂在默认预算下本机红（A/B：`main` 上同样红）；`BUDGET_SEC=420` 后绿。CI（Jazzy）上同一门禁为绿。**不是代码问题。**
  6. `bash scripts/lint_shell.sh` 的 shellcheck 段本机跳过（未装 shellcheck，CI 的 apt 步骤已装且通过）。
  7. **本地 clang-format 是 21.1.8，CI 是 apt 旧版**：尾随注释对齐规则不同 → 本地绿、CI 红。新增/改动的 C++ 代码注释一律**独占行**。
  8. **CI 的「Prepare ROS 2 jazzy」步骤会挂起**（`ros-tooling/setup-ros@v0.7` 无 step 级超时）：实测卡住 22 分钟无输出（该步正常耗时 2.5 分钟）→ 判为基础设施抖动（**#72 已立案**）。处置：`gh run cancel <runId>` → `gh run rerun <runId>` → attempt 2 绿。**不是代码问题，不要因此改代码或改 CI 配置。**
