# STATE

- 更新时间：2026-10-28T22:18:00+08:00（exec-20261028 · #101 收口 + #100 调研）
- 当前任务：**今日队列已清空**——#101 → `needs-info`（选瓣输入来源待裁定）、#100 → `in-review`（调研已交）
- 分支：`main` = `079f58f`（= `origin/main`，PR #103 已 squash 合入）；无临时分支（`agent/issue-101-corridor-short-arc-guard` 已删）
- 未完成工作：
  - **#101 剩余项待 Planner 裁定**：入口选瓣歧义（赛道左右镜面对称，无几何依据）。三选项 A（mission 级航线输入，建议）/ B（门禁改 signed 计圈）/ C（破对称），见 Issue #101 评论。
  - **#92 挂载**：仍 blocked on #101（重入条件 ①②③ 未变）。
  - **无 WIP 代码**：临时仪表（`[PROBE101]`）与相位门限试验已全部移除，工作树与 `079f58f` 一致。
- 最近提交（`main`）：
  - `079f58f` fix(skidpad): #101 走廊模型退化的持久化修复——可信模型跨帧保持，路径不再塌缩 (#103)
  - `4973311` chore(agent): plan-20261028 验收 #96/#69 关闭、#101 部分通过
  - `a02f541` fix(gates): #92 在线闭环门禁加「未停滞」断言 + 节点注册表入 JSON（保持未挂载）
- 今日日报：已写 `.agent/DIARY/2026-10-28.md`；**补记** `.agent/DIARY/2026-10-27.md`（10-27 两会话按 git/Issue 证据反推，已在文首标注）
- CI 事实：
  - `main` 的 `079f58f` push run `40700768939` = **success**（合入 #103 之后）。
  - `4973311`（plan-20261028）= success（`40685236140`）；`a02f541` Nightly `40589165213` = success。
  - PR #103 CI run `40498109179` = **success**（合入前）。
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
