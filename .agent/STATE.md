# STATE

- 更新时间：2026-10-25T23:30:00+08:00（exec-20261025 收尾中）
- 当前任务：#92（needs-info，skidpad 在线闭环数据面门禁）
- 分支：`agent/issue-92-skidpad-closed-loop-smoke`
- 未完成工作：无当前进行中工作（#92 已部分完成转 needs-info，新门禁脚本 `scripts/skidpad_closed_loop_smoke.sh` 已合入本分支并推送，提出 auto-discovered #95）
- 最近提交：`4e5e668` feat(gates): 新增 skidpad 在线闭环数据面门禁脚本 (Refs #92)
- 今日日报：已写（见 .agent/DIARY/2026-10-25.md）
- CI 事实：main 上 PR #93/#94 merge run 与 docs run 均 success 收敛；分支 `agent/issue-92-skidpad-closed-loop-smoke` 已推
- 已知环境限制：
  1. **多节点 runtime 门禁必须 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`**（漏设 → DDS 发现不收敛假 FAIL）。
  2. **单包构建跑测试必须显式 `-DBUILD_TESTING=ON`**。
  3. 本机 `/tmp` 只读；`ROS_HOME`/`TMPDIR` 由脚本指向 `build/`；验证前 `export PATH="$HOME/.local/bin:$PATH"`。
  4. 本机 sim 时间戳显示异常偏大（1792849xxx）不影响判据（Watch 用墙钟预算、节点用 sim 钟对齐）。
- #92 关键实测事实（交接重点）：
  - 规划器在线发布正常（20Hz、非空、全部有限值无 NaN），profiler 正常转发，指令帧契约 100% 满足。
  - 算法结构性缺陷阻断闭环：`ClusterCones` 简单以 y=0 切分双圆，导致 `ComputeCircleCenterlineCore` 将左圆（cy=-9.125）与右圆（cy=+9.125）平均，生成以车身为中心（cy=0.0）半径 9.125m 的虚假同心圆，全路径距车身恒为 9.125m，触发 Pure Pursuit 拒解紧停（`crosstrack 10.03m > 6.00m`）；进场段视场混杂导致车辆在 [-6.16, -0.66] 停滞。
  - 已立项提交 auto-discovered #95，待 Planner 规划重构。
