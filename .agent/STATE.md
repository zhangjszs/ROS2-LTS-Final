# STATE

- 更新时间：2026-10-24T15:30:00+08:00（exec-20261024 会话2 进行中）
- 当前任务：#91（in-progress，**PR #94 已开等 CI**）；#90 已完成合入 main（`8828780`，PR #93 CI 绿）并转 in-review（报告已留）
- 分支：`agent/issue-91-skidpad-finish-smoke`（commit `73c6015` 已推送）；merge 后回 main
- 未完成工作：#92（P2，skidpad_planner 在线闭环冒烟）未开工——本会话预算优先保 #91 收尾，#92 留下一棒（恢复点见 HANDOFF）
- 最近提交：`73c6015` feat(gates) skidpad_finish_smoke 门禁；`80ba188`（#90 判据核，经 `8828780` 合入）
- 今日日报：未写（会话2 收尾时追加到 .agent/DIARY/2026-10-24.md）
- CI 事实：PR #93（#90）build-test-lint success 13m26s；PR #94（#91）推进中
- 已知环境限制：
  1. **多节点 runtime 门禁必须 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`**（漏设 → DDS 发现不收敛假 FAIL）。
  2. **单包构建跑测试必须显式 `-DBUILD_TESTING=ON`**。
  3. 本机 `/tmp` 只读；`ROS_HOME`/`TMPDIR` 由脚本指向 `build/`；验证前 `export PATH="$HOME/.local/bin:$PATH"`。
  4. 本机 sim 时间戳显示异常偏大（1792849xxx）不影响判据（Watch 用墙钟预算、节点用 sim 钟对齐）。
- #91 关键实测事实（下一棒接 #92 要用）：
  - skidpad 闭环数据面：**单次发布全量中心线** + PP `safety.path_timeout:=600` + safety_monitor `planner_timeout:=600` + PP `algorithm.throttle.speed_high_threshold:=10` + sim `init_x:=-15` → 巡航 5.16 m/s、横向 0.434m、60.9s 完赛
  - 持续重发路径必被 x 轴重叠段劫持（PP 每条消息全局重锚）——#92 换 skidpad_planner 在线输出时同样要面对此问题（规划器持续重发路径 → 需确认其输出不含 x 轴重叠歧义或采取对策）
  - 预演脚本留存 `build/presmoke91.sh`（build/ 不入库，可参考复刻）
