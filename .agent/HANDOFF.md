# HANDOFF

## 本轮概要

exec-20261026 会话1：清空今日队列的**前三单**——#96 合入 main（完成）、#69 修就绪探测单点失效（代码完成，CI 运行中）、#95 修规划器几何缺陷（**部分完成**，CI 运行中）。

| 事项 | 结论 |
| --- | --- |
| #96 合入 skidpad_closed_loop_smoke.sh | **完成**：PR #97 已 squash merge 进 `main`（`7733cbb`），未挂载，CI 绿（attempt 2） |
| #69 DDS 门禁区分环境性/真失败 | **代码完成**（PR #98）：就绪探测改为进程存活优先 + daemon 视图降为辅助；真失败仍判红并点名 |
| #95 skidpad_planner 路径生成三处几何缺陷 | **部分完成**（PR #99）：Core 层三处缺陷已修 + 11 例纯 std 单测；端到端 0.409 → 1.313 rad，`refusing goal` 归零；判据 8 = 4/5，判据 1 待 Planner 重新表述 |

## 已完成

- **#96**：`scripts/skidpad_closed_loop_smoke.sh` 经 PR #97 合入 `main`（`7733cbb`），**保持未挂载**（`drive_gates.sh`/`ci.yml` grep 无命中）。改动仅 `scripts/`（422 行）。负样本臂 rc=1 且原因指向「无 pathlimits 产出」。CI run `40336692425` attempt 2 = success。→ Issue #96 已 `in-review` + 执行报告已贴。
- **#69**：
  - 新增 `scripts/lib/node_readiness.sh`（共用件）：**进程存活为主判据**（`ps -o stat=` 排僵尸态 —— `kill -0` 对僵尸仍返回成功）、`ros2 node list` 降为辅助确认、真失败立即判红并点名、环境性失败提示后由数据面定成败、节点视图查询可注入（`PROBE_NODE_LIST_CMD`）。
  - `skidpad_finish_smoke.sh` / `skidpad_closed_loop_smoke.sh`：`start()` 登记「节点名 + PID」；就绪探测换两段式；数据面失败首行补「疑环境性失败（DDS 发现未收敛）」。
  - 新增 `scripts/node_readiness_selftest.sh`（确定性构造三种情形），挂进 `lint_shell.sh` 的 `[3/4]` 段（**drive_gates 门清单不变**）。
  - 验证：正样本 A/B 对照 —— `main`（无本改动）默认预算下同样红（index 346/349），本分支 `BUDGET_SEC=420` 绿（`finish_event_count: 1`）；负样本 `REQUIRED_LAPS=3` 仍 rc=1。
- **#95**（Core 层，`src/planning/skidpad_planner/`）：
  - `ComputeCircleCenterlineCore`：同心→半径仍取平均（历史行为逐位不变）；**不同心→绝不平均**，相位感知单选（`‖ego−C‖ ≈ 壁半径`），中线半径取 `‖ego−C‖`，弧线**从自车起步**。
  - `Circle2D::rms_residual` + `max_fit_rms` 质量护栏：150° 部分弧的代数拟合（r=16.86、rms=1.27）被拒。
  - 新增 `OrderAlongCorridorCore`（方向感知行走）、`AppendMutualMidpointsCore`（互为最近邻配对）、`AnchorPathAtEgoCore`（锚自车 + 等距重采样）、`ExtendWithPrevPathCore`（上一帧路径补前视）、`ComputeCorridorPathCore`（生产入口）。
  - 节点层 `GeneratePath` 改走 `ComputeCorridorPath`，移除随之失效的三个私有成员。
  - `ComputeCenterlineCore` **排序语义逐位不变**（#24 跨版本差分夹具的事实来源，1000 组逐点比对）。
  - 新增 `test/test_corridor_core.cpp`（11 例），按 #40 登记进 `tests/core_standalone/CMakeLists.txt`。

## 未完成 / 进行中（下一棒最优先看这里）

- **#69（PR #98）**：等 CI 绿。绿后：squash merge → 删临时分支 → Issue #69 去掉 `in-progress`、打 `in-review`、贴执行报告（报告正文已写好，含 A/B 对照表）。
- **#95（PR #99）**：等 CI 绿。绿后：Issue #95 已是 `in-review`（执行报告已贴），**按「部分完成」上报**，并把判据 1 的裁定请求明确交给 Planner。
- **#95 的剩余差距（需要新算法思路，别在配对+排序框架里继续打补丁）**：
  - 现象：切点处两瓣锥桶交错 → 互为最近邻过滤把大量真配对也滤掉 → 可见中线仅 ~4.5 m（9 点）→ 低于 PP 的 `end_decel_points=10` → `Approaching path end` 实测 50 次。
  - 已试过并**无效**：`ExtendWithPrevPathCore`（上一帧路径补前视）——紧弯上 15 m 量程只看得见 ~5 m 走廊，上一帧路径同样只有 ~4.5 m，没有更远的点可借。
  - 已试过并**无效**：三点局部圆外推（实测 R=0.96 vs 真值 9.125，噪声放大到不可用）。
  - 下一步方向：**切点处的瓣归属判别**（用车辆相位/航向判定当前瓣，不靠配对的空间邻近）+ 紧弯走廊外推/建图。
  - 建议同时把 D-011 实测程序**入库**为机检夹具（现在在 `build/proto95/d011_measure.cpp`，未入仓库），让"横偏最小值 = 0""x 回跳 = 0"成为可回归门禁。

## 验证情况

| 验证 | 结果 |
| --- | --- |
| `colcon test --packages-select skidpad_planner track_benchmark` | 626 tests, 0 failures |
| `bash scripts/core_standalone_check.sh` | 34/34 PASS（ASan+UBSan clean） |
| `bash scripts/lint_cpp.sh` / `lint_shell.sh` | 0 issues / 19 脚本 clean + 就绪探测自测全过 |
| `bash scripts/node_readiness_selftest.sh` | 三情形断言全过 |
| `ENABLE_PLANNER=0 skidpad_closed_loop_smoke.sh` | rc=1（负样本可证伪守住） |
| `skidpad_closed_loop_smoke.sh`（正样本） | rc=1（脚本自身 2π 未达）；`cum_right_angle_rad 1.3134`（旧 0.409）、`max_lateral_dev_m 0.25`、`pathlimits_finite True`、`refusing goal` 计数 **0** |
| `BUDGET_SEC=420 skidpad_finish_smoke.sh` | rc=0（`finish_event_count: 1`、`max_lateral_dev: 0.461`） |
| `REQUIRED_LAPS=3 skidpad_finish_smoke.sh` | rc=1（负样本守住） |
| PR #97（#96）CI | **success**（run `40336692425` attempt 2，19 step 全绿） |
| PR #98（#69）/ PR #99（#95）CI | **运行中**（`40355980015` / `40360767468`） |

**未跑**：`bash scripts/drive_gates.sh` 全量门禁（本轮改动的覆盖由两个 PR 的 CI 承担：build/test/core-standalone/lint + 各 runtime 门禁）。

## 风险与注意事项

1. **#95 不要挂 `skidpad_closed_loop_smoke.sh` 进 `drive_gates.sh`/`ci.yml`** —— 正样本仍红（切点处可见中线太短 → PP 末端刹车）。挂载必须等 #95 续单完成、正样本在 CI 上绿。
2. **本机默认 180 s 预算下 `skidpad_finish_smoke.sh` 正样本红是既有环境问题**（非代码）：A/B 对照 `main` 上同样红；`BUDGET_SEC=420` 绿。本机复现绿请加大预算。
3. **CI 的 ros-apt-source 步骤偶发 404**（GitHub API/下载侧抖动，attempt 1 判红、re-run 绿）。遇到先 re-run 再判断是不是自己的锅。
4. **`ComputeCenterlineCore` 是差分基线**：任何改动它都会破坏 `test_icp_apf_planner.cpp` 的 `DifferentialEquivalenceWithLegacyPCL`（1000 组逐点比对）。要改排序语义，先更新 `docs/DEFECT_DIFFERENTIAL.md`（#24 级变更）。
5. **`test_icp_apf_planner.cpp` 的 `OppositeSidesProduceForwardPath` 断言已更新**（旧断言"首点 x > 0"钉的是 #95 要改掉的旧语义）——新的契约是"首点落在自车 0.5 m 内且路径明显向前"。

## 给下一棒的第一步建议

1. `gh pr checks 98` / `gh pr checks 99` 看两个 PR 的 CI；绿了就 merge + 贴/更新执行报告 + 删临时分支。
2. 若 #95 的 CI 因**切点路径长度**判红：那是预期的（脚本未挂 CI，主干不受影响）；按 `in-review` 上报并把续单方向写进 Issue。
3. 续 #95 时**先做切点瓣归属判别**（不要先做外推）：在 `centerline_core.hpp` 的 `ComputeCorridorPathCore` 里引入"当前瓣"判据（车辆相位/航向），让切点处不再依赖配对的空间邻近。

## 今日日报

- 已写入 `.agent/DIARY/2026-10-26.md`。

## 给 Planner 的信号

- **需要 Planner 介入（两项）**：
  1. **#95 判据 1 的表述**：字面条件（路径进入两**圆心** ±1.5 m 邻域）与 #90 已验收的真值中心线互斥（真值 349 点到两圆心的最小距离恒为 9.125 m，邻域内 0 点）；建议改为"非单圆 + 分瓣中线偏差"，或把"全量 108 锥桶下覆盖两瓣"单列为全局建图里程碑。
  2. **#95 续单排期**：剩余差距需要新的算法思路（瓣归属判别 + 走廊外推/建图），建议单独立项。
- **#69、#96 无需介入**：判据已逐条 `[x]` 或有实测证据，等 CI 绿即可验收关闭。
