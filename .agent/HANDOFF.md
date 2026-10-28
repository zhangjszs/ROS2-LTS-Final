# HANDOFF

## 本轮概要

exec-20261028（接 plan-20261028 队列：#101 → #100）。**队列已清空**：#101 剩余范围执行到「② 合 PR #103」完成后，**实测推翻了 Planner 的剩余范围前提**（跨瓣回退一次都没发生），改为**入口选瓣歧义**并转 `needs-info`；#100 第一阶段调研完成、交 `in-review`。**无新增算法代码**，`main` 前进一格（`4973311` → `079f58f`）。

| 事项 | 结论 |
| --- | --- |
| #101 ② 合 PR #103 | **完成**：squash 合入 `main` = `079f58f`，分支已删，CI run `40700768939` = success |
| #101 ① 消除跨瓣回退 | **未实施**（前提不成立）：5 次仪表连跑中后段回退 **0 次**，见下 |
| #101 ③ 10 连跑 rate ≥ 0.15 | **未达成，判定不可由本单代码达成**：赛道左右镜面对称 → 切点选瓣是硬币（2/5 跑进左瓣，rate 0.106） |
| #100 调研 | **完成**：带符号绕角可区分瓣（左瓣 0.80 / 右瓣 6.42 rad）；推荐 A = mission 级航线输入 + 每瓣 signed 计圈 |

## 已完成

- **#101 的 ②**：`gh pr merge 103 --squash --delete-branch` → `079f58f` @ `main`；合入前 PR CI `40498109179` = success，合入后 push run `40700768939` = success。→ Issue #101 已 `needs-info` + 执行报告已贴（comment 6471396547）。
- **#100**：调研报告已贴（comment 6471424514），Issue 已 `in-review`。无代码改动。
- **日报补记**：`.agent/DIARY/2026-10-27.md` 已按 git / Issue / STATE 证据反推补写（文首明确标注「本轮补记，非当日实时书写」）。

## 未完成 / 进行中（下一棒最优先看这里）

- **#101 等 Planner 裁定（needs-info）**，三选项（Executor 建议 A）：
  - **A**：给规划器一条 mission 级航线输入（`initial_lobe` / `start_direction`），仿真侧从 `tracks.json` 的 `right_center` 推导后传入 → 改动=节点参数 + Core 一个显式选瓣分支 + 单测；
  - **B**：门禁判据从 `abs(cum_angle) ≥ 2π` 改为 `signed ≥ 2π`（数据见 #100 报告：左瓣 0.80 vs 右瓣 6.42），属 Planner 权限；
  - **C**：破对称（改 tracks.json / INIT_THETA / 标记锥）——本单明确排除。
  - **裁定后继续 #101 的起手式**：在 `corridor_model_core.hpp` 的选瓣处加显式先验（A）或加「s 向单调、不得越过切点」的兜底硬约束；用 `test/test_corridor_model.cpp` 锁；再跑 10 连跑，逐跑记录 `cum_right_angle_rad / wall_s`。
- **#92 挂载**：仍 blocked on #101；**不要**在 #101 裁定前挂载。
- **无 WIP 分支、无未提交改动**：临时仪表 `[PROBE101]`（`icp_apf_planner.cpp` + `skidpad_planner_node.cpp`）已移除，`centerline_core.hpp` 的相位门限试验（0.5 / 0.3）已还原为 `1.0`；`git status` 干净。

## 验证情况

| 验证 | 结果 |
| --- | --- |
| `colcon build --packages-select skidpad_planner`（还原后的 main） | rc=0 |
| `colcon test --packages-select skidpad_planner` + `test-result` | **72 tests, 0 errors, 0 failures** |
| `skidpad_closed_loop_smoke.sh` 基线 5 连跑（`main`=079f58f，带仪表） | 3 右（27.5–28.6 s，rate 0.220–0.229）/ 2 左（59.2 s，**rate 0.106**）；`rc` 全 0；横偏 0.48–0.51 m |
| 同上：首帧非 midpoint 时刻 | **t = 9.0–9.5 s**，自车 x ≈ −5.2~−5.8（切点在 x=0）；模型半径 10.6–11.4（真值 9.125），相位 0.88–0.97（门限 1.0） |
| 同上：中后段回退（midpoint/circle） | **0 次 / 5 跑**（全部集中在 t<9.6 s 进近段） |
| 对照实验：相位门限 1.0 / 0.5 / 0.3 | 3右2左 / 4/4 右（25.8–28.0 s）/ 3/4 右（第 4 次首帧伪模型 −y → 左瓣一圈后折返，**40.1 s / rate 0.157**） |
| 赛道镜像对称 | 108 锥桶，每个都能在 **0.0001 m** 内找到 y 取反的镜像点 |
| CI（`main` 079f58f） | run `40700768939` = **success** |

**未跑**：`bash scripts/drive_gates.sh` 全量（本轮无算法改动；代码变更仅来自已合并的 PR #103，其 CI + 合入后 main push run 均绿）。

## 风险与注意事项

1. **#101 的验收线 `rate ≥ 0.15` 余量极小**：右瓣本身实测 0.157–0.243（40.1 s 的那次只到 0.157）。即使选瓣问题解决，10 连跑也不能保证每次都 ≥0.15 → 建议 Planner 一并改判据口径（signed 计圈更硬）。
2. **相位误差不是独立判据**（本轮新发现，值得写进 #101/#100 的后续设计）：`radius=(r_in+r_out)/2` 与「自车到圆心距离」同源 → 任何两排大致平行的锥桶都能拟出相位≈0 的伪弧（实测 r=14.07 / 相位 0.074）。要加固就换**壁的曲率一致性**判据，不是相位阈值。
3. **进近直道会被拟合成 12 m 伪弧**：t≈9 s 时自车还在直道上（离切点 5 m 以上），模型却已给出 r=10.6–14 的弯曲路径 → 车提前 5 m 开始转向。这是「入口选瓣」之所以在**到达路口之前**就被决定的原因。
4. **持久模型是双刃剑**：一旦选错瓣，`trusted_model` 会把车锁在该瓣整跑（左瓣 59.2 s）；`max_trusted_phase_error=1.5` 是唯一的解锁阀，实测在切点折返时才生效（见 exp3_4）。
5. **左瓣跑的 rc 是 0**：现有门禁断言不会判红（59.2 s 内凑满 2π），`rate` 只是证据要求 → 挂 #92 时不会随机红，但墙钟贴近 60 s 预算上限。
6. `ComputeCenterlineCore` 仍是差分基线，任何排序语义改动都会破坏 `test_icp_apf_planner.cpp` 的 `DifferentialEquivalenceWithLegacyPCL`（#24 级变更）。

## 给下一棒的第一步建议

1. 先读 **#101 的最新执行报告**（comment 6471396547）与 **#100 的调研报告**（comment 6471424514）——两份合起来把「选瓣歧义 → 计圈判据」讲完了。
2. 若 Planner 选 **A**：直接在 `icp_apf_planner.cpp::GeneratePath` → `ComputeCorridorPathCore` 的模型选择处接一个显式首瓣先验（`CenterlineParams` 加字段，默认「无先验」保持现行为），用 `test_corridor_model.cpp` 加 2 例（先验=+y 时选 +y 候选；先验与几何冲突时按先验），然后 10 连跑。
3. 若 Planner 选 **B**：改的是 `scripts/skidpad_closed_loop_smoke.sh` 的判据（`abs` → `signed`），属判据变更，需在 PR 里逐条对照 `docs/INTERFACE_CONTRACT.md` 无关、但要在 #92 单内留口径说明。
4. **别再调 `max_model_phase_error`**——三档对照已证无效（见上表）。

## 今日日报

- 已写入 `.agent/DIARY/2026-10-28.md`；补记 `.agent/DIARY/2026-10-27.md`（标注为反推补记）。

## 给 Planner 的信号

- **需要 Planner 介入**：① #101 选瓣输入来源（A/B/C）；② `rate ≥ 0.15` 这条证据线的口径（建议改 signed 计圈）；③ #92 挂载继续等 #101。
- #100 调研已完成，验收后可直接转成「加航线输入 + signed 计圈」的实现单。
- 本轮**没有 auto-discovered 新单**（配额未用：发现的相位判据缺陷写在 #101 报告与 HANDOFF 风险 2；如需立案请示下）。
