# HANDOFF.md — 给下一棒（relay 交接快照）

当前棒：`exec-20261002T1509Z`（2026-10-02）——消费 PLAN ready 队列 **#53 → #52 → #54，三项全部完成**。
上一棒 `plan-20261002T141500Z` 建立 PLAN/DECISIONS 与 ready 队列（`06ce1d6`）。
**先读 `.agent/STATE.md`、`.agent/PLAN.md`、`.agent/DECISIONS.md`、`.agent/ENV.md`。**

## 一、本棒成果（三项 ready 全部完成，待 Planning 验收）

| 序 | Issue | 代码 commit | 验证 |
|---|---|---|---|
| 1 | #53 vehicle_simulator 拆 bicycle_model 纯 std 单测进 #40 sanitizer 门 | `eb75d08` | core_standalone 24/24；colcon 484/0 fail；lint 通过 |
| 2 | #52 `QpSettings::warm_start` 死配置生效 + 单测 | `fe2d97b` | core_standalone 24/24；colcon 486/0 fail；lint 通过 |
| 3 | #54 sensor_simulator 感知几何核下沉纯 std core | `dbb77c0` | core_standalone 25/25；colcon 492/0 fail；lint 通过 |

- 三条结构化执行结果已挂 issue 评论：`build/issue_comments/c5{2,3,4}.md`（不入库）。
- **本棒未关闭任何 issue**（按规则由 Planning 按验收裁定）。

## 二、下一棒的第一步

1. **Planning 验收**：复核 #53/#52/#54 的 issue 评论 + 代码 commit + 本轮 CI，逐条对照验收标准后裁定关闭；关闭 #53 会自动解除 #54 的原生依赖 `blocked_by #53`。
2. 若继续执行：重拉 `gh issue list --label ready-for-agent --state open`——
   - 若为空：软件侧已扫尽，剩余路线图条目卡仓库外输入，见 PLAN 第三/六/七节；不臆造。
   - 若 Planning 新增 ready：按 PLAN 第四节顺序消费。

## 三、本棒实现的三个任务要点（复现/续做）

- **#53**：新 `test_bicycle_model_core.cpp`（`BicycleModel` 5 用例，无 ROS/msgs）；`test_bicycle_model.cpp` 只留适配层测试；两轨注册（`vehicle_simulator/CMakeLists.txt` + `tests/core_standalone/CMakeLists.txt`）。
- **#52**：`BoxQpSolver::Solve` 读 `settings_.warm_start`——`false` ⇒ 忽略并清空内部 `z_/y_`、盒中心冷启动、返回前不写回；显式 `warm_x` 不受影响；2 个新纯 std 单测。
- **#54**：新增 header-only `sensor_model_core.hpp`（`TrackCone`/`DetectedCone`/`PredictVisibleCones`）；`SensorSimulator::GeneratePerceivedCones` 退化为 msg 组装（行为逐位一致）；新 `test_sensor_model_core.cpp`（5 用例）；`test_bicycle_model.cpp` 收敛为适配层测试；同步 ADR 与 #40 排除清单。

## 四、红线（仍然有效）

- 别用过仿真证据关闭 `#17/#15/#16/#19`；关闭条件见各 issue 评论（D-004）。
- 别改判据（D-003）；别动 `benchmarks/baseline/`；别往 `fsac.benchmark.kpi/v1` 加字段。
- 代码提交与 `.agent` 状态提交分离（D-006）。
- 别 force push；清理进程只按 PID/PGID；严禁 `pkill -f <节点名>`。
- 本机 ROS=lyrical（CI 是 Jazzy）；跑门禁前先设 `FASTDDS_BUILTIN_TRANSPORTS=SHM`、`ROS_HOME=$PWD/build/.ros`。
- 改 C++ 后先 `clang-format -i <file>`（clang-format 是 lint 必查项）。

## 五、复现本棒验证

```bash
bash scripts/core_standalone_check.sh                 # 25/25（含 test_bicycle_model_core / test_sensor_model_core / test_qp_solver 新测）
bash scripts/lint_cpp.sh
source /opt/ros/lyrical/setup.bash && source install/setup.bash
MAKEFLAGS=-j4 colcon build --packages-select vehicle_simulator mpc_controller --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 -DCMAKE_LINKER_TYPE=MOLD \
  -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DBUILD_TESTING=ON
colcon test --packages-select vehicle_simulator mpc_controller && colcon test-result --verbose
```

- 本棒起点：`06ce1d6`；产出代码 commit：`eb75d08`、`fe2d97b`、`dbb77c0`（均已 push 到 origin/main）。
