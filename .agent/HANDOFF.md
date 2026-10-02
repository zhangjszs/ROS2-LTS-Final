# HANDOFF.md — 给下一棒（relay 交接快照）

当前棒：`exec-20261002T1509Z`（2026-10-02，执行 PLAN ready 队列 #53 → #52 → #54）。
上一棒 `plan-20261002T141500Z` 建立 PLAN/DECISIONS 与 ready 队列（`06ce1d6`）。
**先读 `.agent/STATE.md`、`.agent/PLAN.md`、`.agent/DECISIONS.md`、`.agent/ENV.md`。**

## 一、本棒进度（进行中）

| 序 | Issue | 状态 | 代码 commit | 验证 |
|---|---|---|---|---|
| 1 | #53 拆 bicycle_model 纯 std 单测进 #40 sanitizer 门 | **完成**（待 Planning 验收） | `eb75d08` | core_standalone 24/24；colcon 484/0 fail；lint 通过；纯度 grep clean |
| 2 | #52 `QpSettings::warm_start` 死配置生效 + 单测 | **进行中** | — | — |
| 3 | #54 sensor_simulator 感知核下沉纯 std core | 未开始 | — | — |

- #53 结构化执行结果已挂 issue 评论（`build/issue_comments/c53.md`，不入库）。
- 本棒**未关闭任何 issue**（关闭由 Planning 验收裁定）。

**若本棒异常中断：** 先看 `.agent/LOCK` 与本表——#53 的代码已在 `eb75d08` 且已 push；#52/#54 尚未动工。中断后从 #52 续做即可。

## 二、下一棒的选题顺序

1. 若本棒已推进：#52（若未完）→ #54。
2. 若本棒已完成全部三项：重拉 `gh issue list --label ready-for-agent --state open`，按 PLAN 第四节顺序继续；若为空见下一段。
3. 无 ready 时的处理见 PLAN 第三节/第七节（软件侧已扫尽，剩余卡仓库外输入，不臆造）。

## 三、本轮两个任务的实现要点（给下一棒复现/续做）

### #53（已完成）
- 新 `src/simulation/vehicle_simulator/test/test_bicycle_model_core.cpp`（5 个 `BicycleModelTest`，无 ROS/msgs）。
- `test_bicycle_model.cpp` 只留 `SensorSimulatorTest`。
- 注册点：`src/simulation/vehicle_simulator/CMakeLists.txt`（ament_add_gtest）+ `tests/core_standalone/CMakeLists.txt`（add_core_test，含 `bicycle_model.cpp`）；
  排除清单注释已更新。

### #52（进行中，语义已由 Planning 定死，见 issue body）
- `BoxQpSolver::Solve` 读 `settings_.warm_start`：`false` ⇒ 忽略并清空 `z_`/`y_`、盒中心冷启动、返回前不写回；显式入参 `warm_x` **不受** `warm_start` 影响。
- 加纯 std 单测并注册进 `tests/core_standalone`（`test_qp_solver` 目标已含 `qp_solver.cpp`）。
- 默认 `true` 行为必须逐位不变（既有 test_qp_solver / test_mpc_tracking 保护）。

### #54（未开始）
- ADR 残余：把 `sensor_simulator` 的 FOV/距离/坐标变换/噪声核抽为纯 std core，`SensorSimulator` 退化为 msg 组装。
- 注意：生产代码重构，须以既有 colcon 单测 + 新 core 单测锁"行为逐位等价"。

## 四、红线（仍然有效）

- 别用过仿真证据关闭 `#17/#15/#16/#19`；关闭条件见各 issue 评论（D-004）。
- 别改判据（D-003）；别动 `benchmarks/baseline/`；别往 `fsac.benchmark.kpi/v1` 加字段。
- 代码提交与 `.agent` 状态提交分离（D-006）。
- 别 force push；清理进程只按 PID/PGID；严禁 `pkill -f <节点名>`。
- 本机 ROS=lyrical（CI 是 Jazzy）；跑门禁前先设 `FASTDDS_BUILTIN_TRANSPORTS=SHM`、`ROS_HOME=$PWD/build/.ros`。

## 五、复现本棒验证

```bash
bash scripts/core_standalone_check.sh                 # #53：应 24/24，含 test_bicycle_model_core
source /opt/ros/lyrical/setup.bash && source install/setup.bash
MAKEFLAGS=-j4 colcon build --packages-select vehicle_simulator --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 -DCMAKE_LINKER_TYPE=MOLD \
  -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DBUILD_TESTING=ON
colcon test --packages-select vehicle_simulator && colcon test-result --verbose   # 484/0 fail
bash scripts/lint_cpp.sh
```

- 本棒起点：`06ce1d6`；#53 代码 commit：`eb75d08`（已 push 到 origin/main）。
