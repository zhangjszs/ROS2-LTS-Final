# HANDOFF.md — 给下一棒（relay 交接快照）

当前棒：`exec-20261002T1509Z`（2026-10-02，执行 PLAN ready 队列 #53 → #52 → #54）。
上一棒 `plan-20261002T141500Z` 建立 PLAN/DECISIONS 与 ready 队列（`06ce1d6`）。
**先读 `.agent/STATE.md`、`.agent/PLAN.md`、`.agent/DECISIONS.md`、`.agent/ENV.md`。**

## 一、本棒进度（进行中）

| 序 | Issue | 状态 | 代码 commit | 验证 |
|---|---|---|---|---|
| 1 | #53 拆 bicycle_model 纯 std 单测进 #40 sanitizer 门 | **完成**（待 Planning 验收） | `eb75d08` | core_standalone 24/24；colcon 484/0 fail；lint 通过 |
| 2 | #52 `QpSettings::warm_start` 死配置生效 + 单测 | **完成**（待 Planning 验收） | `fe2d97b` | core_standalone 24/24；colcon 486/0 fail；lint 通过 |
| 3 | #54 sensor_simulator 感知核下沉纯 std core | **进行中** | — | — |

- 结构化执行结果已挂 #53、#52 评论（`build/issue_comments/c5{2,3}.md`，不入库）。
- 本棒**未关闭任何 issue**（关闭由 Planning 验收裁定）。

**若本棒异常中断：** 看 `.agent/LOCK` 与本表——#53(`eb75d08`)/#52(`fe2d97b`) 均已 push；#54 尚未完成，从 #54 续做即可。

## 二、下一棒的选题顺序

1. 若本棒未完成：#54 续做（要点见下）。
2. 若本棒三项均完成：重拉 `gh issue list --label ready-for-agent --state open`；若为空见 PLAN 第三节/第七节（软件侧已扫尽，剩余卡仓库外输入，不臆造）。

## 三、剩余任务要点

### #54（进行中）：sensor_simulator 感知几何核下沉
- 目标：把 `GeneratePerceivedCones` 的**纯数学**（平移/旋转到 base_link、FOV 与距离过滤、可选高斯噪声，`sensor_simulator.cpp:53-107`）抽成纯 std core 头；`SensorSimulator` 只做中性类型→`common_msgs` 消息组装；新增 core 单测进 `tests/core_standalone`。
- 约束：公共 API 与行为**逐位一致**（默认 `fov=120/range=15/noise=0`；`noise<=0` 不采样；默认种子 42；`dist_sq<0.25` 与 `x_base<=0.2` 过滤；`confidence=95`）；core 头不得 include `common_msgs`/`rclcpp`。
- 同步更新 `tests/core_standalone/CMakeLists.txt` 排除清单与 `docs/CORE_ROS_BOUNDARY_ADR.md` 的"仍依赖中间件的模块"条目。
- 验收：既有 `SensorSimulatorTest` 仍绿 + 新 core 单测（同种子输出可复现且与改前一致）+ `core_standalone_check.sh` 与 `colcon test` 全绿 + lint。

### 已完成任务（复现用）
- #53：新 `test_bicycle_model_core.cpp`（5 用例，无 ROS/msgs），`test_bicycle_model.cpp` 只留 `SensorSimulatorTest`；两轨注册（`vehicle_simulator/CMakeLists.txt` + `tests/core_standalone/CMakeLists.txt`）。
- #52：`BoxQpSolver::Solve` 读 `warm_start`（false ⇒ 忽略并清空 `z_/y_`、盒中心冷启动、不写回；`warm_x` 不受影响）；2 个新单测。

## 四、红线（仍然有效）

- 别用过仿真证据关闭 `#17/#15/#16/#19`；关闭条件见各 issue 评论（D-004）。
- 别改判据（D-003）；别动 `benchmarks/baseline/`；别往 `fsac.benchmark.kpi/v1` 加字段。
- 代码提交与 `.agent` 状态提交分离（D-006）。
- 别 force push；清理进程只按 PID/PGID；严禁 `pkill -f <节点名>`。
- 本机 ROS=lyrical（CI 是 Jazzy）；跑门禁前先设 `FASTDDS_BUILTIN_TRANSPORTS=SHM`、`ROS_HOME=$PWD/build/.ros`。
- 改 C++ 后先 `clang-format -i <file>`（clang-format 是 lint 必查项）。

## 五、复现本棒验证

```bash
bash scripts/core_standalone_check.sh                 # 24/24（含 test_bicycle_model_core / test_qp_solver 新测）
bash scripts/lint_cpp.sh
source /opt/ros/lyrical/setup.bash && source install/setup.bash
MAKEFLAGS=-j4 colcon build --packages-select vehicle_simulator mpc_controller --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 -DCMAKE_LINKER_TYPE=MOLD \
  -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DBUILD_TESTING=ON
colcon test --packages-select vehicle_simulator mpc_controller && colcon test-result --verbose
```

- 本棒起点：`06ce1d6`；#53 `eb75d08`、#52 `fe2d97b`（均已 push 到 origin/main）。
