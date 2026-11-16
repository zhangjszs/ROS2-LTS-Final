# Issue #107 执行报告（2026-11-16）

Issue: [#107](https://github.com/zhangjszs/ROS2-LTS-Final/issues/107)

状态: **in-review**。软件实现和可执行核心证据已交付；ROS 2/Jazzy 集成尚未验收。Issue 保持 OPEN，不合并、不上车。

基线 SHA: `8d9479ef930e75192a3391259b1c8486d03a3f79`。开工及发布前分别核对 PR #106，远端未变化。`main` 为 `b769baff033e9d952a4bb936d976e0e7a8850ac8`。

新分支: `fix/vehicle-state-lease-107`

PR: [Draft #108](https://github.com/zhangjszs/ROS2-LTS-Final/pull/108)，base=`fix/fsac-fail-closed-arbiter-20261115`（stacked on #106）。

## Commits

| 提交 | 作用 |
| --- | --- |
| [`c9e70859dfea0f5ec20e75532a180c67028cb22b`](https://github.com/zhangjszs/ROS2-LTS-Final/commit/c9e70859dfea0f5ec20e75532a180c67028cb22b) | 抽离旧接收判定、先提交失败回归；此中间提交故意 RED，不可单独交付 |
| [`41fd324423fbd040d10bfd2061fff7c9da0c43dc`](https://github.com/zhangjszs/ROS2-LTS-Final/commit/41fd324423fbd040d10bfd2061fff7c9da0c43dc) | 独立修复 #106 遗留 C++ 格式 |
| [`345bec67b9a9011ef7ef09219d80948d27c43392`](https://github.com/zhangjszs/ROS2-LTS-Final/commit/345bec67b9a9011ef7ef09219d80948d27c43392) | 独立修正旧 PP 限幅测试夹具与非正目标停车契约的冲突 |
| [`b290425bdc3971ba54797f44ba0770fc0b8a395b`](https://github.com/zhangjszs/ROS2-LTS-Final/commit/b290425bdc3971ba54797f44ba0770fc0b8a395b) | 双租约生产修复、16 项核心回归、ROS 故障注入、契约/CI 接入 |

代码通过已连接 GitHub 的 Git Data API 发布；与本地测试源码的 Git tree 哈希逐项相等，随后 fetch 并将本地分支对齐远端历史。测试代码树为 `5c05aa2fc55c61e4249d01e18baf05e604863991`。证据/交接提交只补文档，不改变该实现。

## 根因：旧代码路径

以基线 SHA 的 `src/safety/safety_monitor/src/safety_monitor.cpp` 为准：

- **205–219**：合法车辆状态设置 `has_vehicle_state_=true`，断流不会撤销这个永久标记。
- **167–202**：合法路径只检查曾收到状态，即可调用 `onPathReceived()` 清 TIMEOUT。
- **264–287**：watchdog 只检查 `planner_lease_`，新路径持续到达时提前返回。
- **241–259**：车辆年龄只在人工 reset 局部使用，没有进入运行期停车判定。

因此“单次状态 + 持续路径”会保持监控健康。PP 的独立状态保护不等同于系统监控保护，也不能覆盖其他控制源。

新代码证据（`b290425`）：`safety_monitor.cpp:22–25` 参数；`:83–96` wall watchdog / 周期停车；`:113–142` 合法输入分别续租；`:145–165` 双源统一停车/恢复/复位门。`input_freshness.h` 是可注入两种时间的纯 C++ 核心。

## 修改：逐文件说明

| 文件 | 具体修改 |
| --- | --- |
| `src/safety/safety_monitor/include/safety_monitor/input_freshness.h` | 新增独立车辆/规划 FreshnessLease；有限状态校验；有限正超时参数；steady 接收活性；ROS/接收时间回退作废两个旧 epoch 租约；health/age 诊断数据 |
| `src/safety/safety_monitor/src/safety_monitor.cpp` | 去掉永久健康布尔及规划代续租；两回调、watchdog、reset 共用健康门；50Hz WallRate，10Hz 持续 stop 心跳；状态/路径订阅走契约 QoS；诊断两源年龄与 freshness |
| `src/safety/safety_monitor/test/test_input_freshness.cpp` | 16 项确定性测试，组合生产停车/任务状态机和最终仲裁核；虚拟时间，无长 sleep |
| `src/safety/safety_monitor/CMakeLists.txt` | 注册新测试；把旧停车状态机改为静态库供节点及两测试链接，避免业务源码在测试中重复编译 |
| `tests/core_standalone/CMakeLists.txt` | 新测试进入 ASan/UBSan 门及覆盖审计，链接同一停车状态机库 |
| `scripts/vehicle_state_lease_smoke.py` | 真实监控/仲裁进程；合法 MPC 油门持续输入；覆盖启动、单帧定位、各断流、恢复顺序、NaN、非法路径、锁存、授权、两种重启、暂停/回退时钟；记录输入、状态、stop 和最终输出的 steady JSON 时间线 |
| `scripts/vehicle_state_lease_smoke.sh` | 独立 DDS 域/localhost、150s 总预算、非零失败、缺依赖 NOT_RUN JSON，超时清理 |
| `scripts/fault_injection_smoke.sh` | arbiter 链先运行独立双租约注入，失败阻断旧门禁；原有场景保留 |
| `scripts/qos_contract_check.sh` | 显式 simulation 模式允许隔离测试出口重映射（不放宽 real 约束）；增加两个 safety_monitor 输入端点断言 |
| `.github/workflows/ci.yml` | 支持 stacked PR 的目标分支，新增独立 Jazzy 注入步骤，上传时间线 JSON |
| `docs/INTERFACE_CONTRACT.md` | 定义各 freshness 状态、时钟域、预算、授权/恢复及设备时间边界 |
| `src/control/pure_pursuit/test/test_throttle_controller.cpp` | 限幅夹具使用正目标但负速度误差；另断言零目标覆盖正 pedal_min。只改测试，不改参数/标定/控制律 |
| `docs/VEHICLE_STATE_LEASE_107.md`、`docs/evidence/issue107/*` | 本报告、原始 RED/GREEN/门禁输出和证据哈希 |
| `.agent/STATE.md`、`.agent/HANDOFF.md`、`.agent/DIARY/2026-11-16.md` | 追加本任务执行事实、边界及 Planner 验收入口；保留其他任务状态 |

独立格式提交涉及以下 10 文件，仅 include 排序/换行/空白变化，无业务修改：

- `src/common/common_msgs/include/common_msgs/task_state_machine.h`
- `src/control/pure_pursuit/include/pure_pursuit/throttle_controller.h`
- `src/control/pure_pursuit/test/test_throttle_controller.cpp`
- `src/safety/safety_monitor/src/command_arbiter_node.cpp`
- `src/safety/safety_monitor/test/test_task_state_machine.cpp`
- `src/sensors/cone_tracker/include/cone_dedup_algo.h`
- `src/sensors/cone_tracker/src/cone_dedup.cpp`
- `src/simulation/track_benchmark/src/benchmark_runner.cpp`
- `src/simulation/vehicle_simulator/include/vehicle_simulator/simulator_node.hpp`
- `src/simulation/vehicle_simulator/src/simulator_node.cpp`

## 判定与恢复规则

1. 车辆接收预算默认 **0.3s**（与 PP 对齐），规划仍 **0.5s**，均不可负值/零/NaN/Inf；未做实车标定。
2. 合法车辆状态只指 x/y/theta/v 有限；Header 是发布时刻，不是设备测量时间。本任务不解释质量枚举，不换算 GPS 时间，不声称解决持续旧观测或无效定位质量。
3. Absent / Stale 都停车。FromFuture 的接收时钟异常或 ROS 回退作废双租约，报告 Absent；两个源须在新 epoch 重新到达。非有限时钟同样 fail closed。
4. 接收租约与 watchdog 使用 steady 时间；ROS 暂停后没有新合法状态仍超时。ROS 前跳不延长接收租约，规划来源戳仍按 ROS 年龄校验。
5. 仅双源 Fresh 可清 TIMEOUT；路径恢复不掩盖陈旧状态。健康数据不清 REQUEST/FAULT，reset 另受双源门约束。
6. 监控 IDLE 不授予任务许可；仲裁器 IDLE/ARMED 保持制动，显式 ARM→START 才可行驶。已有 RUNNING 的可恢复 TIMEOUT 可在健康恢复后恢复，未新增授权。仲裁器重启回任务 IDLE；监控心跳中断超过仲裁预算锁存 FAULT。
7. 名义软件检测预算 = 最后合法接收 + 超时阈值 + 一拍 50Hz watchdog；DDS/调度/仲裁延迟另计。集成测试预声明 0.35s 调度容差，不等于实车制动/停车距离预算。

## 测试证据

环境：Debian 13，GCC 14.2.0，CMake 3.31.6，clang-format 19.1.7，cppcheck 2.17.1；没有 `/opt/ros`、`ros2`、`colcon` 或 `rclpy`。纯核依赖 GTest/Eigen 安装在临时用户前缀，通过 PATH / LD_LIBRARY_PATH / CMAKE_PREFIX_PATH 使用，无生产依赖修改。

容器在 ptrace 下不支持 LSan，设置 `ASAN_OPTIONS=detect_leaks=0`；AddressSanitizer 和 UndefinedBehaviorSanitizer 保持启用，不能据此声称检过泄漏。

| 命令/范围 | 结果 | 原始证据 |
| --- | --- | --- |
| 旧判定回归：`test_input_freshness_red --gtest_output=json:...` | **预期 FAIL, rc=1**；车辆最后接收 100.0s，路径 100.4–101.0s 持续到达，错误地仍健康，共 7 个失败断言 | `evidence/issue107/red.log`、`red.json` |
| 修复后 `test_input_freshness --gtest_output=json:...` | **PASS, rc=0, 16/16** | `green.log`、`green.json` |
| `ASAN_OPTIONS=detect_leaks=0 bash scripts/core_standalone_check.sh` | **PASS, rc=0, 36/36 测试程序**；覆盖审计通过；包含旧 stop/task/arbiter/PP 回归 | `core.log` |
| `bash scripts/lint_cpp.sh` | **rc=0**；clang-format PASS。cppcheck 有基线解析/性能诊断，不能称“0 issues”；脚本未设 error-exitcode，不以 rc=0 消除这些诊断 | `lint.log`、`baseline-cppcheck.log` |
| `bash scripts/lint_shell.sh` | **PASS, rc=0**；bash -n / 禁止 no-daemon 断言 / readiness 自测通过；shellcheck 缺失，建议层跳过 | `shell.log` |
| `python3 -m py_compile scripts/vehicle_state_lease_smoke.py` | **PASS, rc=0**；只验证 Python 语法 | 可按命令重现 |
| 下列带完整参数 `colcon build` | **NOT_RUN**，启动命令 rc=127，缺 colcon/ROS | `ros-build.log` |
| `colcon test --packages-select safety_monitor pure_pursuit` | **NOT_RUN**，rc=127 | `ros-test.log` |
| `colcon test-result --verbose` | **NOT_RUN**，rc=127 | `ros-test-result.log` |
| `bash scripts/vehicle_state_lease_smoke.sh build/issue107/integration` | **NOT_RUN**，rc=2，缺 rclpy | `ros-integration.json` |
| `bash scripts/fault_injection_smoke.sh build/issue107/fault` | **NOT_RUN**，rc=2；前置注入明确缺 rclpy，旧场景未运行 | `fault.log` |

**RED 的证据范围**：这是从旧节点接收判定抽离的纯 C++ 逻辑复现，不是运行旧 ROS 节点或真实 Topic 的 RED。不能把上述核心 PASS 写成 Topic、QoS、ROS adapter、HIL 或实车 PASS。首次全量核心测试还有旧 PP 夹具失败，已在原始基线 worktree 单独复核相同失败（`baseline-throttle.log`），并由独立测试提交修正。

在已安装 GTest/Eigen 的环境重放 RED：

```bash
git worktree add ../issue107-red c9e70859dfea0f5ec20e75532a180c67028cb22b
cd ../issue107-red
cmake -S tests/core_standalone -B build/core_standalone -DCORE_ASAN=ON
cmake --build build/core_standalone --target test_input_freshness --parallel 4
ASAN_OPTIONS=detect_leaks=0 build/core_standalone/test_input_freshness
# 期望非零；同一用例在修复分支必须返回零
```

Jazzy 补验命令（源依赖须先按工程 rosdep 安装；门禁需要仿真器、profiler 等完整工作区）：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to safety_monitor pure_pursuit \
  --packages-ignore hello_world huat_launch --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 -DCMAKE_LINKER_TYPE=MOLD \
  -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select safety_monitor pure_pursuit
colcon test-result --verbose
bash scripts/vehicle_state_lease_smoke.sh build/vehicle_state_lease
# 完整旧 smoke 另需先构建整个仿真依赖链，使用工程全量命令：
colcon build --symlink-install --packages-ignore hello_world huat_launch --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 -DCMAKE_LINKER_TYPE=MOLD \
  -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DBUILD_TESTING=ON
source install/setup.bash
timeout 20 ros2 daemon stop || true
ros2 daemon start
sleep 3
bash scripts/fault_injection_smoke.sh
```

本地若有既知 UDP 发现限制，再按工程约定设 `FASTDDS_BUILTIN_TRANSPORTS=SHM`；不以修改超时/判据迁就失败。

JSON 时间线使用 steady 接收时刻，记录最后输入、trigger、`/system/stop`、`/system/state` 与每帧最终制动值。不得用没有新消息的旧快照冒充持续保护。异常/超时返回非零；缺依赖返回 2 并写 NOT_RUN。

## CI 状态

- #106 已有失败：[run 43686642036](https://github.com/zhangjszs/ROS2-LTS-Final/actions/runs/43686642036)，失败于 clang-format，后续构建/测试未执行。该失败不归因于本修复。
- #108 新 CI：[run 43830539804](https://github.com/zhangjszs/ROS2-LTS-Final/actions/runs/43830539804)，发布报告时运行中；尚无 Jazzy 成功结论。若后续状态变化，以 PR 检查和后续证据追加为准。
- 本机 cppcheck 2.17.1 对既有 GTest 文件产生解析诊断；已在原基线的 `test_velocity_profiler.cpp` 复核同样症状。未扩展本单去重构无关测试或宣称静态分析全干净。

## 遗留风险与回退

- ROS 适配层编译、Topic/QoS、并发和 DDS 时序尚未在本机执行；新注入脚本的运行正确性也待 Jazzy 验证。
- 定位质量、设备测量时间语义、实际制动/执行器延迟、硬件 watchdog/驱动使能仍无标定/HIL/实车证据。
- `HuatStop` 仍只有 bool，REQUEST 依赖独立请求通道和两级 reset 协调；断连、请求持久重发、快速重启等系统风险仍属 #16。
- 仲裁器在已经 RUNNING 的 TIMEOUT 恢复后允许继续；监控重启若短于心跳失联阈值，是否要求全新人工 ARM→START，仍由 #16 的系统级重启授权协议验收。本单不改变既有任务状态机或硬件使能策略。
- 回退使用 revert 生产修复提交恢复原行为；这会重新引入已确认的定位断流缺口，必须继续禁止作为已验收实车基线。更安全的处置是保留 Draft/OPEN，在隔离环境修复验证。

## 下一步：Planner 验收

1. 复核 RED/GREEN 的证据层级、完整 SHA、源码树与 36 个核心程序结果，确认定位门与规划门真正独立。
2. 在 Jazzy 跑新增注入和旧故障门禁，审查每一场景 JSON 时间线、持续心跳、两种恢复顺序、REQUEST/FAULT 与重启授权，补齐真实 Topic RED/GREEN。
3. 检查 #108 CI 全部结果，区分新失败、基线失败和环境缺失；本单不得以“静态分析/核心通过”验收全部集成标准。
4. 单独安排 HIL/台架/实车预算与执行器验证；仅 Planner 根据证据决定是否关闭 #107、处理 #106 的合入顺序。
