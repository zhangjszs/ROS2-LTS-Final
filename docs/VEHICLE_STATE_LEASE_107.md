# Issue #107 最终执行报告（2026-11-16）

Issue: [#107](https://github.com/zhangjszs/ROS2-LTS-Final/issues/107)

状态: **OPEN + in-review**。代码和可证伪的软件证据已交付；不关闭 Issue、不合并 PR、不部署实车。

基线 SHA: `8d9479ef930e75192a3391259b1c8486d03a3f79`。开工及首次发布前均确认 #106 HEAD 未变化；没有覆盖基线分支。

新分支: `fix/vehicle-state-lease-107`

PR: [Draft #108](https://github.com/zhangjszs/ROS2-LTS-Final/pull/108)，base=`fix/fsac-fail-closed-arbiter-20261115`，stacked on #106。

已验证的控制/测试代码 SHA: `d903b10262ba6777dc548a96063fb8e50633e5af`。本报告及证据索引提交只更新文档。

## 结果与证据范围

先在抽离的旧生产判定上取得纯 C++ RED，再实现双租约，随后用远端 Jazzy 临时构建原始 SHA 的监控补齐真实 Topic RED→GREEN。原始监控通过启动场景后，在“单帧合法车辆状态 + 持续 10Hz 规划”场景超过预算仍不停车；修复版同一场景及完整 21 场景通过。

本机 Debian 13 没有 ROS 2/colcon/rclpy，最初 ROS 门禁明确 NOT_RUN。后续 ROS 结果全部来自 GitHub Actions Jazzy，不能写成本机执行。执行后段本地环境断连，源代码已在 GitHub 保存，后续测试修正通过连接的 GitHub 提交并由 CI 执行。

**代码版本 `d903b10262ba6777dc548a96063fb8e50633e5af` 的全量 [Jazzy CI 43839420046](https://github.com/zhangjszs/ROS2-LTS-Final/actions/runs/43839420046) 为 SUCCESS。** 677 包测试、36 个核心测试程序、新增 21 场景、原有 10 故障场景、QoS、闭环正常/故障样本、八字正负样本均通过。后续本报告提交只更新文档和证据索引，其检查状态以 PR Checks 为准。

## Commits：完整 SHA 与链接

| Commit | 内容 |
| --- | --- |
| [c9e70859dfea0f5ec20e75532a180c67028cb22b](https://github.com/zhangjszs/ROS2-LTS-Final/commit/c9e70859dfea0f5ec20e75532a180c67028cb22b) | test(safety): extract existing liveness decision and reproduce vehicle dropout (#107) |
| [41fd324423fbd040d10bfd2061fff7c9da0c43dc](https://github.com/zhangjszs/ROS2-LTS-Final/commit/41fd324423fbd040d10bfd2061fff7c9da0c43dc) | style: correct inherited PR 106 C++ formatting separately |
| [345bec67b9a9011ef7ef09219d80948d27c43392](https://github.com/zhangjszs/ROS2-LTS-Final/commit/345bec67b9a9011ef7ef09219d80948d27c43392) | test(pp): align inherited pedal clamp fixture with nonpositive-target stop contract |
| [b290425bdc3971ba54797f44ba0770fc0b8a395b](https://github.com/zhangjszs/ROS2-LTS-Final/commit/b290425bdc3971ba54797f44ba0770fc0b8a395b) | fix(safety): require independent steady receive leases for state and path (#107) |
| [b2bb06e00d3f8596a21d5c90987bc15a8b2ca853](https://github.com/zhangjszs/ROS2-LTS-Final/commit/b2bb06e00d3f8596a21d5c90987bc15a8b2ca853) | docs(safety): record issue 107 red-green evidence and explicit ROS validation gaps |
| [0ee77e3875a003ae58ea61100a7be55d55e9c375](https://github.com/zhangjszs/ROS2-LTS-Final/commit/0ee77e3875a003ae58ea61100a7be55d55e9c375) | test(safety): expire both independent leases before recovery-order scenarios |
| [09db7a7c9ce6fe0dc6e61c78b0047e476b5a58ba](https://github.com/zhangjszs/ROS2-LTS-Final/commit/09db7a7c9ce6fe0dc6e61c78b0047e476b5a58ba) | test(safety): align isolated smoke parameters and stop requests with PR 106 contract |
| [6d402f012c2728839a45e98158e569517d28aea5](https://github.com/zhangjszs/ROS2-LTS-Final/commit/6d402f012c2728839a45e98158e569517d28aea5) | test(safety): prove baseline monitor RED and fixed monitor GREEN on real ROS topics |
| [d573712859317c40059ccfd66179d0cdc03de3ee](https://github.com/zhangjszs/ROS2-LTS-Final/commit/d573712859317c40059ccfd66179d0cdc03de3ee) | test(safety): record monitor-side receive lease timestamps in fault timelines |
| [4070a10ed7a968aa790d51e22059b26c401702c6](https://github.com/zhangjszs/ROS2-LTS-Final/commit/4070a10ed7a968aa790d51e22059b26c401702c6) | test(safety): distinguish explicit request reset from data-only recovery and idle reset |
| [7a1c765a5b38190327acf0440550bde79a3c59e3](https://github.com/zhangjszs/ROS2-LTS-Final/commit/7a1c765a5b38190327acf0440550bde79a3c59e3) | test(safety): reject false RED caused by startup failure or missing planner traffic |
| [b85d0c6351613970ee196bd661880bb360a9474e](https://github.com/zhangjszs/ROS2-LTS-Final/commit/b85d0c6351613970ee196bd661880bb360a9474e) | test(safety): stamp simulation fixture paths in the same ROS clock domain |
| [9197772172dbb1234c238d8e11bba279cb70faae](https://github.com/zhangjszs/ROS2-LTS-Final/commit/9197772172dbb1234c238d8e11bba279cb70faae) | ci(safety): run required fault gate despite unrelated smoke failures after successful build |
| [0ced8f225434ab0358e529ee50225138c341263b](https://github.com/zhangjszs/ROS2-LTS-Final/commit/0ced8f225434ab0358e529ee50225138c341263b) | test(safety): await explicit reset acknowledgement across independent DDS topics |
| [d903b10262ba6777dc548a96063fb8e50633e5af](https://github.com/zhangjszs/ROS2-LTS-Final/commit/d903b10262ba6777dc548a96063fb8e50633e5af) | test(safety): acknowledge restarted monitor epoch and retain fault priority on late join |

## 根因：基线代码路径

- `src/safety/safety_monitor/src/safety_monitor.cpp:205–219`：有限值校验后永久设置 `has_vehicle_state_`；断流不会撤销。
- 同文件 `167–202`：新规划只检查“曾收到状态”即可调用 `onPathReceived()` 清 TIMEOUT。
- 同文件 `264–287`：watchdog 只检查 planner lease，新规划持续时提前返回。
- 同文件 `241–259`：车辆年龄只用于人工 reset，没有进入运行期停车判定。

因此规划恢复可掩盖陈旧状态。系统保护不能借助 PP 私有 watchdog 来证明，测试持续提供合法 MPC 正油门以避免这种掩盖。行号以基线 SHA 为准。

## 修改：逐文件说明

| 文件 | 修改 |
| --- | --- |
| `src/safety/safety_monitor/include/safety_monitor/input_freshness.h` | 新增独立车辆/规划 FreshnessLease、有限状态校验、有限正超时参数、可注入 steady/ROS 时间；回退/非有限时钟作废旧 epoch，提供 age/freshness |
| `src/safety/safety_monitor/src/safety_monitor.cpp` | 去掉永久健康布尔和车辆代续规划租约；回调/watchdog/reset 共用双源门；50Hz WallRate、10Hz 持续 stop；契约 QoS；诊断精确 steady 检查时刻及接收年龄 |
| `src/safety/safety_monitor/test/test_input_freshness.cpp` | 16 项确定性纯核回归；组合生产停车/任务状态机和仲裁核；覆盖未 ARM/START、锁存、重启、时钟、非有限输入与恢复顺序 |
| `src/safety/safety_monitor/CMakeLists.txt` | 注册测试；停车状态机静态库供节点及测试链接，避免重复编译业务源码 |
| `tests/core_standalone/CMakeLists.txt` | 新测试进入 ASan/UBSan 和覆盖审计，链接同一状态机库 |
| `scripts/vehicle_state_lease_smoke.py` | 生产监控/仲裁进程，合法 MPC 帧持续输入；21 场景、自动断言、JSON 时间线、失败快照；显式 reset 等待状态确认；新监控双 Absent 确认其 epoch |
| `scripts/vehicle_state_lease_smoke.sh` | 独立 DDS 域/localhost、150s 总预算、进程清理、缺依赖 NOT_RUN、超时非零 |
| `scripts/vehicle_state_lease_red_green.sh` | 备份/临时构建原始监控并精确判定 RED，恢复源码重建 GREEN；EXIT 恢复源文件；拒绝启动失败/无持续规划等假 RED |
| `scripts/fault_injection_smoke.sh` | 先运行独立双租约场景；旧外部停车走 request/撤销/两级 reset；显式 daemon 启动；仿真时间戳；晚加入在监控已被杀的前提下允许安全 FAULT，强化零油门+有效制动断言 |
| `scripts/qos_contract_check.sh` | 隔离重映射出口显式 simulation；断言监控真实订阅车辆和规划两个输入 |
| `scripts/closed_loop_sim_smoke.sh` | Python 路径发布器显式 use_sim_time，统一来源时间域，判据不变 |
| `scripts/closed_loop_fault_smoke.sh` | 同上，保留全部故障及预算判据 |
| `scripts/acceleration_finish_smoke.sh` | 隔离仿真显式 simulation；Python 路径戳使用仿真时间 |
| `scripts/skidpad_finish_smoke.sh` | 同上；不改变圈数/完赛负样本判据 |
| `scripts/skidpad_closed_loop_smoke.sh` | 隔离仿真显式 simulation，符合 #106 拒绝 real autostart 的约束 |
| `scripts/mpc_reject_smoke.sh` | Python 路径戳使用仿真时间；不改变拒解判据 |
| `.github/workflows/ci.yml` | 支持 stacked PR 的目标分支；真实 RED→GREEN；保存 JSON；构建成功后即使无关 smoke 失败也执行关键 fault gate，原失败仍保留 |
| `docs/INTERFACE_CONTRACT.md` | 双租约、时间域、预算、恢复/授权、设备测量时间边界 |
| `src/control/pure_pursuit/test/test_throttle_controller.cpp` | 独立修正旧限幅夹具：使用正目标但负误差；另验证零目标覆盖正 pedal_min；只改测试 |
| `.agent/STATE.md`、`.agent/HANDOFF.md` | 更新本任务执行事实、验证边界及 Planner 入口，保留原队列历史 |
| `.agent/DIARY/2026-11-16.md` | 追加最终结论和测试问题，不重写既有日记 |
| `docs/VEHICLE_STATE_LEASE_107.md`、`docs/evidence/issue107/*` | 本报告、既有核心原始日志/JSON、CI 摘要及原始 JSON artifact 索引 |

独立格式提交 `41fd324` 只处理 include/空白/换行，涉及以下十文件（PP 测试另有上述独立夹具修改）：

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

1. 车辆接收预算 `safety.state_timeout_sec=0.3` 秒，与 PP 默认值对齐；规划 `planner_timeout=0.5` 秒保持原预算。必须有限且 >0；均未完成实车标定。
2. 合法状态在本单指 x/y/theta/v 有限；只有合法接收续车辆租约。Header 是发布时刻，未解释设备 GPS 测量时间或质量枚举。
3. Absent / Stale 都停车；仅双源 Fresh 清 TIMEOUT。接收时钟 FromFuture 或 ROS 回退作废旧 epoch 两租约，对外呈 Absent；必须重收双源。非有限时钟同样 fail closed。
4. 接收租约与 watchdog 用 steady 时间；暂停 ROS 不冻结断流保护。规划来源戳仍按 ROS 时间验证，合法区间保持 `[-0.05, planner_timeout]`，无效帧不续租。
5. 健康数据不清 REQUEST/FAULT。监控 IDLE 不授予任务许可；仲裁器 IDLE/ARMED 无动力，须显式 ARM→START。
6. 既有 REQUEST 显式 reset 会保留原 RUNNING；FAULT/FINISHED reset 才回 IDLE。此为原任务契约，未在本单改变。是否要求所有 reset 后重新 ARM→START 属于 #16 的协议决策。
7. 软件名义检测预算为最后合法接收 + 对应 timeout + 一拍 50Hz watchdog；DDS/调度/仲裁延迟另计。测试 0.35s 调度容差不等于实际制动建立或停车距离预算。

## 测试证据：命令、环境与结果

本机：Debian 13 / GCC 14.2.0 / CMake 3.31.6 / clang-format 19.1.7 / cppcheck 2.17.1。GTest/Eigen 使用临时用户前缀，无生产依赖修改。容器 ptrace 不支持 LSan，`ASAN_OPTIONS=detect_leaks=0`；AddressSanitizer/UndefinedBehaviorSanitizer 保持启用，不声称检过泄漏。远端：Ubuntu 24.04 / ROS 2 Jazzy，工作流包含完整 rosdep 依赖及 mold/ccache。

| 验证 | 结果及证据 |
| --- | --- |
| 旧判定 `test_input_freshness_red` | 预期 FAIL rc=1，1 用例 7 失败断言；状态最后接收 100.0s，规划到 101.0s 仍错误健康；`red.log/red.json` |
| 修复后 `test_input_freshness` | PASS rc=0，16/16；`green.log/green.json` |
| `ASAN_OPTIONS=detect_leaks=0 bash scripts/core_standalone_check.sh` | PASS，36/36 测试程序，含旧 stop/task/arbiter/PP 回归；`core.log` |
| `bash scripts/lint_cpp.sh` | 命令 rc=0，clang-format PASS；本机 cppcheck 有基线解析/性能诊断，不能称 0 issues；`lint.log/baseline-cppcheck.log` |
| `bash scripts/lint_shell.sh`、Python py_compile | 本机及断连前已执行部分 PASS；最后远端脚本修改由 CI 实际执行验证；本机 shellcheck 建议层缺失明确跳过 |
| 本机 colcon build/test/test-result、ROS 注入 | NOT_RUN：缺 colcon/rclpy，rc=127/2；既有 `ros-*.log` 和 NOT_RUN JSON 保留 |
| Jazzy 全量 colcon build/test/test-result | PASS；已核对 677 tests、0 errors/0 failures/0 skipped；最新代码各对应步骤亦通过 |
| `bash scripts/vehicle_state_lease_red_green.sh` | 真实原始监控精确 RED，修复监控 GREEN；21/21 Topic 场景，含实际两种重启/时钟/锁存/授权 |
| `bash scripts/fault_injection_smoke.sh` | PASS：独立 21 场景前置 + 原有 10/10 场景；最终出口零油门/有效制动、锁存和晚加入已自动判定 |
| headless / QoS / benchmark / 闭环正常及故障样本 | 全部 PASS；八字完赛正样本及 REQUIRED_LAPS=3 可证伪负样本也通过 |

原始核心证据已提交于本目录下 `evidence/issue107/`，`manifest.json` 提供初始 14 文件 SHA256。远端证据索引为 `ci_runs.json`，包含精确运行 SHA、原生 artifact URL 和服务端 ZIP SHA256；每个 `ci-<run>-excerpt.log` 是日志摘录，原始 Topic `summary.json/timeline.json` 在对应 artifact ZIP 中。CI 原始 artifact 按仓库配置保留 14 天，验收时应下载归档；脚本可按固定提交重新生成。

时间线含 harness 输入发送/输出接收，以及 `monitor_input_leases` 中监控检查时刻与两源最后合法接收；没有把发送时刻冒充接收时刻。首次 21 GREEN 的 artifact 实测含 1108 条监控侧租约记录。

Jazzy 对应构建/测试命令（CLI cmake-args 重列默认项，开启测试）：

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
bash scripts/vehicle_state_lease_red_green.sh build/vehicle_state_lease_red_green
# 旧 smoke 还需要仿真器/profiler 等完整工作区；CI 实际采用同一参数的全量构建/包测试。
bash scripts/fault_injection_smoke.sh
```

## CI 状态：原有失败与新增问题

- #106 原 [43686642036](https://github.com/zhangjszs/ROS2-LTS-Final/actions/runs/43686642036) 失败于格式，后续未执行。由独立格式提交修复；后续 CI lint 已通过。
- 新增恢复夹具最初只等 0.3s 定位到期，规划 0.5s 尚 Fresh；以及把 REQUEST 显式 reset 误判为必须 IDLE。均根据源码/失败 JSON 修测试前提，未放宽停止阈值或改生产授权规则。
- DDS 不保证跨 Topic 顺序。单次 reset 曾因仲裁器尚未处理恢复而被安全拒绝；现按幂等语义有限重发，等待状态确认。重启监控以新实例双 Absent/steady epoch 作为就绪确认，保留严格输出不变量。
- 旧闭环 Python feeder 用系统时间戳、消费者用仿真时间，严格来源年龄门正确拒收。修正测试时钟，不删除安全门。旧晚加入场景先杀了监控，故可合法进入更安全的 FAULT；只放宽字面任务态到 IDLE/FAULT，同时强化零油门+有效制动，清洁重启 IDLE 在独立场景另验。
- setup-ros 多次 HTTP404 属环境失败，均在业务测试前结束，不计作 RED；已对失败 job 重试。

最新代码 [43839420046](https://github.com/zhangjszs/ROS2-LTS-Final/actions/runs/43839420046) **SUCCESS**，准确绑定 `d903b10262ba6777dc548a96063fb8e50633e5af`；没有剩余的新代码 CI 失败。本次仅文档/证据提交不改变受测控制代码，最终文档 HEAD 的新检查可能仍在运行。

## 遗留风险与回退

- 无 HIL/台架/实车证据，未做实际制动距离、执行器延迟、硬件 watchdog/驱动使能标定；没有实车部署或认证。
- 仅证明合法状态的接收活性。持续旧观测、定位质量枚举及 GPS/ROS 时间标定仍需 #12/#16 后续证据，不能由本单替代。
- HuatStop 仍只有 bool；REQUEST 的独立请求通道、两级 reset、快速重启/旧消息 epoch 和操作员安全网关的系统级协议仍需 #16 验收。
- 本单未把单调接收租约扩展为所有控制源的时钟协议；其他源暂停/回退及硬件断连矩阵仍需独立覆盖。
- 回退本分支会重引定位断流缺口；应保持隔离软件测试及 Draft，不能把原始基线当已验证实车方案。没有部署动作需要回滚。

## 下一步：Planner 验收清单

1. 核对完整 SHA、纯核与真实 Topic RED→GREEN；确认单帧定位场景曾先解除 TIMEOUT，规划持续到达，故障后 stop 与最终制动持续。
2. 审查 21 场景及旧 10 场景 JSON，重点为双恢复顺序、REQUEST/FAULT、ARM→START、新实例 epoch、暂停/回退；区分节点接收与 harness 发送。
3. 核对最新代码 CI 和最终文档提交的检查，确认原有门禁无退化；不得用本机 NOT_RUN 或单项 PASS 代替完整结论。
4. 安排 HIL/实车预算、设备质量/测量时间及硬件保护验证；仅 Planner 根据证据决定 #107 是否关闭和 #106/#108 合入顺序。
