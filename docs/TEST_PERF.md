# TEST_PERF — 测试性能基线与优化台账

> **目的**：记录 FSAC 2026 测试套件的性能基准、合理性审计结论、加速方案与优化历史，
> 让"测试为什么慢、砍/改什么、改完快多少"可追溯。
>
> **数据口径**：
> - 本地 = 开发机（24C/19G，ROS 2 **lyrical**，`/opt/ros/lyrical`），入口 `bash scripts/drive_gates.sh`；
> - CI = GitHub Actions `ubuntu-24.04`（4 核，ROS 2 **jazzy**），工作流 `.github/workflows/ci.yml`（push/PR）与 `nightly.yml`（cron）；
> - 两者跑**同一套** `scripts/drive_gates.sh` 门禁判据（ci.yml 为等价分步），逐门禁计时见 `build/drive_gates/last.json` 与 `history.jsonl`。
>
> **幂等规则**：本文件存在即视为基准已建立，后续会话只读它作对照基准，仅在「优化记录」表追加条目；
> 重置基准的唯一方式是人工删除本文件。
>
> **覆盖率现状**：仓库**没有**覆盖率门禁（无 lcov/gcov 配置；`core_standalone_check.sh` 的"覆盖审计"
> 指"每个测试文件必须在 CMakeLists 登记或写明排除原因"，不是行覆盖率）。行覆盖率先行记为"未测量"，
> 建立方案见阶段三 C-6。

---

## 一、测试规模与构成（2026-10-12 实测）

| 项 | 数量 | 说明 |
| --- | --- | --- |
| gtest `TEST()` 用例 | **475** | 41 个 `src/**/test/test_*.cpp`；无 `TEST_F`/`TEST_P` 参数化（仅 2 处 fixture） |
| pytest 用例 | **31** | `src/vision_ros/test/test_vision_quality.py`（ament_python，随 `colcon test` 执行） |
| `colcon test` 执行总数 | **506** | 上两项之和；`hello_world` 的 5 个 linter 测试被 `--packages-ignore` 排除（CI 不跑） |
| core_standalone 独立测试可执行 | **28** | `tests/core_standalone/CMakeLists.txt` 的 `add_core_test`，ASan+UBSan、无 ROS |
| 仅 colcon 轨（无 sanitizer）的测试文件 | **12** | 41 个测试文件中 29 个已在 core_standalone 登记；余 12 个依赖生成消息头/PCL，在 CMakeLists 排除清单中逐个注明原因（#40 约定） |
| 运行时门禁（DDS/多节点） | **6** | headless-smoke、qos-contract、closed-loop、closed-loop-fault、fault-injection、mpc-reject-smoke |
| 其他门禁 | **8** | build、test-run、test-enforce、lint、cpp20、core-standalone、benchmark、tuning-precheck |

**通过率**：`colcon test` 506/506 通过（`test-enforce` PASS，0 失败，见下）。

## 二、基准测量（2026-10-12）

### 2.1 本地全量实测（本次，环境退化，仅作过程证据）

`FASTDDS_BUILTIN_TRANSPORTS=SHM bash scripts/drive_gates.sh`，总时长 **3215s**，`hard_fail=1`：

| 门禁 | 本次本地 | 结果 | 备注 |
| --- | --- | --- | --- |
| build | 104s | PASS | ccache 热（97% cacheable） |
| test-run | 3s | PASS | 506 用例计算总量仅 **0.36s** |
| test-enforce | 1s | PASS | 0 失败 |
| lint | 3s | PASS | clang-format + cppcheck + bash -n |
| cpp20 | 111s | PASS | 见阶段二 B-2：与 build 门禁参数不一致，白跑一遍 configure |
| core-standalone | 6s | PASS | 28 个 ASan+UBSan 可执行（增量） |
| benchmark | 6s | PASS | 离线确定性核心 + 基线对账 |
| tuning-precheck | 1s | PASS | |
| headless-smoke | 821s | **FAIL** | 环境性失败，见 2.3 |
| qos-contract | 227s | **FAIL** | 同上 |
| closed-loop | 198s | **FAIL** | 同上 |
| closed-loop-fault | 1337s | **FAIL** | 同上（墙钟看门狗兜底拉长） |
| fault-injection | 202s | **FAIL** | 同上 |
| mpc-reject-smoke | 194s | **FAIL** | 同上 |

### 2.2 健康参照（本地 2026-10-08 全量 656s 全绿 + CI Nightly 2026-10-11 逐门禁）

同代码在同机器健康网络下的上次全量：**656s 全绿**。CI Nightly（10-11，jazzy）逐门禁：

| 门禁 | 本地 10-08 | CI Nightly 10-11 | 备注 |
| --- | --- | --- | --- |
| build | 108s | 83s | ccache 热 |
| test-run + test-enforce | 3+1s | 11+0s | |
| lint | 2s | 1s（FAIL，clang-format 18/21 skew，#61 已修） | |
| cpp20 | 105s | 1s（同因 FAIL） | 与 build 参数双轨，见 B-2 |
| core-standalone | 6s | **86s（冷构建）** | ASan+UBSan 全编 |
| benchmark | 6s | 4s | |
| tuning-precheck | 2s | 1s | |
| headless-smoke | 19s | 17s | |
| qos-contract | 16s | 15s | |
| closed-loop | 17s | 13s | |
| **closed-loop-fault** | **258s** | **320s** | **最大单项**（6 相串行） |
| fault-injection | 59s | 46s | |
| mpc-reject-smoke | 53s | 48s | 两臂串行 |
| **门禁合计** | **656s** | **646s** | 另加 apt/rosdep/setup-ros ≈ 4-5 min |

CI 端到端（含环境准备）：近 5 次成功 **12m29s / 13m43s / 14m31s / 14m48s**（中位 ≈ 14m）；
Nightly 14m55s / 15m15s。

### 2.3 本地 5 个 DDS 门禁失败的定性（环境性，非代码缺陷）

证据链：
1. 失败门禁的日志显示节点**均正常启动并干净退出**（如 urinay "process has finished cleanly"），
   失败点是"15s 内未在 ROS 图上发现节点"——本机当日 DDS 发现严重退化（`ros2 node list` 单次调用
   从秒级劣化到 60s+；AGENTS.md 已预警 VPN 挡 UDP 组播场景）；
2. **同一 commit**（`8c7db61` 之后主干）在 CI 上三天前全绿（run `38257427111` 等），Nightly 10-11
   六个 DDS 门禁同样全绿且快（headless 17s / qos 15s / closed-loop 13s / closed-loop-fault 320s）；
3. Nightly 10-11 唯一的失败是 lint/cpp20 的 **clang-format 版本 skew**（CI apt 18 vs 本地 21），
   与 DDS 无关，#61 已修复。

**结论**：本地当日测量只能采信非 DDS 门禁（build/test/lint/cpp20/core/benchmark/tuning）；
DDS 门禁的基线以 2.2 的健康数据为准。这条"本地环境退化 → 全红"的传导路径本身列入阶段三 F-1 治理。

另记一笔 CI 基础设施抖动：run `38258621269` 在 `Prepare ROS 2 jazzy`（setup-ros 动作）挂死 45 分钟，
被 job 级 `timeout-minutes: 45` 兜底取消——不是仓库门禁问题，但说明该步骤缺**步骤级**超时（阶段二 A-3）。

### 2.4 用例级 Top 10 最慢测试（`build/*/test_results/**/*.xml`，本次实测）

全部 506 个用例计算总时长 **0.36s**；最慢用例 0.137s。单测层面**不存在值得优化的耗时**：

| # | 耗时 | 测试 |
| --- | --- | --- |
| 1 | 0.137s | `mpc_controller.MpcAcceptanceContract.LargerIterationBudgetNeverLosesAcceptance` |
| 2 | 0.087s | `lidar_cluster.LidarClusterComponentTest.ConstructWithoutBadWeakPtr` |
| 3 | 0.030s | `skidpad_planner.IcpApfPlanner.DifferentialEquivalenceWithLegacyPCL` |
| 4 | 0.020s | `safety_monitor.JThreadCooperativeCancellationTest.RequestStopGracefulExit` |
| 5 | 0.015s | `vision_ros.test_vision_quality.TestQualityLadder.test_sharp_is_good` |
| 6 | 0.011s | `lidar_cluster.LidarClusterComponentTest.DirectLidarClusterInstantiation` |
| 7 | 0.010s | `safety_monitor.JThreadCooperativeCancellationTest.ConditionVariableAnyInterruptibleWait` |
| 8 | 0.009s | `vision_ros...TestDefaultDriftGate.test_node_defaults_match_test_constants` |
| 9 | 0.008s | `vision_ros...TestDefaultDriftGate.test_quality_thresholds_mirror_node_defaults` |
| 10 | 0.007s | `vision_ros...TestQualityLadder.test_heavy_blur_is_unusable` |

### 2.5 门禁级 Top 10 耗时（真正的耗时分布，健康基线）

| # | 门禁 | 本地 10-08 | CI 10-11 | 占比(本地) |
| --- | --- | --- | --- | --- |
| 1 | closed-loop-fault | 258s | 320s | 39% |
| 2 | build | 108s | 83s | 16% |
| 3 | cpp20 | 105s | ~1s* | 16% |
| 4 | fault-injection | 59s | 46s | 9% |
| 5 | mpc-reject-smoke | 53s | 48s | 8% |
| 6 | core-standalone（冷） | 6s(热) | 86s(冷) | — |
| 7 | headless-smoke | 19s | 17s | 3% |
| 8 | qos-contract | 16s | 15s | 2% |
| 9 | closed-loop | 17s | 13s | 3% |
| 10 | test-run | 3s | 11s | <1% |

\* CI 10-11 cpp20 与 build 同参数（ccache 全中）故快；本地 10-08 为 105s 说明参数双轨见 B-2。

---

## 三、阶段一：测试合理性审计（只分析，未改动）

### 3.1 🔴 建议删除：**无**

逐类核查证据：
- **Flaky**：无按设计抖动的测试。RNG 两处（`test_icp_apf_planner`、`test_sensor_model_core`）**全部固定种子**；
  无墙钟/时序断言（`grep steady_clock|elapsed` 在 `test_*.cpp` 零命中）；`test_stop_state_machine` 的
  3 处 sleep 为 5-20ms、断言余量 ≥4×，不构成抖动源。DDS 门禁的环境敏感性是**运行环境**问题（见 2.3 / F-1），
  不是测试自身 flaky。
- **重复测试**：见 3.2 黄项 R-1（评分核 13 个同名用例双轨），但核心轨有 sanitizer 加成、ROS 轨有 PCL 适配层
  差异，属"可重构去重"而非"纯重复"，且耗时在毫秒级——按删除处理收益为负（丢 sanitizer 覆盖）。
- **过时测试**：无。测试与 issue 编号挂钩（#12/#14/#15/#16/#17/#22/#24/#30/#32/#38/#40/#43/#44/#45/#50/#53/#54/#59/#60/#61），
  均为现行判据的可执行规格。
- **无断言/弱断言**：无。启发式扫描（TEST 块 <5 行）命中的全是单断言边界用例（如 `BoundaryValues`、
  `ZeroSpeedReturnsAlphaMin`），属精确边界测试而非弱测试。
- **过度 mock**：无。全仓 `test_*.cpp` **零 gmock**，测试直连真实头文件/真实类型。

### 3.2 🟡 建议重构

| # | 对象 | 问题 | 依据 | 预计节省 |
| --- | --- | --- | --- | --- |
| R-1 | `test_scoring_core.cpp`（core 轨，31 例） vs `test_lidar_cluster_scoring.cpp`（colcon 轨，32 例） | **13 个同名用例双轨**：ScoreAspectPenalty×4、ScoreSizePenalty×4、ComputeConfidence×3、ScoreTilt×2 | 逐一双轨存在；core 轨用例更全且带 ASan+UBSan | ~0s（毫秒级）；价值在维护成本与"改一处忘另一处"的漂移风险 |
| R-2 | `test_pure_pursuit.cpp`（49 例单文件） | 粒度过大：pp_math / input_guard / vehicle_command_encoder 三个单元混居 | 单文件 49 个 TEST()，为全仓最大 | ~0s；可读性/定位成本 |
| R-3 | 28 个测试文件在 colcon 轨与 core_standalone 轨**各跑一遍** | 双轨执行 | 设计使然（sanitizer 加成），**不建议合并**；记录在案避免后人误删 | — |
| R-4 | 层级：无"单元可覆盖却写成 E2E"的案例 | `test_pure_pursuit.cpp` 名为节点测试实为纯函数测试（只因 encoder 拉生成消息头进不了 core 轨） | 读码确认 | — |

### 3.3 🟢 保留

其余全部：475 个 gtest 用例 + 31 个 pytest + 6 个 DDS 集成门禁 + 8 个静态/离线门禁。
共同特征：行为断言、边界/负样本齐全、与 issue 判据一一对应、种子确定、无墙钟依赖。

### 3.4 耗时 Top 10 处置优先级

按 2.5 的门禁级排序，优先处理 **closed-loop-fault（39%）→ cpp20（16%，纯浪费）→ build（16%）**；
用例级 Top 10 全部 <0.14s，**不处理**。

---

## 四、阶段二：加速方案（附 diff / 预估 / 风险；均**待用户确认后实施**）

> 实施规范：每项落地后按第五节回写「日期+改动+前后指标+commit」；未确认项进第六节待办。
> 与仓库约定冲突时以仓库为准（#17 证据链、调参冻结协议、issue 驱动流程）。

### B-1（最高优先）CI 路径过滤 + 并发取消——砍掉纯文档/agent 提交的全量 CI

**诊断**：`.agent/` 与 `*.md` 提交也触发 11-15 min 全量 CI。近 12 次运行中 4 次为
`chore(agent)`（仅改 `.agent/` 文本），每次 11-15 min 纯浪费；快速失败的红（3m 级）也是同类。
另无 `concurrency`：连续 push 不会取消旧在途运行。

**方案 diff**（`ci.yml` 与 `nightly.yml` 同改）：

```diff
 on:
   push:
     branches: [ main ]
+    paths-ignore:
+      - '**.md'
+      - '.agent/**'
+      - 'docs/**'
   pull_request:
     branches: [ main ]
+    paths-ignore:
+      - '**.md'
+      - '.agent/**'
+      - 'docs/**'
+
+concurrency:
+  group: ${{ github.workflow }}-${{ github.ref }}
+  cancel-in-progress: true
```

（nightly 的 `schedule` 不受 paths 影响，只需加 `concurrency`。）

**预估**：每周省 3-5 次全量 CI ≈ 45-70 runner-min/周；连续 push 旧运行即取消。
**风险**：低。仅改触发条件与并发语义，判据不变；注意 `docs/INTERFACE_CONTRACT.md` 等文档改动不再触发 CI
（若认为契约文档需联动校验，可把 `docs/INTERFACE_CONTRACT.md` 移出 ignore 列表）。

### B-2 统一构建参数——消除 cpp20 门禁与 build 门禁的双轨 configure

**诊断**：CLI `--cmake-args` 会**整体替换** `colcon_defaults.yaml` 的列表。build 门禁传
`Release+MOLD+ccache+TESTING=ON`；`check_cpp20.sh` 只传 `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_TESTING=ON`，
与 defaults 文件里的 `Release+MOLD+ccache` 不一致 → 每次全量跑都要再走一遍 18 包 configure
（本地实测 105-111s，占全量 16%）。CI 上因 ccache 命中显得快，但 configure 成本仍在。

**方案 diff**（`scripts/check_cpp20.sh`）：

```diff
-colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_TESTING=ON
+colcon build --symlink-install \
+  --cmake-args \
+    -DCMAKE_BUILD_TYPE=Release \
+    -DCMAKE_CXX_STANDARD=20 \
+    -DCMAKE_LINKER_TYPE=MOLD \
+    -DCMAKE_C_COMPILER_LAUNCHER=ccache \
+    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
+    -DBUILD_TESTING=ON \
+    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

**预估**：cpp20 门禁 105s → ~10-20s（与 build 门禁同参数后为增量 no-op）；本地全量 656s → ~560s（-14%），
Nightly 同益。**风险**：低。编译参数与 build 门禁逐项一致，仅新增 compile_commands 导出（供 clangd）。
注意：改后首次跑仍有一次全编（参数切换），之后即增量。

### B-3 closed-loop-fault 六臂并行（最大单项 39%）

**诊断**：`closed_loop_fault_smoke.sh` 的 6 个场景（nominal/offroute/collision/reverse/dnf/timeout）
串行执行，墙钟上限合计 600s，实测 258-320s。六臂相互独立（各自起一套 sim→profiler→PP→sim 回路），
仅共用 DDS 发现域。

**方案**：每臂分配独立 `ROS_DOMAIN_ID`（如 `42+i`）后台并行，`wait` 汇总后**沿用现有判据**
（JSON 断言逐字不改）。骨架：

```bash
for i in "${!SCENARIOS[@]}"; do
  ( export ROS_DOMAIN_ID=$((42+i)); run_arm "${SCENARIOS[$i]}" ) &
done
wait
```

**预估**：258-320s → ~120-150s（瓶颈转为最慢的 nominal 臂 + 启动开销），全量再省 ~15%。
**风险**：**中**。① 容差型判据在 CPU 争抢下，CI 4 核跑 6 套 100Hz 物理可能低于实时——脚本按**仿真时长**
喂入已缓解，但墙钟看门狗是兜底，需先在 CI 实测 2-3 晚（可借 Nightly 的 history.jsonl 数连续绿）；
② 域隔离若配置不当会跨臂串扰（判据必须仍逐臂读各自的 `$OUT_DIR/<name>.json`）。
**建议**：先落 `PARALLEL_ARMS` 开关（默认 1），Nightly 试跑转硬门禁后再开 CI。

### B-4 CI job 拆分——lint/core 快速失败 + 构建一次 + 运行时门禁并行

**诊断**：`ci.yml` 单 job 14 min 全串行：lint(1m) → core(1.5m) → build(2m) → test(0.2m) →
headless/qos/benchmark/closed-loop/fault/mpc(≈8m)。lint 红要等 8 分钟才对开发者可见。

**方案**（判据不变，只改拓扑）：
- job `static`：lint + core-standalone（≈3 min，快速失败）；
- job `build-test`：build + colcon test + test-result，`actions/upload-artifact` 上传 `install/`（≈4 min）；
- job `runtime-gates`：下载 install artifact 后，6 个 DDS 门禁拆 2-3 个并行步骤或 matrix
  （headless+qos+benchmark 一组 ≈1 min；closed-loop 组 ≈6 min；fault-injection+mpc 一组 ≈2 min）。

**预估**：CI 端到端 14m → ≈8m（关键路径 = build-test 4m + runtime 6m）。
**风险**：**中高**。① `install/` artifact 体积（需实测，估 100-300MB）与缓存命中；
② #17 的"连续绿"证据链目前挂在 Nightly 的 drive_gates history.jsonl 上，CI 拆分不破坏它
  （Nightly 仍单 job 全跑），但 ci.yml 的步骤级超时护栏要逐 job 重设；
③ 与 B-3 叠加时门禁并行度需重新配平。
**建议**：与 B-3 合并为一轮改动，先出实测数据再合。

### B-5 本地增量执行（affected packages）——单包改动的本地反馈

**诊断**：本地改一个包也要全量 656s；`QUICK=1`（30s）不覆盖构建与单测。

**方案**：新增 `scripts/affected_gates.sh`（本地 opt-in，不进 CI）：

```bash
mapfile -t PKGS < <(git diff --name-only origin/main... -- src | \
  sed -n 's|^src/\([^/]*\)/.*|\1|p' | sort -u)
[ ${#PKGS[@]} -eq 0 ] && { echo "no src changes"; exit 0; }
colcon build --symlink-install --packages-above ${PKGS[*]} --cmake-args ...同 B-2...
colcon test --packages-select ${PKGS[*]} && colcon test-result --verbose
```

**预估**：单包改动本地 656s → ~150-250s。
**风险**：低-中。漏跨包回归（`--packages-above` 覆盖被依赖方，不覆盖依赖方）；仅作本地加速，
合入前仍跑全量。**符合** AGENTS.md"不另写判据"——只做选择，不改判据。

### B-6 缓存细化

**诊断**：ccache key 已含 `colcon_defaults.yaml` + `src/**/CMakeLists.txt`（合理），但**不含头文件**——
改 `.hpp` 不换 key，靠 restore-keys 前缀匹配仍命中旧条目（结果正确性由 ccache 内容寻址保证，
这里只是命中率/时效问题）。

**方案 diff**（两个 workflow 的 cache key）：

```diff
-          key: ccache-${{ runner.os }}-${{ env.ROS_DISTRO }}-${{ hashFiles('colcon_defaults.yaml', 'src/**/CMakeLists.txt') }}
+          key: ccache-${{ runner.os }}-${{ env.ROS_DISTRO }}-${{ hashFiles('colcon_defaults.yaml', 'src/**/CMakeLists.txt', 'src/**/*.hpp', 'src/**/*.h') }}
```

**预估**：冷启动构建命中率提升（头文件驱动的重编更早失效）；单次省 0-2 min，方差大。
**风险**：低。另评估过"缓存整个 `build/`"（体积过大）与"Docker layer 缓存"（无容器化，不适用）——均不采纳。

### B-7 环境/runner：维持现状

- CI 4 核配 `parallel-workers: 4`（colcon_defaults 对 CI 同样生效）已配平；升配大 runner
  （2× 计费分钟换 ~1.5× 构建）不划算。
- 本地 24C/19G 的 `parallel-workers: 4` 是防 swap 死锁的正确值（BUILD_OPTIMIZATION.md 有血泪数据），不动。
- 预构建镜像：CI 每次 apt+setup-ros ≈ 4-5 min，占端到端 30%；可评估自托管 runner 或容器镜像预热，
  但属基础设施决策，超出测试优化范围，记入待办。

---

## 五、阶段三：测试之外的优化建议

- **F-1 Flaky/环境抖动治理**（今天实测触发）：本地退化网络让 5 个 DDS 门禁全红而 CI 全绿。建议：
  ① `headless_smoke.sh` 区分"节点进程退出"（真失败）与"发现超时"（环境失败），后者自动重试一次再判红；
  ② 给每个 DDS 门禁的失败输出首行打印"疑环境性失败，请核对 CI 同 commit 结论"的提示；
  ③ #17 的 history.jsonl"连续绿"机制是良好范式，可推广到全部 DDS 门禁的观察期管理。
- **F-2 门禁时长监控与预算告警**：`drive_gates` 的 `last.json` 已含逐门禁秒数。建议加
  `scripts/gate_budget_check.py`：与 `history.jsonl` 最近 10 次中位数比，>1.5× 时在输出中 WARN
  （不阻断），Nightly summary 一并展示——今天 closed-loop-fault 从 320s 劣化到 1337s 这类漂移即可自动暴露。
- **F-3 本地快速反馈固化**：`QUICK=1`（30s）与 pre-commit hook 已存在但散落在 CLAUDE.md；
  建议在 README 增"改完先跑什么"一节（QUICK=1 → affected(B-5) → 全量），并明确"全量以 CI 为准"。
- **F-4 测试编写规范**（可并入 CONTRIBUTING/CLAUDE.md）：新测试优先进纯 std core 并在
  `tests/core_standalone/CMakeLists.txt` 登记（#40 约定）；RNG 必须固定种子（现状 100% 合规）；
  禁止墙钟断言；DDS 门禁只写容差判据；测试命名挂 issue 号（现状良好，保持）。
- **F-5 测试金字塔长期结构**：当前 506 用例几乎全是单元/组件级、6 个集成门禁均有时钟预算，结构健康。
  方向：继续把感知/规划数学核下沉 core（#59/#60/#61 已开局），把 12 个"仅 colcon 轨"文件逐个化解，
  最终让 sanitizer 覆盖全部算法逻辑。
- **F-6 覆盖率基线**：给 core_standalone 加 `--coverage` + lcov（纯 CMake，改一行 add_compile_options），
  先在 Nightly 产报告建基线，阈值经用户确认后再开硬门；**不得低于本次基准（未测量→首次测量即为基线）**。
- **F-7 setup-ros 步骤级超时**：run `38258621269` 在 setup-ros 挂死 45 min 靠 job 级兜底；
  给 `Prepare ROS 2` 步骤加 `timeout-minutes: 10`，失败即重试换 runner。

---

## 六、优化记录（每次实施后追加）

| 日期 | 改动 | 指标前 | 指标后 | commit |
| --- | --- | --- | --- | --- |
| 2026-10-12 | 建立本基准文件：本地全量实测 1 次（3215s，5 个 DDS 门禁环境性失败，证据见 2.3）+ CI 历史 12 次运行拆解 | — | 见第一/二节 | 未提交（待用户确认后随首批优化一并提交） |
| 2026-10-13 | 将 10 项待办建议建为 issue #63–#72（`ready-for-human`），本文件第七节加编号索引与实施顺序 | — | — | 见 #63–#72 |
| 2026-10-13 | 补立 4 项（审计黄项/阶段三遗漏）：#73 评分核双轨去重、#74 本地快速反馈文档、#75 测试编写规范、#76 colcon-only 下沉伞形 | — | — | 见 #73–#76；文档提交 `c0652e1` 之后 |

## 七、待办建议（未确认/未实施）

已全部建为 GitHub issue（tracker：`zhangjszs/ROS2-LTS-Final`，标签 `ready-for-human`，由维护人跟进）：

| # | 建议 | Issue | 预估收益 | 风险 | 阻塞点 |
| --- | --- | --- | --- | --- | --- |
| T-1 | B-1 CI paths-ignore + concurrency | [#63](https://github.com/zhangjszs/ROS2-LTS-Final/issues/63) | 45-70 runner-min/周 | 低 | 契约文档是否保留触发 |
| T-2 | B-2 check_cpp20.sh 统一构建参数 | [#64](https://github.com/zhangjszs/ROS2-LTS-Final/issues/64) | 本地全量 -14% | 低 | 首次切换有一次全编 |
| T-3 | B-3 closed-loop-fault 六臂并行（PARALLEL_ARMS 开关） | [#65](https://github.com/zhangjszs/ROS2-LTS-Final/issues/65) | 最大单项 -50% | 中 | Nightly 试跑转正 |
| T-4 | B-4 CI job 拆分 | [#66](https://github.com/zhangjszs/ROS2-LTS-Final/issues/66) | CI 14m→8m | 中高 | artifact 实测；与 #65 同轮 |
| T-5 | B-5 affected_gates.sh 本地增量 | [#67](https://github.com/zhangjszs/ROS2-LTS-Final/issues/67) | 单包改动 -60~75% | 低-中 | 确认不进 CI |
| T-6 | B-6 ccache key 加头文件 | [#68](https://github.com/zhangjszs/ROS2-LTS-Final/issues/68) | 0-2 min/次 | 低 | 无 |
| T-7 | F-1 环境性失败识别与重试 | [#69](https://github.com/zhangjszs/ROS2-LTS-Final/issues/69) | 减少假红 | 低 | 无 |
| T-8 | F-2 gate_budget_check.py 时长监控 | [#70](https://github.com/zhangjszs/ROS2-LTS-Final/issues/70) | 漂移可见 | 低 | 无 |
| T-9 | F-6 core_standalone 覆盖率基线 | [#71](https://github.com/zhangjszs/ROS2-LTS-Final/issues/71) | 覆盖率从"未测量"到有基线 | 低 | 阈值策略 |
| T-10 | F-7 setup-ros 步骤级超时 | [#72](https://github.com/zhangjszs/ROS2-LTS-Final/issues/72) | 少一次 45min 级浪费 | 低 | 验证 step 级 timeout 生效方式 |
| T-11 | R-1 评分核测试双轨去重（13 个同名用例） | [#73](https://github.com/zhangjszs/ROS2-LTS-Final/issues/73) | 消漂移风险；用例 506→~493 | 低 | 无 |
| T-12 | F-3 README/CLAUDE.md 本地快速反馈一节 | [#74](https://github.com/zhangjszs/ROS2-LTS-Final/issues/74) | 降低假红时乱改概率 | 低 | 措辞与 #67 对齐 |
| T-13 | F-4 测试编写规范入库（含 R-2 粒度先例） | [#75](https://github.com/zhangjszs/ROS2-LTS-Final/issues/75) | 审计结论固化为准入清单 | 低 | 无 |
| T-14 | F-5 伞形：12 个仅 colcon 轨文件下沉 core | [#76](https://github.com/zhangjszs/ROS2-LTS-Final/issues/76) | sanitizer 全覆盖算法逻辑 | 中 | 各子项按 #59 模式单独立项 |

实施顺序建议：#63/#64/#68/#72（低风险 YAML/脚本，可一批）→ #69/#70（可观测性）→
#65（Nightly 试跑）→ #66（依赖 #65 结论）→ #67 → #71；测试质量线 #73/#75/#76 与文档线 #74 可随时穿插
（均不依赖性能线结论）。每项落地后按第五节回写优化记录表。

---

*方法论备注：本地测量优先跑全量拿真实数据；本机当日 DDS 退化导致运行时门禁不可采信，
故 DDS 门禁基线取"同机器健康日（10-08）+ CI Nightly（10-11）"双源交叉验证，局限性已在 2.3 注明。*
