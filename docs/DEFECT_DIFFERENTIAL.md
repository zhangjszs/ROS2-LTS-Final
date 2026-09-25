# ROS1 → ROS2 语义差分台账（issue #24）

把本轮审查发现的缺陷变成**可长期运行的验收用例**：每条差异给出 ROS1 行为、ROS2 期望行为、
差异分类与可点击依据。正确性不以"ROS1 原输出"为标准，而由明确契约 + 解析基本工况 + 故障期望共同判断。

- 合成夹具（无 ROS context、无实车、CI 门禁）：
  - [`src/control/pure_pursuit/test/test_defect_differential.cpp`](../src/control/pure_pursuit/test/test_defect_differential.cpp)
  - [`src/planning/velocity_profiler/test/test_velocity_profiler.cpp`](../src/planning/velocity_profiler/test/test_velocity_profiler.cpp)
  - 两处同时注册进 `tests/core_standalone/`（不 source ROS、ASan+UBSan）与 `colcon test`。
- 对照基线：ROS1 `1e7de39c`，ROS2 `c6d42ab1`（结论来自当前主分支代码对照，未证明全部历史迁移来源）。
- ROS1 issue 链接前缀：`https://github.com/zhangjszs/ROS1-LTS-Final/issues/`

## 分类定义

| 分类 | 含义 | 处置 |
| --- | --- | --- |
| 缺陷修复差异 | ROS1 行为错误，ROS2 已修正 | 用夹具锁定，禁止退回 |
| 预期算法差异 | 两栈实现不同但都合理 | 记录依据，不判对错 |
| 接口回归 | ROS2 丢失了 ROS1 具备的能力 | 必须补回或显式标注缺口 |
| 证据不足 | 缺运行环境/数据，无法判定 | 保持开放项，不得写成"已验证" |

## 必测矩阵逐行结论

| # | 场景 | ROS1 缺陷 | ROS2 期望行为（可判定） | 夹具用例 | 分类 |
| --- | --- | --- | --- | --- | --- |
| 1 | 合法零速、超速减速 | [#6](https://github.com/zhangjszs/ROS1-LTS-Final/issues/6) 负油门→255；[#7](https://github.com/zhangjszs/ROS1-LTS-Final/issues/7) 零速被替换为巡航 | 0 m/s 是合法显式停车目标；非有限/超范围在窄化前 clamp 并降级为"无油门+锁定制动"；油门制动互斥 | `ZeroSpeedIsACommandNotAMissingValue`、`NegativeThrottleCannotBecomeFullPedal255` | 缺陷修复差异 |
| 2 | 0/1/2 点与畸形轨迹 | [#8](https://github.com/zhangjszs/ROS1-LTS-Final/issues/8) 单点越界；[#9](https://github.com/zhangjszs/ROS1-LTS-Final/issues/9) 空路径刷新 watchdog 却静默 | 几何 ≥2 点且全有限才可用；不可用时速度权威给出**明确不可行（0）**、仲裁给出确定性安全制动而非"沉默" | `MalformedGeometryIsRejectedNotFollowed`、`DegenerateGeometryMustNotYieldDrivableSpeed` | 缺陷修复差异 |
| 3 | 位姿断流、旧戳重复、时间跳变 | [#11](https://github.com/zhangjszs/ROS1-LTS-Final/issues/11) 位姿过期但路径持续 | 接收活性 ≠ 观测有效性：超龄/未来戳拒收，且不刷新行驶许可；禁用态由 `tolerance<0` 显式表达 | `ArrivalIsNotFreshnessAndFutureStampsAreRejected` | 缺陷修复差异 |
| 4 | 终点、停机、重启/晚加入 | [#10](https://github.com/zhangjszs/ROS1-LTS-Final/issues/10) 完赛发全零命令后退出 | 安全降级必须带制动字节（全零≠安全）；完赛/abort 锁存，活性恢复不得解除，仅 `reset` 且回 IDLE；stop 话题 transient_local 使晚加入可复原 | `FinishAndAbortLatchAndRestartGrantsNoPermission` | 缺陷修复差异 |
| 5 | 非零位姿下 map/base_link 等价输入 | —（本仓 #3） | 物理转角/加速度与坐标原点解耦：同物理量→同出口字节；参照点平移不改变命令 | `PhysicalCommandIsIndependentOfPoseOrigin`（MPC 完整等价性见 `mpc_tracking`、`test_path_reference_builder`） | 预期算法差异（本仓缺陷已修复） |
| 6 | 中位及正负转向、制动满量程 | [#12](https://github.com/zhangjszs/ROS1-LTS-Final/issues/12) 零位与缩放分歧 | 零位/比例/限幅由单一标定表达；正负对称、极限 clamp 不越界；**混用两套标定必然得到错误零位** | `SteeringNeutralAndScaleAreSharedNotDuplicated` | 缺陷修复差异 |
| 7 | 曲率限速与最低速度冲突 | [#13](https://github.com/zhangjszs/ROS1-LTS-Final/issues/13) 最低速度覆盖横向约束 | 向心加速度约束优先，允许剖面出现低于 `min_velocity` 的目标速度；退化几何不给可行驶速度 | `CornerLimitOverridesMinimumCruisingSpeed`、`DegenerateGeometryMustNotYieldDrivableSpeed` | 缺陷修复差异 |

## 本会话差分夹具新发现的缺陷（已修）

| 项 | 内容 |
| --- | --- |
| 缺陷 | `VelocityProfiler::ComputeProfile()` 在输入点数 < 2 时把 `target_speed` 赋为 `limits_.min_velocity`（默认 2.0 m/s）；节点侧只挡 `path.empty()`，故**单点/含 NaN 路径会拿到一个"可行驶"速度剖面**并以契约话题下发 |
| 归属 | 矩阵第 2 行 + 第 7 行；即 ROS1 [#9](https://github.com/zhangjszs/ROS1-LTS-Final/issues/9)、[#13](https://github.com/zhangjszs/ROS1-LTS-Final/issues/13) 的同一缺陷类在 ROS2 里的新形态（此前未被 #1–#12 任一 issue 覆盖） |
| 修复 | 退化分支改为 `target_speed = 0.0`（#11 语义下的合法"停车目标"，即明确不可行）；节点侧改用 `contract::geometryValid()` 守卫，几何非法时**原样透传不臆造速度**并告警 |
| 回归 | `DegenerateGeometryMustNotYieldDrivableSpeed`（含 0 点、1 点、含 NaN 三种输入）+ 既有 `EmptyAndDegeneratePath` |

## 有意保持"证据不足"的部分（不得提前收口）

| 项 | 缺什么 | 归属 |
| --- | --- | --- |
| ROS1/ROS2 适配层同输入对跑 | 需要可用的 ROS1 环境与同一份输入回放通道 | 本 issue（夹具已按中立格式写断言，不依赖双版本同时在线） |
| 真实 bag 的时钟域/丢帧/类型映射 | 真实车载数据与设备时间协议未确认 | #18、#23 |
| 底盘协议逐字段与 VCU 文档对照 | 硬件资料/台架时间 | #15、#23 |
| 硬件急停优先级与 VCU 断连/掉电行为 | 实车 | #16 |

## 与其它 issue 的衔接

- 故障注入矩阵（软件可观测子集）已固化为 `scripts/fault_injection_smoke.sh`（#16），其判据来自 `/system/state`；本台账的"接口回归"类差异由该脚本在集成层复验。
- 差异口径变化必须同步 `benchmarks/baseline/`（#17）：`scripts/benchmark_regression.sh` 第 6 步会在口径静默漂移时失败。
- 本台账是 #25 第 3 篇（"从运行成功到可重复验证：故障注入和回归"）与 #26 证据页的直接素材；引用时须同时给出上表用例名与提交号，**未运行的验证不得写成已完成**。
