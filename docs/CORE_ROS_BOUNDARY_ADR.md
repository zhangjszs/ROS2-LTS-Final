# ADR：Core/ROS 适配边界（#22）

- 状态：已采纳
- 日期：2026-09-25
- 相关：#14（接口契约）、#15（执行器编解码）、#16（任务与安全状态/仲裁）、#17（离线评测）、#22（本决策）

## 背景

ROS 1 基线的算法逻辑与中间件强耦合（catkin 构建、节点内联算法、编码散落）。
本仓库将"可独立编译、可回放、可单测"的算法核（Core）与 ROS 适配层分离，
使 Core 可在无 ROS、无 DDS 的环境下构建与验证。

## 决策

### 1. 依赖方向与模块责任

- 单向依赖：ROS 适配层 → Core。Core **不 include 任何 ROS 头**（`rclcpp`、消息生成头、`rclcpp::Time` 等）。
- Core 仅依赖 C++20 标准库，以 header-only 提供；节点侧的参数声明、QoS、TF/时钟、日志、消息构造、发布与节点状态置位等副作用全部留在适配层。
- Core API 接受显式输入（状态、轨迹、参数、dt），返回显式输出或状态枚举；失败原因用枚举表达，由适配层分流日志/降级动作。
- 类型解耦用 C++20 `concept` 鸭子类型（如 `BaseLinkConeLike` / `LineParamsLike` / `ThrottleParamsLike`），不搬运 ROS 消息类型、不复制字段定义。

### 2. Core 存放位置

- 包内 Core：`<pkg>/include/...`，如 `mpc_controller/path_reference_builder.h`、`pure_pursuit/throttle_controller.h`、`straight_line_planner/cone_boundary.h`。
- 跨包共用纯层：`common_msgs/include/common_msgs/*.h`（手写头，下游不带前缀 include）。
- 命名：`namespace contract`（契约层）与 `namespace <pkg>_core`（包内 Core）。

### 3. 构建与验证双轨

- ROS 轨：colcon/ament 构建，Core 单测以 `ament_add_gtest` 注册进 `colcon test` 与 CI 门禁。
- 独立轨：`tests/core_standalone/` 为标准 CMake/CTest 工程，复用同一批测试源文件（单一事实来源）；`scripts/core_standalone_check.sh` 在无 ROS 环境下构建并运行，默认开启 ASan+UBSan。两轨均进 CI。

### 4. 单一源码来源与版本消费

- Core 头以本仓库为唯一权威源演进；**不通过 ROS1/ROS2 两仓库手工复制共用**。
- 跨仓库 ROS1 侧消费方式（差分回放/对照）由 #23、#24 跟踪落实；本仓库不因此反向引入中间件依赖。

### 5. 组合方式

- 横向（PP 几何 / MPC）、纵向（ThrottleController）、任务与安全状态（TaskSafetyStateMachine）、执行器编码（vehicle_command_codec）相互独立，由适配层组合；最终指令由 #16 仲裁节点唯一出口产生。

## 已解耦模块（证据 = 可脱离 ROS 构建并运行通过的单测）

| Core 头 | 职责 | 适配层消费方 | 独立测试（standalone 轨） |
| --- | --- | --- | --- |
| `common_msgs/interface_contract.h` | #14 话题/坐标系/有效性契约与校验器 | MPC 节点、vehicle_state、velocity_profiler、simulator（QoS 助手 `interface_contract_qos.hpp`） | `test_interface_contract` |
| `common_msgs/vehicle_command_codec.h` | #15 底盘协议帧/校验和/执行器标定 | PP `vehicle_command_encoder`、MPC 节点、simulator | `test_vehicle_command_codec` |
| `common_msgs/task_state_machine.h` | #16 任务×安全状态机 | 仲裁层（`command_arbitration.h`） | `test_task_state_machine` |
| `common_msgs/command_arbitration.h` | #16 单帧仲裁 + 跨帧去抖滤波 | safety_monitor `command_arbiter_node` | `test_command_arbitration`、`test_command_arbiter_filter` |
| `pure_pursuit/throttle_controller.h` | #22-1 纵向控制律（P+I/抗饱和/blend） | PP 控制器 | `test_throttle_controller` |
| `mpc_controller/path_reference_builder.h`（+ `mpc_types.hpp`） | #22-2 path→ReferencePoint（航向/曲率/速度选择） | MPC 节点 | `test_path_reference_builder` |
| `straight_line_planner/cone_boundary.h` | #22-3 锥桶左右分离与边界宽度判定 | line_detector、straight_line_planner 节点 | `test_cone_boundary` |
| `track_benchmark/kpi_evaluator.hpp`（+ `track_generator.hpp`） | #17 KPI 判定/赛道生成 | 离线 benchmark runner | `test_kpi_evaluator` |

## 仍依赖中间件的模块

- 各 `*_node.cpp` 适配层（参数/QoS/时钟/发布/日志），属设计内边界，不视为残留耦合。
- `vehicle_simulator/sensor_simulator.hpp` 直接引用消息类型（传感器注入），尚未下沉。
- 后续候选（各自独立、行为可证等价后再动）：velocity_profiler 梯形规划、safety_monitor 看门狗计时、lidar_cluster 几何核、skidpad_planner `IcpApfPlanner::ClusterCones`。

## 后果与边界

- Core 可独立编译与 ASan 检查，回归成本低；适配层保持薄，行为等价以"逐条件等价 + 位级回归（#17）"双重验证。
- 例外：实车/ROS1 侧验证不在本仓库范围（#15 台架标定、#23/#24 迁移对照）。
