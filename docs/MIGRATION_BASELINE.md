# ROS1(LTS) → ROS2 功能与接口迁移覆盖清单（源码对照基线）

## 〇、对照基线与结论边界

- **ROS 1 对照基线**：`1e7de39cef2675bedd61831e7bf17fc0d83322d5`
  - 本机仓库路径：`FSAC/2025/`（来源仓库：`zhangjszs/ROS1-LTS-Final`，完整历史非浅克隆，175 提交）
- **ROS 2 对照基线**：`c6d42ab1c335a4d32ffdd6b5db74c5abecd25060`
  - 本机仓库路径：`FSAC/2026/`（来源仓库：`zhangjszs/ROS2-LTS-Final`，main 分支对应提交）
- **结论边界声明**：
  - 本清单**仅比较固定提交**在源码树、接口定义与算法实现层面的事实对照。
  - **未证明全部历史迁移来源**：ROS 2 部分算法为重构或重新实现，非直接 git 迁移代码。
  - **未完成整栈/实车验收**：ROS 1 存在相关代码不代表已完成整车闭环验收；ROS 2 源码存在不代表已完成实车/硬件验收。
  - 本文档不包含产品决策（如端到端首选赛制）、不包含架构决策（如定位后端强迁与否）、不包含仓库外硬件驱动事实确认（如 VCU 外部驱动）。

---

## 一、状态定义说明

每条能力项的实施状态严格限定在以下五个值之一，禁止使用「待办」「TBD」「优化」等模糊表述：

| 状态值 | 判定标准 | 处置与闭环规则 |
|---|---|---|
| `保留` | ROS 1 既有能力在 ROS 2 中具有明确对应实现，核心功能与安全语义延续或强化 | 源码级可定位，纳入 CI/单测回归保护 |
| `替换` | ROS 2 采用了不同技术架构、数据结构或算法方案达成等效/改进功能 | 记录差异点与等效性，明确不退回旧实现 |
| `暂缓` | 经过评估无需直接对齐，或改造成本高、依赖重，暂不纳入当前主干迭代 | 记录暂缓理由与边界条件，不强迁复杂实现 |
| `未找到` | 在 ROS 2 对照基线源码树中未检索到对应模块或包的完整实现 | 记录缺口事实，拆分或关联跟踪 issue |
| `证据不足` | 缺少必要的外部输入（运行时/bag/实车标定/外部驱动仓库），当前无法断言其等效性 | 明确注明缺失输入类型，禁止提前标记为已完成 |

---

## 二、逐领域迁移对照矩阵

### 1. 轨迹消息（Trajectory Messages & Profiling）

| 能力项 | ROS 1 入口 (`路径:行`) | ROS 2 入口 (`路径:行`) | 消息与参数差异 | 坐标/时间语义 | 外部依赖 | 验证证据 | 状态 | 对应 Issue |
|---|---|---|---|---|---|---|---|---|
| 轨迹消息与动态字段 | `src/autodrive_msgs/msg/HUAT_PathLimits.msg:8` | `src/common/common_msgs/msg/HuatPathLimits.msg:3` | ROS 1 具备独立的 `target_speeds[]`、`curvatures[]` 动态数组与 `stamp` 墙钟时间戳；ROS 2 移除独立速度/曲率数组，速度通过 `Point.z` 编码，无独立曲率数组 | 坐标均为赛道参考系；ROS 1 含独立 `stamp` 与 `header.stamp`；ROS 2 仅保留 `header.stamp` | 无（ROS 基础消息） | 源码定义对照；ROS 2 `velocity_profiler_node.cpp:112` 编码验证 | `替换` | #14, #24 |
| 速度剖面与曲率规划 | `src/planning_core/include/planning_core/speed_profile.hpp:65` | `src/planning/velocity_profiler/src/velocity_profiler.cpp:30` | ROS 1 依赖 `planning_core::ComputeSpeedProfile` 计算并输出速度与曲率；ROS 2 由 `VelocityProfiler::ComputeProfile` 执行前向/后向加速度与向心力约束梯形规划 | 均为沿路径几何弧长的速度标量计算（单位：m/s） | C++ 标准库（纯数学/无外部第三方库） | 差分用例：`src/planning/velocity_profiler/test/test_velocity_profiler.cpp` | `保留` | #14, #24 |
| 路径维度与契约校验 | `src/planning_ros/include/planning_ros/contract_utils.hpp:1` | `src/planning/velocity_profiler/src/velocity_profiler_node.cpp:112` | ROS 1 强制校验 `path`、`target_speeds`、`curvatures` 三者长度严格 1:1 一致；ROS 2 采用单数组 `path[i].z`，消除多数组长度不匹配的越界隐患 | 保持一致 | 无 | `test_velocity_profiler.cpp` 单点/退化路径守卫断言 | `替换` | #14, #24 |

### 2. 定位与状态估计（Localization & State Estimation）

| 能力项 | ROS 1 入口 (`路径:行`) | ROS 2 入口 (`路径:行`) | 消息与参数差异 | 坐标/时间语义 | 外部依赖 | 验证证据 | 状态 | 对应 Issue |
|---|---|---|---|---|---|---|---|---|
| 惯导数据坐标转换 | `src/vehicle_interface_ros/scripts/ins_bridge.py:1` | `src/localization/vehicle_state/src/vehicle_state_estimator.cpp:51` | ROS 1 通过 Python 脚本桥接 INS；ROS 2 实现 C++20 `GeodeticToEnu` 高性能大地坐标向局部 ENU 坐标转换并发布 `HuatCarstate` | WGS84 经纬高 → 局部站心切平面 ENU 笛卡尔坐标；时间戳透传自 INS 硬件戳 | 硬件惯导驱动/消息 | `src/localization/vehicle_state/test` 或节点离线转换测试 | `替换` | #23 |
| 锥桶地图构建与匹配 | `src/localization_core/src/location_mapper.cpp:42` | 未找到 | ROS 1 具备 `LocationMapper` 进行锥桶全局地图与局部感知的相关性匹配与回环；ROS 2 源码中无此实现 | 局部锥桶系到全局 map 系对齐 | 激光/视觉锥桶检测话题 | ROS 2 仓库代码检索（无对应文件） | `未找到` | #23 |
| ESKF 状态估计滤波 | `src/localization_core/src/imu_state_estimator.cpp:10` | 未找到 | ROS 1 包含基于误差状态卡尔曼滤波（ESKF）的 IMU 与里程计融合估计器；ROS 2 仅有原始 INS 坐标转换，未包含 ESKF 融合层 | 状态向量包含位置、速度、姿态、IMU 零偏 | IMU 原始高频数据与轮速反馈 | ROS 2 仓库代码检索（仅有简单 vehicle_state_estimator） | `证据不足`（缺真实 bag 与传感器数据输入） | #18, #23 |
| 因子图优化后端 | `src/localization_core/src/factor_graph_optimizer.cpp:48` | 未找到 | ROS 1 引入 GTSAM 因子图优化后端，综合位姿图、回环约束与圆约束；ROS 2 未引入 GTSAM 依赖与因子图代码 | 全局非线性最小二乘位姿优化 | GTSAM 库 | ROS 2 仓库代码检索；CMakeLists 无 GTSAM 依赖 | `暂缓`（依路线图不强迁高复杂度因子图） | #23 |

### 3. 底盘与执行器接口（Chassis & Actuator Interface）

| 能力项 | ROS 1 入口 (`路径:行`) | ROS 2 入口 (`路径:行`) | 消息与参数差异 | 坐标/时间语义 | 外部依赖 | 验证证据 | 状态 | 对应 Issue |
|---|---|---|---|---|---|---|---|---|
| 底盘网络通信层 (UDP) | `src/vehicle_interface_core/src/udp_socket.cpp:1` | 未找到 | ROS 1 通过底层 POSIX UDP Socket 直接收发 VCU 二进制数据包；ROS 2 仓库内无网络通讯包，控制输出终止于 ROS 话题层 | 控制周期通常为 20ms~50ms，含时间戳与序列号 | 车载局域网 / VCU 控制器硬件 | ROS 2 仓库代码检索（无 `vehicle_interface_*` 包） | `证据不足`（缺外部驱动仓库事实与 VCU 协议资料） | #15, #23 |
| 底盘协议节点包装 | `src/vehicle_interface_ros/src/Node.cpp:1` | 未找到 | ROS 1 订阅 `HUAT_VehicleCmd` 并编码打包发送至 UDP，同时解析 UDP 状态包发布 `HUAT_VehicleStatus`；ROS 2 无此硬件桥接节点 | 硬件反馈与软件指令桥接 | VCU 协议规范与硬件连线 | ROS 2 检索确认无对应节点 | `证据不足`（缺 VCU 协议资料与实车台架） | #15, #23 |
| 控制指令帧编码与守卫 | `src/autodrive_msgs/msg/HUAT_VehicleCmd.msg:1` | `src/control/pure_pursuit/src/control/vehicle_command_encoder.cpp:1` | ROS 1 与 ROS 2 均维护包含 `head1`, `head2`, `speed`, `steering`, `brake`, `checksum` 的紧凑指令帧；ROS 2 剥离为独立的 `VehicleCommandEncoder` 纯函数并实现转向标定守卫 | 纯字节级二进制帧协议；转角弧度与原始量化值映射 | 无 | `src/control/pure_pursuit/test/test_pure_pursuit.cpp` 及指令编解码断言 | `保留` | #15, #24 |
### 4. 控制与安全保护（Control Protection & Safety Arbitration）

| 能力项 | ROS 1 入口 (`路径:行`) | ROS 2 入口 (`路径:行`) | 消息与参数差异 | 坐标/时间语义 | 外部依赖 | 验证证据 | 状态 | 对应 Issue |
|---|---|---|---|---|---|---|---|---|
| 输入超时与保鲜看门狗 | `src/control_ros/src/control_node.cpp:151` | `src/control/pure_pursuit/src/control/input_guard.cpp:6` | ROS 1 内嵌在 control_node 中，空路径仍刷新看门狗时间戳致缺陷；ROS 2 提取为纯函数 `InputGuard`，空路径与超时明确触发软制动，位姿超时触发硬制动 | 接收时间与消息时间戳双重校验 | rclcpp 始终或仿真时钟 | 差分回归测试：`EmptyAndDegeneratePath`、`ArrivalIsNotFreshnessAndFutureStampsAreRejected` | `保留` | #24 |
| 安全状态机与紧急停车 | `src/control_core/src/controller_base.cpp:121` | `src/safety/safety_monitor/src/state_machine/stop_state_machine.cpp:1` | ROS 1 由 controller_base 内部状态枚举驱动制动力；ROS 2 建立独立的 `SafetyMonitor` 节点与 `StopStateMachine` 状态机，区分 IDLE、RUNNING、FINISH、EMERGENCY_STOP | 状态迁移单向锁存机制（除显式 Reset） | `/stop` 与心跳话题 | `src/safety/safety_monitor/test/test_stop_state_machine.cpp` | `保留` | #16, #24 |
| 纵向制动约束与超速保护 | `src/control_core/src/controller_base.cpp:121` | `src/planning/velocity_profiler/src/velocity_profiler.cpp:30` | ROS 1 由控制节点直接计算比例制动力；ROS 2 采用规划层向心加速度/附着力圆限速与控制层油门/制动互斥双重保护 | 物理加速度与速度约束（m/s, m/s^2） | 车辆动力学与摩擦力参数 | `CornerLimitOverridesMinimumCruisingSpeed` 测试 | `保留` | #19, #24 |

### 5. 仿真与评测工具（Simulation & Benchmark Suite）

| 能力项 | ROS 1 入口 (`路径:行`) | ROS 2 入口 (`路径:行`) | 消息与参数差异 | 坐标/时间语义 | 外部依赖 | 验证证据 | 状态 | 对应 Issue |
|---|---|---|---|---|---|---|---|---|
| 车辆动力学仿真器 | `src/simulation_core/src/vehicle_dynamics.cpp:8` | `src/simulation/vehicle_simulator/src/bicycle_model.cpp:40` | ROS 1 采用较复杂的非线性魔术公式轮胎动力学模型；ROS 2 在基线阶段采用离散动力学/运动学双轨自行车模型，提高数值稳定度与仿真速度 | 车辆坐标系到全局坐标系的 RK4 / Euler 积分推进 | 无（纯标量 C++ 算法） | `src/simulation/vehicle_simulator/test/test_bicycle_model.cpp` | `替换` | #17 |
| 非线性魔术公式轮胎模型 | `src/simulation_core/src/tire_model.cpp:1` | 未找到 | ROS 1 实现了完整的 Pacejka 魔术公式（P96）；ROS 2 自行车模型当前采用线性化侧向力增益模型 | 侧偏角（slip angle）到侧向附着力 | 真实轮胎台架试验参数 | ROS 2 仓库代码检索确认无 Pacejka 独立模型 | `证据不足`（缺轮胎极限实车标定数据） | #17 |
| 赛道生成与基线 KPI 评估 | `src/simulation_core/src/track_loader.cpp:10` | `src/simulation/track_benchmark/src/kpi_evaluator.cpp:25` | ROS 1 仅通过 YAML 读取预制赛道并缺乏量化 KPI 闭环；ROS 2 建立完整的 `track_benchmark` 套件，包含 CSV/程序化赛道生成与循迹耗时/曲率/加速度离群 KPI 评测 | 赛道标准几何坐标 | 无 | `src/simulation/track_benchmark/test/test_kpi_evaluator.cpp` | `替换` | #17 |
| 仿真主节点与闭环冒烟 | `src/simulation_ros/src/simulation_node.cpp:32` | `src/simulation/vehicle_simulator/src/simulator_node.cpp:15` | ROS 1 仿真依赖 ROS 1 专用话题与服务；ROS 2 提供标准 ROS 2 节点包装，并原生适配全自动化 CI 门禁 | 支持 `/clock` 仿真时钟步进 | ROS 2 中间件 | `scripts/check_cpp20.sh` 与仿真回归测试用例 | `替换` | #17 |

---

## 三、缺口与未解决依赖分析（汇总）

根据上述对照，以下领域存在确定性缺口，按输入缺失类型与对应 Issue 分类汇总：

1. **底盘接入缺口（VCU 外部驱动与协议）**：
   - **缺失输入**：外部驱动仓库事实、VCU 通讯协议规范、实车台架标定数据。
   - **影响**：ROS 2 无法在无物理驱动或实车适配层的情况下直接输出到线控底盘网络。
   - **归属跟踪**：关联父单 **#23**（待产品/外部事实决策）与执行单 **#15**（共用执行器适配层与车辆标定配置）。
2. **定位后端演进缺口（SLAM 建图与 ESKF 滤波）**：
   - **缺失输入**：端到端赛制决策（直线加速/八字绕环无需复杂 SLAM，高速循迹需锥桶跟踪/建图）、真实车载传感器 bag 数据。
   - **影响**：当前 `vehicle_state_estimator` 仅能依赖高精度 INS 站心坐标转换，在 GPS/INS 信号丢失或无惯导工况下缺乏自主定位冗余。
   - **归属跟踪**：关联父单 **#23**（定位后端架构决策）与 **#18**（rosbag 数据集与感知质量评估）。
3. **仿真轮胎动力学校准缺口**：
   - **缺失输入**：实车极限速度与轮胎侧偏刚度实测标定数据。
   - **影响**：高附着极限工况下的动力学仿真精度受限。
   - **归属跟踪**：关联 **#17**（可复现仿真场景、可信 KPI 与自动回归）。

---

## 四、建议后续子任务（不自行关闭父单）

为推进 ROS 2 体系逐步收敛，建议由 Planner 在合适时机拆分或推进以下行动项：

1. **子任务建议 A（底盘驱动对接）**：
   - 一旦获得 VCU 协议文档或确定外部驱动仓库，建立 `vehicle_interface_ros2` 适配包或轻量 CAN/UDP 驱动节点，承接 `common_msgs/msg/HuatVehicleCmd` 并实现心跳反馈。
2. **子任务建议 B（合成定位夹具先行）**：
   - 在缺乏 Noetic 运行时与真实 bag 的约束下，参照 #24 模式为 `vehicle_state` 编写基于已知轨迹和合成惯导信号的无 ROS 核心单元测试（纯 std 坐标转换断言与边界守卫）。
3. **子任务建议 C（赛制适配分层）**：
   - 待 #23 用户决策确定首个端到端赛制后，若首选为直线加速或八字绕环，可直接以现行 `vehicle_state`（纯 INS）作为定位基线推进闭环，将全功能建图定位后端解耦并排期。
