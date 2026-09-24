# 统一接口契约（issue #14）

定义模块间交换数据的**单一事实来源**：话题、发布/订阅责任、坐标系、单位、符号、更新率、
QoS、源时间、有效期、非法数据处理。目标是接入时**显式解释**语义，不再隐式假设。

- 机器可判定的原语集中在 `common_msgs/include/common_msgs/interface_contract.h`
  （纯 std，无 ROS 依赖）：话题/坐标系常量、有效性哨兵、几何/速度/来源年龄校验器、QoS 描述符。
- 该契约的**跨 ROS 版本语义差分与断言执行**由 #24 承担（本文件不只是文档，判定逻辑有单测覆盖：
  `pure_pursuit/test/test_interface_contract.cpp`）。

## 1. 话题契约

| 话题 | 消息 | 发布者 | 主要消费者 | frame | 单位/符号 | 更新率 | QoS(depth,rel,durable) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `/localization/vehicle_state` | `HuatCarstate` | vehicle_state / 仿真器 | MPC、PP、safety、benchmark | `map` | x,y[m] θ[rad,CCW+] v[m/s] | ~INS 率 | (10,true,false) `kQosState` |
| `/planning/pathlimits` | `HuatPathLimits` | 规划器 / velocity_profiler | MPC、PP | `map` 或 `base_link` | x,y[m]；速度[m/s] | 事件 | (1,true,false) `kQosPath` |
| `/vehicle_command` | `HuatVehicleCmd` | 控制器（经适配层编码） | 底盘 / 仿真器 | — | 见 §4（raw 字节） | 控制率 | (10,true,false) `kQosCommand` |
| `/system/stop` | `HuatStop` | safety_monitor | MPC、PP、各执行 | — | bool stop | 事件 | (1,true,**true**) `kQosStop`（锁存，见 #8） |

QoS 值即 `interface_contract.h` 中对应 `kQos*` 描述符；消费者据此构造 `rclcpp::QoS`。

## 2. 轨迹（HuatPathLimits）

- **几何**：`path[]`（`geometry_msgs/Point`，仅 x/y 有意义；z 不再是速度载体）。
- **参考速度**：`target_speeds[]`（`float64`）。
  - 与 `path` **等长**时视为显式速度，逐点须有限且 ≥0；**0 m/s 合法 = 停车目标**（#11）。
  - 缺失（长度不符）时消费者**不得**静默把缺失当 0：走 `kUnknownSpeed=NaN` 语义，
    用 `targetSpeedsEffective()` 判定；不满足即回退或拒绝（见迁移矩阵）。
- **速度语义已从 `Point.z` 彻底移除**：消费者不再读 `Point.z` 作速度（已删 MPC 的 z 回退，PP 本就不读）；发布侧统一用 `target_speeds[]`，缺失时走消费者配置默认。
- **标识/状态**：`replan` 表达重规划请求；轨迹几何版本经 `header` 与来源话题约定。

## 3. 坐标系（#3）

- 唯一约定：`map`（全局）、`base_link`（车体系，原点=车辆、航向 0）。其余 frame 一律视为未知。
- 消费者**必须**按 `header.frame_id` 显式分派：`isFrameSupported()` 为假 → **拒绝并进入明确降级**
  （MPC：`has_path_=false` → 看门狗急停），绝不把未知系当 map 用。
- 等价性：同一几何在 `map` 与 `base_link` 输入应得到等价控制（MPC 用局部路径契约，参考位姿
  对 base_link 取 (0,0,0)）。测试 `GlobalAndLocalFrameInputsYieldEquivalentControls` 覆盖非零位姿。

## 4. 控制指令（#2/#15）

- 控制器内部统一用**物理前轮转角 `steering_rad`[rad]**（右正）与**纵向加速度**。
- 到底盘 raw 的字节编码由**共用执行器适配层**负责，均在 `common_msgs` 纯 std 头中：
  - 转角：`SteeringCalibration`（`vehicle_command` 的 `steering` 字节），
    `raw = neutral + deg·units_per_degree`，clamp `[min_raw,max_raw]`；默认对齐仿真协议 90/1/±25°。
  - 纵向：`ActuatorCalibration`（`vehicle_command_codec.h`），加速度↔`pedal_ratio`/`brake_force` 百分比，
    油门与制动**互斥**；关键安全约束：**clamp-before-narrow + 非有限降级**（对齐 ROS1 #6“负油门→255”
    缺陷类），非有限/缺失一律降级为“无油门 + 安全制动”。
  - 指令帧常量与 16 位累加和校验：`kCmdHead1/2`/`kCmdLength`/`checksumRaw`，由 PP/MPC/仿真共用，
    杜绝协议拼装在校验和/帧头/字节窄化处散落。
- 标定版本经 `actuator.calibration_version` 参数透传（默认 `sim-default-0`）；真实底盘协议待实车标定后仅改参数/递增版本，
  禁止在模块内散落硬编码。往返一致性与方向/量化由 `test_steering_calibration`、非有限/限幅安全由
  `test_vehicle_command_codec` 断言。

## 5. 车辆状态时间与质量（#12）

- `header.stamp` = **发布时刻**；设备**测量时刻**经 `device_gps_week_number`/`device_sec_of_week`
  透传（<0 = 未知，`kUnknownDeviceTime`）。二者不得互相替代。
- `quality_validated` 标记质量是否经可信标定/映射；未标定链保持 **false**，消费者不得据此当作已验证。
- 区分**接收活性**（话题是否按时到达）与**源数据年龄**（`sourceAgeAcceptable()`：测量时刻与接收时刻之差 ≤ 容差，
  容差 <0 禁用）；持续到达但陈旧/未来戳的测量不得刷新看门狗。

## 6. 兼容路径与退出条件 · 迁移覆盖矩阵

| 模块 | 现状 | 迁移到契约 | 状态 |
| --- | --- | --- | --- |
| MPC 控制器 | frame 门禁 + 局部路径契约 + `target_speeds` 优先 | 采用 `contract::isFrameSupported`/`kFrameBaseLink`；速度逐点由 `contract::selectReferenceSpeed` 集中判定（`target_speeds` 优先，缺失→`path.reference_speed_default`）；stop 订阅 `makeQoS(kQosStop)`；**已移除 `Point.z` 速度回退** | ✅ 已采用 |
| Pure Pursuit | 速度来自参数 `algorithm.throttle.target_speed`（不读 path 速度） | stop 订阅 `makeQoS(kQosStop)` | ✅ 无 z 依赖；话题名可按参数覆盖，闭环冒烟在契约话题上验证接线 |
| velocity_profiler | 速度权威：输出 `target_speeds[]`（与 path 等长，不写 z 速度） | 话题名缺省引用 `contract::kTopicPathLimits`/`kTopicVehicleState`；输出经 `targetSpeedsEffective` 单测锁定为合法载体 | ✅ 已采用 |
| skidpad / straight_line 规划器 | `MakePoint(x, y, 0.0)`：z 恒为 0，仅为几何占位（不载速） | — | ✅ 已无 z 速度语义 |
| vehicle_state / 仿真器 | 透传质量/设备时间（#12） | 话题名缺省引用 `contract::kTopicVehicleState`；仿真器 state 发布/指令·stop 订阅均由 `makeQoS(kQos*)` 构造（stop 锁存） | ✅ 已采用 |
| safety_monitor | `transient_local` 锁存 stop（#8） | stop 发布 `makeQoS(kQosStop)` | ✅ 已采用 |

**stop 链路已端到端统一到契约**：发布端（safety_monitor）与订阅端（MPC/PP）的锁存 QoS 均由
`common_msgs::contract::makeQoS(kQosStop)` 单一来源构造（`interface_contract_qos.hpp`），杜绝三处散落构造漂移；行为与原 `KeepLast(1).reliable().transient_local()` 完全一致。

**兼容退出条件**：`Point.z` 作速度的回退分支已删除，且其选择逻辑集中到 `contract::selectReferenceSpeed`
并由 `test_interface_contract` 锁定；vehicle_state / velocity_profiler / 仿真器 的契约话题名缺省值与 QoS
均已改引 `contract::kTopic*` / `makeQoS(kQos*)`。运行时端点/QoS 兼容检查 `scripts/qos_contract_check.sh`
已升级为**闭环集成冒烟**：无界面拉起 sim→velocity_profiler→pure_pursuit→sim 四节点回路，用 `ros2 topic info -v`
断言每条契约话题 pub/sub 端点齐备且 QoS 合规（stop=TRANSIENT_LOCAL、state/path/command=RELIABLE），并纳入 CI。
矩阵全部 ✅ 即具备关闭 #14 的条件。

## 7. 任务与安全状态、最终指令仲裁（#16）

两个新的纯 std 共用层（`common_msgs/include/common_msgs/`，无 ROS context、可单测），与 §2/§4/§5 判定原语衔接、不重复实现：

- **`task_state_machine.h` · `TaskSafetyStateMachine`**：统一“任务态（IDLE/ARMED/RUNNING/FINISHED/FAULT）× 安全态
  （NORMAL/DEGRADED/STOP）”的显式迁移。**职责边界**：`safety_monitor` 的 `StopStateMachine` 仍负责“看门狗触发时机”（#7/#8），
  本机只作“系统级编排”。与 #8 锁存一致：`TIMEOUT` 停可被 `onResume()` 清除；`REQUEST`/`FAULT` 停锁存(sticky)，
  仅 `onReset()` 清除；`FAULT`/`FINISHED` 触发锁存停；停态下 `onArm()/onStart()` 一律拒绝。`canDrive()` 仅在
  非停且 `RUNNING` 时为真（ARMED=已就绪未起步，仍按安全制动）。
- **`command_arbitration.h` · `CommandArbitrator`**：多控制源（PP/MPC）→ 唯一最终 `/vehicle_command` 的**单帧纯函数**仲裁。
  优先级（高→低）：**安全/故障（stopActive）> 任务态约束（非 canDrive）> 正常控制源选择**。可信候选需同时满足
  `present` + `verifyChecksum`（#15）+ `sourceAgeAcceptable`（#12）；冲突按 `preferred` 选，无任一可信源则安全降级。
  安全降级输出确定性：`ActuatorCalibration`（#15）的 `emergencyBrakeRaw()` + 零油门 + 零转角，绝不越界/负值→255。
- **`command_arbitration.h` · `CommandArbiterFilter`**（跨帧有状态）：在单帧仲裁器外加一层去抖/切换驻留（`switch_dwell_sec`）
  与 stale 老化，时钟由调用方（节点用 ROS clock / 单测用虚拟时钟）逐帧传入。**语义**：已 committed 且当前可信→保持（sticky）；
  冷启动（无 committed）→立即接管任一可信源；从已失效的 committed 源切换→需候选连续可信 >=dwell（防抓不住抖动新源）。

**接线节点 `safety_monitor/command_arbiter_node`**（#16 独立出口）：订阅两路控制源（`topics.source_a` 默认
`/control/vehicle_command`、`topics.source_b` 默认 `/mpc/vehicle_command`）+ 锁存 `stop`（`makeQoS(kQosStop)`），
经 `CommandArbiterFilter` 在 `topics.output`（默认 `/vehicle_command`）产出**单一**最终指令；启动即发安全制动，
源话题/输出/超时/去抖/急停字节均参数化，无散落硬编码。

**验收门禁（软件侧）**：`test_task_state_machine`（8）+ `test_command_arbitration`（单帧矩阵 10：源超时/数据无效/
checksum 失败/stop 锁存/源冲突/无任一可信源降级）+ `test_command_arbiter_filter`（跨帧 6：去抖/sticky/stale 回退/
stop 复位后重接管 等）纳入 `colcon test` → CI。`headless_smoke.sh` 断言 `command_arbiter_node` 无界面启动；
`qos_contract_check.sh` 新增仲裁器接线断言（两路源 sub + 单一输出 pub），与 #14 闭环共存且不扰动其 `/vehicle_command` 不变量。

**仍卡在硬件/台架/集成（不据此关闭 #16）**：
- 实车故障注入端到端验收（真实 VCU 断连/掉电/急停按钮优先级）：依赖实车，未标定项保持“未标定”。
- 把仲裁器接入 `fsac.launch` 作为权威单出口（控制器改发源话题）：需 e2e 时序验证；本次仅新增节点+单测+冒烟，未改现有 launch（避免扰动 #14/#17）。
- 具体底盘通道/硬件急停优先级属仓库外，按 #15/#23 记录接口与证据归属。
