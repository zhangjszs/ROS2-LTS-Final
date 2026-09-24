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

- 控制器内部统一用**物理前轮转角 `steering_rad`[rad]**（右正）与**纵向加速度/目标速度**。
- 到底盘 raw 的字节编码由**共用执行器适配层**（`SteeringCalibration`，#15）负责：
  `raw = neutral + deg·units_per_degree`，clamp `[min_raw,max_raw]`；默认对齐仿真协议 90/1/±25°。
- 真实底盘协议待实车标定后仅改参数，禁止在模块内散落硬编码。往返一致性与方向/量化由
  `test_steering_calibration` 断言。

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
