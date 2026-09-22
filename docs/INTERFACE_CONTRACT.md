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
- **速度语义不再藏在 `Point.z`**：新链路一律用 `target_speeds`；`Point.z` 的读取仅作**兼容旧发布者的过渡路径**（见 §6）。
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
| MPC 控制器 | frame 门禁 + 局部路径契约 + `target_speeds` 优先 | 采用 `contract::isFrameSupported`/`kFrameBaseLink` | ✅ 已采用（本 PR） |
| Pure Pursuit | 订阅 `target_speeds`，含显式零速测试 | 采用契约常量/校验器（话题名/QoS/frame） | ⏳ 待接入（判定已就绪） |
| velocity_profiler | 发布 `target_speeds`（不写 z 速度） | 直接引用契约话题常量 | ⏳ 待接入 |
| skidpad / straight_line 规划器 | 仍向 `Point.z` 写值（旧协议） | 迁移为 `target_speeds[]`，z 仅作占位/高度 | ⏳ **兼容退出项**：下游全部读 `target_speeds` 后移除 z 速度语义 |
| vehicle_state / 仿真器 | 透传质量/设备时间（#12） | 复用契约哨兵常量 | ⏳ 待接入 |
| safety_monitor | `transient_local` 锁存 stop（#8） | 由 `kQosStop` 构造 QoS | ⏳ 待接入 |

**兼容退出条件**：当某话题的全部消费者均改用契约判定、且无发布者依赖 `Point.z` 传速后，删除对应
回退分支；在此之前保留回退但记录为过渡。集成验证遵循"一条链路迁移→逐步推广"，避免一次性改动所有算法。
