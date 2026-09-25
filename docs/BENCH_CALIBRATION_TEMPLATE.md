# 台架 / 实车执行器标定记录模板（issue #15）

本文件是 #15 交付物第 6 条的执行模板：**记录输入、反馈、误差、时间、配置版本与测试条件**。
标定值一旦确认，写回 [vehicle_calibration.yaml](../src/launch/huat_launch/config/vehicle_calibration.yaml)
并递增 `actuator.calibration_version`；代码中的默认值只是仿真基线，不代表实车事实。

范围边界：本模板只覆盖**软件可观测的接口层**（指令字节 ↔ 车辆反馈）。硬件急停优先级、底盘通道
与 VCU 内部接管逻辑属仓库外，记录在 #23 的迁移清单里，不在本模板内下结论。

---

## 0. 记录元数据（每次上bench必填）

| 字段 | 值 |
|---|---|
| 日期 / 操作人 | |
| 车辆编号 / 底盘版本 | |
| 软件提交（`git rev-parse --short HEAD`） | |
| `actuator.calibration_version`（测前） | |
| 配置文件与校验和（`sha256sum vehicle_calibration.yaml`） | |
| 电量 / 轮胎 / 气压等测试条件 | |
| 反馈数据来源（VCU 日志 / INS / ROS bag 路径） | |

时间源必须写明：车载 `header.stamp` 来自哪个时钟（INS 周秒 or 系统时钟），采样频率与延迟。
参考 #14 §5：接收活性与源数据年龄不得混用。

---

## 1. 未标定项清单（当前全部未标定）

| 参数 | 软件默认值 | 需要的证据 | 状态 |
|---|---|---|---|
| 转向零位 `steering.neutral` | 90 raw | 怠速车轮回正时读 VCU 反馈转角 | 未标定 |
| 转向方向符号 | 右正 | 指令 +5° 观察实际偏转方向 | 未标定 |
| 转向比例 `units_per_degree` | 1.0 raw/° | 多点位指令 ↔ 实测转角斜率 | 未标定 |
| 转向限幅 `min_raw/max_raw` | 65 / 115 | 机械极限 + VCU 饱和点 | 未标定 |
| 转向速率限制 | 未在标定层 | 满舵到反舵的建立时间 | 未标定 |
| `actuator.max_accel` | 5.0 m/s² | 满油门实测纵向加速度（平直路段） | 未标定 |
| `actuator.max_decel` | 8.0 m/s² | 满制动实测减速度 | 未标定 |
| `actuator.pedal_full_scale` | 100.0 | VCU 油门/制动量化域（0–100 还是 0–255） | 未标定 |
| `actuator.emergency_brake_raw` | 80 | 急停指令的制动建立时间与停稳距离 | 未标定 |
| `actuator.soft_brake_raw` | 40 | 末端减速档位的实际减速度 | 未标定 |
| 油门 ↔ 加速度曲线 | 线性假设 | 至少 5 个油门档位的实测点 | 未标定 |
| 制动 ↔ 减速度曲线 | 线性假设 | 至少 5 个制动档位的实测点 | 未标定 |
| 指令超时与底盘接管行为 | 无（软件侧仅 `arbitration.source_timeout_sec`） | 拔线/停发后 VCU 的实际反应 | 未标定 |
| 执行器延迟（指令→生效） | 无 | 指令阶跃与反馈阶跃的时间差 | 未标定 |

未标定期间：`HuatCarstate.quality_validated` 保持 `false`（#12），
`safety.state_source_age_tolerance_sec` 保持 `-1`（禁用年龄判定，不臆判）。

---

## 2. 转向标定（逐点记录）

指令侧：`ros2 topic pub --once /vehicle_command common_msgs/msg/HuatVehicleCmd "{...}"`
（或用 `huat_launch` 的调试入口）。反馈侧读 VCU 转角 / INS 航向变化率。

`raw` 与角度的换算只用 `SteeringCalibration`：`raw = neutral + deg × units_per_degree`。

| # | 指令 deg | 指令 raw | 实测转角 deg | 误差 deg | 建立时间 s | 备注 |
|---|---|---|---|---|---|---|
| 1 | 0 | | | | | 零位判定 |
| 2 | +5 | | | | | 方向符号 |
| 3 | -5 | | | | | 方向符号 |
| 4 | +15 | | | | | 线性区 |
| 5 | -15 | | | | | 线性区 |
| 6 | +25 | | | | | 饱和点 |
| 7 | -25 | | | | | 饱和点 |

判定：往返误差 ≤ 1 raw 单位；左右对称差 ≤ 1 raw；出现任何"符号相反"或"饱和点与限幅不符"
→ 先改 `steering.*` 参数并递增 `calibration_version`，不改代码。

## 3. 纵向标定（油门 / 制动）

`ActuatorCalibration` 的假设是**线性 + 互斥**；本表就是用来判定这个假设是否成立。
若实测明显非线性，记录曲线并在 issue #15 下开子任务改映射，不能把线性假设当实车事实。

油门（`brake_force = 0`）：

| # | pedal raw | 实测加速度 m/s² | 预测（线性） | 误差 | 延迟 s |
|---|---|---|---|---|---|
| 1 | 20 | | | | |
| 2 | 40 | | | | |
| 3 | 60 | | | | |
| 4 | 80 | | | | |
| 5 | 100 | | | | |

制动（`pedal_ratio = 0`，制动优先）：

| # | brake raw | 实测减速度 m/s² | 预测（线性） | 误差 | 建立时间 s | 停稳距离 m |
|---|---|---|---|---|---|---|
| 1 | 20 | | | | | |
| 2 | 40（= `soft_brake_raw`） | | | | | |
| 3 | 60 | | | | | |
| 4 | 80（= `emergency_brake_raw`） | | | | | |
| 5 | 100 | | | | | |

## 4. 安全降级与互斥检查（必须在断开车轮/架起车辆时做）

| 场景 | 指令 | 期望出口字节 | 实测 | 通过 |
|---|---|---|---|---|
| 油门制动互斥 | pedal=50 且 brake=50 | 只允许一个非零（制动优先） | | |
| 非有限输入（软件注入 NaN） | `accel=NaN` | pedal=0，brake=`emergency_brake_raw` | | |
| 负油门窄化 | 试图输出 -1 | 不得出现 255（ROS1 #6 缺陷类） | | |
| 指令停发 | 停发 `vehicle_command` | 底盘侧行为（记录实际反应，不臆断） | | |
| 锁存 stop 期间 | `/system/stop=true` | 出口为安全制动，油门恒 0 | | |
| 急停按钮 | 物理按钮 | 硬件优先级高于任何软件指令 | | |

## 5. 完成定义

- [ ] 第 1 节所有"未标定"项都有实测值或明确标注"底盘不提供该反馈"。
- [ ] `vehicle_calibration.yaml` 的值与本记录一致，`calibration_version` 已递增。
- [ ] 第 4 节全部通过（尤其"负值不产生 255"）。
- [ ] 记录表连同 bag / VCU 日志路径归档，并在 issue #15 里引用。
- [ ] `docs/INTERFACE_CONTRACT.md` §4 的标定状态同步更新。
