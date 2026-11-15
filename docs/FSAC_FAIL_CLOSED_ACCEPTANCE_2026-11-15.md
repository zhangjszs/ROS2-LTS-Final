# FSAC Fail-Closed 实车控制链路：修复与验收台账

> 2026-11-15 · 基线 `main@b769baff033e9d952a4bb936d976e0e7a8850ac8` · 本分支仅供隔离测试，不允许未经验证直接部署实车。

## 安全不变量

1. **单一命令权威**：`deployment_mode:=real` 时 `use_arbiter:=false` 必须拒绝启动；`/vehicle_command` 只能由 command_arbiter_node 发布。仅靠 launch 参数无法阻止外部 ROS 参与者向同名 topic 发布，实车还需网络隔离和硬件仲裁。
2. **显式授权**：实车默认 IDLE、自动启动关闭；监控有效且明确释放停车后，操作员才可发布 ARM → START。不得通过参数设置自动恢复运行。
3. **停车覆盖**：启动停车、非法规划输入、超时、完赛请求、故障与监控断流必须使最终命令为零油门加有效制动。FAULT > REQUEST > TIMEOUT。
4. **复位权限**：完赛和故障锁存不得因轨迹刷新、监控恢复自动解除。复位需要确认监控释放停车，并确保上游 stop_request 不再持续置位。实际车辆仍需独立的人工确认、机械制动/驱动使能和硬件急停。
5. **不信任历史**：新时间戳不代表新规划。历史路径复用须有绝对年龄界限；无效输入不续租。

## 本分支涉及的修改

- 安全监控：启动即锁存停车；规划消息校验点数、map/base_link 坐标系、有限坐标及来源年龄；10Hz 周期发布停车状态；复位须有新鲜定位与轨迹。
- 仲裁器：实车模式固定 `/vehicle_command` 出口；从独立 stop_request 输入获得 REQUEST 语义；监控心跳断流锁存 FAULT；ARM/START/reset 增加健康门禁。
- 任务状态机：FAULT 不可被 FINISH、REQUEST、TIMEOUT 降级。
- PP：零速/非有限目标不再下发正油门；控制节点对合法零速目标发送制动；非法路径清除。
- 直线/八字规划：历史轨迹最大复用时间默认 0.5s，过期后输出空轨迹，不再当作新规划无限重发。
- ConeTracker：t 与 t-1 的位置/时间成对差分；全局 map 坐标锥桶不再重复减去车辆位移。
- 视觉：stand-ard / lightweight / fallback_only 模式按启动参数分流，默认不强制 HSV。
- 仿真：命令断流制动；离线 benchmark 明示 PP 是运动学 surrogate，不等同于生产 PP 链路。

## 必须完成的验收矩阵（本分支尚未执行 ROS 2 环境测试）

| 编号 | 注入 / 步骤（仅隔离 HIL 或仿真） | 预期 |
| --- | --- | --- |
| S01 | 实车模式传 `use_arbiter:=false` 或 `arbiter_autostart:=true` | launch 拒绝启动 |
| S02 | 无车辆状态/轨迹重启安全监控及仲裁器 | `/vehicle_command` 仅制动；`/system/stop` 为 true |
| S03 | 仅发 ARM、没有 START；或安全监控尚未恢复 | 不允许输出动力 |
| S04 | 监控健康后 ARM → START，输入有效且新鲜的候选控制帧 | 才可选定合法控制源 |
| S05 | 轨迹为空、1 点、NaN、Inf、未知 frame_id、陈旧/未来时间戳 | 不更新 planner lease，不解除停车 |
| S06 | 请求停车后规划继续持续输出、重复发 false | REQUEST 锁存，不得恢复行驶 |
| S07 | FAULT 后收到 FINISH / REQUEST / TIMEOUT | 仍为 FAULT |
| S08 | 监控周期心跳中断超过配置期限，随后自动恢复 | 锁存 FAULT；恢复心跳不能自动解除 |
| S09 | 规划 target_speeds 显式为 0 | 车载 PP 发无油门且制动（非低速固定电流） |
| S10 | 直线边界持续退化但非空 / 八字规划持续失败，超过 0.5s | 历史路径停止续租，下游进入停车 |
| S11 | 车沿 x 恒速移动、锥桶固定在 map | t/t-1 车速差分准确，锥桶不受虚假平移 |
| S12 | vision 标准/轻量/回退模式 | 分别选择 ONNX 原尺寸/ONNX 轻量/HSV；参数类型正确 |
| S13 | simulator 接受有效油门后断流超过 0.5s | 制动，不保持旧油门 |
| S14 | 跨进程重启、DDS transient_local、异常退出、无效复位、重复请求 | 无意外行驶授权或停车清除 |

## 环境执行命令（需具备 ROS 2 Jazzy + 项目依赖）

```bash
colcon build --symlink-install --packages-up-to safety_monitor pure_pursuit   straight_line_planner skidpad_planner cone_tracker vision_ros vehicle_simulator track_benchmark
source install/setup.bash
colcon test --packages-select safety_monitor pure_pursuit cone_tracker   straight_line_planner skidpad_planner vehicle_simulator track_benchmark
colcon test-result --verbose
```

还需要：`python3 -m py_compile src/vision_ros/launch/vision.launch.py src/launch/huat_launch/launch/main/fsac.launch.py`、launch 参数组合检查、`ros2 topic info /vehicle_command --verbose` 确认唯一发布者、rosbag 及故障注入回放、离线台架制动字节/转向零位验证。

## 未解决的验收阻断

- 目前没有在真实 ROS 2 Jazzy / 完整传感器环境运行或通过上述测试；没有实车/HIL 结果。
- `HuatStop` 线上仍只有 `bool stop`；REQUEST 语义依赖仲裁器额外订阅停车请求 topic。必须用启动顺序、丢包、晚加入、重启验证，必要时增加显式 reason/sequence 契约。
- ROS 2 `/system/task_event` 目前是普通 topic，不是经认证的物理上车许可。实车必须通过受限的操作员安全网关、制动器和驱动使能实现授权。
- `planner_timeout`、历史路径复用上限、monitor heartbeat 等暂时均为初始值，不等于制动距离或功能安全认证阈值。
- 离线 track_benchmark 中 PP 为数学 surrogate，不能用其 KPI 代替真实 PP/仲裁/执行器回归；需要新增全链路闭环基准。
- 完赛后 RaceDirector 仍可能保持 stop_request=true；复位前必须让请求源退出/明确释放，并验证监控和仲裁器的双重锁存被正确协调。

**判定：** 代码处于待验收状态。CI + 故障注入 + HIL + 操作员/硬件安全确认通过之前，不得宣称可安全上车或合并到实车基线。
