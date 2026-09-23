# vehicle_state

车辆状态估计节点（ROS 2 / `ament_cmake`）。订阅惯导（INS）原始数据，解算出以车辆起始位姿为原点的
局部平面位置、航向角与速度等信息，并对外发布整车状态与 TF，供下游感知/规划/控制模块使用。

## 功能

- 订阅 Asensing 惯导话题，将大地坐标（经纬度/高程）转换为以车身起始位置为基准的 ENU 平面坐标。
- 方位角初始化采用多帧圆均值（circular mean），防止首帧漂移传播（参数 `azimuth_init_frames`，默认 5）。
- 以 50 Hz 自旋，发布 `common_msgs/HuatCarstate` 整车状态并通过 `tf2` 广播车辆位姿 TF。
- 通过 `diagnostic_updater` 上报节点健康状态（惯导数据接收频率、时间戳等）。
- 支持测量时间/质量透传与设备时间重复帧判定（issue #12，默认关闭，未经设备标定前不丢帧）。

## 话题与参数

| 项目 | 值 |
| --- | --- |
| 订阅 | `/INS/ASENSING_INS`（`common_msgs/HuatASENSING`） |
| 发布 | `/localization/vehicle_state`（`common_msgs/HuatCarstate`）、TF |
| 配置 | `config/vehicle_state.yaml` |

关键参数：`ins_topic`、`vehicle_state_topic`、`azimuth_init_frames`。

## 构建与运行

```bash
colcon build --packages-select vehicle_state
ros2 run vehicle_state vehicle_state
```

## 目录结构

```
vehicle_state/
├── include/vehicle_state_estimator.h   # 状态估计器声明
├── src/
│   ├── vehicle_state_estimator.cpp     # 解算与发布实现
│   └── main.cpp                        # 节点入口
├── config/vehicle_state.yaml           # 运行时参数
└── test/test_vehicle_state_estimator.cpp
```
