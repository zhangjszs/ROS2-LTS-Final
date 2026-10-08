# ROS2-LTS-Final — HUAT FSAC 无人驾驶系统 (ROS 2 & Modern C++20)

[![CI](https://github.com/zhangjszs/ROS2-LTS-Final/actions/workflows/ci.yml/badge.svg)](https://github.com/zhangjszs/ROS2-LTS-Final/actions/workflows/ci.yml)
[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy%20(CI)%20%7C%20Rolling-22314E?logo=ros)](https://docs.ros.org/)
[![C++ Standard](https://img.shields.io/badge/C%2B%2B-20-00599C?logo=c%2B%2B)](https://en.cppreference.com/w/cpp/20)
[![Build System](https://img.shields.io/badge/Build-Colcon%20%2B%20CMake-064F8C?logo=cmake)](https://colcon.readthedocs.io/)
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)

本项目为 **HUAT FSAC 自动驾驶车队 2026 赛季** 核心技术代码库，承接上一代 [ROS1-LTS-Final](https://github.com/zhangjszs/ROS1-LTS-Final) 与 [ROS1_FIN_VERSION](https://github.com/HUAT-FSAC/ROS1_FIN_VERSION)。
代码架构全面升级至 **ROS 2** 通信中间件与构建体系，并深度应用 **Modern C++20** 进行高性能、零拷贝重构。

---

## 目录
- [一、项目架构与包全景](#一项目架构与包全景)
- [二、技术栈与工程亮点](#二技术栈与工程亮点)
- [三、工作区目录树](#三工作区目录树)
- [四、环境准备与极速构建](#四环境准备与极速构建)
- [五、快速启动与仿真调试](#五快速启动与仿真调试)
- [六、技术文档中心](#六技术文档中心)
- [七、开源协议](#七开源协议)

---

## 一、项目架构与包全景

系统包含感知（激光/相机/多模融合）、定位、安全诊断、多赛制规划、控制与可视化模块：

```text
src/
├── common/                  # 通用工具与基础设施
│   ├── common_msgs/         # 车队通用自定义消息接口
│   └── ins/                 # 惯导 (INS) 消息转换桥接
├── autodrive_msgs/          # 自动驾驶统一状态/控制协议接口
├── sensors/                 # 传感器与核心感知算法
│   ├── lidar_cluster/       # 激光雷达点云预处理与欧式聚类
│   ├── cone_fusion/         # 雷达与相机锥桶空间融合 (TF 全局变换)
│   └── cone_tracker/        # 锥桶目标追踪 (KD-Tree + 卡尔曼滤波 + 滑窗去重)
├── vision_ros/              # 视觉深度学习目标检测 ROS 2 包装层
├── localization/
│   └── vehicle_state/       # 车辆状态估计与里程计位姿融合
├── planning/                # 赛道决策与路径规划
│   ├── urinay/              # 核心高速赛道循迹规划器 (Ur-inay)
│   ├── skidpad_planner/     # 八字绕环 (Skidpad) 专用规划器
│   └── straight_line_planner/# 直线加速专用规划器
├── control/
│   └── pure_pursuit/        # 经典自适应前瞻纯追踪横向控制器
├── safety/
│   └── safety_monitor/      # 规划心跳监视与双冗余紧急停车 (AS/EBS) 监控
├── visualization/
│   └── fsac_viz/            # 赛道锥桶、轨迹与车辆状态 RViz2 可视化插件
└── launch/
    └── huat_launch/         # 车载工控机全系统集中式启动管理
```

---

## 二、技术栈与工程亮点

1. **ROS 1 到 ROS 2 全面迁移**：
   - 通信底层切换为 DDS，告别 ROS Master 单点故障；
   - 采用 `ament_cmake` / `rclcpp` / `rclpy` 标准现代化 API。
2. **Modern C++20 重构落地**：
   - 引入 `<numbers>` 精确数学常数、Designated Initializers 聚合初始化；
   - 推进 `std::format` 高性能日志、`std::span` 零拷贝连续视图与 `std::ranges` 惰性数据流（详见文档中心）。
3. **Colcon 现代化构建极限优化**：
   - 引入 `mold` 现代高并发链接器与 `ccache` 缓存编译；
   - 彻底解决 ROS 2 跨包链接爆炸与 CPU/内存争夺死锁问题，构建提速高达 **20 倍**（详见 [BUILD_OPTIMIZATION.md](docs/BUILD_OPTIMIZATION.md)）。

---

## 三、工作区目录树

```text
.
├── src/                     # ROS 2 源代码包 (16 个功能模块)
├── docs/                    # 系统设计、性能分析与现代化重构文档
│   ├── BUILD_OPTIMIZATION.md       # 构建架构差异与提速 20x 优化指南
│   └── CPP20_REFACTORING_ROADMAP.md # C++20 渐进式重构 14 天落地计划
├── colcon_defaults.yaml     # 预置构建配置 (带 ccache、mold、并发限流)
├── LICENSE                  # BSD 3-Clause 开源协议
└── README.md                # 统一导览文档
```

---

## 四、环境准备与极速构建

### 1. 基础依赖安装

```bash
# 推荐使用 ROS 2 Humble / Iron / Rolling
sudo apt update
sudo apt install -y ccache mold
```

### 2. 编译工程

本项目根目录已内置 `colcon_defaults.yaml`，自动启用 mold 链接器与并发限制：

```bash
# 进入工作空间
cd /path/to/ROS2-LTS-Final

# 极速并行构建 (自动应用 colcon_defaults.yaml 优化参数)
colcon build --symlink-install

# 刷新环境
source install/setup.bash
```

---

## 五、快速启动与仿真调试

```bash
# 1. 启动全系统综合节点
ros2 launch huat_launch full_system.launch.py

# 2. 单独启动感知融合与可视化
ros2 launch huat_launch perception.launch.py
ros2 launch fsac_viz rviz.launch.py
```

### 3. 一键演示：正常 vs 故障（`scripts/demo.sh`）

**问题背景**：上面的 launch 命令只把节点拉起来——既不注入故障，也不给结论；仓库访客/新成员缺一个"一条命令看懂系统能跑什么"的入口。

**启动方法**（两臂都是 CI 常绿门禁脚本，本入口只做串联、回显与退出码传播）：

```bash
# 前置：已完成第四节构建（install/setup.bash 存在）；
#       本机若 UDP 组播被 VPN 挡住，先 export FASTDDS_BUILTIN_TRANSPORTS=SHM（CI 上不需要）
bash scripts/demo.sh
```

| 臂（脚本） | 演示内容 | 判据 |
|---|---|---|
| 正常臂 `closed_loop_sim_smoke.sh` | 多节点闭环真的闭合：仿真器 → 速度规划 → Pure Pursuit → 安全/仲裁 → 回到仿真器 | `/clock` 随仿真时间推进、车速真的上升、出口指令帧合法（帧头/校验和）、评测器吃到样本 |
| 故障臂 `fault_injection_smoke.sh` | 注入 10 类软故障（路径停发、空路径、源帧被篡改、外部停车与恢复、任务级锁存停、监控退出、仲裁器重启…） | 每个故障有可重复的预期终态：检测 → 状态迁移 → 出口零油门 + 制动；时间线落盘 `build/demo/fault/*.json` |

任一臂失败、或故障臂证据文件缺失/字段不符 → 脚本非零退出，不做"跑完就算过"的宽容。架构与话题契约细节见 [一、项目架构与包全景](#一项目架构与包全景) 与 [INTERFACE_CONTRACT.md](docs/INTERFACE_CONTRACT.md)。

**已知限制**：

- **本演示只提供仿真证据，不构成实车可用结论**（D-004）：硬件急停链路、VCU 断连/掉电、台架标定需实车/台架；感知与规划质量评估（真实 bag）见 #18 / #19。
- 故障臂只覆盖软件可观测的软故障，真实断连/掉电/硬件急停不在其内。
- 需要已构建的工作区与 ROS 2 环境，耗时约 1–2 分钟；全程无界面（不拉 RViz、不需要 DISPLAY），输出落在 `build/demo/`。
- 视频录制、大 bag 托管、发布 release 不在本入口范围（属完整展示 #26）。

**相关**：[#26 完整展示（视频 / bag / release，父跟踪）](https://github.com/zhangjszs/ROS2-LTS-Final/issues/26) · [#25 技术博客](https://github.com/zhangjszs/ROS2-LTS-Final/issues/25)

---

## 六、技术文档中心

- 📘 [构建与编译优化指南 (BUILD_OPTIMIZATION.md)](docs/BUILD_OPTIMIZATION.md)
- 🚀 [C++20 重构推进路线图 (CPP20_REFACTORING_ROADMAP.md)](docs/CPP20_REFACTORING_ROADMAP.md)
- 🤖 [Claude Code 开发与架构指南 (CLAUDE.md)](CLAUDE.md)


---

## 七、开源协议

本项目遵循 [BSD-3-Clause](LICENSE) 开源许可证。
欢迎各类交流与贡献！
