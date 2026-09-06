# FSAC 自动驾驶代码库构建与编译优化指南

> **适用对象**：车队感知、规划、控制等各模块代码维护者与后续新队员。  
> **背景**：本项目由 ROS 1 迁移至 ROS 2，在迁移初期遇到了“全量编译耗时 20+ 分钟甚至死锁卡死”、“单个包链接耗时达 9~10 分钟”、“链接命令行膨胀至 350KB”等严重性能瓶颈。本文档深度剖析该问题的成因、ROS 1 与 ROS 2 的底层构建架构差异，以及标准解决方案。

---

## 目录
1. [现象与优化成效](#一-现象与优化成效)
2. [为什么在 ROS 1 (catkin) 下没问题，迁移到 ROS 2 (colcon) 会爆发严重问题？](#二-为什么在-ros-1-catkin-下没问题迁移到-ros-2-colcon-会爆发严重问题)
3. [根本原因定性：是代码烂、架构配合不好，还是编译参数不对？](#三-根本原因定性是代码烂架构配合不好还是编译参数不对)
4. [规范与最佳实践：以后写 CMakeLists.txt 的准则](#四-规范与最佳实践以后写-cmakeliststxt-的准则)
5. [工作区常用编译命令速查](#五-工作区常用编译命令速查)

---

## 一、 现象与优化成效

在优化前，构建存在三大致命瓶颈：
1. **依赖参数雪崩**：以 `lidar_cluster_viewer` 为例，其链接脚本 `link.txt` 膨胀到 **356 KB**，包含了 **5,905 个链接参数**；`fsac_viz` 单包仅链接就卡住 **558 秒（9.3 分钟）**。
2. **CPU 与内存争夺死锁**：AMD Ryzen 9 7945HX（24 线程）配合 `parallel-workers: 6` 时，内部并发理论高达 $6 \times 24 = 144$ 个编译进程，瞬间挤爆 19GB 物理内存，造成系统 Swap 疯狂换页，整机处于假死状态。
3. **单元测试重复编译**：每个包的 GTest 测试目标不仅编译测试代码，还把已经编译过的业务 `.cpp` 源码重新编译了一遍。

### 优化前后实测数据对比

| 构建阶段 / 测试项 | 优化前 (Legacy) | 优化后 (Modern + Mold + Ccache) | 效果 |
| :--- | :--- | :--- | :--- |
| **`fsac_viz` 首次全新构建** | **558 秒（9.3 分钟）** | **27.8 秒** | ⚡ **提速 20 倍** |
| **`fsac_viz` 链接命令行大小** | **222 KB**（上百次重复库） | **9.4 KB** | 📉 **参数体积缩小 95.7%** |
| **`lidar_cluster_viewer` 参数量** | **5,905 个** | **237 个** | 📉 **参数数量缩减 96%** |
| **`lidar_cluster` 增量构建** | 曾耗时 400s+ 卡死中断 | **0.35 秒** | ⚡ **即时完成** |
| **全工作区全新构建** | 15~20+ 分钟（频繁死锁） | **2 分 44 秒**（含 PCL 探查） | 🚀 **稳定且高速** |
| **全工作区日常增量构建** | 需数分钟逐包排队 | **2.66 秒**（15 个包全部完成） | ⚡ **秒级反馈** |

---

## 二、 为什么在 ROS 1 (catkin) 下没问题，迁移到 ROS 2 (colcon) 会爆发严重问题？

很多队员会困惑：“这套算法代码在 ROS 1 跑得好好的，`catkin build` 几十秒就完了，怎么换到 ROS 2 就彻底瘫痪了？”

核心原因并不是算法变了，而是 **ROS 底层架构与构建体系发生了颠覆性的重构**：

### 1. 统一巨型单体 vs 极度碎片化微组件（核心病因）
- **ROS 1 时代**：
  ROS 1 的核心高度集中，通常一个 `find_package(catkin REQUIRED COMPONENTS roscpp std_msgs ...)`，所有依赖都收敛在统一的 `${catkin_LIBRARIES}` 和 `${catkin_INCLUDE_DIRS}` 变量中。链接器最终只需面对 `libroscpp.so` 等屈指可数的几个预编译动态库，链接极快。
- **ROS 2 时代**：
  ROS 2 进行了极度的模块化拆解：
  - 中间件层：`rmw`、`rmw_implementation`、`rmw_fastrtps_cpp`
  - 序列化与类型层：`rosidl_runtime_c`、`rosidl_typesupport_c`、`rosidl_typesupport_fastrtps_cpp`、`rosidl_typesupport_introspection_cpp` 等等
  - 核心执行层：`rcl`、`rclcpp`、`rcutils`、`rcpputils`、`tracetools`
  在 ROS 2 中，**哪怕只是发一个简单消息，底层就对应了 8~12 个不同的细粒度动态库**！

如果直接套用 ROS 1 的写法，在 `CMakeLists.txt` 里把 8~10 个包的 `${pkg}_LIBRARIES` 堆叠链接：
每个包的 `_LIBRARIES` 变量又各自递归携带了一整套 ROS 2 上游微组件。CMake 在递归扁平化展开时，导致同一动态库被反复加入：
- `librosidl_runtime_c.so` 重复出现了 **175 次**；
- `librcutils.so` 重复出现了 **165 次**；
- `libstd_msgs__rosidl_...` 重复出现了 **146 次**；
- 命令行直接被撑大到 350KB（5900+ 个参数）。
系统默认使用的是 30 年前设计的单线程 **GNU `ld`** 链接器，遇到这种天量级别的拓扑图直接陷入数十分钟的符号扫描与重定位死循环。

---

### 2. 隔离构建（Isolated Build）与双重无节制并发
- **ROS 1 (catkin_make)**：
  传统的 `catkin_make` 是把所有包融合成一个统一的顶层 CMake 树，全局依赖完全去重。
- **ROS 2 (colcon)**：
  `colcon` 采用的是严格的“隔离构建（Isolated Build）”，每个包是独立的 CMake 实例。
  **更严重的是并发策略冲突**：
  - 第一重：`colcon` 的 `--parallel-workers 6`（同时启动 6 个包构建）；
  - 第二重：`colcon` 会给每一个包的底层 `make` 强行传入系统的 CPU 线程上限：`-j24 -l24`！
  - **结果**：$6 \times 24 = 144$ 个编译器/链接器进程在同时运行。
  - PCL（点云库）与 Eigen 属于巨型模板库，单个 `g++` 编译进程就要吃掉 1GB~1.5GB 内存。10 几个进程并发时直接打满 19GB 物理内存，操作系统开始疯狂将内存数据写入 2GB 的虚拟内存 Swap（Swap Thrashing），CPU 时间全部浪费在上下文切换和换页等待上，系统完全“假死”。

---

### 3. C++ 标准演进与编译期模板膨胀
- ROS 1 大多使用 C++11/14，逻辑较为直接；
- ROS 2 深度拥抱 C++17，大量依赖泛型模板、编译期反射推断（Type Introspection）和智能指针。
- 每一个包含 ROS 2 消息的 `.cpp` 文件，编译期展开后的抽象语法树（AST）比 ROS 1 时代大了一个数量级，导致单个文件的编译时间本身就高于 ROS 1。

---

## 三、 根本原因定性：是代码烂、架构配合不好，还是编译参数不对？

这是一个非常客观的工程问题，归纳为以下三点：

1. **并不是核心算法代码“写得太烂”**：
   车队的运动规划、雷达聚类、锥桶融合、纯追踪控制等核心业务逻辑完全没问题，数学推导和逻辑分支都很正常。
2. **根因在于：带着 ROS 1 的“肌肉记忆”去写 ROS 2 的 CMakeLists.txt（工程规范不适配）**：
   - 绝大多数同学在迁移时，机械地把：
     ```cmake
     # ROS 1
     include_directories(${catkin_INCLUDE_DIRS})
     target_link_libraries(node ${catkin_LIBRARIES})
     ```
     改写成了：
     ```cmake
     # ROS 2 中的“大忌”写法：
     include_directories(${rclcpp_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ...)
     target_link_libraries(node ${rclcpp_LIBRARIES} ${geometry_msgs_LIBRARIES} ...)
     ```
     这种写法破坏了现代 CMake 的依赖图谱去重机制，直接引爆了 5900+ 参数的链接器崩溃。
   - 此外，在编写单元测试时，粗暴地把业务 `.cpp` 文件再次丢进 `ament_add_gtest(...)` 中重新编译一遍，而不是直接链接已有的库，导致相同代码在每个包里被 `g++` 编译了两遍。
3. **编译参数与系统工具链缺乏现代化调优**：
   - 未限制并发，任由 144 个进程冲垮 19GB 物理内存；
   - 默认开启了日常并不需要的 `BUILD_TESTING=ON`；
   - 依然停留在单线程 GNU `ld` 时代，未启用现代化的并行链接器（`mold`）与编译器缓存（`ccache`）。

---

## 四、 规范与最佳实践：以后写 CMakeLists.txt 的准则

为保证以后的新包不会再次引入编译缓慢问题，请所有队员遵循以下 CMake 准则：

### ❌ 严禁的旧写法（引发 350KB 依赖爆炸）
```cmake
# 绝对不要这样写！
include_directories(include ${rclcpp_INCLUDE_DIRS} ${common_msgs_INCLUDE_DIRS})
target_link_libraries(my_node
  ${common_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)
```

###  现代标准的 ROS 2 推荐写法（自动去重、毫秒级链接）
使用现代 CMake 的**目标命名空间（Target Namespace）**：
```cmake
add_executable(my_node src/my_node.cpp)

# 仅指定自身头文件目录
target_include_directories(my_node PRIVATE include)

# 直接使用 Target 链接，CMake 会自动推导头文件、编译宏并对数百个底层动态库进行完美去重
target_link_libraries(my_node
  common_msgs::common_msgs
  geometry_msgs::geometry_msgs
  rclcpp::rclcpp
  std_msgs::std_msgs
  # 若有第三方库（如 PCL），单独在此追加
  ${PCL_LIBRARIES}
)
```

###  测试代码（GTest）编写规范
测试代码**严禁**重复加入已有的业务 `.cpp` 源文件进行重复编译：
```cmake
# 错误：重新编译了一遍 cone_utils.cpp
# ament_add_gtest(test_algo test/test.cpp src/cone_utils.cpp)

# 正确：直接链接之前已经编译好的库目标
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_algo test/test.cpp)
  target_link_libraries(test_algo my_package_lib)
endif()
```

---

## 五、 工作区常用编译命令速查

本项目根目录已配置 `colcon_defaults.yaml`，日常构建已自动集成 `mold`、`ccache` 并锁定了安全并发。

### 1. 日常快速构建（全工作区）
```bash
# 秒级增量构建
colcon build
```

### 2. 单包独立构建
```bash
colcon build --packages-select lidar_cluster
```

### 3. 需要运行单元测试时（开启测试构建）
```bash
# 日常默认关闭了测试以保证极速，需要跑测试时传入：
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test
```

### 4. 查看 ccache 缓存命中情况
```bash
ccache -s
```
*(随着开发进行，ccache 的命中率会越来越高，即使执行 `colcon build --cmake-clean-cache`，也能在几十秒内依靠缓存极速恢复产物)*
