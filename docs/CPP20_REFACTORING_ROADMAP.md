# FSAC 自动驾驶代码库 C++20 每日精进与求职亮点计划表

> **使用方法**：
> 1. 本计划分为 **14 天（Day 1 ~ Day 14）**，每天推进一项独立的 C++20 特性重构。
> 2. 每天开始时，直接复制对应天的 **【推进 Prompt】** 发给 AI，即可自动接续当天的教学、重构与编译验证。
> 3. 完成后在对应项打勾 `[x]`，实现渐进式掌握与工程落地。

---

## 进度看板与每日任务清单

- [x] **Day 1**: [工程基线升级与数学常数库 `<numbers>`](#day-1-工程基线升级与数学常数库-numbers)
- [x] **Day 2**: [聚合类型指定初始化器 (Designated Initializers)](#day-2-聚合类型指定初始化器-designated-initializers)
- [x] **Day 3**: [现代高性能格式化输出 `std::format` (第一部分：性能分析与日志)](#day-3-现代高性能格式化输出-stdformat-第一部分性能分析与日志)
- [ ] **Day 4**: [状态机与监控诊断的 `std::format` 重构 (第二部分：安全诊断)](#day-4-状态机与监控诊断的-stdformat-重构-第二部分安全诊断)
- [ ] **Day 5**: [非拥有式连续内存视图 `std::span` (第一部分：控制与保护模块)](#day-5-非拥有式连续内存视图-stdspan-第一部分控制与保护模块)
- [ ] **Day 6**: [`std::span` 重构点云预处理缓冲区 (第二部分：雷达数据零拷贝)](#day-6-stdspan-重构点云预处理缓冲区-第二部分雷达数据零拷贝)
- [ ] **Day 7**: [`std::span` 重构锥桶去重与追踪接口 (第三部分：感知接口解耦)](#day-7-stdspan-重构锥桶去重与追踪接口-第三部分感知接口解耦)
- [ ] **Day 8**: [函数式流式流水线 `std::ranges::views` (第一部分：前瞻点搜索)](#day-8-函数式流式流水线-stdrangesviews-第一部分前瞻点搜索)
- [ ] **Day 9**: [惰性求值流式重构锥桶多重清洗流水线 (第二部分：感知融合)](#day-9-惰性求值流式重构锥桶多重清洗流水线-第二部分感知融合)
- [ ] **Day 10**: [使用 `std::ranges` 算法替代手写循环与排序 (第三部分：现代算法族)](#day-10-使用-stdranges-算法替代手写循环与排序-第三部分现代算法族)
- [ ] **Day 11**: [编译期概念与约束 `Concepts` (第一部分：二维/三维空间点类型约束)](#day-11-编译期概念与约束-concepts-第一部分二维三维空间点类型约束)
- [ ] **Day 12**: [`Concepts` 重构 KD-Tree 树节点与距离度量 (第二部分：泛型数据结构)](#day-12-concepts-重构-kd-tree-树节点与距离度量-第二部分泛型数据结构)
- [ ] **Day 13**: [宇宙飞船操作符 `<=>` (三路比较器) 简化全部自定义数据结构](#day-13-宇宙飞船操作符--三路比较器-简化全部自定义数据结构)
- [ ] **Day 14**: [`std::jthread` 协作式中断重构与求职简历技术包装](#day-14-stdjthread-协作式中断重构与求职简历技术包装)

---

## 详细任务与每日推进 Prompt

### Day 1: 工程基线升级与数学常数库 `<numbers>`
- **目标文件**：各模块 `CMakeLists.txt`、`src/visualization/fsac_viz/src/fsd_viz_node.cpp`、`src/sensors/cone_fusion/src/cone_fusion_utils.cpp`
- **核心特性**：
  - CMake 标准提升到 C++20 (`set(CMAKE_CXX_STANDARD 20)`)
  - 引入 `<numbers>`，使用 `std::numbers::pi_v<double>` 替换手写宏与浮点魔数
- **面试考核点**：为什么不能用宏 `#define PI`？C++20 `<numbers>` 如何在编译期提供架构最高精度常数推导？
- **【推进 Prompt】**：
  > “请帮我推进 C++20 计划的 **Day 1**：将工程各 CMakeLists.txt 的 C++ 标准升级为 C++20，并在可视化与传感器工具库中用 `<numbers>` 替换手写 PI 与角度弧度转换魔数。请先讲解技术原理与面试考点，再动手修改代码并编译验证。”

---

### Day 2: 聚合类型指定初始化器 (Designated Initializers)
- **目标文件**：`src/control/pure_pursuit/src/pure_pursuit_params.cpp`、`src/planning/urinay/src/utils/urinay_params.cpp`、`VehicleState`
- **核心特性**：使用 `.field = value` 的自解释语法初始化参数与状态结构体。
- **面试考核点**：C++20 指定初始化器与 C 语言 C99 的差异？为什么能杜绝构造函数传参顺序错误导致的无人车漂移/失控隐患？
- **【推进 Prompt】**：
  > “请帮我推进 C++20 计划的 **Day 2**：在纯追踪算法和车辆状态模块中，引入 C++20 指定初始化器（Designated Initializers）重构参数与状态聚合体。请先讲解技术原理与面试考点，再动手修改代码并编译验证。”

---

### Day 3: 现代高性能格式化输出 `std::format` (第一部分：性能分析与日志)
- **目标文件**：`src/sensors/lidar_cluster/src/profiler/frame_profiler.cpp`、`src/sensors/lidar_cluster/include/string_utils.h`
- **核心特性**：消灭低效的 `std::stringstream` 与不安全的 `sprintf`，全面迁移到 `std::format`。
- **面试考核点**：`std::format` 为什么比 `stringstream` 快 3~5 倍？它是如何在编译期实现类型安全性检查的？
- **【推进 Prompt】**：
  > “请帮我推进 C++20 计划的 **Day 3**：在雷达聚类的性能分析器（FrameProfiler）和字符串工具中，使用 C++20 `std::format` 彻底消灭 `std::stringstream`。请先讲解底层原理与面试考点，再动手修改代码并编译验证。”

---

### Day 4: 状态机与监控诊断的 `std::format` 重构 (第二部分：安全诊断)
- **目标文件**：`src/safety/safety_monitor/src/safety_monitor.cpp`、`src/sensors/cone_fusion/src/cone_fusion_utils.cpp`
- **核心特性**：使用 `std::format` 构建结构化诊断报告和状态机异常原因输出。
- **面试考核点**：格式化字符串的扩展能力与零堆内存分配优化。
- **【推进 Prompt】**：
  > “请帮我推进 C++20 计划的 **Day 4**：在安全监控（safety_monitor）和锥桶融合诊断中，使用 C++20 `std::format` 重构状态机异常原因拼接与诊断输出。请先讲解技术原理与面试考点，再动手修改代码并编译验证。”

---

### Day 5: 非拥有式连续内存视图 `std::span` (第一部分：控制与保护模块)
- **目标文件**：`src/control/pure_pursuit/src/control/input_guard.cpp`、`vehicle_command_encoder.cpp`
- **核心特性**：使用 `std::span<const T>` 替代 `const std::vector<T>&` 和裸指针 `const T* data, size_t n`。
- **面试考核点**：什么是“非拥有式视图（Non-owning View）”？为什么说 `std::span` 是现代 C++ 数组传参的最佳范式？
- **【推进 Prompt】**：
  > “请帮我推进 C++20 计划的 **Day 5**：在控制模块的输入保护（input_guard）中，使用 C++20 `std::span` 替代传统 vector 引用与裸指针，实现零拷贝数据传递。请先讲解技术原理与面试考点，再动手修改代码并编译验证。”

---

### Day 6: `std::span` 重构点云预处理缓冲区 (第二部分：雷达数据零拷贝)
- **目标文件**：`src/sensors/lidar_cluster/src/preprocessor/point_cloud_preprocessor.cpp`
- **核心特性**：利用 `std::span` 的 `subspan()` 切片机制，零拷贝处理点云通道与环形扇区数据。
- **面试考核点**：自动驾驶大规模点云处理中，如何通过 `std::span` 消除多次局部拷贝开销？
- **【推进 Prompt】**：
  > “请帮我推进 C++20 计划的 **Day 6**：在点云预处理器中引入 `std::span` 和 `subspan()`，实现对点云缓冲区和扇区分段的零拷贝切片处理。请先讲解技术原理与面试考点，再动手修改代码并编译验证。”

---

### Day 7: `std::span` 重构锥桶去重与追踪接口 (第三部分：感知接口解耦)
- **目标文件**：`src/sensors/cone_tracker/src/cone_dedup_algo.cpp`
- **核心特性**：感知数据聚合层统一采用 `std::span` 接口，解耦容器实现。
- **面试考核点**：接口设计原则——为什么库函数的公共 API 应尽量依赖 View 而非具体 Container？
- **【推进 Prompt】**：
  > “请帮我推进 C++20 计划的 **Day 7**：在锥桶去重跟踪算法（cone_tracker）中，将算法接口全面升级为 `std::span`，实现对不同来源锥桶数据的统一零拷贝访问。请先讲解技术原理与面试考点，再动手修改代码并编译验证。”

---

### Day 8: 函数式流式流水线 `std::ranges::views` (第一部分：前瞻点搜索)
- **目标文件**：`src/control/pure_pursuit/src/pure_pursuit_controller.cpp`
- **核心特性**：使用 `std::views::filter` 与 `std::views::transform` 管道流式搜索前瞻路径点。
- **面试考核点**：什么是惰性求值（Lazy Evaluation）？Ranges 视图与传统嵌套 for 循环在指令展开上的差异？
- **【推进 Prompt】**：
  > “请帮我推进 C++20 计划的 **Day 8**：在纯追踪控制算法中，使用 C++20 `std::ranges` 管道操作符重构路径点距离计算与前瞻点搜寻。请先讲解惰性求值原理与面试考点，再动手修改代码并编译验证。”

---

### Day 9: 惰性求值流式重构锥桶多重清洗流水线 (第二部分：感知融合)
- **目标文件**：`src/sensors/cone_fusion/src/cone_fusion.cpp`
- **核心特性**：锥桶距离阈值筛选、视场角剔除、置信度过滤组合为单一管道，彻底消灭中间临时 vector 分配。
- **面试考核点**：如何避免多段过滤产生的多次堆内存分配（Heap Allocation）？
- **【推进 Prompt】**：
  > “请帮我推进 C++20 计划的 **Day 9**：在锥桶数据融合节点（cone_fusion）中，使用 `std::views::filter` 将多步锥桶筛选与清洗重构为无额外内存分配的流式管道。请先讲解底层机制与面试考点，再动手修改代码并编译验证。”

---

### Day 10: 使用 `std::ranges` 算法替代手写循环与排序 (第三部分：现代算法族)
- **目标文件**：`src/planning/skidpad_planner/src/skidpad_planner_node.cpp`、`src/planning/straight_line_planner/src/line_detector.cpp`
- **核心特性**：使用 `std::ranges::sort`、`std::ranges::min_element`、投影（Projections）简化算法。
- **面试考核点**：C++20 Ranges 算法中的“投影（Projection）”如何省去写复杂自定义比较 lambda 的样板代码？
- **【推进 Prompt】**：
  > “请帮我推进 C++20 计划的 **Day 10**：在八字绕环（skidpad）与直线加速规划器中，使用 `std::ranges` 算法及其投影特性（Projections）替代手写循环与老式 `std::sort`。请先讲解技术原理与面试考点，再动手修改代码并编译验证。”

---

### Day 11: 编译期概念与约束 `Concepts` (第一部分：二维/三维空间点类型约束)
- **目标文件**：`src/planning/urinay/include/structures/Point.hpp`、`src/planning/urinay/src/structures/Point.cpp`
- **核心特性**：定义 `SpatialPoint`、`CalculablePoint` 等 Concept，替代老旧的 `std::enable_if_t`。
- **面试考核点**：SFINAE 的硬伤是什么？Concepts 如何在编译期生成人类可读的清晰报错？
- **【推进 Prompt】**：
  > “请帮我推进 C++20 计划的 **Day 11**：在规划算法的基础几何库（Point/Vector）中，定义 C++20 `Concepts`（概念与约束），替换传统模板特化。请先讲解 Concepts 对比 SFINAE 的优势与面试考点，再动手修改代码并编译验证。”

---

### Day 12: `Concepts` 重构 KD-Tree 树节点与距离度量 (第二部分：泛型数据结构)
- **目标文件**：`src/planning/urinay/src/utils/KDTree.cpp`、`src/planning/urinay/include/utils/KDTree.hpp`
- **核心特性**：用 Concept 约束 KDTree 支持的搜索点类型与距离度量函数。
- **面试考核点**：模板元编程在自动驾驶感知/规划底层基础结构中的工业级落地标准。
- **【推进 Prompt】**：
  > “请帮我推进 C++20 计划的 **Day 12**：在 KD-Tree 空间索引算法中，利用 C++20 `Concepts` 对泛型几何节点类型进行编译期约束。请先讲解技术原理与面试考点，再动手修改代码并编译验证。”

---

### Day 13: 宇宙飞船操作符 `<=>` (三路比较器) 简化全部自定义数据结构
- **目标文件**：`Point.cpp`、`Edge.cpp`、`Triangle.cpp`、`ConeCandidate`
- **核心特性**：使用 `auto operator<=>(const T&) const = default;` 自动合成全套 6 种比较操作符。
- **面试考核点**：三路比较器的弱序（weak_ordering）与全序（strong_ordering）区别？如何避免手工实现 `<` 和 `==` 产生的逻辑不一致 Bug？
- **【推进 Prompt】**：
  > “请帮我推进 C++20 计划的 **Day 13**：使用 C++20 三路比较操作符 `<=>`（宇宙飞船操作符），一键重构规划器中几何结构体与候选目标的比较器。请先讲解原理、比较类别与面试考点，再动手修改代码并编译验证。”

---

### Day 14: `std::jthread` 协作式中断重构与求职简历技术包装
- **目标文件**：`src/safety/safety_monitor/src/safety_monitor.cpp`
- **核心特性**：
  - 用 `std::jthread` 和 `std::stop_token` 替代原始 `std::thread` 与手写 `atomic<bool>` 中断标志；
  - 提炼整套重构的技术总结，生成自动驾驶 C++ 岗位的简历亮点话术与高频面试问答。
- **面试考核点**：RAII 线程管理与安全协作式取消机制。
- **【推进 Prompt】**：
  > “请帮我推进 C++20 计划的 **Day 14**：在安全监控后台线程中使用 `std::jthread` 和 `std::stop_token` 实现现代协作式线程取消，并帮我梳理一套针对自动驾驶公司面试的高光简历项目描述与问答话术。”

---

## 每天推进计划的统一通用 Prompt 模板

如果你不想翻看具体某一天的 Prompt，可以直接发送以下这一句通用指令：

```text
请帮我推进 C++20 重构计划的下一天任务：先查看 docs/CPP20_REFACTORING_ROADMAP.md 中尚未勾选的第一项，先为我讲解该 C++20 新特性的底层原理、为什么比旧代码更好以及自动驾驶面试的核心考点，然后动手修改代码并使用 colcon build 验证，最后更新文档中的勾选状态。
```
