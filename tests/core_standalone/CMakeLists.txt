# ==============================================================================
# #22：Core/ROS 适配边界——无 ROS 独立构建/测试入口（标准 CMake/CTest）。
# 仅依赖 C++20 标准库 + 系统 GTest；不 source ROS、不使用 colcon/DDS。
# 测试源文件复用各包既有单测（单一事实来源，colcon 侧另行注册同一批文件）。
# 用法：
#   cmake -S tests/core_standalone -B build/core_standalone
#   cmake --build build/core_standalone && ctest --test-dir build/core_standalone
# 或 scripts/core_standalone_check.sh（默认 ASan+UBSan 检查）。
# ==============================================================================
cmake_minimum_required(VERSION 3.16)
project(fsac_core_standalone LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(CORE_ASAN "Enable AddressSanitizer + UndefinedBehaviorSanitizer" ON)
if(CORE_ASAN)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
    add_link_options(-fsanitize=address,undefined)
endif()

find_package(GTest REQUIRED)
# #40：MPC 数学核（mpc_model/qp_solver）自带 Eigen 头，而本入口的设计前提就是“不 source ROS”，
# 所以直接依赖系统的 Eigen3（与 CI 构 mpc_controller 时用的是同一份）。
find_package(Eigen3 REQUIRED)
enable_testing()

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# 纯 std core 头的 include 路径（与仓库 include 约定一致：手写头不带前缀）。
set(CORE_INCLUDE_DIRS
    "${REPO_ROOT}/src/common/common_msgs/include"
    "${REPO_ROOT}/src/common/common_msgs/include/common_msgs"
    "${REPO_ROOT}/src/control/pure_pursuit/include"
    "${REPO_ROOT}/src/control/mpc_controller/include"
    "${REPO_ROOT}/src/planning/straight_line_planner/include"
    "${REPO_ROOT}/src/planning/velocity_profiler/include"
    "${REPO_ROOT}/src/safety/safety_monitor/include"
    "${REPO_ROOT}/src/sensors/cone_tracker/include"
    "${REPO_ROOT}/src/sensors/cone_fusion/include"
    "${REPO_ROOT}/src/simulation/track_benchmark/include"
    "${REPO_ROOT}/src/simulation/vehicle_simulator/include"
    "${REPO_ROOT}/src/planning/urinay/include"
    "${EIGEN3_INCLUDE_DIR}"
)

# add_core_test(<name> [HAS_MAIN] SOURCES <src>...)
# HAS_MAIN 用于自带 main() 的测试（其余链接 GTest::gtest_main）。
function(add_core_test name)
    cmake_parse_arguments(ARG "HAS_MAIN" "" "SOURCES" ${ARGN})
    add_executable(${name} ${ARG_SOURCES})
    target_include_directories(${name} PRIVATE ${CORE_INCLUDE_DIRS})
    if(ARG_HAS_MAIN)
        target_link_libraries(${name} PRIVATE GTest::gtest)
    else()
        target_link_libraries(${name} PRIVATE GTest::gtest_main)
    endif()
    add_test(NAME ${name} COMMAND ${name})
endfunction()

# #14 接口契约（纯 std 校验器/常量）
add_core_test(test_interface_contract
    SOURCES "${REPO_ROOT}/src/control/pure_pursuit/test/test_interface_contract.cpp")
# #15 执行器协议编解码（纯 std 帧/校验和/标定）
add_core_test(test_vehicle_command_codec
    SOURCES "${REPO_ROOT}/src/control/pure_pursuit/test/test_vehicle_command_codec.cpp")
# #38 转向编解码的非有限输入/误配标定守卫（旧实现在此会报 std::clamp / lround UB）
add_core_test(test_steering_calibration
    SOURCES "${REPO_ROOT}/src/control/pure_pursuit/test/test_steering_calibration.cpp")
# #16 任务×安全状态机与最终指令仲裁（纯 std 共用层）
add_core_test(test_task_state_machine
    SOURCES "${REPO_ROOT}/src/safety/safety_monitor/test/test_task_state_machine.cpp")
add_core_test(test_command_arbitration
    SOURCES "${REPO_ROOT}/src/safety/safety_monitor/test/test_command_arbitration.cpp")
add_core_test(test_command_arbiter_filter
    SOURCES "${REPO_ROOT}/src/safety/safety_monitor/test/test_command_arbiter_filter.cpp")
# #22 增量 1：PP 纵向控制律 core
add_core_test(test_throttle_controller
    SOURCES "${REPO_ROOT}/src/control/pure_pursuit/test/test_throttle_controller.cpp")
# #22 增量 2：MPC path→ReferencePoint core
add_core_test(test_path_reference_builder
    SOURCES "${REPO_ROOT}/src/control/mpc_controller/test/test_path_reference_builder.cpp")
# #45（#19 协议 P0 前置）：车辆参数一致性自检（文本扫四处真值文件，需仓库根宏）
add_core_test(test_param_consistency
    SOURCES "${REPO_ROOT}/src/control/mpc_controller/test/test_param_consistency.cpp")
target_compile_definitions(test_param_consistency PRIVATE FSAC_REPO_ROOT="${REPO_ROOT}")
# #22 增量 3：straight_line_planner 锥桶边界 core（自带 main）
add_core_test(test_cone_boundary HAS_MAIN
    SOURCES "${REPO_ROOT}/src/planning/straight_line_planner/test/test_cone_boundary.cpp")
# #17 离线评测核（KPI 判定 + 赛道生成实现源）
add_core_test(test_kpi_evaluator
    SOURCES
        "${REPO_ROOT}/src/simulation/track_benchmark/test/test_kpi_evaluator.cpp"
        "${REPO_ROOT}/src/simulation/track_benchmark/src/kpi_evaluator.cpp"
        "${REPO_ROOT}/src/simulation/track_benchmark/src/track_generator.cpp")

# #24：ROS1→ROS2 语义差分合成夹具（跨包纯 std 共用层 + 速度规划核；分类表见 docs/DEFECT_DIFFERENTIAL.md）
add_core_test(test_defect_differential
    SOURCES
        "${REPO_ROOT}/src/control/pure_pursuit/test/test_defect_differential.cpp"
        "${REPO_ROOT}/src/planning/velocity_profiler/test/test_velocity_profiler.cpp"
        "${REPO_ROOT}/src/planning/velocity_profiler/src/velocity_profiler.cpp")
# #32：cone_tracker 差分夹具（ROS1 确认/coast 语义可执行规格 + ROS2 现状锁定）
add_core_test(test_cone_tracker_differential
    SOURCES
        "${REPO_ROOT}/src/sensors/cone_tracker/test/test_cone_tracker_differential.cpp"
        "${REPO_ROOT}/src/sensors/cone_tracker/src/cone_dedup_algo.cpp")
# #30：新鲜度租约语义（纯 std 头）
add_core_test(test_freshness_lease
    SOURCES
        "${REPO_ROOT}/src/safety/safety_monitor/test/test_freshness_lease.cpp")
# #50：连续拒解看门狗（纯 std 头；colcon 侧注册同一文件）
add_core_test(test_reject_watchdog
    SOURCES
        "${REPO_ROOT}/src/control/mpc_controller/test/test_reject_watchdog.cpp")

# ===================== #40：把剩下的纯 std 单测拉进 ASan+UBSan 门 =====================
# 原则：能脱 ROS 编译的全部注册；确实不行的写进下方“排除清单”，不允许默默漏着。

# 控制：MPC 数学核（盒约束 QP + 运动学模型），只依赖 Eigen
add_core_test(test_qp_solver
    SOURCES
        "${REPO_ROOT}/src/control/mpc_controller/test/test_qp_solver.cpp"
        "${REPO_ROOT}/src/control/mpc_controller/src/qp_solver.cpp")
add_core_test(test_mpc_tracking
    SOURCES
        "${REPO_ROOT}/src/control/mpc_controller/test/test_mpc_tracking.cpp"
        "${REPO_ROOT}/src/control/mpc_controller/src/mpc_model.cpp"
        "${REPO_ROOT}/src/control/mpc_controller/src/qp_solver.cpp")

# 评测：速度包络 + KPI 报告的 schema 边界（speed_envelope 为 header-only）
add_core_test(test_speed_envelope
    SOURCES
        "${REPO_ROOT}/src/simulation/track_benchmark/test/test_speed_envelope.cpp")
# #44（#19 B2）：控制器诊断采集层（solve_time 百分位/未收敛率/降级计数，header-only 纯 std）
add_core_test(test_controller_diag
    SOURCES
        "${REPO_ROOT}/src/simulation/track_benchmark/test/test_controller_diag.cpp")

# 安全：停车状态机（锁存/重启语义，纯 std 共用层）
add_core_test(test_stop_state_machine
    SOURCES
        "${REPO_ROOT}/src/safety/safety_monitor/test/test_stop_state_machine.cpp"
        "${REPO_ROOT}/src/safety/safety_monitor/src/state_machine/stop_state_machine.cpp")

# 感知：锥桶去重算法与融合工具（均不依赖 PCL/msgs）
add_core_test(test_cone_dedup_algo
    SOURCES
        "${REPO_ROOT}/src/sensors/cone_tracker/test/test_cone_dedup_algo.cpp"
        "${REPO_ROOT}/src/sensors/cone_tracker/src/cone_dedup_algo.cpp")
add_core_test(test_cone_fusion_utils
    SOURCES
        "${REPO_ROOT}/src/sensors/cone_fusion/test/test_cone_fusion_utils.cpp"
        "${REPO_ROOT}/src/sensors/cone_fusion/src/cone_fusion_utils.cpp")

# 规划：赛程主管状态机（纯 std）
add_core_test(test_race_director
    SOURCES
        "${REPO_ROOT}/src/planning/urinay/test/test_race_director.cpp"
        "${REPO_ROOT}/src/planning/urinay/src/race/race_director.cpp")

# #53：仿真运动学核（纯 std）——此前被 test_bicycle_model 的文件级排除掩盖
add_core_test(test_bicycle_model_core
    SOURCES
        "${REPO_ROOT}/src/simulation/vehicle_simulator/test/test_bicycle_model_core.cpp"
        "${REPO_ROOT}/src/simulation/vehicle_simulator/src/bicycle_model.cpp")

# #54：感知几何核（纯 std，header-only）——FOV/距离/坐标变换/噪声
add_core_test(test_sensor_model_core
    SOURCES
        "${REPO_ROOT}/src/simulation/vehicle_simulator/test/test_sensor_model_core.cpp")

# 排除清单（#40 逐个数过，不靠“碰巧没加”）——这些测试文件本身零 rclcpp，
# 但传递依赖 ROS 生成消息头或 PCL，无法在“不 source ROS”的入口里编译：
#   test_command_roundtrip      → pure_pursuit/vehicle_command_encoder.h 组的是
#                                common_msgs::msg::HuatVehicleCmd（需生成消息）
#   test_vehicle_state_estimator → vehicle_state_estimator.h 直接 include rclcpp + msgs
#   test_bicycle_model          → 现只剩 SensorSimulator 适配层测试，因 sensor_simulator.hpp
#                                拉 common_msgs/msg/huat_cone.hpp 而排除；纯 std 的
#                                BicycleModel 用例在 test_bicycle_model_core、感知几何核在
#                                test_sensor_model_core（均见上方 add_core_test）
#   test_icp_apf_planner         → icp_apf_planner.hpp include common_msgs/geometry msgs
#   test_way / test_kdtree_concepts / test_point_concepts
#                                → Way/delaunay/kdtree 链路 include msgs 与 PCL
#   test_three_way_comparison    → 文件开头只有 std include，但背后半部分拉
#                                structures/Node.hpp（include common_msgs/msg/huat_cone.hpp）
#   test_lidar_cluster_scoring / test_lidar_cluster_component → PCL 点云类型
#   test_line_detector           → line_detector.hpp include common_msgs/msg/huat_cone.hpp
#                                 （测试文件自己只 include 该头，没直接碰 rclcpp）
#   test_pure_pursuit           → 节点级测试，依赖 rclcpp
# 它们仍由 colcon 侧 `colcon test` 跑（普通构建，无 sanitizer）；要在 sanitizer 下跑
# 需要先做 #22 风格的 core/ROS 拆分，那不在本 issue 范围。
