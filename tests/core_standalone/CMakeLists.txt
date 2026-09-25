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
    "${REPO_ROOT}/src/simulation/track_benchmark/include"
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
