cmake_minimum_required(VERSION 3.16)
project(track_benchmark)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vehicle_simulator REQUIRED)
# #19 B1：离线 runner 接入 MPC 数学核（mpc_controller_lib 无 rclcpp 依赖，仅链 ROS 基础消息库）
find_package(mpc_controller REQUIRED)
# #36：第三口径复用 velocity_profiler_lib（参考 track_benchmark 已依赖 mpc_controller_lib 的先例）
find_package(velocity_profiler REQUIRED)
find_package(Eigen3 REQUIRED)  # mpc_model.hpp 自带 Eigen 头（与 mpc_controller 同一老式依赖声明）

include_directories(
  include
)

# 评测与赛道生成算法库
add_library(track_benchmark_lib SHARED
  src/track_generator.cpp
  src/kpi_evaluator.cpp
)
target_include_directories(track_benchmark_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(track_benchmark_lib
  common_msgs::common_msgs
)

# 评测 ROS 节点可执行程序
add_executable(track_benchmark_node
  src/benchmark_node.cpp
  src/benchmark_main.cpp
)
target_link_libraries(track_benchmark_node
  track_benchmark_lib
  rclcpp::rclcpp
  common_msgs::common_msgs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  std_msgs::std_msgs
  visualization_msgs::visualization_msgs
)

# #17-B：离线确定性核心回归 runner（纯定步长积分，不依赖 ROS 调度）
add_executable(benchmark_runner src/benchmark_runner.cpp)
target_include_directories(benchmark_runner PRIVATE
  ${vehicle_simulator_INCLUDE_DIRS}
  ${mpc_controller_INCLUDE_DIRS}
  ${velocity_profiler_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(benchmark_runner
  track_benchmark_lib
  ${vehicle_simulator_LIBRARIES}
  ${mpc_controller_LIBRARIES}
  ${velocity_profiler_LIBRARIES}
  common_msgs::common_msgs
)

install(TARGETS
  track_benchmark_lib
  track_benchmark_node
  benchmark_runner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kpi_evaluator
    test/test_kpi_evaluator.cpp
  )
  target_link_libraries(test_kpi_evaluator
    track_benchmark_lib
  )

  # #44（#19 B2）：控制器诊断采集层（header-only 纯 std，也注册进 tests/core_standalone）
  ament_add_gtest(test_controller_diag
    test/test_controller_diag.cpp
  )
  # #47：诊断层引用 mpc::QpAcceptance（qp_solver.hpp，纯 std+Eigen 头，无 rclcpp），
  # 跟 runner 同一老式变量接法取 include 路径（不新建库、不改依赖方向）。
  target_include_directories(test_controller_diag PRIVATE
    ${mpc_controller_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
  # #29：速度包络单一实现回归门（header-only 纯 std，链包内 lib 取 include 路径）
  ament_add_gtest(test_speed_envelope
    test/test_speed_envelope.cpp
  )
  target_link_libraries(test_speed_envelope
    track_benchmark_lib
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(track_benchmark_lib)
ament_export_dependencies(
  rclcpp
  common_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

ament_package()
