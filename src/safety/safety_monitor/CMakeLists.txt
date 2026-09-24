cmake_minimum_required(VERSION 3.16)
project(safety_monitor)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(ament_cmake REQUIRED)
find_package(common_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(safety_monitor
  src/safety_monitor.cpp
  src/state_machine/stop_state_machine.cpp
)

target_link_libraries(safety_monitor
  common_msgs::common_msgs
  diagnostic_updater::diagnostic_updater
  rclcpp::rclcpp
  std_msgs::std_msgs
)

# #16：最终指令仲裁节点（订阅 PP/MPC 源 + 锁存 stop -> 单一 /vehicle_command）
add_executable(command_arbiter_node
  src/command_arbiter_node.cpp
)

target_link_libraries(command_arbiter_node
  common_msgs::common_msgs
  rclcpp::rclcpp
)

install(TARGETS
  safety_monitor
  command_arbiter_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stop_state_machine
    test/test_stop_state_machine.cpp
    src/state_machine/stop_state_machine.cpp
  )
  target_link_libraries(test_stop_state_machine
    common_msgs::common_msgs
    diagnostic_updater::diagnostic_updater
    rclcpp::rclcpp
    std_msgs::std_msgs
  )
  # #16：统一任务×安全状态机 + 最终指令仲裁器（纯 std 共用层，无 ROS context）
  ament_add_gtest(test_task_state_machine
    test/test_task_state_machine.cpp
  )
  target_link_libraries(test_task_state_machine
    common_msgs::common_msgs
  )
  ament_add_gtest(test_command_arbitration
    test/test_command_arbitration.cpp
  )
  target_link_libraries(test_command_arbitration
    common_msgs::common_msgs
  )
  # #16 接线：跨帧去抖/stale 的有状态滤波器（确定性、虚拟时钟可测）
  ament_add_gtest(test_command_arbiter_filter
    test/test_command_arbiter_filter.cpp
  )
  target_link_libraries(test_command_arbiter_filter
    common_msgs::common_msgs
  )
endif()

ament_export_dependencies(
  common_msgs
  diagnostic_updater
  rclcpp
  std_msgs
)

ament_package()
