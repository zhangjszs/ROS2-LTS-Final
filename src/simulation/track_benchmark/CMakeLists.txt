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

install(TARGETS
  track_benchmark_lib
  track_benchmark_node
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
