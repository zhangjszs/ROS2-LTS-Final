cmake_minimum_required(VERSION 3.16)
project(velocity_profiler)

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
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# 速度规划算法核心库
add_library(velocity_profiler_lib SHARED
  src/velocity_profiler.cpp
)
target_include_directories(velocity_profiler_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(velocity_profiler_lib
  common_msgs::common_msgs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  visualization_msgs::visualization_msgs
)

# ROS 节点可执行程序
add_executable(velocity_profiler_node
  src/velocity_profiler_node.cpp
  src/velocity_profiler_main.cpp
)
target_link_libraries(velocity_profiler_node
  velocity_profiler_lib
  rclcpp::rclcpp
  common_msgs::common_msgs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  visualization_msgs::visualization_msgs
  std_msgs::std_msgs
)

install(TARGETS
  velocity_profiler_lib
  velocity_profiler_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/ launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_velocity_profiler
    test/test_velocity_profiler.cpp
  )
  target_link_libraries(test_velocity_profiler
    velocity_profiler_lib
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(velocity_profiler_lib)
ament_export_dependencies(
  rclcpp
  common_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

ament_package()
