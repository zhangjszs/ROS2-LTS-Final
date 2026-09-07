cmake_minimum_required(VERSION 3.16)
project(fsac_viz)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(common_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Unified visualization node
add_executable(fsd_viz_node src/fsd_viz_node.cpp)
target_include_directories(fsd_viz_node PRIVATE include)
target_link_libraries(fsd_viz_node
  common_msgs::common_msgs
  geometry_msgs::geometry_msgs
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  visualization_msgs::visualization_msgs
)

install(TARGETS
  fsd_viz_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(
  common_msgs
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

ament_package()
