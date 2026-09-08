cmake_minimum_required(VERSION 3.16)
project(lidar_cluster)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Empty CMAKE_BUILD_TYPE compiles without -O; 10Hz lidar cannot afford that.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()


find_package(ament_cmake REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Threads REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree search segmentation visualization)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_viewer src/pcd_viewer.cpp)
target_link_libraries(${PROJECT_NAME}_viewer
  common_msgs::common_msgs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  pcl_conversions::pcl_conversions
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  visualization_msgs::visualization_msgs
  ${PCL_LIBRARIES}
)


link_directories(${PCL_LIBRARY_DIRS})
add_library(${PROJECT_NAME} SHARED
  src/distortion_adjuster.cpp
  src/lidar_cluster.cpp
  src/utility.cpp
  src/cluster_visualizer.cpp
  src/lidar_cluster_component.cpp
  src/imu_subscriber.cpp
  src/profiler/frame_profiler.cpp
  src/ground_segmentation/svd_ground_strategy.cpp
  src/ground_segmentation/ransac_ground_strategy.cpp
  src/preprocessor/point_cloud_preprocessor.cpp
  src/lidar_cluster_scoring.cpp
)
target_link_libraries(${PROJECT_NAME}
  common_msgs::common_msgs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  pcl_conversions::pcl_conversions
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_components::component
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  visualization_msgs::visualization_msgs
  ${PCL_LIBRARIES}
)

rclcpp_components_register_nodes(${PROJECT_NAME} "lidar_cluster::LidarClusterComponent")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_viewer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES lidar_cluster_component_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  common_msgs
  std_msgs
  nav_msgs
  visualization_msgs
  pluginlib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lidar_cluster_scoring test/test_lidar_cluster_scoring.cpp)
  target_link_libraries(test_lidar_cluster_scoring ${PROJECT_NAME})

  ament_add_gtest(test_lidar_cluster_component test/test_lidar_cluster_component.cpp)
  target_link_libraries(test_lidar_cluster_component ${PROJECT_NAME})
endif()

ament_package()
