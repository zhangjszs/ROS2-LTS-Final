cmake_minimum_required(VERSION 3.8)
project(cone_fusion)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(autodrive_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_utils src/cone_fusion_utils.cpp)

add_executable(${PROJECT_NAME}_node src/cone_fusion.cpp src/cone_fusion_main.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_utils
  autodrive_msgs::autodrive_msgs
  common_msgs::common_msgs
  diagnostic_msgs::diagnostic_msgs
  diagnostic_updater::diagnostic_updater
  geometry_msgs::geometry_msgs
  message_filters::message_filters
  pcl_conversions::pcl_conversions
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  ${PCL_LIBRARIES}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cone_fusion_utils
    test/test_cone_fusion_utils.cpp
  )
  target_link_libraries(test_cone_fusion_utils
    ${PROJECT_NAME}_utils
    common_msgs::common_msgs
  )
endif()

ament_export_dependencies(
  autodrive_msgs
  common_msgs
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  message_filters
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
