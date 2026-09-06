cmake_minimum_required(VERSION 3.8)
project(vehicle_state)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(common_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_lib
  src/vehicle_state_estimator.cpp
)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  common_msgs::common_msgs
  diagnostic_msgs::diagnostic_msgs
  diagnostic_updater::diagnostic_updater
  geometry_msgs::geometry_msgs
  rclcpp::rclcpp
  std_msgs::std_msgs
  tf2::tf2
  tf2_ros::tf2_ros
)

add_executable(vehicle_state src/main.cpp)
target_link_libraries(vehicle_state PRIVATE
  ${PROJECT_NAME}_lib
)

install(TARGETS vehicle_state
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
