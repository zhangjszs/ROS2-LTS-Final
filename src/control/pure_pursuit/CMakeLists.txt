cmake_minimum_required(VERSION 3.8)
project(pure_pursuit)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
  include
)

add_executable(pure_pursuit_controller
  src/pure_pursuit_controller.cpp
  src/pure_pursuit_params.cpp
  src/control/vehicle_command_encoder.cpp
  src/control/input_guard.cpp
)

target_link_libraries(pure_pursuit_controller
  common_msgs::common_msgs
  diagnostic_updater::diagnostic_updater
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  rclcpp::rclcpp
  std_msgs::std_msgs
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  Eigen3::Eigen
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS
  pure_pursuit_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pure_pursuit
    test/test_pure_pursuit.cpp
    src/control/input_guard.cpp
    src/control/vehicle_command_encoder.cpp
  )
  target_link_libraries(test_pure_pursuit
    common_msgs::common_msgs
    diagnostic_updater::diagnostic_updater
    geometry_msgs::geometry_msgs
    nav_msgs::nav_msgs
    rclcpp::rclcpp
    std_msgs::std_msgs
    tf2::tf2
    tf2_eigen::tf2_eigen
  )
endif()

ament_export_dependencies(
  common_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
)

ament_package()
