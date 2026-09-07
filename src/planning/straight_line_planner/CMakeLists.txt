cmake_minimum_required(VERSION 3.16)
project(straight_line_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(common_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_core
  src/straight_line_planner_node.cpp src/line_detector.cpp
)
target_link_libraries(${PROJECT_NAME}_core
  common_msgs::common_msgs
  geometry_msgs::geometry_msgs
  rclcpp::rclcpp
  std_msgs::std_msgs
)

add_executable(${PROJECT_NAME}
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_core ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_line_detector test/test_line_detector.cpp)
  target_link_libraries(test_line_detector ${PROJECT_NAME}_core)
endif()

ament_package()
