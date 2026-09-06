cmake_minimum_required(VERSION 3.8)
project(urinay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/structures/Point.cpp src/structures/Node.cpp src/structures/Edge.cpp
  src/structures/Circle.cpp src/structures/Triangle.cpp src/structures/Trace.cpp
  src/structures/Vector.cpp src/structures/Way.cpp
  src/modules/delaunay_triangulator.cpp src/modules/WayComputer.cpp
  src/modules/urinay_visualizer.cpp src/utils/Time.cpp src/utils/urinay_params.cpp
  src/utils/Failsafe.cpp src/utils/KDTree.cpp src/race/race_director.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  common_msgs::common_msgs
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  visualization_msgs::visualization_msgs
  ament_index_cpp::ament_index_cpp
  Eigen3::Eigen
)

add_executable(${PROJECT_NAME}_exec src/main.cpp)
target_link_libraries(${PROJECT_NAME}_exec PRIVATE ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS ${PROJECT_NAME}_exec RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
