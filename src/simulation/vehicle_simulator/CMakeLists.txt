cmake_minimum_required(VERSION 3.16)
project(vehicle_simulator)

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
find_package(rosgraph_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

# 仿真核心算法库
add_library(vehicle_simulator_lib SHARED
  src/bicycle_model.cpp
  src/sensor_simulator.cpp
)
target_include_directories(vehicle_simulator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vehicle_simulator_lib
  common_msgs::common_msgs
)

# 仿真节点可执行文件
add_executable(vehicle_simulator_node
  src/simulator_node.cpp
  src/simulator_main.cpp
)
target_link_libraries(vehicle_simulator_node
  vehicle_simulator_lib
  rclcpp::rclcpp
  common_msgs::common_msgs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  std_msgs::std_msgs
  rosgraph_msgs::rosgraph_msgs
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

# 安装配置
install(TARGETS
  vehicle_simulator_lib
  vehicle_simulator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config tracks launch
  DESTINATION share/${PROJECT_NAME}
)

# 单元测试
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bicycle_model
    test/test_bicycle_model.cpp
  )
  target_link_libraries(test_bicycle_model
    vehicle_simulator_lib
  )
  # #53：BicycleModel 纯 std 单测（脱 ROS/msgs）。与 tests/core_standalone 注册同一份文件，
  # 以便在无 ROS 的 ASan+UBSan 门里运行。
  ament_add_gtest(test_bicycle_model_core
    test/test_bicycle_model_core.cpp
  )
  target_link_libraries(test_bicycle_model_core
    vehicle_simulator_lib
  )
  # #54：感知几何核纯 std 单测（header-only）。与 tests/core_standalone 同源；
  # 仅需 common_msgs 的手写纯头（cone_types.h）包含路径，故链接 common_msgs。
  ament_add_gtest(test_sensor_model_core
    test/test_sensor_model_core.cpp
  )
  target_link_libraries(test_sensor_model_core
    common_msgs::common_msgs
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(vehicle_simulator_lib)
ament_export_dependencies(
  rclcpp
  common_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

ament_package()
