cmake_minimum_required(VERSION 3.16)
project(mpc_controller)

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
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# MPC 控制算法核心库
add_library(mpc_controller_lib SHARED
  src/qp_solver.cpp
  src/mpc_model.cpp
)
target_include_directories(mpc_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(mpc_controller_lib
  common_msgs::common_msgs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
)

# MPC ROS 节点可执行程序
add_executable(mpc_controller_node
  src/mpc_controller_node.cpp
  src/mpc_main.cpp
)
target_link_libraries(mpc_controller_node
  mpc_controller_lib
  rclcpp::rclcpp
  common_msgs::common_msgs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  std_msgs::std_msgs
  diagnostic_updater::diagnostic_updater
)

install(TARGETS
  mpc_controller_lib
  mpc_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_qp_solver
    test/test_qp_solver.cpp
  )
  target_link_libraries(test_qp_solver
    mpc_controller_lib
  )

  ament_add_gtest(test_mpc_tracking
    test/test_mpc_tracking.cpp
  )
  target_link_libraries(test_mpc_tracking
    mpc_controller_lib
  )

  # #22：mpc_core::BuildReferencePath（path→ReferencePoint 纯算法，header-only core）
  ament_add_gtest(test_path_reference_builder
    test/test_path_reference_builder.cpp
  )
  target_link_libraries(test_path_reference_builder
    mpc_controller_lib
  )

  # #45（#19 协议 P0 前置）：车辆参数一致性自检。它要读四处真值文件，
  # 所以必须拿到仓库根；拿不到时路径拼不出来→用例失败（不会默声通过）。
  get_filename_component(FSAC_REPO_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
  ament_add_gtest(test_param_consistency
    test/test_param_consistency.cpp
  )
  target_compile_definitions(test_param_consistency PRIVATE FSAC_REPO_ROOT="${FSAC_REPO_ROOT_DIR}")
endif()

ament_export_include_directories(include)
ament_export_libraries(mpc_controller_lib)
ament_export_dependencies(
  rclcpp
  common_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_eigen
)

ament_package()
