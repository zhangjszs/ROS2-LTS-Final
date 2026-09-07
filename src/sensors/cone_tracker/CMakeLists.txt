cmake_minimum_required(VERSION 3.16)
project(cone_tracker)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(common_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_algo src/cone_dedup_algo.cpp)

add_executable(${PROJECT_NAME}_node src/cone_dedup.cpp src/cone_dedup_main.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_algo
  common_msgs::common_msgs
  pcl_conversions::pcl_conversions
  rclcpp::rclcpp
  std_msgs::std_msgs
  ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cone_dedup_algo
    test/test_cone_dedup_algo.cpp
  )
  target_link_libraries(test_cone_dedup_algo
    ${PROJECT_NAME}_algo
  )
endif()

ament_export_dependencies(
  common_msgs
  pcl_conversions
  rclcpp
  std_msgs
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_algo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
