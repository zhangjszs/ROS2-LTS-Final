cmake_minimum_required(VERSION 3.16)

project(common_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HuatConeCluster.msg"
  "msg/HuatInsP2.msg"
  "msg/HuatLanePoint.msg"
  "msg/HuatLocalPlanning.msg"
  "msg/HuatPathData.msg"
  "msg/HuatCone.msg"
  "msg/HuatMap.msg"
  "msg/HuatPathLimits.msg"
  "msg/HuatTracklimits.msg"
  "msg/HuatASENSING.msg"
  "msg/Pose2D.msg"
  "msg/HuatCarstate.msg"
  "msg/HuatNewASENSING.msg"
  "msg/HuatTimeAndAzimuth.msg"
  "msg/HuatControlCommand.msg"
  "msg/HuatVehicleCmd.msg"
  "msg/HuatStop.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

install(DIRECTORY include/ DESTINATION include)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
