cmake_minimum_required(VERSION 3.16)
project(vision_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(autodrive_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  vision_ros/vision_node_py.py
  vision_ros/cone_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  FILES_MATCHING
    PATTERN "*.onnx"
    PATTERN "*.pt"
)

ament_package()
