cmake_minimum_required(VERSION 3.8)
project(ros2_omnibot_objecttracking_aruco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# python and cpp dependencies
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# C++ executable
add_executable(objecttracking_aruco_node_cpp src/omnibot_objecttracking_aruco.cpp)

ament_target_dependencies(
  objecttracking_aruco_node_cpp
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

# Install C++ Executable
install(
  TARGETS objecttracking_aruco_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Instal Python Executable
install(
  PROGRAMS ros2_omnibot_objecttracking_aruco/omnibot_objecttracking_aruco.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
