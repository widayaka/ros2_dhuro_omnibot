cmake_minimum_required(VERSION 3.8)
project(ros2_omnibot_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ros2_omnibot_camera/omnibot_camera_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
