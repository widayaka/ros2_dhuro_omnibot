# ROS2 Dhuro Omnibot - Omnidirectional Mobile Robot
## Introduction:
This repository contains packages used to run the Dhuro Omnibot mobile robot based on Robot Operating System 2 (ROS2). Packages in this repository can be used for educational and research purposes in the robotics field, especially for autonomous mobile robots. The robot platform in this repository uses an omnidirectional mobile robot with an "X" configuration. 
## Materials:
- Raspberry Pi 5 8GB RAM
- Logitech C525 Camera
- RPLidar C1
- ESP32-based Dhuro Board MC-01: master board to perform sensor calculations, send and receive data from/to Raspberry Pi 5, send and receive data from/to slave board
- ESP32-based Dhuro Board SC-01: slave board specifically used for the motor driver and to perform kinematic calculations of the robot
- Ubuntu 24.04 LTS running on Raspberry Pi 5
- ROS2 Jazzy Jalisco Distribution
## Dhuro Omnibot Packages:
- ros2_omnibot_camera
- ros2_omnibot_description
- ros2_omnibot_linefollowing
- ros2_omnibot_master
- ros2_omnibot_navigation
- ros2_omnibot_objecttracking_aruco
- ros2_oomnibot_objecttracking_hsv
- ros2_omnibot_wallfollowing
## How to Use:
