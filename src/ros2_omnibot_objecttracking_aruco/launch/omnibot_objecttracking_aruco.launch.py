from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='ros2_omnibot_camera',
                executable='omnibot_camera_publisher.py',
                output='screen'
            ),
            
            Node(
                package='ros2_omnibot_objecttracking_aruco',
                executable='omnibot_objecttracking_aruco.py',
                output='screen'
            ),
        ]
    )