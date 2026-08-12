# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription(
#         [
#             Node(
#                 package='ros2_omnibot_master',
#                 executable='omnibot_master.py'
#             ),

#             Node(
#                 package='ros2_omnibot_description',
#                 executable='omnibot_description.launch.py'
#             ),
            
#             Node(
#                 package='sllidar_ros2',
#                 executable='sllidar_c1_launch.py'
#             ),
#         ]
#     )