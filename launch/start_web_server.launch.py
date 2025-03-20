import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    fast_api_node = Node(
       package='ros2_web_control',
       executable='fast_api_node',
    )

    return LaunchDescription([
        fast_api_node,
    ])