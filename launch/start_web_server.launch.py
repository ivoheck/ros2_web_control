import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    fast_api_node = Node(
       package='ros2_web_control',
       executable='fast_api_node',
       parameters=[os.path.join(get_package_share_directory('ros2_web_control'),'config/web_control_config.yaml')]
    )

    return LaunchDescription([
        fast_api_node,
    ])