import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # package_name = "ros2_web_control" 
    # frontend_dir = os.path.join(get_package_share_directory(package_name), "frontend")  

    # frontend_process = ExecuteProcess(
    #         cmd=["npm", "run", "dev"],
    #         cwd=frontend_dir,  
    #         shell=True,  
    #     )
    
    backend_node = Node(
       package='ros2_web_control',
       executable='backend_node',
    )

    return LaunchDescription([
        backend_node
    ])