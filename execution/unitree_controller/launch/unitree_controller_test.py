import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('unitree_controller'),
        'config',
        'unitree_controller_test.yaml'
    )
    return LaunchDescription([
        Node(
            package='unitree_controller',
            executable='unitree_controller',
            name='unitree_controller',
            parameters=[config],
        ),
    ])