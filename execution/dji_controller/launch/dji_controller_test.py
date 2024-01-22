import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dji_controller'),
        'config',
        'dji_controller_test.yaml'
    )
    return LaunchDescription([
        Node(
            package='dji_controller',
            executable='dji_controller',
            name='dji_controller',
            parameters=[config],
        ),
    ])