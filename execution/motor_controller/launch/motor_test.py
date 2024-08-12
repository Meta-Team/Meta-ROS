import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('motor_controller'),
        'config',
        'motor_config.yaml'
    )
    return LaunchDescription([
        Node(
            package='motor_controller',
            executable='motor_controller',
            name='motor_controller',
            parameters=[config],
            output='screen',
        ),
    ])