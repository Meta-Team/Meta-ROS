import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('agv_chassis'),
        'config',
        'agv_chassis_test.yaml'
    )
    return LaunchDescription([
        Node(
            package='agv_chassis',
            executable='agv_chassis',
            name='agv_chassis',
            parameters=[config],
        ),
        Node(
            package='dji_controller',
            executable='dji_controller',
            name='dji_controller',
            parameters=[config],
        ),
        Node(
            package='teleop_moving',
            executable='teleop_moving',
            name='teleop_moving',
        )
    ])