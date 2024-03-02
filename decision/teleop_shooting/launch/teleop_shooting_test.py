import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('teleop_shooting'),
        'config',
        'teleop_shooting.yaml'
    )
    return LaunchDescription([
        Node(
            package="teleop_shooting",
            executable="teleop_shooting",
            name="teleop_shooting",
            parameters=[config],
        ),
        Node(
            package="shoot_load",
            executable="shoot_load",
            name="shoot_load",
            parameters=[config],
        ),
        Node(
            package='dji_controller',
            executable='dji_controller',
            name='dji_controller',
            parameters=[config],
        ),
    ])