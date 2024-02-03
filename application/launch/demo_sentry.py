import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('application'),
        'config',
        'teleop_keyboard_config.yaml'
    )
    return LaunchDescription([
        Node(
            package='omni_chassis',
            executable='omni_chassis',
            name='omni_chassis',
            parameters=[config],
        ),
        Node(
            package='dji_controller',
            executable='dji_controller',
            name='dji_controller',
            parameters=[config],
        ),
        Node(
            package='teleop_keyboard',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            parameters=[config],
        ),
        Node(
            package="teleop_moving",
            executable="teleop_moving",
            name="teleop_moving",
            parameters=[config],
        ),
        Node(
            package="uni_gimbal_demo",
            executable="uni_gimbal_demo",
            name="uni_gimbal_demo",
            parameters=[config],
        ),
        Node(
            package="aiming_demo",
            executable="aiming_demo",
            name="aiming_demo",
            parameters=[config],
        ),
        Node(
            package="teleop_shooting",
            executable="teleop_shooting",
            name="teleop_shooting",
            parameters=[config],
        )
    ])