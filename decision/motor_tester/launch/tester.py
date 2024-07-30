import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('motor_tester'),
        'config',
        'motor_info.yaml'
    )

    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('motor_controller'),
                'launch/motor_test.py'
            )
        )
    )

    ld = LaunchDescription([
        Node(
            package='dbus_control',
            executable='dbus_control_node',
            name='dbus_control_node'
        ),
        Node(
            package='motor_tester',
            executable='motor_tester',
            name='motor_tester',
            parameters=[config],
            output='screen'
        )
    ])

    ld.add_action(motor_launch)

    return ld