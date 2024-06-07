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
        get_package_share_directory('application'),
        'config',
        'sentry_config.yaml'
    )

    ld = LaunchDescription([
        # percep
        Node(
            package='dbus_control',
            executable='dbus_control_node',
            name='dbus_control_node',
            parameters=[config],
        ),

        # deci
        Node(
            package='dbus_vehicle',
            executable='dbus_vehicle',
            name='dbus_vehicle',
            parameters=[config],
        ),

        # decomp
        Node(
            package='agv_chassis',
            executable='agv_chassis',
            name='agv_chassis',
            parameters=[config],
        ),

        # exec
    ])

    return ld