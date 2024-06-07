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
        Node(
            package='auto_sentry',
            executable='auto_sentry',
            name='auto_sentry',
            parameters=[config],
        ),

        # decomp
        Node(
            package='omni_chassis',
            executable='omni_chassis',
            name='omni_chassis',
            parameters=[config],
        ),
        Node(
            package='uni_gimbal',
            executable='uni_gimbal',
            name='uni_gimbal',
            parameters=[config],
        ),
        Node(
            package='shoot_load',
            executable='shoot_load',
            name='shoot_load',
            parameters=[config],
        ),


        # exec
    ])

    return ld