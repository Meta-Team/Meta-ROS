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
        'wl_config.yaml'
    )

    ahrs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fdilink_ahrs'),
                        'launch/ahrs_driver.launch.py')
        )
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
            package='dbus_wl',
            executable='dbus_wl',
            name='dbus_wl',
            parameters=[config],
        ),

        # decomp
        Node(
            package='wheel_leg_rl',
            executable='wheel_leg_rl',
            name='wheel_leg_rl',
            parameters=[config],
        ),

        # exec
        Node(
            package='motor_controller',
            executable='motor_controller',
            name='motor_controller',
            parameters=[config],
        ),
    ])

    ld.add_action(ahrs_launch)

    return ld