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
        'engineer_config.yaml'
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('scara_moveit'),
                        'launch/scara.py')
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
            package='dbus_engineer',
            executable='dbus_engineer',
            name='dbus_engineer',
            parameters=[config],
        ),

        # decomp
        Node(
            package='agv_chassis',
            executable='agv_chassis',
            name='agv_chassis',
            parameters=[config],
        ),
        # Node(
        #     package='relay_sucker',
        #     executable='relay_sucker',
        #     name='relay_sucker',
        #     parameters=[config],
        # ),

        # exec
        Node(
            package='dji_controller',
            executable='dji_controller',
            name='dji_controller',
            parameters=[config],
        ),
        Node(
            package='unitree_controller',
            executable='unitree_controller',
            name='unitree_controller',
            parameters=[config],
        ),
        # Node(
        #     package='serial_relay',
        #     executable='serial_relay_node',
        #     name='serial_relay_node',
        #     parameters=[config],
        # ),
    ])

    ld.add_action(moveit_launch)

    return ld