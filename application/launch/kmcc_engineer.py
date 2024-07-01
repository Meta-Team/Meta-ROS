import os
from pathlib import Path
from struct import pack
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
            package='referee_serial',
            executable='referee_serial_node',
            name='referee_serial_node',
            parameters=[config],
        ),

        # deci
        Node(
            package='km_engineer',
            executable='km_engineer',
            name='km_engineer',
            parameters=[config],
        ),
        Node(
            package='cc_arm',
            executable='cc_arm',
            name='cc_arm',
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
        Node(
            package='motor_controller',
            executable='motor_controller',
            name='motor_controller',
            parameters=[config],
        ),
        Node(
            package='serial_relay',
            executable='serial_relay_node',
            name='serial_relay_node',
            parameters=[config],
        ),
    ])

    ld.add_action(moveit_launch)

    return ld