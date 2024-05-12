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
        get_package_share_directory('serial_relay'),
        'config',
        'relay_config.yaml'
    )

    ld = LaunchDescription([
        Node(
            package='serial_relay',
            executable='serial_relay_node',
            name='serial_relay_node',
            parameters=[config],
        ),
    ])

    return ld