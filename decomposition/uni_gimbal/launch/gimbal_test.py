import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    ahrs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fdilink_ahrs'),
                        'launch/ahrs_driver.launch.py')
        )
    )

    config = os.path.join(
        get_package_share_directory('uni_gimbal'),
        'config',
        'gimbal.yaml'
    )

    ld = LaunchDescription([
        Node(
            package='uni_gimbal',
            executable='uni_gimbal',
            name='uni_gimbal',
            parameters=[config],
        ),
        Node(
            package='dji_controller',
            executable='dji_controller',
            name='dji_controller',
            parameters=[config],
        )
    ])

    ld.add_action(ahrs_launch)

    return ld