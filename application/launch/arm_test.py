import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('scara_moveit'),
                        'launch/scara_test.py')
        )
    )

    ld = LaunchDescription([
        Node(
            package="joy_arm",
            executable="joy_arm",
            name="joy_arm",
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
        )
    ])
    
    ld.add_action(moveit_launch)

    return ld