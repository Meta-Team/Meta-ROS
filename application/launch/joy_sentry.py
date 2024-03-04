import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('application'),
        'config',
        'sentry_config.yaml'
    )
    return LaunchDescription([
        # percep
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[config],
        ),

        # deci
        Node(
            package='joy_vehicle',
            executable='joy_vehicle',
            name='joy_vehicle',
            parameters=[config],
        ),

        # decomp
        Node(
            package="uni_gimbal",
            executable="uni_gimbal",
            name="uni_gimbal",
            parameters=[config],
        ),
        Node(
            package='omni_chassis',
            executable='omni_chassis',
            name='omni_chassis',
            parameters=[config],
        ),

        # exec
        Node(
            package='dji_controller',
            executable='dji_controller',
            name='dji_controller',
            parameters=[config],
        ),
    ])