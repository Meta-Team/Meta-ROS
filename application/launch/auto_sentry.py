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
        Node(
            package='omni_chassis',
            executable='omni_chassis',
            name='omni_chassis',
            parameters=[config],
        ),
        Node(
            package='dji_controller',
            executable='dji_controller',
            name='dji_controller',
            namespace='dji_controller_chassis',
            parameters=[config],
        ),
        Node(
            package='motor_feedback',
            executable='motor_feedback',
            name='motor_feedback',
            namespace='motor_feedback_chassis',
            parameters=[config],
        ),
    ])