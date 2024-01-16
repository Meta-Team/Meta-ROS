import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('omni_chassis'),
        'config',
        'motor_config.yaml'
    )
    return LaunchDescription([
        Node(
            package='motor_feedback',
            executable='motor_feedback',
            name='motor_feedback',
            namespace='motor_feedback_chassis',
            parameters=[config],
        ),
    ])