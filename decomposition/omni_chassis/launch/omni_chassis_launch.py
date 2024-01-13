from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='omni_chassis',
            executable='omni_chassis',
            name='omni_chassis',
        )
    ])