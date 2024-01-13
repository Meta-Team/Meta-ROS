from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanum_chassis',
            executable='mecanum_chassis',
            name='mecanum_chassis',
        )
    ])