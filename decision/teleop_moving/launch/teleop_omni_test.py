import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    chassis_config = os.path.join(
        get_package_share_directory('omni_chassis'),
        'config',
        'omni_chassis_test.yaml'
    )
    control_config = os.path.join(
        get_package_share_directory('teleop_moving'),
        'config',
        'teleop_moving.yaml'
    )
    return LaunchDescription([
        Node(
            package='omni_chassis',
            executable='omni_chassis',
            name='omni_chassis',
            parameters=[chassis_config],
        ),
        Node(
            package='dji_controller',
            executable='dji_controller',
            name='dji_controller',
            parameters=[chassis_config],
        ),
        Node(
            package='teleop_moving',
            executable='teleop_moving',
            name='teleop_moving',
            parameters=[control_config],
        )
    ])