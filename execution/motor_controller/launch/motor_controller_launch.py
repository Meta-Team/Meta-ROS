from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import yaml

node_parameters = yaml.safe_load(open(
    os.path.join(get_package_share_directory('motor_controller'), 'config', 'motor_config.yaml'),
      'r'))

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_controller',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
            emulate_tty=True,
            parameters=[node_parameters]
        )
    ])