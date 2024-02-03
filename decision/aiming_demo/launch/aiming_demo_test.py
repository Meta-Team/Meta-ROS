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
        # perception
        # Node(
        #     package='teleop_keyboard',
        #     executable='teleop_keyboard',
        #     name='teleop_keyboard',
        #     parameters=[config],
        # ),
        # decision
        # Node(
        #     package="teleop_shooting",
        #     executable="teleop_shooting",
        #     name="teleop_shooting",
        #     parameters=[config],
        # ),
        Node(
            package="aiming_demo",
            executable="aiming_demo",
            name="aiming_demo",
            parameters=[config],
        ),
        # decomposition
        Node(
            package="uni_gimbal_demo",
            executable="uni_gimbal_demo",
            name="uni_gimbal_demo",
            parameters=[config],
        ),
        # Node(
        #     package="shoot_load",
        #     executable="shoot_load",
        #     name="shoot_load",
        #     parameters=[config],
        # ),
        # execution
        # Node(
        #     package='dji_controller',
        #     executable='dji_controller',
        #     name='dji_controller',
        #     parameters=[config],
        # ),
    ])