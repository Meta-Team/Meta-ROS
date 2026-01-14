from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='meta_arm_urdf').find('meta_arm_urdf')
    default_model_path = os.path.join(pkg_share, 'urdf/meta_arm.urdf')
    gazebo_ros_pkg = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model', 
            default_value=default_model_path,
            description='Absolute path to robot urdf file'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'meta_arm', '-topic', 'robot_description'],
            output='screen'),
    ])
