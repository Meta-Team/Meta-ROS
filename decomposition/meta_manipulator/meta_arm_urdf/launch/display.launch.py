from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='meta_arm_urdf').find('meta_arm_urdf')
    default_model_path = os.path.join(pkg_share, 'urdf/meta_arm.urdf')
    
    # Check if a default rviz config exists, otherwise don't load one or use a generic one if available
    # For now, we omit the argument if we don't have a file, or create a placeholder.
    # default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model', 
            default_value=default_model_path,
            description='Absolute path to robot urdf file'),
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'),
    ])
