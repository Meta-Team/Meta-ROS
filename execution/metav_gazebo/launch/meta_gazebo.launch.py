
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        'world_sdf',
        default_value='',
        description='The world SDF file to load'),
    DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='The name of the robot to spawn'),
    DeclareLaunchArgument(
        'bridge_config_file',
        default_value='',
        description='The path to the bridge configuration file'),
    DeclareLaunchArgument(
        'robot_x',
        default_value='0.0',
        description='The initial x-coordinate for the robot'),
    DeclareLaunchArgument(
        'robot_y',
        default_value='0.0',
        description='The initial y-coordinate for the robot'),
    DeclareLaunchArgument(
        'robot_z',
        default_value='0.0',
        description='The initial z-coordinate for the robot'),
]

def generate_launch_description():
    world_sdf = LaunchConfiguration('world_sdf')
    robot_name = LaunchConfiguration('robot_name')
    bridge_config_file = LaunchConfiguration('bridge_config_file')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')

    gz_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description',
                   '-name', robot_name, '-allow_renaming', 'true',
                   '-x', robot_x, '-y', robot_y, '-z', robot_z],
        output='both',
        emulate_tty=True
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
        launch_arguments=[
            ('gz_args', [world_sdf,
                        ' -r',
                        ' -v 3',])
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='both',
        emulate_tty=True
    )

    return LaunchDescription([
        gz_spawn_robot,
        gazebo_launch,
        bridge,
    ])
