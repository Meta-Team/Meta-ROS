# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='If true, use simulated clock'),
]

def load_pid_controller(joint_name,):
    return ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                joint_name + '_pid_controller'],
        output='screen'
    )

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('metav_description'),
                 'urdf', 'sentry', 'sentry.xacro']
            ),
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description_content},]
    )

    gz_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'sentry', '-allow_renaming', 'true'],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_omni_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'omni_wheel_controller'],
        output='screen'
    )

    wheels_pid_controller = load_pid_controller('wheels')

    gimbal_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'gimbal_controller'],
        output='screen'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
        launch_arguments=[
            ('gz_args', [PathJoinSubstitution([FindPackageShare('metav_gazebo'),'worlds','empty_world.sdf']),
                        ' -r',
                        ' -v 3',])
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([FindPackageShare('metav_bringup'), 'config', 'ros_gz_bridge.yaml']),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        # Launch gazebo environment
        gazebo_launch,
        # Load joint state broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_robot,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        # Load wheels PID controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[wheels_pid_controller],
            )
        ),
        # Load omni wheel controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wheels_pid_controller,
                on_exit=[load_omni_wheel_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_omni_wheel_controller,
                on_exit=[gimbal_controller],
            )
        ),
        # Load robot state publisher
        node_robot_state_publisher,
        # Spawn robot in Gazebo
        gz_spawn_robot,
        # Gazebo Gazebo ROS 2 bridge
        bridge,
        # Launch Arguments
        *ARGUMENTS
    ])