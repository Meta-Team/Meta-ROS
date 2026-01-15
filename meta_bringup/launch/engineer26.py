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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

# Necessary dirty work that lets us import modules from the meta_bringup package
import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('meta_bringup'), 'launch'))

from launch_utils import load_controller, register_loading_order, register_sequential_loading

ARGUMENTS = [
    DeclareLaunchArgument(
        'enable_simulation',
        default_value='false',
        description='If true, the simulation will be started'),
]

def generate_launch_description():
    # Launch Arguments
    enable_simulation = LaunchConfiguration('enable_simulation')
    moveit_config = MoveItConfigsBuilder("engineer_arm").robot_description(file_path="config/engineer26.xacro").to_moveit_configs()
    meta_manipulator_moveit_config = (
        MoveItConfigsBuilder("engineer_arm") # package prefix of engineer_arm_moveit_config
        # .robot_description(file_path="config/meta_arm.urdf.xacro")
        .robot_description(file_path="config/engineer26.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[meta_manipulator_moveit_config.to_dict()],
    )
    
    # Get Chassis URDF via xacro
    robot_description_content = Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            # PathJoinSubstitution([FindPackageShare('metav_description'), 'urdf', 'engineer26', 'engineer26.xacro']),
            PathJoinSubstitution([FindPackageShare('engineer_arm_moveit_config'), 'config', 'engineer26.xacro']),
            ' ',
            'is_simulation:=', enable_simulation,
    ])

    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': enable_simulation,
            'robot_description': robot_description_content,
            'publish_frequency': 100.0}
        ],
        output='both',
        emulate_tty=True
    )

    robot_config = PathJoinSubstitution([FindPackageShare('meta_bringup'), 'config', 'engineer26.yaml'])
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_config],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output='both',
        emulate_tty=True,
        condition=UnlessCondition(enable_simulation)
    )

    load_joint_state_broadcaster = load_controller('joint_state_broadcaster')

    # List of controllers to be loaded sequentially
    # Order in this list is IMPORTANT
    load_controllers = [
        load_controller('wheels_pid_controller'), # pid go first since bottom side
        load_controller('omni_chassis_controller'),
        load_controller('end_effector_vel2eff_pid_controller'), # pid go first since bottom side
        load_controller('end_effector_pos2vel_pid_controller'),
        load_controller('forward_end_effector_pos_controller'),
        # load_controller('meta_manipulator_controller'), # actually a JTC
        load_controller('forward_debug1_controller'), # actually a JTC
        load_controller('forward_debug2_controller'), # actually a JTC
        load_controller('forward_debug3_controller'), # actually a JTC
        load_controller('forward_debug4_controller'), # actually a JTC
        load_controller('forward_debug5_controller'), # actually a JTC
        load_controller('forward_debug6_controller'), # actually a JTC
        load_controller('forward_debug7_controller'), # actually a JTC
    ]
    dbus_control_node = Node(
        package='dbus_control',
        executable='dbus_control_node',
        name='dbus_control_node',
        parameters=[robot_config],
        output='both',
        emulate_tty=True,
    )
    # decision node
    engineer26_node = Node(
        package='engineer26',
        executable='engineer26_node',
        name='engineer26',
        output='both',
        parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                robot_config,
            ],
        emulate_tty=True
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        # 参数格式: x y z yaw pitch roll parent_frame child_frame
        arguments=["--frame-id", "world", "--child-frame-id", "base"],
        # arguments=["0", "0", "0", "0", "0", "0", "world", "base"],
    )


    return LaunchDescription([
        # Launch Arguments
        *ARGUMENTS,
        # Load robot state publisher
        node_robot_state_publisher,
        # Launch controller manager (if not in simulation)
        controller_manager,
        # Load joint state broadcaster
        load_joint_state_broadcaster,
        # Load controllers
        *register_sequential_loading(load_joint_state_broadcaster, *load_controllers),
        dbus_control_node,
        # referee_system_node,
        # static_tf,
        # run_move_group_node,
        engineer26_node,
    ])
