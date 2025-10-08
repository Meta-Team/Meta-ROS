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

import os
import sys

from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('meta_bringup'), 'launch'))

from launch_utils import load_controller, register_loading_order, register_sequential_loading

ARGUMENTS = [
    DeclareLaunchArgument(
        'enable_simulation',
        default_value='false',
        description='If true, the simulation will be started',
    )
]

def generate_launch_description():
    # get uddf via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('metav_description'), 'urdf', 'playground', 'armor_tester.xacro'])
    ])

    # node for robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_content}
        ],
        output='both',
        emulate_tty=True
    )

    # ros2 control related launch, to read the yaml configure file
    robot_controller_config = PathJoinSubstitution(
        [
            FindPackageShare('meta_bringup'),
            'config',
            'armor_tester.yaml'
        ]
    )

    # controller manager node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controller_config],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output='both',
        emulate_tty=True,
    )

    # node for dbus_controll, not a controller
    dbus_control_node = Node(
        package='dbus_control',
        executable='dbus_control_node',
        name='dbus_control_node',
        parameters=[robot_controller_config],
        output='both',
        emulate_tty=True
    )

    # node for armor_tester, and the executable is armor_tester_node
    armor_tester_node = Node(
        package='armor_tester',
        executable='armor_tester_node',
        name='armor_tester',
        output='both',
        parameters=[robot_controller_config],
        emulate_tty=True
    )

    load_joint_state_broadcaster = load_controller('joint_state_broadcaster')

    load_controllers = [
        load_controller('armor_tester_controller')
    ]

    return LaunchDescription([
        # lauch arguments
        *ARGUMENTS,
        # load robot state publisher
        dbus_control_node,
        armor_tester_node,
        node_robot_state_publisher,
        # launch controller manager(if not in simulation)
        controller_manager,
        # load joint state broadcaster
        load_joint_state_broadcaster,
        # load controller
        *register_sequential_loading(load_joint_state_broadcaster, *load_controllers),
    ])
