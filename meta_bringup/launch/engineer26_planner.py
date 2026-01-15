# run_planner.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("engineer_arm").robot_description(file_path="config/meta_arm.urdf.xacro").to_moveit_configs()

    return LaunchDescription([
        Node(
            package="engineer26_planner",
            executable="engineer26_planner_node",
            output="screen",
            prefix="xterm -e", # Optional: Opens a separate terminal window so you can type comfortably
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        )
    ])