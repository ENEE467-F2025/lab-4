import os

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ur3e_robotiq_moveit_config.launch_common import load_yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):

    # General Arguements
    description_pkg = "ur3e_robotiq_description"
    description_file = "ur3e_robotiq.urdf.xacro"
    moveit_config_pkg = "ur3e_robotiq_moveit_config"
    kinematics_file = "kinematics.yaml"
    semantic_description_file = "ur3e_robotiq.srdf"
    moveit_controllers_file = "moveit_controllers.yaml"
    joint_limits_file = "joint_limits.yaml"
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_pkg), "rviz", "view_robot.rviz"]
    )

    moveit_config = (
        MoveItConfigsBuilder("ur3e_robotiq")
        .robot_description(
            file_path="config/ur3e_robotiq.urdf.xacro"
        )
        .robot_description_semantic(
            file_path="config/ur3e_robotiq.srdf"
        )
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .planning_scene_monitor(
            publish_robot_description_semantic=True
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "stomp"],
            default_planning_pipeline="ompl",
        )
        .joint_limits(
            file_path="config/joint_limits.yaml"
        )
        .moveit_cpp(
            file_path="config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(launch_rviz),
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    nodes_to_start = [move_group_node, rviz_node]

    return nodes_to_start

def generate_launch_description():

    declared_arguements = []

    declared_arguements.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Whether or not to launch rviz with the moveit configuration",
        )
    )
    declared_arguements.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="SIf true, use simulated clock",
        )
    )

    return LaunchDescription(declared_arguements + [OpaqueFunction(function=launch_setup)])