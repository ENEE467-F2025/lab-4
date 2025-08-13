import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):

    # general arguments
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    python_file = LaunchConfiguration("python_file")

    moveit_config = (
        MoveItConfigsBuilder("ur3e_robotiq", package_name="ur3e_robotiq_moveit_config")
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
    ).to_dict()
    moveit_config.update({"use_sim_time": True})

    moveit_py_node = Node(
        name="moveit_py",
        package="ur3e_robotiq_moveit_py",
        executable=python_file,
        output="both",
        parameters=[
            moveit_config
        ],
    )

    nodes_to_start = [
        moveit_py_node,
    ]

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
            description="If true, use simulated clock",
        )
    )
    declared_arguements.append(
        DeclareLaunchArgument(
            "python_file",
            default_value="test.py",
            description="Python API file name",
        )
    )

    return LaunchDescription(declared_arguements + [OpaqueFunction(function=launch_setup)])