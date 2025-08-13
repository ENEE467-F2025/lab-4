# Copyright 2024 Open Source Robotics Foundation, Inc.
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
#
# Author: Denis Stogl (Stogl Robotics Consulting)
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch.events import Shutdown

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):

    # General arguements
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur3e_robotiq_description"),
                 "urdf", "ur3e_robotiq.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(value=robot_description_content, value_type=str)}

    # get controller params
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ur3e_robotiq_description"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    # get rviz config
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur3e_robotiq_description"), "rviz", "view_robot.rviz"]
    )


    # nodes
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name",
                   "ur3e_robotiq", "-allow_renaming", "true"],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robotiq_hande_controller",
            "--param-file",
            robot_controllers,
            ],
    )
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--param-file",
            robot_controllers,
        ]
    )
    forward_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--inactive",
            "forward_velocity_controller",
            "--param-file",
            robot_controllers,
        ]
    )
    # Bridge, definitely need to parameterize this bs
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/fixed/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/fixed/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/fixed/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/fixed/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/wrist/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/wrist/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/wrist/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/wrist/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        output="screen"
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            {"use_sim_time": use_sim_time}
        ]
    )
    # Spawn gazebo
    start_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py"
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("gz_args", [" -r -v 1 empty.sdf", " --physics-engine gz-physics-bullet-featherstone-plugin"])
        ]
    )

    # event handlers
    jsb_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    gripper_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )
    jtc_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )
    fvc_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[forward_velocity_controller_spawner],
        )
    )

    nodes_to_start = [
        start_gz_sim,
        jsb_event_handler,
        gripper_event_handler,
        jtc_event_handler,
        fvc_event_handler,
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        rviz_node,
    ]

    return nodes_to_start

def generate_launch_description():

    # General arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="If true launches an rviz node",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])