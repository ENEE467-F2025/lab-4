import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

    objects = load_yaml("ur3e_robotiq_gz", "config/objects.yaml")
    objects = objects["objects"]
    load_nodes=[]

    for obj in objects.keys():
        name = obj # object name
        xyz = objects[name]["xyz"]
        rpy = objects[name]["rpy"]

        load_nodes.append(Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            parameters=[{
                'file': name,
                'name': name,
                'x': xyz[0],
                'y': xyz[1],
                'z': xyz[2],
                'R': rpy[0],
                'P': rpy[1],
                'Y': rpy[2],
            }],
        ))

    return LaunchDescription(
        load_nodes
    )