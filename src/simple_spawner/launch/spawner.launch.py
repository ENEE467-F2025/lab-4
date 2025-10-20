from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

package_name = 'simple_spawner'
package_path = get_package_share_directory(package_name)

# File paths
model_path = os.path.join(package_path, 'models', 'cuboid.sdf')
bridge_cfg = os.path.join(package_path, 'config', 'bridge_spawner.yaml')
world_spawn = os.path.join(package_path, 'worlds', 'spawner_world.sdf')
world_empty = 'empty.sdf'  
rviz_tf = os.path.join(package_path, 'rviz', 'view_tf.rviz')
rviz_base = os.path.join(package_path, 'rviz', 'base.rviz')

def generate_launch_description():

    spawn_cuboid_cfg = LaunchConfiguration('spawn_cuboid')
    declare_spawn_cuboid = DeclareLaunchArgument(
        'spawn_cuboid', default_value='true',
        description='When true, spawn cuboid immediately. When false, wait for external simple_spawner.'
    )

    # Gazebo world selection
    gazebo_spawn = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_spawn],
        output='screen',
        condition=IfCondition(spawn_cuboid_cfg)
    )

    gazebo_empty = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_empty],
        output='screen',
        condition=UnlessCondition(spawn_cuboid_cfg)
    )

    # Bridges
    bridge_spawn = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '--ros-args', '-p', f'config_file:={bridge_cfg}'],
        output='screen',
        condition=IfCondition(spawn_cuboid_cfg)
    )

    bridge_empty = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/world/empty/create@ros_gz_interfaces/srv/SpawnEntity@gz.msgs.EntityFactory@gz.msgs.Boolean'],
        output='screen',
        condition=UnlessCondition(spawn_cuboid_cfg)
    )

    # RViz
    rviz_spawn = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_tf],
        output='screen',
        condition=IfCondition(spawn_cuboid_cfg)
    )

    # Auto spawn in TRUE mode only
    spawn_cuboid_timer = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service', '-s', '/world/spawner_world/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--req', f"sdf_filename: '{model_path}'\nname: 'cuboid'"
                ],
                output='screen'
            )
        ],
        condition=IfCondition(spawn_cuboid_cfg)
    )

    return LaunchDescription([
        declare_spawn_cuboid,
        gazebo_spawn, gazebo_empty,
        bridge_spawn, bridge_empty,
        rviz_spawn,
        spawn_cuboid_timer
    ])
