#!/usr/bin/env python3

import sys
from copy import deepcopy
from math import sin, cos
import random

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from simple_spawner.sdf_templates import *

import tempfile
import shutil
import subprocess
import yaml 
from ament_index_python import get_package_share_directory


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w

class SimpleSpawner(Node):
    def __init__(self):
        super().__init__('simple_spawner')
        self.declare_parameter('world_name', 'empty')
        self.world_name = self.get_parameter('world_name').value

        self._rviz_process = None
        self.launch_rviz()

        # Prompt user
        self.model_name, self.template = self._prompt_user_choice()

        # Gazebo spawn client
        self.spawn_service_name = f'/world/{self.world_name}/create'
        self.spawn_client = self.create_client(SpawnEntity, self.spawn_service_name)

        # TF broadcaster
        # TODO: Create class variable br to store TransformBroadcaster object
        # self.br = None # <- MODIFY. Exercise 1 d
        self.model_pose = None

        # Subscriber for pose topic
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/model/{self.model_name}/pose',
            self.on_pose,
            10
        )

        # Continuous TF broadcast
        self.tf_timer = self.create_timer(0.05, self.broadcast_tf)

        # Attempt spawn
        self.spawn_timer = self.create_timer(0.5, self._try_spawn)
        self._spawn_future = None
        self._spawned = False

    def _prompt_user_choice(self):
        options = ["cube", "cylinder", "sphere"]
        print("Select an object to spawn:")
        for i, opt in enumerate(options, 1):
            print(f"  {i}) {opt.capitalize()}")
        choice = None
        if sys.stdin.isatty():
            while choice not in [1,2,3]:
                try:
                    choice = int(input("Enter choice (1-3): "))
                except ValueError:
                    continue
        else:
            choice = 1  # default cube if not interactive
        name = options[choice-1]
        return name, TEMPLATES[name]

    def _try_spawn(self):
        if self._spawned:
            self.spawn_timer.cancel()
            return
        if self._spawn_future is not None and not self._spawn_future.done():
            return
        if not self.spawn_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(f'Waiting for {self.spawn_service_name} service...')
            return
        self._spawn_future = self._call_spawn_service(self.template.format(name=self.model_name))

    # TODO: Complete this method. See manual.
    # Fill in the data for the SpawnEntity.Request object, request
    def _call_spawn_service(self, sdf_xml):
        request = None                               # MODIFY
        request.entity_factory.name = None           # MODIFY
        request.entity_factory.allow_renaming = None # MODIFY
        request.entity_factory.sdf = None            # MODIFY; ie sdf_xml
        
        # Set the inital pose of the entity (object)
        pose = Pose()
        pose.position.x = random.uniform(-0.5,0.5)
        pose.position.y = random.uniform(-0.5,0.5)
        pose.position.z = 0.05
        rot = quaternion_from_euler(0,0,0)
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]
        request.entity_factory.pose = deepcopy(pose)
        self.model_pose = deepcopy(pose)

        future = None # MODIFY using self.spawn_client call_async method
        future.add_done_callback(self._on_spawn_response)
        return future

    def _on_spawn_response(self, future):
        self._spawn_future = None
        if future.cancelled():
            self.get_logger().warn('Spawn request cancelled.')
            return
        if future.exception() is not None:
            self.get_logger().error(f'Spawn failed: {future.exception()}')
            return
        result = future.result()
        if result.success:
            self._spawned = True
            self.get_logger().info(f'Spawned {self.model_name} successfully.')
        else:
            self.get_logger().error('Spawn service reported failure.')

    def on_pose(self, msg: PoseStamped):
        self.model_pose = msg.pose
    
    # TODO: Complete this method, Exercise 1e.
    def broadcast_tf(self):
        if self.model_pose is None:
            return
        T = TransformStamped()
        # Fill the remainder of the fields of the T instance with the fields as specified in the manual
        T.header.stamp = self.get_clock().now().to_msg()
        T.header.frame_id = "world"
        T.child_frame_id = self.model_name
        #####################################################
        # TODO: fill T using model_pose class variable
        #####################################################
        # T.transform.translation ...
        ######################################################
        self.br.sendTransform(T)

    def launch_rviz(self):
        # Copy base.rviz to a temp file
        base_rviz = get_package_share_directory('simple_spawner') + '/rviz/base.rviz'
        tmp_rviz = tempfile.NamedTemporaryFile(suffix='.rviz', delete=False)
        shutil.copy(base_rviz, tmp_rviz.name)
        tmp_rviz.close()

        # Launch RViz pointing to this config
        self.get_logger().info(f"Launching RViz with config: {tmp_rviz.name}")
        self._rviz_process = subprocess.Popen(['rviz2', '-d', tmp_rviz.name])

def main():
    rclpy.init()
    node = SimpleSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
